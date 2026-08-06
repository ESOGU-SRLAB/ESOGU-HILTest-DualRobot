#!/usr/bin/env python3
"""
Limit-aware 2*pi joint-goal wrapping.

WHY THIS EXISTS
---------------
IK returns ONE branch of a solution family. For a revolute joint, theta and
theta +/- 2*pi are the SAME physical configuration -- identical link poses, so
identical collisions and identical camera orientation -- but they are very
different NUMBERS to travel to. The planner stores whatever branch the IK solver
happened to return, so two consecutive viewpoints can be stored as e.g.
+100 deg and -260 deg: the same joint angle, yet the arm dutifully spins a full
extra turn between them. On a real run that is wasted inspection time, extra wear,
and cable wind-up.

The fix is to re-express each goal angle as the 2*pi-equivalent NEAREST to a
reference angle (the arm's current measured position at execution time, or the
previous viewpoint at plan time).

WHY THE LIMIT CHECK IS THE WHOLE POINT
--------------------------------------
An earlier, limit-BLIND version of this rule was tried and had to be reverted: it
happily rewrote an in-limits value (Kawasaki joint1 = 1.928 -> -4.355) into one the
joint cannot physically reach, and the arm ended up at unrelated, wrongly-oriented
poses. So every candidate here is accepted ONLY if it lands inside that joint's
URDF limits (minus a safety margin). This makes the transform a no-op exactly where
it must be:

    joint range < 2*pi   -> at most one equivalent is in-limits -> nothing changes
                            (Kawasaki joint2/3/5, and UR elbow, which the UR URDF
                            deliberately clamps to +/-pi)
    joint range >= 2*pi  -> a real choice exists -> we take the nearest one
                            (UR pan/lift/wrist1..3 and Kawasaki joint4/joint6, all +/-2*pi)

Prismatic joints (the UR rail, world_to_agv) are never touched: metres do not wrap.

Pure Python + NumPy on purpose -- no ROS imports -- so the planner node, the
executors, and offline tooling can all share one implementation.
"""
import math
import xml.etree.ElementTree as ET

TWO_PI = 2.0 * math.pi

# Joint types whose position is an ANGLE and therefore 2*pi-periodic.
_REVOLUTE_TYPES = ("revolute", "continuous")


class JointLimit:
    """One joint's wrap-relevant facts: is it an angle, and how far can it go?

    `lower`/`upper` are radians for revolute joints. A `continuous` joint has no
    URDF limits, so it gets (-inf, +inf) and every equivalent is admissible.
    """

    __slots__ = ("name", "jtype", "lower", "upper")

    def __init__(self, name, jtype, lower, upper):
        self.name = name
        self.jtype = jtype
        self.lower = lower
        self.upper = upper

    @property
    def is_revolute(self):
        return self.jtype in _REVOLUTE_TYPES

    @property
    def span(self):
        return self.upper - self.lower

    @property
    def wrappable(self):
        """True only if a second in-limits equivalent can possibly exist. A span of
        EXACTLY 2*pi (Kawasaki joint1, +/-180 deg) is not wrappable in practice: the
        only alternative sits on the opposite limit edge, which the margin rejects."""
        return self.is_revolute and self.span > TWO_PI

    def __repr__(self):
        return (f"JointLimit({self.name!r}, {self.jtype!r}, "
                f"{math.degrees(self.lower):.1f}deg, {math.degrees(self.upper):.1f}deg)")


def parse_joint_limits(urdf_xml):
    """Parse a URDF string into {joint_name: JointLimit}.

    Only joints that actually move are returned; fixed/mimic joints are irrelevant
    here. A revolute joint missing its <limit> tag is skipped rather than guessed at,
    so an unparseable model degrades into "do not wrap" instead of into a bad wrap.
    """
    limits = {}
    root = ET.fromstring(urdf_xml)
    for joint in root.iter("joint"):
        name = joint.get("name")
        jtype = (joint.get("type") or "").lower()
        if not name or jtype in ("fixed", "floating", "planar", ""):
            continue
        if jtype == "continuous":
            limits[name] = JointLimit(name, jtype, -math.inf, math.inf)
            continue
        tag = joint.find("limit")
        if tag is None:
            continue
        try:
            lower = float(tag.get("lower"))
            upper = float(tag.get("upper"))
        except (TypeError, ValueError):
            continue
        if upper < lower:
            lower, upper = upper, lower
        limits[name] = JointLimit(name, jtype, lower, upper)
    return limits


def wrap_angle_to(value, reference, limit, margin=0.05):
    """Nearest 2*pi-equivalent of `value` to `reference`, kept inside `limit`.

    Returns `value` unchanged when the joint is prismatic, when no equivalent is
    admissible, or when no limit information is available. Never returns an angle
    outside [lower + margin, upper - margin].
    """
    if limit is None or not limit.is_revolute:
        return value
    lo = limit.lower + margin
    hi = limit.upper - margin
    if lo > hi:  # margin wider than the joint's own range -- refuse to touch it
        return value

    # k = 0 is `value` itself. Search outward far enough to cover any real joint
    # (+/-2 turns spans the +/-720 deg widest case here) and keep the admissible
    # candidate closest to the reference.
    best = None
    best_dist = math.inf
    for k in range(-3, 4):
        cand = value + k * TWO_PI
        if cand < lo or cand > hi:
            continue
        dist = abs(cand - reference)
        if dist < best_dist:
            best, best_dist = cand, dist
    if best is None:
        # `value` itself is out of limits (a stale or badly-seeded IK solution).
        # Leave it alone: silently relocating an invalid goal would hide the problem.
        return value
    return best


def wrap_to_reference(goal, names, reference, limits, margin=0.05, min_gain=0.0):
    """Wrap a whole joint goal towards a reference configuration.

    goal      -- list of target positions (radians / metres), in `names` order
    names     -- joint names, same length as goal
    reference -- {joint_name: measured_or_previous_position}; joints missing here
                 are left untouched (we never guess a reference)
    limits    -- {joint_name: JointLimit} from parse_joint_limits()
    margin    -- radians kept clear of each limit edge
    min_gain  -- only accept a rewrite that shortens this joint's travel by at
                 least this many radians, so we do not churn goals for nothing

    Returns (new_goal, changes) where changes is a list of
    (name, old_value, new_value, saved_radians), most-saving first.
    """
    out = list(goal)
    changes = []
    for i, name in enumerate(names):
        limit = limits.get(name) if limits else None
        if limit is None or not limit.wrappable:
            continue
        if name not in reference:
            continue
        ref = float(reference[name])
        old = float(goal[i])
        new = wrap_angle_to(old, ref, limit, margin=margin)
        saved = abs(old - ref) - abs(new - ref)
        if abs(new - old) > 1e-9 and saved >= min_gain:
            out[i] = new
            changes.append((name, old, new, saved))
    changes.sort(key=lambda c: -c[3])
    return out, changes


def describe_changes(changes):
    """One-line human summary of what wrap_to_reference() rewrote (degrees)."""
    if not changes:
        return "no joints wrapped"
    parts = [f"{n} {math.degrees(o):.1f} -> {math.degrees(v):.1f} "
             f"(saves {math.degrees(s):.0f} deg)" for n, o, v, s in changes]
    total = sum(math.degrees(c[3]) for c in changes)
    return f"wrapped {len(changes)} joint(s), saving {total:.0f} deg total: " + "; ".join(parts)


def path_travel(goal, names, reference, revolute_only=True, limits=None):
    """Total |delta| between a reference configuration and a goal -- the quantity the
    wrap is trying to shrink. Used for logging and for offline before/after checks."""
    total = 0.0
    for i, name in enumerate(names):
        if name not in reference:
            continue
        if revolute_only:
            limit = limits.get(name) if limits else None
            if limit is not None and not limit.is_revolute:
                continue
        total += abs(float(goal[i]) - float(reference[name]))
    return total
