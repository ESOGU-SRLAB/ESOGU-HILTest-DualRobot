#!/usr/bin/env python3
"""Tests for limit-aware 2*pi joint-goal unwinding.

The rule these tests pin down: an angle may be re-expressed as a different
2*pi-equivalent ONLY when the joint's URDF limits leave room for it. A previous,
limit-blind version of this rewrite drove the Kawasaki to unreachable goals, so the
"narrow joints never move" cases below are the important ones.
"""
import math

import pytest

from viewpoint_planner.joint_wrap import (JointLimit, describe_changes,
                                          parse_joint_limits, wrap_angle_to,
                                          wrap_to_reference)

D = math.radians

# The real limits of the two arms in the cell (UR10e joint_limits.yaml and
# rs005l_macro.xacro), plus their linear axes.
CELL_URDF = "<robot name='cell'>" + "".join(
    f"<joint name='{n}' type='{t}'><limit lower='{lo}' upper='{hi}'/></joint>"
    for n, t, lo, hi in [
        ("world_to_agv", "prismatic", 0.0, 3.0),
        ("ur10e_base_to_robot_mount", "prismatic", 0.0, 2.5),
        ("ur10e_shoulder_pan_joint", "revolute", D(-360), D(360)),
        ("ur10e_shoulder_lift_joint", "revolute", D(-360), D(360)),
        ("ur10e_elbow_joint", "revolute", D(-180), D(180)),
        ("ur10e_wrist_1_joint", "revolute", D(-360), D(360)),
        ("ur10e_wrist_2_joint", "revolute", D(-360), D(360)),
        ("ur10e_wrist_3_joint", "revolute", D(-360), D(360)),
        ("joint1", "revolute", D(-180), D(180)),
        ("joint2", "revolute", D(-80), D(135)),
        ("joint3", "revolute", D(-172), D(118)),
        ("joint4", "revolute", D(-360), D(360)),
        ("joint5", "revolute", D(-145), D(145)),
        ("joint6", "revolute", D(-360), D(360)),
    ]) + "</robot>"

LIMITS = parse_joint_limits(CELL_URDF)

WIDE = ["ur10e_shoulder_pan_joint", "ur10e_shoulder_lift_joint", "ur10e_wrist_1_joint",
        "ur10e_wrist_2_joint", "ur10e_wrist_3_joint", "joint4", "joint6"]
NARROW = ["ur10e_elbow_joint", "joint1", "joint2", "joint3", "joint5"]


def test_parses_types_and_limits():
    assert LIMITS["joint2"].jtype == "revolute"
    assert LIMITS["joint2"].lower == pytest.approx(D(-80))
    assert LIMITS["world_to_agv"].jtype == "prismatic"
    assert "fixed_thing" not in parse_joint_limits(
        "<robot name='r'><joint name='fixed_thing' type='fixed'/></robot>")


def test_continuous_joint_has_no_limits():
    limits = parse_joint_limits(
        "<robot name='r'><joint name='spin' type='continuous'/></robot>")
    assert limits["spin"].wrappable
    assert wrap_angle_to(D(-260), D(100), limits["spin"]) == pytest.approx(D(100))


@pytest.mark.parametrize("name", WIDE)
def test_wide_joints_are_wrappable(name):
    assert LIMITS[name].wrappable, f"{name} spans more than 2*pi and should be wrappable"


@pytest.mark.parametrize("name", NARROW)
def test_narrow_joints_are_never_wrappable(name):
    """A range of 2*pi or less admits no second in-limits equivalent. This is the
    guard that keeps the old 'joint1 1.928 -> -4.355' failure from coming back."""
    assert not LIMITS[name].wrappable


def test_users_example_saves_a_full_turn():
    """+100 deg then -260 deg is the same angle; the arm should not drive 360 deg."""
    goal, changes = wrap_to_reference(
        [D(-260)], ["ur10e_shoulder_lift_joint"],
        {"ur10e_shoulder_lift_joint": D(100)}, LIMITS)
    assert goal[0] == pytest.approx(D(100))
    assert len(changes) == 1
    assert changes[0][3] == pytest.approx(2 * math.pi)  # saved radians


def test_result_is_always_physically_identical():
    """Whatever is returned must differ from the original by a whole number of turns."""
    for value in (D(-359.9), D(270.0), D(-279.1), D(190.7)):
        new = wrap_angle_to(value, D(0.0), LIMITS["ur10e_wrist_2_joint"])
        turns = (new - value) / (2 * math.pi)
        assert turns == pytest.approx(round(turns), abs=1e-9)


def test_never_leaves_the_joint_limits():
    limit = LIMITS["joint4"]  # +/-360 deg
    for value in (D(-350), D(350), D(10), D(-10)):
        for ref in (D(-355), D(0), D(355)):
            new = wrap_angle_to(value, ref, limit, margin=0.05)
            assert limit.lower + 0.05 <= new <= limit.upper - 0.05


def test_out_of_limits_input_with_no_valid_equivalent_is_left_alone():
    """joint2 spans -80..135 deg, so 200 deg has no in-limits equivalent at all. The
    goal is bad, and silently relocating it elsewhere would hide that -- return it
    untouched and let planning fail loudly instead."""
    assert wrap_angle_to(D(200), D(0), LIMITS["joint2"]) == pytest.approx(D(200))


def test_out_of_limits_input_is_rescued_when_an_equivalent_fits():
    """The mirror case: joint1 spans +/-180 deg, so a stored 400 deg goal is
    unreachable as written but its 40 deg equivalent is the same pose and is fine."""
    assert wrap_angle_to(D(400), D(0), LIMITS["joint1"]) == pytest.approx(D(40))


def test_prismatic_axes_are_never_touched():
    goal, changes = wrap_to_reference(
        [2.4], ["world_to_agv"], {"world_to_agv": 0.1}, LIMITS)
    assert goal == [2.4]
    assert changes == []


def test_missing_reference_leaves_joint_alone():
    goal, changes = wrap_to_reference(
        [D(-260)], ["ur10e_shoulder_lift_joint"], {}, LIMITS)
    assert goal[0] == pytest.approx(D(-260))
    assert changes == []


def test_min_gain_suppresses_pointless_rewrites():
    """190 deg vs its -170 deg equivalent saves only 20 deg against a 0 deg reference
    -- both are about half a turn away, so neither is worth rewriting for."""
    _, changes = wrap_to_reference(
        [D(190)], ["joint6"], {"joint6": D(0)}, LIMITS, min_gain=D(30))
    assert changes == []


def test_min_gain_still_allows_a_worthwhile_rewrite():
    goal, changes = wrap_to_reference(
        [D(190)], ["joint6"], {"joint6": D(0)}, LIMITS, min_gain=D(10))
    assert goal[0] == pytest.approx(D(-170))
    assert changes[0][3] == pytest.approx(D(20))


def test_unknown_joint_is_left_alone():
    goal, changes = wrap_to_reference(
        [D(-260)], ["mystery_joint"], {"mystery_joint": D(100)}, LIMITS)
    assert goal[0] == pytest.approx(D(-260))
    assert changes == []


def test_empty_limits_table_disables_everything():
    """No URDF -> no limits -> behave exactly as before rather than guess."""
    goal, changes = wrap_to_reference(
        [D(-260)], ["ur10e_shoulder_lift_joint"],
        {"ur10e_shoulder_lift_joint": D(100)}, {})
    assert goal[0] == pytest.approx(D(-260))
    assert changes == []


def test_margin_wider_than_range_refuses_to_touch():
    tiny = JointLimit("tiny", "revolute", -0.01, 0.01)
    assert wrap_angle_to(5.0, 0.0, tiny, margin=1.0) == pytest.approx(5.0)


def test_describe_changes_is_readable():
    _, changes = wrap_to_reference(
        [D(-260)], ["ur10e_shoulder_lift_joint"],
        {"ur10e_shoulder_lift_joint": D(100)}, LIMITS)
    text = describe_changes(changes)
    assert "ur10e_shoulder_lift_joint" in text and "360" in text
    assert describe_changes([]) == "no joints wrapped"
