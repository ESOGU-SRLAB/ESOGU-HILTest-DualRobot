import logging

import numpy as np


class RobotAllocator:
    """Greedy multi-robot viewpoint allocation.

    Given ONE shared pool of candidate viewpoints + its coverage matrix, and a
    per-robot reachability (IK) function, this greedily selects viewpoints and
    assigns each to exactly one arm. Two goals at once:

      * HIGHER total coverage than a single arm. A target that only one arm can
        reach (e.g. an interior/far face on the Kawasaki's side that the single
        Y-rail UR can never get to) is still covered, because the allocator picks
        whichever arm can see it. The reachable candidate set is the UNION of
        both arms, so the coverage ceiling rises above the single-arm one.

      * FASTER inspection. The two arms move simultaneously, so the wall-clock
        time is ~max(time_ur, time_kawasaki). Balancing the assigned COUNT
        (balance=True) shortens that only while the arms are about equally fast.
        They are not: measured on the doors job, the UR finished all 8 of its
        stops while the Kawasaki was still on its second, so an equal split
        leaves the UR idle and the run is entirely paced by the Kawasaki. The
        two knobs for that case are a per-arm cap (max_per_robot_by_name) and
        balance=False, which together push shared work onto the fast arm and
        spend the slow arm's few stops only where it is irreplaceable.

    The greedy is the same max-coverage core as the single-arm SetCoverOptimizer
    (pick the candidate covering the most still-uncovered surface next), extended
    with an arm-assignment step: among the arms that can actually reach the
    chosen candidate, give it to the one that currently has FEWER viewpoints
    (load balancing), or to the FIRST listed arm when balance=False. A candidate
    no arm can reach -- or that only a capped-out arm could reach -- is dropped.
    """

    def __init__(self, coverage_threshold=0.98, max_per_robot=None,
                 balance=True, min_new_points=1, min_marginal_coverage=0.0,
                 max_per_robot_by_name=None, logger=None):
        self.coverage_threshold = coverage_threshold
        # None or <=0 -> no per-arm budget (cover until the threshold). Otherwise
        # each arm takes at most this many viewpoints, so a run can be capped at
        # "each arm makes at most N stops".
        self.max_per_robot = max_per_robot if (max_per_robot and max_per_robot > 0) else None
        # PER-ARM OVERRIDE of the budget above, {robot_name: cap}. The two arms are
        # not interchangeable: they run in parallel but at very different speeds, so
        # the wall-clock cost of a run is set by the SLOWER arm's count, not by the
        # total. Capping only that arm is what shortens the run -- a shared
        # max_per_robot cannot express it, because lowering it throttles both.
        # Entries with a cap <= 0 are ignored (that arm falls back to max_per_robot).
        self.max_per_robot_by_name = {
            str(n): int(v) for n, v in (max_per_robot_by_name or {}).items() if int(v) > 0
        }
        # When True, a candidate reachable by both arms goes to the arm with the
        # fewer current assignments (shorter parallel run). When False, arms keep
        # a fixed priority order and only balance implicitly.
        self.balance = balance
        # Tail cutter (ABSOLUTE): stop adding viewpoints once the best remaining one
        # would add FEWER than this many new target points. The coverage curve has a
        # sharp knee -- the first viewpoints add hundreds of points each, then a long
        # tail adds only 1-3 each. Raising this trades a little coverage for a much
        # shorter plan (fewer waypoints). 1 = keep every useful viewpoint.
        self.min_new_points = max(1, int(min_new_points))
        # Tail cutter (FRACTIONAL, mirrors the solo SetCoverOptimizer): stop once the
        # best remaining viewpoint adds fewer than this FRACTION of all target points
        # as new coverage. Scales with mesh sampling density (0.005 = 0.5% ~= 25 new
        # points on a 5000-target mesh). The effective floor used is the LARGER of
        # this and min_new_points, so either knob can tighten the cut. 0 disables it.
        self.min_marginal_coverage = max(0.0, float(min_marginal_coverage))
        self.logger = logger or logging.getLogger(__name__)

    def allocate(self, candidates, coverage_matrix, robots):
        """Allocate viewpoints across the arms.

        candidates: list of viewpoint dicts (each has 'position', 'rotation').
        coverage_matrix: [num_candidates, num_targets] bool array.
        robots: ordered list of dicts, each:
            {
              'name': str,                      # e.g. 'ur' / 'kawasaki'
              'reachability_fn': callable | None,
                  # candidate -> (reachable: bool, joint_solution) . If None the
                  # arm is treated as able to reach every candidate (UNVERIFIED --
                  # the caller is responsible for logging that loudly).
            }

        Returns (assignments, final_coverage) where assignments is
        {robot_name: [selected candidate dicts]} in selection order. Each kept
        candidate is annotated with 'robot', 'joint_solution', 'rank',
        'new_points_covered', 'total_points_visible', 'cumulative_coverage'.
        """
        num_c, num_t = coverage_matrix.shape
        names = [r['name'] for r in robots]
        budgets = ", ".join(f"{n}={self._budget(n)}" for n in names)
        self.logger.info(
            f"Multi-robot allocation: {num_c} candidates, {num_t} targets, arms={names}, "
            f"coverage_threshold={self.coverage_threshold * 100:.1f}%, "
            f"budgets [{budgets}] (shared max_per_robot={self.max_per_robot}), "
            f"balance={self.balance}."
        )
        if not self.balance:
            self.logger.info(
                f"balance=False: a candidate both arms can reach always goes to '{names[0]}'. "
                f"The other arms only receive candidates '{names[0]}' cannot reach, which is "
                "what makes a small per-arm cap on a slow arm spend its budget where it is "
                "actually needed instead of on shared, high-gain viewpoints."
            )

        assignments = {r['name']: [] for r in robots}
        if num_c == 0 or num_t == 0:
            self.logger.error("No candidates or no target points; cannot allocate.")
            return assignments, 0.0

        uncovered = np.ones(num_t, dtype=bool)
        covered = np.zeros(num_t, dtype=bool)
        available = np.ones(num_c, dtype=bool)   # candidate still selectable?
        # Memoized IK results: (candidate_idx, robot_name) -> (bool, joint_state).
        ik_cache = {}
        ik_queries = 0
        excluded_unreachable = 0
        excluded_at_budget = 0
        order = 0  # global selection order (for cumulative coverage)

        target_covered = int(num_t * self.coverage_threshold)

        # Effective tail floor = the larger of the absolute (min_new_points) and the
        # fractional (min_marginal_coverage * num_t) cutters, so either can tighten.
        frac_floor = int(np.ceil(self.min_marginal_coverage * num_t)) \
            if self.min_marginal_coverage > 0.0 else 0
        tail_floor = max(self.min_new_points, frac_floor)
        if tail_floor > 1:
            self.logger.info(
                f"Tail cut floor = {tail_floor} new points "
                f"(min_new_points={self.min_new_points}, "
                f"min_marginal_coverage={self.min_marginal_coverage * 100:.2f}% "
                f"= {frac_floor}); stops once the best next viewpoint adds fewer."
            )

        def reach(idx, robot):
            nonlocal ik_queries
            key = (idx, robot['name'])
            if key not in ik_cache:
                fn = robot['reachability_fn']
                if fn is None:
                    ik_cache[key] = (True, None)
                else:
                    ik_queries += 1
                    ok, sol = fn(candidates[idx])
                    ik_cache[key] = (bool(ok), sol)
            return ik_cache[key]

        while np.sum(covered) < target_covered:
            if all(self._at_budget(n, assignments[n]) for n in assignments):
                self.logger.info("Every arm reached its per-robot budget; stopping allocation.")
                break

            # Marginal new coverage for every still-available candidate at once.
            gains = np.count_nonzero(coverage_matrix & uncovered, axis=1)
            gains[~available] = -1
            best_idx = int(np.argmax(gains))
            if gains[best_idx] <= 0:
                remaining_pct = 100 * np.sum(uncovered) / num_t
                self.logger.warning(
                    "Allocation exhausted: no remaining candidate adds new coverage. Stopping with "
                    f"{remaining_pct:.2f}% of the surface still uncovered (occlusion / camera range / "
                    "neither arm can reach it)."
                )
                break
            if gains[best_idx] < tail_floor:
                self.logger.info(
                    f"Tail cut: best remaining viewpoint adds only {int(gains[best_idx])} new points "
                    f"(< floor {tail_floor}). Stopping to keep the plan short "
                    f"(coverage {np.sum(covered) / num_t * 100:.2f}%)."
                )
                break

            # Which arms can reach this candidate AND still have budget? Order
            # them so the least-loaded eligible arm wins (load balancing).
            eligible = []
            for r in robots:
                if self._at_budget(r['name'], assignments[r['name']]):
                    continue
                ok, sol = reach(best_idx, r)
                if ok:
                    eligible.append((len(assignments[r['name']]), r['name'], sol))

            if not eligible:
                # No arm can (reachably, within budget) take the best candidate.
                # Retire it so the greedy moves on to the next-best one.
                available[best_idx] = False
                # It was unreachable by every non-budget arm; count it only if it
                # truly failed IK on all arms (not merely budget-blocked).
                if all(reach(best_idx, r)[0] is False for r in robots):
                    excluded_unreachable += 1
                else:
                    # An arm COULD have reached it but is capped out. This is the
                    # direct coverage cost of a per-arm budget, so it is counted
                    # separately -- otherwise a cap looks like an unreachable mesh.
                    excluded_at_budget += 1
                continue

            if self.balance:
                eligible.sort(key=lambda e: e[0])   # fewest current assignments first
            count, name, sol = eligible[0]

            # Commit the assignment.
            mask = coverage_matrix[best_idx]
            new_pts = int(np.count_nonzero(mask & ~covered))
            covered |= mask
            uncovered &= ~mask
            available[best_idx] = False

            vp = candidates[best_idx]
            vp['robot'] = name
            vp['joint_solution'] = sol
            vp['new_points_covered'] = new_pts
            vp['total_points_visible'] = int(mask.sum())
            vp['cumulative_coverage'] = float(np.sum(covered) / num_t)
            vp['global_order'] = order
            order += 1
            assignments[name].append(vp)

            self.logger.info(
                f"[{len(assignments[name]):>2}] {name} <- candidate {best_idx} "
                f"(+{new_pts} new pts). Total coverage {vp['cumulative_coverage'] * 100:.2f}% "
                f"(ur={len(assignments.get('ur', []))}, kawa={len(assignments.get('kawasaki', []))})."
            )

        final_coverage = float(np.sum(covered) / num_t)

        # Per-arm rank (position in that arm's own visiting order).
        for name in assignments:
            for rank, vp in enumerate(assignments[name]):
                vp['rank'] = rank

        totals = ", ".join(f"{n}={len(assignments[n])}" for n in assignments)
        self.logger.info(
            f"Allocation done: coverage {final_coverage * 100:.2f}% "
            f"(threshold {self.coverage_threshold * 100:.1f}%), assigned [{totals}], "
            f"{ik_queries} IK queries, {excluded_unreachable} candidates unreachable by any arm."
        )
        if excluded_at_budget:
            capped = [n for n in assignments if self._budget(n) is not None]
            self.logger.warning(
                f"{excluded_at_budget} candidate(s) were dropped ONLY because the arm that "
                f"could reach them had already spent its budget (capped arms: {capped}). "
                "That is the coverage price of the cap; raise the cap to buy it back."
            )
        if final_coverage + 1e-9 < self.coverage_threshold:
            self.logger.warning(
                f"Coverage target NOT met: {final_coverage * 100:.2f}% < "
                f"{self.coverage_threshold * 100:.1f}%. This multi-robot ceiling is still higher than a "
                "single arm's, but interior/occluded faces neither camera can see remain uncovered."
            )
        return assignments, final_coverage

    def _budget(self, name):
        """Viewpoint cap for one arm: its own override, else the shared cap, else None."""
        return self.max_per_robot_by_name.get(name, self.max_per_robot)

    def _at_budget(self, name, selected_list):
        cap = self._budget(name)
        return cap is not None and len(selected_list) >= cap
