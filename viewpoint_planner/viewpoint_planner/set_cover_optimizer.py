import logging
import numpy as np


class SetCoverOptimizer:
    """
    Selects a minimal subset of viewpoints that cover the target points.
    Uses a weighted greedy set cover algorithm.
    """
    def __init__(self, coverage_threshold=0.98, travel_weight=1.0, logger=None,
                 min_marginal_coverage=0.0):
        self.coverage_threshold = coverage_threshold
        # travel_weight scales the robot-travel penalty in the greedy score.
        #   travel_weight = 0.0 -> pure max-coverage set cover: always pick the
        #       candidate covering the most new points -> FEWEST viewpoints
        #       (best when the goal is minimum number of points).
        #   travel_weight > 0.0 -> bias toward nearby candidates to shorten the
        #       robot path, at the possible cost of a few extra viewpoints.
        self.travel_weight = travel_weight
        # AUTONOMOUS waypoint-count control (the "marginal-gain floor"). Greedy
        # stops as soon as the next viewpoint it would add covers fewer than
        # min_marginal_coverage * num_targets NEW surface points -- i.e. it refuses
        # to spend a stop on a viewpoint that inspects almost nothing new. This is
        # what makes the number of waypoints emerge from the geometry (the knee of
        # the diminishing-returns curve) instead of a hand-picked max_viewpoints.
        # 0.0 disables it (fall back to coverage_threshold / max_viewpoints only).
        # Example: 0.01 on a 4300-target mesh -> stop when the best next viewpoint
        # adds < 43 new points. Raise it -> fewer, higher-value viewpoints; lower
        # it -> more coverage from a longer tail.
        self.min_marginal_coverage = min_marginal_coverage
        self.logger = logger or logging.getLogger(__name__)

    def calculate_cost(self, current_vp, candidate_vp):
        """
        Calculates a heuristic cost to move from current_vp to candidate_vp.
        With travel_weight=0 the cost is constant (1.0), reducing the greedy
        selection to classic max-coverage set cover (minimizes viewpoint count).
        """
        if current_vp is None or self.travel_weight <= 0.0:
            return 1.0  # Base cost for the first viewpoint / travel-agnostic mode
        travel = np.linalg.norm(current_vp['position'] - candidate_vp['position'])
        return 1.0 + self.travel_weight * travel

    def optimize(self, candidates, coverage_matrix, max_viewpoints=None, reachability_fn=None):
        """
        Selects a subset of candidates to cover the targets.
        coverage_matrix: [num_candidates, num_targets] bool array.

        max_viewpoints: optional hard cap on the number of viewpoints (budget).
            Because the greedy step always takes the candidate with the highest
            marginal coverage first, stopping at N gives the *most efficient N*
            viewpoints -- i.e. "complete the inspection with only 12 stops".
            Whichever of (coverage_threshold, max_viewpoints) is hit first ends
            the loop.

        reachability_fn: optional callable(candidate) -> (bool, joint_solution).
            When given, reachability is checked *inside* the greedy loop and only
            reachable candidates are ever selected -- so the reported coverage is
            the coverage actually deliverable by the robot, and the minimal set
            is chosen from reachable viewpoints (not destroyed by a post-filter).
            IK is only queried for candidates the greedy step actually wants, so
            it stays cheap. Reachable candidates get candidate['joint_solution'].
        """
        num_c, num_t = coverage_matrix.shape
        mode = "max-coverage (fewest viewpoints)" if self.travel_weight <= 0.0 else \
            f"travel-optimized (travel_weight={self.travel_weight})"
        budget = f", max_viewpoints={max_viewpoints}" if max_viewpoints else ""
        reach = ", reachability-aware" if reachability_fn is not None else ""
        self.logger.info(
            f"Starting greedy set-cover optimization: {num_c} candidates, {num_t} targets, "
            f"coverage_threshold={self.coverage_threshold * 100:.1f}%, mode={mode}{budget}{reach}."
        )

        if num_c == 0 or num_t == 0:
            self.logger.error("No candidates or no target points provided; cannot optimize coverage.")
            return [], [], 0.0

        uncovered = np.ones(num_t, dtype=bool)
        selected_indices = []
        excluded = set()          # candidates known unreachable -> never reconsider
        reach_checks = 0          # how many IK queries we actually made
        current_vp = None

        target_covered = int(num_t * self.coverage_threshold)

        # Marginal-gain floor: the minimum NUMBER of new target points a viewpoint
        # must contribute to be worth a stop. Derived from the fraction so it scales
        # with the mesh sampling density. 0 disables it.
        min_new_points = int(np.ceil(self.min_marginal_coverage * num_t)) \
            if self.min_marginal_coverage > 0.0 else 0
        if min_new_points > 0:
            self.logger.info(
                f"Autonomous stop enabled: marginal-gain floor = {min_new_points} new points "
                f"({self.min_marginal_coverage * 100:.2f}% of {num_t} targets). Greedy will stop when "
                "the best next viewpoint covers fewer than this many new points."
            )

        while np.sum(uncovered) > num_t - target_covered:
            if max_viewpoints is not None and len(selected_indices) >= max_viewpoints:
                self.logger.info(
                    f"Reached max_viewpoints safety cap ({max_viewpoints}); stopping greedy selection."
                )
                break

            best_idx = -1
            best_score = -1
            best_new = 0          # actual NEW points covered by the best-scoring candidate

            # Find candidate that covers the most uncovered points per unit cost
            for i in range(num_c):
                if i in selected_indices or i in excluded:
                    continue

                new_covered = np.sum(coverage_matrix[i] & uncovered)
                if new_covered == 0:
                    continue

                cost = self.calculate_cost(current_vp, candidates[i])
                score = new_covered / cost

                if score > best_score:
                    best_score = score
                    best_idx = i
                    best_new = new_covered

            if best_idx == -1:
                # No more candidates can cover anything new
                remaining_pct = 100 * np.sum(uncovered) / num_t
                self.logger.warning(
                    f"Greedy search exhausted: no remaining candidate covers any additional target point. "
                    f"Stopping early with {remaining_pct:.2f}% of the chassis surface still uncovered. "
                    "This is usually caused by occlusion, camera range/FOV limits, or unreachable viewpoints "
                    "(check the reachability/IK filtering step)."
                )
                break

            # Autonomous stop: the best remaining candidate (ignoring reachability,
            # so an upper bound on any reachable one) adds too little to justify a
            # stop. reachable <= all, so if the best-of-all is below the floor every
            # reachable one is too -> stop cleanly here rather than padding the plan
            # with the diminishing-returns tail.
            if min_new_points > 0 and best_new < min_new_points:
                cov_now = 100 * (1 - np.sum(uncovered) / num_t)
                self.logger.info(
                    f"Marginal-gain floor reached: best next viewpoint adds only {int(best_new)} new points "
                    f"(< {min_new_points}). Stopping at {len(selected_indices)} viewpoints, "
                    f"coverage {cov_now:.2f}% -- the remaining candidates are the inefficient tail."
                )
                break

            # Reachability gate: only commit to a viewpoint the robot can reach.
            # If the top candidate is unreachable, exclude it and re-scan for the
            # next-best reachable one (keeps the selection a true greedy over the
            # reachable set).
            if reachability_fn is not None:
                reach_checks += 1
                is_reachable, joint_solution = reachability_fn(candidates[best_idx])
                if not is_reachable:
                    excluded.add(best_idx)
                    continue
                candidates[best_idx]['joint_solution'] = joint_solution

            selected_indices.append(best_idx)
            uncovered &= ~coverage_matrix[best_idx]
            current_vp = candidates[best_idx]

            cov_percent = 100 * (1 - np.sum(uncovered) / num_t)
            self.logger.info(
                f"Selected VP #{len(selected_indices)} (candidate {best_idx}, score={best_score:.1f}). "
                f"Current coverage: {cov_percent:.2f}%."
            )

        if reachability_fn is not None:
            self.logger.info(
                f"Reachability-aware greedy made {reach_checks} IK queries; "
                f"{len(excluded)} candidates rejected as unreachable, {len(selected_indices)} selected."
            )

        # Redundancy pruning: greedy can keep a viewpoint that later selections
        # made redundant. Drop any selected viewpoint whose targets are already
        # covered by the others (down to the coverage threshold). This is the
        # correct, coverage-preserving version of "merge nearby waypoints" --
        # unlike clustering camera positions with k-means, it can never silently
        # drop coverage, because orientation/occlusion are baked into the
        # coverage matrix it checks against.
        selected_indices = self._prune_redundant(selected_indices, coverage_matrix, num_t)
        # Recompute what is still uncovered by the pruned set.
        uncovered = np.ones(num_t, dtype=bool)
        for idx in selected_indices:
            uncovered &= ~coverage_matrix[idx]

        final_coverage = 1.0 - (np.sum(uncovered) / num_t)

        if final_coverage + 1e-9 < self.coverage_threshold:
            self.logger.warning(
                f"Target coverage NOT reached: achieved {final_coverage * 100:.2f}% "
                f"vs requested {self.coverage_threshold * 100:.1f}%. "
                f"{len(selected_indices)} viewpoints selected before giving up."
            )
        else:
            self.logger.info(
                f"Optimization complete. Selected {len(selected_indices)} viewpoints. "
                f"Final coverage: {final_coverage * 100:.2f}% (threshold {self.coverage_threshold * 100:.1f}%)."
            )

        # Annotate each kept viewpoint, in greedy order, with how many NEW target
        # points it contributes and the running coverage after it. This is the
        # "how informative is this waypoint" ranking the visualization uses:
        # rank 0 adds the most new coverage, and the marginal gain decays down
        # the list (the diminishing-returns tail).
        covered = np.zeros(num_t, dtype=bool)
        for rank, idx in enumerate(selected_indices):
            new_pts = int(np.sum(coverage_matrix[idx] & ~covered))
            covered |= coverage_matrix[idx]
            candidates[idx]['rank'] = rank
            candidates[idx]['new_points_covered'] = new_pts
            candidates[idx]['total_points_visible'] = int(coverage_matrix[idx].sum())
            candidates[idx]['cumulative_coverage'] = float(np.sum(covered) / num_t)

        selected_candidates = [candidates[i] for i in selected_indices]
        return selected_candidates, selected_indices, final_coverage

    def _prune_redundant(self, selected_indices, coverage_matrix, num_t):
        """Reverse-deletion pass over the greedy result.

        Considers viewpoints from least to most coverage and removes any whose
        targets are fully covered by the remaining ones, as long as the kept set
        still meets the coverage threshold. Cheap (O(|selected| * num_t)) and
        can only shrink the plan, never lower coverage below the threshold.
        """
        if len(selected_indices) <= 1:
            return selected_indices

        target_covered = int(num_t * self.coverage_threshold)
        kept = list(selected_indices)
        # Least-covering viewpoints are the likeliest to be redundant, so try
        # dropping those first.
        order = sorted(kept, key=lambda i: int(coverage_matrix[i].sum()))

        for idx in order:
            others = [k for k in kept if k != idx]
            if not others:
                break
            covered_by_others = int(np.any(coverage_matrix[others], axis=0).sum())
            if covered_by_others >= target_covered:
                kept.remove(idx)

        removed = len(selected_indices) - len(kept)
        if removed > 0:
            self.logger.info(
                f"Redundancy pruning removed {removed} viewpoint(s) whose coverage was already "
                f"provided by the rest; {len(kept)} viewpoints remain (coverage still "
                f">= {self.coverage_threshold * 100:.1f}%)."
            )
        else:
            self.logger.debug("Redundancy pruning: no viewpoint was redundant; plan unchanged.")
        # Preserve the greedy selection order for the ones we kept.
        return [i for i in selected_indices if i in set(kept)]
