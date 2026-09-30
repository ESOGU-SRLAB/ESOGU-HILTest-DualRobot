#!/usr/bin/env python3
"""
Compatibility shim -- the implementation now lives in `pymoveit2_real.joint_wrap`.

It moved because limit-aware 2*pi unwinding is not a viewpoint-planning concern: it
applies to ANY joint goal handed to MoveIt, and `pymoveit2_real.MoveIt2.set_joint_goal`
now applies it for every use case (inspection, cleaning, pick-and-place, HRC) instead of
each one re-implementing it. `viewpoint_planner` already exec_depends on
`pymoveit2_real`, so nothing about the dependency graph changed.

Existing imports (`from viewpoint_planner.joint_wrap import ...`) keep working; there is
still exactly ONE implementation behind them. Prefer importing from `pymoveit2_real`
in new code.
"""
from pymoveit2_real.joint_wrap import (  # noqa: F401
    TWO_PI,
    JointLimit,
    describe_changes,
    parse_joint_limits,
    path_travel,
    wrap_angle_to,
    wrap_to_reference,
)

__all__ = [
    "TWO_PI",
    "JointLimit",
    "describe_changes",
    "parse_joint_limits",
    "path_travel",
    "wrap_angle_to",
    "wrap_to_reference",
]
