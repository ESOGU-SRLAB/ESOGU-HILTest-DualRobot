"""pytest setup for STLC-generated tests running in the harness container.

The generated tests construct SimCollisionAwareRobotController() directly, and a
rclpy Node cannot be created before rclpy.init(). Initialise once for the whole
session. pymoveit2 spins the node itself while waiting, so no executor is needed.

Tests that call rclpy.init()/rclpy.shutdown() themselves would otherwise raise
("init called twice") or tear the context down under the remaining tests, so
both become no-ops while the session context is alive.
"""

import pytest
import rclpy

_real_init = rclpy.init
_real_shutdown = rclpy.shutdown


def _init(*args, **kwargs):
    if not rclpy.ok():
        _real_init(*args, **kwargs)


def _shutdown(*args, **kwargs):
    pass


@pytest.fixture(scope="session", autouse=True)
def ros_context():
    _init()
    rclpy.init = _init
    rclpy.shutdown = _shutdown
    yield
    rclpy.init = _real_init
    rclpy.shutdown = _real_shutdown
    if rclpy.ok():
        _real_shutdown()
