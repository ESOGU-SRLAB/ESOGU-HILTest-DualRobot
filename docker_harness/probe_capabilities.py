"""Answers robot_capabilities.json open_questions Q1-Q4 on the live /sim stack.

Moves the SIM robot only. Run with the stack up (use_fake_hardware:=true):
  docker run --rm --network host --ipc=host -e PYTHONUNBUFFERED=1 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    -v ~/colcon_ws/src/docker_harness/probe_capabilities.py:/harness_ws/probe.py:ro \
    ros2-exec-harness:0.2.0 python3 /harness_ws/probe.py
"""
import json, math, time
import rclpy
from sim_robot_joint_goal import CollisionAwareRobotController

HOME = [1.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]
res = {}
rclpy.init()
r = CollisionAwareRobotController()
m = r.moveit2
t0 = time.time()
while m.joint_state is None and time.time() - t0 < 15:
    rclpy.spin_once(r, timeout_sec=0.2)
if m.joint_state is None:
    raise SystemExit("no /sim/joint_states - is the stack running?")
res["startup_s"] = round(time.time() - t0, 2)

def timed(q):
    t = time.time(); ok = r.move_to_joint_angles(q); ec = m.get_last_execution_error_code()
    return {"ok": ok, "error_code": getattr(ec, "val", None), "s": round(time.time() - t, 2)}

res["home"] = timed(HOME)
# Q4: failure path timing (rail outside limits)
res["Q4_rail_out_of_range"] = timed([2.5] + HOME[1:])
# Q1: scaling outside (0,1]
for s in (1.5, 0.0, -0.1):
    m.max_velocity = s
    q = list(HOME); q[1] = 0.3 if q[1] == 0.0 else 0.0
    res[f"Q1_max_velocity_{s}"] = timed(q)
    HOME = q
m.max_velocity = 0.05
res["Q1_reference_0.05"] = timed([1.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0])
# Q2: move_home_safe poses
for name, pos in (("Q2_intermediate", [-0.3, 0.3, 1.2]), ("Q2_home", [-0.3, 0.3, 0.95])):
    t = time.time(); r.move_to_position(pos, [-1.0, 0.0, 0.0, 0.0]); ec = m.get_last_execution_error_code()
    res[name] = {"error_code": getattr(ec, "val", None), "s": round(time.time() - t, 2)}
print("PROBE_RESULT " + json.dumps(res))
r.destroy_node()
