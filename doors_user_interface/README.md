# Doors — Multirobot Sensing / Cleaning Dashboard

Derived from `harmony_user_interface/`. Runs on **port 8082** so it and the HARMONY
dashboard can be up at the same time.

```bash
source /home/cem/colcon_ws/install/setup.bash
cd ~/colcon_ws/src/doors_user_interface
python3 app.py          # http://localhost:8082
```

## Operator flow

| Step | Button | What actually happens |
|------|--------|-----------------------|
| 1 | **START SYSTEM** | `hil_test_whole_unified.launch.py harmony:=true` (plus the fake-hardware flags if that toggle is on) |
| 2 | **Robots are Ready** | `doors_inspection doors_mission.launch.py only_sim:=… force_replan:=…` |
| 3 | **START SENSING ROBOT** | publishes `START`; `doors_mission_node` launches the doors inspection — **both arms**, the planned coverage tour |
| 4 | — | tour ends → KPI3 fault pop-up → the four scenario defects appear in the table and in RViz |
| 5 | **CONFIRM** | publishes `CONFIRM`; `cleaning_mission_runner` drives the **UR only**. The Kawasaki is already parked at home |

`STOP` cancels the running trajectories, waits for the arms to report zero velocity,
and only then tears the inspection launch down. Do not expect it to be instant — that
wait is the point.

## The three toggles

* **Use Fake Hardware** — read at step 1. Adds the mock-hardware flags to the HIL launch.
* **Sim Cameras Only** (`only_sim`) — read at step 2. Off means the real SICKs are
  captured too. Forced on when Use Fake Hardware is on, since there are no real
  cameras in that case. This is a different question from Use Fake Hardware: one is
  about the robot, the other about the cameras.
* **Force Replan** — read at step 2. Cached trajectories are replayed **without
  re-checking collision**, so turn this on for one run after any change to the
  collision model. It is off by default because a full replan is slow.

## Why sensing looks different from HARMONY

HARMONY scanned with the UR alone on hand-written waypoints and finished in seconds.
Here both arms run the planned coverage tour and it takes minutes, so `robot_status`
carries progress (`Scanning: UR 4/9  Kawasaki 2/3`). `doors_mission_node` scrapes that
from the inspection launch's own log lines.

The four defects are the same four — `doors_mission_node` imports them from
`pymoveit2_real.harmony_defects` rather than redefining them, so the two scenarios
cannot drift to different coordinates. They already sit on the doors, which is why
the cleaning side needed no changes at all.

## Known gaps

* The doors are physically ~5° off the CAD model (8 cm at the bottom, 1–2 cm at mid
  height). DEF-01 / DEF-02 sit high on the door and already had a measured ~6 cm reach
  shortfall.
* The plan on disk was produced before `viewpoint_distances` and `coverage_threshold`
  were changed; re-planning is still pending.
* `kawa_vp_003` collects almost nothing from the doors on real hardware.
