# Guidance test routes: circle, climb, and custom mission

This experimental plan records the route-test configuration of July 22,
2026. Use the active [README](README.md) for current launcher names and model
settings. Historical report paths identify measurements from that session.

## Shared environment

The GUI and headless entry points share one core launcher,
`bumblebee_guidance.sh`. Select plans with `BUMBLEBEE_HUNTER_PLAN` and
`BUMBLEBEE_TARGET_PLAN`. Both default to `missions/long_straight.plan`.
The interactive `formation.py --delay` default is 2.0 s. The acceptance
suite separately uses its closed route with a fixed 12 s delay.

The pursuer is SysID 1 on ports 14551 and 14553. The target is SysID 2 on
14561. QGroundControl uses 14550, and Gazebo FDM uses 9002 and 9012.
`tzi_emir.py` connects to 14553. The bbox bridge starts automatically and
publishes `[x,y,w,h,horizontal_cov,validity]` on `tracker_bbox`.
`load_plan.py` uses `MISSION_ITEM_INT` for distant, precise coordinates.

Route 1 used the long northbound straight plan, extending about 200 km,
with both aircraft at 50 m and a 2 s delay. Measured separation was about
66 m, with 1,592 bbox detections in 60 s. The report
`reports/long_straight_test/verification_20260722_165124.json` recorded
`passed: false` because the 10 km first leg prevented waypoint 3 from being
reached within 180 s. Use `--require-waypoints 1` for this long straight
scenario. Its purpose is to measure unnecessary guidance commands when the
target is directly ahead.

## Procedure for every route

1. Store each route as a separate QGroundControl `.plan` under `missions/`.
   Select it through the existing environment variables or the core default.
   Keep the launchers shared.
2. Use relative-altitude frame 3, takeoff command 22, waypoint command 16,
   home `41.101658,28.545652`, and cruise speed 18 m/s.
3. Start headless with both plan variables, run `./formation.py --yes`, then
   `./verify_flight.py --duration N --require-waypoints 1 --report-dir reports/<route_name>`.
   Stop with `./stop.sh` and confirm that the ports are free.
4. Require stable attitude, continuing bbox detections, and separation of
   at least 20 m. Document any scenario that deliberately expects a smaller
   separation. Run the existing `--verify` suite after each route change.
5. Preserve thresholds, delays, and parameters unless a change is explicitly
   justified in the report. Back up existing files before changing them.

## Route 2: large circle

Test horizontal tracking while the target follows a broad circle.
Create `missions/circle.plan` with its center about 3 km north of home,
a radius of 600–800 m, 12–16 waypoints, and a fixed altitude of 60 m.
Either turn direction is suitable. The approximate minimum turn radius at
18 m/s is 35 m, so this circle provides a generous margin. Use `DO_JUMP`
(command 177) to repeat the route indefinitely for runs longer than 180 s.

Start with the same plan for both aircraft and the 2 s takeoff delay.
An optional `circle_hunter.plan` may place the pursuer 5 m higher. Record
the bbox horizontal-center error in pixels as a time series, using guidance
logs or `tracker_bbox`, and count the guidance commands.

## Route 3: sustained climb

Create `missions/climb.plan` with northbound waypoints about 2 km apart.
Increase altitude by 100 m per leg: 50, 150, 250, 350, and 450 m.
At 18 m/s this requests roughly 0.9 m/s of sustained climb.

Before running this scenario:

- Make the altitude limit in `verify_flight.py` configurable with `--max-alt`.
  Preserve the default of 200 m for the acceptance suite and use 500 m here.
- Check `RTL_ALT` and battery failsafe settings for operation up to 450 m.

Use the same plan for the pursuer with a 2 s delay. Record bbox vertical-center
error and the altitude difference as time series.

## Route 4: user-defined mission

Save the QGroundControl mission as `missions/custom.plan`. Load the same plan
for both aircraft through the existing environment variables. The integer
mission loader supports distant coordinates.

Add a small `tools/validate_plan.py` utility to check frame 3, a takeoff item,
the home position, and reasonable altitudes. Report invalid input without
silently changing the user's mission. Then run the shared test procedure.

## Evaluation order and report

Run the circle scenario first, prepare the climb prerequisites, run the
climb, and finally validate the custom-mission workflow. For each route,
record the report path, minimum separation, bbox count, tracking-error
summary, and acceptance-suite result. Distinguish completed measurements
from planned tests and document any remaining setup requirements.
