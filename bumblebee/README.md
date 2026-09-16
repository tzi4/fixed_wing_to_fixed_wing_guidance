# Erenimbus: Gazebo / ArduPlane SITL visual guidance

This package runs a pursuer (SysID 1) and the `emir-gazebo-plane2` target
(SysID 2) with a shared home, dedicated MAVLink ports, and a Redis
`tracker_bbox` channel. Its launcher implements the simulation workflow
independently of the parent directory and the external `iq_sim` checkout.

The pursuer is **Erenimbus**, a copy of Emir's working 1.5 kg
`emir-gazebo-plane` model in `models/emir_aircraft_temp`. The camera uses
1920×1080 images, nominal `hfov` 0.42 rad, `near` 0.1, 30 Hz, and
`/webcam/image_raw`. The launcher uses `params/temp_emir.parm` with
`MAV_SYSID 1`, `TECS_SYNAIRSPEED 1`, `AHRS_WIND_MAX 1`, and `ARSPD_TYPE 0`,
and the world `worlds/temp_multi_uav.world`.

The Bumblebee model was retired on July 29, 2026 because its flight behavior
remained unresolved. The earlier CAD/manufacturer package is excluded from
this repository because redistribution permission was unavailable. The
active Erenimbus environment does not depend on that package.

## Quick start

For a new computer, follow [INSTALL.md](INSTALL.md) for the verified software
versions, external dependencies, environment variables, and setup checks.

```bash
./start_simulation.sh                    # GUI: gzclient, QGroundControl, bbox window
./start_simulation.sh --headless         # No GUI; bbox uses --no-display
./start_simulation.sh --stop             # Stop processes started by this package
./stop.sh                        # Stop and clean up known remaining processes
./bumblebee_guidance.sh --verify      # Two consecutive headless acceptance runs
BUMBLEBEE_VIDEO=1 ./start_simulation.sh   # Record camera video with bbox overlays
```

`start_simulation.sh` delegates to the core launcher `bumblebee_guidance.sh` using
`exec`. The core starts Gazebo, two ArduPlane SITL instances, the Redis
mission key, mission plans, and the bbox bridge. Shutdown uses the package's
process groups recorded in `run/pids`. `stop.sh` also removes known orphan
processes using the configured ports.

`--verify` runs the safe closed route twice from separate clean starts,
writing JSON and CSV reports under `reports/`. The `BUMBLEBEE_*` environment
variables configure the active Erenimbus environment:

| Variable | Default |
| --- | --- |
| `BUMBLEBEE_WORLD` | `worlds/temp_multi_uav.world` |
| `BUMBLEBEE_PARAM` | `params/temp_emir.parm` |
| `BUMBLEBEE_HUNTER_MODEL` | `models/emir_aircraft_temp/model.sdf` |

| Aircraft | SysID | MAVLink | Additional MAVLink | QGC | Gazebo FDM |
| --- | ---: | ---: | ---: | ---: | ---: |
| Pursuer (Erenimbus) | 1 | 14551 | 14553 | 14550 | 9002 |
| Target (`emir-gazebo-plane2`) | 2 | 14561 | - | 14550 | 9012 |

Both MAVProxy streams send a copy to QGroundControl's default UDP port
14550. Automation uses 14551, 14553, and 14561. `tzi_emir.py` connects to 14553.

## Mission routes and separation

By default, both aircraft receive `missions/long_straight.plan`: altitude 50 m,
with straight northbound waypoints at approximately 10, 50, 100, and 200 km.
The takeoff delay provides separation. The interactive default for
`formation.py --delay` is 2 s, corresponding to about 36 m of lead.

The launcher defines the plan paths in one place. Override them with
`BUMBLEBEE_HUNTER_PLAN` and `BUMBLEBEE_TARGET_PLAN`.

Alternative plans place the pursuer 5 m above the target. The short straight
plans use 55 m (`straight_hunter.plan`) and 50 m (`straight.plan`). The closed
plans use 65 m (`safe_closed_route_hunter.plan`) and 60 m
(`safe_closed_route.plan`). A 12 s delay gives roughly 200 m of separation
along the route. The acceptance suite uses the closed route and fixes the
delay at 12 s, independently of the interactive 2.0 s default. Physical
collisions remain enabled.

## Color detection and Redis

`bbox_to_redis.py` detects the target in `/webcam/image_raw` using HSV
thresholds and publishes `[x,y,w,h,horizontal_cov,validity]` as JSON on
Redis channel `tracker_bbox`. The launcher starts the bridge automatically,
with a window in GUI mode and `--no-display` in headless mode.
`BUMBLEBEE_BBOX=0` disables it. Start the guidance program separately.

### Purple target

Since July 28, 2026, the target uses `models/purple_target` and
`model://purple_target`. Red detection overlapped with the background: the
Gazebo sky dome rendered approximately BGR (127,127,255) near the horizon,
producing false detections across the full 1920-pixel width when the aircraft
banked. Ground textures also produced red line artifacts at grazing angles.

Measurements from three recordings, 900 frames, and 1.99e8 chromatic pixels
found sky near H=112 (54%), grass near H=47 (26%), and runway near H=32 (14%).
The red detection window covered 1.9% of background pixels, compared with
0.0012% for the purple window. The active world retains its sky, runway, and
grass. `worlds/ab_clean_emir.world` is retained only for reproducing historical
red-target comparisons with simplified background visuals.

| Variable | Default | Effect |
| --- | --- | --- |
| `BUMBLEBEE_TARGET_COLOR` | `purple` | `red` selects the historical red window |
| `BUMBLEBEE_HSV_SMIN` / `_VMIN` | Depends on color | Override the lower saturation/value limits |

Purple uses `H 140-160, S>=120, V>=60`. Gazebo/Purple is RGB (1,0,1), or
OpenCV H=150. Red uses `H 0-10 + 170-180, S>=70, V>=50`.

### Video overlay

The overlay displays `Code: <running scripts>` and the aircraft name,
right-aligned, for example `... +4     [Erenimbus]`. Text widths are measured
with `cv2.getTextSize`. Long script lists are truncated with `...` while
preserving at least 24 px between the labels.

The aircraft name comes from `BUMBLEBEE_AIRCRAFT`, then from the model directory
in `BUMBLEBEE_HUNTER_MODEL`. The `models/emir_aircraft_temp` directory maps to
`Erenimbus`. If neither source is available, the name is omitted.
`start_simulation.sh` configures this automatically. The overlay affects only the
rendered `canvas`, leaving published bbox data unchanged.

Gazebo parses `models/purple_target/model.config` as XML. Escape angle brackets
in description text. Invalid XML prevents the target model from loading,
so the FDM connection on port 9012 and its heartbeat never appear.

```bash
# The launcher loads the default plan into both aircraft automatically.
./load_plan.py --plan missions/long_straight.plan --ports 14551:1
./load_plan.py --plan missions/long_straight.plan --ports 14561:2
./formation.py --yes                 # Interactive default delay: 2.0 s
./bbox_to_redis.py --no-display
python3 tzi_emir.py                  # Visual guidance on port 14553
./verify_flight.py --duration 180 --report-dir reports/manual

# Select the short straight route with 5 m altitude separation.
BUMBLEBEE_HUNTER_PLAN=missions/straight_hunter.plan \
BUMBLEBEE_TARGET_PLAN=missions/straight.plan ./start_simulation.sh --headless
```

## Guidance and analysis programs

| File | Purpose |
| --- | --- |
| `goat_cam_offset.py` | Active visual guidance using camera offsets |
| `tzi_emir.py` | Guidance consuming `tracker_bbox` on port 14553 |
| `plane_follow.lua` | ArduPilot Lua tracking script |
| `old_guidance/goat_gimbal_aircraft.py` | Earlier virtual-gimbal reference |
| `tools/guidance_analysis.py` | Recover guidance outputs from aircraft `.BIN` logs |
| `tools/camera_angle_test.py` | Check camera/autopilot alignment using guidance CSV data |

## Acceptance reports

`verify_flight.py` checks the expected SysID heartbeat, arming and AUTO mode,
at least three pursuer waypoints, cruising airspeed between 12 and 30 m/s,
finite and stable attitude, altitude below 200 m, absence of critical
failsafe/crash/EKF messages, and continuous 1920×1080 camera output.

It computes instantaneous 3D separation from both vehicles'
`GLOBAL_POSITION_INT` messages. `min_separation_m` must be at least 20 m.
The number of bbox messages received during the first 60 s is reported as
the soft metric `tracker_bbox_messages_60s`. Both consecutive `--verify`
runs must pass for full acceptance.

## Local data directories

These directories and generated logs are excluded from Git:

| Directory | Contents |
| --- | --- |
| `flight_logs/` | Guidance CSV and altitude-difference logs |
| `27_july_logs/` | July 27 flights: ten `.BIN` logs and `code_logs/` |
| `july_1_logs/` | July 1 flights on a different board/aircraft, `AHRS_TRIM_Y` −2.01 |
| `new_verification_logs/` | July 23 verification flights |
| `reports/`, `run/`, `logs/`, `archive/` | Acceptance reports, process state, raw logs, and backups |
