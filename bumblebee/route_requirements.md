# Visual guidance experiments: scenarios, scoring, and speed control

This design specification records the measured Bumblebee configuration of
July 25, 2026. Its 12 kg model and 1280×720 camera describe that historical
experiment, not the active Erenimbus setup. See [README.md](README.md) for
the current environment. Proposed infrastructure and tests below are
requirements, not evidence that they have been implemented or passed.

The goal is a reproducible experiment framework in which controller changes
are evaluated against a fixed scenario generator, runner, scorer, and safety
monitor. Following the separation used in Karpathy's autoresearch workflow,
the controller researcher changes controller files while an independent
evaluator controls scoring and hidden validation seeds.

## 1. Reference environment

- Launch headless with `./bumblebee_guidance.sh --headless` and stop with
  `./stop.sh`. The fixed ports support one environment at a time. Run tests
  serially, with a clean state before and after each scenario. Select plans
  through `BUMBLEBEE_HUNTER_PLAN` and `BUMBLEBEE_TARGET_PLAN`.
- `formation.py --yes [--delay N]` starts the target first. The interactive
  default delay is 2.0 s. A scenario may specify its own delay explicitly.
- The historical `model://bumblebee` represented a 12.0 kg twin-motor aircraft
  with real autotune parameters. Comparisons used seven flight logs, cruise
  throttle differences of 0.01–0.04, and roll-rate rise times of 0.24–0.26 s
  versus about 0.26 s in simulation. Preserve the measured mass, inertia,
  LiftDrag, collision, and parameter settings. The historical package check
  was `tools/validate_package.py`, with expected exit status 0.
- The camera was at `(0.68, 0, 0)`, with 1280×720 output at 30 Hz and a
  0.27 rad field of view: horizontal ±7.74°, vertical ±4.37°.
  Its topic was `/webcam/image_raw`.
- The red-target bridge published `[x,y,w,h,horizontal_coverage_pct,validity]`
  on Redis channel `tracker_bbox`, independently of GUI display. It published
  no message when detection failed. The evaluator must interpret silence
  as target loss.
- The baseline `tzi_emir.py` used MAVLink port 14553 for heading and altitude
  commands. Speed control was the first planned extension.
- The thrust plugin produced zero thrust below approximately 18% commanded
  throttle. This affects low-throttle descent scenarios, while cruise
  operation around 57–70% lies above the deadband.
- Python 3 can process JSON without a `jq` dependency. Keep experiment outputs
  and backups separate from the source and from immutable evaluation files.

## 2. Measured speed-control behavior

These findings came from the July 25 campaign under `reports/speed_tests/`.
An earlier proposed 1.22 speed conversion factor was a mistaken interpretation
of an open speed loop and must not be used as calibration.

### Synthetic airspeed and ground speed

Both aircraft used `ARSPD_TYPE 0`. Setting type 1 caused a SITL PANIC in this
configuration. With `TECS_SYNAIRSPEED 0`, TECS set `_SKE_weighting=0` and speed
commands could receive `COMMAND_ACK=ACCEPTED` without changing flight speed.
The aircraft continued at its `TRIM_THROTTLE` trim condition.

Setting `TECS_SYNAIRSPEED 1` in both aircraft closed the speed loop.
The pursuer used `params/bumblebee.parm`. The target used
`params/target.parm` with `AIRSPEED_CRUISE 20`, included in the launcher's
`--defaults` chain.

Synthetic IAS follows `IAS = |V_gps − W_EKF3|` in
`AP_AHRS::_airspeed_EAS()`. Even in a windless simulation, an unobserved EKF3
wind estimate can drift on straight legs. Measured IAS-minus-GPS offsets
were +0.29 m/s for the pursuer and −2.19 m/s for the target. Equal IAS
commands therefore do not imply equal ground speed.

`AHRS_WIND_MAX 1` limits the difference to approximately ±1 m/s. It is an
`AP_Int8` parameter, so 1 is the smallest effective positive value. Turns
update the wind estimate, and the offset varies over time. Log both IAS
and GPS speed. A `speed_source` abstraction should use GPS ground speed
for these simulation experiments and pitot airspeed for physical flight.
For a desired ground speed, estimate the IAS-minus-GPS offset online and
add it to the command. `formation.py`'s `equalise_speed()` demonstrated
agreement within 0.33 m/s at takeoff.

This Gazebo backend does not call `update_eas_airspeed()`. Selecting
`ARSPD_TYPE 100` also produced zero readings, so this setup did not simulate
a working pitot sensor.

### Commands and envelope

Use `MAV_CMD_GUIDED_CHANGE_SPEED` (43000): param1=0 for airspeed,
param2 in m/s, and param3 for acceleration. Heading uses 43002 and altitude
uses 43001. In this setup, altitude is relative to home. The old AMSL
comment in `tzi2.py` was incorrect.

`DO_CHANGE_SPEED` (178) also worked, but a 43000 command overrode its effect
until re-entering the mode. Do not mix these command paths. Measured speed
settling times were 2–13 s for the pursuer and 7–17 s for the target.

Out-of-envelope commands return `MAV_RESULT_FAILED` and leave the previous
target speed active. Clip guidance requests to `[minimum+1, maximum−1]`
before sending them, and enforce the same envelope in the immutable safety
layer. The pursuer's `AIRSPEED_MIN/MAX` values were 15/24 m/s, and the
target's were 9/22 m/s. Legacy `ARSPD_FBW_*` and `TRIM_ARSPD_CM` names were
ignored in ArduPilot 4.7. At 22 m/s the target required about 97% throttle,
leaving little reserve. Flight logs showed minimum sustained level speeds
between 13.7 and 16.3 m/s. Use these constraints when selecting scenarios.

The earlier pursuer/target speeds of 25.6/20.2 m/s, with overtaking at about
13 s after a 2 s takeoff delay, described the open-loop configuration.
With functioning speed commands, delay and speed jointly control the
initial pursuit geometry.

### Stored parameters and command test

`run/sitl*/eeprom.bin` overrides values supplied through `--defaults`.
One pursuer file specified `AIRSPEED_MAX 24`, while the live value remained
20 and rejected 22/24 m/s commands. Verify live values with `param fetch`
and deliberately reset stored parameters when required. For physical
flight, also inspect `ARSPD_RATIO`: enabled `ARSPD_AUTOCAL` produced a
measured 2.6% change between flights.

```bash
./command_sender.py --connect udpin:127.0.0.1:14553 --sysid 1 \
  --heading 90 --alt 80 --speed 22 --hold 60
```

The tool records acknowledgements and CSV data. Combined-command tests
measured pursuer heading 89–90°, altitude 80.0 m, and IAS 22.00 m/s for
requests 90°/80 m/22 m/s. Target requests 180°/60 m/18 m/s produced
180°/60.0 m/IAS 18.00 m/s. Reuse this command pattern in guidance.

## 3. Research and evaluation roles

The researcher may change `guidance_lab/controller.py` and
`guidance_lab/controller_config.yaml`. The independent evaluation layer
owns `scenario_generator.py`, `run_experiment.py`, `evaluator.py`,
`safety_monitor.py`, and hidden validation seeds under `hidden_tests/`.

Each experiment records a hypothesis, changes the controller, runs static,
smoke, and development checks, and proceeds to hidden validation only when
promising. Accept or reject it from the recorded metrics. The initial
controller can wrap the baseline heading/altitude logic. Start with PD/PID
gains, filters, gain scheduling, feedforward, angular-rate estimation, and
latency compensation. MPC and learned models are later research options.

## 4. Scenario definition

Represent target motion as parameterized primitives: constant heading and
altitude, constant turn rate, climb, coordinated turn and climb, S-turn,
turn-direction reversal, acceleration/deceleration, and approach/departure.
Use mission files for coarse routes and GUIDED command sequences for precise
turn-rate or climb-rate requests. Random waypoints alone can create
unflyable turns and poor coverage of relevant behavior.

```yaml
initial:
  relative_azimuth_deg: 5        # Horizontal field of view: ±7.74 degrees
  relative_elevation_deg: -2     # Vertical field of view: ±4.37 degrees
  range_m: 150                   # Sample within 80–400 m
  target_speed_mps: 20           # Command 43000; envelope 9–22 m/s
  pursuer_speed_mps: 20          # Envelope 15–24 m/s; account for IAS/GPS offset
  formation_delay_s: 12
maneuvers:
  - {start_s: 10, duration_s: 15, turn_rate_deg_s: 4, climb_rate_mps: 0}
  - {start_s: 30, duration_s: 10, turn_rate_deg_s: -6, climb_rate_mps: 1.5}
environment:
  wind_speed_mps: 0
  detector_dropout_prob: 0.0
```

An initial target outside the narrow camera cone belongs to a reacquisition
scenario. Validate physical feasibility before running: turn rate must
respect `g·tan(bank_max)/V`, climb should remain near or below 5 m/s
(measured 4.9–5.7 m/s), and speed must remain inside the envelope. Reject
or resample infeasible scenarios before they reach the runner.

Use 20 fixed development regressions plus 30 resampled cases per experiment.
Expose their detailed results to the researcher. Use approximately 100
hidden validation seeds and expose only aggregate results. Reserve a sparse,
human-controlled holdout for manually prepared or flown profiles. Add each
confirmed failure to the permanent development regressions.

## 5. Metrics and safety gates

For image width W, height H, and center (cx,cy), use normalized pixel error:

`e(t) = sqrt(((x-cx)/(W/2))^2 + ((y-cy)/(H/2))^2)`

Report mean and 95th-percentile error, time in the center region, time out
of view, reacquisition time, heading/altitude/speed command smoothness,
saturation duration, minimum and maximum range, and envelope violations.

An initial loss is
`L = 0.35·tracking + 0.25·worst_tail + 0.15·lost + 0.10·reacq + 0.10·smooth + 0.05·saturation`,
where `worst_tail` is the mean loss over the worst 20% of scenarios.
Calibrate these weights against measured behavior.

Separate hard gates disqualify stall-speed violations, excessive bank or
pitch, insufficient separation, NaNs or exceptions, sustained saturation,
target loss beyond a defined duration T, flight-zone violations, and unsafe
command jumps. A good average score cannot compensate for a gate violation.

The controller receives only bbox/image data and permitted telemetry.
The evaluator may access full simulation ground truth through Gazebo model
state or MAVLink `GLOBAL_POSITION_INT`. A single `udpin:14550` logger can
separate aircraft by SysID without competing sockets.

## 6. Candidate acceptance

Compare the candidate and champion on identical seeds. Require:

1. Zero hard-gate violations.
2. At least 2% lower mean development loss.
3. No degradation in the worst 20% of scenarios.
4. No scenario-family regression greater than 5%.
5. Improvement retained on hidden validation.

Store structured JSON fields `experiment_id`, `parent_commit`, `hypothesis`,
`changed_files`, `scenario_seeds`, `dev_metrics`, `val_metrics`,
`worst_scenarios`, `runtime`, `accepted`, and `reason`.
The proposed research workflow uses `experiment/NNNNNN` branches based on
`loop_testing`, merging accepted candidates there and retaining rejected
results separately. This describes the experimental design, not a claim
that these branches currently exist.

## 7. Test funnel

Run static import, interface, finite-value, and limit checks first. Next run
five short smoke scenarios: right turn, left turn, climb, descent, and a
receding target. Successful candidates proceed to the 20+30 development
cases, then hidden validation. Later adversarial testing can start with
random search and mutations of failing scenarios, followed by CMA-ES or
Bayesian search. Language models may suggest scenario families while
numerical search finds difficult parameters within each family.

HITL and physical flight remain infrequent and human-controlled. Shadow
mode can record candidate commands without applying them. SITL speedup and
Gazebo accelerated stepping were untested and must not be used for scoring
until their effect on calibration is established.

Include five metamorphic tests: horizontal mirroring with reversed heading
correction and comparable scores, invariance to world-heading rotation,
frame rates of 30/20/15 FPS, increasing camera latency, and quiet commands
for zero tracking error.

## 8. Domain randomization

Initially vary only a few pixels of bbox jitter, single-frame dropout near
probability 0.02, camera latency with small variance, and light wind.
Measure detector latency and dropout before choosing distributions.
Broader mass, center-of-gravity, and inertia variation belongs to a later
phase because premature broad randomization may produce overly conservative
controllers.

## 9. Prototype deliverables

Provide the immutable evaluation layer and controller wrapper, YAML scenario
schema and sampler, feasibility filter, 20 fixed regressions, runner with
timestamped bbox/GPS/command logs and clean shutdown, per-scenario and batch
JSON scoring, five smoke tests, five metamorphic tests, a candidate/champion
comparison, and a short `guidance_lab/README.md`.

Log IAS and GPS speed from the outset. Implement `speed_source` and the
43000-only command pattern so later speed-controller research uses the
same infrastructure. Develop the circle and climb routes from
[the route-test plan](mission_route_tests.md) as scenario families.
Use `--require-waypoints 1` for long legs and incorporate mission validation.
Report actual test outputs and limitations. Label every unexecuted test
explicitly, and provide a sample candidate/champion comparison only when run.
