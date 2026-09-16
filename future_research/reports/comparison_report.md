# Guidance architecture comparison

This historical review compares `gimbal_roll_pitch.py`, `tzi_final.py`,
and `tzi.py` for autonomous target tracking. It focuses on thread structure,
control calculations, and target-loss handling. Descriptions apply to the
reviewed source snapshots and do not establish flight safety.

## Threads and data flow

In `gimbal_roll_pitch.py`, `RedisListener` and `MavlinkManager` inherit
from `threading.Thread`. A thread-safe `queue.Queue()` connects image data
to the main control loop, which reads with `timeout=0.05`. Each queued
sample is processed once. Queue synchronization protects this handoff,
while other shared state still requires its own synchronization.

`tzi.py` and `tzi_final.py` use listener and processor functions running in
threads. A `threading.Lock()` protects `self.latest_bbox`. The processor
wakes at 30 Hz with `time.sleep(1.0/30.0)`, reads the latest sample, and skips
it when its timestamp matches the previous processed sample.

## Controllers and filtering

`gimbal_roll_pitch.py` requests roll and pitch. A soft deadzone makes the
proportional error zero inside the deadzone and subtracts the deadzone width
outside it: `e_p = e − sign(e)·deadzone`.

Its leaky integrator uses `I = I × 0.995` in the deadzone. At output
saturation, `I = I × 0.9` reduces windup. The derivative is filtered with
alpha 0.03:

`D_new = alpha·D_raw + (1−alpha)·D_prev`

The TZI variants request heading, altitude, and throttle or airspeed.
Heading and altitude use proportional and derivative terms without an
integral term. The reviewed throttle controller uses an integral clamp
between −10 and +55, without the same leaky-integrator or derivative-filter
structure. The command type differs between source variants and must be
checked in the corresponding sender.

## Target loss

`gimbal_roll_pitch.py` retains the last roll and pitch commands during a
5 s coasting interval. After a longer loss, it requests roll and pitch 0.0,
resets stored PID errors and integrals, and emits a warning. A level-attitude
request is an implemented response, not a guarantee of a safe trajectory.

The reviewed `tzi_final.py` stops issuing new guidance updates while waiting
for a target and reports `[HEARTBEAT]` after 5 s without one. This review
records no corresponding PID reset. The independent sender may retain its
last active setpoints, so actual autopilot behavior must be evaluated with
the command thread as well as the vision loop.

## Startup behavior

The reviewed `tzi.py` initialized `TestCommander` with
`self.cmd_thread.update(self.test_heading_deg, self.test_alt_m, self.test_throttle_pct)`.
That activates default targets, including a 50 m altitude, before any bbox
has arrived. It can therefore send commands without a detected target.

In the reviewed `tzi_final.py`, the initialization call is absent and the
sender starts with `self.active = False`. Its first update follows a bbox
and the associated control calculation. This distinction matters when
checking startup behavior and test overrides in each archived implementation.
