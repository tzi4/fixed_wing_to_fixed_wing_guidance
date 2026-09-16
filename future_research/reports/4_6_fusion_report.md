# Guidance fusion analysis and implementation plan

Prepared by Claude Opus 4.6 (Thinking), June 22, 2026.

This historical note compares five source implementations and records the
Opus 4.6 fusion proposal. Tables describe the reviewed source snapshots.
Reported flight experience does not establish validation of every calculated
command, especially where the sender contains overrides or disabled calls.
The archived fused implementation is
[`tzi_4.6_fused.py`](../guidance/tzi_4.6_fused.py).

## 1. Code Comparison Table

### 1.1 Overview

| Feature | tzi.py | tzi2.1.py | tzi_final.py | goat_gimbal.py | initial_camp_emir.py |
| --- | --- | --- | --- | --- | --- |
| **Test status** | ✅ Worked in simulation | ❌ Never tested | ❌ Not tested | ✅ Reported flight test | ✅ Reported flight test |
| **Architecture** | System→tziGuidance + TestCommander | System→tziGuidance + TestCommander | System→tziGuidance + TestCommander | Monolithic (Redis+Mav+Controller) | Monolithic (Redis+Mav+Controller) |
| **Resolution** | 640×480 | 640×480 | 1280×720 | 1280×720 | 1280×720 |
| **Error format** | Pixels | Pixels | Angular (degrees) | Angular (degrees) | Angular (degrees) |
| **Airspeed method** | TRIM_THROTTLE | DO_CHANGE_SPEED (178) | DO_CHANGE_SPEED (178) | DO_CHANGE_SPEED (178) — fixed | DO_CHANGE_SPEED (178) — PID |
| **CSV Log** | ❌ | ❌ | ❌ | ✅ | ✅ |
| **Coasting/Failsafe** | ❌ | ❌ | ✅ | ✅ | ✅ |

---

### 1.2 Heading Control (Detailed)

| Parameter | tzi.py | tzi2.1.py | tzi_final.py | goat_gimbal.py | initial_camp_emir.py |
| --- | --- | --- | --- | --- | --- |
| **Controller type** | PD | PD | PD (angular) | P + Rate PID | P + Rate PID |
| **Error source** | `u_virt - center` (px) | `u_virt - center` (px) | `atan(px/fx)` (°) | `atan(px/fx)` (°) | `atan(px/fx)` (°) |
| **Kp** | 0.03125 px⁻¹ | 0.0469 px⁻¹ | 0.255 °⁻¹ | 3.0 °⁻¹ | 3.0 °⁻¹ |
| **Kd** | 0.02 | 0.02 | 0.163 | 0 | 0 |
| **Deadzone** | None | None | 0.78° | 0.78° | 0.78° |
| **Heading rate** | **40** deg/s (fixed!) | **40** deg/s (fixed!) | **40** deg/s (fixed!) | 0.85–5.0 (dynamic PID) | 0.85–**7.5** (dynamic PID) |
| **Max delta** | ±10° | ±15° | ±10° | ±35° | ±35° |
| **param1** | 1 (raw heading) | 1 (raw heading) | 1 (raw heading) | 0 (course over ground) | 0 (course over ground) |
| **Low-pass deriv** | None | None | None | ✅ α=0.05 | ✅ α=0.05 |

> ⚠️ **param1 difference is critical!** While tzi codes use `param1=1` (raw magnetic heading = nose direction), Emir implementations use `param1=0` (course over ground). In windy weather, the COG is affected by the wind and may deviate from the direction of the nose. It should be tested which one works better in air.

---

### 1.3 Altitude Control (Detailed)

| Parameter | tzi.py | tzi2.1.py | tzi_final.py | goat_gimbal.py | initial_camp_emir.py |
| --- | --- | --- | --- | --- | --- |
| **Controller type** | PD | PD | PD (angular) | PD (Kd=0.12) | PD (Kd=0.1) + Rate PID |
| **Kp** | 0.0208 px⁻¹ | 0.0417 px⁻¹ | 0.170 °⁻¹ | 0.7 °⁻¹ | 1.5 °⁻¹ |
| **Kd** | 0.01 | 0.01 | 0.0816 | 0.12 | 0.1 |
| **Max delta** | ±5 m | ±10 m | ±2 m | ±10 m | ±10 m |
| **Alt frame** | **AMSL** | **AMSL** | **AMSL** | **Relative** | **Relative** |
| **Climb rate** | 0 (default) | 0 (default) | 0 (default) | **0.8 m/s (fixed)** | **0.3–5.0 m/s (dynamic PID)** |

The reviewed TZI sources read `loc_msg.alt` (AMSL), while the Emir sources
read `msg.relative_alt` (relative to home). Relative-to-home altitude is not
generally height above terrain. Verify that the value supplied to
`GUIDED_CHANGE_ALTITUDE` matches its `MAV_FRAME_GLOBAL_RELATIVE_ALT` command
frame. Mixing AMSL and relative values can create an altitude offset.


---

### 1.4 Airspeed Control (Detailed)

| Parameter | tzi.py | tzi2.1.py | tzi_final.py | goat_gimbal.py | initial_camp_emir.py |
| --- | --- | --- | --- | --- | --- |
| **Method** | **TRIM_THROTTLE** (param_set) | DO_CHANGE_SPEED | DO_CHANGE_SPEED | DO_CHANGE_SPEED **(FIXED)** | DO_CHANGE_SPEED **(PID)** |
| **PID entry** | bbox sqrt_area (px) | bbox sqrt_area (px) | bbox angular (°) | — (fixed 20 m/s) | coverage % (EMA) |
| **Kp / Ki / Kd** | 4.7 / 0.45 / 0.6 | 0.78 / 0.075 / 0.10 | 6.377 / 0.610 / 0.814 | — | 0.2 / 0.03 / 0 |
| **Base** | 50 (throttle) | 15 m/s | 15 m/s | 20 m/s | 17 m/s |
| **Range** | 50–127 (throttle) | 10–22.8 m/s | 10–22.8 m/s | — | 14–22 m/s |
| **Slew rate** | None | None | ✅ 2.0 m/s² | — | ✅ 1.0 m/s/s |
| **GUIDED init** | None | None | ✅ (current speed is taken) | — | None |
| **Integral anti-windup** | Asymmetric clamp | Asymmetric clamp | Asymmetric clamp | — | Band-limited + clamp |

> **TRIM_THROTTLE and DO_CHANGE_SPEED:** TRIM_THROTTLE provides indirect speed control by changing the TECS cruise throttle parameter — it does not bypass TECS, but the range is narrow and the effect is non-linear. `DO_CHANGE_SPEED (178)` provides safer control by giving target speed directly to TECS (stall protection remains). The fact that tzi.py did well with TRIM_THROTTLE is probably due to the lack of wind/turbulence in the simulation.

---

## 2. Architectural Comparison

### TZI architecture (tzi.py, tzi2.1.py, tzi_final.py)

```
┌─────────────┐     ┌────────────────┐     ┌─────────────────┐
│ RedisListener│────▶│ VisionProcessor│────▶│ guide_aircraft() │
│ (Thread) │bbox │ (30 Hz Thread) │ │ calculate PID/PD │
└─────────────┘     └────────────────┘     └────────┬────────┘
                                                     │ update()
                    ┌────────────────┐               │
                    │ MAVLinkReader  │     ┌─────────▼──────────┐
                    │ (cache Thread) │     │  TestCommander      │
                    └────────────────┘     │  (5 Hz Thread)      │
                                           │  heading+alt+speed  │
                                           └─────────────────────┘
```

**Advantages:**
- Command sending (5 Hz) and visual processing (30 Hz) are fully separated
- MAVLink reading from cache → no race condition
- TestCommander continues to send the last targets even if the bbox does not arrive (inherent coasting)
- Clean inheritance: `System` → `tziGuidance`

### Emir architecture (goat_gimbal.py, initial_camp_emir.py)

```
┌─────────────────┐     ┌──────────────────┐
│ RedisListener    │────▶│   data_queue      │
│ (Thread)         │     │   (Queue)         │
└─────────────────┘     └────────┬──────────┘
                                  │ get()
┌─────────────────┐     ┌────────▼──────────┐
│ MavlinkManager  │◀───▶│ AutopilotController│
│ (Thread + Lock)  │     │ (Main Thread loop) │
└─────────────────┘     └───────────────────┘
```

**Advantages:**
- Queue-based data flow (producer-consumer pattern)
- CSV logging (perfect for offline analysis)
- Coasting/Failsafe mechanism clearly coded
- Heading rate and altitude rate dynamically controlled by PID

**Limitations:**
- `recv_match` is under lock in MavlinkManager → this may block telemetry reading when sending commands
- Command sending depends on the main loop (there is a limit of 10 Hz, but if the bbox does not arrive, it will go into coasting)

---

## 3. Findings in the reviewed sources

### Disabled heading and altitude sends in `goat_gimbal.py`

```python
# goat_gimbal.py, line 549-551:
# self.mavlink.send_heading_target(hdg, rate) # ← DISABLED
# self.mavlink.send_altitude_target(alt) # ← DISABLED
self.mavlink.send_airspeed_target(spd)              # ← ONLY THIS IS ACTIVE
```

The reviewed source sends only airspeed. A reported flight test must be matched to its exact source version before attributing heading or altitude performance to this implementation.

### Three active command paths in `initial_camp_emir.py`

```python
# initial_camp_emir.py, line 668-670:
self.mavlink.send_heading_target(target_heading, heading_rate)  # ✅ ACTIVE
self.mavlink.send_altitude_target(target_alt, altitude_rate)     # ✅ ACTIVE
self.mavlink.send_speed_target(cmd_speed)                        # ✅ ACTIVE
```

Among the sources described here as flight-tested, this implementation has active heading, altitude, and speed send calls.

### 🟡 tzi.py Heading Rate = 40 deg/s

In the `_send_heading` function in tzi.py, `param3=40` (heading rate) is sent as a constant 40 deg/s. That's a very aggressive spin rate. Even if it works in simulation, it can be dangerous in the air. initial_camp_emir's dynamic heading rate approach (0.85–7.5 deg/s) is safer.

### Heading and altitude overrides in `tzi_final.py`

tzi_final.py TestCommander line 181-182:
```python
hdg = 0.0   # ← OVERRIDE! PID output is bypassed
alt = 40.0  # ← OVERRIDE! PID output is bypassed
```

These overrides are consistent with airspeed-only tuning. This snapshot does not demonstrate applied heading or altitude PID control.

---

## 4. Fusion Strategy — Opus 4.6 Proposal

### Recommended Approach: **TZI architecture with Emir rate controllers**

```
tzi.py architecture ──────────────┐
(Thread Separation, TestCommander) │
                                │
initial_camp_emir.py ─────────────┼──▶ FUSION CODE (tzi_fusion.py)
(Flight-derived PID, Rate PID) │
                                │
tzi_final.py ──────────────────┤
(Angular error, Deadzone, Slew) │
                                │
goat_gimbal.py ────────────────┘
(CSV logging, LP Derivative)
```

### Source contributions

| Source Code | Feature | Rationale |
| --- | --- | --- |
| **tzi.py** | Architectural structure (System→Guidance, TestCommander thread, MAVLink reader) | Separate vision, telemetry, and command threads |
| **initial_camp_emir.py** | Heading PID + Rate PID, Altitude PD + Rate PID, Coverage-based Speed ​​PID, Anti-windup, Coasting/Failsafe | Reported flight experience with three active command paths |
| **tzi_final.py** | Angular error calculation (`atan`), Deadzone mechanism, Slew rate limit, GUIDED entry initialization | Angular units and constrained transitions |
| **goat_gimbal.py** | CSV logging (FlightLogger class), Low-pass filtered derivative | Indispensable for offline analysis |

---

## 5. TestCommander Update

The reviewed TZI TestCommander sends `(heading, altitude, throttle/airspeed)`. **heading_rate** and **altitude_rate** will also be added for fusion code:

```python
# Updated TestCommander.update() signature:
def update(self, heading_deg, heading_rate_dps, alt_m, alt_rate_mps, airspeed_ms):
    with self.lock:
        self.target_heading_deg = float(heading_deg)
        self.target_heading_rate = float(heading_rate_dps)
        self.target_alt_m = float(alt_m)
        self.target_alt_rate = float(alt_rate_mps)
        self.target_airspeed_ms = float(airspeed_ms)
        self.active = True
```

```python
# Updated _send_heading():
def _send_heading(self, heading_deg, heading_rate_dps):
    self.master.mav.command_long_send(
        ...,
        43002,
        0,
        1,                  # param1: raw magnetic heading (tzi approach)
        heading_deg,        # param2: target heading
        heading_rate_dps,   # param3: DYNAMIC rate (Emir approach)
        0, 0, 0, 0
    )
```

---

## 6. Historical implementation plan

### Phase 1: simulation benchmark (estimated one day)
1. Run `initial_camp_emir.py` in simulation (active on three control channels)
2. Run `tzi.py` in simulation (with TRIM_THROTTLE)
3. Extract performance metrics from CSV logs

### Phase 2: fusion implementation (estimated 2–3 days)
> The historical note records source creation by Opus 4.6. See the archived implementation linked above.

### Phase 3: controlled flight tests (estimated 1–2 days)

The proposed sequence isolates control channels before combining them:
```
Test 1: heading=fixed, altitude=fixed → airspeed ONLY PID
Test 2: airspeed=constant, altitude=constant → ONLY heading PID
Test 3: airspeed=fixed, heading=fixed → altitude PID only
Test 4: active in heading+altitude → airspeed fixed
Test 5: All three → FULL CONTROL
```

### Phase 4: offline PID tuning (estimated 1–2 days)
1. **Log collection:** Make a few flights with the fusion code, collect logs CSV
2. **System identification:** Step response analysis (command → actual response)
3. **Transfer function:** Fit model for each axis
4. **PID tuning:** Optimal Kp, Ki, Kd with Ziegler-Nichols / Python `control` library
5. **Verification:** Test in simulation with new parameters → verify in air

---

## 7. Additional Ideas & Advanced Strategies

### Idea 1: dual-mode speed control

A proposed fallback uses cruise-throttle control when direct airspeed control is unresponsive. This proposal requires separate validation:
```
if DO_CHANGE_SPEED is running:
    airspeed = DO_CHANGE_SPEED PID
else if airspeed is not responding (> 5s stale):
    airspeed = TRIM_THROTTLE PID (tzi.py mode)
```

### Idea 2: adaptive heading rate

The proposal varies rate with angular error:
- Small angular error (< 5°): 0.85–5 deg/s (Emir approach)
- Large angular error (> 15°): 10–40 deg/s (tzi approach)

### Idea 3: Kalman target tracking

A proposed filter would smooth bbox measurements and predict target motion during detection loss:
- State: `[target_x, target_y, target_vx, target_vy, target_area]`
- During Coasting: Estimated target position with Kalman → feed to PID

---

## 8. Open Questions

> **1. What exactly did the goat_gimbal test in the air?**
> Was only airspeed sent when the heading and altitude commands were disabled? Or was a different version used in the airborne test?

> **2. How did the initial_camp_emir perform in the air?**
> What problems occurred while working actively on the three control channels?

> **3. param1: raw heading (1) vs course over ground (0)**
> Which one should be preferred in windy weather?

> **4. What are the real camera intrinsic parameters?**
> - goat_gimbal: fx=3045.737, fy=3045.565 (HFOV=0.415 rad)
> - tzi_final: f=4711.91
> - tzi.py: f=467.7 (for 640×480)
> - **Which is the correct calibration data?**

> **5. Altitude reference: AMSL or relative to home?**
> Match the altitude value to the command frame. For
> `MAV_FRAME_GLOBAL_RELATIVE_ALT`, use altitude relative to home.


---

## 9. Generated Fusion Code

The historical prototype was named `tzi_fusion.py`. This repository archives it as `tzi_4.6_fused.py`.

### Recorded source contributions
| Source | Contribution |
| --- | --- |
| tzi.py | Architecture, TestCommander, virtual gimbal, MAVLink reader thread |
| initial_camp_emir.py | Heading P+Rate PID, Altitude PD+Rate PID, Coverage-based speed PID, anti-windup, coasting/failsafe |
| tzi_final.py | Angular error (`atan`), deadzone 0.78°, slew rate 1 m/s/s, GUIDED entry init |
| goat_gimbal.py | CSV FlightLogger, low-pass filtered derivative alpha=0.05 |

### Validation recorded in the historical note
- ✅ Python syntax check passed
- ✅ All module imports are successful
- ⏳ Simulation test not performed
- ⏳ Flight test not performed

---

*This report and the code `tzi_fusion.py` were prepared by Claude Opus 4.6 (Thinking).*
