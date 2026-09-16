# Visual guidance research archive

This directory contains the 13 August 2026 snapshot of research extending the
optical tracking foundation in `tzi.py`. These research variants have not all
been developed into complete competition or real-flight systems.

Many files implement working experiments, but the directory as a whole remains
**under development**. Parameters depend on specific camera, aircraft, and SITL
configurations. The code should not be deployed directly on a real aircraft.

## Range estimation from target telemetry

`guidance/teva.py` calculates the three-dimensional separation between the two
aircraft from target positions received at approximately 1 Hz from the
competition server and the pursuing aircraft's `GLOBAL_POSITION_INT` messages.
It uses dead reckoning between target samples and returns to image-only behavior
when telemetry becomes stale.

The implemented scope is:

- Target telemetry supplies **range information** for the completed
  **range-dependent altitude-axis controller**.
- Heading and target position within the camera frame come from the camera.
- Telemetry is not used to track a target that is no longer visible.
- Horizontal-axis control, speed control, and integrated real-flight validation
  remain research tasks.

Further work includes safe fusion of telemetry and vision, range-dependent gain
scheduling, closing-speed management, horizontal-axis extensions, and safe
reacquisition after target loss.

## Directory layout

| Directory | Contents |
| --- | --- |
| `guidance/` | TEVA, fused variants, `tzi_final`, `tzi_emir`, and training-camp experiments |
| `detection/` | Color/SiamRPN and earlier detection pipelines publishing Redis `tracker_bbox` |
| `simulation/` | Gazebo/SITL launchers, formation, mission upload, and speed helpers |
| `telemetry_tools/` | Server simulator, actual-position logging, range calibration, and analysis |
| `tests/` | Unit tests for target lock, bounding-box selection, and TEVA logic |
| `reports/` | Technical comparisons of fusion and control variants |
| `historical_experiments/` | Virtual-gimbal and earlier control experiments |

The reproducible Erenimbus SITL environment is under `../bumblebee/`. Its
`teva.py`, tools, mission files, and verification workflow provide the working
integration of this research.

Run the tests from the repository root:

```bash
python3 -m unittest \
  bumblebee.test_teva_logic \
  future_research.tests.test_teva_logic \
  future_research.tests.test_guidance_logic
```

## Handover

This archive records Tarık Z. İnci's final handover of this research. Sources,
experiment reports, and supporting tools are provided together so the team can
continue the work. Future changes should distinguish experimental results from
behavior validated in flight and preserve source attribution.
