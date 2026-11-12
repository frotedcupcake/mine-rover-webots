# STATUS — work stopped early (usage limit reached)

The build is **incomplete**. The session hit its usage limit part-way through section 6.

## Built and actually run
- Setup: Webots R2025a confirmed, `.venv` (Python 3.11 from Homebrew) with all packages,
  ffmpeg installed via Homebrew, `run.sh`, `run_headless.sh`, `runtime.ini` for both controllers.
- `protos/`: MineRover, Rubble, GasMarker. Loads and drives in Webots (0.28 m/s at 70 %).
- `tools/build_worlds.py` generates all 7 worlds (mine + 6 test worlds) and the gas yaml files.
- `rover_core/` (fusion, navigator, attitude, gas model, sweep, hazard log) and `config/rover.yaml`.
- `tests/`: **82 pytest tests pass** (`.venv/bin/python -m pytest tests`).
- Both controllers and the scenario runner (`scenarios/run_scenario.py`, `scenarios/run_all.sh`).
- Scenario runs so far: **only `obstacles` seed 1 was run — PASS** (reached the end, 0 collisions).

## NOT done / not verified
- The other 6 scenarios (pit_ahead, side_pit, ramp, gas_ahead, roof_fall, mine) have never been run.
  Expect tuning to be needed. No pass rates exist yet.
- `scenarios/run_all.sh` (5 seeds each) has not been run.
- Camera sweep images, screenshots, the hazard label, chase camera and video recording are
  written but untested. The mine world has never been looked at in the GUI.
- No demo screenshots or video in `reports/demo/`.
- `README.md` and `DECISIONS.md` are not written.

## Decisions made so far (to go into DECISIONS.md)
- Webots R2025a ships no textures on disk, so textures are generated locally
  (`tools/make_textures.py`); nothing is downloaded at runtime.
- Webots sonar ignores rays hitting a surface at more than 22.5 degrees, so flat walls were
  invisible. Walls carry dense vertical rock ribs (real collision geometry) so they echo.
- IR sensors use `redColorSensitivity 0` (dark rock would otherwise read as "far").
- IMU noise, gyro bias, motor dead zone and motor mismatch are added in the controller.
- Key clash in the spec: A and D steer only in MANUAL; otherwise A = AUTO, D = drop rock
  (Shift+A / Shift+D work in any mode).
- A cable/pipe runs along the wall instead of a rail track (rails would trip the wheels).
- Roof fall drops three rocks, because the two sonars leave a blind strip dead ahead.
- AVOID also needs 1 m of open room on one sonar before it exits, and reverses direction
  after 100 degrees, so the rover does not drive back out at the bend.

## What to do next
1. `.venv/bin/python scenarios/run_scenario.py <name> --seed 1` for each remaining scenario;
   read `reports/<name>_s1/webots.log` and fix what fails.
2. `./run.sh mine` to look at the world and check lighting.
3. `scenarios/run_all.sh`, then write README.md and DECISIONS.md and make the demo outputs.
