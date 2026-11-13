# Autonomous Mine Rover for Damage and Danger Assessment — Webots simulation

ECE357 project. A 4WD skid-steer rover drives into a dark mine tunnel, senses hazards with
cheap sensors (2 ultrasonic, 4 IR, an IMU, a gas sensor, a pan camera), fuses them into
SAFE / CAUTION / DANGER with a reason, reacts, and writes a report with photographs.

Current state, pass rates and known gaps are in [STATUS.md](STATUS.md). Design choices that
were not in the brief are in [DECISIONS.md](DECISIONS.md).

## Install (macOS, Apple Silicon)

1. Webots R2025a at `/Applications/Webots.app` (already installed here).
2. Python environment and textures:
   ```bash
   /opt/homebrew/bin/python3.11 -m venv .venv
   .venv/bin/pip install pytest numpy opencv-python-headless pyyaml pillow
   .venv/bin/python tools/make_textures.py     # procedural rock/wood textures (already committed)
   .venv/bin/python tools/build_worlds.py      # regenerates worlds/*.wbt and config/gas_sources/*.yaml
   ```
3. `controllers/*/runtime.ini` hold the **absolute** path to `.venv/bin/python3`. If you move
   the project folder, edit both files.
4. `brew install ffmpeg` is only needed for `tools/make_demo.sh` (video compression).

## Run the demo

```bash
./run.sh                 # opens worlds/mine.wbt in the Webots GUI; the rover starts in AUTO
./run.sh side_pit        # any test world: corridor obstacles pit_ahead side_pit gas_ahead ramp
./run_headless.sh mine   # no window, as fast as possible (Ctrl-C to stop)
```

Click the 3D view once so it has keyboard focus. Webots will warn that the GPU is below its
recommended spec; that is expected. Each run writes `reports/<world>_<date>_<time>/`.

## Keyboard

| Key | Action |
|---|---|
| `M` | MANUAL mode |
| `W` `S` | forward / reverse (MANUAL). Arrow keys work too |
| `A` `D` | turn left / right **in MANUAL** |
| `A` | AUTO mode **when not in MANUAL** (`Shift+A` works from any mode) |
| `D` | drop a roof-fall rock ahead **when not in MANUAL** (`Shift+D` from any mode) |
| `X` | STOP mode |
| `Space` | emergency stop (STOP mode) |
| `R` | resume after HOLD or STUCK (refused while the level is still DANGER); at HOME: start a new trip in |
| `C` | camera sweep now |
| `L` | headlamp on/off |
| `V` | chase camera on/off (off lets you move the view yourself) |

Manual forward is refused over a drop or within 15 cm of an obstacle. Keys held on the
keyboard time out after 0.4 s (dead-man).

**Rover commands need the dashboard link.** `M`, the drive keys, `A`, `X`, `Space`, `R`, `C`
and `L` only reach the rover while the link is up, which is within 3 m of the entrance (the
second line of the on-screen label shows `link UP` / `link down`). `D` and `V` are simulation
controls and always work. Nothing depends on an operator: see RETURN below.

## Trip: in, RETURN, HOME

No operator is assumed underground. The rover drives in until something tells it to turn back,
then drives out on the reverse of its route heading with the same avoidance, floor reflex and
Danger rules, and is HOME when the link comes back (or after 1.2 x its outward driving time).
At HOME it flushes the log and uploads the run folder to `reports/<run>/dashboard/`; if the
link is down it retries every 2 s.

| Turns back (RETURN) when | Note in the log |
|---|---|
| nothing open in the +-100 degree sector after the 8 s AVOID | `blocked ahead` |
| both floor IRs lose the floor | `full-width pit` |
| pitch tilt over 20 degrees | `slope too steep` |
| 3 pits within 2 minutes | `3 pits in 2 minutes` |
| battery: 5 s average below 10.5 V (latched) | `battery reserve` |
| 15 minutes after the trip started | `mission timer` |
| DANGER: stop, sweep, log, hold 30 s | `danger hold finished` |

Blocked again on the way back: STUCK "blocked both ways", with the full log flushed first.
Battery 5 s average below 9.6 V: STUCK "battery critical" (latched, log flushed). Below 10.2 V
is a warning only (label, status line, `summary.html`).

## Tests and scenarios

```bash
.venv/bin/python -m pytest tests                              # 84 unit tests, no Webots needed
.venv/bin/python scenarios/run_scenario.py gas_ahead --seed 3 # one scenario, headless
.venv/bin/python scenarios/run_scenario.py mine --gui         # watch a scenario in real time
scenarios/run_all.sh                                          # all 8 scenarios x 5 seeds -> reports/scenario_results.md
tools/make_demo.sh                                            # screenshots + video -> reports/demo/ (opens windows, ~5 min)
```

| Scenario | World | What must happen |
|---|---|---|
| `obstacles` | obstacles | reaches the far end, 0 collisions |
| `pit_ahead` | pit_ahead | CAUTION FLOOR_LOST, never falls, RETURN (pit), HOME, upload |
| `side_pit` | side_pit | gets past the half-width pit, never falls |
| `ramp` | ramp | too-steep ramp (25 deg): CAUTION TILT, back-off, RETURN (slope too steep), HOME |
| `ramp_gentle` | ramp_gentle | climbable 10 deg hump: drives over it and carries on, no TILT, no RETURN |
| `gas_ahead` | gas_ahead | DANGER GAS_BUILDUP, 5 sweep images, hold 30 s within 2.5 m, RETURN, HOME, upload |
| `roof_fall` | corridor | rocks dropped at t = 7 s give DANGER ROOF_FALL, hold 30 s, RETURN, HOME |
| `mine` | mine | no collision, no fall, reaches the gas, DANGER, hold 30 s, RETURN, link back, HOME, upload |
| `gas_dropout` (addition) | corridor | gas radio silent for 3 s gives CAUTION STALE_DATA and a stop, then the rover carries on |
| `blocked_ahead` | corridor | dead end: RETURN (blocked ahead), HOME, upload |
| `blocked_both` | corridor | dead end, then a barrier dropped behind: STUCK "blocked both ways", log flushed first |
| `battery_reserve` | mine | low starting charge: RETURN (battery reserve), HOME, upload |
| `mission_timer` | long_corridor | timer cut to 25 s: RETURN (mission timer), HOME, upload |
| `danger_return` | gas_return | long tunnel: DANGER, hold, RETURN, link back, HOME, upload |
| `gas_return` | gas_return | as above, plus a second gas pocket on the way back: second hold, then it keeps returning |

Each run has two seeds. The noise seed sets the Webots sensor noise, the IMU/gas noise and the
motor mismatch; the shift seed sets the start-pose shift (0 = none) and where dropped rocks land.
`--seed N` sets both; `--noise-seed`/`--shift-seed` set one, and `--all --combos 1:0 2:7` takes
NOISE:SHIFT pairs. `./run.sh` is noise seed 1, shift seed 0. Scenario files are plain yaml (`scenarios/*.yaml`): world, duration, timed events
(`roof_fall`, `gas_dropout`, `cmd`), an early-finish condition and the expected outcomes.
`run_scenario.py` exits with code 0 on pass and 1 on fail (it reads the verdict from `metrics.json`).

## What a run produces (`reports/<run>/`)

- `hazard_log.csv` — 5 rows per second: level, reason, navigator state, every reading, motor commands
- `events.jsonl` — hazard, level, navigator, impact, sweep and image events
- `sweeps/*.jpg` — captioned photographs (time, level, reason, gas, distance, tilt, pan angle)
- `metrics.json` — path length, collisions, fell_in_pit, duration, hazards seen, scenario checks
- `summary.html` — hazard events with their photographs, navigator timeline
- `webots.log` — console output including the one-per-second status line (scenario runs)

## Layout

```
worlds/        mine.wbt + 6 test worlds (generated by tools/build_worlds.py)
protos/        MineRover.proto, Rubble.proto, GasMarker.proto, textures/
controllers/   rover_controller/ (robot), mine_supervisor/ (world side)
rover_core/    pure Python, no Webots: conditioning, attitude, fusion, navigator, sweep, gas model, hazard log
config/        rover.yaml (every threshold and speed), gas_sources/<world>.yaml
tests/         pytest for rover_core
scenarios/     *.yaml, run_scenario.py, run_all.sh
tools/         make_textures.py, build_worlds.py, make_demo.sh
meshes/        Fusion 360 export goes here
reports/       run outputs; reports/demo/ holds the screenshots and video
```

## Importing Fusion 360 meshes

1. In Fusion 360 export the chassis body as **STL** (binary is fine) or OBJ. Set the export
   units to **metres**. Orient the model so **X is forward, Y is left, Z is up**, with the
   origin at the centre of the 0.25 x 0.20 x 0.07 m chassis box.
2. Save it as `meshes/chassis.stl` (a plain box placeholder is there now).
3. In the world file (or in `tools/build_worlds.py`, in the `DEF ROVER MineRover` block) add:
   ```
   DEF ROVER MineRover {
     useMeshes TRUE
     chassisMesh "../meshes/chassis.stl"
   }
   ```
4. Only the picture changes. Collision stays the simple boxes and cylinders, so driving
   behaviour and test results do not change. If the model looks 1000 times too big, it was
   exported in millimetres.

Every rover dimension is a field of `MineRover.proto` (chassis size, wheel diameter and
width, wheelbase, track, ground clearance, sensor angles and offsets, pan range and speed,
camera resolution), so the model can follow the real build without editing the proto body.

## What is real and what is faked

| Thing | In the simulation | Honest note |
|---|---|---|
| Ultrasonic (HC-SR04) | Webots `DistanceSensor` type "sonar": 12 rays in a 15 degree cone, nearest ray wins, 1 % noise, 60 ms | Webots drops any ray that hits a surface more than 22.5 degrees off normal, so smooth flat walls are invisible at a glancing angle. The mine walls carry dense vertical rock ribs (real collision geometry) so they echo like rough rock. A real HC-SR04 has a wider, lumpier beam and multipath echoes that are not modelled |
| Sonar blind strip | The two sonars point 25 degrees outward, so nothing dead ahead and narrower than about 30 cm is seen until the forward IRs trip at 12 cm | This is geometry, and it is true of the real layout too |
| Forward / downward IR | `DistanceSensor` type "infra-red", thresholded in software (12 cm obstacle, 4 cm floor lost), debounced over 3 samples | Surface colour sensitivity is switched off (`redColorSensitivity 0`). Real IR modules read dark rock as farther away and are upset by sunlight |
| IMU (MPU6050) | Ideal Webots accelerometer and gyro at 5 ms, plus white noise and a constant gyro bias added in the controller | No temperature drift, no vibration from real motors |
| Gas sensor | The supervisor computes `peak(t) * exp(-d^2 / 2 sigma^2)` at the rover and radios it. The rover adds noise, a 1 s lag and a 2 s warm-up | Not a gas simulation: no airflow, no diffusion, no cross-sensitivity. The green sphere is only a marker |
| Roof fall | The supervisor spawns three rocks about 0.4 m ahead and shoves the chassis for 30 ms | The shove stands in for debris striking the rover |
| Motors | Velocity-controlled wheels, 0.15 N.m limit, dead zone below 35 % command, +-3 % left/right mismatch | No gearbox backlash; the battery does not slow the motors |
| Battery | `rover_core/battery.py`: a simulated 3S 4000 mAh pack (open-circuit voltage from the charge left, minus current x 0.15 ohm) outputs what the Uno would send, `batt_x10` = ADC counts x 10. The rover converts it to volts and judges a 5 s average | Draw (2 A driving, 0.6 A idle), internal resistance and the voltage curve are placeholders until measured on the real pack. The navigator never sees charge or current |
| Dashboard link | The supervisor stands in for the laptop at the entrance: link up within 3 m (drops 0.3 m further out), sent as a heartbeat in the 20 Hz radio message; no message for 0.5 s = link down | Real radio range depends on the tunnel; 3 m is a stand-in |
| Camera | Webots camera 640x480, 60 degree FOV, on a +-120 degree pan at 60 deg/s | No motion blur, no dust, no lens distortion |
| `inertial_unit` | Ground truth, used only to report the attitude filter's error in `metrics.json` | Never used by the autonomy |

## Tuning (`config/rover.yaml`)

All thresholds and speeds are in this one file; restart the simulation after editing.
Useful ones:

- `fusion.gas_danger_ppm`, `gas_slope_ppm_s`, `tilt_deg`, `obstacle_m` — the proposal's thresholds
- `fusion.roof_*` — roof-fall timing and distances
- `navigator.slow_dist_m`, `avoid_dist_m`, `clear_dist_m` — when to slow, spin, and stop spinning
- `navigator.backoff_*`, `sidestep_s`, `bias_s` — the pit / tilt back-off manoeuvre
- `drive.cruise`, `slow`, `spin` — speeds as fractions of the 0.4 m/s top speed (keep them above `min_cmd`)
- `imu.*_noise*`, `gas.noise_ppm`, `drive.motor_mismatch` — how imperfect the simulated hardware is

Gas sources are per world in `config/gas_sources/<world>.yaml`. That folder is regenerated by
`tools/build_worlds.py`, so change the numbers in the `gas_source(...)` calls there.
Items marked `(addition)` in `rover.yaml` are not in the proposal.

## Proposal mapping: Enter / Sense / Fuse / Report

| Proposal step | Where it lives | What it does |
|---|---|---|
| **Enter** | `rover_core/navigator.py`, `protos/MineRover.proto` | AUTO mode drives in: CRUISE (slow below 80 cm, steer to the open side), AVOID (spin until clear), BACKOFF (reverse, turn away, sidestep), HOLD, STUCK. MANUAL and STOP modes from the keyboard. Guards: no forward drive over a lost floor, stop on stale data, 0.4 s dead-man |
| **Sense** | `rover_core/conditioning.py`, `attitude.py`, `gas_model.py`, `controllers/rover_controller/` | Ultrasonic median-of-3, IR debounce, complementary filter (alpha 0.98) with gyro-bias calibration, impact detection, gas sensor lag and warm-up |
| **Fuse** | `rover_core/fusion.py` | The proposal's table, evaluated every 50 ms: gas above threshold and rising -> DANGER; impact + new obstacle -> DANGER roof fall; floor lost or tilt -> CAUTION; plain obstacle -> SAFE. Findings hold 2 s; entering CAUTION/DANGER requests a camera sweep |
| **Report** | `rover_core/sweep.py`, `caption.py`, `hazard_log.py`, `controllers/mine_supervisor/` | Five captioned photographs per sweep, `hazard_log.csv`, `events.jsonl`, `metrics.json`, `summary.html`, a status line each second, and the coloured hazard label in the 3D view |
