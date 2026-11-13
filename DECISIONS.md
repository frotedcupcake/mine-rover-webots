# Decisions made without asking

Each entry: what was decided, and why. Items that go beyond the brief are marked **addition**.

## Setup
- **Python 3.11 from Homebrew for `.venv`** (not the pyenv 3.10 that `python3` resolves to).
  Webots R2025a accepted it; `runtime.ini` in both controller folders points at it by absolute path.
- **ffmpeg installed with Homebrew.** It worked, so video is enabled.
- **Textures are generated locally** (`tools/make_textures.py`). Webots R2025a ships no
  texture or proto library on disk; it downloads them from GitHub on first use. Local
  procedural JPEGs mean nothing is fetched at runtime.
- **Contact sound URLs are blanked** in every world for the same reason.

## World
- **Walls carry dense vertical rock ribs with real collision geometry.** Webots' sonar model
  ignores any ray that meets a surface more than 22.5 degrees off normal, so a flat wall is
  invisible unless the rover faces it almost square. With ribs every 10–12 cm the sonars get
  an echo at any angle, which is how rough rock behaves. Cost: about 350 static cylinders in
  the mine.
- **Roof is a one-sided plane with no collision.** Visible from inside, see-through from
  above, so an overview shot is possible.
- **A pipe and a cable run along one wall instead of a rail track.** Rails on the floor
  would be obstacles for 65 mm wheels.
- **The entrance is a blocked adit mouth** (a wall with a pale "daylight" panel and a timber
  frame), so the rover cannot drive out of the world.
- **Mine layout:** tunnel A 9.2 m along +X, 90 degree left bend, tunnel B 7 m along +Y
  (about 15.6 m of centreline). Side bay with the 25 degree ramp on the right near the
  start; rubble at 3.1 m and 4.6 m; pit over the left half at 5.9–6.7 m, 0.5 m deep; third
  rubble pile and the gas source in tunnel B. The rover does not visit the ramp bay in AUTO;
  the `ramp` test world covers the ramp.
- **Tunnel height 1.0 m, pit depth 0.5 m, 3–4 dim lamps per world.**
- **Rubble piles are static** (no physics) and made mostly of spheres so the sonar sees them
  from any side.

## Rover model
- **Track 0.18 m is narrower than the 0.20 m chassis**, so the chassis is drawn as a narrow
  body between the wheels with a 0.20 m deck over them. Collision is body box + deck.
- **Ground clearance 0.03 m** so the front overhang clears the foot of a 25 degree ramp.
- **Forward IRs are mounted about 8 cm above the floor.** Any lower and they see the ramp
  surface as an obstacle and the rover never climbs far enough to measure the tilt.
- **Down IRs sit on the wheel centre-lines**, so if an IR sees floor at least half the wheel
  behind it is supported.
- **Wheel torque limit 0.15 N.m**, friction 1.0. Top speed 0.4 m/s at 100 %.
- **IR colour sensitivity off** (`redColorSensitivity 0`); dark rock would otherwise read as
  far away.
- **IMU noise and gyro bias are added in the controller**, not in the proto, so the bias
  calibration has something to remove. Motor dead zone (35 %) and a +-3 % left/right mismatch
  are also in the controller.
- **An emitter was added** to the rover (the brief lists only a receiver) to send telemetry
  to the supervisor for the label, metrics and scenario checks. **addition**
- **Amber status beacon** with a faint point light, so the rover is visible in the dark.
  **addition**
- **Headlamp is on by default** (`lamp_default_on` in `rover.yaml`); sweeps force it on.

## Keyboard
- **A and D each had two meanings in the brief.** In MANUAL they steer. Outside MANUAL,
  A selects AUTO and D drops the rock. `Shift+A` / `Shift+D` do the mode / rock action from
  any mode.
- **`V` toggles the chase camera.** **addition**
- **`R` is refused while the level is still DANGER.**

## Fusion
- Reason codes: `GAS_BUILDUP`, `GAS_EXTREME`, `ROOF_FALL`, `FLOOR_LOST`, `TILT`,
  `STALE_DATA`, `GAS_ELEVATED`, `OBSTACLE`, `CLEAR`. Highest priority wins.
- **Additions asked for in the brief:** `GAS_EXTREME` (3000 ppm steady), `GAS_ELEVATED`
  (300 ppm, off at 240), `STALE_DATA` (0.5 s).
- **Roof fall, three additions made while testing:**
  1. The required drop in distance grows by 0.4 m/s after the impact
     (`roof_approach_mps`), so driving towards something already there does not count.
  2. If the rover turns more than 15 degrees after the impact the impact is dropped
     (`roof_max_turn_deg`). Found on the ramp: backing off the ramp bumped the floor, then
     the turn brought a wall into view and it was reported as a roof fall.
  3. A forward IR that trips within 1.5 s of the impact, and was not tripped before, also
     counts as the new obstacle (`roof_ir_window_s`). The two sonars leave a blind strip
     dead ahead and some rock falls landed entirely inside it.
- **GAS_ELEVATED uses its hysteresis instead of the 2 s hold; ROOF_FALL uses its 15 s latch.**
- **Staleness is not judged in the first second**, and stale data does not trigger a sweep.
- **"Err closer" for the ultrasonic median:** three echoes give the median; two echoes and
  a miss give the closer echo; one echo among misses is ignored.
- **Impacts are ignored during the 2 s gyro calibration.**
- **The accelerometer only corrects tilt when |a| is within 0.15 g of 1 g**, so braking does
  not tip the estimate. **addition**

## Navigator
- **AVOID leaves only when both sonars read 50 cm clear for 0.25 s *and* one of them sees at
  least 1 m of room.** Facing a side wall from the middle of a 1.2 m tunnel reads just over
  50 cm, which alone would let the rover exit pointing at a wall. **addition**
- **Route heading (gyro yaw).** Set when AUTO starts, kept through back-to-back AVOIDs, and
  re-based only after 3 s of uninterrupted CRUISE (then low-pass, tau 2 s). No encoders on the
  real rover, so it is time-based, not distance-based. **addition (step-0 fix, 2026-11-13)**
- **AVOID only finishes within +/-100 degrees of the route heading.** It sweeps between the two
  sector limits instead of spinning freely. Replaces the old "reverse the spin after 100
  degrees" rule, which let two chained AVOIDs turn the rover round at the third rubble pile
  (89 -> 150 -> 250 degrees). **addition (step-0 fix)**
- **Stop-and-confirm exit.** On the first clear reading inside the sector the rover stops, waits
  0.2 s, then needs 0.25 s of clear readings. If the check fails it turns on the same way and
  ignores clear readings for the next 10 degrees. No clear direction after 8 s -> STUCK "no
  clear direction found" (to become RETURN). **addition (step-0 fix)**
- **The only route bend in the mine is +90 degrees** (tunnel A -> B), 10 degrees inside the
  sector; in all logged mine runs it was taken in CRUISE (route lag at most 30.6 degrees).
- **One-sided pit: after the 60 degree turn the rover sidesteps for 1.3 s, then turns back
  to its original heading**, then biases away from that side for 8 s. A plain 60 degree turn
  in a 1.2 m tunnel just points the rover at the opposite wall. **addition**
- **Bias is a virtual obstacle at 0.60 m on the pit side.**
- **Tilt back-off keeps reversing until tilt is under 10 degrees** (at most 3 s). Pitch tilt
  turns 160 degrees; roll tilt turns 60 degrees away from the high side.
- **Only pits count towards "3 in 2 minutes".** The third pit goes straight to STUCK.
- **Gentle centring** when both sonars see a wall within 1.6 m and nothing is near; otherwise
  the gyro heading is held. **addition**
- **Floor-lost motor cut and the dead-man run every 5 ms step**, not just at 20 Hz.
- **The rover waits for gyro calibration and gas warm-up (2 s) before moving.**
- **Sweep on CAUTION pauses the rover, then it continues.** DANGER sweeps, then holds.

## Supervisor, scenarios, reports
- **Collision = any contact point on the rover higher than 12 mm above the wheel bottoms**
  (walls, rocks, chassis strikes). Wheel-on-floor contacts are not collisions.
- **Fell in pit = chassis more than 12 cm below its start height.**
- **Roof fall drops three rocks** across the tunnel (see fusion note 3) and applies three
  10 ms pushes of about 45 N.
- **Run folder name comes from the supervisor** and is radioed to the rover so both write to
  the same `reports/<run>/`.
- **Two independent seeds.** Noise seed (`MINE_NOISE_SEED`): `WorldInfo.randomSeed` (through a
  temporary copy of the world), IMU/gas noise and motor mismatch. Shift seed (`MINE_SHIFT_SEED`):
  start-pose shift (0 = none) and where dropped rocks land. The rover's default noise seed is 1,
  matching `randomSeed 1` in the world files, so plain `./run.sh` is exactly noise 1 / shift 0.
  Runner: `--seed N` (both N), `--noise-seed`, `--shift-seed`, `--combos NOISE:SHIFT`; run
  folders are `<scenario>_n<noise>s<shift>`. Legacy `MINE_SEED` is still read as a fallback.
- **The `roof_fall` scenario is the corridor world plus a timed event**; there is no
  separate rock-fall world.
- **`side_pit` and `roof_fall` do not check collisions**, because the brief lists no
  collision check for them (dropped rocks sometimes touch the rover).
- **Camera images turned out to work with `--no-rendering`** on this machine (the mine runs save 15 images that way). `gas_ahead` still keeps rendering on, as the brief suggested, because its check needs the images.
- **Video and screenshots run in real time with the window not minimised.** A minimised
  Webots window records black frames.
- **The rover camera overlay is hidden for video/screenshot runs** by writing a small
  `.wbproj` next to the temporary world.
- **The overview screenshot adds a temporary light and clears the fog** for that one frame.

## RETURN mode (commit 2, 2026-11-13)
The design is the user's (HANDOFF.md section 5). These are the choices made while building it;
each was reported at checkpoint 1.
- **The turn-round is the normal AVOID with one extra rule.** On RETURN the route heading is
  turned by 180 degrees and an AVOID starts. Until the rover is within `return_turn_tol_deg`
  (20) of the new route, clear readings are ignored; from there the usual sector and
  stop-and-confirm exit apply. Without that rule the rover would leave the spin at the first
  clear reading inside the sector, which in open ground is 100 degrees off the way home.
- **A manoeuvre that interrupts the turn-round resumes it.** A sweep, a Danger hold or a
  back-off that ends while the rover still faces more than 100 degrees off the route goes back
  to turning, not to CRUISE (which would drive it further in).
- **Blocked-ahead back-offs reverse first, then turn back.** Full-width pit, slope too steep and
  the third pit all do the normal sweep and reverse (0.7 s, or until level) before RETURN, so
  the rover never spins with its nose over an edge. The old 160 degree turn
  (`backoff_turn_both_deg`) is gone; the turn-round replaces it.
- **Only pitch tilt counts as "slope too steep".** Roll tilt (one side riding up on something)
  still turns 60 degrees away and carries on, as before.
- **"3 pits in 2 minutes" is counted per leg.** The count restarts on RETURN, otherwise the
  half-width pit in the mine (met on the way in and again on the way back) could add up to a
  false "blocked both ways".
- **HOME needs the turn-round to be finished.** If the link never dropped (blocked within 3 m of
  the entrance, as in `pit_ahead` and `ramp`), the rover still turns to face out and only then
  is HOME. HOME is checked in CRUISE only.
- **Driving time = time in CRUISE, AVOID or BACKOFF.** Sweeps, holds and CALIBRATING are not
  counted. HOME by time needs return time >= 1.2 x outward time.
- **Danger hold timer starts after the sweep** and is not restarted by later sweeps in the same
  hold. `R` (link up, level no longer DANGER) still resumes early.
- **Danger re-arm.** A Danger finding the rover has held for is ignored until it clears: gas
  build-up until the reading is below `gas_danger_ppm`, extreme gas until below
  `gas_extreme_ppm`, roof fall until its latch expires. Fusion still reports DANGER; only the
  stop is suppressed. Sweeps still run.
- **Battery.** `rover_core/battery.py`. The navigator gets a `BatteryStatus` whose level comes
  from the 5 s average voltage alone and latches (OK -> RETURN -> WARN -> CRITICAL). Nothing is
  judged until 5 s of data exist. CRITICAL zeroes the motors in every mode; in AUTO the state
  becomes STUCK "battery critical" (HOME stays HOME). `R` is refused while critical, and a new
  trip from HOME is refused once the return level has latched. Config load fails unless
  return_v > warn_v > critical_v.
- **Link.** Up at <= 3.0 m from the entrance, down beyond 3.3 m (`link_hyst_m`). The entrance is
  the rover's nominal start position (before any start shift). With the link down every key
  except `D` and `V` is ignored, including Space.
- **Upload = flush, write summary.html, copy the run folder to `reports/<run>/dashboard/`.**
  Without the link it retries every `upload_retry_s` (2 s).
- **Scenario files may override config** (`cfg:`), passed to both controllers as `MINE_CFG`;
  used for the low starting charge and the shortened mission timer.
- **Scenario event triggers use `when:`** (`return`, `home`, `hold`, `reach:<region>`), not
  `on:`, because YAML reads a bare `on` as `true`.
- **`max_gas_distance_m` is now measured where the rover held**, not at its final position
  (which is HOME). `home_max_entrance_m` is 3.35 = link range + hysteresis.
