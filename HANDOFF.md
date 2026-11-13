# HANDOFF — read this before doing anything

Written 2026-11-13 at the end of a session, for a new session that has no other context.
The project is a Webots R2025a simulation of an ECE357 "Autonomous Mine Rover for Damage and
Danger Assessment". README.md explains the layout; DECISIONS.md has every tuning decision;
STATUS.md has the overall state. This file explains **where the work stopped and what to do
next**.

---

## 1. How the user wants to work (follow exactly)

- The user is available. **Ask whenever a decision is theirs; never ask about things you can
  verify yourself** (run it, read the code, read the logs).
- Check-ins are short: **what you found, the options, your recommendation.**
- **Ask before changing navigator or fusion behaviour** (`rover_core/navigator.py`,
  `rover_core/fusion.py`, their config). Small obvious bugs (crashes, typos, test-harness
  errors) may be fixed directly; say so afterwards.
- **Never loosen a check to make it pass.** If a check itself looks wrong, explain why and let
  the user decide. Never reword a STUCK/HOME note just to satisfy a `final_note_contains`.
- **After any logic fix, rerun all earlier test layers** to catch regressions.
- Verify by actually running Webots. Report outcomes faithfully, failures included.
- Commit after each milestone. Use the same author as the existing commits
  (`git log -1 --format='%an <%ae>'`, pass it with `git -c user.name=... -c user.email=...`).
  The author email is private: never send it to any external service.
- Standing constraints from the original brief: do **not** install Webots with Homebrew;
  `inertial_unit` is ground truth for metrics only, **never** used by the autonomy; the
  navigator may only use inputs the real rover has (sonars, IRs, IMU gyro/accel, gas, battery
  voltage, the radio link). **No wheel encoders** on the real rover.
- A **GateGuard hook** on this machine blocks the first Bash command of a session, the first
  Edit/Write of each file, and destructive commands until you state some facts (the request
  in one sentence, what the command does; for edits: who imports the file, affected
  functions, data format, the user's instruction verbatim). State them in your message and
  retry.

## 2. Commands

```
./run.sh                                   # GUI, real time; = noise seed 1, shift seed 0
./run_headless.sh mine                     # same setup, no window
.venv/bin/python -m pytest -q              # unit tests (89 pass at 486d510)
.venv/bin/python scenarios/run_scenario.py mine --noise-seed 1 --shift-seed 0
.venv/bin/python scenarios/run_scenario.py --all --combos 1:0 0:0 1:1 2:2 3:3 4:4 5:5 --jobs 3
```
Each run writes `reports/<scenario>_n<noise>s<shift>/` (metrics.json with the checks,
events.jsonl, hazard_log.csv, summary.html, sweeps/, webots.log). `reports/` is git-ignored
except `reports/scenario_results.md`, `reports/scenario_results_seeds6-20.md` (pre-step-0)
and `reports/demo/`. Webots runs are deterministic:
identical seeds give identical runs, GUI or headless.

---

## 3. Step 0 — done (commit `486d510`)

**Bug:** in the `./run.sh` setup the rover turned round at the third rubble pile (~56 s) and
drove back toward the entrance. Cause: two chained AVOIDs. The first started at 89 deg,
spun right, reversed after 100 deg (the old rule), exited at ~150 deg on a reading taken
mid-spin; a second AVOID 0.04 s later took it to ~250 deg.

**What changed** (`rover_core/navigator.py`, `config/rover.yaml` navigator section):
- **Route heading** (gyro yaw): set at AUTO start; reset when switching into AUTO; kept
  across back-to-back AVOIDs; re-based only after `route_settle_s` = **3 s of uninterrupted
  CRUISE**, then low-pass with `route_tau_s` 2 s (`_update_route`). Time-based because the
  real rover has no encoders. A sweep that returns to CRUISE restarts the 3 s.
- **AVOID only finishes within +/-`avoid_max_dev_deg` (100 deg) of the route heading**; it
  sweeps between the two sector limits instead of spinning freely (`_start_avoid`, `_avoid`).
- **Stop-and-confirm exit:** first clear reading inside the sector -> stop (CONFIRM),
  settle `confirm_settle_s` 0.2 s, then need `clear_time_s` 0.25 s of clear readings. If the
  check fails, turn on the same way and ignore clear readings for `confirm_skip_deg` 10 deg.
  Clear = min(sonar L, R) >= 0.5 m and max >= 1.0 m and no front IR.
- **8 s with no exit -> STUCK "no clear direction found"** (to become RETURN in commit 2).
- **Seeds decoupled.** Noise seed `MINE_NOISE_SEED` (WorldInfo.randomSeed, IMU/gas noise,
  motor mismatch); shift seed `MINE_SHIFT_SEED` (start-pose shift, 0 = none; dropped-rock
  placement). Rover default noise seed is 1 = world `randomSeed 1`, so `./run.sh` is exactly
  n1s0. Runner flags `--seed N` (both), `--noise-seed`, `--shift-seed`, `--combos N:S`.
  Legacy `MINE_SEED` is still read as a fallback.
- `route_deg` logged in hazard_log.csv, nav events and the controller status line.

**Evidence:**
- `./run.sh` x3 with the window open (`reports/mine_20261113_121939/_121941/_121943`):
  identical runs; passes the third pile in CRUISE (no AVOID), closest to the gas 1.26 m,
  first DANGER 75.4 s, final HOLD, 0 collisions, no fall, path 12.35 m, ~91 s sim.
- The setup the user first watched (world noise 1, rover noise 0): AVOID at 56.7 s spins
  right, stops to check, leaves 0.7 s later, reaches the gas.
- Mine on 21 combos (1:0, 0:0, 2..10:0, 1:1..1:10): **21/21**. n1s4 was 215 s / 33.5 m
  (turned round) before the fix; now 91 s / 12.4 m. AVOIDs exit at most 13 deg off route.
- Unit tests: **89 passed** (old exit-while-spinning test replaced by a stricter
  stop-and-confirm test; 5 new tests for the sector, back-to-back AVOIDs, route re-basing).
- Regression, 8 scenarios x 7 combos (1:0, 0:0, 1:1..5:5): **53/56**
  (`reports/scenario_results.md`).

**Bend list (checked against the +/-100 deg sector):** the mine has one route bend, tunnel
A -> tunnel B, **+90 deg** (left), 10 deg inside the limit. The side bay (-90 deg, ramp) is
scenery and never entered. Test worlds are straight. In all 25 mine runs with route
logging the bend was taken in CRUISE; route lag peaked at 30.6 deg and caught up within ~5 s.

## 4. Known failure: `pit_ahead` 4/7

- Fails: n0s0, n3s3, n5s5, check `final note contains 'route blocked'`; they end STUCK
  "no clear direction found". **All three: 0 collisions, no fall**, stopped safely ~18 s after
  the second backoff.
- Why: when **both** floor IRs see the full-width pit, BACKOFF turns 160 deg. The rover then
  cruises > 3 s back toward the adit, so the route heading re-bases to the reverse
  direction (~-143 deg). At the adit's end wall nothing is open within +/-100 deg -> timeout.
- Why the other 4 pass: their start shift makes **one** IR hit the pit first, so they get
  one-sided sidesteps that never turn them round, and the third pit within 2 min gives
  STUCK "route blocked (pits)".
- The user chose to commit with this recorded (not to change the check). **Commit 2 fixes it
  by design:** a full-width pit counts as blocked ahead -> RETURN -> HOME, and `pit_ahead`'s
  expectation changes to that (an approved, stricter expectation, not a loosened one).

---

## 5. Commit 2 — RETURN mode (approved, NOT started)

No operator is assumed underground. STUCK-at-the-first-blockage is replaced by a RETURN mode.
RETURN is a **direction of travel, not new driving behaviour**: CRUISE, AVOID (sector +
stop-and-confirm), BACKOFF, sweeps, HOLD, the 5 ms floor-lost reflex and the Danger rules
are reused unchanged; only the target heading changes.

```
IN ──trigger──▶ RETURN ──link up, or return driving time ≥ 1.2 × outward driving time──▶ HOME → upload log
                   └── blocked again ──▶ STUCK "blocked both ways" (full log flushed first)
DANGER (either way): stop → finish sweep → log → HOLD danger_hold_s (30 s) → RETURN / keep returning
```

**Triggers for RETURN**
1. **Blocked ahead:** the whole +/-100 deg sector blocked after the 8 s AVOID timeout;
   **a full-width pit** (both IRs, the "both" backoff); **a too-steep slope** (TILT backoff);
   **3 pits in 2 minutes**.
2. **Battery at the return reserve** — see below.
3. **Mission timer expired** — `mission_time_s` 900 (15 min), a backstop.
4. **Danger hold finished** — on DANGER: stop, complete the sweep, log, hold
   `danger_hold_s` 30 s, then RETURN.

**Steering on RETURN:** target heading = route heading + 180 deg, then the route heading
re-bases after 3 s of cruising exactly as on the way in (so the bend is taken on the way
back). **No path retracing** (no encoders). Same avoidance, IR floor reflex and Danger rules.

**Battery (user's change — voltage only):** the real rover measures only battery voltage
(divider on the Arduino Uno A1); there is no current sensor. So:
- Sim battery outputs a **terminal voltage that sags under motor load** (open-circuit
  voltage falls with charge used; minus an internal-resistance drop proportional to motor
  command). Charge is internal to the sim model only.
- The navigator sees **only the voltage**. Return reserve = a **filtered voltage below a
  threshold** (low-pass, so a load sag alone does not trigger it). **No charge counting**
  in the navigator.
- Critical battery (lower threshold) -> flush the log, then STUCK "battery critical".
- Mission timer stays as the backstop.
- **Open item — ask the user before picking numbers:** pack chemistry/cell count, full and
  empty voltages, divider ratio, and the reserve/critical thresholds. Nothing in the repo
  states them (README says the sim has "no battery sag"). Tests must be able to set the
  starting charge and force a sudden drop.

**Danger re-arm (user's change):** gas can trigger a new stop on the way back, but only after
the reading has **fallen below the threshold and risen again** — the gas the rover is
retreating from does not stop it again and again. Add a scenario with a **second gas pocket
on the return path**.

**Dashboard link (supervisor stands in for the laptop at the entrance):** link up within
`link_range_m` **3 m** of the entrance (with hysteresis), sent in the existing 20 Hz radio
message; no message for 0.5 s = link down.
- Rover commands (M, drive keys, X, Space, R, C, L) only work with the link up.
  D (drop a rock) and V (camera) are simulation controls and always work.
- MANUAL / R stays an **optional** override when the link is up; nothing depends on it.
- R from HOME starts a new trip in (route heading flipped back toward the mine).

**HOME and upload:** HOME when the link comes back **or** return driving time >= 1.2 x
outward driving time (`return_time_factor` 1.2; times count driving only, not sweeps or
holds). Then upload: flush the log, write the summary, copy the run folder to
`reports/<run>/dashboard/`, log it. If the link is down at HOME (the time-limit case) the
upload waits and retries until the link appears.

**STUCK "blocked both ways":** blocked again while returning. **Flush the full log to disk
first** (`hazard_log.flush()` with fsync, also called at RETURN and HOME).

**Telemetry / metrics.json gain:** direction (in/return), return reason, HOME reached,
uploaded, battery voltage, link state.

**New config** (`config/rover.yaml`): `danger_hold_s 30`, `mission_time_s 900`,
`link_range_m 3.0`, `return_time_factor 1.2`, battery model + thresholds (see open item).
`route_settle_s 3` and `avoid_max_dev_deg 100` already exist.

**Scenario expectations (stricter, not looser):**
- `mine`, `gas_ahead`: hold >= `danger_hold_s` within 2.5 m of the gas, then RETURN, then
  HOME with the upload done. **The `./run.sh` setup must still reach the gas; stopping at
  the third rubble pile is a FAIL.**
- `roof_fall`: hold, then RETURN, then HOME.
- `pit_ahead` (full-width pit): RETURN, then HOME.
- **Two ramp scenarios:** a **climbable** ramp the rover drives over and continues past
  (needs a new, gentler ramp world; today's `ramp` world is 25 deg, over the 20 deg tilt
  limit), and the **too-steep** ramp giving RETURN then HOME.
- **New RETURN scenarios:** blocked ahead (dead end), blocked both ways (supervisor drops a
  full-width barrier behind the rover), battery reserve, Danger then return, mission timer,
  second gas pocket on the return path.

**Approved defaults:** Danger hold **30 s**, mission time **15 min**, link range **3 m**.

**Not built, from the earlier plan, only if needed:** if the third pile's 0.46 m gap ever
defeats stop-and-confirm, the spin may back up ~0.2 m and rescan within the same 8 s. Step 0
did not need it.

**Where it goes:** `rover_core/navigator.py` (`_auto`, `_avoid` timeout at ~line 394,
`_start_backoff` "both"/pits at ~line 428, `step` key handling ~line 137), a new
`rover_core/battery.py` used by `controllers/rover_controller/rover_controller.py`,
`rover_core/hazard_log.py` (`flush`), `controllers/mine_supervisor/mine_supervisor.py`
(link, barrier event, second gas pocket, `evaluate()` checks), `tools/build_worlds.py`
(new worlds), `scenarios/*.yaml`, tests in `tests/`.

---

## 6. After commit 2: the test campaign (user's plan)

**Step 1 — plan first, wait for the user's OK.** Show a short plan: how each layer is built,
how long the full run takes on this Mac (measure a few runs), and how many Webots instances
run in parallel (runner default is 3).

**Layers**
1. **Unit tests:** every fusion rule, navigator transition, guard and attitude sign, plus
   edge cases: exact thresholds, all sensors "no echo", gas flickering around thresholds,
   impact during a sweep. Plus the RETURN transitions.
2. **Scenario matrix:** every scenario on **20 noise-seed x start-shift combinations plus the
   exact `./run.sh` setup (n1s0)**. Fail on collision, pit fall, heading back toward the
   entrance (only wrong while travelling *in*; needs a ground-truth depth along the tunnel
   centreline from the supervisor), wrong or late hazard level, missing or incomplete sweep,
   timeout. Includes the RETURN cases: blocked ahead, blocked both ways, battery reserve,
   Danger then return (and the second gas pocket, mission timer, both ramps).
3. **Combined hazards:** gas near the pit, rock dropped mid-sweep, gas rising during backoff,
   obstacle on the ramp, rubble close together. Highest hazard must win.
4. **50 procedurally generated random mines** (random bends, rubble, pits, ramps, gas), fixed
   generator seeds so failures reproduce. Invariants only: no fall, no collision, Danger
   always stops, always ends in a sane state with a reason.
   - **Main set: bends <= 90 deg** (the +/-100 deg sector leaves 10 deg margin at 90).
   - **Separate sharp-bend group, 100–120 deg, reported outside the pass rate.**
5. **Fault injection:** dead or stuck ultrasonic, IR stuck either way, IMU drift, gas data
   stopped or frozen, camera failing mid-sweep, controller freezing 1 s (dead-man must stop
   the motors), critical battery. Must fail safe.
6. **Endurance:** 30 simulated minutes of AUTO in the mine with resumes; watch control-loop
   timing, memory, drift, log size, loops.

**Step 2 — build and run one layer at a time.** After each layer report the pass rate and
every failure in 3–5 lines. For each real bug give the root cause and proposed fix and ask
before changing navigator/fusion behaviour. Rerun earlier layers after any logic fix.

**Step 3 — wrap up.** `./test_all.sh` runs everything (`--quick` = a few-minute subset);
`reports/test_campaign.md` with pass rate per layer, every failure, root cause, fix, anything
still failing, total runtime; run `./run.sh` as the user would and show it reaches the gas,
holds, returns and reaches HOME (the user will also watch it with the window open); update
STATUS.md; commit.

---

## 7. Exact next step

1. Send the user one short check-in asking for the **battery numbers** (pack chemistry and
   cell count, full/empty voltage, divider ratio on A1) and proposing reserve/critical
   voltage thresholds and a filter time constant. That is the only open decision in the
   RETURN design.
2. Then build commit 2 exactly as in section 5. Unit-test the RETURN transitions first, then
   add the worlds/scenarios, then run the full regression (all scenarios x 1:0, 0:0,
   1:1..5:5) and `./run.sh` (must reach the gas, hold 30 s, return, HOME, upload).
   `pit_ahead` must then pass 7/7 against its new RETURN -> HOME expectation.
3. Report in the user's format and commit. **Then** Step 1 of the campaign (plan only, wait
   for OK).
