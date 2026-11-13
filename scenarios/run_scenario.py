#!/usr/bin/env python3
"""Run one scenario (or all of them) in Webots and report pass/fail.

    .venv/bin/python scenarios/run_scenario.py gas_ahead --seed 3              # noise 3, shift 3
    .venv/bin/python scenarios/run_scenario.py mine --noise-seed 1 --shift-seed 0   # = ./run.sh
    .venv/bin/python scenarios/run_scenario.py mine --gui            # watch it in real time
    .venv/bin/python scenarios/run_scenario.py --all --seeds 1 2 3 4 5 --jobs 3

Two independent seeds per run:
  noise seed  Webots WorldInfo.randomSeed (sensor noise) + the rover's IMU/gas noise and
              motor mismatch.  The world files say randomSeed 1 and the rover defaults to 1.
  shift seed  start-pose shift from the scenario's start_jitter (0 = no shift) and where
              dropped rocks land.
./run.sh is exactly noise seed 1, shift seed 0.  --seed N means noise N and shift N;
--combos takes NOISE:SHIFT pairs, e.g. --combos 1:0 2:7.
Each run writes reports/<scenario>_n<noise>s<shift>/ (metrics.json, hazard_log.csv,
events.jsonl, summary.html, sweeps/, webots.log).
"""
import argparse
import concurrent.futures as cf
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import time

import yaml

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
WEBOTS = os.environ.get("WEBOTS", "/Applications/Webots.app/Contents/MacOS/webots")


def scenario_names():
    return sorted(os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(ROOT, "scenarios", "*.yaml")))


def combo_label(c):
    return f"n{c[0]}s{c[1]}"


def run_one(name, combo=(1, 1), gui=False, render=None, video=False, shots=False, timeout=900, run_name=None, quiet=False):
    spath = os.path.join(ROOT, "scenarios", f"{name}.yaml")
    with open(spath) as fh:
        sc = yaml.safe_load(fh)
    render = bool(sc.get("render")) if render is None else render
    render = render or gui or video or shots
    noise, shift = combo
    run = run_name or f"{name}_{combo_label(combo)}"
    run_dir = os.path.join(ROOT, "reports", run)
    if os.path.isdir(run_dir):
        shutil.rmtree(run_dir)                  # only ever this run's own previous output
    os.makedirs(run_dir)
    # seeded copy of the world (Webots sensor noise follows WorldInfo.randomSeed)
    src = os.path.join(ROOT, "worlds", f"{sc['world']}.wbt")
    tmp = os.path.join(ROOT, "worlds", f"_run_{run}.wbt")
    with open(src) as fh:
        text = fh.read()
    text = re.sub(r"randomSeed \d+", f"randomSeed {int(noise)}", text, count=1)
    with open(tmp, "w") as fh:
        fh.write(text)
    proj = os.path.join(ROOT, "worlds", f"._run_{run}.wbproj")
    if video or shots:
        # hide the rover camera overlay so it does not cover the recorded 3D view
        with open(proj, "w") as fh:
            fh.write("Webots Project File version R2025a\nrenderingDevicePerspectives: rover:camera;0;1;0;0\n")
    env = dict(os.environ, MINE_SCENARIO=spath, MINE_NOISE_SEED=str(noise), MINE_SHIFT_SEED=str(shift), MINE_RUN=run,
               MINE_RENDER="1" if render else "0", MINE_VIDEO="1" if video else "0",
               MINE_SHOTS="1" if shots else "0")
    env.pop("MINE_SEED", None)          # legacy single seed: must not override the two seeds above
    # per-scenario config overrides (e.g. a low starting charge), read by both controllers
    env["MINE_CFG"] = json.dumps(sc.get("cfg") or {})
    if gui:
        cmd = [WEBOTS, "--mode=realtime", "--stdout", "--stderr", tmp]
    else:
        # video/screenshots need every frame drawn: real time. Everything else: fast mode.
        mode = "realtime" if (video or shots) else "fast"
        cmd = [WEBOTS, "--batch", f"--mode={mode}", "--stdout", "--stderr"]
        if not (video or shots):
            cmd.append("--minimize")        # a minimised window records black video frames
        if not render:
            cmd.append("--no-rendering")
        cmd.append(tmp)
    t0 = time.time()
    log_path = os.path.join(run_dir, "webots.log")
    status = "ok"
    with open(log_path, "w") as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=env)
        while True:
            try:
                proc.wait(timeout=3)
                break
            except subprocess.TimeoutExpired:
                pass
            if time.time() - t0 > timeout:
                status = "timeout"
            elif controller_died(log_path):
                status = "controller crashed"      # the simulation would otherwise idle until the timeout
            if status != "ok":
                proc.kill()
                proc.wait()
                break
    wall = time.time() - t0
    for p in (tmp, os.path.join(ROOT, "worlds", f"._run_{run}.wbproj")):
        if os.path.exists(p):
            os.remove(p)
    result = {"scenario": name, "combo": combo_label(combo), "run": run, "wall_s": round(wall, 1), "status": status,
              "passed": False, "checks": [], "metrics": {}}
    mpath = os.path.join(run_dir, "metrics.json")
    if os.path.exists(mpath):
        with open(mpath) as fh:
            m = json.load(fh)
        result["metrics"] = m
        result["checks"] = m.get("checks", [])
        result["passed"] = bool(m.get("passed")) and status == "ok"
        if "passed" not in m and status == "ok":
            result["status"] = "no verdict (simulation ended early?)"
    elif status == "ok":
        result["status"] = "no metrics.json"
    if not quiet:
        print_result(result)
    return result


def controller_died(log_path):
    """True once Webots has reported a controller exiting with an error."""
    try:
        with open(log_path, errors="replace") as fh:
            head = fh.read(20000)
    except OSError:
        return False
    return "controller exited with status" in head or "Traceback (most recent call last)" in head


def print_result(r):
    m = r["metrics"]
    print(f"{'PASS' if r['passed'] else 'FAIL'}  {r['scenario']:<10} {r['combo']:<7} sim {m.get('duration_s', '?')} s  "
          f"wall {r['wall_s']} s  path {m.get('path_length_m', '?')} m  collisions {m.get('collisions', '?')}  "
          f"fell {m.get('fell_in_pit', '?')}  final {m.get('final_nav', '?')}  [{r['status']}]", flush=True)
    for c in r["checks"]:
        if not c["ok"]:
            print(f"        FAILED: {c['check']}  ({c['detail']})", flush=True)


def run_all(names, combos, jobs, **kw):
    todo = [(n, c) for n in names for c in combos]
    results = []
    with cf.ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = {ex.submit(run_one, n, s, quiet=True, **kw): (n, s) for n, s in todo}
        for fut in cf.as_completed(futs):
            r = fut.result()
            print_result(r)
            results.append(r)
    order = [combo_label(c) for c in combos]
    results.sort(key=lambda r: (names.index(r["scenario"]), order.index(r["combo"])))
    write_report(results, names, combos)
    return results


def write_report(results, names, combos):
    seeds = [combo_label(c) for c in combos]
    lines = ["# Scenario results", "",
             f"Generated {time.strftime('%Y-%m-%d %H:%M')} by `scenarios/run_all.sh` "
             f"(noise/shift seeds {', '.join(seeds)}; headless fast mode).", "",
             "| scenario | " + " | ".join(seeds) + " | pass rate |",
             "|---|" + "---|" * (len(seeds) + 1)]
    total = ok_total = 0
    for n in names:
        rs = {r["combo"]: r for r in results if r["scenario"] == n}
        cells, ok = [], 0
        for s in seeds:
            r = rs.get(s)
            good = bool(r and r["passed"])
            ok += good
            cells.append("PASS" if good else "FAIL")
        total += len(seeds)
        ok_total += ok
        lines.append(f"| {n} | " + " | ".join(cells) + f" | {ok}/{len(seeds)} |")
    lines += ["", f"**Overall: {ok_total}/{total} runs passed.**", "", "## Failed checks", ""]
    fails = [r for r in results if not r["passed"]]
    if not fails:
        lines.append("None.")
    for r in fails:
        lines.append(f"- `{r['run']}` ({r['status']}): " + "; ".join(
            f"{c['check']} ({c['detail']})" for c in r["checks"] if not c["ok"]) or "no verdict")
    lines += ["", "## Per-run details", "",
              "| run | sim s | path m | collisions | fell | final nav | hazards seen | images |", "|---|---|---|---|---|---|---|---|"]
    for r in results:
        m = r["metrics"]
        lines.append(f"| {r['run']} | {m.get('duration_s', '')} | {m.get('path_length_m', '')} | {m.get('collisions', '')} | "
                     f"{m.get('fell_in_pit', '')} | {m.get('final_nav', '')} | {', '.join(m.get('hazards_seen', []))} | "
                     f"{m.get('sweep_images', '')} |")
    with open(os.path.join(ROOT, "reports", "scenario_results.md"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    slim = [{k: v for k, v in r.items() if k != "metrics"} for r in results]
    with open(os.path.join(ROOT, "reports", "scenario_results.json"), "w") as fh:
        json.dump(slim, fh, indent=1)
    print(f"\nOverall: {ok_total}/{total} passed. Table: reports/scenario_results.md", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", nargs="?", help="scenario name (see scenarios/*.yaml)")
    ap.add_argument("--seed", type=int, default=None, help="noise seed = shift seed = N")
    ap.add_argument("--noise-seed", type=int)
    ap.add_argument("--shift-seed", type=int)
    ap.add_argument("--combos", nargs="*", help="NOISE:SHIFT pairs for --all, e.g. 1:0 2:7")
    ap.add_argument("--all", action="store_true", help="run every scenario")
    ap.add_argument("--only", nargs="*", help="with --all: just these scenarios")
    ap.add_argument("--seeds", type=int, nargs="*", default=[1, 2, 3, 4, 5])
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--gui", action="store_true", help="open the Webots window, real time")
    ap.add_argument("--render", action="store_true", help="keep rendering on (camera images) even if the yaml says no")
    ap.add_argument("--video", action="store_true", help="record reports/<run>/run.mp4 (needs rendering)")
    ap.add_argument("--shots", action="store_true", help="save 3D-view screenshots at each hazard")
    ap.add_argument("--run-name")
    ap.add_argument("--timeout", type=int, default=900)
    a = ap.parse_args()
    kw = dict(gui=a.gui, render=True if a.render else None, video=a.video, shots=a.shots, timeout=a.timeout)
    if a.all:
        names = a.only or scenario_names()
        combos = [tuple(int(x) for x in c.split(":")) for c in a.combos] if a.combos else [(s, s) for s in a.seeds]
        results = run_all(names, combos, a.jobs, **kw)
        sys.exit(0 if all(r["passed"] for r in results) else 1)
    if not a.name:
        ap.error(f"give a scenario name: {', '.join(scenario_names())}")
    base = 1 if a.seed is None else a.seed
    combo = (base if a.noise_seed is None else a.noise_seed, base if a.shift_seed is None else a.shift_seed)
    r = run_one(a.name, combo, run_name=a.run_name, **kw)
    sys.exit(0 if r["passed"] else 1)


if __name__ == "__main__":
    main()
