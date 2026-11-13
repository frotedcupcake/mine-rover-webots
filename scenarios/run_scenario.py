#!/usr/bin/env python3
"""Run one scenario (or all of them) in Webots and report pass/fail.

    .venv/bin/python scenarios/run_scenario.py gas_ahead --seed 3
    .venv/bin/python scenarios/run_scenario.py mine --gui            # watch it in real time
    .venv/bin/python scenarios/run_scenario.py --all --seeds 1 2 3 4 5 --jobs 3

Each run writes reports/<scenario>_s<seed>/ (metrics.json, hazard_log.csv, events.jsonl,
summary.html, sweeps/, webots.log).  The seed changes the start pose, the simulated
sensor noise (Webots randomSeed + the controller's IMU/gas noise) and motor mismatch.
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


def run_one(name, seed=1, gui=False, render=None, video=False, shots=False, timeout=900, run_name=None, quiet=False):
    spath = os.path.join(ROOT, "scenarios", f"{name}.yaml")
    with open(spath) as fh:
        sc = yaml.safe_load(fh)
    render = bool(sc.get("render")) if render is None else render
    render = render or gui or video or shots
    run = run_name or f"{name}_s{seed}"
    run_dir = os.path.join(ROOT, "reports", run)
    if os.path.isdir(run_dir):
        shutil.rmtree(run_dir)                  # only ever this run's own previous output
    os.makedirs(run_dir)
    # seeded copy of the world (Webots sensor noise follows WorldInfo.randomSeed)
    src = os.path.join(ROOT, "worlds", f"{sc['world']}.wbt")
    tmp = os.path.join(ROOT, "worlds", f"_run_{run}.wbt")
    with open(src) as fh:
        text = fh.read()
    text = re.sub(r"randomSeed \d+", f"randomSeed {int(seed)}", text, count=1)
    with open(tmp, "w") as fh:
        fh.write(text)
    proj = os.path.join(ROOT, "worlds", f"._run_{run}.wbproj")
    if video or shots:
        # hide the rover camera overlay so it does not cover the recorded 3D view
        with open(proj, "w") as fh:
            fh.write("Webots Project File version R2025a\nrenderingDevicePerspectives: rover:camera;0;1;0;0\n")
    env = dict(os.environ, MINE_SCENARIO=spath, MINE_SEED=str(seed), MINE_RUN=run,
               MINE_RENDER="1" if render else "0", MINE_VIDEO="1" if video else "0",
               MINE_SHOTS="1" if shots else "0")
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
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            status = "timeout"
    wall = time.time() - t0
    for p in (tmp, os.path.join(ROOT, "worlds", f"._run_{run}.wbproj")):
        if os.path.exists(p):
            os.remove(p)
    result = {"scenario": name, "seed": seed, "run": run, "wall_s": round(wall, 1), "status": status,
              "passed": False, "checks": [], "metrics": {}}
    mpath = os.path.join(run_dir, "metrics.json")
    if os.path.exists(mpath):
        with open(mpath) as fh:
            m = json.load(fh)
        result["metrics"] = m
        result["checks"] = m.get("checks", [])
        result["passed"] = bool(m.get("passed")) and status == "ok"
        if "passed" not in m:
            result["status"] = "no verdict (simulation ended early?)"
    else:
        result["status"] = "no metrics.json"
    if not quiet:
        print_result(result)
    return result


def print_result(r):
    m = r["metrics"]
    print(f"{'PASS' if r['passed'] else 'FAIL'}  {r['scenario']:<10} seed {r['seed']}  sim {m.get('duration_s', '?')} s  "
          f"wall {r['wall_s']} s  path {m.get('path_length_m', '?')} m  collisions {m.get('collisions', '?')}  "
          f"fell {m.get('fell_in_pit', '?')}  final {m.get('final_nav', '?')}  [{r['status']}]", flush=True)
    for c in r["checks"]:
        if not c["ok"]:
            print(f"        FAILED: {c['check']}  ({c['detail']})", flush=True)


def run_all(names, seeds, jobs, **kw):
    todo = [(n, s) for n in names for s in seeds]
    results = []
    with cf.ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = {ex.submit(run_one, n, s, quiet=True, **kw): (n, s) for n, s in todo}
        for fut in cf.as_completed(futs):
            r = fut.result()
            print_result(r)
            results.append(r)
    results.sort(key=lambda r: (names.index(r["scenario"]), r["seed"]))
    write_report(results, names, seeds)
    return results


def write_report(results, names, seeds):
    lines = ["# Scenario results", "",
             f"Generated {time.strftime('%Y-%m-%d %H:%M')} by `scenarios/run_all.sh` "
             f"(seeds {', '.join(map(str, seeds))}; headless fast mode).", "",
             "| scenario | " + " | ".join(f"seed {s}" for s in seeds) + " | pass rate |",
             "|---|" + "---|" * (len(seeds) + 1)]
    total = ok_total = 0
    for n in names:
        rs = {r["seed"]: r for r in results if r["scenario"] == n}
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
    ap.add_argument("--seed", type=int, default=1)
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
        results = run_all(names, a.seeds, a.jobs, **kw)
        sys.exit(0 if all(r["passed"] for r in results) else 1)
    if not a.name:
        ap.error(f"give a scenario name: {', '.join(scenario_names())}")
    r = run_one(a.name, a.seed, run_name=a.run_name, **kw)
    sys.exit(0 if r["passed"] else 1)


if __name__ == "__main__":
    main()
