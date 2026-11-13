"""Per-run report: hazard_log.csv (5 rows/s), events.jsonl, sweeps/, summary.html."""
import csv
import html
import json
import os

CSV_FIELDS = ["t", "level", "reason", "nav", "mode", "us_left_m", "us_right_m", "ir_front_left",
              "ir_front_right", "floor_lost_left", "floor_lost_right", "pitch_deg", "roll_deg",
              "yaw_deg", "gas_ppm", "gas_slope", "left_cmd", "right_cmd", "route_deg"]
LEVEL_COLOUR = {"SAFE": "#2e9e4f", "CAUTION": "#d98a00", "DANGER": "#d0312d"}


def _num(v, nd=3):
    if v is None or v == float("inf"):
        return ""
    return round(float(v), nd)


class HazardLog:
    def __init__(self, run_dir, csv_hz=5):
        self.run_dir = run_dir
        self.sweep_dir = os.path.join(run_dir, "sweeps")
        os.makedirs(self.sweep_dir, exist_ok=True)
        self._period = 1.0 / csv_hz
        self._next_row = 0.0
        self._csv_fh = open(os.path.join(run_dir, "hazard_log.csv"), "w", newline="")
        self._csv = csv.DictWriter(self._csv_fh, fieldnames=CSV_FIELDS)
        self._csv.writeheader()
        self._ev_fh = open(os.path.join(run_dir, "events.jsonl"), "w")
        self.events = []
        self.images = []                 # dicts: file, t, sweep, angle, level, reason

    # ---- rows -------------------------------------------------------
    def row(self, t, r, a, cmd):
        if t + 1e-9 < self._next_row:
            return False
        self._next_row = max(self._next_row + self._period, t - self._period)
        self._csv.writerow({
            "t": round(t, 2), "level": a.level, "reason": a.reason, "nav": cmd.state, "mode": cmd.mode,
            "us_left_m": _num(r.us_left), "us_right_m": _num(r.us_right),
            "ir_front_left": int(r.ir_front_left), "ir_front_right": int(r.ir_front_right),
            "floor_lost_left": int(r.floor_lost_left), "floor_lost_right": int(r.floor_lost_right),
            "pitch_deg": _num(r.pitch_deg, 1), "roll_deg": _num(r.roll_deg, 1), "yaw_deg": _num(r.yaw_deg, 1),
            "gas_ppm": _num(r.gas_ppm, 1), "gas_slope": _num(a.gas_slope, 1),
            "left_cmd": _num(cmd.left, 2), "right_cmd": _num(cmd.right, 2),
            "route_deg": _num(getattr(cmd, "route", None), 1)})
        self._csv_fh.flush()
        return True

    # ---- events -----------------------------------------------------
    def event(self, t, kind, **data):
        ev = {"t": round(t, 2), "type": kind}
        ev.update(data)
        self.events.append(ev)
        self._ev_fh.write(json.dumps(ev) + "\n")
        self._ev_fh.flush()
        return ev

    def image(self, t, filename, sweep, angle, level, reason, why):
        rec = dict(file=filename, t=round(t, 2), sweep=sweep, angle=angle, level=level, reason=reason, why=why)
        self.images.append(rec)
        self.event(t, "image", **{k: v for k, v in rec.items() if k != "t"})

    def close(self):
        for fh in (self._csv_fh, self._ev_fh):
            try:
                fh.close()
            except Exception:
                pass

    # ---- summary ----------------------------------------------------
    def write_summary(self, title="Mine rover run"):
        metrics = None
        mpath = os.path.join(self.run_dir, "metrics.json")
        if os.path.exists(mpath):
            try:
                with open(mpath) as fh:
                    metrics = json.load(fh)
            except Exception:
                metrics = None
        hazards = [e for e in self.events if e["type"] == "hazard"]
        nav = [e for e in self.events if e["type"] == "nav"]
        out = ["<!doctype html><html><head><meta charset='utf-8'>",
               f"<title>{html.escape(title)}</title>",
               "<style>body{font-family:-apple-system,Helvetica,Arial,sans-serif;margin:24px;background:#16130f;color:#e8e2d8}"
               "h1,h2{font-weight:600}table{border-collapse:collapse;margin:8px 0 20px}"
               "td,th{border:1px solid #4a4238;padding:4px 10px;text-align:left;font-size:14px}"
               "th{background:#2a241d}.lvl{font-weight:700}.imgs{display:flex;flex-wrap:wrap;gap:8px;margin:6px 0 22px}"
               ".imgs figure{margin:0}.imgs img{width:300px;border:1px solid #4a4238}"
               "figcaption{font-size:12px;color:#b5ab9c}code{color:#d8c9a8}</style></head><body>",
               f"<h1>{html.escape(title)}</h1>",
               f"<p>Run folder: <code>{html.escape(os.path.basename(self.run_dir))}</code> &middot; "
               f"{len(hazards)} hazard events &middot; {len(self.images)} sweep images</p>"]
        if metrics:
            out.append("<h2>Metrics</h2><table>")
            for k, v in metrics.items():
                if isinstance(v, (list, dict)):
                    v = json.dumps(v)
                out.append(f"<tr><th>{html.escape(str(k))}</th><td>{html.escape(str(v))}</td></tr>")
            out.append("</table>")
        out.append("<h2>Hazard events</h2>")
        if not hazards:
            out.append("<p>None - the run stayed SAFE.</p>")
        for e in hazards:
            col = LEVEL_COLOUR.get(e.get("level"), "#888")
            out.append(f"<h3><span class='lvl' style='color:{col}'>{html.escape(str(e.get('level')))}</span> "
                       f"{html.escape(str(e.get('reason')))} at t={e['t']} s</h3>")
            det = ", ".join(f"{k}={e[k]}" for k in ("response", "gas_ppm", "gas_slope", "front_m", "tilt_deg") if e.get(k) is not None)
            out.append(f"<p>{html.escape(det)}</p>")
            imgs = [i for i in self.images if i.get("why") == e.get("reason") and abs(i["t"] - e["t"]) < 40
                    and i["t"] >= e["t"] - 0.5]
            first = min((i["sweep"] for i in imgs), default=None)
            imgs = [i for i in imgs if i["sweep"] == first]
            if imgs:
                out.append("<div class='imgs'>")
                for i in imgs:
                    out.append(f"<figure><a href='sweeps/{i['file']}'><img src='sweeps/{i['file']}'></a>"
                               f"<figcaption>pan {i['angle']:+.0f}&deg; at t={i['t']} s</figcaption></figure>")
                out.append("</div>")
        if nav:
            out.append("<h2>Navigator timeline</h2><table><tr><th>t (s)</th><th>mode</th><th>state</th><th>note</th></tr>")
            for e in nav[-200:]:
                out.append(f"<tr><td>{e['t']}</td><td>{e.get('mode', '')}</td><td>{e.get('state', '')}</td>"
                           f"<td>{html.escape(str(e.get('note', '')))}</td></tr>")
            out.append("</table>")
        other = [i for i in self.images]
        if other:
            out.append("<h2>All sweep images</h2><div class='imgs'>")
            for i in other:
                out.append(f"<figure><a href='sweeps/{i['file']}'><img src='sweeps/{i['file']}'></a>"
                           f"<figcaption>sweep {i['sweep']} ({html.escape(str(i['why']))}) pan {i['angle']:+.0f}&deg; "
                           f"t={i['t']} s</figcaption></figure>")
            out.append("</div>")
        out.append("<p>Files: <code>hazard_log.csv</code>, <code>events.jsonl</code>, <code>metrics.json</code>, "
                   "<code>sweeps/</code></p></body></html>")
        path = os.path.join(self.run_dir, "summary.html")
        with open(path, "w") as fh:
            fh.write("\n".join(out))
        return path
