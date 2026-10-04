"""Draw the reward landscape the robot actually found, from trial logs.

The simulator could colour every point on a grid because it had a model to
score them with. The robot has only the poses it tried, so this is a scatter
of trials, one dot each, coloured by outcome and reward. A candidate has four
numbers, so there are two views: knee against ankle, as in the simulator, and
hip against speed. Hover over a dot for the trial behind it.

    python landscape.py trials.jsonl
    python landscape.py wood.jsonl tile.jsonl -o surfaces.html

Standard library only, so it runs on the Pi; open the HTML in any browser.
"""

import argparse
import html
import json
import sys

KINDS = ["stood", "unsupported", "tipped", "impeded", "restricted", "infeasible"]
KIND_LABELS = {"stood": "stood (deeper = higher reward)", "unsupported": "did not stand",
               "tipped": "tipped", "impeded": "impeded", "restricted": "restricted",
               "infeasible": "skipped, never moved"}
STOOD_STEPS = 5

# (name, index into the candidate, lowest, highest, unit) -- the search box
AXES = {"hip": (0, -20, 20, "°"), "knee": (1, -60, 45, "°"),
        "ankle": (2, 0, 140, "°"), "speed": (3, 0.1, 1.0, "")}
VIEWS = [("knee", "ankle"), ("hip", "speed")]

W, H = 330, 270
LEFT, BOTTOM, TOP, RIGHT = 46, 34, 10, 10

STYLE = """
:root { --bg:#ffffff; --ink:#0b0b0b; --muted:#898781; --grid:#e1e0d9;
  --s0:#B5D4F4; --s1:#85B7EB; --s2:#378ADD; --s3:#185FA5; --s4:#0C447C;
  --unsupported:#F0997B; --tipped:#E24B4A; --impeded:#7F77DD; --skipped:#B4B2A9; }
@media (prefers-color-scheme: dark) { :root { --bg:#1a1a19; --ink:#f0efec;
  --muted:#898781; --grid:#2c2c2a; --s0:#0C447C; --s1:#185FA5; --s2:#378ADD;
  --s3:#85B7EB; --s4:#B5D4F4; --unsupported:#D85A30; --tipped:#E24B4A;
  --impeded:#9085e9; --skipped:#5F5E5A; } }
body { background:var(--bg); color:var(--ink); font-family:system-ui,sans-serif;
  margin:16px; max-width:720px; }
h1 { font-size:18px; font-weight:500; margin:0 0 4px; }
h2 { font-size:15px; font-weight:500; margin:22px 0 4px; }
p, li { font-size:13px; color:var(--muted); margin:2px 0; }
.legend { display:flex; flex-wrap:wrap; gap:12px; font-size:12px; margin:8px 0; }
.legend span::before { content:""; display:inline-block; width:10px; height:10px;
  border-radius:50%; margin-right:5px; vertical-align:-1px; background:var(--c); }
.views { display:flex; flex-wrap:wrap; gap:16px; }
svg text { fill:var(--muted); font-size:11px; }
svg .grid { stroke:var(--grid); }
"""


def kind(record):
    for key in ("infeasible", "restricted", "impeded", "tipped", "unsupported"):
        if key in record:
            return key
    return "stood"


def colour(record, low, high):
    k = kind(record)
    if k == "stood":
        step = 0 if high <= low else int((record["reward"] - low) / (high - low) * STOOD_STEPS)
        return "var(--s%d)" % min(STOOD_STEPS - 1, step)
    return {"unsupported": "var(--unsupported)", "tipped": "var(--tipped)",
            "impeded": "var(--impeded)"}.get(k, "var(--skipped)")


def describe(record):
    c = record["candidate"] + [None] * (4 - len(record["candidate"]))
    seed = record.get("trial_seed")
    lines = ["trial %d (%s), reward %.1f" % (record["trial"], record["label"], record["reward"])
             + (", trial seed %d" % seed if seed is not None else ""),
             "hip %.1f, knee %.1f, ankle %.1f" % tuple(c[:3])
             + (", speed %.2f" % c[3] if c[3] is not None else "")]
    k = kind(record)
    if k == "stood":
        line = "stood in %.2f s" % record.get("stand_s", 0)
        if "height_mm" in record:
            line += " at %.0f mm" % record["height_mm"]
        lines.append(line + ", level %.1f°" % record.get("level_deg", 0))
    else:
        lines.append("%s: %s" % (k, record.get(k, "")))
    return "\n".join(lines)


def scatter(records, x_name, y_name, low, high, best):
    xi, x0, x1, xu = AXES[x_name]
    yi, y0, y1, yu = AXES[y_name]
    pw, ph = W - LEFT - RIGHT, H - TOP - BOTTOM
    sx = lambda v: LEFT + (v - x0) / (x1 - x0) * pw
    sy = lambda v: TOP + ph - (v - y0) / (y1 - y0) * ph
    parts = ['<svg width="%d" height="%d" role="img" aria-label="%s against %s">'
             % (W, H, y_name, x_name)]
    for t in range(5):
        xv = x0 + (x1 - x0) * t / 4
        yv = y0 + (y1 - y0) * t / 4
        parts.append('<line class="grid" x1="%.1f" y1="%d" x2="%.1f" y2="%d"/>'
                     % (sx(xv), TOP, sx(xv), TOP + ph))
        parts.append('<line class="grid" x1="%d" y1="%.1f" x2="%d" y2="%.1f"/>'
                     % (LEFT, sy(yv), LEFT + pw, sy(yv)))
        fmt = "%.1f" if x_name == "speed" else "%.0f"
        parts.append('<text x="%.1f" y="%d" text-anchor="middle">%s%s</text>'
                     % (sx(xv), H - BOTTOM + 14, fmt % xv, xu))
        fmt = "%.1f" if y_name == "speed" else "%.0f"
        parts.append('<text x="%d" y="%.1f" text-anchor="end">%s%s</text>'
                     % (LEFT - 6, sy(yv) + 4, fmt % yv, yu))
    parts.append('<text x="%.1f" y="%d" text-anchor="middle">%s</text>'
                 % (LEFT + pw / 2, H - 4, x_name))
    parts.append('<text transform="translate(12,%.1f) rotate(-90)" text-anchor="middle">%s</text>'
                 % (TOP + ph / 2, y_name))
    # Never-moved trials first and small, so the ones that ran sit on top
    for record in sorted(records, key=lambda r: KINDS.index(kind(r)), reverse=True):
        c = record["candidate"]
        if len(c) <= max(xi, yi):
            continue
        moved = kind(record) not in ("infeasible", "restricted")
        parts.append('<circle cx="%.1f" cy="%.1f" r="%d" fill="%s"%s><title>%s</title></circle>'
                     % (sx(c[xi]), sy(c[yi]), 5 if moved else 3, colour(record, low, high),
                        "" if moved else ' fill-opacity="0.7"', html.escape(describe(record))))
    if best is not None and len(best["candidate"]) > max(xi, yi):
        parts.append('<circle cx="%.1f" cy="%.1f" r="9" fill="none" stroke="#1baf7a" '
                     'stroke-width="2"><title>best: %s</title></circle>'
                     % (sx(best["candidate"][xi]), sy(best["candidate"][yi]),
                        html.escape(describe(best))))
    parts.append("</svg>")
    return "".join(parts)


def section(path, records):
    stood = [r for r in records if kind(r) == "stood"]
    low = min((r["reward"] for r in stood), default=0)
    high = max((r["reward"] for r in stood), default=1)
    best = max((r for r in stood if r["label"] == "search"), key=lambda r: r["reward"],
               default=None)
    counts = {k: sum(1 for r in records if kind(r) == k) for k in KINDS}
    first = records[0] if records else {}
    out = ["<h2>%s</h2>" % html.escape(path),
           "<p>%d trials, surface %s, %s, seed %s</p>"
           % (len(records), html.escape(str(first.get("surface", "?"))),
              "with the model" if first.get("use_model") else "measured only",
              first.get("seed", "?")),
           "<p>%s</p>" % ", ".join("%d %s" % (counts[k], k) for k in KINDS if counts[k])]
    if best is not None:
        out.append("<p>best: %s</p>" % html.escape(describe(best)).replace("\n", " · "))
    if stood:
        out.append("<p>stood rewards from %.1f to %.1f</p>" % (low, high))
    out.append('<div class="views">%s</div>'
               % "".join(scatter(records, x, y, low, high, best) for x, y in VIEWS))
    return "".join(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", help="trials.jsonl files")
    parser.add_argument("-o", "--out", default="landscape.html")
    parser.add_argument("--all", action="store_true",
                        help="include the noise-check repeats, not just search trials")
    options = parser.parse_args()

    sections = []
    for path in options.logs:
        with open(path) as handle:
            records = [json.loads(line) for line in handle if line.strip()]
        if not options.all:
            records = [r for r in records if r.get("label") == "search"]
        sections.append(section(path, records))

    legend = "".join('<span style="--c:%s">%s</span>'
                     % ({"stood": "var(--s3)", "unsupported": "var(--unsupported)",
                         "tipped": "var(--tipped)", "impeded": "var(--impeded)"}
                        .get(k, "var(--skipped)"), KIND_LABELS[k])
                     for k in KINDS)
    page = ("<!doctype html><html><head><meta charset=\"utf-8\">"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
            "<title>Standing landscape</title><style>%s</style></head><body>"
            "<h1>Standing landscape</h1>"
            "<p>One dot per trial. Small dots never moved. The green ring is the best "
            "search trial. Hover over a dot for its details.</p>"
            "<div class=\"legend\">%s</div>%s</body></html>") % (STYLE, legend, "".join(sections))
    with open(options.out, "w", encoding="utf-8") as handle:
        handle.write(page)
    print("wrote %s" % options.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
