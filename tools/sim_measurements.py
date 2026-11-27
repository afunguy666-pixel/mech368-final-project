"""Extract handheld USB records and independently surveyed virtual ground marks."""
import argparse
from bisect import bisect_left
import csv
from math import cos, degrees, hypot, sin
from pathlib import Path

from telemetry_wire import Stream, apply_marks, read_csv, write_csv


def usb(base, run):
    stream = Stream(run, "simulation")
    rows = []
    for line in base.with_suffix(".log").read_bytes().splitlines():
        if b" remote> " in line:
            rows.extend(stream.feed(line.split(b" remote> ", 1)[1] + b"\n"))
    if not rows:
        raise ValueError(f"no valid remote USB frames in {base}.log")
    return rows


def table(path, fields, rows):
    with path.open("x", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(fields)
        writer.writerows(rows)


def bench(base, output):
    rows = [r for r in usb(base, "teach") if r["kind"] == "wheels"]
    truth = read_csv(base.with_suffix(".csv"))
    times = [float(r["t_ms"]) for r in truth]
    stages = [("left_rev", 3000, 5000), ("right_rev", 7500, 9500),
              ("roll_mm", 12000, 16000), ("turn_deg", 18500, 22500), ("turn_deg", 25000, 29000)]
    trials = []
    for kind, t0, t1 in stages:
        # Use the stationary plateaux. Counts are decoded by the actual ISR
        # and have passed through BLE, the remote, CDC and the desktop decoder.
        before = next(r for r in reversed(rows) if r["robot_ms"] < t0)
        after = next(r for r in rows if r["robot_ms"] > t1 + 10)
        a = truth[bisect_left(times, t0 - 200)]
        b = truth[bisect_left(times, t1 + 200)]
        value = 2.0  # two complete turns of the index on the individual tyre
        if kind == "roll_mm":
            value = 1000 * hypot(float(b["x_m"]) - float(a["x_m"]), float(b["y_m"]) - float(a["y_m"]))
        elif kind == "turn_deg":
            value = degrees(float(b["yaw_rad"]) - float(a["yaw_rad"]))
        trials.append((kind, round(value, 5), after["left_count"] - before["left_count"],
                       after["right_count"] - before["right_count"]))
    table(output, ("kind", "value", "left_delta", "right_delta"), trials)


# Ordered survey gates on course.world. The three x gates mark course sectors.
# Eight separate y crossings across the four lanes are held out for validation.
GATES = [("y_m", .36, 1), ("y_m", .96, 1), ("x_m", .62, 1),
         ("y_m", 1.05, -1), ("y_m", .47, -1), ("x_m", 1.035, 1),
         ("y_m", .56, 1), ("y_m", 1.09, 1), ("x_m", 1.41, 1),
         ("y_m", .80, -1), ("y_m", .42, -1)]


def lap(base, run, output):
    telemetry = usb(base, run)
    truth = read_csv(base.with_suffix(".csv"))
    x0, y0, h0 = (float(truth[0][k]) for k in ("x_m", "y_m", "yaw_rad"))
    marks = [dict(log_segment=1, robot_ms=2500, course_segment=0)]
    checks = []
    i = 0
    for axis, value, direction in GATES:
        for j in range(i + 1, len(truth)):
            a, b = truth[j - 1], truth[j]
            v0, v1 = float(a[axis]), float(b[axis])
            if (v0 - value) * direction < 0 <= (v1 - value) * direction:
                f = (value - v0) / (v1 - v0)
                ms, x, y = (float(a[k]) + f * (float(b[k]) - float(a[k]))
                            for k in ("t_ms", "x_m", "y_m"))
                if axis == "x_m":
                    marks.append(dict(log_segment=1, robot_ms=round(ms), course_segment=len(marks)))
                else:
                    dx, dy = x - x0, y - y0
                    checks.append((1, round(ms, 2), round(cos(h0)*dx + sin(h0)*dy, 6),
                                   round(-sin(h0)*dx + cos(h0)*dy, 6)))
                i = j
                break
        else:
            raise ValueError(f"course did not cross survey gate {axis}={value}")
    # Retain startup through the first stopped environment frame after finish.
    finish = next(r["robot_ms"] for r in telemetry if r["kind"] == "env" and r["nav_state"] == 18)
    telemetry = [r for r in telemetry if r["robot_ms"] <= finish]
    output.mkdir(parents=True, exist_ok=True)
    write_csv(output / f"{run}.csv", apply_marks(telemetry, marks))
    table(output / f"{run}-ground.csv", ("log_segment", "robot_ms", "x_m", "y_m"), checks)
    table(output / f"{run}-markers.csv", ("log_segment", "robot_ms", "course_segment"),
          ((m["log_segment"], m["robot_ms"], m["course_segment"]) for m in marks))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("bench")
    p.add_argument("report_base", type=Path)
    p.add_argument("output", type=Path)
    p = sub.add_parser("raw", help="all handheld USB records of one simulated run")
    p.add_argument("report_base", type=Path)
    p.add_argument("output", type=Path)
    p = sub.add_parser("lap")
    p.add_argument("report_base", type=Path)
    p.add_argument("run", choices=("teach", "live"))
    p.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.command == "bench":
        bench(args.report_base, args.output)
    elif args.command == "raw":
        write_csv(args.output, usb(args.report_base, "live"))
    else:
        lap(args.report_base, args.run, args.output)
