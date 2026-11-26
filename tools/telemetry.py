"""MECH 368 receive-only recorder, replay and track comparison."""
import argparse
from collections import Counter, deque
import csv
from math import nan
from pathlib import Path
import sys
import time

from calibrate_drive import estimate
from telemetry_wire import CLIPPED, FIELDS, Stream, apply_marks, read_csv, write_csv
from trajectory import Alignment, Reference, ground_errors, plot_tracks, poses, quality, rms


class View:
    def __init__(self, title, reference=(), on_key=None):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.fig, self.ax = plt.subplots(figsize=(7, 5))
        self.title = title
        self.reference = Reference(reference) if reference else None
        self.ref_line = None
        self.ax.set(xlabel="Forward (m)", ylabel="Left (m)", title=title, aspect="equal")
        if reference:
            rx, ry = [], []
            for p in reference:
                if p.gap or p.flags & CLIPPED:
                    rx.append(nan)
                    ry.append(nan)
                if not p.flags & CLIPPED:
                    rx.append(p.x)
                    ry.append(p.y)
            self.ref_line, = self.ax.plot(rx, ry, color="#244d73", label="Teach", visible=False)
        self.line, = self.ax.plot([], [], color="#cb6325", label="Received XY")
        self.events, = self.ax.plot([], [], "x", color="#9c3854", ms=4, label="Flagged sample")
        self.ax.legend()
        self.ax.grid(alpha=.25)
        self.x, self.y = deque(maxlen=12000), deque(maxlen=12000)
        self.ex, self.ey = deque(maxlen=12000), deque(maxlen=12000)
        self.segment = None
        self.alignment = Alignment()
        if on_key:
            self.fig.canvas.mpl_connect("key_press_event", lambda event: on_key(event.key))
        self.plt.show(block=False)

    def add(self, row):
        if row["kind"] != "pose":
            return
        seg = int(row["log_segment"])
        if self.segment != seg:
            self.segment = seg
            self.alignment = Alignment()
            self.x.clear()
            self.y.clear()
            self.ex.clear()
            self.ey.clear()
            if self.ref_line is not None:
                self.ref_line.set_visible(False)
        if int(row["flags"]) & CLIPPED:
            self.x.append(nan)
            self.y.append(nan)
            return
        x, y = (int(row[k]) / 1000 for k in ("x_mm", "y_mm"))
        status = "unanchored"
        if row["course_segment"] != "":
            if self.alignment.origin is None:
                self.x.clear()
                self.y.clear()
                if self.ref_line is not None:
                    self.ref_line.set_visible(True)
            p = self.alignment.add(row)
            x, y = p.x, p.y
            reason = self.reference.match(p).reason if self.reference else quality(p)
            if reason and reason not in ("matched", "inactive"):
                self.ex.append(x)
                self.ey.append(y)
            status = f"S{p.segment} {reason}"
        if int(row["gap"]):
            self.x.append(nan)
            self.y.append(nan)
        self.x.append(x)
        self.y.append(y)
        self.ax.set_title(f"{self.title}\nLog {seg}   |   {status}   |   {row['robot_ms']} ms")

    def refresh(self):
        if not self.plt.fignum_exists(self.fig.number):
            return False
        self.line.set_data(self.x, self.y)
        self.events.set_data(self.ex, self.ey)
        self.ax.relim(visible_only=True)
        self.ax.autoscale_view()
        self.fig.canvas.draw_idle()
        self.plt.pause(.001)
        return True


class Sink:
    def __init__(self, file):
        self.file = file
        self.writer = csv.DictWriter(file, FIELDS)
        self.writer.writeheader()
        self.file.flush()
        self.failed = False

    def append(self, row):
        if self.failed:
            return
        try:
            self.writer.writerow(row)
            self.file.flush()
        except OSError as exc:
            self.failed = True
            print(f"CSV stopped ({exc}). Still draining telemetry.", file=sys.stderr)


def record(args):
    import serial
    stream = Stream(args.run, "hardware")
    ref = []
    if args.reference:
        rows = read_csv(args.reference)
        if args.run != "live" or {(r["source"], r["run"]) for r in rows} != {("hardware", "teach")}:
            raise ValueError("a live COM overlay requires a teach CSV from the same physical course")
        ref = poses(rows, args.teach_log)
    title = f"{args.run}  |  S at fixed start, M at next marker"
    view = None if args.headless else View(title, ref, stream.mark)
    import os
    keyboard = None
    if os.name == "nt" and sys.stdin.isatty():
        import msvcrt as keyboard
    port = None
    opened = offline = False
    next_open = next_draw = 0
    last_pose = None
    print("S marks fixed start, M marks the next course segment. Ctrl+C ends the log.")
    with open(args.output, "x", encoding="utf-8", newline="") as file:
        sink = Sink(file)
        try:
            while True:
                now = time.monotonic()
                if keyboard and keyboard.kbhit():
                    stream.mark(keyboard.getwch().lower())
                if view and now >= next_draw:
                    if last_pose is None or now - last_pose > .5:
                        view.ax.set_title(f"{title}\nWaiting for a fresh robot pose")
                    if not view.refresh():
                        break
                    next_draw = now + .12
                if port is None and now >= next_open:
                    try:
                        port = serial.serial_for_url(args.port, baudrate=115200, timeout=.025)
                    except serial.SerialException:
                        if not offline:
                            print(f"Waiting for {args.port}", file=sys.stderr)
                        offline = True
                        next_open = now + 1
                    else:
                        stream.reopen()
                        opened, offline = True, False
                        print(f"Listening on {args.port}")
                if port is None:
                    time.sleep(.03)
                    continue
                try:
                    chunk = port.read(min(4096, max(1, port.in_waiting)))
                except (serial.SerialException, OSError):
                    port.close()
                    port = None
                    last_pose = None
                    continue
                for row in stream.feed(chunk):
                    sink.append(row)
                    if row["kind"] == "pose":
                        last_pose = time.monotonic()
                    if view:
                        view.add(row)
        except KeyboardInterrupt:
            pass
        finally:
            if port is not None:
                port.close()
        if sink.failed or not opened:
            raise SystemExit(1)


def replay(args):
    rows = read_csv(args.input)
    counts = Counter(r["kind"] for r in rows)
    print(f"{len(rows)} records, {dict(counts)}, {sum(int(r['gap']) for r in rows)} missing frames")
    if args.summary:
        return
    if args.speed <= 0:
        raise ValueError("speed must be positive")
    view = View("Replay")
    last_seg, start = None, 0
    for row in rows:
        if row["kind"] != "pose":
            continue
        t = int(row["robot_ms"]) / 1000 / args.speed
        if row["log_segment"] != last_seg:
            last_seg, start = row["log_segment"], time.monotonic() - t
        while time.monotonic() < start + t:
            if not view.refresh():
                return
            time.sleep(min(.02, max(0, start + t - time.monotonic())))
        view.add(row)
    if view.refresh():
        view.plt.show(block=True)


def compare(args):
    teach_rows, live_rows = read_csv(args.teach), read_csv(args.live)
    if {r["run"] for r in teach_rows} != {"teach"} or {r["run"] for r in live_rows} != {"live"}:
        raise ValueError("comparison needs one teach run and one live run")
    sources = {r["source"] for r in teach_rows + live_rows}
    if len(sources) != 1:
        raise ValueError("teach and live must use the same observation source")
    teach, live = poses(teach_rows, args.teach_log), poses(live_rows, args.live_log)
    ref = Reference(teach)
    matches = [ref.match(p) for p in live]
    errors = [m.error for m in matches if m.reason == "matched"]
    print(dict(Counter(m.reason for m in matches)))
    if errors:
        print(f"Matched teach/live XY RMS {1000 * rms(errors):.1f} mm ({len(errors)} poses)")
    if args.matches:
        with open(args.matches, "x", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(("robot_ms", "course_segment", "progress_m", "result", "xy_error_m", "lateral_m"))
            for m in matches:
                p = m.point
                writer.writerow((p.ms, p.segment, round(p.progress, 4), m.reason,
                                 round(m.error, 4) if m.reason == "matched" else "",
                                 round(m.lateral, 4) if m.reason == "matched" else ""))
    plot_tracks(teach, live, matches, args.plot, sources == {"simulation"})


def verify(args):
    rows = read_csv(args.input)
    points = poses(rows, args.log_segment)
    seg = args.log_segment if args.log_segment is not None else int(rows[0]["log_segment"])
    truth = [r for r in read_csv(args.truth) if int(r["log_segment"]) == seg]
    errors = ground_errors(points, truth)
    print(f"{len(errors)}/{len(truth)} independent observations, XY RMS {1000*rms(errors):.1f} mm, "
          f"maximum {1000*max(errors):.1f} mm")


def calibrate(args):
    cl, cr, ll, lr, track = estimate(read_csv(args.input))
    for name, cp, roll in (("left", cl, ll), ("right", cr, lr)):
        print(f"{name}: sign {1 if cp > 0 else -1:+d}, {abs(cp):.3f} decoded counts/rev, "
              f"loaded rollout {roll:.3f} mm/rev")
    print(f"effective track {track:.3f} mm")
    if args.header:
        with open(args.header, "x", encoding="ascii") as file:
            file.write("// Measured drive profile, metres and decoded counts per wheel revolution.\n")
            for name, cp, roll in (("left", cl, ll), ("right", cr, lr)):
                file.write(f"constexpr Wheel {name} = {{{1 if cp > 0 else -1}, {abs(cp):.6f}f, "
                           f"{roll/1000:.8f}f}};\n")
            file.write(f"constexpr float trackMetres = {track/1000:.8f}f;\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    rec = sub.add_parser("record")
    rec.add_argument("--port", required=True)
    rec.add_argument("--run", choices=("teach", "live"), required=True)
    rec.add_argument("--output", required=True, type=Path)
    rec.add_argument("--reference", type=Path)
    rec.add_argument("--teach-log", type=int)
    rec.add_argument("--headless", action="store_true")
    rep = sub.add_parser("replay")
    rep.add_argument("input")
    rep.add_argument("--speed", type=float, default=1)
    rep.add_argument("--summary", action="store_true")
    comp = sub.add_parser("compare")
    comp.add_argument("teach")
    comp.add_argument("live")
    comp.add_argument("--teach-log", type=int)
    comp.add_argument("--live-log", type=int)
    comp.add_argument("--plot", required=True)
    comp.add_argument("--matches")
    truth = sub.add_parser("verify")
    truth.add_argument("input")
    truth.add_argument("truth")
    truth.add_argument("--log-segment", type=int)
    cal = sub.add_parser("calibrate")
    cal.add_argument("input")
    cal.add_argument("--header", help="write an explicit build profile")
    mark = sub.add_parser("mark")
    mark.add_argument("input")
    mark.add_argument("markers")
    mark.add_argument("output")
    args = parser.parse_args()
    if args.command == "mark":
        write_csv(args.output, apply_marks(read_csv(args.input), read_csv(args.markers)))
    else:
        {"record": record, "replay": replay, "compare": compare,
         "verify": verify, "calibrate": calibrate}[args.command](args)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError) as exc:
        raise SystemExit(str(exc))
