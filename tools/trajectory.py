"""Fixed-start SE(2) alignment and read-only teach/live comparison, in metres."""
from bisect import bisect_left
from dataclasses import dataclass
from math import atan2, cos, hypot, nan, radians, sin, sqrt

from telemetry_wire import (BIAS_READY, CALIBRATING, CLIPPED, DETOUR, LATCHED,
                            OFF_LINE, RUNNING, stationary)


def wrap(angle):
    return atan2(sin(angle), cos(angle))


@dataclass
class Point:
    ms: int
    segment: int
    x: float
    y: float
    yaw: float
    travel: float        # signed distance from fixed start
    progress: float      # signed distance from this course marker
    flags: int
    gap: int
    reversing: bool
    uncertain_marker: bool = False


class Alignment:
    """One explicitly marked origin per connection, shared by live view and replay."""
    def __init__(self):
        self.origin = None
        self.previous = None

    def add(self, row):
        seg, ms, flags = (int(row[k]) for k in ("course_segment", "robot_ms", "flags"))
        x, y, yaw, s = (int(row[k]) / 1000 for k in ("x_mm", "y_mm", "yaw_mrad", "travel_mm"))
        a = self.previous
        if a is None:
            if seg != 0 or not stationary(row):
                raise ValueError("start marker 0 requires a stationary, bias-ready pose")
            self.origin = x, y, yaw, s
        elif seg not in (a.segment, a.segment + 1) or ms <= a.ms:
            raise ValueError("course markers or pose times out of order")
        x0, y0, yaw0, s0 = self.origin
        travel = s - s0
        gap = int(row["gap"])
        if a is not None and ms - a.ms > 250:
            gap = max(1, gap)
        if a is None or seg != a.segment:
            self.start = self.high = travel
            self.uncertain = a is not None and bool(gap or flags & CLIPPED)
        self.high = max(self.high, travel)
        x, y = x - x0, y - y0
        self.previous = Point(ms, seg, cos(yaw0)*x + sin(yaw0)*y, -sin(yaw0)*x + cos(yaw0)*y,
                              wrap(yaw - yaw0), travel, travel - self.start, flags, gap,
                              travel < self.high - 0.003, self.uncertain)
        return self.previous


def poses(rows, log_segment=None):
    streams = {int(r["log_segment"]) for r in rows if r["kind"] == "pose"}
    if log_segment is None:
        if len(streams) != 1:
            raise ValueError(f"select one independently anchored log segment from {sorted(streams)}")
        log_segment = streams.pop()
    raw = [r for r in rows if r["kind"] == "pose" and int(r["log_segment"]) == log_segment
           and r["course_segment"] != ""]
    if not raw:
        raise ValueError(f"log segment {log_segment} needs a stationary course marker 0")
    alignment = Alignment()
    return [alignment.add(row) for row in raw]


def quality(p):
    if p.gap:
        return "packet gap"
    if p.flags & CLIPPED or not p.flags & BIAS_READY:
        return "invalid pose"
    if p.flags & LATCHED:
        return "E-stop"
    if p.flags & DETOUR:
        return "detour"
    if p.flags & OFF_LINE:
        return "line loss"
    if p.flags & CALIBRATING or not p.flags & RUNNING:
        return "inactive"
    if p.reversing:
        return "reverse"
    if p.uncertain_marker:
        return "uncertain marker"
    return ""


@dataclass
class Match:
    point: Point
    reason: str
    error: float = nan
    lateral: float = nan


class Reference:
    def __init__(self, points):
        self.legs = {}
        for a, b in zip(points, points[1:]):
            if a.segment != b.segment or quality(a) or quality(b):
                continue
            if not 0 < b.progress - a.progress <= 0.085 or b.ms - a.ms > 250:
                continue
            self.legs.setdefault(a.segment, []).append((a, b))

    def match(self, p):
        reason = quality(p)
        if reason:
            return Match(p, reason)
        # Never snap to a global nearest XY point. Require a unique contiguous
        # reference interval in this physical segment at this signed mileage.
        candidates = [(a, b) for a, b in self.legs.get(p.segment, [])
                      if a.progress <= p.progress < b.progress]
        if len(candidates) != 1:
            return Match(p, "no unique mileage")
        a, b = candidates[0]
        f = (p.progress - a.progress) / (b.progress - a.progress)
        x, y = a.x + f * (b.x - a.x), a.y + f * (b.y - a.y)
        yaw = wrap(a.yaw + f * wrap(b.yaw - a.yaw))
        dx, dy = p.x - x, p.y - y
        err = hypot(dx, dy)
        if abs(wrap(yaw - p.yaw)) > radians(30) or err > 0.10:
            return Match(p, "heading / position")
        for seg, leg in self.legs.items():
            if seg == p.segment:
                continue
            for c, d in leg:
                vx, vy = d.x - c.x, d.y - c.y
                length2 = vx*vx + vy*vy
                u = min(1, max(0, ((p.x-c.x)*vx + (p.y-c.y)*vy) / length2)) if length2 else 0
                distance = hypot(c.x + u*vx - p.x, c.y + u*vy - p.y)
                other_yaw = c.yaw + u*wrap(d.yaw - c.yaw)
                if distance <= err + 0.025 and abs(cos(other_yaw - p.yaw)) >= cos(radians(25)):
                    return Match(p, "parallel ambiguity")
        return Match(p, "matched", err, -sin(yaw) * dx + cos(yaw) * dy)


def ground_errors(points, truth):
    """Survey/video coordinates already in the fixed start frame. No fitted transform."""
    times = [p.ms for p in points]
    errors = []
    for row in truth:
        t = float(row["robot_ms"])
        i = bisect_left(times, t)
        if i == len(times):
            continue
        b = points[i]
        if times[i] == t:
            a, f = b, 0.0
        elif i:
            a = points[i - 1]
            if b.ms - a.ms > 250:
                continue
            f = (t - a.ms) / (b.ms - a.ms)
        else:
            continue
        if a.flags & CLIPPED or b.flags & CLIPPED or b.gap:
            continue
        x = a.x + f * (b.x - a.x)
        y = a.y + f * (b.y - a.y)
        errors.append(hypot(x - float(row["x_m"]), y - float(row["y_m"])))
    if not errors:
        raise ValueError("no independent observations overlap valid robot samples")
    return errors


def rms(values):
    return sqrt(sum(x * x for x in values) / len(values))


def plot_tracks(teach, live, matches, output, simulated=False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig, (ax, lower) = plt.subplots(2, figsize=(10, 7.5), height_ratios=[3, 1.1],
                                    layout="constrained")
    for name, track, color, style, width in (("Teach", teach, "#244d73", "solid", 2.7),
                                              ("Live", live, "#d56b24", "dashed", 1.7)):
        segments = [[(a.x, a.y), (b.x, b.y)] for a, b in zip(track, track[1:])
                    if not b.gap and not (a.flags | b.flags) & CLIPPED]
        ax.add_collection(LineCollection(segments, colors=color, linestyles=style,
                                         linewidths=width, label=name))
    layers = (("Detour", lambda m: m.point.flags & DETOUR, ".", "#7c55ac"),
              ("Line loss", lambda m: m.point.flags & OFF_LINE, "+", "#b73d52"),
              ("Packet gap", lambda m: m.point.gap, "X", "#111111"),
              ("Unreliable match", lambda m: m.reason not in ("matched", "detour", "line loss",
                                                               "inactive", "packet gap"), "x", "#828282"))
    for label, predicate, marker, color in layers:
        ps = [m.point for m in matches if predicate(m) and not m.point.flags & CLIPPED]
        if ps:
            ax.scatter([p.x for p in ps], [p.y for p in ps], s=62 if label == "Packet gap" else 18,
                       marker=marker, color=color, label=label, zorder=6 if label == "Packet gap" else 4,
                       alpha=0.85)
            if label == "Packet gap" and len(ps) == 1:
                ax.annotate(f"{ps[0].gap} missing pose frames", (ps[0].x, ps[0].y),
                            xytext=(12, -19), textcoords="offset points", fontsize=8,
                            arrowprops=dict(arrowstyle="-", color=color, lw=.7))
    anchors = [live[0]] + [b for a, b in zip(live, live[1:]) if a.segment != b.segment]
    for p in anchors:
        ax.scatter(p.x, p.y, s=28, facecolor="white", edgecolor="#243b39", zorder=5)
        ax.annotate(f"S{p.segment}", (p.x, p.y), xytext=(5, 5), textcoords="offset points")
    ax.autoscale()
    ax.set_aspect("equal", adjustable="datalim")
    ax.set(xlabel="Forward from start (m)", ylabel="Left of start (m)")
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1), frameon=False)
    lower.plot([m.point.travel for m in matches], [m.error * 1000 for m in matches],
               marker=".", ms=2.5, lw=1, color="#17665b")
    for p in anchors[1:]:
        lower.axvline(p.travel, color="#adb7bd", lw=0.75, ls=":")
    lower.set(xlabel="Live signed travel from start (m)", ylabel="Matched XY error (mm)")
    for axis in (ax, lower):
        axis.grid(alpha=0.2)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle(("Firmware simulation" if simulated else "Recorded runs") + "  |  Teach and live",
                 fontsize=18, fontweight="bold")
    fig.savefig(output, dpi=155)
    plt.close(fig)
