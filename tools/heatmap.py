"""Temperature samples paired to robot poses on the robot's own clock.

Each environment frame carries the robot time at which the HS3003 reading
was collected (robot_ms) and how long before that the conversion was
requested (temp_age_ms, 10 ms resolution). The air sample is taken from
the request time, robot_ms - temp_age_ms. The pose at that instant is
linearly interpolated between the two pose frames that bracket it in the
same log_segment. PC receive time is never used.

A sample is dropped (with a reason) when it has no valid temperature or
conversion time, when no pose brackets it inside its log segment (no
interpolation across a reconnect), when the bracketing poses are further
apart than max_gap_ms or frames were lost between them, or when either
pose is out of range or taken before the gyro bias was fixed.

The sensor's thermal response time is NOT compensated: the reading lags
the air the robot drove through. Do not read that lag as a measured
property of the field.
"""
from bisect import bisect_left
from collections import Counter
from dataclasses import dataclass
from math import hypot, isfinite, nan

from telemetry_wire import BIAS_READY, CLIPPED


@dataclass
class Pose:
    ms: int
    x: float
    y: float
    flags: int
    gap: int


@dataclass
class Pair:
    segment: int
    env_ms: int
    sample_ms: float
    temp: float
    x: float = nan
    y: float = nan
    result: str = "paired"


def pose_tracks(rows):
    """Pose frames per log segment, sorted by robot time."""
    tracks = {}
    for r in rows:
        if r["kind"] != "pose":
            continue
        tracks.setdefault(int(r["log_segment"]), []).append(
            Pose(int(r["robot_ms"]), int(r["x_mm"]) / 1000, int(r["y_mm"]) / 1000,
                 int(r["flags"]), int(r["gap"] or 0)))
    for track in tracks.values():
        track.sort(key=lambda p: p.ms)
    return tracks


def usable(p):
    return not p.flags & CLIPPED and p.flags & BIAS_READY


def pair_temperatures(rows, max_gap_ms=250):
    tracks = pose_tracks(rows)
    pairs = []
    for r in rows:
        if r["kind"] != "env":
            continue
        seg, ms = int(r["log_segment"]), int(r["robot_ms"])
        if r["temp_c"] in ("", None):
            pairs.append(Pair(seg, ms, nan, nan, result="no valid temperature"))
            continue
        temp = float(r["temp_c"])
        if r["temp_age_ms"] in ("", None):
            pairs.append(Pair(seg, ms, nan, temp, result="no conversion time"))
            continue
        t = ms - int(r["temp_age_ms"])
        pair = Pair(seg, ms, t, temp)
        pairs.append(pair)
        track = tracks.get(seg, [])
        times = [p.ms for p in track]
        i = bisect_left(times, t)
        if not track:
            pair.result = "no pose in log segment"
            continue
        if i < len(track) and track[i].ms == t:
            a = b = track[i]
        elif i == 0:
            pair.result = "before first pose of segment"
            continue
        elif i == len(track):
            pair.result = "after last pose of segment"
            continue
        else:
            a, b = track[i - 1], track[i]
            if b.ms - a.ms > max_gap_ms or b.gap:
                pair.result = "pose gap"
                continue
        if not (usable(a) and usable(b)):
            pair.result = "invalid pose / gyro bias not ready"
            continue
        f = 0.0 if b.ms == a.ms else (t - a.ms) / (b.ms - a.ms)
        pair.x = a.x + f * (b.x - a.x)
        pair.y = a.y + f * (b.y - a.y)
    return pairs


def check_one_frame(rows, segments):
    """Segments can share a map only if the robot did not restart between them.

    A robot restart resets millis(), so a later segment then starts earlier
    than the previous one ended. Raise instead of silently mixing frames.
    """
    spans = []
    for seg in segments:
        ms = [int(r["robot_ms"]) for r in rows if int(r["log_segment"]) == seg]
        spans.append((min(ms), max(ms), seg))
    for (_, end, a), (start, _, b) in zip(spans, spans[1:]):
        if start <= end:
            raise ValueError(f"log segment {b} starts before segment {a} ends on the robot clock "
                             "(robot restarted?): select one with --log-segment")


def select(rows, log_segment=None):
    segments = sorted({int(r["log_segment"]) for r in rows})
    if log_segment is not None:
        if log_segment not in segments:
            raise ValueError(f"log segment {log_segment} not in {segments}")
        segments = [log_segment]
    else:
        check_one_frame(rows, segments)
    return [r for r in rows if int(r["log_segment"]) in segments], segments


def transport_losses(rows):
    """Environment frames that never reached the CSV (sequence gaps)."""
    return sum(int(r["gap"] or 0) for r in rows if r["kind"] == "env")


def summary(pairs):
    counts = Counter(p.result for p in pairs)
    paired = counts.pop("paired", 0)
    return paired, counts


def idw_grid(samples, extent, cell, radius, power=2):
    """Inverse-distance estimate per cell centre from samples within radius.

    Cells with no sample within radius stay NaN (unknown).
    """
    x0, x1, y0, y1 = extent
    nx = max(1, round((x1 - x0) / cell))
    ny = max(1, round((y1 - y0) / cell))
    grid = [[nan] * nx for _ in range(ny)]
    for j in range(ny):
        cy = y0 + (j + 0.5) * cell
        for i in range(nx):
            cx = x0 + (i + 0.5) * cell
            num = den = 0.0
            for sx, sy, st in samples:
                d = hypot(cx - sx, cy - sy)
                if d > radius:
                    continue
                if d < 1e-6:
                    num, den = st, 1.0
                    break
                w = d ** -power
                num += w * st
                den += w
            if den:
                grid[j][i] = num / den
    return grid, nx, ny


def plot(rows, pairs, segments, output, cell=0.05, radius=0.12, max_gap_ms=250,
         interpolate=True, title=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch, Rectangle

    good = [p for p in pairs if p.result == "paired"]
    if not good:
        raise ValueError("no temperature sample could be paired with a pose")
    tracks = pose_tracks(rows)
    xs, ys = [], []
    for seg in segments:
        prev = None
        for p in tracks.get(seg, []):
            if not usable(p):
                prev = None
                continue
            if prev is None or p.gap or p.ms - prev.ms > max_gap_ms:
                xs.append(nan)
                ys.append(nan)
            xs.append(p.x)
            ys.append(p.y)
            prev = p
        xs.append(nan)
        ys.append(nan)
    finite = [(x, y) for x, y in zip(xs, ys) if isfinite(x)] + [(p.x, p.y) for p in good]
    margin = max(radius, 0.1)
    extent = (min(x for x, _ in finite) - margin, max(x for x, _ in finite) + margin,
              min(y for _, y in finite) - margin, max(y for _, y in finite) + margin)

    temps = sorted(p.temp for p in good)
    centre = temps[len(temps) // 2]
    span = max(0.5, centre - temps[0], temps[-1] - centre)
    norm = TwoSlopeNorm(vcenter=centre, vmin=centre - span, vmax=centre + span)
    cmap = LinearSegmentedColormap.from_list(
        "cool-neutral-warm", ["#1b4f8f", "#257de0", "#efefef", "#ec4647", "#9e2223"])

    fig, ax = plt.subplots(figsize=(9, 7), layout="constrained")
    ink, grid_ink = "#2b2b2b", "#d0d0d0"
    x0, x1, y0, y1 = extent
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor="#f6f6f6", edgecolor="#c9c9c9",
                           hatch="////", linewidth=0, zorder=0))
    handles = []
    if interpolate:
        samples = [(p.x, p.y, p.temp) for p in good]
        grid, nx, ny = idw_grid(samples, extent, cell, radius)
        import numpy as np
        data = np.array(grid)
        # Opaque, lightened cells: interpolation reads as paler than a
        # measured dot, and the unknown hatch never shows through it.
        rgba = cmap(norm(np.nan_to_num(data, nan=centre)))
        rgba[..., :3] = 0.45 * rgba[..., :3] + 0.55
        rgba[..., 3] = np.where(np.isnan(data), 0.0, 1.0)
        ax.imshow(rgba, extent=(x0, x0 + nx * cell, y0, y0 + ny * cell), origin="lower",
                  interpolation="nearest", zorder=1)
        handles.append(Patch(facecolor="#f5c3c3", edgecolor="none",
                             label=f"Interpolated (IDW, within {radius*100:.0f} cm of a sample)"))
    handles.append(Patch(facecolor="#f6f6f6", hatch="////", edgecolor="#c9c9c9",
                         label="Unknown (no sample nearby)"))
    ax.plot(xs, ys, color="#6b6b6b", lw=1.2, zorder=3)
    handles.append(Line2D([], [], color="#6b6b6b", lw=1.2, label="Trajectory (breaks at gaps / reconnects)"))
    sc = ax.scatter([p.x for p in good], [p.y for p in good], c=[p.temp for p in good], cmap=cmap,
                    norm=norm, s=46, edgecolors="white", linewidths=1.2, zorder=5)
    ax.scatter([p.x for p in good], [p.y for p in good], s=58, facecolors="none",
               edgecolors=ink, linewidths=0.6, zorder=6)
    handles.append(Line2D([], [], marker="o", ls="", markerfacecolor="#efefef", markeredgecolor=ink,
                          markersize=7, label=f"Measured sample ({len(good)})"))
    bar = fig.colorbar(sc, ax=ax, shrink=0.8, pad=0.02)
    bar.set_label("Temperature (°C)", color=ink)
    bar.ax.axhline(centre, color=ink, lw=0.6)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set(xlabel="Robot odometry x, forward at power-up (m)", ylabel="Robot odometry y, left (m)")
    ax.grid(color=grid_ink, lw=0.5, alpha=0.6)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    ax.legend(handles=handles, loc="upper left", bbox_to_anchor=(0, -0.09), ncol=2, frameon=False,
              fontsize=8.5)
    paired, dropped = summary(pairs)
    sources = {r["source"] for r in rows}
    head = title or ("Temperature map  |  " + ("SIMULATION" if sources == {"simulation"} else "recorded run"))
    detail = (f"log segments {', '.join(map(str, segments))}  ·  {paired} paired  ·  "
              f"{sum(dropped.values())} dropped  ·  {transport_losses(rows)} lost in transport\n"
              f"Neutral grey = median {centre:.2f} °C  ·  pose at the HS3003 request time on the robot "
              "clock  ·  sensor lag not compensated")
    ax.set_title(f"{head}\n{detail}", fontsize=11, color=ink, loc="left")
    fig.savefig(output, dpi=150)
    plt.close(fig)
