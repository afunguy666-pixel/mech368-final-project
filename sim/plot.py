#!/usr/bin/env python3
"""Render optional plots from the MECH 368 SIL scenario CSVs."""
import argparse
import csv
from math import cos, hypot, sin
from pathlib import Path
import sys


def read_csv(directory, name):
    with (directory / f"{name}.csv").open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def numbers(rows, key):
    return [float(row[key]) for row in rows]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", nargs="?", type=Path, default=Path("build/sil"))
    args = parser.parse_args()
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        parser.exit(2, "Plotting is optional; install matplotlib to generate figures.\n")

    cases = [
        ("straight", "01  Straight"),
        ("turn90", "02  Straight + 90° turn"),
        ("sensor_error", "03  Sensor-error scenario"),
    ]
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.3), layout="constrained")
    fig.suptitle("MECH 368  |  Tracking comparison", x=0.02, ha="left", fontweight="bold")
    for ax, (case, title) in zip(axes, cases):
        rows = read_csv(args.directory, case)
        for prefix, label, color, style in [
            ("true", "Ground truth", "#243048", "-"),
            ("enc", "Encoder", "#CE673E", "--"),
            ("fused", "Encoder + gyro", "#00798C", ":"),
        ]:
            x, y = numbers(rows, f"{prefix}_x_m"), numbers(rows, f"{prefix}_y_m")
            ax.plot(x, y, label=label, color=color, linestyle=style, linewidth=2)
            ax.scatter(x[-1], y[-1], color=color, s=22, zorder=3)
        if case == "turn90":
            tip = rows[-1]
            x, y, yaw = (float(tip[k]) for k in ("true_x_m", "true_y_m", "true_yaw_rad"))
            ax.annotate("", xy=(x + 0.045*cos(yaw), y + 0.045*sin(yaw)), xytext=(x, y),
                        arrowprops={"arrowstyle": "-|>", "color": "#243048", "linewidth": 1.5})
        ax.set(title=title, xlabel="x (m)", ylabel="y (m)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.22)
        tip = rows[-1]
        errors = [100 * hypot(float(tip[f"{method}_x_m"]) - float(tip["true_x_m"]),
                              float(tip[f"{method}_y_m"]) - float(tip["true_y_m"]))
                  for method in ("enc", "fused")]
        if case == "sensor_error":
            ax.text(0.03, 0.04, f"Endpoint error: {errors[0]:.1f} vs {errors[1]:.1f} cm",
                    transform=ax.transAxes, fontsize=8, color="#4F6072")
    axes[-1].legend(loc="best", fontsize=8)
    fig.savefig(args.directory / "tracking.png", dpi=180)
    plt.close(fig)

    rows = read_csv(args.directory, "watchdog")
    t = [v / 1000 for v in numbers(rows, "t_ms")]
    fig, ax = plt.subplots(figsize=(9.5, 4.5), layout="constrained")
    ax.set_title("MECH 368  |  Connected but silent: watchdog test", loc="left", fontweight="bold")
    ax.step(t, numbers(rows, "left_cmd_rad_s"), where="post", label="Left wheel", color="#243048", linewidth=2)
    ax.step(t, numbers(rows, "right_cmd_rad_s"), where="post", label="Right wheel", color="#00798C", linestyle="--", linewidth=2)
    stop = next((i for i, row in enumerate(rows) if "watchdog_timeout" in row["event"]), None)
    if stop is not None:
        last_accepted = next((i for i in range(stop - 1, -1, -1) if rows[i]["accepted"] == "1"), None)
        if last_accepted is not None:
            ax.axvspan(t[last_accepted], t[stop], color="#CE673E", alpha=0.13,
                       label=f"{(t[stop]-t[last_accepted])*1000:.0f} ms software watchdog")
            ax.axvline(t[stop], color="#CE673E", linestyle=":", linewidth=2)
    ax.set(xlabel="Virtual time (s)", ylabel="Wheel command (rad/s)")
    ax.grid(alpha=0.22)
    ax.legend(loc="best")
    ax.text(0.99, 0.09, "Simulated commands; not a physical stop measurement", transform=ax.transAxes, ha="right", color="#4F6072", fontsize=8)
    fig.savefig(args.directory / "watchdog.png", dpi=180)
    plt.close(fig)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, KeyError, ValueError, IndexError) as exc:
        sys.exit(f"Cannot plot SIL CSVs: {exc}")
