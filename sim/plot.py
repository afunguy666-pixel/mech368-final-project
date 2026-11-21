#!/usr/bin/env python3
"""根据 MECH 368 SIL 场景的 CSV 生成可选的图表。"""
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
        parser.exit(2, "画图是可选功能；安装 matplotlib 后才能生成图表。\n")

    cases = [
        ("straight", "01  直行"),
        ("turn90", "02  直行 + 90° 转弯"),
        ("sensor_error", "03  传感器误差场景"),
    ]
    # 中文需要 CJK 字体：按顺序尝试常见字体，都没有时中文会显示为方框。
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                         "font.sans-serif": ["Noto Sans CJK SC", "Source Han Sans SC", "Microsoft YaHei",
                                             "PingFang SC", "SimHei", "WenQuanYi Zen Hei", "DejaVu Sans"],
                         "axes.unicode_minus": False})
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.3), layout="constrained")
    fig.suptitle("MECH 368  |  轨迹估计对比", x=0.02, ha="left", fontweight="bold")
    for ax, (case, title) in zip(axes, cases):
        rows = read_csv(args.directory, case)
        for prefix, label, color, style in [
            ("true", "真值", "#243048", "-"),
            ("enc", "编码器", "#CE673E", "--"),
            ("fused", "编码器 + 陀螺仪", "#00798C", ":"),
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
            ax.text(0.03, 0.04, f"终点误差：{errors[0]:.1f} 对比 {errors[1]:.1f} cm",
                    transform=ax.transAxes, fontsize=8, color="#4F6072")
    axes[-1].legend(loc="best", fontsize=8)
    fig.savefig(args.directory / "tracking.png", dpi=180)
    plt.close(fig)

    rows = read_csv(args.directory, "watchdog")
    t = [v / 1000 for v in numbers(rows, "t_ms")]
    fig, ax = plt.subplots(figsize=(9.5, 4.5), layout="constrained")
    ax.set_title("MECH 368  |  连着但不通：看门狗测试", loc="left", fontweight="bold")
    ax.step(t, numbers(rows, "left_cmd_rad_s"), where="post", label="左轮", color="#243048", linewidth=2)
    ax.step(t, numbers(rows, "right_cmd_rad_s"), where="post", label="右轮", color="#00798C", linestyle="--", linewidth=2)
    stop = next((i for i, row in enumerate(rows) if "watchdog_timeout" in row["event"]), None)
    if stop is not None:
        last_accepted = next((i for i in range(stop - 1, -1, -1) if rows[i]["accepted"] == "1"), None)
        if last_accepted is not None:
            ax.axvspan(t[last_accepted], t[stop], color="#CE673E", alpha=0.13,
                       label=f"{(t[stop]-t[last_accepted])*1000:.0f} ms 软件看门狗")
            ax.axvline(t[stop], color="#CE673E", linestyle=":", linewidth=2)
    ax.set(xlabel="虚拟时间 (s)", ylabel="车轮指令 (rad/s)")
    ax.grid(alpha=0.22)
    ax.legend(loc="best")
    ax.text(0.99, 0.09, "模拟的指令值，不是物理停车的测量结果", transform=ax.transAxes, ha="right", color="#4F6072", fontsize=8)
    fig.savefig(args.directory / "watchdog.png", dpi=180)
    plt.close(fig)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, KeyError, ValueError, IndexError) as exc:
        sys.exit(f"无法根据 SIL CSV 画图：{exc}")
