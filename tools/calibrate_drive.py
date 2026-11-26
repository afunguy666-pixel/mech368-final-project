"""Calculate independent wheel scales and effective track from measured trials."""
import math
import statistics


def estimate(rows):
    groups = {}
    for row in rows:
        kind = row["kind"]
        value = float(row["value"])
        if kind not in ("left_rev", "right_rev", "roll_mm", "turn_deg"):
            raise ValueError(f"unknown calibration trial {kind}")
        if not math.isfinite(value) or value == 0:
            raise ValueError("each trial needs a finite, nonzero independent measurement")
        left, right = (int(row[k] or 0) for k in ("left_delta", "right_delta"))
        groups.setdefault(kind, []).append((value, left, right))
    if set(groups) != {"left_rev", "right_rev", "roll_mm", "turn_deg"}:
        raise ValueError("supply left_rev, right_rev, loaded roll_mm and turn_deg trials")
    scales = []
    for wheel, i in (("left", 1), ("right", 2)):
        cp = [r[i] / r[0] for r in groups[wheel + "_rev"]]
        if any(c == 0 or c * cp[0] <= 0 for c in cp):
            raise ValueError(f"{wheel} encoder direction is inconsistent")
        signed_cpr = statistics.median(cp)
        mm_per_count = [r[0] / r[i] for r in groups["roll_mm"] if r[i]]
        if len(mm_per_count) != len(groups["roll_mm"]) or any(k * signed_cpr <= 0 for k in mm_per_count):
            raise ValueError(f"{wheel} straight rollout disagrees with forward direction")
        scales.append((signed_cpr, statistics.median(mm_per_count)))
    (cl, dl), (cr, dr) = scales
    widths = [(r * dr - l * dl) / math.radians(angle) for angle, l, r in groups["turn_deg"]]
    if any(w <= 0 for w in widths):
        raise ValueError("pivot wheel directions disagree with independent CCW-positive yaw")
    return cl, cr, cl * dl, cr * dr, statistics.median(widths)
