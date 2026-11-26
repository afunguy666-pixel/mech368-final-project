"""Decoder shared by Windows COM recording and the two-sketch simulator importer."""
import binascii
import csv
import struct

RUNNING, CALIBRATING, DETOUR, OFF_LINE, DRIVING, BIAS_READY, LATCHED, CLIPPED = (
    1, 2, 4, 8, 16, 32, 64, 128)
KINDS = {1: "pose", 2: "ir", 3: "wheels", 4: "env"}
FIELDS = ("source run log_segment bridge_epoch session kind seq robot_ms course_segment "
          "gap flags x_mm y_mm yaw_mrad travel_mm ir0 ir1 ir2 ir3 error_mm "
          "left_count right_count bias_mdps battery_adc battery_mv temp_c nav_state "
          "safety_flags bias_samples temp_age_ms").split()


def decode(line):
    if len(line) != 61 or line[:1] != b"@":
        return None
    try:
        data = binascii.unhexlify(line[1:])
    except (binascii.Error, ValueError):
        return None
    epoch, session = struct.unpack_from("<II", data)
    raw = data[8:28]
    if (not session or binascii.crc_hqx(data[:28], 0xffff) != int.from_bytes(data[28:], "little")
            or binascii.crc_hqx(raw[:18], 0xffff) != int.from_bytes(raw[18:], "little")):
        return None
    tag, flags, seq, ms = struct.unpack_from("<BBHI", raw)
    if tag >> 4 != 1 or (tag & 15) not in KINDS:
        return None
    kind = KINDS[tag & 15]
    row = dict(bridge_epoch=epoch, session=session, kind=kind, flags=flags, seq=seq, robot_ms=ms)
    if kind == "pose":
        values = struct.unpack_from("<hhhi", raw, 8)
        row.update(zip(("x_mm", "y_mm", "yaw_mrad", "travel_mm"), values))
    elif kind == "ir":
        *ir, error = struct.unpack_from("<4Hh", raw, 8)
        row.update((f"ir{i}", v) for i, v in enumerate(ir))
        row["error_mm"] = error if error != -32768 else ""
    elif kind == "wheels":
        left, right, bias = struct.unpack_from("<iih", raw, 8)
        row.update(left_count=left, right_count=right, bias_mdps=bias if bias != -32768 else "")
    else:
        adc, mv, temp, state, safety, samples, age = struct.unpack_from("<HHh4B", raw, 8)
        row.update(battery_adc=adc, battery_mv=mv, temp_c=temp / 100 if temp != -32768 else "",
                   nav_state=state, safety_flags=safety, bias_samples=samples,
                   temp_age_ms=age * 10 if age != 255 else "")
    return row


class Stream:
    """Bounded framing, link segmentation, robot-clock extension and per-kind loss."""
    def __init__(self, run, source):
        self.run = run
        self.source = source
        self.segment = 0
        self.reopen()

    def reopen(self):
        self.buffer = b""
        self.key = None
        self.course = ""
        self.pending_mark = None

    def mark(self, key):
        if self.key is None:
            return
        if key == "s" and self.course == "":
            self.pending_mark = 0
        elif key == "m" and self.course != "":
            self.pending_mark = self.course + 1

    def feed(self, chunk):
        pieces = (self.buffer + chunk).split(b"\n")
        tail = pieces.pop()
        self.buffer = tail[tail.rfind(b"@"):] if b"@" in tail[-61:] else b""
        for piece in pieces:
            row = decode(piece[piece.rfind(b"@"):].rstrip(b"\r"))
            if row is None:
                continue
            key = (row["bridge_epoch"], row["session"])
            if key != self.key:
                self.key = key
                self.segment += 1
                self.sequences = {}
                self.latest_ms = row["robot_ms"]
                self.course = ""
                self.pending_mark = None
            raw_ms = row["robot_ms"]
            delta_ms = ((raw_ms - self.latest_ms + (1 << 31)) % (1 << 32)) - (1 << 31)
            ms = self.latest_ms + delta_ms
            kind = row["kind"]
            previous = self.sequences.get(kind)
            gap = 0
            if previous is not None:
                prev_seq, prev_ms = previous
                if ms <= prev_ms:
                    continue
                # Use robot time to disambiguate a long outage spanning seq wrap.
                period = 100 if kind == "pose" else 1000
                step = (row["seq"] - prev_seq) & 0xffff
                wraps = max(0, round(((ms - prev_ms) / period - step) / 65536))
                step += wraps * 65536
                if not step:
                    continue
                gap = step - 1
            self.latest_ms = max(ms, self.latest_ms)
            self.sequences[kind] = row["seq"], ms
            if kind == "pose" and self.pending_mark is not None:
                if self.pending_mark or stationary(row):
                    self.course = self.pending_mark
                self.pending_mark = None
            row.update(robot_ms=ms, source=self.source, run=self.run, log_segment=self.segment,
                       course_segment=self.course, gap=gap)
            yield row


def stationary(row):
    flags = int(row["flags"])
    return bool(flags & BIAS_READY) and not flags & (CALIBRATING | RUNNING | DRIVING | CLIPPED)


def write_csv(path, rows):
    with open(path, "x", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path):
    with open(path, encoding="utf-8-sig", newline="") as file:
        return list(csv.DictReader(file))


def apply_marks(rows, marks):
    """Marker times are robot milliseconds observed on independent track landmarks."""
    groups = {}
    for mark in marks:
        seg, ms, course = (int(mark[k]) for k in ("log_segment", "robot_ms", "course_segment"))
        group = groups.setdefault(seg, [])
        if course != len(group) or (group and ms <= group[-1]):
            raise ValueError("markers must be 0,1,... in strictly increasing time per log segment")
        group.append(ms)
    applied = {}
    for row in rows:
        seg = int(row["log_segment"])
        i = applied.get(seg, -1)
        boundaries = groups.get(seg, [])
        if row["kind"] == "pose" and i + 1 < len(boundaries) and int(row["robot_ms"]) >= boundaries[i + 1]:
            if i < 0 and not stationary(row):
                raise ValueError("start marker needs a stationary, bias-ready pose at the fixed origin")
            if i + 2 < len(boundaries) and int(row["robot_ms"]) >= boundaries[i + 2]:
                raise ValueError("multiple markers inside one pose gap")
            i += 1
            applied[seg] = i
        row["course_segment"] = i if i >= 0 else ""
    if any(applied.get(seg, -1) + 1 != len(group) for seg, group in groups.items()):
        raise ValueError("marker has no subsequent pose in its log segment")
    return rows
