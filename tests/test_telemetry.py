import binascii
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import replace
import io
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from calibrate_drive import estimate
from telemetry import Sink, record
from telemetry_wire import BIAS_READY, DETOUR, OFF_LINE, RUNNING, Stream, apply_marks, read_csv
from trajectory import Point, Reference, ground_errors, poses


def packet(seq, ms, epoch=1, session=11, flags=BIAS_READY, values=(0, 0, 0, 0)):
    def checked(data):
        return data + struct.pack("<H", binascii.crc_hqx(data, 0xffff))
    frame = checked(struct.pack("<BBHIhhhi", 0x11, flags, seq, ms, *values))
    return b"@" + checked(struct.pack("<II", epoch, session) + frame).hex().encode() + b"\n"


class RecorderTests(unittest.TestCase):
    def test_wire_resync_crc_loss_and_wrap(self):
        s = Stream("teach", "test")
        raw = packet(65535, 0xffffffd0)
        self.assertEqual(list(s.feed(raw[:10])), [])
        a, = s.feed(raw[10:])
        s.mark("s")
        b, = s.feed(b"@fragment" + packet(1, 152))
        self.assertEqual((b["gap"], b["course_segment"], b["robot_ms"] - a["robot_ms"]), (1, 0, 200))
        bad = bytearray(packet(2, 252))
        bad[23] = ord("F")
        self.assertEqual(list(s.feed(bad)), [])
        self.assertEqual(list(s.feed(packet(1, 152))), [])
        # Inner CRC is checked even when a valid serial envelope was rebuilt.
        data = bytearray.fromhex(packet(2, 252)[1:-1].decode())
        data[14] ^= 1
        data[-2:] = struct.pack("<H", binascii.crc_hqx(data[:-2], 0xffff))
        self.assertEqual(list(s.feed(b"@" + data.hex().encode() + b"\n")), [])

    def test_reconnect_requires_physical_origin(self):
        s = Stream("live", "test")
        rows = list(s.feed(packet(0, 10)))
        s.mark("s")
        rows.extend(s.feed(packet(1, 110)))
        rows.extend(s.feed(packet(0, 550, epoch=2, session=22)))
        self.assertEqual(rows[-1]["log_segment"], 2)
        self.assertEqual(rows[-1]["course_segment"], "")
        with self.assertRaises(ValueError):
            poses(rows)
        with self.assertRaises(ValueError):
            poses(rows, 2)
        self.assertEqual(poses(rows, 1)[0].x, 0)
        s.reopen()
        self.assertEqual(next(s.feed(packet(1, 650, epoch=2, session=22)))["log_segment"], 3)

    def test_full_disk_drops_and_decoder_keeps_running(self):
        class FailingDisk(io.StringIO):
            full = False
            def flush(self):
                if self.full:
                    raise OSError("volume full")
        file = FailingDisk()
        sink = Sink(file)
        file.full = True
        s = Stream("teach", "test")
        with redirect_stderr(io.StringIO()):
            for seq in range(10):
                sink.append(next(s.feed(packet(seq, 100*seq))))
        self.assertTrue(sink.failed)
        self.assertEqual(next(s.feed(packet(10, 1000)))["gap"], 0)

    def test_record_entry_reopens_without_joining_or_writing_to_port(self):
        class FakePort:
            def __init__(self, chunks):
                self.chunks = iter(chunks)
                self.in_waiting = 128
            def read(self, size):
                value = next(self.chunks)
                if isinstance(value, BaseException):
                    raise value
                return value
            def close(self):
                pass
        # Captured from the compiled handheld on the repository's virtual course.
        golden = b"@010000007FB41100112001003C050000000000000000000000007BDE4712\n"
        ports = iter([FakePort([golden, b"@partial", OSError("unplug")]),
                      FakePort([golden, KeyboardInterrupt()])])
        serial = SimpleNamespace(serial_for_url=lambda *a, **kw: next(ports), SerialException=OSError)
        with tempfile.TemporaryDirectory() as tmp:
            args = SimpleNamespace(output=Path(tmp) / "capture.csv", run="live", port="COM5",
                                   reference=None, headless=True)
            with patch.dict(sys.modules, {"serial": serial}), redirect_stdout(io.StringIO()):
                record(args)
            rows = read_csv(args.output)
        self.assertEqual([r["log_segment"] for r in rows], ["1","2"])
        self.assertEqual([(r["robot_ms"], r["flags"], r["x_mm"]) for r in rows],
                         [("1340","32","0")] * 2)

    def test_straight_pivot_and_fixed_heading(self):
        s = Stream("teach", "test")
        rows = []
        for i, v in enumerate(((500, -450, 1571, 200), (500, 550, 1571, 1200),
                               (500, 550, 3142, 1200))):
            rows.extend(s.feed(packet(i, i*100, values=v)))
        track = poses(apply_marks(rows, [dict(log_segment=1, robot_ms=0, course_segment=0)]))
        self.assertAlmostEqual(track[1].x, 1, places=4)
        self.assertAlmostEqual(track[1].y, 0, places=3)
        self.assertEqual(track[1].progress, track[2].progress)
        self.assertAlmostEqual(track[2].yaw, math.pi/2, places=3)
        self.assertLess(max(ground_errors(track, [dict(robot_ms=50, x_m=.5, y_m=0)])), .0002)
        rows[0]["flags"] |= RUNNING
        with self.assertRaises(ValueError):
            poses(rows)

    def test_parallel_lane_and_invalid_reference(self):
        p = Point(0, 0, 0, 0, 0, 0, 0, RUNNING | BIAS_READY, 0, False)
        q = replace(p, ms=100, x=.05, progress=.05, travel=.05)
        rival = replace(p, segment=1, y=.05)
        ref = Reference([p, q, rival, replace(q, segment=1, y=.05)])
        live = replace(p, x=.025, progress=.025)
        self.assertEqual(ref.match(live).reason, "matched")
        self.assertEqual(ref.match(replace(live, y=.04)).reason, "parallel ambiguity")
        self.assertEqual(ref.match(replace(live, yaw=math.pi)).reason, "heading / position")
        self.assertEqual(ref.match(replace(live, progress=.5)).reason, "no unique mileage")
        for kw in [dict(gap=1), dict(flags=p.flags | DETOUR), dict(flags=p.flags | OFF_LINE)]:
            self.assertNotEqual(ref.match(replace(live, **kw)).reason, "matched")
            self.assertNotEqual(Reference([p, replace(q, **kw)]).match(live).reason, "matched")

    def test_fit_independent_directions_cpr_rollout_and_track(self):
        trials = [("left_rev", 2, -1000, 0), ("right_rev", 2, 0, 1200),
                  ("roll_mm", 500, -1250, 1500), ("turn_deg", 180, 400, 480),
                  ("turn_deg", -180, -400, -480)]
        rows = [dict(kind=k, value=v, left_delta=l, right_delta=r) for k,v,l,r in trials]
        cl, cr, ll, lr, track = estimate(rows)
        self.assertEqual((cl, cr, ll, lr), (-500, 600, 200, 200))
        self.assertAlmostEqual(track, 320/math.pi)


if __name__ == "__main__":
    unittest.main()
