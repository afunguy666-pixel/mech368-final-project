"""Compile both production sketches using a private, credit-aware ArduinoBLE copy."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cli", default="arduino-cli")
    parser.add_argument("--trace-accepted", action="store_true",
                        help="bench build: car D0 acceptance trace and handheld D3 heartbeat interruption")
    parser.add_argument("--ble-library", type=Path, help="installed ArduinoBLE directory")
    args = parser.parse_args()
    source = args.ble_library
    if source is None:
        sketchbook = subprocess.check_output(
            [args.cli, "config", "get", "directories.user"], encoding="utf-8").strip()
        source = Path(sketchbook) / "libraries" / "ArduinoBLE"
    # Only the single-threaded BLE owner reads this snapshot. Stock writeValue
    # waits in HCIClass::sendAclPkt when no credits remain. The public ArduinoBLE
    # interface has no way to ask about those credits before a notification.
    build = ROOT / "build"
    build.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=build, prefix="ble-") as tmp:
        ble = Path(tmp) / "ArduinoBLE"
        shutil.copytree(source, ble, ignore=shutil.ignore_patterns(".git", "examples", "extras"))
        hci = ble / "src/utility/HCI.h"
        header = hci.read_text(encoding="utf-8")
        signature = "  virtual int sendAclPkt("
        implementation = hci.with_suffix(".cpp").read_text(encoding="utf-8")
        if header.count(signature) != 1 or "while (_pendingPkt >= _maxPkt)" not in implementation:
            raise SystemExit("Unknown ArduinoBLE HCI implementation. Review its credit handling before building.")
        accessor = "  int availableAclPackets() const { return int(_maxPkt) - int(_pendingPkt); }\n"
        hci.write_text(header.replace(signature, accessor + signature), encoding="utf-8")
        for sketch in ("robot", "remote"):
            flags = []
            output = "arduino-" + sketch
            if args.trace_accepted:
                define = "MECH368_TRACE_ACCEPTED" if sketch == "robot" else "MECH368_BENCH_SILENCE"
                flags = ["--build-property", "compiler.cpp.extra_flags=-D" + define]
                output += "-bench"
            subprocess.run([args.cli, "compile", "--fqbn", "arduino:mbed_nano:nano33ble",
                            *flags, "--library", str(ble), "--libraries", str(ROOT / "libraries"),
                            "--build-path", str(build / output),
                            str(ROOT / "firmware" / sketch)], check=True)


if __name__ == "__main__":
    main()
