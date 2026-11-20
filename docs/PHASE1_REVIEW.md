# Phase 1 review — awaiting approval

Base: `c8ab0aba27543d3209b08c5290a7b848492d6815` (current main when cloned).
The provided `6fe2986` is its ancestor; the later setup/test comments are retained.
No commit, push, branch publication or board upload has been performed.

Proposed commit title: `Add phase-one BLE protocol, safety integration and host tests`

## Exact scope

| File | Change |
| --- | --- |
| firmware/robot/robot.ino | BLE peripheral, per-write validation, status notifications, software gate and built-in LED |
| firmware/remote/remote.ino | Central scan/connect/discover/subscribe/retry states, zero heartbeat, ACK timeout, sticky serial E-stop, built-in LED |
| libraries/Mech368BLE/library.properties | New portable library metadata |
| libraries/Mech368BLE/src/Mech368Protocol.h | 20-byte codec, CRC, field bounds, connection/challenge/sequence freshness, receiver adapter and remote ACK tracking |
| tests/ble_protocol_test.cpp | Standalone protocol, safety integration and simulated exchange/loss tests |
| scripts/test-host.sh | Run all host test sources with C++11 and warnings as errors |
| .github/workflows/ci.yml | Add pinned ArduinoBLE 1.4.1 dependency |
| README.md | Describe implemented phase and remaining work |
| docs/BLE_PROTOCOL.md | Exact protocol, limitations and unperformed bench checklist |
| docs/PHASE1_REVIEW.md | This review and validation record |

Original MotionSafety header, its tests, compile script and directory structure
are preserved. No motor pins, PWM calls or motor-driver writes have been added.

## Local validation on 2026-11-20

- `bash scripts/test-host.sh`: PASS for both motion_safety_test.cpp and
  ble_protocol_test.cpp, including a simulated exchange with frozen replay/loss.
- AddressSanitizer + UndefinedBehaviorSanitizer protocol run: PASS with leak
  detection disabled. LeakSanitizer is unsupported under this environment's
  ptrace instrumentation; no leak-check result is claimed.
- `bash scripts/compile-firmware.sh`: PASS with Arduino CLI 1.3.1,
  arduino:mbed_nano 4.4.1, ArduinoBLE 1.4.1, target arduino:mbed_nano:nano33ble.
- Robot: 327256 bytes flash (33%); 69712 bytes globals (26%).
- Remote: 327648 bytes flash (33%); 69784 bytes globals (26%).
- Toolchain archives downloaded through the available download route and
  SHA checksums checked against Arduino's package index; the normal CLI network
  download was unavailable in this environment.
- `git diff --check`: PASS. No GitHub Actions run for this unsubmitted change.

## Material limitations

No two-board hardware test, timing measurement, physical output verification,
or electrical safety validation. Remote sends zero motion only. APDS recovery,
IMU steering and physical E-stop input remain unimplemented. Serial `e` is only
a bench trigger. GPIO motor output is absent, not proof of electrical inhibition.

Application scheduling is cooperative; ArduinoBLE connection/discovery and HCI
operations can still block. The 100 ms ATT timeout does not bound whole service
discovery. The 450 ms software gate is tested, but a <=500 ms physical stop and
uninterrupted LED response are not established. Sessions/challenges are freshness
checks, not authenticated pairing or persistent anti-replay protection.

Approval requested only after reviewing the patch: create a separate feature
branch from this base and commit/push these ten files, leaving main unchanged.
Recheck remote head before publication; if it moved, preserve and reconcile
upstream changes before proceeding.
