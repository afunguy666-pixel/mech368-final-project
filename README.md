# MECH 368 mobile robot starter

Local starter for two Arduino Nano 33 BLE Sense Rev2 boards: one on the robot,
one in the handheld remote. The only working project logic here is a pure C++
motion-permission state machine. Both sketches are compile-only scaffolds:
they have no pin assignments, sensor reads, BLE traffic, or motor outputs.
This is not a working robot, a demonstrated safety system, or a submission-ready project.

| Path | Purpose |
| --- | --- |
| `firmware/robot/` | Robot sketch; integration point for BLE, local recovery, and motor gating |
| `firmware/remote/` | Remote sketch; integration point for pose estimation, BLE, and the E-stop request |
| `libraries/Mech368Safety/` | Portable state machine shared with the host test |
| `tests/` | Deterministic host assertions for watchdog and E-stop behavior |
| `.github/workflows/ci.yml` | Independent host-test and board-compile jobs; no uploads to boards |

## Run locally

On Linux/macOS, or in a Windows Bash environment with the tools on PATH:

```sh
bash scripts/test-host.sh

# Requires Arduino CLI 1.3.1 on PATH (same toolchain CI uses).
arduino-cli core update-index
arduino-cli core install arduino:mbed_nano@4.4.1
bash scripts/compile-firmware.sh
```

The host test requires a C++11 compiler; set `CXX` if necessary. Compile only:
neither script flashes hardware. Arduino's [Rev2 manual](https://docs.arduino.cc/tutorials/nano-33-ble-sense-rev2/cheat-sheet/)
identifies the Mbed OS Nano package. The build target is `arduino:mbed_nano:nano33ble`.

## Implement and verify on the bench

1. **BLE and pose steering.** Verify the two actual boards and confirm which
   endpoint advertises. Implement bounded, nonblocking BLE processing. Map
   calibrated remote roll and pitch to linear and angular velocity. Define
   command units, bounds, packet format and sequence/session handling. Validate
   type, size, version, bounds, and freshness before refreshing the watchdog;
   cached, malformed, duplicated or stale packets must not extend it. Treat
   reported disconnects immediately, and implement the required link-loss LED on
   each device (including detection of a stale peer on the remote).
2. **Motor inhibition and timing.** The state machine expires at 450 ms after
   the last new valid motion packet, leaving 50 ms for scheduling and output
   latency. That is not a hardware timing claim. With motors electrically
   isolated first, capture the last accepted packet and verified zero/disable
   output and demonstrate an end-to-end delay no longer than 500 ms under
   worst observed polling load. Test no packets at startup, packet silence,
   invalid traffic, abrupt disconnect and reconnect. Confirm actual motor
   driver polarity, disable behavior and electrical defaults before connecting
   motor power. All drive modes must share the same output gate.
3. **Emergency stop.** Show that a valid remote E-stop takes precedence over
   motion and a simultaneous recovery gesture, stays latched across a
   disconnect, and ignores further drive commands. Generate a new local
   APDS-9960 gesture event to clear the latch; confirm motion is still blocked
   until another good command arrives after recovery. The latch here is
   RAM-only. Verify startup/reset leaves physical outputs disabled; define
   a deliberate re-arm strategy before allowing motors to operate after reset.
4. **Remaining course hardware.** Verify the well-marked low-battery
   indicator and its threshold on your pack. Characterize wheel direction
   and encoder scaling, validate differential steering, and then test IR
   track following, obstacle avoidance, IMU/odometry pose tracking, and mapping
   against the course handout. Start motor tests with wheels clear of the
   bench and a safe independent means to cut power. Log measurements and
   observed behavior rather than treating compilation as acceptance.
