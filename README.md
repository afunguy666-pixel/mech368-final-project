# MECH 368 mobile robot starter

Development scaffold for two Arduino Nano 33 BLE Sense Rev2 boards: one on the
robot, one in the handheld remote. Phase 1 adds BLE command/status transport,
portable packet validation and integration with the existing motion-permission
state machine. Remote traffic is zero-speed heartbeat or a serial-triggered
E-stop. All actual motor outputs remain absent/disabled; no driver pins assumed.
This is not a working robot, a demonstrated safety system, or a submission-ready project.

See [BLE protocol and bench checklist](docs/BLE_PROTOCOL.md) for the exact wire
format, timing, setup API blocking limitations and unperformed hardware tests.

| Path | Purpose |
| --- | --- |
| `firmware/robot/` | Robot sketch; integration point for BLE, local recovery, and motor gating |
| `firmware/remote/` | Remote sketch; integration point for pose estimation, BLE, and the E-stop request |
| `libraries/Mech368BLE/` | Portable packet codec, receiver and remote ACK tracking |
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
arduino-cli lib install ArduinoBLE@1.4.1
bash scripts/compile-firmware.sh
```

The host test requires a C++11 compiler; set `CXX` if necessary. Compile only:
neither script flashes hardware. Arduino's [Rev2 manual](https://docs.arduino.cc/tutorials/nano-33-ble-sense-rev2/cheat-sheet/)
identifies the Mbed OS Nano package. The build target is `arduino:mbed_nano:nano33ble`.

## Implement and verify on the bench

1. **BLE and pose steering.** Verify the two actual boards and confirm which
   endpoint advertises. Measure BLE processing latency, including synchronous setup calls. Map
   calibrated remote roll and pitch to linear and angular velocity. Review the implemented
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

## Software-in-the-loop (SIL) simulator

Run the host tests and generate five CSV traces without an Arduino, BLE hardware,
Python, or network access:

```sh
bash scripts/run-sil.sh
```

The traces go to `build/sil/`; pass a directory as the first argument to change
the destination. Use `CXX` to select a C++11 compiler. Run from Bash (for example,
Git Bash or WSL on Windows with a compiler on PATH). The CI host job also runs
this command; the existing firmware compilation remains independent.

Optional, with Python and matplotlib available, generate `tracking.png` and
`watchdog.png` from the CSV directory:

```sh
python sim/plot.py build/sil
```


The simulator advances a **virtual clock in 10 ms steps**, without sleeping.
A scripted remote generates commands every 50 ms. It passes correctly encoded
frames through an in-process queue with a fixed 20 ms delivery delay and a
scripted drop interval. The real portable `Receiver` and its `MotionSafety`
instance gate command values before the kinematic model consumes them.
Challenge/status is sampled instantly by the scripted remote; the simulator
does not emulate the actual BLE stack, timings, link budget, or remote firmware.

| CSV | What the test checks |
| --- | --- |
| `straight.csv` | Ideal straight motion: 0.50 m in 2 s after first accepted command; both estimators agree to within encoder resolution. |
| `turn90.csv` | One second straight (0.25 m), then a one-second pivot at 1.571 rad/s: near 90° without further translation. |
| `sensor_error.csv` | Straight/pivot/straight with fixed seed 368, opposite 2% wheel scale biases, bounded encoder/gyro noise, and fixed gyro bias. The test checks repeatability and nonzero finite error rather than assuming fusion wins. |
| `watchdog.csv` | While still virtually connected, commands go to zero at 970 ms (520 ms last accepted + 450 ms). A fresh frame restores control at 1070 ms. Explicit disconnect at 1300 ms zeros commands immediately; reconnect at 1450 ms remains blocked until a fresh frame at 1470 ms. |
| `estop.csv` | E-stop at 320 ms sets both wheel commands to zero and survives disconnect/reconnect. A synthetic **local** gesture at 900 ms clears the latch but leaves commands zero until the fresh frame at 920 ms. |

Each row records the receiver state, the gated chassis command (mm/s and
mrad/s), wheel commands (rad/s), and three XY/yaw poses (m and rad). Poses are
at the timestamp before applying its command to the next 10 ms interval; yaw
is unwrapped. Event labels mark packet delivery/drop, watchdog expiry,
disconnect, reconnect, E-stop, and local recovery.

The virtual geometry is a 0.10 m track width and 0.020 m wheel radius, with 1024
counts per wheel revolution. These and the noise/bias parameters are grouped in
`sil::Config` (`sim/Simulation.h`) and can be supplied to `sil::run`. Default CSVs
use the in-code defaults. Encoder deltas come from successive rounded cumulative
counts so small increments are not repeatedly rounded away. Ground
truth is ideal differential-drive motion without slip, inertia, or braking
dynamics. Both estimators use the same observed wheel increments for distance;
encoder-only yaw comes from their difference, while fused yaw increments are a
fixed 10% encoder / 90% gyro blend. The weights and biases are illustrative,
not calibrated. A lower error here does not establish performance on the
actual robot or guarantee the blend helps under other conditions.

SIL validates the gating and model under the given scripts, not hardware
stopping behavior. Still verify the actual BLE exchange, loop and driver
latency, E-stop/re-arm I/O, encoder scaling, gyro calibration, and mechanical
behavior on a bench before applying motor power for a loaded test.
