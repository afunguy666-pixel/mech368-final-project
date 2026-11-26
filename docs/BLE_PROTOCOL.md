# Phase 1 BLE protocol and bench validation

[中文版 / Chinese](BLE_PROTOCOL.zh-CN.md)

## Scope and requirements

Source: `mech368-final-project.pdf`, course handout, page 3 (read 2026-11-20).
Two Nano 33 BLE Sense Rev2 boards communicate using ArduinoBLE. The course
requires zero motor speeds within at most 500 ms without a valid packet,
flashing disconnect indicators at both ends, and a remote E-stop latched until
an APDS-9960 gesture on the robot. The direct-wired firmware gates the DRV8833
through D8 and reads the onboard APDS. Loaded stopping time, handheld IMU
steering and a physical remote E-stop input remain bench work.

Robot = peripheral/server; remote = central/client. Service UUID
`c4130001-3680-4f01-9e10-627e70bca001`. Command characteristic uses `0002`
in place of `0001`, Write Without Response; status uses `0003`, Read + Notify.
Robot sends status every 50 ms. Remote replies with one command to each valid
status notification (so one every 50 ms). No application `delay()`, USB wait, connected-device wait
loop, or blocking read of user input is used. Commands and notifications fit
the default 20-byte ATT payload.

## Wire format (version 1)

All multibyte integers are explicitly little-endian. No struct casting, floats,
packing pragmas, or native-endian serialization. Exactly 20 bytes:

| Offset | Bytes | Field |
| --- | --- | --- |
| 0 | 2 | Magic `36 8b` |
| 2 | 1 | Version `01` |
| 3 | 1 | Type: 1 motion, 2 E-stop, 3 status |
| 4 | 2 | uint16 sequence; status echoes last accepted command |
| 6 | 4 | uint32 connection session, nonzero |
| 10 | 2 | int16 linear velocity in mm/s; status uses flags instead |
| 12 | 2 | int16 angular velocity in mrad/s; status requires zero |
| 14 | 4 | uint32 robot-issued challenge, nonzero |
| 18 | 2 | CRC-16/CCITT-FALSE of bytes 0..17, init FFFF, poly 1021, no reflection/xorout |

Motion bounds are +/-1000 mm/s and +/-4000 mrad/s. These are parser bounds,
not approved robot speeds. The remote sends ONLY zero motion in this phase.
E-stop must carry zero velocity fields. Unknown types, flag bits, or versions
are rejected. Status flags: bit 0 ACK valid, bit 1 E-stop latched, bit 2 software
motion permitted, bit 3 driver disabled (the firmware commands nSLEEP low).
An ACK means the protocol accepted the command, not that a motor moved.

## Freshness and safety

- Robot establishes a new session on each connection; old command permission
  and sequence tracking clear, but MotionSafety's E-stop latch persists.
- Robot issues a challenge with a 200 ms local lifetime and replaces it after
  any accepted command. Remote echoes it. This bounds acceptance of delayed
  commands without synchronizing clocks. A challenge can be used only once.
- Duplicate, reordered and half-range-ambiguous uint16 sequences are rejected.
  Wrap from 65535 to 0 works. More than 32767 skipped commands requires reconnect.
- Validation failures do not refresh MotionSafety or the application ACK.
  The software gate expires exactly 450 ms after the last accepted motion;
  explicit disconnect invalidates permission immediately when observed.
- Each GATT write callback processes the frame immediately, so E-stop is not
  lost when a motion write follows in the same BLE poll. A 21-byte characteristic
  capacity exposes oversize writes as invalid even if the BLE library truncates
  their payload; only length 20 is accepted.
- Remote considers the application healthy only on a progressing ACK matching
  its most recently sent sequence. Duplicate status, unsent sequence ACKs,
  malformed status, or wrong session cannot refresh its 450 ms health timer.
  Delayed ACKs can conservatively cause reconnect; tune with measurements.
- Robot flashes its built-in LED when disconnected, timed out, BLE init fails,
  or E-stop is latched. Remote flashes until it has fresh ACK progress.
- Serial `e` on the remote requests an E-stop. The remote repeats it across
  loss or reconnect until the robot acknowledges a matching stop sequence with
  its latch set. It then resumes sending zero-speed heartbeats.
- Any fresh APDS swipe on the car (UP, DOWN, LEFT or RIGHT) clears that latch. A stop received in the same poll
  takes priority. The car requires a later valid motion packet and a separate
  local UP to start again. A local DOWN cancels navigation.

## Limits of nonblocking behavior

ArduinoBLE 1.4.1 public connect, service discovery, subscription and disconnect
APIs are synchronous internally. The remote performs them in separate loop
phases and sets an ATT timeout of 100 ms; discovery can contain multiple such
waits. Retry is scheduled at 1 s. The robot never performs central discovery.
Steady-state application transfer uses write-without-response and notifications,
not ATT request/response waits. ArduinoBLE/HCI can still wait for transport flow
control, and `BLE.poll()` has no proven worst-case bound here. Remote LED updates
may pause during connection operations. This is cooperative application code,
not a fully asynchronous BLE stack or a verified 500 ms end-to-end stop.
Before motor enable, measure worst-case loop/stop latency, including radio
congestion and reconnect; use an asynchronous stack or independent hardware
watchdog if measured bounds cannot meet the budget.

CRC, UUID, session and challenge are not security authentication. Session uses
runtime timing plus a counter, is not persistent or cryptographically random,
and can repeat across reset. There is no bonding, peer allowlist, encryption
policy, or protection against a malicious radio peer. Multiple matching robots
are not distinguished: operate only one bench pair until pairing is implemented.
The E-stop latch resides in RAM. On boot the car commands nSLEEP low before
setting the bridge inputs or starting the sensors and radio.

## Reproducible checks

```
bash scripts/test-host.sh
arduino-cli core install arduino:mbed_nano@4.4.1
arduino-cli lib install ArduinoBLE@1.4.1 Arduino_APDS9960@1.0.4 Arduino_BMI270_BMM150@1.2.4
bash scripts/compile-firmware.sh
```

Host tests use C++11, no Arduino/BLE library and no hardware. They exercise wire
format, an independent CRC check vector, every single-bit corruption of a sample
frame, size/type/version/bounds errors, session/challenge/sequence rejection,
time and sequence rollover, watchdog integration, E-stop persistence and remote
ACK freshness. Original MotionSafety source and tests are unchanged.

## Required bench checks (NOT yet performed)

1. Keep motor power disconnected. Flash one sketch to each board; verify both
   can boot without USB serial and discover/reconnect in either power-up order.
2. Capture status/command traffic: 20-byte layout, 50 ms schedules, echoed ACK,
   changing challenge/session. Confirm both indicators stop flashing on health.
3. Interrupt either direction separately, then power off each board. Confirm
   robot permission zero and remote fault indication within the measured budget.
   Distinguish radio link loss from a connected-but-silent peer.
4. Inject short/oversized/bad-CRC/out-of-range/duplicate/old-session/expired-token
   frames; verify they cannot extend the watchdog or ACK health.
5. Send `e`, then reconnect. Check D8 stays low with the latch set. Swipe UP on
   the car to clear the latch. Confirm D8 stays low through the next heartbeat
   and only rises after another UP.
6. Measure worst loop time, last valid packet to disabled output and LED latency
   under interference and connection setup. Capture D8, bridge outputs and wheel
   speed to distinguish driver inhibition from the time the wheels reach rest.
