# Firmware-in-the-loop simulator

[中文版 / Chinese](SIMULATOR.zh-CN.md)

Test the **unmodified** robot and remote sketches on a PC before the car
exists. `scripts/run-fw-sim.sh` compiles `firmware/robot/robot.ino`,
`firmware/remote/remote.ino` and the example sketches against host stand-ins
for the Arduino core, ArduinoBLE, the Rev2 IMU (`Arduino_BMI270_BMM150`), the
gesture sensor (`Arduino_APDS9960`) and the temperature sensor
(`Arduino_HS300x`). Two virtual boards then run on a
virtual clock, connected by a simulated BLE radio. The robot board is wired to
a virtual differential-drive vehicle on a virtual course.

This is a different tool from the kinematic SIL in `sim/Simulation.h`
(`scripts/run-sil.sh`). That one re-implements the loop around the portable
`Receiver`. This one runs your actual `setup()`/`loop()`/BLE callbacks.

```bash
bash scripts/run-fw-sim.sh
```

That runs every scenario and writes replays to `build/fwsim/`. Other commands:

```bash
bash scripts/run-fw-sim.sh estop_latch
```

```bash
bash scripts/run-fw-sim.sh --seeds 20 link_stability
```

```bash
bash scripts/run-fw-sim.sh --list
```

The first runs one scenario with a full event and serial log. The second runs
a scenario with seeds 1..20 (Monte-Carlo over radio timing and noise). The
third lists the scenarios. You need Bash and a C++17 compiler (`CXX`
overrides `c++`). No Arduino CLI, board, Python or network is needed.

For each run you get:

| File | Contents |
| --- | --- |
| `<scenario>.html` | Self-contained replay: top-down view, robot trail, sensor rays, LEDs, link/status flags, logic-analyser strip, events + serial log, checks. Open it in any browser. |
| `<scenario>.csv` | 10 ms samples: pose, wheel speeds, motor duty, battery, line offset, LEDs, link, status flags, sensor readings |
| `<scenario>.map.csv` | Temperature-map points the sketch printed (`MAP x y tempC`), with the robot's true position and the true temperature there |
| `<scenario>.frames.csv` | Every BLE frame on the air: send/deliver time, direction, fate (delivered/lost/blocked/corrupted/link down), decoded Mech368 fields |
| `<scenario>.log` | Timestamped events, both boards' `Serial` output, and the check results |

## What is simulated

| Part | Model |
| --- | --- |
| Time | Virtual microsecond clock per board. Each `loop()` costs 1 ms plus 0–0.3 ms random jitter. API calls cost a few µs, so busy-wait loops on `millis()` terminate. `delay()` advances the clock. A single `setup()`/`loop()` call stuck for more than 20 s of virtual time aborts the run. |
| Pins | `pinMode`, `digitalWrite/Read`, `analogWrite/Read` (+ resolution), `attachInterrupt` (CHANGE/RISING/FALLING), `noInterrupts`, `pulseIn`, Nano 33 BLE pin numbers (`D0`–`D13`, `A0`–`A7`, `LEDR/G/B`). |
| Serial | Output is captured with timestamps. Scenario scripts can type input. |
| BLE | Peripheral + central subset of ArduinoBLE 1.4: advertise/scan/connect/discover/subscribe, write without response, notify, event handlers (run inside `BLE.poll()`). Radio: 10 ms + 0–5 ms latency, in-order delivery, 2 s supervision timeout, randomised blocking connect/discover/subscribe durations. Faults: loss, bit corruption, one-way blocking ("connected but silent"), out of range, board power-off. |
| Vehicle | Motor drivers `pwm_dir`, `in_in` (DRV8833 style), `en_in_in` (L298N/TB6612 style), each with an optional `sleep=` pin that turns the outputs off while LOW. First-order motor speed response scaled by battery voltage, deadband, exact differential-drive kinematics, quadrature encoders that fire your ISRs, collision circle. |
| Sensors | Analog/digital reflective line sensors over tape, HC-SR04-style ultrasonic (trigger pulse + `pulseIn`), Sharp-style analog IR distance, reflective IR obstacle pairs through a comparator (`ir_proximity`: digital output within `range` over a `fov_deg` beam), comparator line sensors also readable with `analogRead` (0 V / 3.3 V), battery divider, IMU gyro/accelerometer/magnetometer from the vehicle motion (with bias/noise), APDS gestures and proximity from the script, HS300x temperature from the world's temperature field with first-order lag, noise and offset (each read assumed to block ~10 ms). The remote's IMU follows a scripted hand tilt. |
| Course | `sim/worlds/*.world`: tape lines, ovals, boxes, circles, straight or rounded border walls, and a temperature field (`ambient` plus `heat` hot/cold spots). The firmware link scenarios run on `course.world`. |

All parameters live in two commented text files. **Every number in them is a
placeholder.** Replace them with your wiring and bench measurements:

- `sim/vehicles/example.vehicle`: pins, motor driver type, wheel and track
  size, encoder PPR, sensor positions, battery divider, gyro bias.
- `sim/worlds/course.world`: the final-project field traced from Figure 6 of
  the course handout (5 cm black track, serpentine path, three cones, rounded
  field border). The figure has no dimensions, so the layout uses an assumed
  scale of about 2.0 m x 1.5 m. Measure the real track in KAIS 1220 and update
  the coordinates. The hot corner (+12 °C) and cool corner (-8 °C) on 22 °C
  ambient are also assumptions. `oval.world` and `oval_obstacle.world` are
  simple practice loops.

## Writing a scenario

Scenarios are C++ files in `sim/fwsim/scenarios/`. Each scenario runs in a
fresh process, so sketch globals start from power-up every time.

```cpp
#include "Simulation.h"
using namespace fwsim;

FWSIM_SCENARIO(my_estop_test, "E-stop during a link loss still latches") {
  sim.loadWorld("sim/worlds/course.world");
  sim.loadVehicle("sim/vehicles/example.vehicle");
  sim.useSketch(Robot, "robot");            // firmware/robot/robot.ino
  sim.useSketch(Remote, "remote", 300);     // powered on at 300 ms
  sim.linkLoss(4000, 5000);                 // out of range 4-5 s
  sim.serialInput(Remote, 4500, "e");       // operator presses E-stop meanwhile
  sim.runUntil(9000);
  FWSIM_CHECK(sim, sim.blinking(Robot, LED_BUILTIN, 6000, 9000), "robot LED flashes");
}
```

| Stimulus | |
| --- | --- |
| `serialInput(board, ms, "text")` | type into a board's `Serial` |
| `gesture(ms, GESTURE_UP)` | APDS gesture in front of the robot |
| `tilt(ms, rollDeg, pitchDeg)` | keyframe for the remote's hand pose (interpolated) |
| `setInput(board, ms, pin, level)` / `setAnalog(board, ms, pin, volts)` | force a button or analog input |
| `setBattery(ms, volts)` | battery voltage step |
| `linkLoss(fromMs, toMs)`, `block(remoteToRobot, fromMs, toMs)`, `powerOff(board, ms)` | radio and power faults |
| `at(ms, [](Simulation& s) { ... }, "label")` | anything else, e.g. `s.radio().lossCentralToPeripheral = 0.1` or `s.world().boxes.clear()` |
| `seed(n)`, `board(b).timeOffsetUs` | random seed; start `millis()` near its 32-bit wrap |

| Observation | |
| --- | --- |
| `blinking`, `steady`, `levelAt`, `transitions` | any output pin on either board (LEDs...) |
| `frames()`, `statusSentBefore(ms)`, `lastCommandDelivered(ms)` | decoded BLE traffic |
| `vehicle()` (`pose`, `travelled`, `collisions`, `distanceReading`...), `maxLineOffset` | physical state |
| `serialContains(board, "text")`, `hasEvent("text")`, `samples()` | logs and trace |
| `mapPoints()`, `scoreMap(cellSize)` | temperature-map points and score (see below) |

Pins in `sim.levelAt(...)` are the same constants the sketch uses
(`LED_BUILTIN`, `LEDR`...). LEDs on the Nano 33 BLE RGB pins are active-low.

## Temperature mapping

Any line of the form `MAP x y tempC` (or `MAP,x,y,tempC`), printed on either
board's `Serial`, is collected as a map point. The simulator records the
robot's true position at the moment the line is printed. A point relayed
through the remote is printed later, which counts against its accuracy.
`scoreMap()` and the replay's "Temperature map" panel then report the
handout's mapping metrics:

| Metric | Meaning |
| --- | --- |
| points, unique, density | logged points; distinct 1 cm positions; unique points per m² of field |
| coverage | fraction of 0.2 m grid cells containing a point |
| position error | logged position vs. true position when printed (mean, max) |
| temp error | logged temperature vs. true air temperature where the robot really was |
| map error | logged temperature vs. true temperature at the logged position (what the heat map shows) |

The replay draws the true field underneath and the logged points on top,
coloured with the same scale. Run `course_heat_map` to see it with the demo
sketch. The demo enables mapping after it receives `pose X Y HEADING_DEG` on
`Serial` (the known starting pose), calibrates the gyro bias while it waits,
and estimates its pose from encoder distance and gyro heading.

## Your own sketches

Every `firmware/<name>/<name>.ino` and `sim/examples/<name>/<name>.ino` is
compiled in automatically under the sketch name `<name>`. To try a change
without touching the original, copy the sketch folder somewhere, rename the
folder and `.ino`, and substitute it into the existing scenarios:

```bash
FWSIM_SKETCHES=/path/to/remote_v2 bash scripts/run-fw-sim.sh --seeds 10 --sketch remote=remote_v2
```

`sim/examples/line_follower` is a demo (not course firmware). It drives
motors with encoder speed control, follows the line, stops for obstacles,
starts and stops on gestures, and lights the red LED on low battery. It shows
how the vehicle file's pins reach ordinary Arduino code.

`sim/examples/line_tracker` runs `libraries/Mech368Line` (calibrated IR array,
PD line following, 90-degree corners, cone detours) on
`sim/vehicles/ir_tracker.vehicle` and `ir_tracker_lm393.vehicle`. Its
`track_*` scenarios are described in [LINE_FOLLOWING.md](LINE_FOLLOWING.md).

## Findings on the Phase 1 firmware (2026-11-21)

- **Watchdog, E-stop latch, corruption and power-loss handling behaved as
  specified** in every seed: each status reporting "motion permitted" was
  within 450 ms of a delivered motion command, no corrupted frame extended
  the watchdog, and the E-stop latch survived link loss and reconnect.
- **Fixed: the remote intermittently dropped a healthy link.** The remote
  used to send commands on its own free-running 50 ms timer, independent of
  the robot's 50 ms status timer. In bad phase alignments each command
  carried the challenge from the status *before* last, so the robot rejected
  every other command as stale, and the ACK never matched the remote's most
  recently sent sequence. The remote then declared the link unhealthy and
  reconnected (about 1.5-2 s without control). `link_stability` failed on
  10/10 seeds, and one E-stop latched 815 ms after the request because it
  landed during such a reconnect.
  `firmware/remote/remote.ino` now sends exactly one command after each
  valid status notification, as `docs/BLE_PROTOCOL.md` describes. Afterwards
  all 14 scenarios that existed then passed on 10/10 seeds (17 today). This fix is validated in simulation
  only: confirm link stability and E-stop latency on the two real boards.
  A side effect to be aware of: if status notifications stop, the remote
  also stops sending (including E-stop frames). Its commands would carry a
  stale challenge in that case anyway, and the robot's watchdog stops motion.

- **Mapping lessons from the demo sketch** (simulated; the same mechanisms
  apply on the board):
  - Integrating each gyro reading as a fixed 10 ms (`gz * 0.01`) lost about
    8° over the six 90° turns. The blocking temperature reads and `pulseIn`
    calls make the loop skip IMU samples. The logged positions drifted to
    17 cm mean / 42 cm maximum error. Integrating over the measured time
    between reads (`micros()`) brought this to 1.6 cm / 6.4 cm.
  - With a 2 s sensor time constant at 0.25 m/s, each reading describes the
    air about 0.5 m back along the path. This smears the map (about 1.6 °C
    mean error). Measure the real HS300x response time, then either drive
    slower near the hot and cool areas or tag readings with an earlier pose.
- **Course geometry (assumed scale): the field border is close to the
  track.** In `course.world` each bottom joint of the track lies about 19 cm
  from the border, and each top joint about 24 cm. A simple "stop if anything
  is closer than 20 cm ahead" rule, like the demo sketch's, stops in front of
  the border before the turn if the border is a physical wall. Check the real
  field. Obstacle detection should probably only count objects on or near
  the track, or use a smaller threshold near the ends of a leg.

## Limits: what passing here does NOT show

- ArduinoBLE, HCI and the radio are behavioural models with assumed timings,
  not the real stack. Real latency, blocking times, flow control and
  supervision timeout must still be measured on the two boards.
- The Arduino API stand-ins are a permissive subset. For example
  `digitalWrite(pin, 1)` compiles here. The board build
  (`scripts/compile-firmware.sh`) remains the authority on what compiles.
  The host build does **not** generate Arduino's automatic function
  prototypes: declare a function before using it. Sketch-local `.cpp`
  files are not compiled.
- Libraries other than ArduinoBLE, `Arduino_BMI270_BMM150`,
  `Arduino_APDS9960` and `Arduino_HS300x` (plus this repository's
  `libraries/`) are not available. Add a stand-in under
  `sim/fwsim/hostduino/` if you need one.
- The temperature field is a smooth assumed model with no air currents, and
  the robot does not heat its own sensor.
- One boot per run: a board can be powered off, but not reset or powered
  on again within a scenario.
- Vehicle physics has no wheel slip, inertia of the chassis, motor
  saturation current or battery sag under load. Collisions only block
  translation. IMU axes assume the board is mounted flat, x forward, z up.
  Interrupts run instantly and never preempt more than one level.
- `delay()` does not process BLE events. Only `BLE.poll()` and blocking BLE
  calls do, as in the real library.

Simulation results are evidence about the code's logic under the modelled
conditions, never a substitute for the bench checks in
[BLE_PROTOCOL.md](BLE_PROTOCOL.md) and the README.
