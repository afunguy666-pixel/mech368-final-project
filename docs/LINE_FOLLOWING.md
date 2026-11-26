# Line following with the IR pairs

[中文版 / Chinese](LINE_FOLLOWING.zh-CN.md)

Autonomous navigation for the handout task: follow the 5 cm black track with
IR sensor pairs, take its 90-degree corners, go around the cones (handout
Figure 5) and stop at the end. The logic is a portable library; a sketch
wires it to the robot and the firmware simulator checks it on the field map.

| Path | Purpose |
| --- | --- |
| `libraries/Mech368Line/src/LineSensors.h` | Sensor calibration, normalisation, line position, lost-line memory |
| `libraries/Mech368Line/src/LineNavigator.h` | PD line follower and the track state machine (corners, lost line, end, detour) |
| `libraries/Mech368Line/src/DriveMath.h` | Twist/wheel kinematics (handout eq. 4, 6), odometry, wheel speed PI, gyro bias, roll/pitch |
| `tests/line_nav_test.cpp` | Host tests (run by `scripts/test-host.sh`) |
| `libraries/Mech368Tracker/src/DirectLineTracker.cpp` | Direct wiring, encoder scale, gyro, DRV8833 outputs and calibration sweep |
| `firmware/robot/robot.ino` | Autonomous car with BLE E-stop, local APDS recovery and a watchdog on the drive outputs |
| `sim/examples/line_tracker/line_tracker.ino` | Standalone navigation example with local gestures |
| `sim/vehicles/ir_tracker.vehicle`, `ir_tracker_lm393.vehicle` | Simulated robot with analog / LM393 line sensors |
| `sim/fwsim/scenarios/line_tracker.cpp` | Course scenarios (see Results) |

**Operating the direct-wired car.** Place the car on the line, let the gyro settle
and wait for a fresh BLE heartbeat. Swipe UP on the car to calibrate and follow.
Disconnect, heartbeat expiry or remote E-stop drives D8 (nSLEEP) low and cancels
the current manoeuvre. After reconnection a fresh UP is required to start.
After an E-stop any fresh local swipe clears the latch. A later heartbeat and
a separate UP allow movement. An interrupted calibration repeats around the original heading.
The paired firmware compiles for the Nano. Loaded stopping time and sensor
thresholds still need bench measurements.

## Robot geometry

The measurements come from the default configuration of `Car/寻线小车装配体.SLDASM`, including the four-pair line clamp, the front obstacle bracket and both retainers. The floor frame starts directly below the drive axle midpoint, with x forward and y left.

| Item | Geometry or simulator setting |
| --- | --- |
| Drive wheels | 65 mm diameter, 92 mm centre spacing |
| Ball caster | Spacer thickness increased from 9.9 to 14.9 mm, moving the ball down 5.0 mm |
| Support on level ground | With the chassis level, the caster extends about 0.003 mm below the wheel tangent plane. The three-contact solution is about 0.004° nose-up. The simulator uses 0°. |
| Line sensor clamp | 71 mm across and 16.4 mm fore–aft at the mounting pads. The pads are 4.8 mm thick and their lower face is 5.5 mm above the floor. |
| Line probe mounting height | The nominal low point of each lens sits 5.0 mm above the floor and projects 0.5 mm below the clamp. |
| Four line spots | The simulator samples directly below each pair's midpoint, 75.75 mm ahead of the axle at y = −30, −10, +10 and +30 mm (A0 to A3). |
| Obstacle sensing | The nominal LTE-4206 / LTR-4206 lens tips sit 78.25 mm ahead of the axle at y = ±20.00 mm. On each side the emitter is 28.25 mm above the floor and the receiver is 21.75 mm above it. |
| Obstacle bore axes | Both pairs point straight ahead at 0.00° yaw and pitch relative to the chassis. The left (D12) and right (D11) axes are 40.00 mm apart. |
| Collision check | 150 mm diameter circle centred 15.25 mm ahead of the axle. The projected CAD parts span a circle of about 149.6 mm diameter around that centre. |

Each emitter and receiver enters its stepped bore from above. The flange seats against the shoulder between the 3.45 mm guide and 4.50 mm counterbore. The fore–aft axes tilt inward 21.8° each and meet at the floor. They are 4.0 mm apart at 5.0 mm height and 4.4 mm apart at the clamp underside. Use a 5 mm gauge under the lenses before securing them. [Clamp and pair mounting dimensions](ir-pair-mount.png)

The centre of each shroud opening is 7.05 mm ahead of its seating shoulder. Both centres are 20.00 mm from the vehicle centreline. The simulator casts from the nominal lens tips with an assumed 80 mm range and 20° full field of view. The replay’s **CAD underside** panel overlays the four line spots and both forward sensor axes. [Obstacle mount angles](obstacle-mount-yaw.png) · [Side elevation and ground contacts](cad-ground-contact.png)

## Sensor hardware

Kit parts: LTE-4206 (IR emitter), LTR-4206 (IR phototransistor), LM393 (dual
comparator).

**Line array: 4 sensors read as analog voltages.** Phototransistor emitter to GND, collector to an
analog pin with a pull-up resistor to **3.3 V** (start with 10-47 kOhm and
adjust for a large floor/tape difference). Black tape reflects less IR, so the
transistor conducts less and the pin reads **high** (`BLACK_IS_HIGH = true`).
Analog readings give a continuous line position, which is what makes PD
steering smooth.

**LM393 alternative:** comparator output (open collector, pull-up to **3.3 V**)
into the same pins. The code reads it with `analogRead()` as 0 or full
scale, and the calibration sweep handles it automatically. Position is then
quantised to 0, +-1, +-2, +-3 cm, which still works (see Results) but
steers less smoothly and raises more false corner alarms. Set each threshold with the sensor over the tape edge.

**Never pull any signal up to 5 V.** The Nano 33 BLE pins are 3.3 V only. The
IR LEDs can run from 5 V through their own resistors (about 20 mA each).

A centred 5 cm tape covers the two inner sensors (+-10 mm) and leaves the
outer ones (+-30 mm) white, but only by 5 mm. Three sensors including one
outer go black both at a corner and when the robot is just 5-15 mm off
centre. The sketch therefore sets `confirmCornerByLoss` (see the state
machine). An array whose outer sensors sit more than one tape width (plus the
sensor spot) from the opposite inner sensor would not need it, for example
+-15/+-45 mm. That is wider than the CAD frame allows.

**Obstacle pair:** two forward-facing LTE/LTR pairs at ±20 mm on the front frame. Set the LM393
threshold for a detection distance of 6–8 cm on a cone
(handout Figure 5). Output LOW = object (`OBSTACLE_ACTIVE`). Keep the range
short: at the end of the track the border wall must not trigger a detour.

Ambient IR from lamps and sunlight shifts the readings: calibrate in the lab
where the demo runs, and shield the sensors from the side.

## Algorithm

### 1. Calibration and normalisation (`LineSensorArray`)
At start the robot pivots +-40 degrees on the line (gyro-controlled), so every
sensor sees both tape and floor. Each sensor's minimum and maximum become 0
(floor) and 1 (tape). Calibration is rejected if any sensor's spread is under
`MIN_CAL_SPAN` counts. A sensor that never left the floor cannot be scaled.
This is the method of Pololu's QTRSensors library (`calibrate()` /
`readCalibrated()`), reimplemented here.

### 2. Line position
`position = sum(v_i * y_i) / sum(v_i)` over sensors above a noise floor,
where `y_i` is the sensor's lateral offset in metres (+ = left). Same idea as
QTRSensors `readLineBlack()`, but in metres so any spacing works. When no
sensor sees black, the position is reported just beyond the outer sensor on
the side the line was last seen. The controller keeps turning toward it, and
the navigator knows which way to search.

### 3. PD steering with speed scheduling (`LineFollower`)
`omega = kp * e + kd * de/dt` (derivative low-pass filtered: digital sensors
move the position in steps), `v = cruise - (cruise - minSpeed) * min(1, |e| /
fullError)`. The robot slows down as the error grows, the same idea as the
`base = PB - KS * |error|` speed reduction used by the La Voiture robot. The
twist goes through `limitTwist()`, which keeps the turn rate and gives up
forward speed if a wheel would exceed its limit. Then `toWheels()` applies
handout eq. (4)/(6), and a per-wheel PI loop on the encoders runs the motors.

Choosing kp: with the sensors a distance L ahead of the axle and speed v, the
offset obeys `e'' + kp*L*e' + kp*v*e = 0`. So `wn = sqrt(kp*v)` and
`zeta = L*sqrt(kp) / (2*sqrt(v))`. For L = 0.07575 m (CAD), v = 0.25 m/s,
and kp = 90, wn = 4.7 rad/s and zeta = 0.72. A longer L damps more, a higher v damps less.

### 4. Track state machine (`LineNavigator`)

| Situation | Detected by | Action |
| --- | --- | --- |
| Corner | >= 3 sensors black including exactly one outer, 2 samples in a row | Drive `cornerAdvance` (sensor offset + half tape width) so the axle sits on the corner, pivot toward it until the line is centred (at least 45 deg, at most 135), settle 120 ms, follow. Next corner armed after 10 cm |
| Corner, `confirmCornerByLoss` (CAD array) | Drive straight for up to 7 cm from a debounced outer-sensor pattern. A sustained pattern on the opposite side starts a new 7 cm check on that side. | When the tape disappears, advance another 50.75 mm and pivot toward the confirmed side. If the tape continues, resume following and re-arm when the pattern clears. Obstacle sensing stays active. |
| Line lost to a side | No black for 4 samples, last position beyond `sideTolerance` (between inner and outer sensor) | Pivot toward that side up to 120 deg, then sweep back to -120 deg. Not found: stop (`lost`) |
| End of track | No black, last position near the centre | Creep 4 cm; still nothing: stop (`finished`) |
| Cone | Obstacle sensor for 3 samples | Brake, pivot 60 deg away from the sensor that fired, drive out 18 cm sideways, turn parallel, pass 32 cm, turn 60 deg back in, drive to the line, advance onto it, pivot back toward the detour side until centred, follow |
| Blocked on the way out | Obstacle sensor while driving out | Reverse to the start of the leg and try the other side once. Blocked again or while passing: stop (`blocked`) for a human |

The corner and rejoin sequences follow the intersection state machine of the
La Voiture robot: detect (debounced), stop, advance so the axle is on the turn
point, gyro-guided pivot, settle, re-arm after some distance. Here the pivot
ends when the line is centred under the sensors, not at a fixed angle. The
gyro only bounds it, so corners that are not exactly 90 deg still work.

After the detour the robot pivots toward the side it swerved to. That points
it along the line in the original direction, also when the line it meets is
the next leg of the track (this happens at the cone on the bottom joint).

During a detour the "line direction" is the gyro heading low-pass filtered
while following. Right after a corner, where it has not settled, it is the
pre-corner heading +- `cornerAngle` (90 deg). Set `cornerAngle = 0` for a
track whose corners are not square.

## Tuning on the robot

1. Bench: wheels off the ground. Check motor directions and encoder signs
   (forward duty must count up), then the speed PI: step `targetLeft/Right`
   and log the speed.
2. Sensors: print raw readings over floor and tape for each sensor and set the
   pull-up so the difference is at least a few hundred counts. Check
   `BLACK_IS_HIGH` (the sketch warns if inner sensors read lighter than outer).
3. Straight line at low `cruise` (0.15 m/s). Raise `kp` until it starts to
   oscillate, then halve it. Add `kd` to damp the oscillation. Then raise
   `cruise`.
4. Corners: if the pivot ends early or late, adjust `cornerAdvance` (axle
   position) before touching the pivot rates.
5. Cones: measure the obstacle sensors' trigger distance on a real cone. Set
   `sideOffset` to cone radius + half the robot width + 2-3 cm, and
   `passLength` from the geometry above.
6. Log the serial output (`nav: <state> (<reason>) at (x, y)`) on every run.

## Results in the simulator (placeholders, not measurements)

The times below run from power-on to the firmware's finish or stop command and include the start gesture and calibration sweep. Motor response, optical response and field dimensions are simulator inputs awaiting bench measurements.

```sh
bash scripts/run-fw-sim.sh robot_track_cones robot_track_estop robot_track_watchdog
bash scripts/run-fw-sim.sh track_cones            # standalone example
bash scripts/run-fw-sim.sh --seeds 10 track_serpentine track_cones track_cones_lm393 track_cones_on_line
```

The cone scenarios also check the obstacle inputs in isolation. A 1 mm diameter target placed 60 mm from either lens on its centre ray or at either 10° edge pulls its corresponding pin low. At 20° in either direction both inputs stay high. The paired cases also monitor D8 during E-stop and heartbeat loss.

| Scenario | Seed 368 | Seeds 1–10 |
| --- | --- | --- |
| `robot_track_cones` | Paired demo firmware stopped at 49.68 s, 3 detours, zero contacts, 4.2 mm peak offset while following | 10/10 |
| `track_serpentine` | Stopped at 33.31 s, 6 corners, 3.0 mm peak offset while following | 10/10 |
| `track_cones` | Stopped at 47.69 s, 3 detours, zero contacts | 10/10 |
| `track_cones_lm393` | Stopped at 49.35 s, 3 detours, zero contacts, 11.5 mm peak offset while following | 10/10 |
| `track_cones_on_line` | Stopped at 47.54 s, 3 detours, one retry on the other side, zero contacts | 10/10 |
| `track_gesture_stop` | Stopped on the local DOWN gesture | 10/10 |

Earlier stress runs with the previous geometry used wheel mismatch, extra gyro bias, slower motors, blurrier spots and shifted cones. The current checks assume a level floor, rigid wheels and a fixed optical response. Ambient IR, changing floor reflectance, tyre slip and actual mounted sensor height require bench measurements.

## References

- Pololu QTRSensors library (calibration and weighted-average line position):
  https://github.com/pololu/qtr-sensors-arduino
- Pololu 3pi / Zumo line-follower examples (calibration sweep, PD on the line
  position).
- Brett Beauregard, "Improving the Beginner's PID" (derivative kick,
  anti-windup, sample time): http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
- La Voiture robot (Elec-291-Group/La-Voiture, STM32): intersection state
  machine pattern and speed reduction with error. No code was copied (that
  repository has no licence).
- MECH 368 final project handout: differential-drive kinematics, Figure 5
  (obstacle detour), Figure 6 (field map).
