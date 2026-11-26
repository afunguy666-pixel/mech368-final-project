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
| `sim/examples/line_tracker/line_tracker.ino` | Example sketch: sensors, encoders, gyro, DRV8833 outputs (slow decay, nSLEEP, nFAULT), calibration sweep |
| `sim/vehicles/ir_tracker.vehicle`, `ir_tracker_lm393.vehicle` | Simulated robot with analog / LM393 line sensors |
| `sim/fwsim/scenarios/line_tracker.cpp` | Course scenarios (see Results) |

**Status:** simulation and host tests only. Nothing here has run on hardware,
and the sketch has not been compiled for the board in this environment (it is
compiled for the simulator with warnings on). It has no BLE link and no
MotionSafety gate: before it drives the real robot, merge it into
`firmware/robot` so every motor command passes the single safety-gated
output and the E-stop stops the navigator (`motionPermitted()` marks the spot).

## Sensor hardware

Kit parts: LTE-4206 (IR emitter), LTR-4206 (IR phototransistor), LM393 (dual
comparator).

**Line array (recommended): 4 sensors, 3 cm apart, 6 cm ahead of the axle,
read as analog voltages.** Phototransistor emitter to GND, collector to an
analog pin with a pull-up resistor to **3.3 V** (start with 10-47 kOhm and
adjust for a large floor/tape difference). Black tape reflects less IR, so the
transistor conducts less and the pin reads **high** (`BLACK_IS_HIGH = true`).
Analog readings give a continuous line position, which is what makes PD
steering smooth.

**LM393 alternative:** comparator output (open collector, pull-up to **3.3 V**)
into the same pins. The code reads it with `analogRead()` as 0 or full
scale, and the calibration sweep handles it automatically. Position is then
quantised to 0, +-1.5, +-3, +-4.5 cm, which still works (see Results) but
steers less smoothly. Set each threshold with the sensor over the tape edge.

**Never pull any signal up to 5 V.** The Nano 33 BLE pins are 3.3 V only. The
IR LEDs can run from 5 V through their own resistors (about 20 mA each).

Why this layout: with 3 cm spacing on 5 cm tape, a centred line covers the
two inner sensors and never the outer ones. One outer sensor turning black
while the inner ones are black means the black area is wider than the tape:
a corner. The 9 cm bar fits inside the 15 cm robot.

**Obstacle pair:** two LTE/LTR pairs at the front corners, toed out about
10 degrees, through an LM393 with the threshold set for 6-8 cm on a cone
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
`zeta = L*sqrt(kp) / (2*sqrt(v))`. For L = 0.06 m, v = 0.25 m/s, kp = 90:
wn = 4.7 rad/s, zeta = 0.57. A longer L damps more, a higher v damps less.

### 4. Track state machine (`LineNavigator`)

| Situation | Detected by | Action |
| --- | --- | --- |
| Corner | >= 3 sensors black including exactly one outer, 2 samples in a row | Drive `cornerAdvance` (sensor offset + half tape width) so the axle sits on the corner, pivot toward it until the line is centred (at least 45 deg, at most 135), settle 120 ms, follow. Next corner armed after 10 cm |
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

All vehicle and track numbers are assumptions (see the `.vehicle` files and
`course.world`). Times include 1.5 s standing still for the gyro bias and the
~1.8 s calibration sweep.

```sh
bash scripts/run-fw-sim.sh track_cones            # one run with full log + replay
bash scripts/run-fw-sim.sh --seeds 10 track_serpentine track_cones track_cones_lm393 track_cones_on_line
```

| Scenario | Course | Result (seed 368) | 10 seeds |
| --- | --- | --- | --- |
| `track_serpentine` | No cones, analog array | End at 31.7 s, 6 corners, max 7 mm offset while following | 10/10 |
| `track_cones` | Handout cones, analog | End at 43.2 s, 3 detours, 0 collisions | 10/10 |
| `track_cones_lm393` | Handout cones, LM393 array | End at 43.5 s, 3 detours, 0 collisions | 10/10 |
| `track_cones_on_line` | Cones on the tape centre | End at 44.7 s, 3 detours (one retried on the other side), 0 collisions | 10/10 |
| `track_gesture_stop` | DOWN gesture mid-run | Stops | 10/10 |

A one-off stress check (not committed) also completed the course with a 4 %
wheel size mismatch, 1.5 deg/s gyro bias, weaker motors with a larger
deadband, blurrier sensors, and cones moved by 4-5 cm.

What the simulator does not cover: ambient IR, uneven floor and tape
reflectance, sensor height changes, wheel slip in pivots, cone shapes other
than circles, real comparator hysteresis, and the BLE link. Check each one on
the bench.

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
