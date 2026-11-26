// Line following and track navigation for the MECH 368 course: a PD line
// follower plus a state machine for 90-degree corners, lost line, end of
// track and cone detours (handout Figure 5). Pure logic: the sketch reads the
// sensors, integrates heading and distance, and turns the returned twist into
// wheel speeds through the robot's single safety-gated output path.
//
// The corner and detour sequences follow the pattern of an intersection state
// machine: detect (debounced) -> stop -> advance so the axle sits on the turn
// point -> pivot with gyro feedback -> settle -> re-arm after some distance.
// Here the pivot ends when the line is centred under the sensors rather than
// at a fixed gyro angle, so a corner that is not exactly 90 degrees still
// works, and the gyro only bounds the search.
#ifndef MECH368_LINE_NAVIGATOR_H
#define MECH368_LINE_NAVIGATOR_H

#include "DriveMath.h"
#include "LineSensors.h"

namespace mech368 {
namespace line {

// PD (optionally PID) steering on the line position, with speed scheduling:
// the robot slows down as the error grows (cuts the overshoot on bends).
class LineFollower {
 public:
  float kp = 90.0f;           // rad/s per m of line offset
  float kd = 2.0f;            // rad/s per (m/s) of offset rate
  float ki = 0.0f;            // rad/s per (m*s); keep 0 unless the robot drifts
  float integralLimit = 0.02f;
  float derivativeFilter = 0.3f;  // 0..1, weight of the newest derivative sample
  float maxOmega = 6.0f;      // rad/s
  float cruise = 0.25f;       // m/s on a centred line
  float minSpeed = 0.10f;     // m/s at large error
  float fullError = 0.03f;    // m: error at which the speed reaches minSpeed

  void reset() {
    primed_ = false;
    derivative_ = integral_ = 0.0f;
  }
  Twist update(float error, float dt) {
    if (dt <= 0.0f) dt = 1e-3f;
    if (!primed_) {
      last_ = error;
      primed_ = true;
    }
    // Derivative of the measurement, low-pass filtered: digital sensors move
    // the position in steps, and an unfiltered derivative turns each step
    // into a spike.
    const float raw = (error - last_) / dt;
    last_ = error;
    derivative_ += derivativeFilter * (raw - derivative_);
    if (ki != 0.0f) integral_ = clampf(integral_ + error * dt, -integralLimit, integralLimit);
    const float omega = clampf(kp * error + kd * derivative_ + ki * integral_, -maxOmega, maxOmega);
    const float slow = clampf(fabsf(error) / fullError, 0.0f, 1.0f);
    return Twist(cruise - (cruise - minSpeed) * slow, omega);
  }

 private:
  bool primed_ = false;
  float last_ = 0.0f, derivative_ = 0.0f, integral_ = 0.0f;
};

enum class NavState : uint8_t {
  Idle,               // stopped, waiting for start()
  Follow,             // PD on the line
  CornerAdvance,      // wide black seen: drive the axle over the corner
  CornerPivot,        // pivot toward the corner until the line is centred
  Settle,             // brief stop before following again
  Search,             // line lost to one side: sweep for it
  EndCheck,           // line lost straight ahead: creep to confirm the end
  AvoidBrake,         // obstacle: stop
  AvoidTurnOut,       // pivot away from the obstacle
  AvoidLegOut,        // drive out to the side
  AvoidRetreat,       // blocked on the way out: back up to try the other side
  AvoidTurnParallel,  // turn parallel to the original line
  AvoidPass,          // drive past the obstacle
  AvoidTurnIn,        // turn back toward the line
  AvoidSeek,          // drive until the line is found
  RejoinAdvance,      // drive the axle onto the line
  RejoinPivot,        // pivot back onto the line
  Finished,           // end of track reached
  Lost,               // line not found: needs a human
  Blocked,            // obstacle while detouring: needs a human
};

inline const char* navStateName(NavState s) {
  switch (s) {
    case NavState::Idle: return "idle";
    case NavState::Follow: return "follow";
    case NavState::CornerAdvance: return "corner_advance";
    case NavState::CornerPivot: return "corner_pivot";
    case NavState::Settle: return "settle";
    case NavState::Search: return "search";
    case NavState::EndCheck: return "end_check";
    case NavState::AvoidBrake: return "avoid_brake";
    case NavState::AvoidTurnOut: return "avoid_turn_out";
    case NavState::AvoidLegOut: return "avoid_leg_out";
    case NavState::AvoidRetreat: return "avoid_retreat";
    case NavState::AvoidTurnParallel: return "avoid_turn_parallel";
    case NavState::AvoidPass: return "avoid_pass";
    case NavState::AvoidTurnIn: return "avoid_turn_in";
    case NavState::AvoidSeek: return "avoid_seek";
    case NavState::RejoinAdvance: return "rejoin_advance";
    case NavState::RejoinPivot: return "rejoin_pivot";
    case NavState::Finished: return "finished";
    case NavState::Lost: return "lost";
    case NavState::Blocked: return "blocked";
  }
  return "?";
}

// Every number is a starting point for tuning on the real robot and track.
struct NavConfig {
  // Debounce: consecutive control steps a condition must hold.
  int lostSamples = 4, wideSamples = 2, obstacleSamples = 3, foundSamples = 2;
  // Corners: "wide black" = at least wideCount black sensors including one
  // outer sensor. A 5 cm tape can only cover the inner sensors; a corner
  // (perpendicular tape) darkens one outer sensor as well.
  int wideCount = 3;
  float sensorAhead = 0.060f;      // m from the wheel axle to the sensor bar
  // Driven after the corner is seen, so the axle stops on the corner: the bar
  // sees the near edge of the crossing tape, whose centre is half a tape
  // width further on (sensorAhead + 2.5 cm for the 5 cm course tape).
  float cornerAdvance = 0.085f;
  float cornerRearm = 0.10f;       // m of following before another corner can trigger
  // Turn of each corner (rad). After a corner the line's direction is taken
  // as the heading before it +- cornerAngle (a cleaner reference for a detour
  // that starts right after the corner than the heading the pivot stopped
  // at). Set 0 to use the measured heading instead (tracks without square
  // corners).
  float cornerAngle = degToRad(90);
  float approachSpeed = 0.15f;     // m/s for advances, creeping and detours
  float pivotRate = 3.0f;          // rad/s while pivoting to find the line
  float pivotSlowRate = 1.8f;      // rad/s once a sensor sees black
  float minPivot = degToRad(45);   // turn at least this much before accepting the line
  float maxPivot = degToRad(135);  // give up the corner pivot and search instead
  float centredTolerance = 0.012f; // m: line "centred" under the sensor bar
  float settleMs = 120.0f;
  // Lost line.
  // Lost while |last position| > sideTolerance -> the line went off to that
  // side: search there. Otherwise the line ended ahead. Set it between the
  // inner and outer sensor offsets: when the tape end passes under the bar
  // one inner sensor often leaves last, which must still count as "ahead".
  float sideTolerance = 0.022f;
  float searchAngle = degToRad(120);
  float endCreep = 0.04f;          // m driven to confirm the end of the track
  // Heading control for detours.
  float headingGain = 5.0f;        // rad/s per rad
  float headingTolerance = degToRad(3);
  float maxTurnRate = 3.5f;        // rad/s
  float minTurnRate = 0.8f;        // rad/s, overcomes static friction
  // Obstacle detour (handout Figure 5). Lateral clearance from the line to the
  // robot centre while passing, length driven parallel to the line, and the
  // turn out / turn in angles.
  float brakeMs = 150.0f;
  float turnOut = degToRad(60);
  // sideOffset: cone on the tape (handout Figure 5) needs cone radius (8 cm)
  // + robot radius (7.5 cm) + margin. passLength: detection range + cone
  // diameter + robot radius, minus what the leg out already covers.
  float sideOffset = 0.18f;
  float passLength = 0.32f;
  float turnIn = degToRad(60);
  float seekMax = 0.35f;           // m driven toward the line before giving up
  float rejoinAdvance = 0.085f;    // m driven after the line is found (as cornerAdvance)
  float minRejoinPivot = degToRad(20);
  float obstacleRearm = 0.05f;     // m of following before another detour can start
  int preferredSide = -1;          // detour side when both sensors fire: +1 left, -1 right
};

struct NavInputs {
  float dt = 0.005f;          // s since the previous update
  LineReading line;
  bool obstacleLeft = false;  // front-left obstacle sensor
  bool obstacleRight = false; // front-right obstacle sensor
  float heading = 0.0f;       // rad, gyro-integrated (any offset)
  float distance = 0.0f;      // m, odometry path length (signed)
};

class LineNavigator {
 public:
  NavConfig cfg;
  LineFollower follower;

  void start(const NavInputs& in) {
    clearCounters();
    follower.reset();
    cornerMark_ = obstacleMark_ = in.distance - 1.0f;  // armed
    trackHeading_ = in.heading;
    if (in.line.onLine) enter(NavState::Follow, in, "start on the line");
    else beginSearch(in, in.line.lastSeen >= 0.0f ? 1 : -1, "start off the line");
  }
  void stop() { state_ = NavState::Idle; reason_ = "stopped"; ++transitions_; }

  NavState state() const { return state_; }
  const char* reason() const { return reason_; }
  uint32_t transitions() const { return transitions_; }  // bumps on every state change
  int detours() const { return detours_; }
  int corners() const { return corners_; }
  bool running() const {
    return state_ != NavState::Idle && state_ != NavState::Finished && state_ != NavState::Lost &&
           state_ != NavState::Blocked;
  }

  Twist update(const NavInputs& in) {
    countConditions(in);
    switch (state_) {
      case NavState::Idle:
      case NavState::Finished:
      case NavState::Lost:
      case NavState::Blocked:
        return Twist();
      case NavState::Follow: return follow(in);
      case NavState::CornerAdvance:
        if (travelled(in) >= cfg.cornerAdvance) {
          enter(NavState::CornerPivot, in, side_ > 0 ? "pivot left" : "pivot right");
          return Twist();
        }
        return Twist(cfg.approachSpeed, holdHeading(in, holdHeading_));
      case NavState::CornerPivot: {
        const float turned = turnedSince(in);
        if (turned >= cfg.minPivot && centred(in)) {
          ++corners_;
          trackHeading_ = cfg.cornerAngle > 0.0f ? wrapAngle(startHeading_ + side_ * cfg.cornerAngle) : in.heading;
          cornerMark_ = in.distance;
          return settle(in, "corner done");
        }
        if (turned >= cfg.maxPivot) {
          beginSearch(in, -side_, "corner pivot overran, sweeping back");
          return Twist();
        }
        return Twist(0.0f, side_ * (in.line.onLine ? cfg.pivotSlowRate : cfg.pivotRate));
      }
      case NavState::Settle:
        if (elapsedMs(in) >= cfg.settleMs) {
          follower.reset();
          enter(NavState::Follow, in, "following");
        }
        return Twist();
      case NavState::Search: return search(in);
      case NavState::EndCheck:
        if (found_ >= cfg.foundSamples) {
          follower.reset();
          enter(NavState::Follow, in, "line continues");
          return Twist();
        }
        if (travelled(in) >= cfg.endCreep) {
          enter(NavState::Finished, in, "end of track");
          return Twist();
        }
        return Twist(cfg.approachSpeed, holdHeading(in, holdHeading_));
      default: return avoid(in);
    }
  }

  // Heading of the line being followed (low-pass of the gyro heading).
  float trackHeading() const { return trackHeading_; }

 private:
  Twist follow(const NavInputs& in) {
    // Low-pass the heading along the line: the detour uses it as "the line's
    // direction", which is steadier than the instantaneous heading.
    trackHeading_ = wrapAngle(trackHeading_ + 0.02f * wrapAngle(in.heading - trackHeading_));
    const bool obstacleArmed = in.distance - obstacleMark_ >= cfg.obstacleRearm;
    if (obstacleArmed && (obstacleL_ >= cfg.obstacleSamples || obstacleR_ >= cfg.obstacleSamples)) {
      int away;
      if (obstacleL_ >= cfg.obstacleSamples && obstacleR_ >= cfg.obstacleSamples) {
        away = cfg.preferredSide;
      } else {
        away = obstacleL_ >= cfg.obstacleSamples ? -1 : 1;  // go around the other side
      }
      side_ = away;
      lineHeading_ = trackHeading_;
      retried_ = false;
      ++detours_;
      enter(NavState::AvoidBrake, in, away > 0 ? "obstacle right: detour left" : "obstacle left: detour right");
      return Twist();
    }
    const bool cornerArmed = in.distance - cornerMark_ >= cfg.cornerRearm;
    if (cornerArmed && wide_ >= cfg.wideSamples) {
      side_ = wideSide_;
      holdHeading_ = in.heading;
      enter(NavState::CornerAdvance, in, side_ > 0 ? "corner left" : "corner right");
      return Twist(cfg.approachSpeed, 0.0f);
    }
    if (lost_ >= cfg.lostSamples) {
      if (fabsf(in.line.lastSeen) > cfg.sideTolerance) {
        beginSearch(in, in.line.lastSeen > 0.0f ? 1 : -1, "line lost to one side");
      } else {
        holdHeading_ = in.heading;
        enter(NavState::EndCheck, in, "line ended ahead");
      }
      return Twist();
    }
    return follower.update(in.line.position, in.dt);
  }

  void beginSearch(const NavInputs& in, int side, const char* why) {
    side_ = side;
    sweepBack_ = false;
    enter(NavState::Search, in, why);
  }
  Twist search(const NavInputs& in) {
    if (found_ >= cfg.foundSamples && fabsf(in.line.position) <= 2.0f * cfg.centredTolerance) {
      cornerMark_ = in.distance;
      return settle(in, "line found");
    }
    const float turned = wrapAngle(in.heading - startHeading_) * side_;
    if (!sweepBack_ && turned >= cfg.searchAngle) sweepBack_ = true;
    if (sweepBack_ && turned <= -cfg.searchAngle) {
      enter(NavState::Lost, in, "line not found");
      return Twist();
    }
    const float rate = in.line.onLine ? cfg.pivotSlowRate : cfg.pivotRate;
    return Twist(0.0f, (sweepBack_ ? -side_ : side_) * rate);
  }

  Twist avoid(const NavInputs& in) {
    const bool blocked = obstacleL_ >= cfg.obstacleSamples || obstacleR_ >= cfg.obstacleSamples;
    const bool lineFound = found_ >= cfg.foundSamples;
    switch (state_) {
      case NavState::AvoidBrake:
        if (elapsedMs(in) >= cfg.brakeMs) {
          target_ = wrapAngle(lineHeading_ + side_ * cfg.turnOut);
          enter(NavState::AvoidTurnOut, in, "turning out");
        }
        return Twist();
      case NavState::AvoidTurnOut:
        if (turnTo(in, target_)) enter(NavState::AvoidLegOut, in, "leg out");
        return Twist(0.0f, holdHeading(in, target_));
      case NavState::AvoidLegOut:
        if (blocked) {
          // Something on this side too (a wall, another cone): reverse along
          // the same path and go around the other side, once.
          if (retried_) return block(in);
          retreat_ = travelled(in);
          enter(NavState::AvoidRetreat, in, "blocked on the way out: backing up");
          return Twist();
        }
        if (travelled(in) >= cfg.sideOffset / sinf(cfg.turnOut)) {
          target_ = lineHeading_;
          enter(NavState::AvoidTurnParallel, in, "turning parallel");
          return Twist();
        }
        return Twist(cfg.approachSpeed, holdHeading(in, target_));
      case NavState::AvoidRetreat:
        if (travelled(in) >= retreat_) {
          retried_ = true;
          side_ = -side_;
          target_ = wrapAngle(lineHeading_ + side_ * cfg.turnOut);
          enter(NavState::AvoidTurnOut, in, side_ > 0 ? "retrying on the left" : "retrying on the right");
          return Twist();
        }
        return Twist(-cfg.approachSpeed, holdHeading(in, target_));
      case NavState::AvoidTurnParallel:
        if (turnTo(in, target_)) enter(NavState::AvoidPass, in, "passing");
        return Twist(0.0f, holdHeading(in, target_));
      case NavState::AvoidPass:
        if (lineFound) return rejoin(in);
        if (blocked) return block(in);
        if (travelled(in) >= cfg.passLength) {
          target_ = wrapAngle(lineHeading_ - side_ * cfg.turnIn);
          enter(NavState::AvoidTurnIn, in, "turning back in");
          return Twist();
        }
        return Twist(cfg.approachSpeed, holdHeading(in, target_));
      case NavState::AvoidTurnIn:
        if (turnTo(in, target_)) enter(NavState::AvoidSeek, in, "seeking the line");
        return Twist(0.0f, holdHeading(in, target_));
      case NavState::AvoidSeek:
        if (lineFound) return rejoin(in);
        if (blocked) return block(in);
        if (travelled(in) >= cfg.seekMax) {
          enter(NavState::Lost, in, "line not found after the detour");
          return Twist();
        }
        return Twist(cfg.approachSpeed, holdHeading(in, target_));
      case NavState::RejoinAdvance:
        if (travelled(in) >= cfg.rejoinAdvance) enter(NavState::RejoinPivot, in, "pivoting onto the line");
        return Twist(cfg.approachSpeed, holdHeading(in, holdHeading_));
      case NavState::RejoinPivot: {
        // Turn toward the side we swerved to: that points the robot along the
        // line in the original direction of travel, also when the line found
        // is the next leg of the track.
        const float turned = turnedSince(in);
        if (turned >= cfg.minRejoinPivot && centred(in)) {
          obstacleMark_ = cornerMark_ = in.distance;
          trackHeading_ = in.heading;
          return settle(in, "back on the line");
        }
        if (turned >= cfg.maxPivot) {
          beginSearch(in, -side_, "rejoin pivot overran, sweeping back");
          return Twist();
        }
        return Twist(0.0f, side_ * (in.line.onLine ? cfg.pivotSlowRate : cfg.pivotRate));
      }
      default:
        return Twist();
    }
  }
  Twist rejoin(const NavInputs& in) {
    holdHeading_ = in.heading;
    enter(NavState::RejoinAdvance, in, "line found");
    return Twist(cfg.approachSpeed, 0.0f);
  }
  Twist block(const NavInputs& in) {
    enter(NavState::Blocked, in, "obstacle during the detour");
    return Twist();
  }
  Twist settle(const NavInputs& in, const char* why) {
    enter(NavState::Settle, in, why);
    return Twist();
  }

  // Rotate in place toward target; true once within tolerance.
  bool turnTo(const NavInputs& in, float target) const {
    return fabsf(wrapAngle(target - in.heading)) <= cfg.headingTolerance;
  }
  float holdHeading(const NavInputs& in, float target) const {
    const float err = wrapAngle(target - in.heading);
    float w = clampf(cfg.headingGain * err, -cfg.maxTurnRate, cfg.maxTurnRate);
    if (fabsf(err) > cfg.headingTolerance && fabsf(w) < cfg.minTurnRate) w = signf(err) * cfg.minTurnRate;
    return w;
  }
  bool centred(const NavInputs& in) const {
    return found_ >= cfg.foundSamples && fabsf(in.line.position) <= cfg.centredTolerance;
  }
  float turnedSince(const NavInputs& in) const { return wrapAngle(in.heading - startHeading_) * side_; }
  float travelled(const NavInputs& in) const { return fabsf(in.distance - startDistance_); }
  float elapsedMs(const NavInputs&) const { return elapsed_ * 1000.0f; }

  void countConditions(const NavInputs& in) {
    elapsed_ += in.dt;
    lost_ = in.line.onLine ? 0 : lost_ + 1;
    found_ = in.line.onLine ? found_ + 1 : 0;
    obstacleL_ = in.obstacleLeft ? obstacleL_ + 1 : 0;
    obstacleR_ = in.obstacleRight ? obstacleR_ + 1 : 0;
    const bool wide = in.line.blackCount >= cfg.wideCount && in.line.leftOuterBlack != in.line.rightOuterBlack;
    const int side = in.line.leftOuterBlack ? 1 : -1;
    if (wide && (wide_ == 0 || side == wideSide_)) {
      ++wide_;
      wideSide_ = side;
    } else {
      wide_ = 0;
    }
  }
  void clearCounters() { lost_ = found_ = obstacleL_ = obstacleR_ = wide_ = 0; }
  void enter(NavState next, const NavInputs& in, const char* why) {
    if (next == NavState::CornerAdvance) cornerMark_ = in.distance;
    state_ = next;
    reason_ = why;
    startHeading_ = in.heading;
    startDistance_ = in.distance;
    elapsed_ = 0.0f;
    ++transitions_;
  }

  NavState state_ = NavState::Idle;
  const char* reason_ = "not started";
  uint32_t transitions_ = 0;
  int lost_ = 0, found_ = 0, obstacleL_ = 0, obstacleR_ = 0, wide_ = 0, wideSide_ = 0;
  int side_ = 1;  // +1 left, -1 right
  bool sweepBack_ = false, retried_ = false;
  float retreat_ = 0.0f;
  float startHeading_ = 0.0f, startDistance_ = 0.0f, elapsed_ = 0.0f;
  float holdHeading_ = 0.0f, target_ = 0.0f, lineHeading_ = 0.0f, trackHeading_ = 0.0f;
  float cornerMark_ = -1.0f, obstacleMark_ = -1.0f;
  int detours_ = 0, corners_ = 0;
};

}  // namespace line
}  // namespace mech368
#endif
