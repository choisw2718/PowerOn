#ifndef POWERON_WALL_FOLLOWER_H
#define POWERON_WALL_FOLLOWER_H

// Hardware-independent controller. Distances: mm, angles: degrees (left +).
// No Arduino calls, dynamic allocation, or sensor-driver assumptions here.
#include <math.h>
#include <stdint.h>

namespace wall_follow {

struct Config {
  float corridorMm = 610.0f;
  // Distance from vehicle centre to each outward-facing sensor, NOT body width.
  float leftOffsetMm = 0.0f;
  float rightOffsetMm = 0.0f;
  float minRangeMm = 100.0f; // GP2Y0A21YK0F nominal range: 100..800 mm.
  float maxRangeMm = 800.0f;
  float wallMaxCentreMm = 480.0f;
  float openingCentreMm = 650.0f;
  float widthToleranceMm = 120.0f;
  float approachFrontMm = 600.0f;
  float turnFrontMm = 360.0f;
  float exitFrontMm = 700.0f;
  // Front distance measured at sensor face; does not yet include a bumper offset.
  float stopFrontMm = 150.0f;
  float stopSideMm = 120.0f;
  float kpDegPerMm = 0.08f;
  float kdDegPerMmPerSec = 0.025f;
  float errorFilterSeconds = 0.12f;
  float straightSteerLimitDeg = 18.0f;
  float steerRateDegPerSec = 20.0f;
  float settleSteerRateDegPerSec = 90.0f; // Re-centre promptly after a corner.
  float turnSteerDeg = 30.0f;
  // PWM percentages, NOT measured vehicle speed or motor voltage.
  float straightDuty = 60.0f;
  float approachDuty = 58.0f;
  float turnDuty = 58.0f;
  float exitErrorMm = 100.0f;
  float exitErrorRateMmPerSec = 150.0f;
  uint32_t sampleTimeoutMs = 300;
  uint32_t minTurnMs = 400;
  uint32_t maxTurnMs = 4500;
  uint32_t settleMs = 500;
  uint8_t confirmSamples = 3;
};

struct Sample {
  float leftMm = 0.0f;
  float frontMm = 0.0f;
  float rightMm = 0.0f;
  uint32_t capturedMs = 0;
  bool available = false;
  bool leftValid = false;
  bool frontValid = false;
  bool rightValid = false;
};

enum class State : uint8_t { Off, Follow, Approach, TurnLeft, TurnRight, Settle, Fault };
enum class Fault : uint8_t {
  None, BadConfig, Stale, InvalidRange, FrontClose, SideClose,
  NoWall, WidthMismatch, CornerUnclear, TurnTimeout
};

inline const char *stateName(State state) {
  switch (state) {
    case State::Off: return "OFF";
    case State::Follow: return "FOLLOW";
    case State::Approach: return "APPROACH";
    case State::TurnLeft: return "TURN_LEFT";
    case State::TurnRight: return "TURN_RIGHT";
    case State::Settle: return "SETTLE";
    default: return "FAULT";
  }
}

inline const char *faultName(Fault fault) {
  switch (fault) {
    case Fault::None: return "none";
    case Fault::BadConfig: return "bad_config";
    case Fault::Stale: return "stale_ranges";
    case Fault::InvalidRange: return "invalid_range";
    case Fault::FrontClose: return "front_too_close";
    case Fault::SideClose: return "side_too_close";
    case Fault::NoWall: return "both_walls_lost";
    case Fault::WidthMismatch: return "width_mismatch";
    case Fault::CornerUnclear: return "corner_unclear";
    default: return "turn_timeout";
  }
}

struct Output {
  State state = State::Off;
  Fault fault = Fault::None;
  float duty = 0.0f;
  float steerDeg = 0.0f;
  float errorMm = 0.0f;
  // 0: no reference, 1: left wall, 2: right wall, 3: both walls.
  uint8_t wallReference = 0;
  uint16_t turnsCompleted = 0;
};

class Controller {
 public:
  explicit Controller(const Config &config = Config()) : config_(config) {}
  const Output &output() const { return output_; }

  void stop() { output_ = Output(); }

  bool start(const Sample &sample, uint32_t now) {
    stop();
    if (!validConfig()) { fail(Fault::BadConfig); return false; }
    output_.state = State::Follow;
    havePreviousSample_ = false;
    reference_ = 0;
    cornerCandidate_ = 0;
    cornerCount_ = exitCount_ = 0;
    haveExitError_ = false;
    update(sample, now);
    return output_.state != State::Fault;
  }

  void update(const Sample &sample, uint32_t now) {
    if (output_.state == State::Off || output_.state == State::Fault) return;
    if (!sample.available || now - sample.capturedMs > config_.sampleTimeoutMs) {
      fail(Fault::Stale); return;
    }
    if (!sample.leftValid || !sample.frontValid || !sample.rightValid ||
        !validRange(sample.leftMm) || !validRange(sample.frontMm) ||
        !validRange(sample.rightMm)) {
      fail(Fault::InvalidRange); return;
    }
    // Emergency thresholds use raw sensor distances, even during a turn.
    if (sample.frontMm <= config_.stopFrontMm) { fail(Fault::FrontClose); return; }
    if (sample.leftMm <= config_.stopSideMm || sample.rightMm <= config_.stopSideMm) {
      fail(Fault::SideClose); return;
    }
    const bool turning = output_.state == State::TurnLeft || output_.state == State::TurnRight;
    if (turning && now - stateStartedMs_ >= config_.maxTurnMs) {
      fail(Fault::TurnTimeout); return;
    }
    // Repeated loop ticks must not count the same sample as extra evidence.
    if (havePreviousSample_ && sample.capturedMs == previousSampleMs_) return;
    const float dt = havePreviousSample_
        ? static_cast<float>(sample.capturedMs - previousSampleMs_) * 0.001f : 0.05f;
    previousSampleMs_ = sample.capturedMs;
    havePreviousSample_ = true;
    const float left = sample.leftMm + config_.leftOffsetMm;
    const float right = sample.rightMm + config_.rightOffsetMm;
    const bool leftWall = left < config_.wallMaxCentreMm;
    const bool rightWall = right < config_.wallMaxCentreMm;
    const bool widthOK = leftWall && rightWall &&
        fabsf(left + right - config_.corridorMm) <= config_.widthToleranceMm;

    if (turning) {
      // Do not apply straight-wall PD to oblique ranges during rotation.
      const float error = (left - right) * 0.5f;
      const float rate = haveExitError_ ? (error - exitError_) / dt : 0.0f;
      const bool exitOK = now - stateStartedMs_ >= config_.minTurnMs &&
          sample.frontMm >= config_.exitFrontMm && widthOK &&
          fabsf(error) <= config_.exitErrorMm && haveExitError_ &&
          fabsf(rate) <= config_.exitErrorRateMmPerSec;
      exitCount_ = exitOK ? static_cast<uint8_t>(exitCount_ + 1) : 0;
      haveExitError_ = widthOK;
      exitError_ = error;
      if (exitCount_ >= config_.confirmSamples) {
        ++output_.turnsCompleted;
        output_.state = State::Settle;
        stateStartedMs_ = now;
        reference_ = 0;  // Reset derivative on return to wall control.
      } else {
        output_.duty = config_.turnDuty;
        return;
      }
    }

    // Open side alone never starts a turn: only a blocked front can do so.
    if (sample.frontMm <= config_.turnFrontMm) {
      const bool openLeft = left >= config_.openingCentreMm;
      const bool openRight = right >= config_.openingCentreMm;
      const int8_t candidate = openLeft != openRight ? (openLeft ? 1 : -1) : 0;
      if (candidate != cornerCandidate_) cornerCount_ = 0;
      cornerCandidate_ = candidate;
      if (++cornerCount_ >= config_.confirmSamples) {
        if (candidate == 0) { fail(Fault::CornerUnclear); return; }
        output_.state = candidate > 0 ? State::TurnLeft : State::TurnRight;
        stateStartedMs_ = now;
        output_.duty = config_.turnDuty;
        output_.steerDeg = candidate * config_.turnSteerDeg;
        output_.wallReference = 0;
        exitCount_ = 0;
        haveExitError_ = false;
        return;
      }
      // Confirm geometry while stationary; never creep into an unknown corner.
      output_.state = State::Approach;
      output_.duty = 0.0f;
      output_.steerDeg = 0.0f;
      reference_ = 0;
      return;
    }
    cornerCount_ = 0;
    cornerCandidate_ = 0;
    if (!leftWall && !rightWall) { fail(Fault::NoWall); return; }
    if (leftWall && rightWall && !widthOK) { fail(Fault::WidthMismatch); return; }
    uint8_t reference = leftWall ? (rightWall ? 3 : 1) : 2;
    const float target = config_.corridorMm * 0.5f;
    const float error = reference == 3 ? (left - right) * 0.5f
        : (reference == 1 ? left - target : target - right);
    float rate = 0.0f;
    if (reference != reference_) {
      filteredError_ = error;
    } else {
      const float filtered = filteredError_ + dt / (config_.errorFilterSeconds + dt) *
          (error - filteredError_);
      rate = (filtered - filteredError_) / dt;
      filteredError_ = filtered;
    }
    reference_ = reference;
    output_.errorMm = error;
    output_.wallReference = reference;
    const float steer = clamp(config_.kpDegPerMm * filteredError_ +
        config_.kdDegPerMmPerSec * rate,
        -config_.straightSteerLimitDeg, config_.straightSteerLimitDeg);
    const float step = (output_.state == State::Settle ? config_.settleSteerRateDegPerSec :
        config_.steerRateDegPerSec) * dt;
    output_.steerDeg += clamp(steer - output_.steerDeg, -step, step);
    if (output_.state == State::Settle && (now - stateStartedMs_ < config_.settleMs ||
        fabsf(output_.steerDeg - steer) > 0.1f)) {
      output_.duty = config_.approachDuty;
    } else {
      output_.state = sample.frontMm < config_.approachFrontMm ? State::Approach : State::Follow;
      output_.duty = output_.state == State::Approach ? config_.approachDuty : config_.straightDuty;
    }
  }

 private:
  Config config_;
  Output output_;
  bool havePreviousSample_ = false;
  uint32_t previousSampleMs_ = 0;
  uint32_t stateStartedMs_ = 0;
  uint8_t reference_ = 0;
  float filteredError_ = 0.0f;
  int8_t cornerCandidate_ = 0;
  uint8_t cornerCount_ = 0;
  uint8_t exitCount_ = 0;
  float exitError_ = 0.0f;
  bool haveExitError_ = false;

  static float clamp(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
  }
  bool validRange(float mm) const {
    return isfinite(mm) && mm >= config_.minRangeMm && mm <= config_.maxRangeMm;
  }
  bool validConfig() const {
    const float values[] = {config_.corridorMm, config_.leftOffsetMm, config_.rightOffsetMm,
        config_.minRangeMm, config_.maxRangeMm, config_.wallMaxCentreMm,
        config_.openingCentreMm, config_.widthToleranceMm, config_.approachFrontMm,
        config_.turnFrontMm, config_.exitFrontMm, config_.stopFrontMm, config_.stopSideMm,
        config_.kpDegPerMm, config_.kdDegPerMmPerSec, config_.errorFilterSeconds,
        config_.straightSteerLimitDeg, config_.steerRateDegPerSec,
        config_.settleSteerRateDegPerSec, config_.turnSteerDeg,
        config_.straightDuty, config_.approachDuty, config_.turnDuty,
        config_.exitErrorMm, config_.exitErrorRateMmPerSec};
    for (float value : values) if (!isfinite(value)) return false;
    return config_.leftOffsetMm >= 0 && config_.rightOffsetMm >= 0 &&
        config_.leftOffsetMm < config_.corridorMm * 0.5f &&
        config_.rightOffsetMm < config_.corridorMm * 0.5f &&
        config_.minRangeMm > 0 && config_.maxRangeMm > config_.openingCentreMm &&
        config_.wallMaxCentreMm > config_.corridorMm * 0.5f &&
        config_.openingCentreMm > config_.wallMaxCentreMm && config_.widthToleranceMm > 0 &&
        config_.stopFrontMm >= config_.minRangeMm && config_.stopSideMm >= config_.minRangeMm &&
        config_.stopFrontMm < config_.turnFrontMm && config_.turnFrontMm < config_.approachFrontMm &&
        config_.approachFrontMm < config_.exitFrontMm && config_.exitFrontMm <= config_.maxRangeMm &&
        config_.kpDegPerMm >= 0 && config_.kdDegPerMmPerSec >= 0 && config_.errorFilterSeconds >= 0 &&
        config_.straightSteerLimitDeg > 0 && config_.straightSteerLimitDeg <= config_.turnSteerDeg &&
        config_.turnSteerDeg <= 30 && config_.steerRateDegPerSec > 0 &&
        config_.settleSteerRateDegPerSec > 0 &&
        config_.straightDuty > 0 && config_.straightDuty <= 100 &&
        config_.approachDuty > 0 && config_.approachDuty <= config_.straightDuty &&
        config_.turnDuty > 0 && config_.turnDuty <= config_.approachDuty &&
        config_.exitErrorMm > 0 && config_.exitErrorRateMmPerSec > 0 &&
        config_.sampleTimeoutMs > 0 && config_.maxTurnMs > config_.minTurnMs &&
        config_.confirmSamples > 0;
  }
  void fail(Fault reason) {
    output_.state = State::Fault;
    output_.fault = reason;
    output_.duty = output_.steerDeg = 0.0f;
    output_.wallReference = 0;
  }
};

}  // namespace wall_follow
#endif
