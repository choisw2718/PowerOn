#include <Arduino.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "PowerOnConfig.h"

namespace
{

const float kPi = 3.14159265f;
const float kDegToRad = 0.0174532925f;
const float kRadToDeg = 57.2957795f;
const float kStraightRadiusM = 1.0e6f;
const float kMinRadiusMarginM = 0.001f;
const float kMotionEpsilon = 0.001f;
const uint8_t kEncoderFaultLeftNoSignal = 0x01U;
const uint8_t kEncoderFaultRightNoSignal = 0x02U;
const uint8_t kEncoderFaultLeftImplausible = 0x04U;
const uint8_t kEncoderFaultRightImplausible = 0x08U;

const uint16_t kMotorPwmTop =
    static_cast<uint16_t>((F_CPU / POWERON_MOTOR_PWM_HZ) - 1UL);

/* Timer2 runs at 16 MHz / 64 = 250 kHz, or one tick every 4 us. */
const uint16_t kServoTimerTickUs = 4U;
const uint16_t kServoFrameTicks =
    static_cast<uint16_t>(POWERON_SERVO_FRAME_US / kServoTimerTickUs);
const uint16_t kMaxServoLeftTicks = static_cast<uint16_t>(
    (POWERON_SERVO_LEFT_SAFE_MAX_US + (kServoTimerTickUs / 2U)) /
    kServoTimerTickUs);
const uint16_t kMaxServoRightTicks = static_cast<uint16_t>(
    (POWERON_SERVO_RIGHT_SAFE_MAX_US + (kServoTimerTickUs / 2U)) /
    kServoTimerTickUs);
const uint8_t kLeftServoMask = _BV(PD5);
const uint8_t kRightServoMask = _BV(PD6);

static_assert(POWERON_LEFT_MOTOR_PWM_PIN == 9 &&
                  POWERON_RIGHT_MOTOR_PWM_PIN == 10,
              "Timer1 motor PWM requires Uno pins D9 and D10");
static_assert(POWERON_LEFT_SERVO_PIN == 5 &&
                  POWERON_RIGHT_SERVO_PIN == 6,
              "Timer2 servo scheduler directly drives Uno pins D5 and D6");
static_assert(POWERON_LEFT_ENCODER_A_PIN == A0 &&
                  POWERON_LEFT_ENCODER_B_PIN == A1 &&
                  POWERON_RIGHT_ENCODER_A_PIN == A2 &&
                  POWERON_RIGHT_ENCODER_B_PIN == A3,
              "The encoder ISR requires Uno pins A0 through A3");
static_assert(kMotorPwmTop == 799U, "20 kHz Timer1 TOP must be 799 at 16 MHz");
static_assert((POWERON_SERVO_FRAME_US % kServoTimerTickUs) == 0U,
              "Servo frame must be an integer number of Timer2 ticks");
static_assert(static_cast<uint32_t>(kMaxServoLeftTicks) +
                      static_cast<uint32_t>(kMaxServoRightTicks) <
                  static_cast<uint32_t>(kServoFrameTicks),
              "Both servo pulses must fit inside one frame");
static_assert(POWERON_SERVO_CHANNELS_CROSSED == 0 ||
                  POWERON_SERVO_CHANNELS_CROSSED == 1,
              "Servo channel crossing flag must be 0 or 1");
static_assert(POWERON_LEFT_ENCODER_FORWARD_SIGN == 1 ||
                  POWERON_LEFT_ENCODER_FORWARD_SIGN == -1,
              "Left encoder forward sign must be +1 or -1");
static_assert(POWERON_RIGHT_ENCODER_FORWARD_SIGN == 1 ||
                  POWERON_RIGHT_ENCODER_FORWARD_SIGN == -1,
              "Right encoder forward sign must be +1 or -1");
static_assert(POWERON_ENCODER_COUNTS_PER_MOTOR_REV > 0.0f,
              "Encoder counts per motor revolution must be positive");
static_assert(POWERON_GEAR_RATIO_VERIFIED == 0 ||
                  POWERON_GEAR_RATIO_VERIFIED == 1,
              "Gear-ratio verification flag must be 0 or 1");
static_assert(POWERON_ENCODER_COUNTS_VERIFIED == 0 ||
                  POWERON_ENCODER_COUNTS_VERIFIED == 1,
              "Encoder-count verification flag must be 0 or 1");
static_assert(POWERON_SPEED_PID_TUNED == 0 || POWERON_SPEED_PID_TUNED == 1,
              "PID-tuned flag must be 0 or 1");
static_assert(POWERON_ENCODER_SAMPLE_PERIOD_MS >= POWERON_CONTROL_PERIOD_MS,
              "Encoder sample period must not be shorter than control period");
static_assert(POWERON_ENCODER_BAD_SAMPLE_LIMIT > 0U,
              "At least one implausible sample must be required for a fault");
static_assert(POWERON_MIN_WHEEL_STEER_DEG < 0.0f &&
                  POWERON_MIN_WHEEL_STEER_DEG > -90.0f &&
                  POWERON_MAX_WHEEL_STEER_DEG > 0.0f &&
                  POWERON_MAX_WHEEL_STEER_DEG < 90.0f,
              "Wheel steering limits must straddle zero and stay below 90 deg");
static_assert(POWERON_STEER_STEP_DEG > 0.0f &&
                  POWERON_STEER_STEP_DEG <= 5.0f,
              "Steering key step must be greater than 0 and at most 5 deg");

const uint32_t kUsedPinMask =
    (1UL << POWERON_LEFT_SERVO_PIN) |
    (1UL << POWERON_RIGHT_SERVO_PIN) |
    (1UL << POWERON_LEFT_MOTOR_PWM_PIN) |
    (1UL << POWERON_RIGHT_MOTOR_PWM_PIN) |
    (1UL << POWERON_LEFT_MOTOR_IN1_PIN) |
    (1UL << POWERON_LEFT_MOTOR_IN2_PIN) |
    (1UL << POWERON_LEFT_MOTOR_EN_PIN) |
    (1UL << POWERON_RIGHT_MOTOR_IN1_PIN) |
    (1UL << POWERON_RIGHT_MOTOR_IN2_PIN) |
    (1UL << POWERON_RIGHT_MOTOR_EN_PIN) |
    (1UL << POWERON_LEFT_ENCODER_A_PIN) |
    (1UL << POWERON_LEFT_ENCODER_B_PIN) |
    (1UL << POWERON_RIGHT_ENCODER_A_PIN) |
    (1UL << POWERON_RIGHT_ENCODER_B_PIN);

const uint32_t kUsedPinSum =
    (1UL << POWERON_LEFT_SERVO_PIN) +
    (1UL << POWERON_RIGHT_SERVO_PIN) +
    (1UL << POWERON_LEFT_MOTOR_PWM_PIN) +
    (1UL << POWERON_RIGHT_MOTOR_PWM_PIN) +
    (1UL << POWERON_LEFT_MOTOR_IN1_PIN) +
    (1UL << POWERON_LEFT_MOTOR_IN2_PIN) +
    (1UL << POWERON_LEFT_MOTOR_EN_PIN) +
    (1UL << POWERON_RIGHT_MOTOR_IN1_PIN) +
    (1UL << POWERON_RIGHT_MOTOR_IN2_PIN) +
    (1UL << POWERON_RIGHT_MOTOR_EN_PIN) +
    (1UL << POWERON_LEFT_ENCODER_A_PIN) +
    (1UL << POWERON_LEFT_ENCODER_B_PIN) +
    (1UL << POWERON_RIGHT_ENCODER_A_PIN) +
    (1UL << POWERON_RIGHT_ENCODER_B_PIN);

static_assert(kUsedPinMask == kUsedPinSum, "Every configured I/O pin must be unique");

enum MotorSide : uint8_t
{
  MOTOR_LEFT = 0,
  MOTOR_RIGHT = 1
};

enum ServoEvent : uint8_t
{
  SERVO_EVENT_LEFT_FALL = 0,
  SERVO_EVENT_RIGHT_FALL = 1,
  SERVO_EVENT_FRAME_START = 2
};

enum DriveMode : uint8_t
{
  DRIVE_MODE_VEHICLE_SPEED = 0,
  DRIVE_MODE_WHEEL_RPM = 1,
  DRIVE_MODE_OPEN_LOOP = 2
};

enum SerialKeySequenceState : uint8_t
{
  SERIAL_KEY_IDLE = 0,
  SERIAL_KEY_ESCAPE = 1,
  SERIAL_KEY_ANSI = 2,
  SERIAL_KEY_WINDOWS = 3
};

struct PidController
{
  float kp;
  float ki;
  float kd;
  float integral;
  float previousMeasurement;
  bool first;
};

struct VehicleState
{
  float targetSpeedMps;
  float appliedCenterSpeedMps;
  float measuredSpeedMps;
  float steeringDeg;
  float turnRadiusM;
  float frontLeftSteerDeg;
  float frontRightSteerDeg;
  float targetLeftWheelMps;
  float targetRightWheelMps;
  float targetLeftMotorRpm;
  float targetRightMotorRpm;
  float measuredLeftMotorRpm;
  float measuredRightMotorRpm;
  float leftDutyPercent;
  float rightDutyPercent;
};

struct EncoderRuntime
{
  int32_t windowDelta;
  int32_t lastControlDelta;
  int32_t lastWindowDelta;
  float rawMotorRpm;
  uint32_t validTransitions;
  uint32_t invalidTransitions;
  uint32_t channelAChanges;
  uint32_t channelBChanges;
  uint32_t rejectedSamples;
  uint32_t lastProgressMs;
  uint8_t badSampleStreak;
  bool faultMonitorActive;
  bool monitoredForward;
};

struct EncoderIsrSnapshot
{
  int32_t leftDelta;
  int32_t rightDelta;
  uint16_t leftValid;
  uint16_t rightValid;
  uint16_t leftInvalid;
  uint16_t rightInvalid;
  uint16_t leftAChanges;
  uint16_t leftBChanges;
  uint16_t rightAChanges;
  uint16_t rightBChanges;
};

VehicleState vehicle = {};
PidController leftPid = {
    POWERON_PID_KP, POWERON_PID_KI, POWERON_PID_KD, 0.0f, 0.0f, true};
PidController rightPid = {
    POWERON_PID_KP, POWERON_PID_KI, POWERON_PID_KD, 0.0f, 0.0f, true};
EncoderRuntime leftEncoder = {};
EncoderRuntime rightEncoder = {};

volatile int32_t leftEncoderDelta = 0;
volatile int32_t rightEncoderDelta = 0;
volatile uint16_t leftEncoderValid = 0;
volatile uint16_t rightEncoderValid = 0;
volatile uint16_t leftEncoderInvalid = 0;
volatile uint16_t rightEncoderInvalid = 0;
volatile uint16_t leftEncoderAChanges = 0;
volatile uint16_t leftEncoderBChanges = 0;
volatile uint16_t rightEncoderAChanges = 0;
volatile uint16_t rightEncoderBChanges = 0;
volatile uint8_t previousEncoderBits = 0;
int32_t leftEncoderTotal = 0;
int32_t rightEncoderTotal = 0;
uint32_t encoderWindowElapsedMs = 0;

volatile uint16_t pendingServoLeftTicks =
    static_cast<uint16_t>((static_cast<uint16_t>(POWERON_SERVO_LEFT_NEUTRAL_US) +
                           (kServoTimerTickUs / 2U)) /
                          kServoTimerTickUs);
volatile uint16_t pendingServoRightTicks =
    static_cast<uint16_t>((static_cast<uint16_t>(POWERON_SERVO_RIGHT_NEUTRAL_US) +
                           (kServoTimerTickUs / 2U)) /
                          kServoTimerTickUs);
volatile uint16_t servoDelayRemainingTicks = 0;
uint16_t activeServoLeftTicks = 0;
uint16_t activeServoRightTicks = 0;
ServoEvent servoEvent = SERVO_EVENT_FRAME_START;

bool leftMotorForward = true;
bool rightMotorForward = true;
bool leftMotorEnabled = false;
bool rightMotorEnabled = false;
bool leftDirectionPauseActive = false;
bool rightDirectionPauseActive = false;
DriveMode driveMode = DRIVE_MODE_VEHICLE_SPEED;
bool motionWatchdogArmed = false;
uint8_t encoderFaultFlags = 0;
uint32_t leftDirectionPauseStartedMs = 0;
uint32_t rightDirectionPauseStartedMs = 0;
uint32_t lastMotionCommandMs = 0;
uint32_t commandTimeoutMs = POWERON_DEFAULT_TIMEOUT_MS;
uint32_t lastControlMs = 0;
uint32_t commandCount = 0;
uint32_t rejectedCommandCount = 0;
uint32_t timeoutStopCount = 0;

char serialLine[80];
uint8_t serialLineLength = 0;
bool serialInLine = false;
bool serialDiscardUntilEol = false;
SerialKeySequenceState serialKeySequenceState = SERIAL_KEY_IDLE;

float clampFloat(float value, float low, float high)
{
  if (value < low)
  {
    return low;
  }
  if (value > high)
  {
    return high;
  }
  return value;
}

float absoluteFloat(float value)
{
  return value < 0.0f ? -value : value;
}

int32_t saturatingAdd(int32_t value, int32_t delta)
{
  if (delta > 0 && value > INT32_MAX - delta)
  {
    return INT32_MAX;
  }
  if (delta < 0 && value < INT32_MIN - delta)
  {
    return INT32_MIN;
  }
  return value + delta;
}

void pidReset(PidController &pid)
{
  pid.integral = 0.0f;
  pid.previousMeasurement = 0.0f;
  pid.first = true;
}

float pidUpdate(PidController &pid, float target, float measured, float dtSeconds)
{
  const float error = target - measured;
  float derivative = 0.0f;

  /* Derivative-on-measurement avoids a full-duty kick when the target changes. */
  if (!pid.first)
  {
    derivative = -(measured - pid.previousMeasurement) / dtSeconds;
  }
  else
  {
    pid.first = false;
  }
  pid.previousMeasurement = measured;

  const float candidateIntegral =
      clampFloat(pid.integral + (pid.ki * error * dtSeconds), 0.0f, 100.0f);
  const float candidateOutput =
      (pid.kp * error) + candidateIntegral + (pid.kd * derivative);

  /* Conditional integration prevents wind-up while the output is saturated. */
  if (!((candidateOutput > 100.0f && error > 0.0f) ||
        (candidateOutput < 0.0f && error < 0.0f)))
  {
    pid.integral = candidateIntegral;
  }

  return clampFloat((pid.kp * error) + pid.integral + (pid.kd * derivative),
                    0.0f,
                    100.0f);
}

uint8_t motorEnablePin(MotorSide side)
{
  return side == MOTOR_LEFT ? POWERON_LEFT_MOTOR_EN_PIN
                            : POWERON_RIGHT_MOTOR_EN_PIN;
}

uint8_t motorIn1Pin(MotorSide side)
{
  return side == MOTOR_LEFT ? POWERON_LEFT_MOTOR_IN1_PIN
                            : POWERON_RIGHT_MOTOR_IN1_PIN;
}

uint8_t motorIn2Pin(MotorSide side)
{
  return side == MOTOR_LEFT ? POWERON_LEFT_MOTOR_IN2_PIN
                            : POWERON_RIGHT_MOTOR_IN2_PIN;
}

void writeMotorEnable(MotorSide side, bool enabled)
{
  const bool high = enabled ? (POWERON_MOTOR_ENABLE_ACTIVE_LOW == 0)
                            : (POWERON_MOTOR_ENABLE_ACTIVE_LOW != 0);
  digitalWrite(motorEnablePin(side), high ? HIGH : LOW);
  if (side == MOTOR_LEFT)
  {
    leftMotorEnabled = enabled;
  }
  else
  {
    rightMotorEnabled = enabled;
  }
}

void parkMotorDirection(MotorSide side)
{
  digitalWrite(motorIn1Pin(side), LOW);
  digitalWrite(motorIn2Pin(side), LOW);
}

void writeMotorDirectionPins(MotorSide side, bool forward)
{
  const bool forwardIn1High =
      side == MOTOR_LEFT ? (POWERON_LEFT_DIR_FORWARD_IN1_HIGH != 0)
                         : (POWERON_RIGHT_DIR_FORWARD_IN1_HIGH != 0);
  const bool in1High = forward ? forwardIn1High : !forwardIn1High;
  digitalWrite(motorIn1Pin(side), in1High ? HIGH : LOW);
  digitalWrite(motorIn2Pin(side), in1High ? LOW : HIGH);
}

void configureMotorPwm()
{
  digitalWrite(POWERON_LEFT_MOTOR_PWM_PIN, LOW);
  digitalWrite(POWERON_RIGHT_MOTOR_PWM_PIN, LOW);
  pinMode(POWERON_LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(POWERON_RIGHT_MOTOR_PWM_PIN, OUTPUT);

  const uint8_t savedSreg = SREG;
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  TCNT1 = 0;
  ICR1 = kMotorPwmTop;
  OCR1A = 0;
  OCR1B = 0;
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  SREG = savedSreg;
}

void setMotorDuty(MotorSide side, float dutyPercent)
{
  const float limitedDuty = clampFloat(dutyPercent, 0.0f, 100.0f);

  if (limitedDuty <= 0.001f)
  {
    writeMotorEnable(side, false);
    const uint8_t savedSreg = SREG;
    cli();
    if (side == MOTOR_LEFT)
    {
      TCCR1A &= static_cast<uint8_t>(~(_BV(COM1A1) | _BV(COM1A0)));
      OCR1A = 0;
    }
    else
    {
      TCCR1A &= static_cast<uint8_t>(~(_BV(COM1B1) | _BV(COM1B0)));
      OCR1B = 0;
    }
    SREG = savedSreg;
    digitalWrite(side == MOTOR_LEFT ? POWERON_LEFT_MOTOR_PWM_PIN
                                    : POWERON_RIGHT_MOTOR_PWM_PIN,
                 LOW);
    parkMotorDirection(side);
    return;
  }

  const uint16_t periodCounts = static_cast<uint16_t>(kMotorPwmTop + 1U);
  uint16_t compare = static_cast<uint16_t>(
      ((limitedDuty * static_cast<float>(periodCounts)) / 100.0f) + 0.5f);
  if (compare > kMotorPwmTop)
  {
    compare = kMotorPwmTop;
  }

  writeMotorDirectionPins(side,
                          side == MOTOR_LEFT ? leftMotorForward
                                             : rightMotorForward);

  const uint8_t savedSreg = SREG;
  cli();
  if (side == MOTOR_LEFT)
  {
    OCR1A = compare;
    TCCR1A = static_cast<uint8_t>(
        (TCCR1A & ~_BV(COM1A0)) | _BV(COM1A1) | _BV(WGM11));
  }
  else
  {
    OCR1B = compare;
    TCCR1A = static_cast<uint8_t>(
        (TCCR1A & ~_BV(COM1B0)) | _BV(COM1B1) | _BV(WGM11));
  }
  SREG = savedSreg;
  writeMotorEnable(side, true);
}

void stopMotorOutputs()
{
  setMotorDuty(MOTOR_LEFT, 0.0f);
  setMotorDuty(MOTOR_RIGHT, 0.0f);
  parkMotorDirection(MOTOR_LEFT);
  parkMotorDirection(MOTOR_RIGHT);
}

bool setMotorDirection(MotorSide side, bool forward)
{
  bool &currentForward = side == MOTOR_LEFT ? leftMotorForward
                                             : rightMotorForward;
  bool &pauseActive = side == MOTOR_LEFT ? leftDirectionPauseActive
                                         : rightDirectionPauseActive;
  uint32_t &pauseStartedMs =
      side == MOTOR_LEFT ? leftDirectionPauseStartedMs
                         : rightDirectionPauseStartedMs;
  const bool changed = currentForward != forward;

  if (changed)
  {
    setMotorDuty(side, 0.0f);
    parkMotorDirection(side);
    currentForward = forward;
    pauseStartedMs = millis();
    pauseActive = true;
  }

  /* Direction inputs are changed only while ENABLE is inactive. */
  writeMotorDirectionPins(side, forward);
  return changed;
}

bool motorDirectionReady(MotorSide side, uint32_t nowMs)
{
  bool &pauseActive = side == MOTOR_LEFT ? leftDirectionPauseActive
                                         : rightDirectionPauseActive;
  const uint32_t pauseStartedMs =
      side == MOTOR_LEFT ? leftDirectionPauseStartedMs
                         : rightDirectionPauseStartedMs;

  if (!pauseActive)
  {
    return true;
  }
  if ((nowMs - pauseStartedMs) < POWERON_DIRECTION_CHANGE_PAUSE_MS)
  {
    return false;
  }

  pauseActive = false;
  return true;
}

uint16_t servoUsToTicks(uint16_t microseconds)
{
  return static_cast<uint16_t>((microseconds + (kServoTimerTickUs / 2U)) /
                               kServoTimerTickUs);
}

void setServoPulseTargets(uint16_t leftUs, uint16_t rightUs)
{
  leftUs = constrain(leftUs,
                     static_cast<uint16_t>(POWERON_SERVO_LEFT_SAFE_MIN_US),
                     static_cast<uint16_t>(POWERON_SERVO_LEFT_SAFE_MAX_US));
  rightUs = constrain(rightUs,
                      static_cast<uint16_t>(POWERON_SERVO_RIGHT_SAFE_MIN_US),
                      static_cast<uint16_t>(POWERON_SERVO_RIGHT_SAFE_MAX_US));

  const uint8_t savedSreg = SREG;
  cli();
  pendingServoLeftTicks = servoUsToTicks(leftUs);
  pendingServoRightTicks = servoUsToTicks(rightUs);
  SREG = savedSreg;
}

void scheduleServoDelayTicks(uint16_t ticks)
{
  if (ticks == 0U)
  {
    ticks = 1U;
  }
  const uint16_t chunk = ticks > 256U ? 256U : ticks;
  servoDelayRemainingTicks = static_cast<uint16_t>(ticks - chunk);
  OCR2A = static_cast<uint8_t>(chunk - 1U);
  TCNT2 = 0;
}

void startServoFrame()
{
  activeServoLeftTicks = pendingServoLeftTicks;
  activeServoRightTicks = pendingServoRightTicks;

  /*
   * Pulse the channels sequentially, like Arduino's Servo library does.  This
   * preserves two independently calibrated pulse widths even when they differ
   * by only one 4 us timer tick.  It also avoids scheduling back-to-back
   * compare interrupts for nearly equal simultaneous pulses.
   */
  PORTD = static_cast<uint8_t>(
      (PORTD & static_cast<uint8_t>(~(kLeftServoMask | kRightServoMask))) |
      kLeftServoMask);
  servoEvent = SERVO_EVENT_LEFT_FALL;
  scheduleServoDelayTicks(activeServoLeftTicks);
}

void handleServoTimerCompare()
{
  if (servoDelayRemainingTicks > 0U)
  {
    scheduleServoDelayTicks(servoDelayRemainingTicks);
    return;
  }

  if (servoEvent == SERVO_EVENT_LEFT_FALL)
  {
    PORTD = static_cast<uint8_t>(
        (PORTD & static_cast<uint8_t>(~kLeftServoMask)) | kRightServoMask);
    servoEvent = SERVO_EVENT_RIGHT_FALL;
    scheduleServoDelayTicks(activeServoRightTicks);
    return;
  }

  if (servoEvent == SERVO_EVENT_RIGHT_FALL)
  {
    PORTD &= static_cast<uint8_t>(~kRightServoMask);
    servoEvent = SERVO_EVENT_FRAME_START;
    scheduleServoDelayTicks(static_cast<uint16_t>(
        kServoFrameTicks - activeServoLeftTicks - activeServoRightTicks));
    return;
  }

  startServoFrame();
}

void configureServoPwm()
{
  digitalWrite(POWERON_LEFT_SERVO_PIN, LOW);
  digitalWrite(POWERON_RIGHT_SERVO_PIN, LOW);
  pinMode(POWERON_LEFT_SERVO_PIN, OUTPUT);
  pinMode(POWERON_RIGHT_SERVO_PIN, OUTPUT);

  const uint8_t savedSreg = SREG;
  cli();
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 = 0;
  TCNT2 = 0;
  TIFR2 = _BV(OCF2A);
  TCCR2A = _BV(WGM21);
  startServoFrame();
  TIMSK2 = _BV(OCIE2A);
  TCCR2B = _BV(CS22);  // Prescaler 64.
  SREG = savedSreg;
}

void setServosFromWheelAngles(float leftDegrees, float rightDegrees)
{
  leftDegrees = clampFloat(leftDegrees,
                           POWERON_MIN_WHEEL_STEER_DEG,
                           POWERON_MAX_WHEEL_STEER_DEG);
  rightDegrees = clampFloat(rightDegrees,
                            POWERON_MIN_WHEEL_STEER_DEG,
                            POWERON_MAX_WHEEL_STEER_DEG);

  /* Geometry is always expressed using physical wheel positions. */
  const float leftChannelDegrees =
      POWERON_SERVO_CHANNELS_CROSSED ? rightDegrees : leftDegrees;
  const float rightChannelDegrees =
      POWERON_SERVO_CHANNELS_CROSSED ? leftDegrees : rightDegrees;

  const float leftPulse =
      POWERON_SERVO_LEFT_NEUTRAL_US +
      (POWERON_SERVO_LEFT_DIRECTION * leftChannelDegrees *
       POWERON_SERVO_US_PER_DEG);
  const float rightPulse =
      POWERON_SERVO_RIGHT_NEUTRAL_US +
      (POWERON_SERVO_RIGHT_DIRECTION * rightChannelDegrees *
       POWERON_SERVO_US_PER_DEG);

  setServoPulseTargets(
      static_cast<uint16_t>(clampFloat(leftPulse,
                                       POWERON_SERVO_LEFT_SAFE_MIN_US,
                                       POWERON_SERVO_LEFT_SAFE_MAX_US) +
                            0.5f),
      static_cast<uint16_t>(clampFloat(rightPulse,
                                       POWERON_SERVO_RIGHT_SAFE_MIN_US,
                                       POWERON_SERVO_RIGHT_SAFE_MAX_US) +
                            0.5f));
}

float speedToMotorRpm(float speedMps)
{
  const float wheelRpm =
      (60.0f * absoluteFloat(speedMps)) /
      (2.0f * kPi * POWERON_REAR_WHEEL_RADIUS_M);
  return wheelRpm * POWERON_GEAR_REDUCTION;
}

float speedToWheelRpm(float speedMps)
{
  return (60.0f * speedMps) /
         (2.0f * kPi * POWERON_REAR_WHEEL_RADIUS_M);
}

float wheelRpmToSpeed(float wheelRpm)
{
  return (wheelRpm * (2.0f * kPi * POWERON_REAR_WHEEL_RADIUS_M)) /
         60.0f;
}

float motorRpmToSpeed(float motorRpm)
{
  return ((motorRpm / POWERON_GEAR_REDUCTION) *
          (2.0f * kPi * POWERON_REAR_WHEEL_RADIUS_M)) /
         60.0f;
}

bool isStraightRadius(float radiusM)
{
  return absoluteFloat(radiusM) > (kStraightRadiusM * 0.5f);
}

float centerSteeringForInnerWheel(float innerWheelMagnitudeDeg)
{
  const float innerRadians = innerWheelMagnitudeDeg * kDegToRad;
  const float turnRadiusM =
      (POWERON_FRONT_TRACK_M * 0.5f) +
      (POWERON_WHEELBASE_M / tanf(innerRadians));
  return atanf(POWERON_WHEELBASE_M / turnRadiusM) * kRadToDeg;
}

float minCenterSteeringDeg()
{
  /* A negative command is a right turn, whose right wheel is the inner wheel. */
  return -centerSteeringForInnerWheel(
      absoluteFloat(POWERON_MIN_WHEEL_STEER_DEG));
}

float maxCenterSteeringDeg()
{
  /* A positive command is a left turn, whose left wheel is the inner wheel. */
  return centerSteeringForInnerWheel(POWERON_MAX_WHEEL_STEER_DEG);
}

void updateRearMotorTargets()
{
  const float halfTrack = POWERON_REAR_TRACK_M * 0.5f;
  float leftSpeed = vehicle.targetSpeedMps;
  float rightSpeed = vehicle.targetSpeedMps;

  if (!isStraightRadius(vehicle.turnRadiusM))
  {
    float radius = absoluteFloat(vehicle.turnRadiusM);
    if (radius <= halfTrack)
    {
      radius = halfTrack + kMinRadiusMarginM;
    }

    const float innerRatio = (radius - halfTrack) / radius;
    const float outerRatio = (radius + halfTrack) / radius;
    if (vehicle.turnRadiusM > 0.0f)
    {
      leftSpeed *= innerRatio;
      rightSpeed *= outerRatio;
    }
    else
    {
      leftSpeed *= outerRatio;
      rightSpeed *= innerRatio;
    }
  }

  /* Preserve the differential ratio while keeping every wheel under the limit. */
  const float largestWheelSpeed =
      max(absoluteFloat(leftSpeed), absoluteFloat(rightSpeed));
  float speedScale = 1.0f;
  if (largestWheelSpeed > POWERON_MAX_SPEED_MPS)
  {
    speedScale = POWERON_MAX_SPEED_MPS / largestWheelSpeed;
    leftSpeed *= speedScale;
    rightSpeed *= speedScale;
  }

  /* Never leave a sub-deadband RPM target with an unarmed command watchdog. */
  if (absoluteFloat(leftSpeed) < kMotionEpsilon)
  {
    leftSpeed = 0.0f;
  }
  if (absoluteFloat(rightSpeed) < kMotionEpsilon)
  {
    rightSpeed = 0.0f;
  }

  vehicle.appliedCenterSpeedMps = vehicle.targetSpeedMps * speedScale;
  vehicle.targetLeftWheelMps = leftSpeed;
  vehicle.targetRightWheelMps = rightSpeed;
  vehicle.targetLeftMotorRpm = speedToMotorRpm(leftSpeed);
  vehicle.targetRightMotorRpm = speedToMotorRpm(rightSpeed);

  if (absoluteFloat(leftSpeed) > kMotionEpsilon &&
      setMotorDirection(MOTOR_LEFT, leftSpeed > 0.0f))
  {
    pidReset(leftPid);
  }
  if (absoluteFloat(rightSpeed) > kMotionEpsilon &&
      setMotorDirection(MOTOR_RIGHT, rightSpeed > 0.0f))
  {
    pidReset(rightPid);
  }
}

void applySteering(float centerSteeringDeg)
{
  vehicle.steeringDeg = clampFloat(centerSteeringDeg,
                                   minCenterSteeringDeg(),
                                   maxCenterSteeringDeg());

  if (absoluteFloat(vehicle.steeringDeg) < kMotionEpsilon)
  {
    vehicle.turnRadiusM = kStraightRadiusM;
    vehicle.frontLeftSteerDeg = 0.0f;
    vehicle.frontRightSteerDeg = 0.0f;
    setServosFromWheelAngles(0.0f, 0.0f);
    if (driveMode == DRIVE_MODE_VEHICLE_SPEED)
    {
      updateRearMotorTargets();
    }
    return;
  }

  vehicle.turnRadiusM =
      POWERON_WHEELBASE_M / tanf(vehicle.steeringDeg * kDegToRad);
  float radius = absoluteFloat(vehicle.turnRadiusM);
  const float halfFrontTrack = POWERON_FRONT_TRACK_M * 0.5f;
  if (radius <= halfFrontTrack)
  {
    radius = halfFrontTrack + kMinRadiusMarginM;
  }

  const float innerDeg =
      atanf(POWERON_WHEELBASE_M / (radius - halfFrontTrack)) * kRadToDeg;
  const float outerDeg =
      atanf(POWERON_WHEELBASE_M / (radius + halfFrontTrack)) * kRadToDeg;

  /* Positive/A is a left turn; negative/D is a right turn. */
  if (vehicle.turnRadiusM > 0.0f)
  {
    vehicle.frontLeftSteerDeg = innerDeg;
    vehicle.frontRightSteerDeg = outerDeg;
  }
  else
  {
    vehicle.frontLeftSteerDeg = -outerDeg;
    vehicle.frontRightSteerDeg = -innerDeg;
  }

  vehicle.frontLeftSteerDeg = clampFloat(vehicle.frontLeftSteerDeg,
                                         POWERON_MIN_WHEEL_STEER_DEG,
                                         POWERON_MAX_WHEEL_STEER_DEG);
  vehicle.frontRightSteerDeg = clampFloat(vehicle.frontRightSteerDeg,
                                          POWERON_MIN_WHEEL_STEER_DEG,
                                          POWERON_MAX_WHEEL_STEER_DEG);
  setServosFromWheelAngles(vehicle.frontLeftSteerDeg,
                           vehicle.frontRightSteerDeg);
  if (driveMode == DRIVE_MODE_VEHICLE_SPEED)
  {
    updateRearMotorTargets();
  }
}

void setVehicleSpeed(float speedMps)
{
#if !POWERON_CLOSED_LOOP_READY
  /* Gear ratio, encoder scale, and PID are still unverified for RB35GM. */
  speedMps = 0.0f;
#endif
  const bool modeChanged = driveMode != DRIVE_MODE_VEHICLE_SPEED;
  driveMode = DRIVE_MODE_VEHICLE_SPEED;
  vehicle.targetSpeedMps = clampFloat(speedMps,
                                      -POWERON_MAX_SPEED_MPS,
                                      POWERON_MAX_SPEED_MPS);
  if (absoluteFloat(vehicle.targetSpeedMps) < kMotionEpsilon)
  {
    vehicle.targetSpeedMps = 0.0f;
  }
  if (modeChanged)
  {
    pidReset(leftPid);
    pidReset(rightPid);
  }
  updateRearMotorTargets();
}

void setWheelRpmTargets(float leftWheelRpm, float rightWheelRpm)
{
#if !POWERON_CLOSED_LOOP_READY
  leftWheelRpm = 0.0f;
  rightWheelRpm = 0.0f;
#endif
  const float maxWheelRpm = speedToWheelRpm(POWERON_MAX_SPEED_MPS);
  const bool modeChanged = driveMode != DRIVE_MODE_WHEEL_RPM;
  driveMode = DRIVE_MODE_WHEEL_RPM;

  leftWheelRpm = clampFloat(leftWheelRpm, -maxWheelRpm, maxWheelRpm);
  rightWheelRpm = clampFloat(rightWheelRpm, -maxWheelRpm, maxWheelRpm);
  float leftSpeed = wheelRpmToSpeed(leftWheelRpm);
  float rightSpeed = wheelRpmToSpeed(rightWheelRpm);
  if (absoluteFloat(leftSpeed) < kMotionEpsilon)
  {
    leftSpeed = 0.0f;
  }
  if (absoluteFloat(rightSpeed) < kMotionEpsilon)
  {
    rightSpeed = 0.0f;
  }

  vehicle.targetLeftWheelMps = leftSpeed;
  vehicle.targetRightWheelMps = rightSpeed;
  vehicle.targetSpeedMps = (leftSpeed + rightSpeed) * 0.5f;
  vehicle.appliedCenterSpeedMps = vehicle.targetSpeedMps;
  vehicle.targetLeftMotorRpm = speedToMotorRpm(leftSpeed);
  vehicle.targetRightMotorRpm = speedToMotorRpm(rightSpeed);

  bool directionChanged = false;
  if (leftSpeed != 0.0f)
  {
    directionChanged |= setMotorDirection(MOTOR_LEFT, leftSpeed > 0.0f);
  }
  if (rightSpeed != 0.0f)
  {
    directionChanged |= setMotorDirection(MOTOR_RIGHT, rightSpeed > 0.0f);
  }
  if (modeChanged || directionChanged)
  {
    pidReset(leftPid);
    pidReset(rightPid);
  }
}

void stopDriveOnly()
{
  driveMode = DRIVE_MODE_VEHICLE_SPEED;
  motionWatchdogArmed = false;
  vehicle.targetSpeedMps = 0.0f;
  vehicle.appliedCenterSpeedMps = 0.0f;
  vehicle.targetLeftWheelMps = 0.0f;
  vehicle.targetRightWheelMps = 0.0f;
  vehicle.targetLeftMotorRpm = 0.0f;
  vehicle.targetRightMotorRpm = 0.0f;
  vehicle.leftDutyPercent = 0.0f;
  vehicle.rightDutyPercent = 0.0f;
  pidReset(leftPid);
  pidReset(rightPid);
  stopMotorOutputs();
}

void enterIdle()
{
  stopDriveOnly();
  applySteering(0.0f);
}

void markMotionCommand()
{
  ++commandCount;
  lastMotionCommandMs = millis();
  if (driveMode == DRIVE_MODE_OPEN_LOOP)
  {
    motionWatchdogArmed = vehicle.leftDutyPercent > kMotionEpsilon ||
                          vehicle.rightDutyPercent > kMotionEpsilon;
  }
  else
  {
    motionWatchdogArmed =
        absoluteFloat(vehicle.targetLeftWheelMps) >= kMotionEpsilon ||
        absoluteFloat(vehicle.targetRightWheelMps) >= kMotionEpsilon;
  }
}

void applyRawMotor(float leftPercent, float rightPercent)
{
  driveMode = DRIVE_MODE_OPEN_LOOP;
  vehicle.targetSpeedMps = 0.0f;
  vehicle.appliedCenterSpeedMps = 0.0f;
  vehicle.targetLeftWheelMps = 0.0f;
  vehicle.targetRightWheelMps = 0.0f;
  vehicle.targetLeftMotorRpm = 0.0f;
  vehicle.targetRightMotorRpm = 0.0f;
  const float rawDutyLimit = POWERON_CLOSED_LOOP_READY
                                 ? 100.0f
                                 : POWERON_UNCALIBRATED_MAX_DUTY_PERCENT;
  vehicle.leftDutyPercent =
      clampFloat(absoluteFloat(leftPercent), 0.0f, rawDutyLimit);
  vehicle.rightDutyPercent =
      clampFloat(absoluteFloat(rightPercent), 0.0f, rawDutyLimit);

  if (vehicle.leftDutyPercent > kMotionEpsilon)
  {
    setMotorDirection(MOTOR_LEFT, leftPercent > 0.0f);
  }
  else
  {
    setMotorDuty(MOTOR_LEFT, 0.0f);
    parkMotorDirection(MOTOR_LEFT);
  }

  if (vehicle.rightDutyPercent > kMotionEpsilon)
  {
    setMotorDirection(MOTOR_RIGHT, rightPercent > 0.0f);
  }
  else
  {
    setMotorDuty(MOTOR_RIGHT, 0.0f);
    parkMotorDirection(MOTOR_RIGHT);
  }

  markMotionCommand();
}

void takeEncoderSnapshot(EncoderIsrSnapshot &snapshot)
{
  const uint8_t savedSreg = SREG;
  cli();
  snapshot.leftDelta = leftEncoderDelta;
  snapshot.rightDelta = rightEncoderDelta;
  snapshot.leftValid = leftEncoderValid;
  snapshot.rightValid = rightEncoderValid;
  snapshot.leftInvalid = leftEncoderInvalid;
  snapshot.rightInvalid = rightEncoderInvalid;
  snapshot.leftAChanges = leftEncoderAChanges;
  snapshot.leftBChanges = leftEncoderBChanges;
  snapshot.rightAChanges = rightEncoderAChanges;
  snapshot.rightBChanges = rightEncoderBChanges;
  leftEncoderDelta = 0;
  rightEncoderDelta = 0;
  leftEncoderValid = 0;
  rightEncoderValid = 0;
  leftEncoderInvalid = 0;
  rightEncoderInvalid = 0;
  leftEncoderAChanges = 0;
  leftEncoderBChanges = 0;
  rightEncoderAChanges = 0;
  rightEncoderBChanges = 0;
  SREG = savedSreg;
}

bool updateEncoderSpeed(EncoderRuntime &encoder,
                        float &filteredMotorRpm,
                        uint32_t sampleElapsedMs,
                        uint32_t nowMs)
{
  encoder.lastWindowDelta = encoder.windowDelta;
  encoder.windowDelta = 0;

  const float rawMotorRpm =
      (static_cast<float>(encoder.lastWindowDelta) * 60000.0f) /
      (POWERON_ENCODER_COUNTS_PER_MOTOR_REV *
       static_cast<float>(sampleElapsedMs));
  if (absoluteFloat(rawMotorRpm) > POWERON_ENCODER_MAX_VALID_MOTOR_RPM)
  {
    ++encoder.rejectedSamples;
    if (encoder.badSampleStreak < UINT8_MAX)
    {
      ++encoder.badSampleStreak;
    }
    return false;
  }

  encoder.badSampleStreak = 0;
  encoder.rawMotorRpm = rawMotorRpm;
  if ((nowMs - encoder.lastProgressMs) >= POWERON_ENCODER_ZERO_TIMEOUT_MS)
  {
    encoder.rawMotorRpm = 0.0f;
    filteredMotorRpm = 0.0f;
    return true;
  }

  const float alpha =
      static_cast<float>(sampleElapsedMs) /
      (POWERON_RPM_FILTER_TIME_CONSTANT_MS +
       static_cast<float>(sampleElapsedMs));
  filteredMotorRpm += alpha * (encoder.rawMotorRpm - filteredMotorRpm);
  return true;
}

void updateEncoderFaultMonitor(EncoderRuntime &encoder,
                               bool demanded,
                               bool forward,
                               uint32_t nowMs)
{
  if (!demanded)
  {
    encoder.faultMonitorActive = false;
    return;
  }
  if (!encoder.faultMonitorActive || encoder.monitoredForward != forward)
  {
    encoder.faultMonitorActive = true;
    encoder.monitoredForward = forward;
    encoder.lastProgressMs = nowMs;
  }
}

void latchEncoderFaults(uint8_t newFaults)
{
  newFaults &= static_cast<uint8_t>(~encoderFaultFlags);
  if (newFaults == 0U)
  {
    return;
  }

  encoderFaultFlags |= newFaults;
  stopDriveOnly();
  Serial.print(F("FAULT encoder_or_stall flags="));
  Serial.println(encoderFaultFlags);
}

void clearEncoderFaults()
{
  stopDriveOnly();
  encoderFaultFlags = 0U;
  leftEncoder.badSampleStreak = 0U;
  rightEncoder.badSampleStreak = 0U;
  const uint32_t nowMs = millis();
  leftEncoder.lastProgressMs = nowMs;
  rightEncoder.lastProgressMs = nowMs;
}

void updateVehicleControl(uint32_t elapsedMs)
{
  const uint32_t nowMs = millis();
  EncoderIsrSnapshot snapshot;
  takeEncoderSnapshot(snapshot);

  snapshot.leftDelta *= POWERON_LEFT_ENCODER_FORWARD_SIGN;
  snapshot.rightDelta *= POWERON_RIGHT_ENCODER_FORWARD_SIGN;
  leftEncoder.lastControlDelta = snapshot.leftDelta;
  rightEncoder.lastControlDelta = snapshot.rightDelta;
  leftEncoder.windowDelta =
      saturatingAdd(leftEncoder.windowDelta, snapshot.leftDelta);
  rightEncoder.windowDelta =
      saturatingAdd(rightEncoder.windowDelta, snapshot.rightDelta);
  leftEncoderTotal = saturatingAdd(leftEncoderTotal, snapshot.leftDelta);
  rightEncoderTotal = saturatingAdd(rightEncoderTotal, snapshot.rightDelta);

  leftEncoder.validTransitions += snapshot.leftValid;
  rightEncoder.validTransitions += snapshot.rightValid;
  leftEncoder.invalidTransitions += snapshot.leftInvalid;
  rightEncoder.invalidTransitions += snapshot.rightInvalid;
  leftEncoder.channelAChanges += snapshot.leftAChanges;
  leftEncoder.channelBChanges += snapshot.leftBChanges;
  rightEncoder.channelAChanges += snapshot.rightAChanges;
  rightEncoder.channelBChanges += snapshot.rightBChanges;
  if (snapshot.leftDelta != 0)
  {
    leftEncoder.lastProgressMs = nowMs;
  }
  if (snapshot.rightDelta != 0)
  {
    rightEncoder.lastProgressMs = nowMs;
  }

  encoderWindowElapsedMs += elapsedMs;
  bool leftMeasurementUpdated = false;
  bool rightMeasurementUpdated = false;
  float measurementDtSeconds = 0.0f;
  if (encoderWindowElapsedMs >= POWERON_ENCODER_SAMPLE_PERIOD_MS)
  {
    measurementDtSeconds = static_cast<float>(encoderWindowElapsedMs) / 1000.0f;
    leftMeasurementUpdated =
        updateEncoderSpeed(leftEncoder,
                           vehicle.measuredLeftMotorRpm,
                           encoderWindowElapsedMs,
                           nowMs);
    rightMeasurementUpdated =
        updateEncoderSpeed(rightEncoder,
                           vehicle.measuredRightMotorRpm,
                           encoderWindowElapsedMs,
                           nowMs);
    encoderWindowElapsedMs = 0U;
  }

#if POWERON_CLOSED_LOOP_READY
  vehicle.measuredSpeedMps =
      (motorRpmToSpeed(vehicle.measuredLeftMotorRpm) +
       motorRpmToSpeed(vehicle.measuredRightMotorRpm)) *
      0.5f;
#else
  /* A wheel-speed value would be misleading until the gearbox is identified. */
  vehicle.measuredSpeedMps = 0.0f;
#endif

  const bool leftDirectionReady = motorDirectionReady(MOTOR_LEFT, nowMs);
  const bool rightDirectionReady = motorDirectionReady(MOTOR_RIGHT, nowMs);
  if (driveMode == DRIVE_MODE_OPEN_LOOP)
  {
    leftEncoder.faultMonitorActive = false;
    rightEncoder.faultMonitorActive = false;
    setMotorDuty(MOTOR_LEFT,
                 leftDirectionReady ? vehicle.leftDutyPercent : 0.0f);
    setMotorDuty(MOTOR_RIGHT,
                 rightDirectionReady ? vehicle.rightDutyPercent : 0.0f);
    return;
  }

  const float minimumFaultMotorRpm =
      POWERON_ENCODER_FAULT_MIN_WHEEL_RPM * POWERON_GEAR_REDUCTION;
  updateEncoderFaultMonitor(leftEncoder,
                            vehicle.targetLeftMotorRpm >= minimumFaultMotorRpm,
                            vehicle.targetLeftWheelMps > 0.0f,
                            nowMs);
  updateEncoderFaultMonitor(rightEncoder,
                            vehicle.targetRightMotorRpm >= minimumFaultMotorRpm,
                            vehicle.targetRightWheelMps > 0.0f,
                            nowMs);

  uint8_t newFaults = 0U;
  if (leftEncoder.badSampleStreak >= POWERON_ENCODER_BAD_SAMPLE_LIMIT)
  {
    newFaults |= kEncoderFaultLeftImplausible;
  }
  if (rightEncoder.badSampleStreak >= POWERON_ENCODER_BAD_SAMPLE_LIMIT)
  {
    newFaults |= kEncoderFaultRightImplausible;
  }
  if (leftEncoder.faultMonitorActive && leftDirectionReady &&
      vehicle.leftDutyPercent >= POWERON_ENCODER_FAULT_MIN_DUTY_PERCENT &&
      (nowMs - leftEncoder.lastProgressMs) >=
          POWERON_ENCODER_NO_SIGNAL_TIMEOUT_MS)
  {
    newFaults |= kEncoderFaultLeftNoSignal;
  }
  if (rightEncoder.faultMonitorActive && rightDirectionReady &&
      vehicle.rightDutyPercent >= POWERON_ENCODER_FAULT_MIN_DUTY_PERCENT &&
      (nowMs - rightEncoder.lastProgressMs) >=
          POWERON_ENCODER_NO_SIGNAL_TIMEOUT_MS)
  {
    newFaults |= kEncoderFaultRightNoSignal;
  }
  if (newFaults != 0U)
  {
    latchEncoderFaults(newFaults);
    return;
  }

  if (vehicle.targetLeftMotorRpm <= 0.0f)
  {
    vehicle.leftDutyPercent = 0.0f;
    pidReset(leftPid);
  }
  else if (!leftDirectionReady)
  {
    vehicle.leftDutyPercent = 0.0f;
  }
  else if (leftMeasurementUpdated)
  {
    vehicle.leftDutyPercent =
        pidUpdate(leftPid,
                  vehicle.targetLeftMotorRpm,
                  absoluteFloat(vehicle.measuredLeftMotorRpm),
                  measurementDtSeconds);
  }
  setMotorDuty(MOTOR_LEFT, vehicle.leftDutyPercent);

  if (vehicle.targetRightMotorRpm <= 0.0f)
  {
    vehicle.rightDutyPercent = 0.0f;
    pidReset(rightPid);
  }
  else if (!rightDirectionReady)
  {
    vehicle.rightDutyPercent = 0.0f;
  }
  else if (rightMeasurementUpdated)
  {
    vehicle.rightDutyPercent =
        pidUpdate(rightPid,
                  vehicle.targetRightMotorRpm,
                  absoluteFloat(vehicle.measuredRightMotorRpm),
                  measurementDtSeconds);
  }
  setMotorDuty(MOTOR_RIGHT, vehicle.rightDutyPercent);
}

void checkMotionTimeout()
{
  if (commandTimeoutMs == 0U || !motionWatchdogArmed)
  {
    return;
  }

  if ((millis() - lastMotionCommandMs) >= commandTimeoutMs)
  {
    stopDriveOnly();
    ++timeoutStopCount;
    Serial.println(F("TIMEOUT STOP"));
  }
}

bool parseNumber(const char *text, float &value)
{
  if (text == nullptr || *text == '\0')
  {
    return false;
  }

  char *end = nullptr;
  const double parsed = strtod(text, &end);
  if (end == text || *end != '\0' || parsed != parsed ||
      parsed > 1000000.0 || parsed < -1000000.0)
  {
    return false;
  }
  value = static_cast<float>(parsed);
  return true;
}

void readServoPulseUs(uint16_t &leftUs, uint16_t &rightUs)
{
  const uint8_t savedSreg = SREG;
  cli();
  leftUs = static_cast<uint16_t>(pendingServoLeftTicks * kServoTimerTickUs);
  rightUs = static_cast<uint16_t>(pendingServoRightTicks * kServoTimerTickUs);
  SREG = savedSreg;
}

void printStatus(Print &out)
{
  uint16_t leftServoUs;
  uint16_t rightServoUs;
  readServoPulseUs(leftServoUs, rightServoUs);

  out.print(F("STATUS target_mps="));
  out.print(vehicle.targetSpeedMps, 3);
  out.print(F(" applied_mps="));
  out.print(vehicle.appliedCenterSpeedMps, 3);
  out.print(F(" measured_mps="));
  out.print(vehicle.measuredSpeedMps, 3);
  out.print(F(" steer_deg="));
  out.print(vehicle.steeringDeg, 2);
  out.print(F(" turn_radius_m="));
  out.print(vehicle.turnRadiusM, 3);
  out.print(F(" front_deg_l="));
  out.print(vehicle.frontLeftSteerDeg, 2);
  out.print(F(" front_deg_r="));
  out.print(vehicle.frontRightSteerDeg, 2);
  out.print(F(" target_rpm_l="));
  out.print(vehicle.targetLeftMotorRpm, 1);
  out.print(F(" target_rpm_r="));
  out.print(vehicle.targetRightMotorRpm, 1);
  out.print(F(" rpm_l="));
  out.print(vehicle.measuredLeftMotorRpm, 1);
  out.print(F(" rpm_r="));
  out.print(vehicle.measuredRightMotorRpm, 1);
  out.print(F(" duty_l="));
  out.print(vehicle.leftDutyPercent, 1);
  out.print(F(" duty_r="));
  out.print(vehicle.rightDutyPercent, 1);
  out.print(F(" enc_l="));
  out.print(leftEncoderTotal);
  out.print(F(" enc_r="));
  out.print(rightEncoderTotal);
  out.print(F(" enabled_l="));
  out.print(leftMotorEnabled ? 1 : 0);
  out.print(F(" enabled_r="));
  out.print(rightMotorEnabled ? 1 : 0);
  out.print(F(" servo_us_l="));
  out.print(leftServoUs);
  out.print(F(" servo_us_r="));
  out.print(rightServoUs);
  out.print(F(" timeout_ms="));
  out.print(commandTimeoutMs);
  out.print(F(" timeout_stops="));
  out.print(timeoutStopCount);
  out.print(F(" rejected="));
  out.print(rejectedCommandCount);
  out.print(F(" motor=RB35GM_09TYPE_26P"));
  out.print(F(" closed_loop_ready="));
  out.print(POWERON_CLOSED_LOOP_READY ? 1 : 0);
  out.print(F(" gear_verified="));
  out.print(POWERON_GEAR_RATIO_VERIFIED);
  out.print(F(" encoder_counts_verified="));
  out.print(POWERON_ENCODER_COUNTS_VERIFIED);
  out.print(F(" assumed_counts_motor_rev="));
  out.println(POWERON_ENCODER_COUNTS_PER_MOTOR_REV, 1);
}

void printHelp(Print &out)
{
  out.println(F("Arduino Uno PowerOn controller"));
  out.println(F("Keys: arrows/WASD drive and steer, A/D or LEFT/RIGHT +/-1 deg"));
  out.println(F("Other keys: c center, z speed 0, x/SPACE stop, i idle, h help"));
  out.println(F("Commands: @DRIVE <m/s> <deg>, @SPEED <m/s>, @STEER <deg>, @CENTER, @STOP, @IDLE"));
  out.println(F("Tests: @MOTOR <left%> <right%>, @SERVO <left_us> <right_us>, @TIMEOUT <ms>, @STATUS"));
#if !POWERON_CLOSED_LOOP_READY
  out.println(F("RB35GM CALIBRATION REQUIRED: W/S/@DRIVE/@SPEED motion is locked; @MOTOR is capped at 10%."));
  out.println(F("Verify gear ratio, 52-count assumption, A/B signs, voltage, and PID before enabling closed loop."));
#endif
  out.print(F("Safe center steering range: "));
  out.print(minCenterSteeringDeg(), 2);
  out.print(F(" to +"));
  out.print(maxCenterSteeringDeg(), 2);
  out.println(F(" deg (negative=right, positive=left)"));
}

bool executeCommand(char *line, Print &out)
{
  while (*line == '@' || *line == ':' || *line == ' ' || *line == '\t')
  {
    ++line;
  }

  char *save = nullptr;
  char *verb = strtok_r(line, " \t", &save);
  if (verb == nullptr)
  {
    return false;
  }

  for (char *cursor = verb; *cursor != '\0'; ++cursor)
  {
    if (*cursor >= 'a' && *cursor <= 'z')
    {
      *cursor = static_cast<char>(*cursor - 'a' + 'A');
    }
  }

  float first;
  float second;
  if (strcmp(verb, "DRIVE") == 0)
  {
    if (!parseNumber(strtok_r(nullptr, " \t", &save), first) ||
        !parseNumber(strtok_r(nullptr, " \t", &save), second) ||
        strtok_r(nullptr, " \t", &save) != nullptr)
    {
      return false;
    }
#if !POWERON_CLOSED_LOOP_READY
    if (absoluteFloat(first) > kMotionEpsilon)
    {
      stopDriveOnly();
      ++rejectedCommandCount;
      out.println(F("ERR RB35GM calibration required; use @MOTOR <=10% for bench tests"));
      return true;
    }
#endif
    setVehicleSpeed(first);
    applySteering(second);
    markMotionCommand();
  }
  else if (strcmp(verb, "SPEED") == 0)
  {
    if (!parseNumber(strtok_r(nullptr, " \t", &save), first) ||
        strtok_r(nullptr, " \t", &save) != nullptr)
    {
      return false;
    }
#if !POWERON_CLOSED_LOOP_READY
    if (absoluteFloat(first) > kMotionEpsilon)
    {
      stopDriveOnly();
      ++rejectedCommandCount;
      out.println(F("ERR RB35GM calibration required; use @MOTOR <=10% for bench tests"));
      return true;
    }
#endif
    setVehicleSpeed(first);
    markMotionCommand();
  }
  else if (strcmp(verb, "STEER") == 0)
  {
    if (!parseNumber(strtok_r(nullptr, " \t", &save), first) ||
        strtok_r(nullptr, " \t", &save) != nullptr)
    {
      return false;
    }
    applySteering(first);
    markMotionCommand();
  }
  else if (strcmp(verb, "CENTER") == 0)
  {
    if (strtok_r(nullptr, " \t", &save) != nullptr)
    {
      return false;
    }
    applySteering(0.0f);
    markMotionCommand();
  }
  else if (strcmp(verb, "STOP") == 0)
  {
    if (strtok_r(nullptr, " \t", &save) != nullptr)
    {
      return false;
    }
    stopDriveOnly();
    ++commandCount;
  }
  else if (strcmp(verb, "IDLE") == 0)
  {
    if (strtok_r(nullptr, " \t", &save) != nullptr)
    {
      return false;
    }
    enterIdle();
    markMotionCommand();
  }
  else if (strcmp(verb, "MOTOR") == 0)
  {
    if (!parseNumber(strtok_r(nullptr, " \t", &save), first) ||
        !parseNumber(strtok_r(nullptr, " \t", &save), second) ||
        strtok_r(nullptr, " \t", &save) != nullptr)
    {
      return false;
    }
    applyRawMotor(first, second);
  }
  else if (strcmp(verb, "SERVO") == 0)
  {
    if (!parseNumber(strtok_r(nullptr, " \t", &save), first) ||
        !parseNumber(strtok_r(nullptr, " \t", &save), second) ||
        strtok_r(nullptr, " \t", &save) != nullptr)
    {
      return false;
    }
    setServoPulseTargets(
        static_cast<uint16_t>(clampFloat(first,
                                         POWERON_SERVO_LEFT_SAFE_MIN_US,
                                         POWERON_SERVO_LEFT_SAFE_MAX_US) +
                              0.5f),
        static_cast<uint16_t>(clampFloat(second,
                                         POWERON_SERVO_RIGHT_SAFE_MIN_US,
                                         POWERON_SERVO_RIGHT_SAFE_MAX_US) +
                              0.5f));
    ++commandCount;
  }
  else if (strcmp(verb, "TIMEOUT") == 0)
  {
    if (!parseNumber(strtok_r(nullptr, " \t", &save), first) || first < 0.0f ||
        strtok_r(nullptr, " \t", &save) != nullptr)
    {
      return false;
    }
    commandTimeoutMs =
        first < 0.5f
            ? 0UL
            : static_cast<uint32_t>(clampFloat(first,
                                                POWERON_TIMEOUT_MIN_MS,
                                                POWERON_TIMEOUT_MAX_MS));
    ++commandCount;
  }
  else if (strcmp(verb, "STATUS") == 0)
  {
    if (strtok_r(nullptr, " \t", &save) != nullptr)
    {
      return false;
    }
    printStatus(out);
    return true;
  }
  else if (strcmp(verb, "HELP") == 0)
  {
    if (strtok_r(nullptr, " \t", &save) != nullptr)
    {
      return false;
    }
    printHelp(out);
    return true;
  }
  else
  {
    return false;
  }

  out.print(F("OK target_mps="));
  out.print(vehicle.targetSpeedMps, 3);
  out.print(F(" applied_mps="));
  out.print(vehicle.appliedCenterSpeedMps, 3);
  out.print(F(" steer_deg="));
  out.println(vehicle.steeringDeg, 2);
  return true;
}

void handleSingleKey(char key)
{
  switch (key)
  {
    case 'w':
    case 'W':
#if !POWERON_CLOSED_LOOP_READY
      Serial.println(F("ERR RB35GM calibration required; use @MOTOR <=10% for bench tests"));
      return;
#else
      setVehicleSpeed(vehicle.targetSpeedMps + POWERON_SPEED_STEP_MPS);
      break;
#endif
    case 's':
    case 'S':
#if !POWERON_CLOSED_LOOP_READY
      Serial.println(F("ERR RB35GM calibration required; use @MOTOR <=10% for bench tests"));
      return;
#else
      setVehicleSpeed(vehicle.targetSpeedMps - POWERON_SPEED_STEP_MPS);
      break;
#endif
    case 'a':
    case 'A':
      applySteering(vehicle.steeringDeg + POWERON_STEER_STEP_DEG);
      break;
    case 'd':
    case 'D':
      applySteering(vehicle.steeringDeg - POWERON_STEER_STEP_DEG);
      break;
    case 'c':
    case 'C':
      applySteering(0.0f);
      break;
    case 'z':
    case 'Z':
      setVehicleSpeed(0.0f);
      break;
    case 'x':
    case 'X':
    case ' ':
      stopDriveOnly();
      ++commandCount;
      Serial.println(F("OK STOP"));
      return;
    case 'i':
    case 'I':
      enterIdle();
      markMotionCommand();
      Serial.println(F("OK IDLE target_mps=0.000 steer_deg=0.00"));
      return;
    case 'h':
    case 'H':
    case '?':
      printHelp(Serial);
      return;
    default:
      return;
  }

  markMotionCommand();
  Serial.print(F("OK target_mps="));
  Serial.print(vehicle.targetSpeedMps, 3);
  Serial.print(F(" applied_mps="));
  Serial.print(vehicle.appliedCenterSpeedMps, 3);
  Serial.print(F(" steer_deg="));
  Serial.println(vehicle.steeringDeg, 2);
}

bool handleNavigationSequenceByte(uint8_t incoming)
{
  if (serialKeySequenceState == SERIAL_KEY_ESCAPE)
  {
    if (incoming == '[' || incoming == 'O')
    {
      serialKeySequenceState = SERIAL_KEY_ANSI;
      return true;
    }

    serialKeySequenceState = SERIAL_KEY_IDLE;
    return false;
  }

  if (serialKeySequenceState == SERIAL_KEY_ANSI)
  {
    /* CSI parameters may precede the final A/B/C/D arrow code. */
    if ((incoming >= '0' && incoming <= '9') || incoming == ';')
    {
      return true;
    }

    serialKeySequenceState = SERIAL_KEY_IDLE;
    switch (incoming)
    {
      case 'A':
        handleSingleKey('w');
        break;
      case 'B':
        handleSingleKey('s');
        break;
      case 'C':
        handleSingleKey('d');
        break;
      case 'D':
        handleSingleKey('a');
        break;
      default:
        break;
    }
    return true;
  }

  if (serialKeySequenceState == SERIAL_KEY_WINDOWS)
  {
    serialKeySequenceState = SERIAL_KEY_IDLE;
    switch (incoming)
    {
      case 72U:
        handleSingleKey('w');
        break;
      case 80U:
        handleSingleKey('s');
        break;
      case 77U:
        handleSingleKey('d');
        break;
      case 75U:
        handleSingleKey('a');
        break;
      default:
        break;
    }
    return true;
  }

  if (incoming == 0x1BU)
  {
    serialKeySequenceState = SERIAL_KEY_ESCAPE;
    return true;
  }
  if (incoming == 0x00U || incoming == 0xE0U)
  {
    serialKeySequenceState = SERIAL_KEY_WINDOWS;
    return true;
  }
  return false;
}

void pollSerial()
{
  uint8_t processed = 0;
  while (Serial.available() > 0 && processed < POWERON_SERIAL_BYTES_PER_LOOP)
  {
    ++processed;
    const uint8_t incoming = static_cast<uint8_t>(Serial.read());

    if (serialDiscardUntilEol)
    {
      if (incoming == '\r' || incoming == '\n')
      {
        serialDiscardUntilEol = false;
      }
      continue;
    }

    if (!serialInLine)
    {
      if (handleNavigationSequenceByte(incoming))
      {
        continue;
      }
      if (incoming == '@' || incoming == ':')
      {
        serialInLine = true;
        serialLineLength = 0;
        serialLine[serialLineLength++] = incoming;
      }
      else if (incoming != '\r' && incoming != '\n')
      {
        handleSingleKey(static_cast<char>(incoming));
      }
      continue;
    }

    if (incoming == '\r' || incoming == '\n')
    {
      serialLine[serialLineLength] = '\0';
      if (!executeCommand(serialLine, Serial))
      {
        ++rejectedCommandCount;
        Serial.println(F("ERR bad command or arguments"));
      }
      serialLineLength = 0;
      serialInLine = false;
    }
    else if (serialLineLength < sizeof(serialLine) - 1U)
    {
      serialLine[serialLineLength++] = incoming;
    }
    else
    {
      serialLineLength = 0;
      serialInLine = false;
      serialDiscardUntilEol = true;
      ++rejectedCommandCount;
      Serial.println(F("ERR line too long; discarded through end-of-line"));
    }
  }
}

void configureHardware()
{
  /* Preload the inactive ENABLE level before making the pins outputs. */
  const uint8_t disabledLevel =
      POWERON_MOTOR_ENABLE_ACTIVE_LOW ? HIGH : LOW;
  digitalWrite(POWERON_LEFT_MOTOR_EN_PIN, disabledLevel);
  digitalWrite(POWERON_RIGHT_MOTOR_EN_PIN, disabledLevel);
  pinMode(POWERON_LEFT_MOTOR_EN_PIN, OUTPUT);
  pinMode(POWERON_RIGHT_MOTOR_EN_PIN, OUTPUT);

  digitalWrite(POWERON_LEFT_MOTOR_IN1_PIN, LOW);
  digitalWrite(POWERON_LEFT_MOTOR_IN2_PIN, LOW);
  digitalWrite(POWERON_RIGHT_MOTOR_IN1_PIN, LOW);
  digitalWrite(POWERON_RIGHT_MOTOR_IN2_PIN, LOW);
  pinMode(POWERON_LEFT_MOTOR_IN1_PIN, OUTPUT);
  pinMode(POWERON_LEFT_MOTOR_IN2_PIN, OUTPUT);
  pinMode(POWERON_RIGHT_MOTOR_IN1_PIN, OUTPUT);
  pinMode(POWERON_RIGHT_MOTOR_IN2_PIN, OUTPUT);

  configureMotorPwm();
  stopMotorOutputs();

  setServoPulseTargets(
      static_cast<uint16_t>(POWERON_SERVO_LEFT_NEUTRAL_US + 0.5f),
      static_cast<uint16_t>(POWERON_SERVO_RIGHT_NEUTRAL_US + 0.5f));
  configureServoPwm();

  pinMode(POWERON_LEFT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(POWERON_LEFT_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(POWERON_RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(POWERON_RIGHT_ENCODER_B_PIN, INPUT_PULLUP);
  previousEncoderBits = static_cast<uint8_t>(PINC & 0x0FU);
  PCIFR = _BV(PCIF1);
  PCMSK1 = _BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11);
  PCICR |= _BV(PCIE1);
}

}  // namespace

ISR(TIMER2_COMPA_vect)
{
  handleServoTimerCompare();
}

ISR(PCINT1_vect)
{
  static const int8_t transition[16] = {
      0, -1, 1, 0,
      1, 0, 0, -1,
      -1, 0, 0, 1,
      0, 1, -1, 0};

  const uint8_t current = static_cast<uint8_t>(PINC & 0x0FU);
  const uint8_t oldLeft = previousEncoderBits & 0x03U;
  const uint8_t newLeft = current & 0x03U;
  const uint8_t oldRight = (previousEncoderBits >> 2) & 0x03U;
  const uint8_t newRight = (current >> 2) & 0x03U;

  leftEncoderDelta += transition[(oldLeft << 2) | newLeft];
  rightEncoderDelta += transition[(oldRight << 2) | newRight];
  previousEncoderBits = current;
}

void setup()
{
  configureHardware();
  stopDriveOnly();
  applySteering(0.0f);

  Serial.begin(115200);
  delay(50);
  Serial.println(F("PowerOn Uno RB35GM: outputs safe; controller ready"));
  lastControlMs = millis();
  lastMotionCommandMs = millis();
  printHelp(Serial);
}

void loop()
{
  pollSerial();
  checkMotionTimeout();

  const uint32_t nowMs = millis();
  const uint32_t elapsedMs = nowMs - lastControlMs;
  if (elapsedMs >= POWERON_CONTROL_PERIOD_MS)
  {
    lastControlMs = nowMs;
    updateVehicleControl(static_cast<float>(elapsedMs) / 1000.0f);
  }
}
