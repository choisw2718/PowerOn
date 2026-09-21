#include <Arduino.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/*
 * Independent Arduino Uno controller for:
 *   - one encoderless 12 V rear DC motor,
 *   - two front steering servos,
 *   - USB Serial commands from a laptop.
 *
 * The motor driver is the existing MAI-2MT-DC V3.0. Only its channel A is
 * used. Motor power comes from an external 12 V supply; never from the Uno.
 */

#if !defined(__AVR_ATmega328P__)
#error "Select Arduino Uno (ATmega328P). This sketch is not for Yun, Uno R4, or ESP32."
#endif

#if F_CPU != 16000000UL
#error "This timer setup requires a 16 MHz Arduino Uno."
#endif

namespace
{

/* Existing channel-A wiring, reused for the single center rear motor. */
const uint8_t kMotorPwmPin = 9;    // Driver channel A PWM / Timer1 OC1A
const uint8_t kMotorIn1Pin = 2;    // Driver channel A IN1
const uint8_t kMotorIn2Pin = 4;    // Driver channel A IN2
const uint8_t kMotorEnablePin = 3; // Driver channel A ENABLE (active LOW)

/* The unused channel B is driven to a known, disabled state. */
const uint8_t kUnusedMotorPwmPin = 10;
const uint8_t kUnusedMotorIn1Pin = 7;
const uint8_t kUnusedMotorIn2Pin = 8;
const uint8_t kUnusedMotorEnablePin = 11;

/* Existing two-servo wiring and calibration. */
const uint8_t kLeftServoPin = 5;
const uint8_t kRightServoPin = 6;
const uint16_t kLeftServoMinUs = 948;
const uint16_t kLeftServoNeutralUs = 1398;
const uint16_t kLeftServoMaxUs = 1948;
const uint16_t kRightServoMinUs = 952;
const uint16_t kRightServoNeutralUs = 1402;
const uint16_t kRightServoMaxUs = 1952;
const float kServoUsPerDegree = 10.0f;
const float kLeftServoDirection = 1.0f;
const float kRightServoDirection = 1.0f;
const bool kServoChannelsCrossed = true;

/* Ackermann steering dimensions retained from the original vehicle. */
const float kWheelbaseM = 0.215f;
const float kFrontTrackM = 0.188f;
const float kMinWheelSteerDeg = -45.0f;
const float kMaxWheelSteerDeg = 55.0f;

/* Open-loop motor settings. */
const float kMotorSupplyVoltage = 12.0f;
const float kMaxDutyPercent = 100.0f;
const bool kForwardIn1High = true;
const uint32_t kMotorPwmHz = 20000UL;
const uint32_t kDirectionChangePauseMs = 50UL;
const uint32_t kDefaultCommandTimeoutMs = 1500UL;
const uint32_t kMinCommandTimeoutMs = 200UL;
const uint32_t kMaxCommandTimeoutMs = 600000UL;

const float kPi = 3.14159265f;
const float kDegToRad = 0.0174532925f;
const float kRadToDeg = 57.2957795f;
const float kEpsilon = 0.001f;

const uint16_t kMotorPwmTop =
    static_cast<uint16_t>((F_CPU / kMotorPwmHz) - 1UL);

/* Timer2 runs at 16 MHz / 64: one tick every 4 us. */
const uint16_t kServoTimerTickUs = 4U;
const uint32_t kServoFrameUs = 20000UL;
const uint16_t kServoFrameTicks =
    static_cast<uint16_t>(kServoFrameUs / kServoTimerTickUs);
const uint8_t kLeftServoMask = _BV(PD5);
const uint8_t kRightServoMask = _BV(PD6);

static_assert(kMotorPwmPin == 9, "Timer1 PWM requires Uno D9");
static_assert(kLeftServoPin == 5 && kRightServoPin == 6,
              "Timer2 servo scheduler directly drives Uno D5 and D6");
static_assert(kMotorPwmTop == 799U,
              "20 kHz Timer1 TOP must be 799 at 16 MHz");

enum ServoEvent : uint8_t
{
  SERVO_LEFT_FALL = 0,
  SERVO_RIGHT_FALL = 1,
  SERVO_FRAME_START = 2
};

volatile uint16_t pendingLeftServoTicks =
    kLeftServoNeutralUs / kServoTimerTickUs;
volatile uint16_t pendingRightServoTicks =
    kRightServoNeutralUs / kServoTimerTickUs;
volatile uint16_t activeLeftServoTicks =
    kLeftServoNeutralUs / kServoTimerTickUs;
volatile uint16_t activeRightServoTicks =
    kRightServoNeutralUs / kServoTimerTickUs;
volatile uint16_t servoDelayRemainingTicks = 0;
volatile ServoEvent servoEvent = SERVO_FRAME_START;

float targetSignedDutyPercent = 0.0f;
float steeringCenterDeg = 0.0f;
float frontLeftSteerDeg = 0.0f;
float frontRightSteerDeg = 0.0f;
float appliedDutyPercent = 0.0f;
bool motorForward = true;
bool motorEnabled = false;
bool directionPauseActive = false;
uint32_t directionPauseStartedMs = 0;
uint32_t lastMotorCommandMs = 0;
uint32_t commandTimeoutMs = kDefaultCommandTimeoutMs;
uint32_t timeoutStopCount = 0;

const uint8_t kSerialLineCapacity = 64;
char serialLine[kSerialLineCapacity];
uint8_t serialLineLength = 0;
bool serialDiscardUntilEol = false;

float clampFloat(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

float absoluteFloat(float value)
{
  return value < 0.0f ? -value : value;
}

void writeMotorEnable(bool enabled)
{
  /* This driver board's ENABLE input is low-active. */
  digitalWrite(kMotorEnablePin, enabled ? LOW : HIGH);
  motorEnabled = enabled;
}

void parkMotorDirection()
{
  digitalWrite(kMotorIn1Pin, LOW);
  digitalWrite(kMotorIn2Pin, LOW);
}

void writeMotorDirection(bool forward)
{
  const bool in1High = forward ? kForwardIn1High : !kForwardIn1High;
  digitalWrite(kMotorIn1Pin, in1High ? HIGH : LOW);
  digitalWrite(kMotorIn2Pin, in1High ? LOW : HIGH);
}

void disableMotorOutput()
{
  writeMotorEnable(false);

  const uint8_t savedSreg = SREG;
  cli();
  TCCR1A &= static_cast<uint8_t>(~(_BV(COM1A1) | _BV(COM1A0)));
  OCR1A = 0;
  SREG = savedSreg;

  digitalWrite(kMotorPwmPin, LOW);
  parkMotorDirection();
  appliedDutyPercent = 0.0f;
}

void configureMotorPwm()
{
  digitalWrite(kMotorPwmPin, LOW);
  pinMode(kMotorPwmPin, OUTPUT);

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

void applyMotorDuty(float dutyPercent)
{
  const float limitedDuty = clampFloat(dutyPercent, 0.0f, kMaxDutyPercent);
  if (limitedDuty <= kEpsilon)
  {
    disableMotorOutput();
    return;
  }

  const uint16_t periodCounts = static_cast<uint16_t>(kMotorPwmTop + 1U);
  uint16_t compare = static_cast<uint16_t>(
      ((limitedDuty * static_cast<float>(periodCounts)) / 100.0f) + 0.5f);
  if (compare > kMotorPwmTop)
  {
    compare = kMotorPwmTop;
  }

  /* Direction is set while the driver remains disabled. */
  writeMotorDirection(motorForward);

  const uint8_t savedSreg = SREG;
  cli();
  OCR1A = compare;
  TCCR1A = static_cast<uint8_t>(
      (TCCR1A & ~_BV(COM1A0)) | _BV(COM1A1) | _BV(WGM11));
  SREG = savedSreg;

  writeMotorEnable(true);
  appliedDutyPercent = limitedDuty;
}

void commandMotorPercent(float signedDutyPercent)
{
  const float limited =
      clampFloat(signedDutyPercent, -kMaxDutyPercent, kMaxDutyPercent);
  lastMotorCommandMs = millis();

  if (absoluteFloat(limited) <= kEpsilon)
  {
    targetSignedDutyPercent = 0.0f;
    directionPauseActive = false;
    disableMotorOutput();
    return;
  }

  const bool requestedForward = limited > 0.0f;
  targetSignedDutyPercent = limited;

  if (requestedForward != motorForward)
  {
    disableMotorOutput();
    motorForward = requestedForward;
    writeMotorDirection(motorForward);
    directionPauseStartedMs = millis();
    directionPauseActive = true;
  }
}

void serviceMotor()
{
  if (absoluteFloat(targetSignedDutyPercent) <= kEpsilon)
  {
    return;
  }

  if (directionPauseActive)
  {
    if ((millis() - directionPauseStartedMs) < kDirectionChangePauseMs)
    {
      return;
    }
    directionPauseActive = false;
  }

  applyMotorDuty(absoluteFloat(targetSignedDutyPercent));
}

uint16_t servoUsToTicks(uint16_t microseconds)
{
  return static_cast<uint16_t>((microseconds + (kServoTimerTickUs / 2U)) /
                               kServoTimerTickUs);
}

void setServoPulseTargets(uint16_t leftUs, uint16_t rightUs)
{
  leftUs = constrain(leftUs, kLeftServoMinUs, kLeftServoMaxUs);
  rightUs = constrain(rightUs, kRightServoMinUs, kRightServoMaxUs);

  const uint8_t savedSreg = SREG;
  cli();
  pendingLeftServoTicks = servoUsToTicks(leftUs);
  pendingRightServoTicks = servoUsToTicks(rightUs);
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
  activeLeftServoTicks = pendingLeftServoTicks;
  activeRightServoTicks = pendingRightServoTicks;

  PORTD = static_cast<uint8_t>(
      (PORTD & static_cast<uint8_t>(~(kLeftServoMask | kRightServoMask))) |
      kLeftServoMask);
  servoEvent = SERVO_LEFT_FALL;
  scheduleServoDelayTicks(activeLeftServoTicks);
}

void handleServoTimerCompare()
{
  if (servoDelayRemainingTicks > 0U)
  {
    scheduleServoDelayTicks(servoDelayRemainingTicks);
    return;
  }

  if (servoEvent == SERVO_LEFT_FALL)
  {
    PORTD = static_cast<uint8_t>(
        (PORTD & static_cast<uint8_t>(~kLeftServoMask)) | kRightServoMask);
    servoEvent = SERVO_RIGHT_FALL;
    scheduleServoDelayTicks(activeRightServoTicks);
    return;
  }

  if (servoEvent == SERVO_RIGHT_FALL)
  {
    PORTD &= static_cast<uint8_t>(~kRightServoMask);
    servoEvent = SERVO_FRAME_START;
    scheduleServoDelayTicks(static_cast<uint16_t>(
        kServoFrameTicks - activeLeftServoTicks - activeRightServoTicks));
    return;
  }

  startServoFrame();
}

void configureServoPwm()
{
  digitalWrite(kLeftServoPin, LOW);
  digitalWrite(kRightServoPin, LOW);
  pinMode(kLeftServoPin, OUTPUT);
  pinMode(kRightServoPin, OUTPUT);

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
  TCCR2B = _BV(CS22); // Prescaler 64.
  SREG = savedSreg;
}

float centerSteeringForInnerWheel(float innerWheelMagnitudeDeg)
{
  const float innerRadians = innerWheelMagnitudeDeg * kDegToRad;
  const float turnRadiusM =
      (kFrontTrackM * 0.5f) + (kWheelbaseM / tanf(innerRadians));
  return atanf(kWheelbaseM / turnRadiusM) * kRadToDeg;
}

float minCenterSteeringDeg()
{
  return -centerSteeringForInnerWheel(absoluteFloat(kMinWheelSteerDeg));
}

float maxCenterSteeringDeg()
{
  return centerSteeringForInnerWheel(kMaxWheelSteerDeg);
}

void setServosFromWheelAngles(float leftDegrees, float rightDegrees)
{
  leftDegrees = clampFloat(leftDegrees, kMinWheelSteerDeg, kMaxWheelSteerDeg);
  rightDegrees = clampFloat(rightDegrees, kMinWheelSteerDeg, kMaxWheelSteerDeg);

  const float leftChannelDegrees =
      kServoChannelsCrossed ? rightDegrees : leftDegrees;
  const float rightChannelDegrees =
      kServoChannelsCrossed ? leftDegrees : rightDegrees;

  const float leftPulse =
      kLeftServoNeutralUs +
      (kLeftServoDirection * leftChannelDegrees * kServoUsPerDegree);
  const float rightPulse =
      kRightServoNeutralUs +
      (kRightServoDirection * rightChannelDegrees * kServoUsPerDegree);

  setServoPulseTargets(
      static_cast<uint16_t>(
          clampFloat(leftPulse, kLeftServoMinUs, kLeftServoMaxUs) + 0.5f),
      static_cast<uint16_t>(
          clampFloat(rightPulse, kRightServoMinUs, kRightServoMaxUs) + 0.5f));
}

void applySteering(float centerDegrees)
{
  steeringCenterDeg =
      clampFloat(centerDegrees, minCenterSteeringDeg(), maxCenterSteeringDeg());

  if (absoluteFloat(steeringCenterDeg) <= kEpsilon)
  {
    steeringCenterDeg = 0.0f;
    frontLeftSteerDeg = 0.0f;
    frontRightSteerDeg = 0.0f;
    setServosFromWheelAngles(0.0f, 0.0f);
    return;
  }

  const float turnRadiusM =
      kWheelbaseM / tanf(steeringCenterDeg * kDegToRad);
  float radius = absoluteFloat(turnRadiusM);
  const float halfTrack = kFrontTrackM * 0.5f;
  if (radius <= halfTrack)
  {
    radius = halfTrack + 0.001f;
  }

  const float innerDeg =
      atanf(kWheelbaseM / (radius - halfTrack)) * kRadToDeg;
  const float outerDeg =
      atanf(kWheelbaseM / (radius + halfTrack)) * kRadToDeg;

  /* Positive is left; negative is right. */
  if (turnRadiusM > 0.0f)
  {
    frontLeftSteerDeg = innerDeg;
    frontRightSteerDeg = outerDeg;
  }
  else
  {
    frontLeftSteerDeg = -outerDeg;
    frontRightSteerDeg = -innerDeg;
  }

  setServosFromWheelAngles(frontLeftSteerDeg, frontRightSteerDeg);
}

void readServoPulseUs(uint16_t &leftUs, uint16_t &rightUs)
{
  const uint8_t savedSreg = SREG;
  cli();
  leftUs = pendingLeftServoTicks * kServoTimerTickUs;
  rightUs = pendingRightServoTicks * kServoTimerTickUs;
  SREG = savedSreg;
}

void printStatus(Print &out)
{
  uint16_t leftServoUs;
  uint16_t rightServoUs;
  readServoPulseUs(leftServoUs, rightServoUs);

  out.print(F("STATUS voltage_cmd="));
  out.print((targetSignedDutyPercent / 100.0f) * kMotorSupplyVoltage, 2);
  out.print(F("V duty_cmd="));
  out.print(targetSignedDutyPercent, 1);
  out.print(F("% duty_applied="));
  out.print(appliedDutyPercent, 1);
  out.print(F("% direction="));
  if (absoluteFloat(targetSignedDutyPercent) <= kEpsilon)
  {
    out.print(F("STOP"));
  }
  else
  {
    out.print(targetSignedDutyPercent > 0.0f ? F("FORWARD") : F("REVERSE"));
  }
  out.print(F(" enabled="));
  out.print(motorEnabled ? 1 : 0);
  out.print(F(" steer_center="));
  out.print(steeringCenterDeg, 1);
  out.print(F("deg servo_us_l="));
  out.print(leftServoUs);
  out.print(F(" servo_us_r="));
  out.print(rightServoUs);
  out.print(F(" timeout_ms="));
  out.print(commandTimeoutMs);
  out.print(F(" timeout_stops="));
  out.println(timeoutStopCount);
}

void printHelp(Print &out)
{
  out.println(F("PowerOn Uno single rear motor controller"));
  out.println(F("Line commands (115200 baud, newline):"));
  out.println(F("  VOLTAGE <-12.0..12.0>  signed nominal average motor voltage"));
  out.println(F("  MOTOR <-100..100>      signed PWM duty percent"));
  out.println(F("  STEER <degrees>         +left, -right; safe range is clamped"));
  out.println(F("  CENTER | STOP | IDLE | STATUS | HELP"));
  out.println(F("  TIMEOUT <200..600000>   motor-command watchdog in ms"));
  out.println(F("  KEEPALIVE               refresh watchdog, no reply"));
  out.println(F("VOLTAGE is open-loop PWM, not measured/regulated terminal voltage."));
}

bool parseNumber(const char *text, float &value)
{
  if (text == NULL || *text == '\0')
  {
    return false;
  }

  char *end = NULL;
  const double parsed = strtod(text, &end);
  if (end == text || *end != '\0' || !isfinite(parsed))
  {
    return false;
  }
  value = static_cast<float>(parsed);
  return true;
}

bool parseUnsignedLong(const char *text, uint32_t &value)
{
  if (text == NULL || *text == '\0' || *text == '-')
  {
    return false;
  }

  char *end = NULL;
  const unsigned long parsed = strtoul(text, &end, 10);
  if (end == text || *end != '\0')
  {
    return false;
  }
  value = static_cast<uint32_t>(parsed);
  return true;
}

bool executeCommand(char *line, Print &out)
{
  char *save = NULL;
  char *verb = strtok_r(line, " \t", &save);
  if (verb == NULL)
  {
    return true;
  }

  for (char *p = verb; *p != '\0'; ++p)
  {
    *p = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
  }

  char *argument = strtok_r(NULL, " \t", &save);
  char *extra = strtok_r(NULL, " \t", &save);
  float number = 0.0f;

  if (strcmp(verb, "VOLTAGE") == 0)
  {
    if (!parseNumber(argument, number) || extra != NULL)
    {
      return false;
    }
    const float limitedVoltage =
        clampFloat(number, -kMotorSupplyVoltage, kMotorSupplyVoltage);
    commandMotorPercent((limitedVoltage / kMotorSupplyVoltage) * 100.0f);
    out.print(F("OK VOLTAGE requested="));
    out.print(number, 2);
    out.print(F("V applied_command="));
    out.print(limitedVoltage, 2);
    out.println(F("V"));
    return true;
  }

  if (strcmp(verb, "MOTOR") == 0)
  {
    if (!parseNumber(argument, number) || extra != NULL)
    {
      return false;
    }
    commandMotorPercent(number);
    out.print(F("OK MOTOR duty="));
    out.print(targetSignedDutyPercent, 1);
    out.println(F("%"));
    return true;
  }

  if (strcmp(verb, "STEER") == 0)
  {
    if (!parseNumber(argument, number) || extra != NULL)
    {
      return false;
    }
    applySteering(number);
    out.print(F("OK STEER center="));
    out.print(steeringCenterDeg, 1);
    out.print(F("deg left="));
    out.print(frontLeftSteerDeg, 1);
    out.print(F("deg right="));
    out.print(frontRightSteerDeg, 1);
    out.println(F("deg"));
    return true;
  }

  if (strcmp(verb, "TIMEOUT") == 0)
  {
    uint32_t requestedTimeout = 0;
    if (!parseUnsignedLong(argument, requestedTimeout) || extra != NULL ||
        requestedTimeout < kMinCommandTimeoutMs ||
        requestedTimeout > kMaxCommandTimeoutMs)
    {
      return false;
    }
    commandTimeoutMs = requestedTimeout;
    out.print(F("OK TIMEOUT ms="));
    out.println(commandTimeoutMs);
    return true;
  }

  if (argument != NULL)
  {
    return false;
  }

  if (strcmp(verb, "STOP") == 0)
  {
    commandMotorPercent(0.0f);
    out.println(F("OK STOP"));
    return true;
  }
  if (strcmp(verb, "CENTER") == 0)
  {
    applySteering(0.0f);
    out.println(F("OK CENTER"));
    return true;
  }
  if (strcmp(verb, "IDLE") == 0)
  {
    commandMotorPercent(0.0f);
    applySteering(0.0f);
    out.println(F("OK IDLE"));
    return true;
  }
  if (strcmp(verb, "STATUS") == 0)
  {
    printStatus(out);
    return true;
  }
  if (strcmp(verb, "HELP") == 0 || strcmp(verb, "?") == 0)
  {
    printHelp(out);
    return true;
  }
  if (strcmp(verb, "KEEPALIVE") == 0)
  {
    if (absoluteFloat(targetSignedDutyPercent) > kEpsilon)
    {
      lastMotorCommandMs = millis();
    }
    return true;
  }

  return false;
}

void pollSerial()
{
  while (Serial.available() > 0)
  {
    const char incoming = static_cast<char>(Serial.read());

    if (serialDiscardUntilEol)
    {
      if (incoming == '\r' || incoming == '\n')
      {
        serialDiscardUntilEol = false;
      }
      continue;
    }

    if (incoming == '\r' || incoming == '\n')
    {
      if (serialLineLength == 0U)
      {
        continue;
      }
      serialLine[serialLineLength] = '\0';
      if (!executeCommand(serialLine, Serial))
      {
        Serial.println(F("ERR bad command or arguments; send HELP"));
      }
      serialLineLength = 0;
      continue;
    }

    if (serialLineLength < (kSerialLineCapacity - 1U))
    {
      serialLine[serialLineLength++] = incoming;
    }
    else
    {
      serialLineLength = 0;
      serialDiscardUntilEol = true;
      Serial.println(F("ERR line too long; discarded"));
    }
  }
}

void checkCommandTimeout()
{
  if (absoluteFloat(targetSignedDutyPercent) <= kEpsilon)
  {
    return;
  }

  if ((millis() - lastMotorCommandMs) >= commandTimeoutMs)
  {
    commandMotorPercent(0.0f);
    ++timeoutStopCount;
    Serial.println(F("TIMEOUT STOP"));
  }
}

void configureHardware()
{
  /* Preload safe levels before changing pins to outputs. */
  digitalWrite(kMotorEnablePin, HIGH);
  pinMode(kMotorEnablePin, OUTPUT);
  digitalWrite(kMotorIn1Pin, LOW);
  digitalWrite(kMotorIn2Pin, LOW);
  pinMode(kMotorIn1Pin, OUTPUT);
  pinMode(kMotorIn2Pin, OUTPUT);

  digitalWrite(kUnusedMotorEnablePin, HIGH);
  pinMode(kUnusedMotorEnablePin, OUTPUT);
  digitalWrite(kUnusedMotorPwmPin, LOW);
  digitalWrite(kUnusedMotorIn1Pin, LOW);
  digitalWrite(kUnusedMotorIn2Pin, LOW);
  pinMode(kUnusedMotorPwmPin, OUTPUT);
  pinMode(kUnusedMotorIn1Pin, OUTPUT);
  pinMode(kUnusedMotorIn2Pin, OUTPUT);

  configureMotorPwm();
  disableMotorOutput();
  setServoPulseTargets(kLeftServoNeutralUs, kRightServoNeutralUs);
  configureServoPwm();
}

} // namespace

ISR(TIMER2_COMPA_vect)
{
  handleServoTimerCompare();
}

void setup()
{
  configureHardware();
  applySteering(0.0f);

  Serial.begin(115200);
  delay(50);
  lastMotorCommandMs = millis();
  Serial.println(F("PowerOn Uno single rear motor: outputs safe; ready"));
  printHelp(Serial);
}

void loop()
{
  pollSerial();
  checkCommandTimeout();
  serviceMotor();
}
