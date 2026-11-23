#include "CommandRouter.h"

#include "../ir_wall_following/IrAutonomy.h"
#include "../lidar_drive/DriveLease.h"
#include "../motor/DualRear.h"
#include "../motor/Motor.h"
#include "../steering/AckermannSteering.h"
#include "../wifi/WifiSession.h"
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace protocol {
namespace {

bool pingLedOn = false;

void cancelAutonomy() {
  // Manual input takes control after any live autonomous output stops.
  if (lidar_drive::armed() || ir_autonomy::isDriving()) {
    motor::stop();
    steering::apply(0.0f);
  }
  ir_autonomy::cancel();
  lidar_drive::cancel();
}

void printHelp(Print &out) {
  if (dual_rear::kSelected) {
    out.println(F("Dual rear L298N A/B; STOP | IDLE | STATUS | NETWORK"));
    out.println(F("DUAL ARM | SPEED <signed m/s> <center deg> (renew within 300 ms)"));
    out.println(F("DUAL TEST <L|R> <signed PWM%> (250 ms bench pulse, if configured)"));
    out.println(F("STEER <deg> | CENTER | SENSORS | PING | HELP"));
    out.println(F("Legacy PWM and live autonomy commands are disabled in dual mode."));
    return;
  }
  out.println(F("Nano 33 IoT L298N channel A; TCP ASCII + newline"));
  out.println(F("D9=ENA PWM, D2=IN1, D4=IN2; remove ENA jumper; D3 unused."));
  out.println(F("DRIVE <nominal V> <center deg> | VOLTAGE <nominal V>"));
  out.println(F("MOTOR <-100..100> | STEER <deg> | CENTER | STOP | IDLE"));
  out.println(F("STATUS | NETWORK | SENSORS | PING | HELP"));
  out.println(F("GP2Y0A21YK0F: A0 left, A1 front, A2 right; direct Vo wiring, ratio=1."));
  out.println(F("Auto steering: fractional degrees, 1 us pulse commands, 50 Hz smoothing."));
  out.println(F("AUTO TEST | AUTO START | AUTO STOP | AUTO STATUS"));
  out.println(F("LIDAR START | LIDAR DRIVE <0..65 PWM%> <-28..28 deg> [1..500 ms] | LIDAR STOP"));
  out.println(F("RANGES <left mm> <front mm> <right mm> (TEST only; -1=invalid)"));
  out.println(F("AUTO TEST computes targets only; START needs a live sensor driver + enable."));
  out.println(F("Autonomy stops on stale/invalid ranges; fault requires explicit restart."));
  out.println(F("Steering: equivalent center angle, positive=left; Ackermann wheel angles."));
  out.println(F("Both wheel angles share one radius; servo limits restrict the center angle."));
  out.println(F("DRIVE applies the inherited +/-7 V starting floor."));
  out.println(F("VOLTAGE is open-loop PWM, not measured motor voltage."));
  out.println(F("Manual motor holds the last command until STOP, IDLE, or disconnect."));
}

bool parseFloat(const char *text, float &result) {
  if (text == nullptr || *text == '\0') return false;
  char *end = nullptr;
  const double value = strtod(text, &end);
  if (end == text || *end != '\0' || !isfinite(value)) return false;
  result = static_cast<float>(value);
  return isfinite(result);
}

} // namespace

void begin() {
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
}

void stopSession(bool centerSteering) {
  cancelAutonomy();
  motor::stop();
  if (centerSteering) steering::apply(0.0f);
}

void printStatus(Print &out) {
  motor::printStatus(out);
  steering::printStatus(out);
}

bool executeCommand(char *command, Print &out) {
  char *save = nullptr;
  char *verb = strtok_r(command, " \t", &save);
  if (verb == nullptr) return true;
  for (char *p = verb; *p; ++p)
    *p = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
  char *a = strtok_r(nullptr, " \t", &save);
  char *b = strtok_r(nullptr, " \t", &save);
  char *c = strtok_r(nullptr, " \t", &save);
  char *extra = strtok_r(nullptr, " \t", &save);
  float value = 0.0f, steer = 0.0f;
  if (strcmp(verb, "DUAL") == 0 || strcmp(verb, "SPEED") == 0) {
    if (!dual_rear::kSelected) {
      out.println(F("ERR dual rear mode is disabled"));
      return true;
    }
    motor::service();
    if (strcmp(verb, "SPEED") == 0) {
      if (!parseFloat(a, value) || !parseFloat(b, steer) || c || extra ||
          fabsf(steer) > dual_rear::kMaxCenterSteer) {
        motor::stop();
        out.println(F("ERR SPEED invalid; motor stopped"));
        return true;
      }
      if (dual_rear::command(value, out)) steering::apply(steer);
      return true;
    }
    if (a && strcmp(a, "ARM") == 0 && !b) {
      cancelAutonomy();
      dual_rear::arm(out);
      return true;
    }
    if (a && strcmp(a, "TEST") == 0 && b &&
        (strcmp(b, "L") == 0 || strcmp(b, "R") == 0) &&
        parseFloat(c, value) && !extra) {
      cancelAutonomy();
      dual_rear::benchPulse(strcmp(b, "L") == 0, value, out);
      return true;
    }
    return false;
  }
  if (dual_rear::kSelected &&
      (strcmp(verb, "DRIVE") == 0 || strcmp(verb, "MOTOR") == 0 ||
       strcmp(verb, "VOLTAGE") == 0 || strcmp(verb, "LIDAR") == 0)) {
    motor::stop();
    out.println(F("ERR legacy motor command disabled in dual rear mode"));
    return true;
  }
  if (strcmp(verb, "RANGES") == 0) {
    float right = 0.0f;
    if (!parseFloat(a, value) || !parseFloat(b, steer) || !parseFloat(c, right) || extra)
      return false;
    ir_autonomy::setTestRanges(value, steer, right, out);
    return true;
  }
  if (strcmp(verb, "AUTO") == 0) {
    if (!a || b || c || extra) return false;
    for (char *p = a; *p; ++p)
      *p = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
    if (strcmp(a, "STATUS") == 0) { ir_autonomy::printStatus(out); return true; }
    if (strcmp(a, "STOP") == 0) {
      cancelAutonomy();
      motor::stop();
      out.println(F("OK AUTO STOP"));
      return true;
    }
    const bool preview = strcmp(a, "TEST") == 0;
    if (!preview && strcmp(a, "START") != 0) return false;
    cancelAutonomy();
    motor::stop();
    if (!preview && dual_rear::kSelected) {
      out.println(F("ERR live IR drive disabled in dual rear mode"));
      return true;
    }
    ir_autonomy::start(preview, out);
    return true;
  }
  if (strcmp(verb, "LIDAR") == 0) {
    if (a && strcmp(a, "START") == 0 && !b) {
      cancelAutonomy();
      motor::stop();
      lidar_drive::arm(millis());
      out.println(F("OK LIDAR START"));
      return true;
    }
    if (a && strcmp(a, "STOP") == 0 && !b) {
      cancelAutonomy();
      motor::stop();
      out.println(F("OK LIDAR STOP"));
      return true;
    }
    const uint32_t now = millis();
    lidar_drive::service(now);
    if (!lidar_drive::armed()) {
      out.println(lidar_drive::expired() ? F("ERR LIDAR lease expired") : F("ERR LIDAR not armed"));
      return true;
    }
    float lease = lidar_drive::kWatchdogMs;
    const bool validLease = !extra ||
        (parseFloat(extra, lease) && lease >= 1.0f && lease <= lidar_drive::kWatchdogMs &&
         lease == floorf(lease) && !strtok_r(nullptr, " \t", &save));
    if (!a || strcmp(a, "DRIVE") != 0 || !parseFloat(b, value) ||
        !parseFloat(c, steer) || !validLease || value < 0.0f || value > lidar_drive::kMaxPwm ||
        fabsf(steer) > lidar_drive::kMaxSteerDeg) {
      cancelAutonomy();
      motor::stop();
      out.println(F("ERR LIDAR command rejected; motor stopped"));
      return true;
    }
    lidar_drive::renew(now, static_cast<uint32_t>(lease));
    steering::apply(steer);
    motor::commandPercent(value);
    out.println(extra ? F("OK LIDAR DRIVE LEASE") : F("OK LIDAR DRIVE"));
    return true;
  }
  if (strcmp(verb, "DRIVE") == 0) {
    if (!parseFloat(a, value) || !parseFloat(b, steer) || c || extra) return false;
    cancelAutonomy();
    steering::apply(steer);
    out.print(F("OK DRIVE applied_nominal_V="));
    out.println(motor::commandReliableVoltage(value), 2);
    return true;
  }
  if (strcmp(verb, "VOLTAGE") == 0 || strcmp(verb, "MOTOR") == 0 ||
      strcmp(verb, "STEER") == 0) {
    if (!parseFloat(a, value) || b || c || extra) return false;
    cancelAutonomy();
    if (strcmp(verb, "STEER") == 0) {
      if (dual_rear::kSelected) motor::stop();
      steering::apply(value);
    } else if (strcmp(verb, "VOLTAGE") == 0) {
      motor::commandVoltage(value);
    } else {
      motor::commandPercent(value);
    }
    out.println(F("OK"));
    return true;
  }
  if (a || b || c || extra) return false;
  if (strcmp(verb, "PING") == 0) {
    pingLedOn = !pingLedOn;
    digitalWrite(LED_BUILTIN, pingLedOn ? HIGH : LOW);
    out.print(F("PONG LED="));
    out.println(pingLedOn ? F("ON") : F("OFF"));
    return true;
  }
  if (strcmp(verb, "STOP") == 0) { cancelAutonomy(); motor::stop(); out.println(F("OK STOP")); return true; }
  if (strcmp(verb, "CENTER") == 0) {
    cancelAutonomy();
    if (dual_rear::kSelected) motor::stop();
    steering::apply(0.0f);
    out.println(F("OK CENTER"));
    return true;
  }
  if (strcmp(verb, "IDLE") == 0) { cancelAutonomy(); motor::stop(); steering::apply(0.0f); out.println(F("OK IDLE")); return true; }
  if (strcmp(verb, "STATUS") == 0) { printStatus(out); ir_autonomy::printStatus(out); return true; }
  if (strcmp(verb, "SENSORS") == 0) { ir_autonomy::printSensors(out); return true; }
  if (strcmp(verb, "NETWORK") == 0) { wifi::printNetwork(out); return true; }
  if (strcmp(verb, "HELP") == 0 || strcmp(verb, "?") == 0) { printHelp(out); return true; }
  return false;
}

} // namespace protocol
