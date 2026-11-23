#ifndef POWERON_SHARP_IR_H
#define POWERON_SHARP_IR_H

#include <math.h>
#include <stdint.h>

namespace sharp_ir {

enum class Status : uint8_t { WarmingUp, NotConnected, InRange, FarOrUnknown, TooNear, SignalFault };
inline const char *statusName(Status status) {
  switch (status) {
    case Status::WarmingUp: return "warming_up";
    case Status::NotConnected: return "not_connected";
    case Status::InRange: return "in_range";
    case Status::FarOrUnknown: return "far_or_unknown";
    case Status::TooNear: return "too_near";
    default: return "signal_fault";
  }
}

struct Point { float mm; float volts; };
// Approximate READ-OFF of Sharp Fig. 2/3, not factory calibration constants.
// https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a21yk_e.pdf
// Measure all three units against the actual wall and replace per-unit tables.
constexpr Point kNominalCurve[] = {
  {100, 2.30f}, {150, 1.65f}, {200, 1.30f}, {250, 1.09f}, {300, 0.92f},
  {400, 0.72f}, {500, 0.59f}, {600, 0.50f}, {700, 0.44f}, {800, 0.40f}
};
constexpr uint8_t kCurveSize = sizeof(kNominalCurve) / sizeof(kNominalCurve[0]);

struct Calibration {
  const Point *curve = kNominalCurve;
  uint8_t count = kCurveSize;
  float adcReferenceVolts = 3.3f; // Measure actual board ADC reference/supply.
  // User-confirmed wiring: sensor Vo directly to ADC, no divider.
  // If 10k/10k dividers are installed later, use 0.5 instead.
  float dividerRatio = 1.0f;
  bool allowLowSignalAsFar = false;
};

struct Reading {
  uint16_t adc = 0;
  float sensorVolts = 0;
  float mm = 0;
  Status status = Status::WarmingUp;
  bool usable = false;
};

inline Reading decode(uint16_t adc, const Calibration &calibration) {
  Reading reading;
  reading.adc = adc;
  reading.status = Status::SignalFault;
  const Point *curve = calibration.curve;
  if (!curve || calibration.count < 2 ||
      !isfinite(calibration.adcReferenceVolts) || calibration.adcReferenceVolts <= 0 ||
      !isfinite(calibration.dividerRatio) || calibration.dividerRatio <= 0 ||
      calibration.dividerRatio > 1) return reading;
  for (uint8_t i = 0; i < calibration.count; ++i) {
    if (!isfinite(curve[i].mm) || !isfinite(curve[i].volts) ||
        curve[i].mm < 100 || curve[i].mm > 800 || curve[i].volts <= 0 ||
        (i && (curve[i].mm <= curve[i-1].mm || curve[i].volts >= curve[i-1].volts)))
      return reading;
  }
  reading.sensorVolts = static_cast<float>(adc) * calibration.adcReferenceVolts /
      (4095.0f * calibration.dividerRatio);
  // Rail checks detect some faults, not all disconnected/floating-wire cases.
  if (adc <= 8 || adc >= 4087) return reading;
  if (reading.sensorVolts > curve[0].volts) {
    reading.status = Status::TooNear;
    reading.mm = curve[0].mm;
    return reading;  // No extrapolation into the ambiguous <10 cm region.
  }
  const Point &last = curve[calibration.count - 1];
  if (reading.sensorVolts < last.volts) {
    reading.status = Status::FarOrUnknown;
    reading.mm = last.mm; // Saturation marker, NOT an 80 cm measurement.
    reading.usable = calibration.allowLowSignalAsFar;
    return reading;
  }
  for (uint8_t i = 1; i < calibration.count; ++i) {
    if (reading.sensorVolts >= curve[i].volts) {
      const Point &near = curve[i-1];
      const Point &far = curve[i];
      const float fraction = (near.volts - reading.sensorVolts) / (near.volts - far.volts);
      const float inverseMm = (1.0f - fraction) / near.mm + fraction / far.mm;
      reading.mm = 1.0f / inverseMm;
      reading.status = Status::InRange;
      reading.usable = true;
      return reading;
    }
  }
  return reading;
}

} // namespace sharp_ir
#endif
