"""Export the checked-in Arduino header from the shared vehicle profile."""

import argparse
from pathlib import Path

from . import PARAMETERS


HEADER = (Path(__file__).resolve().parents[1] /
          "vehicle/firmware/PowerOnNano/src/hardware/VehicleProfile.h")
CONSTANTS = {
    "steering": (
        ("float", "kWheelbaseM", "chassis", "wheelbase_m"),
        ("float", "kSteeringPivotTrackM", "chassis", "steering_pivot_track_m"),
        ("float", "kMinWheelSteerDeg", "steering", "wheel_min_deg"),
        ("float", "kMaxWheelSteerDeg", "steering", "wheel_max_deg"),
        ("bool", "kServoChannelsCrossed", "steering", "channels_crossed"),
        ("float", "kLeftServoDirection", "steering", "d5_direction"),
        ("float", "kRightServoDirection", "steering", "d6_direction"),
        ("float", "kLeftServoUsPerDegree", "steering", "d5_us_per_deg"),
        ("float", "kRightServoUsPerDegree", "steering", "d6_us_per_deg"),
        ("uint16_t", "kRightAlignedD5Us", "steering", "right_aligned_d5_us"),
        ("uint16_t", "kRightAlignedD6Us", "steering", "right_aligned_d6_us"),
        ("uint16_t", "kLeftAlignedD5Us", "steering", "left_aligned_d5_us"),
        ("uint16_t", "kLeftAlignedD6Us", "steering", "left_aligned_d6_us"),
        ("uint16_t", "kLeftServoMinUs", "steering", "d5_min_us"),
        ("uint16_t", "kLeftServoMaxUs", "steering", "d5_max_us"),
        ("uint16_t", "kRightServoMinUs", "steering", "d6_min_us"),
        ("uint16_t", "kRightServoMaxUs", "steering", "d6_max_us"),
    ),
    "motor": (
        ("float", "kMotorSupplyVoltage", "motor", "nominal_supply_v"),
        ("float", "kMinimumReliableMotorVoltage", "motor", "manual_start_v"),
    ),
    "dual_rear": (
        ("bool", "kSelected", "dual_rear", "selected"),
        ("int8_t", "kLeftMotorSign", "dual_rear", "left_motor_sign"),
        ("int8_t", "kRightMotorSign", "dual_rear", "right_motor_sign"),
        ("int8_t", "kLeftEncoderSign", "dual_rear", "left_encoder_sign"),
        ("int8_t", "kRightEncoderSign", "dual_rear", "right_encoder_sign"),
        ("float", "kRearTrackM", "dual_rear", "rear_track_m"),
        ("float", "kLeftRolloutM", "dual_rear", "left_rollout_m"),
        ("float", "kRightRolloutM", "dual_rear", "right_rollout_m"),
        ("float", "kLeftCountsPerRev", "dual_rear", "left_counts_per_rev"),
        ("float", "kRightCountsPerRev", "dual_rear", "right_counts_per_rev"),
        ("float", "kBenchPwm", "dual_rear", "bench_pwm_pct"),
        ("float", "kMaxPwm", "dual_rear", "max_pwm_pct"),
        ("float", "kMaxWheelSpeed", "dual_rear", "max_wheel_speed_mps"),
        ("float", "kTargetAcceleration", "dual_rear", "target_accel_mps2"),
        ("float", "kMotorBusV", "dual_rear", "motor_bus_v"),
        ("float", "kLeftFeedForward", "dual_rear", "left_ff_v_per_mps"),
        ("float", "kRightFeedForward", "dual_rear", "right_ff_v_per_mps"),
        ("float", "kLeftKp", "dual_rear", "left_kp"),
        ("float", "kRightKp", "dual_rear", "right_kp"),
        ("float", "kLeftKi", "dual_rear", "left_ki"),
        ("float", "kRightKi", "dual_rear", "right_ki"),
    ),
    "lidar": (
        ("uint32_t", "kWatchdogMs", "lidar", "max_lease_ms"),
        ("float", "kMaxPwm", "lidar", "max_pwm_pct"),
        ("float", "kMaxSteerDeg", "lidar", "max_center_steer_deg"),
    ),
    "ir": (
        ("bool", "kLeftInstalled", "ir_sensors", "left_installed"),
        ("bool", "kFrontInstalled", "ir_sensors", "front_installed"),
        ("bool", "kRightInstalled", "ir_sensors", "right_installed"),
    ),
}


def render_header():
    lines = ["// Generated from hardware/control_profile.json. Do not edit.",
             "#pragma once", "", "#include <stdint.h>", "", "namespace vehicle_profile {", ""]
    for namespace, constants in CONSTANTS.items():
        lines.append(f"namespace {namespace} {{")
        for kind, symbol, section, name in constants:
            value = PARAMETERS[section][name]
            if kind == "bool":
                literal = str(value).lower()
            elif kind == "float":
                literal = f"{float(value)!r}f"
            else:
                literal = str(value)
            lines.append(f"constexpr {kind} {symbol} = {literal};")
        lines.extend((f"}} // namespace {namespace}", ""))
    lines.extend(("} // namespace vehicle_profile", ""))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Verify the header without rewriting it")
    args = parser.parse_args()
    content = render_header()
    if args.check:
        if not HEADER.is_file() or HEADER.read_text(encoding="utf-8") != content:
            parser.exit(1, "VehicleProfile.h is out of date. Run python -m hardware.generate_firmware_profile.\n")
    else:
        HEADER.parent.mkdir(parents=True, exist_ok=True)
        HEADER.write_text(content, encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
