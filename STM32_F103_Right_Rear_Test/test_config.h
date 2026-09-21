#ifndef RIGHT_REAR_TEST_CONFIG_H
#define RIGHT_REAR_TEST_CONFIG_H

#include "rc_config.h"

/*
 * This test intentionally supports only the currently installed right-rear
 * drive channel. Refuse to build if the shared pin map changes underneath the
 * test: silently driving a different timer channel would be unsafe.
 */
#if (RC_RIGHT_REAR_MOTOR_ENABLED != 1U)
#error "Right-rear test requires the shared right-drive pin map to be enabled"
#endif

#if (RC_RIGHT_MOTOR_PWM_PIN != 7U) || \
    (RC_RIGHT_MOTOR_IN1_PIN != 10U) || \
    (RC_RIGHT_MOTOR_IN2_PIN != 13U) || \
    (RC_RIGHT_MOTOR_ENABLE_PIN != 11U)
#error "Right-rear motor pin map no longer matches TIM3_CH2/PA7 and GPIOB controls"
#endif

#if (RC_RIGHT_ENCODER_A_PIN != 8U) || (RC_RIGHT_ENCODER_B_PIN != 9U)
#error "Right encoder pin map no longer matches TIM1_CH1/CH2 on PA8/PA9"
#endif

#if (RC_LEFT_MOTOR_ENABLE_PIN != 1U)
#error "Left-drive lockout expects the active-low ENABLE input on PB1"
#endif

#if (RC_MOTOR_ENABLE_ACTIVE_LOW != 1U)
#error "This test's hardware lockout expects active-low motor ENABLE inputs"
#endif

#define RR_TEST_FIRMWARE_VERSION       "right-rear-rb35gm-test-v2-20260915"
#define RR_TEST_UART_BAUD              115200UL
#define RR_TEST_REPORT_PERIOD_MS       250UL
#define RR_TEST_DIRECTION_PAUSE_MS     100UL
#define RR_TEST_PULSE_MS               1000UL
#define RR_TEST_MAX_DUTY_PERCENT       10U
#define RR_TEST_DEFAULT_DUTY_PERCENT   10U

/* RB35GM working assumption only: 13 periods/channel * x4 = 52 counts. */
#define RR_TEST_ENCODER_COUNTS_PER_MOTOR_REV 52UL

#if (RR_TEST_DEFAULT_DUTY_PERCENT > RR_TEST_MAX_DUTY_PERCENT)
#error "Default test duty exceeds the test safety limit"
#endif

#if (RR_TEST_MAX_DUTY_PERCENT > 10U)
#error "Uncalibrated RB35GM test duty must remain at or below 10 percent"
#endif

#if (RR_TEST_PULSE_MS > 2000UL) || (RR_TEST_DIRECTION_PAUSE_MS < 50UL)
#error "Right-rear test timing is outside the intended safety bounds"
#endif

#endif
