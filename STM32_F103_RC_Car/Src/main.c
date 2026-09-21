#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#include "rc_config.h"
#include "stm32f103_min.h"

#define RC_MOTOR_GPIOB_PIN_MASK \
    ((1UL << RC_LEFT_MOTOR_IN1_PIN) | \
     (1UL << RC_LEFT_MOTOR_IN2_PIN) | \
     (1UL << RC_LEFT_MOTOR_ENABLE_PIN) | \
     (1UL << RC_RIGHT_MOTOR_IN1_PIN) | \
     (1UL << RC_RIGHT_MOTOR_IN2_PIN) | \
     (1UL << RC_RIGHT_MOTOR_ENABLE_PIN))
#define RC_MOTOR_GPIOB_PIN_SUM \
    ((1UL << RC_LEFT_MOTOR_IN1_PIN) + \
     (1UL << RC_LEFT_MOTOR_IN2_PIN) + \
     (1UL << RC_LEFT_MOTOR_ENABLE_PIN) + \
     (1UL << RC_RIGHT_MOTOR_IN1_PIN) + \
     (1UL << RC_RIGHT_MOTOR_IN2_PIN) + \
     (1UL << RC_RIGHT_MOTOR_ENABLE_PIN))

_Static_assert(RC_LEFT_MOTOR_IN1_PIN < 16U &&
               RC_LEFT_MOTOR_IN2_PIN < 16U &&
               RC_LEFT_MOTOR_ENABLE_PIN < 16U &&
               RC_RIGHT_MOTOR_IN1_PIN < 16U &&
               RC_RIGHT_MOTOR_IN2_PIN < 16U &&
               RC_RIGHT_MOTOR_ENABLE_PIN < 16U,
               "motor GPIOB pin number out of range");
_Static_assert(RC_MOTOR_GPIOB_PIN_MASK == RC_MOTOR_GPIOB_PIN_SUM,
               "motor GPIOB pins must be unique");
_Static_assert(RC_ENCODER_FEEDBACK_ENABLED == 0U ||
               RC_ENCODER_FEEDBACK_ENABLED == 1U,
               "RC_ENCODER_FEEDBACK_ENABLED must be 0 or 1");
_Static_assert(RC_DRIVE_CALIBRATION_READY == 0U ||
               RC_DRIVE_CALIBRATION_READY == 1U,
               "RC_DRIVE_CALIBRATION_READY must be 0 or 1");
_Static_assert(RC_LEFT_REAR_MOTOR_ENABLED == 0U ||
               RC_LEFT_REAR_MOTOR_ENABLED == 1U,
               "RC_LEFT_REAR_MOTOR_ENABLED must be 0 or 1");
_Static_assert(RC_RIGHT_REAR_MOTOR_ENABLED == 0U ||
               RC_RIGHT_REAR_MOTOR_ENABLED == 1U,
               "RC_RIGHT_REAR_MOTOR_ENABLED must be 0 or 1");
_Static_assert(RC_MOTOR_PWM_HZ > 0UL && RC_MOTOR_PWM_HZ <= RC_SYSCLK_HZ,
               "invalid motor PWM frequency");

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    uint8_t first;
} PidController;

typedef struct
{
    float target_speed_mps;
    float center_speed_mps;
    float measured_speed_mps;
    float steering_deg;
    float turn_radius_m;
    float yaw_rate_radps;
    float front_left_steer_deg;
    float front_right_steer_deg;
    float target_left_wheel_speed_mps;
    float target_right_wheel_speed_mps;
    float target_left_motor_rpm;
    float target_right_motor_rpm;
    float measured_left_motor_rpm;
    float measured_right_motor_rpm;
    float left_motor_duty_percent;
    float right_motor_duty_percent;
} VehicleState;

typedef enum
{
    MOTOR_LEFT_SIDE = 0,
    MOTOR_RIGHT_SIDE = 1
} MotorSide;

static volatile uint32_t g_ms;
static VehicleState vehicle;
static PidController left_speed_pid;
static PidController right_speed_pid;
static uint32_t last_command_ms;
#if RC_LEFT_ENCODER_FEEDBACK_ENABLED
static uint16_t left_encoder_prev_count;
#endif
#if RC_RIGHT_ENCODER_FEEDBACK_ENABLED
static uint16_t right_encoder_prev_count;
#endif
static uint8_t stopped_by_timeout = 1U;
static uint8_t motor_open_loop;
static uint8_t servo_pwm_enabled;
static uint32_t command_count;
static uint32_t ignored_line_count;
static uint32_t center_command_count;
static uint32_t zero_steer_command_count;
static uint32_t timeout_stop_count;
static volatile uint32_t uart_error_count;
static volatile uint32_t uart_rx_overflow_count;
static uint8_t echo_enabled = 1U;
static uint8_t ack_enabled = 1U;
static uint32_t command_timeout_ms = RC_COMMAND_TIMEOUT_MS;
static uint32_t last_servo_left_us;
static uint32_t last_servo_right_us;
static int32_t last_command_steer_cdeg;
static char last_command_name = '-';
static uint8_t left_motor_forward = 1U;
static uint8_t right_motor_forward = 1U;
static uint8_t left_motor_enabled;
static uint8_t right_motor_enabled;
static uint32_t left_motor_resume_ms;
static uint32_t right_motor_resume_ms;
static uint32_t retained_magic __attribute__((section(".noinit")));
static uint32_t retained_boot_count __attribute__((section(".noinit")));
static uint32_t retained_servo_left_us __attribute__((section(".noinit")));
static uint32_t retained_servo_right_us __attribute__((section(".noinit")));

#define RC_PI                       3.1415926f
#define RC_RAD_TO_DEG               57.29578f
#define RC_DEG_TO_RAD               0.017453292f
#define RC_STRAIGHT_RADIUS_M        1.0e9f
#define RC_MIN_RADIUS_MARGIN_M      0.001f
/* Bump when the retained servo representation/calibration changes so the
 * first reset after a firmware update starts from the new neutral pulses. */
#define RC_RETAINED_MAGIC           0x504F5755UL
#define RC_DIRECTION_CHANGE_PAUSE_MS 50U

/* Single-producer (USART2 IRQ) / single-consumer (main loop) ring buffer so
 * bytes arriving during control ticks or long blocking transmits (e.g. the
 * STATUS reply) are never lost to a 1-byte-data-register overrun. */
#define RC_UART_RX_RING_SIZE        128U
#define RC_UART_RX_RING_MASK        (RC_UART_RX_RING_SIZE - 1U)

static volatile uint8_t uart_rx_ring[RC_UART_RX_RING_SIZE];
static volatile uint32_t uart_rx_head;
static volatile uint32_t uart_rx_tail;

static void motor_set_duty(MotorSide motor, float duty_percent);

static float clampf(float value, float low, float high)
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

static float absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return ((now_ms - deadline_ms) < 0x80000000UL) ? 1U : 0U;
}

static void retained_init(void)
{
    if (retained_magic != RC_RETAINED_MAGIC)
    {
        retained_magic = RC_RETAINED_MAGIC;
        retained_boot_count = 0U;
        /* Cold boot only: park steering at neutral. Warm resets keep the
         * last commanded pulse so the servo holds its angle through a glitch. */
        retained_servo_left_us = (uint32_t)RC_SERVO_LEFT_NEUTRAL_US;
        retained_servo_right_us = (uint32_t)RC_SERVO_RIGHT_NEUTRAL_US;
    }

    /* Guard against corrupted retained values before they drive the servo. */
    retained_servo_left_us =
        (uint32_t)clampf((float)retained_servo_left_us, RC_SERVO_MIN_US, RC_SERVO_MAX_US);
    retained_servo_right_us =
        (uint32_t)clampf((float)retained_servo_right_us, RC_SERVO_MIN_US, RC_SERVO_MAX_US);

    retained_boot_count++;
}

void SysTick_Handler(void)
{
    g_ms++;
}

void SystemInit(void)
{
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U)
    {
    }

    RCC->CFGR = 0U;
    SCB->VTOR = FLASH_BASE;
}

static void gpio_config_pin(GPIO_TypeDef *gpio, uint32_t pin, uint32_t config)
{
    __IO uint32_t *reg;
    uint32_t shift;

    if (pin < 8U)
    {
        reg = &gpio->CRL;
        shift = pin * 4U;
    }
    else
    {
        reg = &gpio->CRH;
        shift = (pin - 8U) * 4U;
    }

    *reg &= ~(0xFUL << shift);
    *reg |= ((config & 0xFUL) << shift);
}

static void gpio_config_output(GPIO_TypeDef *gpio, uint32_t pin)
{
    gpio_config_pin(gpio, pin, 0x2U);  /* Output push-pull, 2 MHz. */
}

static void gpio_config_af_output(GPIO_TypeDef *gpio, uint32_t pin)
{
    gpio_config_pin(gpio, pin, 0xAU);  /* Alternate-function push-pull, 2 MHz. */
}

static void gpio_config_input_floating(GPIO_TypeDef *gpio, uint32_t pin)
{
    gpio_config_pin(gpio, pin, 0x4U);
}

static void gpio_write(GPIO_TypeDef *gpio, uint32_t pin, uint32_t high)
{
    if (high != 0U)
    {
        gpio->BSRR = (1UL << pin);
    }
    else
    {
        gpio->BRR = (1UL << pin);
    }
}

static uint32_t gpio_read_output(GPIO_TypeDef *gpio, uint32_t pin)
{
    return (gpio->ODR >> pin) & 1UL;
}

static uint32_t usart_brr(uint32_t clock_hz, uint32_t baud)
{
    return (clock_hz + (baud / 2U)) / baud;
}

static void clock_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN |
                    RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN |
                    RCC_APB1ENR_TIM4EN |
                    RCC_APB1ENR_USART2EN;
#if RC_RIGHT_ENCODER_FEEDBACK_ENABLED
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
#endif
#if RC_LEFT_ENCODER_FEEDBACK_ENABLED
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
#endif

    (void)RCC->APB2ENR;
    (void)RCC->APB1ENR;

    SYST_RVR = (RC_SYSCLK_HZ / 1000UL) - 1UL;
    SYST_CVR = 0U;
    SYST_CSR = 7U;
}

static void uart_putc(char ch)
{
    while ((USART2->SR & USART_SR_TXE) == 0U)
    {
    }
    USART2->DR = (uint32_t)(uint8_t)ch;
}

static void uart_write(const char *text)
{
    while (*text != '\0')
    {
        uart_putc(*text++);
    }
}

void USART2_IRQHandler(void)
{
    uint32_t status = USART2->SR;

    if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE)) != 0U)
    {
        /* Reading SR then DR clears RXNE and the error flags. A byte flagged
         * with a line error is still delivered: dropping it would corrupt the
         * whole command line for a single noise glitch. */
        uint8_t byte = (uint8_t)USART2->DR;

        if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) != 0U)
        {
            uart_error_count++;
        }

        if ((status & USART_SR_RXNE) != 0U)
        {
            uint32_t next_head = (uart_rx_head + 1U) & RC_UART_RX_RING_MASK;

            if (next_head != uart_rx_tail)
            {
                uart_rx_ring[uart_rx_head] = byte;
                uart_rx_head = next_head;
            }
            else
            {
                uart_rx_overflow_count++;
            }
        }
    }
}

static uint8_t uart_read_byte(uint8_t *byte)
{
    if (uart_rx_tail == uart_rx_head)
    {
        return 0U;
    }

    *byte = uart_rx_ring[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1U) & RC_UART_RX_RING_MASK;
    return 1U;
}

static void uart_write_int(int32_t value)
{
    char buf[12];
    int32_t i = 0;
    uint32_t n;

    if (value < 0)
    {
        uart_putc('-');
        n = (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        n = (uint32_t)value;
    }

    do
    {
        buf[i++] = (char)('0' + (n % 10U));
        n /= 10U;
    } while (n > 0U && i < (int32_t)sizeof(buf));

    while (i > 0)
    {
        uart_putc(buf[--i]);
    }
}

static void uart_write_uint(uint32_t value)
{
    char buf[10];
    int32_t i = 0;

    do
    {
        buf[i++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value > 0U && i < (int32_t)sizeof(buf));

    while (i > 0)
    {
        uart_putc(buf[--i]);
    }
}

static void uart_init(void)
{
    gpio_config_af_output(GPIOA, 2U);
    gpio_config_input_floating(GPIOA, 3U);

    USART2->CR1 = 0U;
    USART2->BRR = usart_brr(RC_SYSCLK_HZ, 115200UL);
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_ISER1 = (1UL << (USART2_IRQ_NUMBER - 32UL));
}

static void uart_echo(char ch)
{
    if (echo_enabled != 0U)
    {
        uart_putc(ch);
    }
}

static void servo_enable_pwm(void)
{
    if (servo_pwm_enabled == 0U)
    {
        TIM4->EGR = TIM_EGR_UG;
        gpio_config_af_output(GPIOB, 8U);
        gpio_config_af_output(GPIOB, 9U);
        TIM4->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
        servo_pwm_enabled = 1U;
    }
}

static void servo_init(void)
{
    /* Both servos on TIM4: left = CH3 (D15/PB8), right = CH4 (D14/PB9).
     * TIM3 is reserved for rear motor PWM (PA6, PA7).
     *
     * Output is enabled immediately with the retained last-commanded pulse so
     * that after a brown-out / glitch reset the servo never loses its drive
     * signal and snaps to neutral; it resumes its prior angle on the first
     * timer cycle instead of waiting for the host to resend a steering command. */
    gpio_config_af_output(GPIOB, 8U);
    gpio_config_af_output(GPIOB, 9U);

    TIM4->PSC = 8U - 1U;
    TIM4->ARR = 20000U - 1U;
    TIM4->CCMR2 = (6UL << 4) | TIM_CCMR_OC3PE |
                  (6UL << 12) | TIM_CCMR_OC4PE;
    TIM4->CCR3 = retained_servo_left_us;
    TIM4->CCR4 = retained_servo_right_us;
    TIM4->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    last_servo_left_us = retained_servo_left_us;
    last_servo_right_us = retained_servo_right_us;
    servo_pwm_enabled = 1U;
}

static void servo_set(float left_deg, float right_deg)
{
    float left_us;
    float right_us;
    uint32_t left_compare;
    uint32_t right_compare;

    left_deg = clampf(left_deg, RC_SERVO_MIN_STEER_DEG, RC_SERVO_MAX_STEER_DEG);
    right_deg = clampf(right_deg, RC_SERVO_MIN_STEER_DEG, RC_SERVO_MAX_STEER_DEG);

    left_us = RC_SERVO_LEFT_NEUTRAL_US + (RC_SERVO_LEFT_DIR * left_deg * RC_SERVO_US_PER_DEG);
    right_us = RC_SERVO_RIGHT_NEUTRAL_US + (RC_SERVO_RIGHT_DIR * right_deg * RC_SERVO_US_PER_DEG);

    left_us = clampf(left_us, RC_SERVO_MIN_US, RC_SERVO_MAX_US);
    right_us = clampf(right_us, RC_SERVO_MIN_US, RC_SERVO_MAX_US);

    left_compare = (uint32_t)(left_us + 0.5f);
    right_compare = (uint32_t)(right_us + 0.5f);
    last_servo_left_us = left_compare;
    last_servo_right_us = right_compare;
    retained_servo_left_us = left_compare;
    retained_servo_right_us = right_compare;

    if (TIM4->CCR3 != left_compare || TIM4->CCR4 != right_compare ||
        servo_pwm_enabled == 0U)
    {
        TIM4->CCR3 = left_compare;
        TIM4->CCR4 = right_compare;
    }
    servo_enable_pwm();
}

static void servo_set_us(float left_us, float right_us)
{
    float left_limit_a;
    float left_limit_b;
    float right_limit_a;
    float right_limit_b;
    float swap;
    uint32_t left_compare;
    uint32_t right_compare;

    left_limit_a = RC_SERVO_LEFT_NEUTRAL_US +
                   (RC_SERVO_LEFT_DIR * RC_SERVO_MIN_STEER_DEG * RC_SERVO_US_PER_DEG);
    left_limit_b = RC_SERVO_LEFT_NEUTRAL_US +
                   (RC_SERVO_LEFT_DIR * RC_SERVO_MAX_STEER_DEG * RC_SERVO_US_PER_DEG);
    right_limit_a = RC_SERVO_RIGHT_NEUTRAL_US +
                    (RC_SERVO_RIGHT_DIR * RC_SERVO_MIN_STEER_DEG * RC_SERVO_US_PER_DEG);
    right_limit_b = RC_SERVO_RIGHT_NEUTRAL_US +
                    (RC_SERVO_RIGHT_DIR * RC_SERVO_MAX_STEER_DEG * RC_SERVO_US_PER_DEG);

    if (left_limit_a > left_limit_b)
    {
        swap = left_limit_a;
        left_limit_a = left_limit_b;
        left_limit_b = swap;
    }
    if (right_limit_a > right_limit_b)
    {
        swap = right_limit_a;
        right_limit_a = right_limit_b;
        right_limit_b = swap;
    }

    left_us = clampf(left_us,
                     clampf(left_limit_a, RC_SERVO_MIN_US, RC_SERVO_MAX_US),
                     clampf(left_limit_b, RC_SERVO_MIN_US, RC_SERVO_MAX_US));
    right_us = clampf(right_us,
                      clampf(right_limit_a, RC_SERVO_MIN_US, RC_SERVO_MAX_US),
                      clampf(right_limit_b, RC_SERVO_MIN_US, RC_SERVO_MAX_US));
    left_compare = (uint32_t)(left_us + 0.5f);
    right_compare = (uint32_t)(right_us + 0.5f);

    last_servo_left_us = left_compare;
    last_servo_right_us = right_compare;
    retained_servo_left_us = left_compare;
    retained_servo_right_us = right_compare;
    TIM4->CCR3 = left_compare;
    TIM4->CCR4 = right_compare;
    servo_enable_pwm();
}

static void motor_init(void)
{
#if RC_REAR_MOTOR_PWM_ENABLED
    uint32_t arr;
#endif

    /* The MAI-2MT-DC ENABLE inputs are active-low. Preload the disabled
     * levels before changing the GPIO modes so a reset cannot briefly start
     * either wheel. Direction inputs are also preloaded low. */
#if RC_MOTOR_ENABLE_ACTIVE_LOW
    gpio_write(GPIOB, RC_LEFT_MOTOR_ENABLE_PIN, 1U);
    gpio_write(GPIOB, RC_RIGHT_MOTOR_ENABLE_PIN, 1U);
#else
    gpio_write(GPIOB, RC_LEFT_MOTOR_ENABLE_PIN, 0U);
    gpio_write(GPIOB, RC_RIGHT_MOTOR_ENABLE_PIN, 0U);
#endif
    gpio_write(GPIOB, RC_LEFT_MOTOR_IN1_PIN, 0U);
    gpio_write(GPIOB, RC_LEFT_MOTOR_IN2_PIN, 0U);
    gpio_write(GPIOB, RC_RIGHT_MOTOR_IN1_PIN, 0U);
    gpio_write(GPIOB, RC_RIGHT_MOTOR_IN2_PIN, 0U);

    gpio_config_output(GPIOB, RC_LEFT_MOTOR_IN1_PIN);
    gpio_config_output(GPIOB, RC_LEFT_MOTOR_IN2_PIN);
    gpio_config_output(GPIOB, RC_LEFT_MOTOR_ENABLE_PIN);
    gpio_config_output(GPIOB, RC_RIGHT_MOTOR_IN1_PIN);
    gpio_config_output(GPIOB, RC_RIGHT_MOTOR_IN2_PIN);
    gpio_config_output(GPIOB, RC_RIGHT_MOTOR_ENABLE_PIN);

#if RC_REAR_MOTOR_PWM_ENABLED
    AFIO->MAPR &= ~AFIO_MAPR_TIM3_REMAP_MASK;
#if RC_LEFT_REAR_MOTOR_ENABLED
    gpio_config_af_output(GPIOA, RC_LEFT_MOTOR_PWM_PIN);
#else
    gpio_config_pin(GPIOA, RC_LEFT_MOTOR_PWM_PIN, 0x0U);
#endif
#if RC_RIGHT_REAR_MOTOR_ENABLED
    gpio_config_af_output(GPIOA, RC_RIGHT_MOTOR_PWM_PIN);
#else
    gpio_config_pin(GPIOA, RC_RIGHT_MOTOR_PWM_PIN, 0x0U);
#endif
#endif

#if RC_REAR_MOTOR_PWM_ENABLED
    arr = (RC_SYSCLK_HZ / RC_MOTOR_PWM_HZ) - 1U;

    TIM3->PSC = 0U;
    TIM3->ARR = arr;
    TIM3->CCMR1 = (6UL << 4) | TIM_CCMR_OC1PE |
                  (6UL << 12) | TIM_CCMR_OC2PE;
    TIM3->CCER = 0U;
#if RC_LEFT_REAR_MOTOR_ENABLED
    TIM3->CCER |= TIM_CCER_CC1E;
#endif
#if RC_RIGHT_REAR_MOTOR_ENABLED
    TIM3->CCER |= TIM_CCER_CC2E;
#endif
    TIM3->CCR1 = 0U;
    TIM3->CCR2 = 0U;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
#endif

    left_motor_forward = 1U;
    right_motor_forward = 1U;
    left_motor_enabled = 0U;
    right_motor_enabled = 0U;
#if RC_LEFT_REAR_MOTOR_ENABLED
    gpio_write(GPIOB, RC_LEFT_MOTOR_IN1_PIN, RC_LEFT_MOTOR_FORWARD_IN1_HIGH);
    gpio_write(GPIOB, RC_LEFT_MOTOR_IN2_PIN, 1U - RC_LEFT_MOTOR_FORWARD_IN1_HIGH);
#endif
#if RC_RIGHT_REAR_MOTOR_ENABLED
    gpio_write(GPIOB,
               RC_RIGHT_MOTOR_IN1_PIN,
               RC_RIGHT_MOTOR_FORWARD_IN1_HIGH);
    gpio_write(GPIOB,
               RC_RIGHT_MOTOR_IN2_PIN,
               1U - RC_RIGHT_MOTOR_FORWARD_IN1_HIGH);
#endif
}

static uint8_t motor_side_is_enabled(MotorSide motor)
{
    return (motor == MOTOR_LEFT_SIDE)
               ? (uint8_t)RC_LEFT_REAR_MOTOR_ENABLED
               : (uint8_t)RC_RIGHT_REAR_MOTOR_ENABLED;
}

static void motor_set_enabled(MotorSide motor, uint8_t enabled)
{
    uint32_t enable_pin = (motor == MOTOR_LEFT_SIDE)
                              ? RC_LEFT_MOTOR_ENABLE_PIN
                              : RC_RIGHT_MOTOR_ENABLE_PIN;
    uint32_t high;
    uint8_t *current_enabled = (motor == MOTOR_LEFT_SIDE)
                                   ? &left_motor_enabled
                                   : &right_motor_enabled;

    if (motor_side_is_enabled(motor) == 0U)
    {
        enabled = 0U;
    }

#if RC_MOTOR_ENABLE_ACTIVE_LOW
    high = (enabled != 0U) ? 0U : 1U;
#else
    high = (enabled != 0U) ? 1U : 0U;
#endif
    gpio_write(GPIOB, enable_pin, high);
    *current_enabled = (enabled != 0U) ? 1U : 0U;
}

static void motor_write_direction_pins(MotorSide motor, uint8_t forward)
{
    uint32_t in1_pin = (motor == MOTOR_LEFT_SIDE)
                           ? RC_LEFT_MOTOR_IN1_PIN
                           : RC_RIGHT_MOTOR_IN1_PIN;
    uint32_t in2_pin = (motor == MOTOR_LEFT_SIDE)
                           ? RC_LEFT_MOTOR_IN2_PIN
                           : RC_RIGHT_MOTOR_IN2_PIN;
    uint32_t forward_in1_high = (motor == MOTOR_LEFT_SIDE)
                                    ? RC_LEFT_MOTOR_FORWARD_IN1_HIGH
                                    : RC_RIGHT_MOTOR_FORWARD_IN1_HIGH;
    uint32_t in1_high = (forward != 0U)
                            ? forward_in1_high
                            : (1U - forward_in1_high);

    gpio_write(GPIOB, in1_pin, in1_high);
    gpio_write(GPIOB, in2_pin, 1U - in1_high);
}

static uint8_t motor_set_direction(MotorSide motor, uint8_t forward)
{
    uint8_t *current_forward = (motor == MOTOR_LEFT_SIDE)
                                   ? &left_motor_forward
                                   : &right_motor_forward;
    uint32_t *resume_ms = (motor == MOTOR_LEFT_SIDE)
                              ? &left_motor_resume_ms
                              : &right_motor_resume_ms;

    if (motor_side_is_enabled(motor) == 0U)
    {
        motor_set_duty(motor, 0.0f);
        return 0U;
    }

    forward = (forward != 0U) ? 1U : 0U;
    uint8_t changed = (*current_forward != forward) ? 1U : 0U;

    if (changed != 0U)
    {
        motor_set_duty(motor, 0.0f);
        *current_forward = forward;
        *resume_ms = g_ms + RC_DIRECTION_CHANGE_PAUSE_MS;
    }

    /* IN1 and IN2 must always be complementary while driving. Rewriting them
     * even when the direction did not change also restores them after STOP. */
    motor_write_direction_pins(motor, forward);
    return changed;
}

static void motor_set_duty(MotorSide motor, float duty_percent)
{
#if RC_REAR_MOTOR_PWM_ENABLED
    uint32_t arr = TIM3->ARR + 1U;
    uint32_t ccr;
    uint8_t *current_enabled = (motor == MOTOR_LEFT_SIDE)
                                   ? &left_motor_enabled
                                   : &right_motor_enabled;
    uint8_t current_forward = (motor == MOTOR_LEFT_SIDE)
                                  ? left_motor_forward
                                  : right_motor_forward;

    if (motor_side_is_enabled(motor) == 0U)
    {
        if (motor == MOTOR_LEFT_SIDE)
        {
            TIM3->CCR1 = 0U;
        }
        else
        {
            TIM3->CCR2 = 0U;
        }
        motor_set_enabled(motor, 0U);
        return;
    }

    duty_percent = clampf(duty_percent, 0.0f, 100.0f);
    ccr = (uint32_t)(((float)arr * duty_percent / 100.0f) + 0.5f);
    if (ccr > TIM3->ARR)
    {
        ccr = TIM3->ARR;
    }

    if (ccr == 0U)
    {
        /* Disable immediately; CCR is preloaded and may not reach the timer
         * shadow register until the next 20 kHz update event. */
        motor_set_enabled(motor, 0U);
        if (motor == MOTOR_LEFT_SIDE)
        {
            TIM3->CCR1 = 0U;
        }
        else
        {
            TIM3->CCR2 = 0U;
        }
        return;
    }

    /* Recover a valid complementary direction state even if STOP previously
     * parked both IN pins low. Never enable the bridge around an unknown
     * input state. */
    motor_write_direction_pins(motor, current_forward);

    if (motor == MOTOR_LEFT_SIDE)
    {
        TIM3->CCR1 = ccr;
    }
    else
    {
        TIM3->CCR2 = ccr;
    }

    if (*current_enabled == 0U)
    {
        /* Transfer the preloaded CCR before active-low ENABLE is asserted.
         * This prevents a stale previous duty from appearing for one PWM
         * period when a stopped wheel is started again. */
        TIM3->EGR = TIM_EGR_UG;
        motor_set_enabled(motor, 1U);
    }
#else
    (void)motor;
    (void)duty_percent;
#endif
}

static void motor_stop_all(void)
{
    motor_set_duty(MOTOR_LEFT_SIDE, 0.0f);
    motor_set_duty(MOTOR_RIGHT_SIDE, 0.0f);
    gpio_write(GPIOB, RC_LEFT_MOTOR_IN1_PIN, 0U);
    gpio_write(GPIOB, RC_LEFT_MOTOR_IN2_PIN, 0U);
    gpio_write(GPIOB, RC_RIGHT_MOTOR_IN1_PIN, 0U);
    gpio_write(GPIOB, RC_RIGHT_MOTOR_IN2_PIN, 0U);
}

static void encoder_init(void)
{
#if RC_LEFT_ENCODER_FEEDBACK_ENABLED
    gpio_config_input_floating(GPIOA, RC_LEFT_ENCODER_A_PIN);
    gpio_config_input_floating(GPIOA, RC_LEFT_ENCODER_B_PIN);
    TIM2->PSC = 0U;
    TIM2->ARR = 0xFFFFUL;
    TIM2->CCMR1 = 1UL |
                  ((uint32_t)RC_ENCODER_INPUT_FILTER << 4) |
                  (1UL << 8) |
                  ((uint32_t)RC_ENCODER_INPUT_FILTER << 12);
    TIM2->CCER = 0U;
    TIM2->SMCR = TIM_SMCR_SMS_ENCODER_3;
    TIM2->CNT = 0U;
    TIM2->CR1 = TIM_CR1_CEN;
    left_encoder_prev_count = 0U;
#else
    gpio_config_pin(GPIOA, RC_LEFT_ENCODER_A_PIN, 0x0U);
    gpio_config_pin(GPIOA, RC_LEFT_ENCODER_B_PIN, 0x0U);
#endif

#if RC_RIGHT_ENCODER_FEEDBACK_ENABLED
    /* Installed right encoder harness:
     * brown=+5 V, green=GND, lavender=A/PA8, blue=B/PA9.
     * PA8 and PA9 are STM32F103RB 5 V-tolerant inputs. Floating mode leaves
     * the encoder output or its external pull-up in control of the level. */
    gpio_config_input_floating(GPIOA, RC_RIGHT_ENCODER_A_PIN);
    gpio_config_input_floating(GPIOA, RC_RIGHT_ENCODER_B_PIN);
    TIM1->PSC = 0U;
    TIM1->ARR = 0xFFFFUL;
    TIM1->CCMR1 = 1UL |
                  ((uint32_t)RC_ENCODER_INPUT_FILTER << 4) |
                  (1UL << 8) |
                  ((uint32_t)RC_ENCODER_INPUT_FILTER << 12);
    TIM1->CCER = 0U;
    TIM1->SMCR = TIM_SMCR_SMS_ENCODER_3;
    TIM1->CNT = 0U;
    TIM1->CR1 = TIM_CR1_CEN;
    right_encoder_prev_count = 0U;
#else
    gpio_config_pin(GPIOA, RC_RIGHT_ENCODER_A_PIN, 0x0U);
    gpio_config_pin(GPIOA, RC_RIGHT_ENCODER_B_PIN, 0x0U);
#endif
}

static float speed_to_motor_rpm(float speed_mps)
{
    float wheel_rpm = (60.0f * absf(speed_mps)) / (2.0f * RC_PI * RC_REAR_WHEEL_RADIUS_M);
    return wheel_rpm * RC_GEAR_RATIO;
}

#if RC_ANY_ENCODER_FEEDBACK_ENABLED && RC_DRIVE_CALIBRATION_READY
static float motor_rpm_to_speed(float motor_rpm)
{
    return ((motor_rpm / RC_GEAR_RATIO) * (2.0f * RC_PI * RC_REAR_WHEEL_RADIUS_M)) /
           60.0f;
}
#endif

#if RC_ANY_OPEN_LOOP_DRIVE_ENABLED
static float speed_to_open_loop_duty(float speed_mps)
{
    float magnitude = absf(speed_mps);
    float normalized;

    if (magnitude < 0.001f)
    {
        return 0.0f;
    }

    normalized = clampf(magnitude / RC_MAX_SPEED_MPS, 0.0f, 1.0f);
    return RC_OPEN_LOOP_START_DUTY_PERCENT +
           ((100.0f - RC_OPEN_LOOP_START_DUTY_PERCENT) * normalized);
}
#endif

static uint8_t is_straight_radius(float radius_m)
{
    return (absf(radius_m) > (RC_STRAIGHT_RADIUS_M * 0.5f)) ? 1U : 0U;
}

static void pid_reset(PidController *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->first = 1U;
}

#if RC_ANY_ENCODER_FEEDBACK_ENABLED
static float pid_update(PidController *pid, float target, float measured, float dt_sec)
{
    float error = target - measured;
    float derivative = 0.0f;
    float output;

    /* `integral` accumulates the I-term in duty-percent directly (ki already
     * applied). Clamping the raw error integral to [0,100] would cap the
     * sustainable steady-state duty at ki*100 = 20%, which cannot hold the
     * target speed under load. */
    pid->integral += pid->ki * error * dt_sec;
    pid->integral = clampf(pid->integral, 0.0f, 100.0f);

    if (pid->first == 0U)
    {
        derivative = (error - pid->prev_error) / dt_sec;
    }
    else
    {
        pid->first = 0U;
    }

    output = (pid->kp * error) + pid->integral + (pid->kd * derivative);
    output = clampf(output, 0.0f, 100.0f);
    pid->prev_error = error;
    return output;
}
#endif

static void update_rear_motor_targets(void)
{
    float half_track = RC_REAR_TRACK_M * 0.5f;
    float left_speed = vehicle.target_speed_mps;
    float right_speed = vehicle.target_speed_mps;

    /* Any closed-loop motion command leaves the raw open-loop test mode. */
    motor_open_loop = 0U;

    vehicle.center_speed_mps = vehicle.target_speed_mps;
    vehicle.yaw_rate_radps = 0.0f;

    if (is_straight_radius(vehicle.turn_radius_m) == 0U)
    {
        float abs_radius = absf(vehicle.turn_radius_m);
        float inner_ratio;
        float outer_ratio;

        if (abs_radius <= half_track)
        {
            abs_radius = half_track + RC_MIN_RADIUS_MARGIN_M;
        }

        inner_ratio = (abs_radius - half_track) / abs_radius;
        outer_ratio = (abs_radius + half_track) / abs_radius;
        vehicle.yaw_rate_radps = vehicle.target_speed_mps / vehicle.turn_radius_m;

        if (vehicle.turn_radius_m > 0.0f)
        {
            left_speed = vehicle.target_speed_mps * inner_ratio;
            right_speed = vehicle.target_speed_mps * outer_ratio;
        }
        else
        {
            left_speed = vehicle.target_speed_mps * outer_ratio;
            right_speed = vehicle.target_speed_mps * inner_ratio;
        }
    }

    vehicle.target_left_wheel_speed_mps = left_speed;
    vehicle.target_right_wheel_speed_mps = right_speed;
#if RC_LEFT_REAR_MOTOR_ENABLED
    vehicle.target_left_motor_rpm = speed_to_motor_rpm(left_speed);
#else
    vehicle.target_left_wheel_speed_mps = 0.0f;
    vehicle.target_left_motor_rpm = 0.0f;
#endif
#if RC_RIGHT_REAR_MOTOR_ENABLED
    vehicle.target_right_motor_rpm = speed_to_motor_rpm(right_speed);
#else
    vehicle.target_right_wheel_speed_mps = 0.0f;
    vehicle.target_right_motor_rpm = 0.0f;
#endif
#if RC_LEFT_REAR_MOTOR_ENABLED
    if (motor_set_direction(MOTOR_LEFT_SIDE, left_speed >= 0.0f) != 0U)
    {
        pid_reset(&left_speed_pid);
    }
#endif
#if RC_RIGHT_REAR_MOTOR_ENABLED
    if (motor_set_direction(MOTOR_RIGHT_SIDE, right_speed >= 0.0f) != 0U)
    {
        pid_reset(&right_speed_pid);
    }
#endif
}

static void apply_front_steering_from_radius(float radius_m, float center_steer_deg)
{
    float abs_radius;
    float inner_rad;
    float outer_rad;
    float left_deg;
    float right_deg;

    vehicle.steering_deg = clampf(center_steer_deg, RC_MIN_STEER_DEG, RC_MAX_STEER_DEG);

    if (absf(radius_m) < RC_MIN_RADIUS_MARGIN_M ||
        is_straight_radius(radius_m) != 0U)
    {
        vehicle.turn_radius_m = RC_STRAIGHT_RADIUS_M;
        vehicle.front_left_steer_deg = 0.0f;
        vehicle.front_right_steer_deg = 0.0f;
        servo_set(0.0f, 0.0f);
        update_rear_motor_targets();
        return;
    }

    vehicle.turn_radius_m = radius_m;
    abs_radius = absf(radius_m);
    if (abs_radius <= (RC_FRONT_TRACK_M * 0.5f))
    {
        abs_radius = RC_FRONT_TRACK_M * 0.5f + RC_MIN_RADIUS_MARGIN_M;
    }

    inner_rad = atanf(RC_WHEELBASE_M / (abs_radius - (RC_FRONT_TRACK_M * 0.5f)));
    outer_rad = atanf(RC_WHEELBASE_M / (abs_radius + (RC_FRONT_TRACK_M * 0.5f)));

    /* The physical front-servo channels are installed left/right opposite to
     * the logical geometry labels. Swap the inner/outer assignments here so
     * D/right gives the observed left wheel the larger (inner) angle and
     * A/left gives the observed right wheel the larger angle. */
    if (radius_m > 0.0f)
    {
        left_deg = outer_rad * RC_RAD_TO_DEG;
        right_deg = inner_rad * RC_RAD_TO_DEG;
    }
    else
    {
        left_deg = -(inner_rad * RC_RAD_TO_DEG);
        right_deg = -(outer_rad * RC_RAD_TO_DEG);
    }

    vehicle.front_left_steer_deg = clampf(left_deg, RC_MIN_STEER_DEG, RC_MAX_STEER_DEG);
    vehicle.front_right_steer_deg = clampf(right_deg, RC_MIN_STEER_DEG, RC_MAX_STEER_DEG);
    servo_set(vehicle.front_left_steer_deg, vehicle.front_right_steer_deg);
    update_rear_motor_targets();
}

static void vehicle_set_speed(float speed_mps)
{
    speed_mps = clampf(speed_mps, -RC_MAX_SPEED_MPS, RC_MAX_SPEED_MPS);
    vehicle.target_speed_mps = speed_mps;
    update_rear_motor_targets();
}

static void vehicle_set_steering(float steering_deg)
{
    float radius_m;

    steering_deg = clampf(steering_deg, RC_MIN_STEER_DEG, RC_MAX_STEER_DEG);
    if (absf(steering_deg) < 0.001f)
    {
        apply_front_steering_from_radius(RC_STRAIGHT_RADIUS_M, 0.0f);
        return;
    }

    radius_m = RC_WHEELBASE_M / tanf(steering_deg * RC_DEG_TO_RAD);
    apply_front_steering_from_radius(radius_m, steering_deg);
}

static void vehicle_stop_drive_only(void)
{
    motor_open_loop = 0U;
    vehicle.target_speed_mps = 0.0f;
    vehicle.center_speed_mps = 0.0f;
    vehicle.target_left_wheel_speed_mps = 0.0f;
    vehicle.target_right_wheel_speed_mps = 0.0f;
    vehicle.target_left_motor_rpm = 0.0f;
    vehicle.target_right_motor_rpm = 0.0f;
    vehicle.left_motor_duty_percent = 0.0f;
    vehicle.right_motor_duty_percent = 0.0f;
    vehicle.yaw_rate_radps = 0.0f;
    pid_reset(&left_speed_pid);
    pid_reset(&right_speed_pid);
    motor_stop_all();
}

static void vehicle_center_steering(void)
{
    vehicle.steering_deg = 0.0f;
    vehicle.turn_radius_m = RC_STRAIGHT_RADIUS_M;
    vehicle.front_left_steer_deg = 0.0f;
    vehicle.front_right_steer_deg = 0.0f;
    apply_front_steering_from_radius(RC_STRAIGHT_RADIUS_M, 0.0f);
}

static void vehicle_enter_idle(void)
{
    vehicle_stop_drive_only();
    vehicle_center_steering();
}

/*
 * Open-loop motor test: drive each rear motor independently at a fixed duty,
 * bypassing both encoder PID loops.
 */
static void apply_raw_motor(float left_percent, float right_percent)
{
    uint8_t left_forward = (left_percent >= 0.0f) ? 1U : 0U;
    uint8_t right_forward = (right_percent >= 0.0f) ? 1U : 0U;
    float raw_duty_limit = RC_DRIVE_CALIBRATION_READY
                               ? 100.0f
                               : RC_UNCALIBRATED_MAX_RAW_DUTY_PERCENT;
    float left_duty = RC_LEFT_REAR_MOTOR_ENABLED
                          ? clampf(absf(left_percent), 0.0f, raw_duty_limit)
                          : 0.0f;
    float right_duty = RC_RIGHT_REAR_MOTOR_ENABLED
                           ? clampf(absf(right_percent), 0.0f, raw_duty_limit)
                           : 0.0f;
    uint32_t now_ms;

    motor_open_loop = 1U;
#if RC_LEFT_REAR_MOTOR_ENABLED
    (void)motor_set_direction(MOTOR_LEFT_SIDE, left_forward);
#else
    (void)left_forward;
#endif
#if RC_RIGHT_REAR_MOTOR_ENABLED
    (void)motor_set_direction(MOTOR_RIGHT_SIDE, right_forward);
#else
    (void)right_forward;
#endif

    vehicle.left_motor_duty_percent = left_duty;
    vehicle.right_motor_duty_percent = right_duty;

    /* Respect the direction-change pause here too: a plug reversal at full
     * duty in the same millisecond would spike the motor driver. The paused
     * side stays at 0%; vehicle_update applies the stored duty once
     * *_motor_resume_ms has passed. */
    now_ms = g_ms;
    motor_set_duty(MOTOR_LEFT_SIDE,
                   (time_reached(now_ms, left_motor_resume_ms) != 0U) ? left_duty : 0.0f);
    motor_set_duty(MOTOR_RIGHT_SIDE,
                   (time_reached(now_ms, right_motor_resume_ms) != 0U) ? right_duty : 0.0f);

    /* Keep the command timeout active so a runaway raw test self-stops. */
    last_command_ms = g_ms;
    stopped_by_timeout = (left_duty > 0.0f || right_duty > 0.0f) ? 0U : 1U;
}

static void mark_motion_command(void)
{
    last_command_ms = g_ms;
    if (absf(vehicle.target_speed_mps) > 0.001f)
    {
        stopped_by_timeout = 0U;
    }
    else
    {
        stopped_by_timeout = 1U;
    }
}

#if RC_ANY_ENCODER_FEEDBACK_ENABLED
static float encoder_delta_to_motor_rpm(uint16_t current,
                                        uint16_t *previous,
                                        float counts_per_rev,
                                        float dt_sec)
{
    int16_t delta = (int16_t)(current - *previous);
    *previous = current;
    return ((float)delta / counts_per_rev) * (60.0f / dt_sec);
}
#endif

static void vehicle_update(float dt_sec)
{
    uint32_t now_ms = g_ms;
#if !RC_ANY_ENCODER_FEEDBACK_ENABLED
    (void)dt_sec;
#endif
#if RC_LEFT_ENCODER_FEEDBACK_ENABLED
    float raw_left_rpm;

    raw_left_rpm =
        absf(encoder_delta_to_motor_rpm((uint16_t)TIM2->CNT,
                                        &left_encoder_prev_count,
                                        RC_LEFT_ENCODER_COUNTS_PER_REV,
                                        dt_sec));
    vehicle.measured_left_motor_rpm +=
        RC_RPM_FILTER_ALPHA * (raw_left_rpm - vehicle.measured_left_motor_rpm);
#else
    vehicle.measured_left_motor_rpm = 0.0f;
#endif

#if RC_RIGHT_ENCODER_FEEDBACK_ENABLED
    float raw_right_rpm;

    raw_right_rpm =
        absf(encoder_delta_to_motor_rpm((uint16_t)TIM1->CNT,
                                        &right_encoder_prev_count,
                                        RC_RIGHT_ENCODER_COUNTS_PER_REV,
                                        dt_sec));
    vehicle.measured_right_motor_rpm +=
        RC_RPM_FILTER_ALPHA * (raw_right_rpm - vehicle.measured_right_motor_rpm);
#else
    vehicle.measured_right_motor_rpm = 0.0f;
#endif

#if !RC_DRIVE_CALIBRATION_READY
    /* Do not publish a fabricated vehicle speed while the ratio is TBD. */
    vehicle.measured_speed_mps = 0.0f;
#elif RC_LEFT_ENCODER_FEEDBACK_ENABLED && RC_RIGHT_ENCODER_FEEDBACK_ENABLED
    vehicle.measured_speed_mps =
        (motor_rpm_to_speed(vehicle.measured_left_motor_rpm) +
         motor_rpm_to_speed(vehicle.measured_right_motor_rpm)) *
        0.5f;
#elif RC_LEFT_ENCODER_FEEDBACK_ENABLED
    vehicle.measured_speed_mps = motor_rpm_to_speed(vehicle.measured_left_motor_rpm);
#elif RC_RIGHT_ENCODER_FEEDBACK_ENABLED
    vehicle.measured_speed_mps = motor_rpm_to_speed(vehicle.measured_right_motor_rpm);
#else
    vehicle.measured_speed_mps = 0.0f;
#endif

    /* In open-loop test mode the duty is driven directly by @MOTOR; keep the
     * encoder measurements above (useful for diagnostics) but skip the PID.
     * Re-apply the stored duty each tick so a side held at 0% during the
     * direction-change pause picks up its requested duty afterwards. */
    if (motor_open_loop != 0U)
    {
        motor_set_duty(MOTOR_LEFT_SIDE,
                       (time_reached(now_ms, left_motor_resume_ms) != 0U)
                           ? vehicle.left_motor_duty_percent
                           : 0.0f);
        motor_set_duty(MOTOR_RIGHT_SIDE,
                       (time_reached(now_ms, right_motor_resume_ms) != 0U)
                           ? vehicle.right_motor_duty_percent
                           : 0.0f);
        return;
    }

#if RC_LEFT_REAR_MOTOR_ENABLED && !RC_LEFT_ENCODER_FEEDBACK_ENABLED
    vehicle.left_motor_duty_percent =
        speed_to_open_loop_duty(vehicle.target_left_wheel_speed_mps);
    motor_set_duty(MOTOR_LEFT_SIDE,
                   (time_reached(now_ms, left_motor_resume_ms) != 0U)
                       ? vehicle.left_motor_duty_percent
                       : 0.0f);
#elif RC_LEFT_REAR_MOTOR_ENABLED
    if (vehicle.target_left_motor_rpm <= 0.0f)
    {
        vehicle.left_motor_duty_percent = 0.0f;
        motor_set_duty(MOTOR_LEFT_SIDE, 0.0f);
        pid_reset(&left_speed_pid);
    }
    else if (time_reached(now_ms, left_motor_resume_ms) == 0U)
    {
        vehicle.left_motor_duty_percent = 0.0f;
        motor_set_duty(MOTOR_LEFT_SIDE, 0.0f);
    }
    else
    {
        vehicle.left_motor_duty_percent = pid_update(&left_speed_pid,
                                                     vehicle.target_left_motor_rpm,
                                                     vehicle.measured_left_motor_rpm,
                                                     dt_sec);
        motor_set_duty(MOTOR_LEFT_SIDE, vehicle.left_motor_duty_percent);
    }
#else
    vehicle.left_motor_duty_percent = 0.0f;
    motor_set_duty(MOTOR_LEFT_SIDE, 0.0f);
#endif

#if RC_RIGHT_REAR_MOTOR_ENABLED && !RC_RIGHT_ENCODER_FEEDBACK_ENABLED
    vehicle.right_motor_duty_percent =
        speed_to_open_loop_duty(vehicle.target_right_wheel_speed_mps);
    motor_set_duty(MOTOR_RIGHT_SIDE,
                   (time_reached(now_ms, right_motor_resume_ms) != 0U)
                       ? vehicle.right_motor_duty_percent
                       : 0.0f);
#elif RC_RIGHT_REAR_MOTOR_ENABLED
    if (vehicle.target_right_motor_rpm <= 0.0f)
    {
        vehicle.right_motor_duty_percent = 0.0f;
        motor_set_duty(MOTOR_RIGHT_SIDE, 0.0f);
        pid_reset(&right_speed_pid);
    }
    else if (time_reached(now_ms, right_motor_resume_ms) == 0U)
    {
        vehicle.right_motor_duty_percent = 0.0f;
        motor_set_duty(MOTOR_RIGHT_SIDE, 0.0f);
    }
    else
    {
        vehicle.right_motor_duty_percent = pid_update(&right_speed_pid,
                                                      vehicle.target_right_motor_rpm,
                                                      vehicle.measured_right_motor_rpm,
                                                      dt_sec);
        motor_set_duty(MOTOR_RIGHT_SIDE, vehicle.right_motor_duty_percent);
    }
#else
    vehicle.right_motor_duty_percent = 0.0f;
    motor_set_duty(MOTOR_RIGHT_SIDE, 0.0f);
#endif
}

static uint8_t parse_float_checked(const char **cursor, float *out_value)
{
    float sign = 1.0f;
    float value = 0.0f;
    float scale = 0.1f;
    uint8_t digits = 0U;

    while (**cursor == ' ' || **cursor == '\t')
    {
        (*cursor)++;
    }

    if (**cursor == '-')
    {
        sign = -1.0f;
        (*cursor)++;
    }
    else if (**cursor == '+')
    {
        (*cursor)++;
    }

    while (**cursor >= '0' && **cursor <= '9')
    {
        value = (value * 10.0f) + (float)(**cursor - '0');
        (*cursor)++;
        digits = 1U;
    }

    if (**cursor == '.')
    {
        (*cursor)++;
        while (**cursor >= '0' && **cursor <= '9')
        {
            value += scale * (float)(**cursor - '0');
            scale *= 0.1f;
            (*cursor)++;
            digits = 1U;
        }
    }

    if (digits == 0U)
    {
        return 0U;
    }
    if (**cursor != '\0' && **cursor != ' ' && **cursor != '\t')
    {
        return 0U;
    }

    *out_value = sign * value;
    return 1U;
}

static uint8_t word_equal(const char *line, const char *word)
{
    while (*word != '\0')
    {
        char a = *line++;
        char b = *word++;
        if (a >= 'a' && a <= 'z')
        {
            a = (char)(a - 'a' + 'A');
        }
        if (a != b)
        {
            return 0U;
        }
    }
    return (*line == '\0' || *line == ' ' || *line == '\t') ? 1U : 0U;
}

static uint8_t line_ended(const char *cursor)
{
    while (*cursor == ' ' || *cursor == '\t')
    {
        cursor++;
    }
    return (*cursor == '\0') ? 1U : 0U;
}

static const char *after_word(const char *line)
{
    while (*line != '\0' && *line != ' ' && *line != '\t')
    {
        line++;
    }
    return line;
}

static void mark_command(char name, float steer_deg)
{
    command_count++;
    last_command_name = name;
    last_command_steer_cdeg = (int32_t)(steer_deg * 100.0f);
    if (absf(steer_deg) < 0.001f)
    {
        zero_steer_command_count++;
    }
}

static void send_status(void)
{
    uart_write("STATUS fw=");
    uart_write(RC_FIRMWARE_VERSION);
    uart_write(" uptime_ms=");
    uart_write_uint(g_ms);
    uart_write(" boot_count=");
    uart_write_uint(retained_boot_count);
    uart_write(" cmd_count=");
    uart_write_uint(command_count);
    uart_write(" ignored_lines=");
    uart_write_uint(ignored_line_count);
    uart_write(" uart_errors=");
    uart_write_uint(uart_error_count);
    uart_write(" rx_overflow=");
    uart_write_uint(uart_rx_overflow_count);
    uart_write(" timeout_ms=");
    uart_write_uint(command_timeout_ms);
    uart_write(" echo=");
    uart_write_uint((uint32_t)echo_enabled);
    uart_write(" ack=");
    uart_write_uint((uint32_t)ack_enabled);
    uart_write(" center_cmds=");
    uart_write_uint(center_command_count);
    uart_write(" zero_steer_cmds=");
    uart_write_uint(zero_steer_command_count);
    uart_write(" timeout_stops=");
    uart_write_uint(timeout_stop_count);
    uart_write(" last_cmd=");
    uart_putc(last_command_name);
    uart_write(" last_cmd_steer_cdeg=");
    uart_write_int(last_command_steer_cdeg);
    uart_write(" servo_pwm=");
    uart_write_uint((uint32_t)servo_pwm_enabled);
    uart_write(" servo_l_us=");
    uart_write_uint(last_servo_left_us);
    uart_write(" servo_r_us=");
    uart_write_uint(last_servo_right_us);
    uart_write(" target_cmps=");
    uart_write_int((int32_t)(vehicle.target_speed_mps * 100.0f));
    uart_write(" target_l_cmps=");
    uart_write_int((int32_t)(vehicle.target_left_wheel_speed_mps * 100.0f));
    uart_write(" target_r_cmps=");
    uart_write_int((int32_t)(vehicle.target_right_wheel_speed_mps * 100.0f));
    uart_write(" measured_cmps=");
    uart_write_int((int32_t)(vehicle.measured_speed_mps * 100.0f));
    uart_write(" steer_cdeg=");
    uart_write_int((int32_t)(vehicle.steering_deg * 100.0f));
    uart_write(" fl_cdeg=");
    uart_write_int((int32_t)(vehicle.front_left_steer_deg * 100.0f));
    uart_write(" fr_cdeg=");
    uart_write_int((int32_t)(vehicle.front_right_steer_deg * 100.0f));
    uart_write(" target_l_rpm=");
    uart_write_int((int32_t)(vehicle.target_left_motor_rpm + 0.5f));
    uart_write(" target_r_rpm=");
    uart_write_int((int32_t)(vehicle.target_right_motor_rpm + 0.5f));
    uart_write(" measured_l_rpm=");
    uart_write_int((int32_t)(vehicle.measured_left_motor_rpm + 0.5f));
    uart_write(" measured_r_rpm=");
    uart_write_int((int32_t)(vehicle.measured_right_motor_rpm + 0.5f));
    uart_write(" duty_l_cpercent=");
    uart_write_int((int32_t)(vehicle.left_motor_duty_percent * 100.0f));
    uart_write(" duty_r_cpercent=");
    uart_write_int((int32_t)(vehicle.right_motor_duty_percent * 100.0f));
    uart_write(" open_loop=");
    uart_write_uint((uint32_t)motor_open_loop);
    uart_write(" feedback=");
    uart_write_uint((uint32_t)RC_ANY_ENCODER_FEEDBACK_ENABLED);
    uart_write(" drive_l=");
    uart_write_uint((uint32_t)RC_LEFT_REAR_MOTOR_ENABLED);
    uart_write(" drive_r=");
    uart_write_uint((uint32_t)RC_RIGHT_REAR_MOTOR_ENABLED);
    uart_write(" feedback_l=");
    uart_write_uint((uint32_t)RC_LEFT_ENCODER_FEEDBACK_ENABLED);
    uart_write(" feedback_r=");
    uart_write_uint((uint32_t)RC_RIGHT_ENCODER_FEEDBACK_ENABLED);
    uart_write(" motor=RB35GM_09TYPE_26P");
    uart_write(" closed_loop_ready=");
    uart_write_uint((uint32_t)RC_DRIVE_CALIBRATION_READY);
    uart_write(" gear_verified=");
    uart_write_uint((uint32_t)DRIVE_GEAR_RATIO_VERIFIED);
    uart_write(" encoder_counts_verified=");
    uart_write_uint((uint32_t)DRIVE_ENCODER_COUNTS_VERIFIED);
    uart_write(" assumed_counts_motor_rev=52");
    uart_write(" l_in1=");
    uart_write_uint(gpio_read_output(GPIOB, RC_LEFT_MOTOR_IN1_PIN));
    uart_write(" l_in2=");
    uart_write_uint(gpio_read_output(GPIOB, RC_LEFT_MOTOR_IN2_PIN));
    uart_write(" l_en_pin=");
    uart_write_uint(gpio_read_output(GPIOB, RC_LEFT_MOTOR_ENABLE_PIN));
    uart_write(" l_enabled=");
    uart_write_uint((uint32_t)left_motor_enabled);
    uart_write(" l_ccr=");
    uart_write_uint(TIM3->CCR1);
    uart_write(" r_in1=");
    uart_write_uint(gpio_read_output(GPIOB, RC_RIGHT_MOTOR_IN1_PIN));
    uart_write(" r_in2=");
    uart_write_uint(gpio_read_output(GPIOB, RC_RIGHT_MOTOR_IN2_PIN));
    uart_write(" r_en_pin=");
    uart_write_uint(gpio_read_output(GPIOB, RC_RIGHT_MOTOR_ENABLE_PIN));
    uart_write(" r_enabled=");
    uart_write_uint((uint32_t)right_motor_enabled);
    uart_write(" r_ccr=");
    uart_write_uint(TIM3->CCR2);
    uart_write(" pwm_arr=");
    uart_write_uint(TIM3->ARR);
    uart_write("\r\n");
}

static void send_help(void)
{
    uart_write("\r\nNUCLEO-F103RB RC car controller ");
    uart_write(RC_FIRMWARE_VERSION);
    uart_write(" ready\r\n");
    uart_write("Single keys (no Enter): w/s speed +-0.05 m/s, a/d steer +-5 deg (-45 to +55), c center, z speed 0, x or SPACE stop, i idle (stop + center), h help\r\n");
    uart_write("Line commands: @DRIVE 0.20 10, @SPEED 0.10, @STEER -10, @CENTER, @STOP, @IDLE, @STATUS, @HELP\r\n");
    uart_write("Setup: @TIMEOUT <ms, 0=off>, @ECHO <0|1>, @ACK <0|1>, @SERVO <left_us> <right_us>, @MOTOR <left%> <right%> (independent open-loop test)\r\n");
    uart_write("Rear drive: two RB-35GM 09TYPE DC 24V W/EC 26P motors.\r\n");
#if !RC_DRIVE_CALIBRATION_READY
    uart_write("RB35GM CALIBRATION REQUIRED: W/S/@DRIVE/@SPEED motion is locked; @MOTOR is capped at 10%.\r\n");
    uart_write("Verify gear ratio, 52-count assumption, A/B signs, voltage, and PID first.\r\n");
#else
    uart_write("@DRIVE applies the steering-based left/right speed split.\r\n");
#endif
#if RC_LEFT_ENCODER_FEEDBACK_ENABLED && RC_RIGHT_ENCODER_FEEDBACK_ENABLED
    uart_write("Dual rear encoder feedback: ON (left PA0/PA1, right PA8/PA9).\r\n");
#elif RC_LEFT_ENCODER_FEEDBACK_ENABLED
    uart_write("Left rear encoder feedback: ON (PA0/PA1); right side uses open-loop duty.\r\n");
#elif RC_RIGHT_ENCODER_FEEDBACK_ENABLED
    uart_write("Right rear encoder feedback: ON (PA8/PA9); left side uses open-loop duty.\r\n");
#else
    uart_write("Rear encoder feedback: OFF.\r\n");
#endif
    uart_write("Motors auto-stop when no command arrives within the timeout (default 1500 ms).\r\n");
}

static void send_ok(char name)
{
    if (ack_enabled == 0U)
    {
        return;
    }
    uart_write("OK ");
    uart_putc(name);
    uart_write(" v_cmps=");
    uart_write_int((int32_t)(vehicle.target_speed_mps * 100.0f));
    uart_write(" steer_cdeg=");
    uart_write_int((int32_t)(vehicle.steering_deg * 100.0f));
    uart_write("\r\n");
}

static void send_err(const char *reason)
{
    if (ack_enabled == 0U)
    {
        return;
    }
    uart_write("ERR ");
    uart_write(reason);
    uart_write("\r\n");
}

/*
 * Immediate single-key control for humans typing into a raw serial terminal
 * (PuTTY/TeraTerm send one byte per keypress, no line buffering). Line
 * commands still require the '@'/':' prefix, so these keys never collide
 * with them or with the Python tool, which only sends '@' lines.
 */
static void handle_key(uint8_t key)
{
    char name = (char)key;

    switch (key)
    {
    case 'w': case 'W':
#if !RC_DRIVE_CALIBRATION_READY
        send_err("RB35GM calibration required; use @MOTOR <=10%");
        return;
#else
        vehicle_set_speed(vehicle.target_speed_mps + RC_SPEED_STEP_MPS);
        name = 'w';
        break;
#endif
    case 's': case 'S':
#if !RC_DRIVE_CALIBRATION_READY
        send_err("RB35GM calibration required; use @MOTOR <=10%");
        return;
#else
        vehicle_set_speed(vehicle.target_speed_mps - RC_SPEED_STEP_MPS);
        name = 's';
        break;
#endif
    case 'a': case 'A':
        vehicle_set_steering(vehicle.steering_deg + RC_STEER_STEP_DEG);
        name = 'a';
        break;
    case 'd': case 'D':
        vehicle_set_steering(vehicle.steering_deg - RC_STEER_STEP_DEG);
        name = 'd';
        break;
    case 'c': case 'C':
        center_command_count++;
        vehicle_center_steering();
        name = 'c';
        break;
    case 'z': case 'Z':
        vehicle_set_speed(0.0f);
        name = 'z';
        break;
    case 'x': case 'X': case ' ': case 0x03:  /* 0x03 = Ctrl-C in a terminal */
        vehicle_stop_drive_only();
        name = 'x';
        break;
    case 'i': case 'I':
        center_command_count++;
        vehicle_enter_idle();
        name = 'i';
        break;
    case 'h': case 'H': case '?':
        send_help();
        return;
    default:
        return;  /* Unknown bytes outside a line are ignored silently. */
    }

    mark_command(name, vehicle.steering_deg);
    mark_motion_command();
    /* Key feedback is always on: keys only ever come from a human. */
    uart_write("KEY ");
    uart_putc(name);
    uart_write(" v_cmps=");
    uart_write_int((int32_t)(vehicle.target_speed_mps * 100.0f));
    uart_write(" steer_cdeg=");
    uart_write_int((int32_t)(vehicle.steering_deg * 100.0f));
    uart_write("\r\n");
}

static void process_line(char *line)
{
    const char *cursor = line;

    while (*cursor == '@' || *cursor == ':' || *cursor == ' ' || *cursor == '\t')
    {
        cursor++;
    }

    if (word_equal(cursor, "STOP"))
    {
        if (line_ended(after_word(cursor)) == 0U)
        {
            ignored_line_count++;
            send_err("bad args");
            return;
        }
        vehicle_stop_drive_only();
        last_command_ms = g_ms;
        stopped_by_timeout = 1U;
        mark_command('P', vehicle.steering_deg);
        send_ok('P');
    }
    else if (word_equal(cursor, "IDLE"))
    {
        if (line_ended(after_word(cursor)) == 0U)
        {
            ignored_line_count++;
            send_err("bad args");
            return;
        }
        center_command_count++;
        vehicle_enter_idle();
        mark_command('I', 0.0f);
        mark_motion_command();
        send_ok('I');
    }
    else if (word_equal(cursor, "CENTER"))
    {
        if (line_ended(after_word(cursor)) == 0U)
        {
            ignored_line_count++;
            send_err("bad args");
            return;
        }
        center_command_count++;
        vehicle_center_steering();
        mark_command('C', 0.0f);
        mark_motion_command();
        send_ok('C');
    }
    else if (word_equal(cursor, "STATUS"))
    {
        if (line_ended(after_word(cursor)) == 0U)
        {
            ignored_line_count++;
            return;
        }
        send_status();
    }
    else if (word_equal(cursor, "HELP"))
    {
        if (line_ended(after_word(cursor)) == 0U)
        {
            ignored_line_count++;
            return;
        }
        send_help();
    }
    else if (word_equal(cursor, "SPEED"))
    {
        float speed;
        cursor = after_word(cursor);
        if (parse_float_checked(&cursor, &speed) == 0U ||
            line_ended(cursor) == 0U)
        {
            ignored_line_count++;
            send_err("bad args");
            return;
        }
#if !RC_DRIVE_CALIBRATION_READY
        if (absf(speed) > 0.001f)
        {
            vehicle_stop_drive_only();
            ignored_line_count++;
            send_err("RB35GM calibration required; use @MOTOR <=10%");
            return;
        }
#endif
        vehicle_set_speed(speed);
        mark_command('V', vehicle.steering_deg);
        mark_motion_command();
        send_ok('V');
    }
    else if (word_equal(cursor, "STEER"))
    {
        float steer;
        cursor = after_word(cursor);
        if (parse_float_checked(&cursor, &steer) == 0U ||
            line_ended(cursor) == 0U)
        {
            ignored_line_count++;
            send_err("bad args");
            return;
        }
        vehicle_set_steering(steer);
        mark_command('S', steer);
        mark_motion_command();
        send_ok('S');
    }
    else if (word_equal(cursor, "SERVO"))
    {
        float left_us;
        float right_us;
        cursor = after_word(cursor);
        if (parse_float_checked(&cursor, &left_us) == 0U ||
            parse_float_checked(&cursor, &right_us) == 0U ||
            line_ended(cursor) == 0U)
        {
            ignored_line_count++;
            send_err("bad args");
            return;
        }
        servo_set_us(left_us, right_us);
        mark_command('R', vehicle.steering_deg);
        mark_motion_command();
        send_ok('R');
    }
    else if (word_equal(cursor, "DRIVE"))
    {
        float speed;
        float steer;
        cursor = after_word(cursor);
        if (parse_float_checked(&cursor, &speed) == 0U ||
            parse_float_checked(&cursor, &steer) == 0U ||
            line_ended(cursor) == 0U)
        {
            ignored_line_count++;
            send_err("bad args");
            return;
        }
#if !RC_DRIVE_CALIBRATION_READY
        if (absf(speed) > 0.001f)
        {
            vehicle_stop_drive_only();
            ignored_line_count++;
            send_err("RB35GM calibration required; use @MOTOR <=10%");
            return;
        }
#endif
        vehicle_set_speed(speed);
        vehicle_set_steering(steer);
        mark_command('D', steer);
        mark_motion_command();
        send_ok('D');
    }
    else if (word_equal(cursor, "MOTOR"))
    {
        float left_percent;
        float right_percent;
        cursor = after_word(cursor);
        if (parse_float_checked(&cursor, &left_percent) == 0U ||
            parse_float_checked(&cursor, &right_percent) == 0U ||
            line_ended(cursor) == 0U)
        {
            ignored_line_count++;
            send_err("bad args");
            return;
        }
        apply_raw_motor(left_percent, right_percent);
        mark_command('M', vehicle.steering_deg);
        send_ok('M');
    }
    else if (word_equal(cursor, "TIMEOUT"))
    {
        float ms;
        cursor = after_word(cursor);
        if (parse_float_checked(&cursor, &ms) == 0U ||
            line_ended(cursor) == 0U || ms < 0.0f)
        {
            ignored_line_count++;
            send_err("bad args");
            return;
        }
        if (ms < 0.5f)
        {
            command_timeout_ms = 0U;
            uart_write("OK T timeout=off, motors will NOT auto-stop\r\n");
        }
        else
        {
            command_timeout_ms = (uint32_t)clampf(ms,
                                                  (float)RC_TIMEOUT_MIN_MS,
                                                  (float)RC_TIMEOUT_MAX_MS);
            if (ack_enabled != 0U)
            {
                uart_write("OK T timeout_ms=");
                uart_write_uint(command_timeout_ms);
                uart_write("\r\n");
            }
        }
        mark_command('T', vehicle.steering_deg);
    }
    else if (word_equal(cursor, "ECHO"))
    {
        float value;
        cursor = after_word(cursor);
        if (parse_float_checked(&cursor, &value) == 0U ||
            line_ended(cursor) == 0U)
        {
            ignored_line_count++;
            send_err("bad args");
            return;
        }
        echo_enabled = (value >= 0.5f) ? 1U : 0U;
        if (ack_enabled != 0U)
        {
            uart_write("OK E echo=");
            uart_write_uint((uint32_t)echo_enabled);
            uart_write("\r\n");
        }
        mark_command('E', vehicle.steering_deg);
    }
    else if (word_equal(cursor, "ACK"))
    {
        float value;
        cursor = after_word(cursor);
        if (parse_float_checked(&cursor, &value) == 0U ||
            line_ended(cursor) == 0U)
        {
            ignored_line_count++;
            send_err("bad args");
            return;
        }
        ack_enabled = (value >= 0.5f) ? 1U : 0U;
        /* Confirm regardless of the new setting so turning ACK off is visible. */
        uart_write("OK A ack=");
        uart_write_uint((uint32_t)ack_enabled);
        uart_write("\r\n");
        mark_command('A', vehicle.steering_deg);
    }
    else
    {
        ignored_line_count++;
        send_err("unknown cmd, try @HELP");
    }
}

static void remote_update(void)
{
    static char line[80];
    static uint32_t len;
    static uint8_t in_line;
    uint8_t byte;

    while (uart_read_byte(&byte) != 0U)
    {
        if (in_line == 0U)
        {
            if (byte == '@' || byte == ':')
            {
                in_line = 1U;
                len = 0U;
                line[len++] = (char)byte;
                uart_echo((char)byte);
            }
            else
            {
                handle_key(byte);
            }
            continue;
        }

        if (byte == '\r' || byte == '\n')
        {
            if (echo_enabled != 0U)
            {
                uart_write("\r\n");
            }
            line[len] = '\0';
            process_line(line);
            len = 0U;
            in_line = 0U;
        }
        else if (byte == 0x08U || byte == 0x7FU)  /* backspace / DEL */
        {
            if (len > 1U)  /* keep the leading '@' */
            {
                len--;
                if (echo_enabled != 0U)
                {
                    uart_write("\b \b");
                }
            }
        }
        else if (len < (sizeof(line) - 1U))
        {
            line[len++] = (char)byte;
            uart_echo((char)byte);
        }
        else
        {
            len = 0U;
            in_line = 0U;
            ignored_line_count++;
            send_err("line too long");
        }
    }

    if ((command_timeout_ms != 0U) &&
        ((g_ms - last_command_ms) > command_timeout_ms) &&
        (stopped_by_timeout == 0U))
    {
        vehicle_stop_drive_only();
        timeout_stop_count++;
        stopped_by_timeout = 1U;
        /* Silent stops read as broken control; say why the car halted. */
        uart_write("TIMEOUT STOP (no command within ");
        uart_write_uint(command_timeout_ms);
        uart_write(" ms; resend or @TIMEOUT <ms>)\r\n");
    }
}

static void hardware_init(void)
{
    retained_init();
    clock_init();
    /* Active-low motor ENABLE pins are the first application GPIOs placed in
     * a defined state. Keep this ahead of UART and servo initialization. */
    motor_init();
    uart_init();
    servo_init();
    encoder_init();

    left_speed_pid.kp = RC_PID_KP;
    left_speed_pid.ki = RC_PID_KI;
    left_speed_pid.kd = RC_PID_KD;
    right_speed_pid.kp = RC_PID_KP;
    right_speed_pid.ki = RC_PID_KI;
    right_speed_pid.kd = RC_PID_KD;
    pid_reset(&left_speed_pid);
    pid_reset(&right_speed_pid);

    vehicle_stop_drive_only();
    last_command_ms = g_ms;
    send_help();
}

int main(void)
{
    uint32_t last_control_ms;

    hardware_init();
    last_control_ms = g_ms;

    while (1)
    {
        uint32_t now_ms;
        uint32_t elapsed_ms;

        remote_update();

        /* Use the real elapsed time as dt: after a long blocking transmit
         * (e.g. the STATUS reply) a fixed-dt catch-up would first see several
         * ticks worth of encoder counts (inflated RPM), then zero counts,
         * kicking the PID both ways. */
        now_ms = g_ms;
        elapsed_ms = now_ms - last_control_ms;
        if (elapsed_ms >= RC_CONTROL_PERIOD_MS)
        {
            last_control_ms = now_ms;
            vehicle_update((float)elapsed_ms / 1000.0f);
        }
    }
}
