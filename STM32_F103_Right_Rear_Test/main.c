#include <stdint.h>

#include "stm32f103_min.h"
#include "test_config.h"

/*
 * Standalone right-rear wheel test for NUCLEO-F103RB.
 *
 * Active peripherals:
 *   USART2 PA2/PA3 : ST-LINK virtual COM port
 *   TIM3_CH2 PA7   : right motor PWM
 *   PB10/PB13/PB11 : right IN1/IN2/active-low ENABLE
 *   TIM1 PA8/PA9   : right encoder A/B
 *
 * The left motor PWM and direction pins and both steering-servo channels are
 * never configured. PB1 is driven high solely as a hardware lockout for the
 * left channel's active-low ENABLE input.
 */

typedef enum
{
    TEST_STOPPED = 0,
    TEST_START_DELAY,
    TEST_FORWARD,
    TEST_REVERSE
} TestState;

static volatile uint32_t g_ms;
static TestState test_state = TEST_STOPPED;
static uint8_t selected_duty = RR_TEST_DEFAULT_DUTY_PERCENT;
static uint8_t applied_duty;
static uint8_t pending_forward = 1U;
static uint32_t start_deadline_ms;
static uint32_t stop_deadline_ms;

static uint16_t encoder_previous_counter;
static int32_t encoder_total;
static uint8_t encoder_previous_ab;
static uint32_t encoder_a_changes;
static uint32_t encoder_b_changes;

static uint32_t last_report_ms;
static int32_t last_report_total;
static uint32_t last_report_a_changes;
static uint32_t last_report_b_changes;

void SystemInit(void)
{
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U)
    {
    }

    RCC->CFGR = 0U;
    SCB->VTOR = FLASH_BASE;
}

void SysTick_Handler(void)
{
    g_ms++;
}

static uint8_t time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1U : 0U;
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
    gpio_config_pin(gpio, pin, 0x2U); /* Push-pull output, 2 MHz. */
}

static void gpio_config_af_output(GPIO_TypeDef *gpio, uint32_t pin)
{
    gpio_config_pin(gpio, pin, 0xAU); /* AF push-pull output, 2 MHz. */
}

static void gpio_config_input_floating(GPIO_TypeDef *gpio, uint32_t pin)
{
    gpio_config_pin(gpio, pin, 0x4U);
}

static void gpio_write(GPIO_TypeDef *gpio, uint32_t pin, uint8_t high)
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

static uint32_t usart_brr(uint32_t clock_hz, uint32_t baud)
{
    return (clock_hz + (baud / 2U)) / baud;
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

static void uart_write_u32(uint32_t value)
{
    char buffer[10];
    uint32_t length = 0U;

    do
    {
        buffer[length++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (length < (uint32_t)sizeof(buffer)));

    while (length != 0U)
    {
        uart_putc(buffer[--length]);
    }
}

static void uart_write_i32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0)
    {
        uart_putc('-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)value;
    }
    uart_write_u32(magnitude);
}

static uint8_t uart_read(char *command)
{
    if ((USART2->SR & USART_SR_RXNE) == 0U)
    {
        return 0U;
    }

    *command = (char)(uint8_t)USART2->DR;
    return 1U;
}

static void clock_init(void)
{
    /* TIM4 and GPIOC deliberately remain disabled: no servo drive is possible. */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN |
                    RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN |
                    RCC_APB1ENR_USART2EN;
    (void)RCC->APB2ENR;
    (void)RCC->APB1ENR;

    SYST_RVR = (RC_SYSCLK_HZ / 1000UL) - 1UL;
    SYST_CVR = 0U;
    SYST_CSR = 7U;
}

static void uart_init(void)
{
    gpio_config_af_output(GPIOA, 2U);
    gpio_config_input_floating(GPIOA, 3U);

    USART2->CR1 = 0U;
    USART2->BRR = usart_brr(RC_SYSCLK_HZ, RR_TEST_UART_BAUD);
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static uint8_t encoder_read_ab(void)
{
    uint32_t inputs = GPIOA->IDR;
    uint8_t a = (uint8_t)((inputs >> RC_RIGHT_ENCODER_A_PIN) & 1UL);
    uint8_t b = (uint8_t)((inputs >> RC_RIGHT_ENCODER_B_PIN) & 1UL);
    return (uint8_t)(a | (uint8_t)(b << 1));
}

static void encoder_reset(void)
{
    TIM1->CNT = 0U;
    encoder_previous_counter = 0U;
    encoder_total = 0;
    encoder_a_changes = 0U;
    encoder_b_changes = 0U;
    encoder_previous_ab = encoder_read_ab();

    last_report_ms = g_ms;
    last_report_total = 0;
    last_report_a_changes = 0U;
    last_report_b_changes = 0U;
}

static void encoder_init(void)
{
    gpio_config_input_floating(GPIOA, RC_RIGHT_ENCODER_A_PIN);
    gpio_config_input_floating(GPIOA, RC_RIGHT_ENCODER_B_PIN);

    TIM1->CR1 = 0U;
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

    encoder_reset();
}

static void encoder_poll(void)
{
    uint16_t counter = (uint16_t)TIM1->CNT;
    int16_t delta = (int16_t)(uint16_t)(counter - encoder_previous_counter);
    uint8_t ab = encoder_read_ab();

    encoder_previous_counter = counter;
    encoder_total += (int32_t)delta;

    if (((ab ^ encoder_previous_ab) & 1U) != 0U)
    {
        encoder_a_changes++;
    }
    if (((ab ^ encoder_previous_ab) & 2U) != 0U)
    {
        encoder_b_changes++;
    }
    encoder_previous_ab = ab;
}

static void motor_output_stop(void)
{
    /* Remove PWM first, then disable the active-low driver and clear direction. */
    TIM3->CCR2 = 0U;
    gpio_write(GPIOB, RC_RIGHT_MOTOR_ENABLE_PIN, 1U);
    gpio_write(GPIOB, RC_RIGHT_MOTOR_IN1_PIN, 0U);
    gpio_write(GPIOB, RC_RIGHT_MOTOR_IN2_PIN, 0U);
    applied_duty = 0U;
}

static void motor_init(void)
{
    uint32_t period_counts = RC_SYSCLK_HZ / RC_MOTOR_PWM_HZ;

    /* Preload safe levels before changing GPIO modes. */
    gpio_write(GPIOB, RC_LEFT_MOTOR_ENABLE_PIN, 1U);
    gpio_write(GPIOB, RC_RIGHT_MOTOR_ENABLE_PIN, 1U);
    gpio_write(GPIOB, RC_RIGHT_MOTOR_IN1_PIN, 0U);
    gpio_write(GPIOB, RC_RIGHT_MOTOR_IN2_PIN, 0U);

    /* PB1 is the only left-drive pin touched: force active-low ENABLE off. */
    gpio_config_output(GPIOB, RC_LEFT_MOTOR_ENABLE_PIN);
    gpio_config_output(GPIOB, RC_RIGHT_MOTOR_ENABLE_PIN);
    gpio_config_output(GPIOB, RC_RIGHT_MOTOR_IN1_PIN);
    gpio_config_output(GPIOB, RC_RIGHT_MOTOR_IN2_PIN);

    AFIO->MAPR &= ~AFIO_MAPR_TIM3_REMAP_MASK;
    TIM3->CR1 = 0U;
    TIM3->PSC = 0U;
    TIM3->ARR = period_counts - 1U;
    TIM3->CCMR1 = (6UL << 12) | TIM_CCMR_OC2PE;
    TIM3->CCER = TIM_CCER_CC2E;
    TIM3->CCR1 = 0U;
    TIM3->CCR2 = 0U;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    /* Connect PA7 only after TIM3_CH2 is known to be at zero duty. */
    gpio_config_af_output(GPIOA, RC_RIGHT_MOTOR_PWM_PIN);
    motor_output_stop();
    test_state = TEST_STOPPED;
}

static void motor_cancel(void)
{
    motor_output_stop();
    test_state = TEST_STOPPED;
}

static void motor_schedule_pulse(uint8_t forward)
{
    motor_output_stop();
    pending_forward = (forward != 0U) ? 1U : 0U;
    start_deadline_ms = g_ms + RR_TEST_DIRECTION_PAUSE_MS;
    test_state = TEST_START_DELAY;
}

static void motor_start_pending(uint32_t now_ms)
{
    uint8_t in1_high = (pending_forward != 0U)
                           ? (uint8_t)RC_RIGHT_MOTOR_FORWARD_IN1_HIGH
                           : (uint8_t)(1U - RC_RIGHT_MOTOR_FORWARD_IN1_HIGH);
    uint8_t safe_duty = (selected_duty <= RR_TEST_MAX_DUTY_PERCENT)
                            ? selected_duty
                            : RR_TEST_MAX_DUTY_PERCENT;
    uint32_t period_counts = TIM3->ARR + 1U;
    uint32_t compare = ((period_counts * (uint32_t)safe_duty) + 50U) / 100U;

    gpio_write(GPIOB, RC_RIGHT_MOTOR_IN1_PIN, in1_high);
    gpio_write(GPIOB, RC_RIGHT_MOTOR_IN2_PIN, (uint8_t)(1U - in1_high));
    TIM3->CCR2 = compare;
    gpio_write(GPIOB, RC_RIGHT_MOTOR_ENABLE_PIN, 0U);

    applied_duty = safe_duty;
    test_state = (pending_forward != 0U) ? TEST_FORWARD : TEST_REVERSE;
    stop_deadline_ms = now_ms + RR_TEST_PULSE_MS;
}

static void motor_service(void)
{
    uint32_t now_ms = g_ms;

    if ((test_state == TEST_START_DELAY) &&
        (time_reached(now_ms, start_deadline_ms) != 0U))
    {
        motor_start_pending(now_ms);
    }
    else if (((test_state == TEST_FORWARD) || (test_state == TEST_REVERSE)) &&
             (time_reached(now_ms, stop_deadline_ms) != 0U))
    {
        motor_cancel();
        uart_write("\r\nAUTO STOP: pulse complete\r\n");
    }
}

static const char *state_name(void)
{
    switch (test_state)
    {
        case TEST_START_DELAY:
            return "WAIT";
        case TEST_FORWARD:
            return "FWD";
        case TEST_REVERSE:
            return "REV";
        default:
            return "STOP";
    }
}

static int32_t calculate_motor_rpm_x10(int32_t count_delta, uint32_t elapsed_ms)
{
    int64_t numerator = (int64_t)count_delta * 600000LL;
    int64_t denominator = (int64_t)RR_TEST_ENCODER_COUNTS_PER_MOTOR_REV *
                          (int64_t)elapsed_ms;

    if (denominator == 0)
    {
        return 0;
    }
    if (numerator >= 0)
    {
        numerator += denominator / 2;
    }
    else
    {
        numerator -= denominator / 2;
    }
    return (int32_t)(numerator / denominator);
}

static void uart_write_fixed_x10(int32_t value_x10)
{
    uint32_t magnitude;

    if (value_x10 < 0)
    {
        uart_putc('-');
        magnitude = (uint32_t)(-(value_x10 + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)value_x10;
    }

    uart_write_u32(magnitude / 10U);
    uart_putc('.');
    uart_putc((char)('0' + (magnitude % 10U)));
}

static void print_report(void)
{
    uint32_t now_ms = g_ms;
    uint32_t elapsed_ms = now_ms - last_report_ms;
    int32_t count_delta = encoder_total - last_report_total;
    uint32_t a_delta = encoder_a_changes - last_report_a_changes;
    uint32_t b_delta = encoder_b_changes - last_report_b_changes;
    int32_t motor_rpm_x10 = calculate_motor_rpm_x10(count_delta, elapsed_ms);
    uint8_t ab = encoder_previous_ab;

    last_report_ms = now_ms;
    last_report_total = encoder_total;
    last_report_a_changes = encoder_a_changes;
    last_report_b_changes = encoder_b_changes;

    uart_write("ms=");
    uart_write_u32(now_ms);
    uart_write(" state=");
    uart_write(state_name());
    uart_write(" duty=");
    uart_write_u32(applied_duty);
    uart_write("% total=");
    uart_write_i32(encoder_total);
    uart_write(" net=");
    uart_write_i32(count_delta);
    uart_write(" motor_rpm_assumed=");
    uart_write_fixed_x10(motor_rpm_x10);
    uart_write(" wheel_rpm=TBD");
    uart_write(" A=");
    uart_putc(((ab & 1U) != 0U) ? '1' : '0');
    uart_write(" B=");
    uart_putc(((ab & 2U) != 0U) ? '1' : '0');
    uart_write(" Achg=");
    uart_write_u32(a_delta);
    uart_write(" Bchg=");
    uart_write_u32(b_delta);
    uart_write("\r\n");
}

static void print_help(void)
{
    uart_write("\r\nCommands (single key, no Enter required):\r\n");
    uart_write("  1       : select 10% duty; selection alone does not run\r\n");
    uart_write("  f       : forward 1 s pulse after a 100 ms safe pause\r\n");
    uart_write("  b       : reverse 1 s pulse after a 100 ms safe pause\r\n");
    uart_write("  x/SPACE : immediate stop\r\n");
    uart_write("  r       : reset right encoder counts\r\n");
    uart_write("  h/?     : show this help\r\n");
}

static void select_duty(uint8_t duty)
{
    selected_duty = (duty <= RR_TEST_MAX_DUTY_PERCENT)
                        ? duty
                        : RR_TEST_MAX_DUTY_PERCENT;
    uart_write("\r\nSelected duty=");
    uart_write_u32(selected_duty);
    uart_write("%. Press f or b to run one pulse.\r\n");
}

static void handle_command(char command)
{
    switch (command)
    {
        case '1':
            select_duty(10U);
            break;
        case 'f':
        case 'F':
            motor_schedule_pulse(1U);
            uart_write("\r\nForward pulse queued.\r\n");
            break;
        case 'b':
        case 'B':
            motor_schedule_pulse(0U);
            uart_write("\r\nReverse pulse queued.\r\n");
            break;
        case 'x':
        case 'X':
        case ' ':
            motor_cancel();
            uart_write("\r\nSTOP\r\n");
            break;
        case 'r':
        case 'R':
            encoder_reset();
            uart_write("\r\nRight encoder counts reset.\r\n");
            break;
        case 'h':
        case 'H':
        case '?':
            print_help();
            break;
        case '\r':
        case '\n':
            break;
        default:
            uart_write("\r\nUnknown key. Press h for help.\r\n");
            break;
    }
}

static void print_banner(void)
{
    uart_write("\r\nRIGHT_REAR_WHEEL_TEST ");
    uart_write(RR_TEST_FIRMWARE_VERSION);
    uart_write("\r\nLEFT_DRIVE=LOCKED_OUT; SERVO=NOT_INITIALIZED\r\n");
    uart_write("AUTO_STOP_MS=1000; MAX_DUTY=10%; BOOT_STATE=STOP\r\n");
    uart_write("MOTOR=RB35GM_09TYPE_24V_26P; COUNTS_PER_MOTOR_REV=52_ASSUMED; GEAR_RATIO=TBD\r\n");
    uart_write("Right motor: PWM=PA7, IN1=PB10, IN2=PB13, ENABLE=PB11(active-low)\r\n");
    uart_write("Right encoder: A=PA8, B=PA9; UART=115200 8N1\r\n");
    uart_write("Lift and support the drive wheels before applying 24 V motor power.\r\n");
    print_help();
}

int main(void)
{
    clock_init();
    uart_init();
    motor_init();
    encoder_init();
    print_banner();

    while (1)
    {
        char command;
        uint32_t now_ms;

        encoder_poll();
        motor_service();

        if (uart_read(&command) != 0U)
        {
            handle_command(command);
        }

        now_ms = g_ms;
        if ((now_ms - last_report_ms) >= RR_TEST_REPORT_PERIOD_MS)
        {
            print_report();
        }
    }
}
