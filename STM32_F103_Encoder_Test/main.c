#include <stdint.h>

#include "stm32f103_min.h"

/*
 * Standalone encoder wiring test for NUCLEO-F103RB.
 *
 * Enabled peripherals:
 *   - USART2 on PA2/PA3 (ST-LINK virtual COM port)
 *   - TIM1 encoder input on PA8/PA9 (installed right encoder)
 *   - SysTick for the report interval
 *
 * GPIOB, TIM3 and TIM4 are deliberately never clocked or configured. This
 * firmware therefore cannot generate motor or steering-servo drive signals.
 */

#define SYSCLK_HZ              8000000UL
#define UART_BAUD              115200UL
#define REPORT_PERIOD_MS       250UL
#define ENCODER_INPUT_FILTER   4UL

typedef struct
{
    TIM_TypeDef *timer;
    uint32_t pin_a;
    uint32_t pin_b;
    uint16_t previous_counter;
    int32_t total;
    uint32_t movement;
    uint32_t a_changes;
    uint32_t b_changes;
    uint8_t previous_ab;
} EncoderState;

static volatile uint32_t g_ms;

static EncoderState right_encoder = {
    TIM1, 8U, 9U, 0U, 0, 0U, 0U, 0U, 0U
};

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

static void gpio_config_af_output(GPIO_TypeDef *gpio, uint32_t pin)
{
    gpio_config_pin(gpio, pin, 0xAU); /* Alternate-function push-pull, 2 MHz. */
}

static void gpio_config_input_floating(GPIO_TypeDef *gpio, uint32_t pin)
{
    gpio_config_pin(gpio, pin, 0x4U);
}

static uint8_t gpio_read_ab(uint32_t pin_a, uint32_t pin_b)
{
    uint32_t inputs = GPIOA->IDR;
    uint8_t a = (uint8_t)((inputs >> pin_a) & 1UL);
    uint8_t b = (uint8_t)((inputs >> pin_b) & 1UL);
    return (uint8_t)(a | (uint8_t)(b << 1));
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

static void clock_init(void)
{
    /* Intentionally omit GPIOB, GPIOC, TIM3 and TIM4. */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN |
                    RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB2ENR;
    (void)RCC->APB1ENR;

    SYST_RVR = (SYSCLK_HZ / 1000UL) - 1UL;
    SYST_CVR = 0U;
    SYST_CSR = 7U;
}

static void uart_init(void)
{
    gpio_config_af_output(GPIOA, 2U);
    gpio_config_input_floating(GPIOA, 3U);

    USART2->CR1 = 0U;
    USART2->BRR = usart_brr(SYSCLK_HZ, UART_BAUD);
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void encoder_timer_init(TIM_TypeDef *timer)
{
    timer->CR1 = 0U;
    timer->PSC = 0U;
    timer->ARR = 0xFFFFUL;
    timer->CCMR1 = 1UL |
                   (ENCODER_INPUT_FILTER << 4) |
                   (1UL << 8) |
                   (ENCODER_INPUT_FILTER << 12);
    timer->CCER = 0U;
    timer->SMCR = TIM_SMCR_SMS_ENCODER_3;
    timer->CNT = 0U;
    timer->CR1 = TIM_CR1_CEN;
}

static void encoder_state_reset(EncoderState *encoder)
{
    encoder->timer->CNT = 0U;
    encoder->previous_counter = 0U;
    encoder->total = 0;
    encoder->movement = 0U;
    encoder->a_changes = 0U;
    encoder->b_changes = 0U;
    encoder->previous_ab = gpio_read_ab(encoder->pin_a, encoder->pin_b);
}

static void encoders_init(void)
{
    /* The unconnected left inputs stay in low-power analog mode. The installed
     * right harness is brown=+5 V, green=GND, lavender=A/PA8, blue=B/PA9.
     * PA8/PA9 are 5 V-tolerant STM32F103RB pins. If the encoder outputs are
     * open-collector without onboard pull-ups, add external pull-ups. */
    gpio_config_pin(GPIOA, 0U, 0x0U);
    gpio_config_pin(GPIOA, 1U, 0x0U);
    gpio_config_input_floating(GPIOA, 8U);
    gpio_config_input_floating(GPIOA, 9U);

    encoder_timer_init(TIM1);
    encoder_state_reset(&right_encoder);
}

static void encoder_poll(EncoderState *encoder)
{
    uint16_t counter = (uint16_t)encoder->timer->CNT;
    int16_t delta = (int16_t)(uint16_t)(counter - encoder->previous_counter);
    uint8_t ab = gpio_read_ab(encoder->pin_a, encoder->pin_b);

    encoder->previous_counter = counter;
    encoder->total += (int32_t)delta;
    if (delta < 0)
    {
        encoder->movement += (uint32_t)(-(int32_t)delta);
    }
    else
    {
        encoder->movement += (uint32_t)delta;
    }

    if (((ab ^ encoder->previous_ab) & 1U) != 0U)
    {
        encoder->a_changes++;
    }
    if (((ab ^ encoder->previous_ab) & 2U) != 0U)
    {
        encoder->b_changes++;
    }
    encoder->previous_ab = ab;
}

static uint8_t uart_poll_command(void)
{
    if ((USART2->SR & USART_SR_RXNE) != 0U)
    {
        char command = (char)(uint8_t)USART2->DR;
        if ((command == 'r') || (command == 'R'))
        {
            encoder_state_reset(&right_encoder);
            uart_write("\r\nCounts reset.\r\n");
            return 1U;
        }
        else if ((command == 'h') || (command == 'H') || (command == '?'))
        {
            uart_write("\r\nr/R: reset counts, h/?: help. Turn the right wheel by hand.\r\n");
        }
    }
    return 0U;
}

static void print_encoder(const char *name,
                          const EncoderState *encoder,
                          int32_t interval_net,
                          uint32_t interval_movement,
                          uint32_t interval_a_changes,
                          uint32_t interval_b_changes)
{
    uint8_t ab = encoder->previous_ab;

    uart_write(name);
    uart_write(" total=");
    uart_write_i32(encoder->total);
    uart_write(" net=");
    uart_write_i32(interval_net);
    uart_write(" move=");
    uart_write_u32(interval_movement);
    uart_write(" A=");
    uart_putc(((ab & 1U) != 0U) ? '1' : '0');
    uart_write(" B=");
    uart_putc(((ab & 2U) != 0U) ? '1' : '0');
    uart_write(" Achg=");
    uart_write_u32(interval_a_changes);
    uart_write(" Bchg=");
    uart_write_u32(interval_b_changes);
}

static void print_banner(void)
{
    uart_write("\r\nNUCLEO-F103RB RB35GM encoder-only test ready\r\n");
    uart_write("Right MCU inputs: PA8/D7=A, PA9/D8=B (TIM1). Physical wire mapping is TBD.\r\n");
    uart_write("RB35GM wires: Purple, Blue, Mint, Brown, Red, Black; all functions TBD.\r\n");
    uart_write("Verify Encoder VCC/GND/A/B and output voltage before connection. Left encoder is disabled.\r\n");
    uart_write("Motor/servo GPIO and timers are NOT initialized. Keep 24 V motor power disconnected.\r\n");
    uart_write("PA8/PA9 are 5 V-tolerant; never apply 24 V to encoder wires.\r\n");
    uart_write("115200 8N1; r resets counts, h shows help. Reporting every 250 ms.\r\n\r\n");
}

int main(void)
{
    uint32_t last_report_ms;
    int32_t previous_right_total = 0;
    uint32_t previous_right_movement = 0U;
    uint32_t previous_right_a_changes = 0U;
    uint32_t previous_right_b_changes = 0U;

    clock_init();
    uart_init();
    encoders_init();
    print_banner();
    last_report_ms = g_ms;

    while (1)
    {
        uint32_t now;

        encoder_poll(&right_encoder);
        if (uart_poll_command() != 0U)
        {
            /* Keep interval fields at zero after a user reset instead of
             * reporting a synthetic reverse jump from the old snapshot. */
            previous_right_total = right_encoder.total;
            previous_right_movement = right_encoder.movement;
            previous_right_a_changes = right_encoder.a_changes;
            previous_right_b_changes = right_encoder.b_changes;
            last_report_ms = g_ms;
        }

        now = g_ms;
        if ((now - last_report_ms) >= REPORT_PERIOD_MS)
        {
            int32_t right_net = right_encoder.total - previous_right_total;
            uint32_t right_movement = right_encoder.movement - previous_right_movement;
            uint32_t right_a_changes = right_encoder.a_changes - previous_right_a_changes;
            uint32_t right_b_changes = right_encoder.b_changes - previous_right_b_changes;

            last_report_ms = now;
            previous_right_total = right_encoder.total;
            previous_right_movement = right_encoder.movement;
            previous_right_a_changes = right_encoder.a_changes;
            previous_right_b_changes = right_encoder.b_changes;

            uart_write("ms=");
            uart_write_u32(now);
            print_encoder("R", &right_encoder, right_net, right_movement,
                          right_a_changes, right_b_changes);
            uart_write("\r\n");
        }
    }
}
