#ifndef STM32F103_MIN_H
#define STM32F103_MIN_H

#include <stdint.h>

#define __IO volatile

typedef struct
{
    __IO uint32_t CRL;
    __IO uint32_t CRH;
    __IO uint32_t IDR;
    __IO uint32_t ODR;
    __IO uint32_t BSRR;
    __IO uint32_t BRR;
    __IO uint32_t LCKR;
} GPIO_TypeDef;

typedef struct
{
    __IO uint32_t CR;
    __IO uint32_t CFGR;
    __IO uint32_t CIR;
    __IO uint32_t APB2RSTR;
    __IO uint32_t APB1RSTR;
    __IO uint32_t AHBENR;
    __IO uint32_t APB2ENR;
    __IO uint32_t APB1ENR;
    __IO uint32_t BDCR;
    __IO uint32_t CSR;
} RCC_TypeDef;

typedef struct
{
    __IO uint32_t EVCR;
    __IO uint32_t MAPR;
    __IO uint32_t EXTICR[4];
    __IO uint32_t MAPR2;
} AFIO_TypeDef;

typedef struct
{
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMCR;
    __IO uint32_t DIER;
    __IO uint32_t SR;
    __IO uint32_t EGR;
    __IO uint32_t CCMR1;
    __IO uint32_t CCMR2;
    __IO uint32_t CCER;
    __IO uint32_t CNT;
    __IO uint32_t PSC;
    __IO uint32_t ARR;
    __IO uint32_t RCR;
    __IO uint32_t CCR1;
    __IO uint32_t CCR2;
    __IO uint32_t CCR3;
    __IO uint32_t CCR4;
    __IO uint32_t BDTR;
    __IO uint32_t DCR;
    __IO uint32_t DMAR;
} TIM_TypeDef;

typedef struct
{
    __IO uint32_t SR;
    __IO uint32_t DR;
    __IO uint32_t BRR;
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t CR3;
    __IO uint32_t GTPR;
} USART_TypeDef;

typedef struct
{
    __IO uint32_t CPUID;
    __IO uint32_t ICSR;
    __IO uint32_t VTOR;
    __IO uint32_t AIRCR;
    __IO uint32_t SCR;
    __IO uint32_t CCR;
    __IO uint32_t SHPR1;
    __IO uint32_t SHPR2;
    __IO uint32_t SHPR3;
    __IO uint32_t SHCSR;
} SCB_TypeDef;

#define PERIPH_BASE                 0x40000000UL
#define APB1PERIPH_BASE             PERIPH_BASE
#define APB2PERIPH_BASE             (PERIPH_BASE + 0x10000UL)
#define AHBPERIPH_BASE              (PERIPH_BASE + 0x20000UL)

#define TIM2_BASE                   (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE                   (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE                   (APB1PERIPH_BASE + 0x0800UL)
#define BKP_BASE                    (APB1PERIPH_BASE + 0x6C00UL)
#define PWR_BASE                    (APB1PERIPH_BASE + 0x7000UL)
#define USART2_BASE                 (APB1PERIPH_BASE + 0x4400UL)

#define AFIO_BASE                   (APB2PERIPH_BASE + 0x0000UL)
#define GPIOA_BASE                  (APB2PERIPH_BASE + 0x0800UL)
#define GPIOB_BASE                  (APB2PERIPH_BASE + 0x0C00UL)
#define GPIOC_BASE                  (APB2PERIPH_BASE + 0x1000UL)
#define TIM1_BASE                   (APB2PERIPH_BASE + 0x2C00UL)
#define RCC_BASE                    (AHBPERIPH_BASE + 0x1000UL)

#define SCS_BASE                    0xE000E000UL
#define SYSTICK_BASE                (SCS_BASE + 0x0010UL)
#define SCB_BASE                    (SCS_BASE + 0x0D00UL)

#define TIM1                        ((TIM_TypeDef *)TIM1_BASE)
#define TIM2                        ((TIM_TypeDef *)TIM2_BASE)
#define TIM3                        ((TIM_TypeDef *)TIM3_BASE)
#define TIM4                        ((TIM_TypeDef *)TIM4_BASE)
#define USART2                      ((USART_TypeDef *)USART2_BASE)
#define AFIO                        ((AFIO_TypeDef *)AFIO_BASE)
#define GPIOA                       ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                       ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC                       ((GPIO_TypeDef *)GPIOC_BASE)
#define RCC                         ((RCC_TypeDef *)RCC_BASE)
#define SCB                         ((SCB_TypeDef *)SCB_BASE)

#define SYST_CSR                    (*((__IO uint32_t *)(SYSTICK_BASE + 0x00UL)))
#define SYST_RVR                    (*((__IO uint32_t *)(SYSTICK_BASE + 0x04UL)))
#define SYST_CVR                    (*((__IO uint32_t *)(SYSTICK_BASE + 0x08UL)))

#define NVIC_ISER0                  (*((__IO uint32_t *)(SCS_BASE + 0x100UL)))
#define NVIC_ISER1                  (*((__IO uint32_t *)(SCS_BASE + 0x104UL)))
#define USART2_IRQ_NUMBER           38UL

#define BKP_DR1                     (*((__IO uint16_t *)(BKP_BASE + 0x04UL)))
#define BKP_DR2                     (*((__IO uint16_t *)(BKP_BASE + 0x08UL)))
#define BKP_DR3                     (*((__IO uint16_t *)(BKP_BASE + 0x0CUL)))
#define PWR_CR                      (*((__IO uint32_t *)(PWR_BASE + 0x00UL)))

#define FLASH_BASE                  0x08000000UL

#define RCC_CR_HSION                (1UL << 0)
#define RCC_CR_HSIRDY               (1UL << 1)

#define RCC_APB2ENR_AFIOEN          (1UL << 0)
#define RCC_APB2ENR_IOPAEN          (1UL << 2)
#define RCC_APB2ENR_IOPBEN          (1UL << 3)
#define RCC_APB2ENR_IOPCEN          (1UL << 4)
#define RCC_APB2ENR_TIM1EN          (1UL << 11)

#define AFIO_MAPR_TIM3_REMAP_MASK   (3UL << 10)
#define AFIO_MAPR_TIM3_REMAP_FULL   (3UL << 10)

#define RCC_APB1ENR_TIM2EN          (1UL << 0)
#define RCC_APB1ENR_TIM3EN          (1UL << 1)
#define RCC_APB1ENR_TIM4EN          (1UL << 2)
#define RCC_APB1ENR_USART2EN        (1UL << 17)
#define RCC_APB1ENR_BKPEN           (1UL << 27)
#define RCC_APB1ENR_PWREN           (1UL << 28)

#define PWR_CR_DBP                  (1UL << 8)

#define TIM_CR1_CEN                 (1UL << 0)
#define TIM_CR1_ARPE                (1UL << 7)
#define TIM_EGR_UG                  (1UL << 0)
#define TIM_CCMR_OC1PE              (1UL << 3)
#define TIM_CCMR_OC2PE              (1UL << 11)
#define TIM_CCMR_OC3PE              (1UL << 3)
#define TIM_CCMR_OC4PE              (1UL << 11)
#define TIM_CCER_CC1E               (1UL << 0)
#define TIM_CCER_CC2E               (1UL << 4)
#define TIM_CCER_CC3E               (1UL << 8)
#define TIM_CCER_CC4E               (1UL << 12)
#define TIM_SMCR_SMS_ENCODER_3      (3UL << 0)

#define USART_SR_RXNE               (1UL << 5)
#define USART_SR_TXE                (1UL << 7)
#define USART_SR_FE                 (1UL << 0)
#define USART_SR_NE                 (1UL << 2)
#define USART_SR_ORE                (1UL << 3)
#define USART_CR1_RE                (1UL << 2)
#define USART_CR1_TE                (1UL << 3)
#define USART_CR1_RXNEIE            (1UL << 5)
#define USART_CR1_UE                (1UL << 13)

#endif
