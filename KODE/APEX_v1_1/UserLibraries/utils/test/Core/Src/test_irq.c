#include "test_irq.h"

#include "main_config.h"

// Test builds only: a mission image must not carry a TIM5 handler.
#if (APEX_CFG_PROFILE_TEST == 1)

#include "main.h"

#include <stddef.h>

static void (*volatile irq_hook)(void);
static volatile uint32_t irq_count;

void TIM5_IRQHandler(void) {
    if ((TIM5->SR & TIM_SR_UIF) != 0u) {
        TIM5->SR = ~TIM_SR_UIF;
        irq_count++;
        void (*hook)(void) = irq_hook;
        if (hook != NULL) {
            hook();
        }
    }
}

void TEST_irq_start(uint32_t rate_hz, void (*hook)(void)) {
    __HAL_RCC_TIM5_CLK_ENABLE();
    TEST_irq_stop();
    if (rate_hz == 0u || rate_hz > 1000000u) {
        return;
    }

    // APB1 timer clock: PCLK1, doubled when the APB1 prescaler is not 1.
    uint32_t clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        clk *= 2u;
    }

    TIM5->CR1  = 0u;
    TIM5->DIER = 0u;
    TIM5->PSC  = (clk / 1000000u) - 1u;          // 1 MHz counter
    TIM5->ARR  = (1000000u / rate_hz) - 1u;
    TIM5->CNT  = 0u;
    TIM5->EGR  = TIM_EGR_UG;                      // load PSC
    TIM5->SR   = 0u;

    irq_count = 0u;
    irq_hook  = hook;
    NVIC_SetPriority(TIM5_IRQn, TEST_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(TIM5_IRQn);
    NVIC_EnableIRQ(TIM5_IRQn);

    TIM5->DIER = TIM_DIER_UIE;
    TIM5->CR1  = TIM_CR1_CEN;
}

void TEST_irq_stop(void) {
    TIM5->CR1  = 0u;
    TIM5->DIER = 0u;
    NVIC_DisableIRQ(TIM5_IRQn);
    NVIC_ClearPendingIRQ(TIM5_IRQn);
    TIM5->SR   = 0u;
    irq_hook   = NULL;
    __DSB();
    __ISB();
}

uint32_t TEST_irq_count(void) {
    return irq_count;
}

#endif // APEX_CFG_PROFILE_TEST
