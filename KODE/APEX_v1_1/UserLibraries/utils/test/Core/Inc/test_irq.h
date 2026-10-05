#ifndef TEST_IRQ_H
#define TEST_IRQ_H

#include <stdint.h>

// Asynchronous interrupt source for concurrency tests.
//
// TIM5 (unused on APEX) calls `hook` from its update interrupt, `rate_hz` times
// per second. The interrupt preempts the code under test at arbitrary
// instructions, which is what exposes a missing or too short critical section.
//
// NVIC priority TEST_IRQ_PRIORITY is numerically above
// configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5): the hook may call the
// FreeRTOS "FromISR" APIs, and it is masked by the circular_buffer critical
// sections in both schedules (BASEPRI under RTOS, PRIMASK in sequential).
//
// Test builds only (APEX_CFG_PROFILE_TEST == 1): the file defines
// TIM5_IRQHandler, so a project that enables TIM5 in CubeMX gets a link error
// instead of a silent conflict.
#define TEST_IRQ_PRIORITY   6u

// Start calling `hook` from the TIM5 interrupt. `rate_hz` in 1..1000000;
// any other value leaves the timer stopped. Restarting while running is allowed.
void TEST_irq_start(uint32_t rate_hz, void (*hook)(void));

// Stop the interrupt. Called from thread context: when it returns, `hook` is
// neither running nor pending.
void TEST_irq_stop(void);

// Interrupts serviced since the last TEST_irq_start() (wraps at 2^32).
uint32_t TEST_irq_count(void);

#endif // TEST_IRQ_H
