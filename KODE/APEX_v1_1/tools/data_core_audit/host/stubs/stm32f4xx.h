#ifndef STM32F4XX_HOST_STUB_H
#define STM32F4XX_HOST_STUB_H
#include <stdint.h>
/* Host stub: a single thread, no interrupt to mask. */
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t x) { (void)x; }
static inline uint32_t __get_IPSR(void) { return 0u; }
#endif
