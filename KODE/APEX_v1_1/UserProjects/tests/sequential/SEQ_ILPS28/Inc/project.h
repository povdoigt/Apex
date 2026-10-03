#ifndef PROJECT_H
#define PROJECT_H

#include "main_config.h"
#include "drivers_config.h"
#include "scheduler.h"

/* =========================================================================
 * Standard project entry points
 * ========================================================================= */

void uart_seq_callback(uint16_t size);

void setup(void);
void loop(void);

#endif // PROJECT_H
