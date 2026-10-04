#ifndef PROJECT_H
#define PROJECT_H

#include "main_config.h"
#include "drivers_config.h"

#include "BMI088_rtos_bench.h"

// In RTOS mode, setup() is the whole application entry point. It runs once, in the
// application thread (defaultTask), after osKernelStart() and the USB init: it may spawn
// and join jobs directly. The thread exits when setup() returns; anything meant to run
// forever is a persistent task spawned from here. There is no loop() in RTOS mode.
void setup(void);

#endif // PROJECT_H
