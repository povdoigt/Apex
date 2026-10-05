#ifndef PROJECT_H
#define PROJECT_H

#include "main_config.h"
#include "drivers_config.h"

#include "cb_seq_test.h"
#include "dt_seq_test.h"
#include "dp_seq_test.h"
#include "dt_rtos_test.h"
#include "dt_rtos_stress.h"

// In RTOS mode, setup() is the whole application entry point. It runs once, in the
// application thread (defaultTask), after osKernelStart() and the USB init: it may spawn
// and join jobs directly. The thread exits when setup() returns; anything meant to run
// forever is a persistent task spawned from here. There is no loop() in RTOS mode.
void setup(void);

// Optional hook, called once from MX_FREERTOS_Init(), before osKernelStart(): thread
// mode, scheduler not started, no task running yet. Here it prepares the pre-kernel
// state checked by T27.
void setup_pre_kernel(void);

#endif // PROJECT_H
