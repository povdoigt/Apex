#ifndef PROJECT_H
#define PROJECT_H

#include "main_config.h"
#include "drivers_config.h"

#include "BMI088_seq_test.h"

// Setup fonction is used any additional execution that needs to be done once at
// the start of the program, after all initializations. It is called once in the main function
// after all peripheral and driver initializations, and before the main loop starts.
// It is typically used to create tasks, initialize variables, or perform any setup that requires
// the drivers to be initialized first.
void setup(void);

// Loop function is used for the main execution of the program. It is called repeatedly in the main
// function after setup() is called. It contains the main logic of the program, and can be used to
// run tasks, read sensors, transmit data, etc.
// NOTE: If using an RTOS, the main loop might be empty and the logic will be implemented in tasks instead.
// In that case, this function can be left empty or used for any non-RTOS related logic that needs to
// run continuously.
void loop(void);


#endif // PROJECT_H