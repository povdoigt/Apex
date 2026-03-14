#ifndef PROJECT_H
#define PROJECT_H

#include "main_config.h"
#include "drivers_config.h"

#include "dt_seq_test.h"

// Setup fonction is used any additional execution that needs to be done once at
// the start of the program, after all initializations. It is called once in the main function
// after all peripheral and driver initializations, and before the main loop starts.
void setup(void);

// Loop function is used for the main execution of the program. It is called repeatedly in the main
// function after setup() is called.
void loop(void);

#endif // PROJECT_H
