
# ifndef RFM96W_FSK_h
# define RFM96W_FSK_h
// include
#include "stm32f4xx_hal.h"
#include "peripherals/spi.h"
#include "utils/scheduler.h"
#include <stdbool.h>

// Max number of octets the LORA Rx/Tx FIFO can hold
#define RFM96_FSK_FIFO_SIZE 255

// The crystal oscillator frequency of the module
#define RFM96_FSK_FXOSC 32000000.0
#define RFM96_FSK_FREQUENCE 869.5 // Frequency in MHz


// Register names (FSK Mode, from table 86)
// Register for common settings
#define RFM96_FSK_REG_OP_MODE                            0x00 // FSK Mode


# endif // RFM96W_FSK_h