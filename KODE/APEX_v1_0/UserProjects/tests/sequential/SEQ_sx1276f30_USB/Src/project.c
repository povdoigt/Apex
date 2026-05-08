#include "project.h"

#include "drivers_config.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "vt100.h"

#include "spi.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

void setup(void) {

    sx127x_Init(&sx127x_2, sx127x_base_config_2, sx127x_modulation_2, (sx127x_mod_config_t){.fsk_ook = sx127x_FSK_OOK_config});

}

void loop(void) {
    
}
