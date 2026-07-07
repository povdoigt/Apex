#ifndef USB_H
#define USB_H

#include "stm32f4xx_hal.h"
#include "scheduler.h"
#include "usbd_cdc_if.h"


void USB_Init(void);

void USB_update_watchdog(uint8_t *usb_watchdog_bfr);

bool USB_is_connected(uint8_t *usb_watchdog_bfr);


typedef struct TASK_USB_Transmit_ARGS {
    USBD_HandleTypeDef *hUsbDeviceFS;
    uint8_t *buff;
    uint16_t len;
} TASK_USB_Transmit_ARGS;

TASK_POOL_CONFIGURE(TASK_USB_Transmit, 10, 512);

void TASK_USB_Transmit(void *argument);


#endif // USB_H