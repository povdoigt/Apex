#include "utils/usb.h"
#include "cmsis_os2.h"
#include "usbd_cdc_if.h"

static StaticSemaphore_t usb_semaphore_buffer;
static osSemaphoreId_t usb_semaphore_id;


void USB_Init(void) {
    const osSemaphoreAttr_t usb_semaphore_attr = {
        .name = "USB_Transmit_Semaphore",
        .cb_mem = &usb_semaphore_buffer,
        .cb_size = sizeof(usb_semaphore_buffer),
    };
    usb_semaphore_id = osSemaphoreNew(1, 1, &usb_semaphore_attr);
}

TASK_POOL_ALLOCATE(TASK_USB_Transmit);

void TASK_USB_Transmit(void *argument) {
    TASK_USB_Transmit_ARGS *args = (TASK_USB_Transmit_ARGS *)argument;

    // for (;;) {
        // Acquire the semaphore before transmitting
        osSemaphoreAcquire(usb_semaphore_id, osWaitForever);

        CDC_Transmit_FS(args->buff, args->len);

        // Release the semaphore after transmission
        osSemaphoreRelease(usb_semaphore_id);

    // }

    osThreadExit_Cstm();
}

