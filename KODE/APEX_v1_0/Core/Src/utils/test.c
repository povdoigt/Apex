#include "utils/test.h"

#include "utils/scheduler.h"

#include "peripherals/adc.h"
#include "peripherals/dma.h"
#include "peripherals/gpio.h"
#include "peripherals/i2c.h"
#include "peripherals/spi.h"
#include "peripherals/tim.h"
#include "peripherals/usart.h"

#include "usbd_cdc_if.h"


void init_obj_pools() {

    // TASK_POOL_init(ASYNC_BMI088_ReadSensorDMA);
    // TASK_POOL_init(ASYNC_BMI088_Sensor_Publisher);

    // TASK_POOL_init(ASYNC_BUZZER_play_note);

    // TASK_POOL_init(ASYNC_LSM303AGR_ReadSensor);

    // TASK_POOL_init(ASYNC_RFM96_BeginPacket);
    // TASK_POOL_init(ASYNC_RFM96_EndPacket);
    // TASK_POOL_init(ASYNC_RFM96_WriteFIFO);
    // TASK_POOL_init(ASYNC_RFM96_SetRxMode);
    // TASK_POOL_init(ASYNC_RFM96_SendPacket);
    // TASK_POOL_init(ASYNC_RFM96_ParsePacket);


    // TASK_POOL_init(ASYNC_W25Q_ReadStatusReg);
    // TASK_POOL_init(ASYNC_W25Q_WriteEnable);
    // TASK_POOL_init(ASYNC_W25Q_WaitForReady);
    // TASK_POOL_init(ASYNC_W25Q_EraseSector);
    // TASK_POOL_init(ASYNC_W25Q_EraseAll);
    // TASK_POOL_init(ASYNC_W25Q_ReadData_Smol);
    // TASK_POOL_init(ASYNC_W25Q_ReadData);
    // TASK_POOL_init(ASYNC_W25Q_PageProgram);
    // TASK_POOL_init(ASYNC_W25Q_WriteData);
    // TASK_POOL_init(ASYNC_W25Q_ScanIfSectorErased);
    // TASK_POOL_init(ASYNC_W25Q_ScanIfBlockErased);
    // TASK_POOL_init(ASYNC_W25Q_ScanIfChipErased);
    // TASK_POOL_init(ASYNC_W25Q_USB_ScanMemory);

    // TASK_POOL_init(ASYNC_I2C_Tx_or_Rx_DMA);
    // TASK_POOL_init(ASYNC_I2C_Mem_Tx_or_Rx_DMA);
    // TASK_POOL_init(ASYNC_SPI_TxRx_DMA);
    // TASK_POOL_init(ASYNC_SPI_TxRx_DMA_static);
    // TASK_POOL_init(ASYNC_UART_Rx_DMA);
    // TASK_POOL_init(ASYNC_UART_Tx_DMA);
    // TASK_POOL_init(ASYNC_UART_RxToIdle_DMA);

    // TASK_POOL_init(ASYNC_fs_read_write);

    // TASK_POOL_init(ASYNC_update_usb_watchdog);
    // TASK_POOL_init(ASYNC_test_usb);
    // TASK_POOL_init(ASYNC_send_usb);
    // TASK_POOL_init(ASYNC_update_IMU);
    // TASK_POOL_init(ASYNC_test_w25q_page);
    // TASK_POOL_init(ASYNC_test_w25q_write);
    // TASK_POOL_init(ASYNC_test_fs);
    // TASK_POOL_init(ASYNC_save_IMU);
    // TASK_POOL_init(ASYNC_filtred_IMU);
    // TASK_POOL_init(ASYNC_test_LSM303AGR);
    // TASK_POOL_init(ASYNC_test_RFM96_Tx);
    // TASK_POOL_init(ASYNC_test_RFM96_Rx);
    // TASK_POOL_init(ASYNC_test_GPS);
    // TASK_POOL_init(ASYNC_test_MESURE_ALL_DATA);
    // TASK_POOL_init(ASYNC_test_SAVE_ALL_DATA);
    // TASK_POOL_init(ASYNC_telem_gps);

    // TASK_POOL_init(ASYNC_Memory_Erase_Process);
    // TASK_POOL_init(ASYNC_SEND_ALL_DATA_USB);
    // TASK_POOL_init(ASYNC_SCAN_IF_MEMORY_EMPTY);

    // TASK_POOL_init(ASYNC_Delay);
}

void init_components(COMPONENTS             *components,
                     ADXL375                *adxl,
                     BMI088                 *bmi,
                     BMP388_HandleTypeDef   *bmp,
                     BUZZER                 *buzzer,
                     GPS_t                  *gps,
                     LED_RGB                *led_rgb_0,
                     LED_RGB                *led_rgb_1,
                     LSM303AGR              *lsm,
                     RFM96_Chip             *lora_chip,
                     W25Q_Chip              *flash_chip
) {
    components->adxl = adxl;
    components->bmi = bmi;
    components->bmp = bmp;
    components->buzzer = buzzer;
    components->gps = gps;
    components->led_rgb_0 = led_rgb_0;
    components->led_rgb_1 = led_rgb_1;
    components->lsm = lsm;
    components->lora = lora_chip;
    components->flash = flash_chip;
}

void init_all_components(COMPONENTS* components) {
    ADXL375_Init(components->adxl, &hspi1, CS_ACC1_GPIO_Port, CS_ACC1_Pin);

    BMI088_Init(components->bmi, &hspi1,
        CS_ACC0_GPIO_Port, CS_ACC0_Pin,
        CS_GRYO_GPIO_Port, CS_GRYO_Pin);

    components->bmp->hi2c = &hi2c3;
    BMP388_Init(components->bmp);

    BUZZER_Init(components->buzzer, &htim3, TIM_CHANNEL_4);
    BUZZER_set_song_bank(&buzzer_song_bank);

    GPS_Init(components->gps, &huart1);

    // LED_RGB_Init(components->led_rgb_0, &htim3, TIM_CHANNEL_3, TIM_CHANNEL_1, TIM_CHANNEL_2);

    // LED_RGB_Init(components->led_rgb_1, &htim4, TIM_CHANNEL_4, TIM_CHANNEL_2, TIM_CHANNEL_1);

    LSM303AGR_Init(components->lsm, &hi2c3);

    RFM96_Init(components->lora, &hspi1, CS_LORA_GPIO_Port, CS_LORA_Pin,
               RESET_LORA_GPIO_Port, RESET_LORA_Pin, 868250e3);

    W25Q_Init(components->flash, &hspi2, CS_FLASH_GPIO_Port, CS_FLASH_Pin);
}


// ===========================================


MACHINE machine;


void init_machine(MACHINE* machine, COMPONENTS* components,
                  FLASH_STREAM* flash_stream
                //   DATAS* datas
) {
    machine->state = SEND_ALL_MEMORY;

    machine->components = components;
    init_all_components(components);

    machine->flash_stream = flash_stream;
    flash_stream_init(flash_stream, components->flash, 0);

    // machine->datas = datas;
    // Datas_Init(datas, components->imu, components->bmp, components->gps);

    machine->usb_watchdog_bfr[0] = USBD_BUSY;
    machine->usb_watchdog_bfr[1] = USBD_BUSY;
    machine->usb_watchdog_bfr[2] = USBD_BUSY;

    machine->last_tick = 0;
    machine->last_time = HAL_GetTick();

    // ===== POST VOLE ===== POUR LIRE DIRECTEMENT ======
    // machine->state = WAIT_FOR_SENDING_DATA;
    // flash_stream->write_ptr = 0xffffff;
    // ===== POST VOLE ==================================
}



// void add_task_if_flag(SCHEDULER* scheduler, MACHINE* machine) {
//     switch (machine->state) {

//     case WAIT_USB: {
//         TASK *task_buz, *task_usb;

//         // task_buz = SCHEDULER_add_task_macro(scheduler, ASYNC_BUZZER_play_note, false);
//         // ASYNC_BUZZER_play_note_init(task_buz, machine->components->buzzer,
//         //                      buzzer_song_bank.beep_0_freqs,
//         //                      buzzer_song_bank.beep_0_durations, BUZZER_BEEP_SONG_SIZE);

//         task_usb = SCHEDULER_add_task_macro(scheduler, ASYNC_update_usb_watchdog, false);
//         ASYNC_update_usb_watchdog_init(task_usb);
//         task_usb->is_done = &(machine->is_done);

//         machine->state = USB_READY;
//         break; }
//     case USB_READY: {
//         if (machine->is_done) {
//             machine->state = TEST_W25Q_PAGE;
//         }
//         break; }

//     case TEST_ASYNC: {
//         machine->state = DEFAULT_STATE;

//         TASK *task_buz, *task_usb;

//         sprintf(machine->tx_buff, "IT'S WORKING, I CAN PRINT THINGS\n");

//         task_buz = SCHEDULER_add_task_macro(scheduler, ASYNC_BUZZER_play_note, false);
//         ASYNC_BUZZER_play_note_init(task_buz, machine->components->buzzer,
//                              buzzer_song_bank.cccp_freqs,
//                              buzzer_song_bank.cccp_durations,
//                              BUZZER_CCCP_SONG_SIZE);

//         task_usb = SCHEDULER_add_task_macro(scheduler, ASYNC_test_usb, false);
//         ASYNC_test_usb_init(task_usb, machine->tx_buff, 26, 200);

//         break; }
//     case TEST_BMI: {
//         machine->state = DEFAULT_STATE;

//         TASK *task_buz, *task_imu, *task_usb;

//         task_buz = SCHEDULER_add_task_macro(scheduler, ASYNC_BUZZER_play_note, false);
//         ASYNC_BUZZER_play_note_init(task_buz, machine->components->buzzer,
//                              buzzer_song_bank.beep_2_freqs,
//                              buzzer_song_bank.beep_2_durations, BUZZER_BEEP_SONG_SIZE);

//         task_imu = SCHEDULER_add_task_macro(scheduler, ASYNC_update_IMU, false);
//         ASYNC_update_IMU_init(task_imu, 10);

//         task_usb = SCHEDULER_add_task_macro(scheduler, ASYNC_send_usb, false);
//         ASYNC_send_usb_init(task_usb, 10);
//         break; }
//     case TEST_W25Q_PAGE: {
//         machine->state = DEFAULT_STATE;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_test_w25q_page, false);
//         ASYNC_test_w25q_page_init(task);

//         break; }
//     case TEST_W25Q_WRITE: {
//         machine->state = DEFAULT_STATE;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_test_w25q_write, false);
//         ASYNC_test_w25q_write_init(task);

//         break; }
//     case TEST_FLASH_STREAM: {
//         machine->state = DEFAULT_STATE;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_test_fs, false);
//         ASYNC_test_fs_init(task, machine->flash_stream);

//         break; }
//     case TEST_SAVE_IMU: {
//         machine->state = DEFAULT_STATE;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_save_IMU, false);
//         ASYNC_save_IMU_init(task);

//         break; }
//     case TEST_FILTERED_IMU: {
//         machine->state = DEFAULT_STATE;

//         TASK* task = SCHEDULER_add_task_macro(scheduler, ASYNC_filtred_IMU, false);
//         ASYNC_filtred_IMU_init(task);

//         break; }
//     case TEST_LSM303AGR: {
//         machine->state = DEFAULT_STATE;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_test_LSM303AGR, false);
//         ASYNC_test_LSM303AGR_init(task, machine->components->lsm);

//         break; }
//     case TEST_RFM96_Tx: {
//         machine->state = DEFAULT_STATE;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_test_RFM96_Tx, false);
//         ASYNC_test_RFM96_Tx_init(task, machine->components->lora);
//         break; }
//     case TEST_RFM96_Rx: {
//         machine->state = DEFAULT_STATE;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_test_RFM96_Rx, false);
//         ASYNC_test_RFM96_Rx_init(task, machine->components->lora);
//         break; }
//     case TEST_GPS: {
//         machine->state = DEFAULT_STATE;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_test_GPS, false);
//         ASYNC_test_GPS_init(task, machine->components->gps);

//         break; }

//     case W25Q_writing_test: {
//         machine->is_done = false;

//         for (int i = 0; i < 256; i++) {
//             machine->tx_buff[i] = i;
//         }
//         uint32_t addr = 0x0000000;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_PageProgram, false);
//         ASYNC_W25Q_PageProgram_init(task, machine->components->flash, (uint8_t*)machine->tx_buff, 256, addr);
//         task->is_done = &(machine->is_done);

//         machine->state = W25Q_WAIT;
//         break; }
//     case W25Q_WAIT: {
//         if (machine->is_done) {
//             machine->state = SEND_ALL_MEMORY;
//         }
//         break; }

//     case MEMORY_ERASE_PROCESS: {
//         machine->state = DEFAULT_STATE;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_Memory_Erase_Process, false);
//         ASYNC_Memory_Erase_Process_init(task, machine->components->flash);
//         break; }

//     case MESURE_ALL_DATA: {
//         machine->state = DEFAULT_STATE;

//         TASK *task_mesure_all_data, *task_save_all_data, *task_telem;
//         // TASK *task_mesure_all_data, *task_save_all_data;
//         ASYNC_test_SAVE_ALL_DATA_CONTEXT *context_save_all_data;

//         task_save_all_data = SCHEDULER_add_task_macro(scheduler, ASYNC_test_SAVE_ALL_DATA, false);
//         ASYNC_test_SAVE_ALL_DATA_init(task_save_all_data, machine);
//         context_save_all_data = (ASYNC_test_SAVE_ALL_DATA_CONTEXT*)(task_save_all_data->context);

//         task_mesure_all_data = SCHEDULER_add_task_macro(scheduler, ASYNC_test_MESURE_ALL_DATA, false);
//         ASYNC_test_MESURE_ALL_DATA_init(task_mesure_all_data, machine, &(context_save_all_data->data_pub));

//         task_telem = SCHEDULER_add_task_macro(scheduler, ASYNC_telem_gps, false);
//         ASYNC_telem_gps_init(task_telem, &(machine->datas), machine->components->lora);

//         break; }

//     case SEND_ALL_MEMORY: {
//         machine->state = DEFAULT_STATE;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_SEND_ALL_DATA_USB, false);
//         ASYNC_SEND_ALL_DATA_USB_init(task);

//         break; }

//     default:
//         break;
//     }
// }



void update_usb_watchdog(uint8_t *usb_watchdog_bfr) {
    usb_watchdog_bfr[0] = usb_watchdog_bfr[1];
    usb_watchdog_bfr[1] = usb_watchdog_bfr[2];
    usb_watchdog_bfr[2] = CDC_Transmit_FS(NULL, 0);
    HAL_Delay(1);
}

bool is_connected_to_usb(uint8_t *usb_watchdog_bfr) {
    return ((usb_watchdog_bfr[0] == USBD_OK) &&
            (usb_watchdog_bfr[1] == USBD_OK) &&
            (usb_watchdog_bfr[2] == USBD_OK));
}

// TASK_POOL_CREATE(ASYNC_update_usb_watchdog);

// // void ASYNC_update_usb_watchdog_init(TASK *self, MACHINE *machine) {
// void ASYNC_update_usb_watchdog_init(TASK *self) {
//     ASYNC_update_usb_watchdog_CONTEXT *context = (ASYNC_update_usb_watchdog_CONTEXT*)self->context;

//     // context->state = &(machine->state);
//     context->usb_watchdog_bfr[0] = USBD_BUSY;
//     context->usb_watchdog_bfr[1] = USBD_BUSY;
//     context->usb_watchdog_bfr[2] = USBD_BUSY;
//     context->next_time = 0;
// }

// TASK_RETURN ASYNC_update_usb_watchdog(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_update_usb_watchdog_CONTEXT *context = (ASYNC_update_usb_watchdog_CONTEXT*)self->context;

//     UNUSED(scheduler);

//     uint32_t current_time = HAL_GetTick();
//     if (current_time >= context->next_time) {
//         context->usb_watchdog_bfr[0] = context->usb_watchdog_bfr[1];
//         context->usb_watchdog_bfr[1] = context->usb_watchdog_bfr[2];
//         context->usb_watchdog_bfr[2] = CDC_Transmit_FS(NULL, 0);
//         context->next_time = HAL_GetTick() + 1;

//         if (is_connected_to_usb(context->usb_watchdog_bfr)) {
//             // *(context->state) = USB_READY;
//             return TASK_RETURN_STOP;
//         }
//     }

//     return TASK_RETURN_IDLE;
// }



// TASK_POOL_CREATE(ASYNC_test_usb);

// void ASYNC_test_usb_init(TASK *self, char *buff, int len, uint32_t delay) {
//     ASYNC_test_usb_CONTEXT *context = (ASYNC_test_usb_CONTEXT*)self->context;

//     context->buff = buff;
//     context->len = len;
//     context->index = -1;
//     context->next_time = 0;
//     context->delay = delay;
// }

// TASK_RETURN ASYNC_test_usb(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_test_usb_CONTEXT *context = (ASYNC_test_usb_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     uint32_t current_tick = HAL_GetTick();
//     if (context->index == -1) {
//         HAL_Delay(1);
//         CDC_Transmit_FS((uint8_t*)"START \"", 8);
//         context->next_time = current_tick + context->delay;
//         context->index = 0;
//     } else if (context->index < context->len) {
//         if (current_tick >= context->next_time) {
//             char* c = context->buff + context->index;
//             CDC_Transmit_FS((uint8_t*)c, 1);
//             context->index++;
//             context->next_time = current_tick + context->delay;
//         }
//     } else {
//         HAL_Delay(1);
//         CDC_Transmit_FS((uint8_t*)"\" END\n", 7);
//         return TASK_RETURN_STOP;
//     }
//     return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_send_usb);

// void ASYNC_send_usb_init(TASK *self, uint32_t delay) {
//     ASYNC_send_usb_CONTEXT *context = (ASYNC_send_usb_CONTEXT*)self->context;

//     context->next_time = 0;
//     context->delay = delay;
// }

// TASK_RETURN ASYNC_send_usb(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_send_usb_CONTEXT *context = (ASYNC_send_usb_CONTEXT*)self->context;

//     UNUSED(scheduler);

//     uint32_t current_tick = HAL_GetTick();

//     if (current_tick >= context->next_time) {
//         char buff[256];
//         char acc_x[10], acc_y[10], acc_z[10];
//         char gyr_x[10], gyr_y[10], gyr_z[10];
//         char time[10];
//         // int num_calls = machine.components->bmi->num_calls;

//         float_format(acc_x, machine.components->bmi->acc_mps2.x, 5, 10);
//         float_format(acc_y, machine.components->bmi->acc_mps2.y, 5, 10);
//         float_format(acc_z, machine.components->bmi->acc_mps2.z, 5, 10);

//         float_format(gyr_x, machine.components->bmi->gyr_rps.x, 5, 10);
//         float_format(gyr_y, machine.components->bmi->gyr_rps.y, 5, 10);
//         float_format(gyr_z, machine.components->bmi->gyr_rps.z, 5, 10);

//         float_format(time, (float)current_tick, 1, 10);

//         sprintf(buff, "%s, %s, %s, %s, %s, %s, %s\n", time, acc_x, acc_y, acc_z,
//                                                             gyr_x, gyr_y, gyr_z);
//         // sprintf(buff, "%s, %s, %s\r\n", acc_x, acc_y, acc_z);
//         // sprintf(buff, "%s, %s, %s\r\n", gyr_x, gyr_y, gyr_z);
//         CDC_Transmit_FS((uint8_t*)buff, strlen(buff));

//         context->next_time = current_tick + context->delay;
//     }
//     return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_update_IMU);

// void ASYNC_update_IMU_init(TASK *self, uint32_t delay) {
//     ASYNC_update_IMU_CONTEXT *context = (ASYNC_update_IMU_CONTEXT*)self->context;

//     context->continue_update = true;
//     context->next_time = 0;
//     context->delay = delay;
// }

// TASK_RETURN ASYNC_update_IMU(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_update_IMU_CONTEXT *context = (ASYNC_update_IMU_CONTEXT*)self->context;

//     uint32_t current_tick = HAL_GetTick();
//     if (context->continue_update) {
//         if (current_tick >= context->next_time) {

//             TASK *task_acc, *task_gyr;

//             task_acc = SCHEDULER_add_task(scheduler, ASYNC_BMI088_ReadAccelerometerDMA,
//                 true, (OBJ_POOL*)ASYNC_BMI088_ReadSensorDMA_POOL);
//             ASYNC_BMI088_ReadSensorDMA_init(task_acc, machine.components->bmi);

//             task_gyr = SCHEDULER_add_task(scheduler, ASYNC_BMI088_ReadGyroscopeDMA,
//                 true, (OBJ_POOL*)ASYNC_BMI088_ReadSensorDMA_POOL);
//             ASYNC_BMI088_ReadSensorDMA_init(task_gyr, machine.components->bmi);

//             context->next_time = current_tick + context->delay;
//         }
//     } else {
//         return TASK_RETURN_STOP;
//     }
//     return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_test_w25q_page);

// void ASYNC_test_w25q_page_init(TASK *self) {
//     ASYNC_test_w25q_page_CONTEXT *context = (ASYNC_test_w25q_page_CONTEXT*)self->context;

//     context->flash_chip = machine.components->flash;

//     context->tx_buf = GMS_alloc(&GMS_memory, ASYNC_test_w25q_page_SIZE);
//     context->rx_buf = GMS_alloc(&GMS_memory, ASYNC_test_w25q_page_SIZE + ASYNC_test_w25q_page_ADDR);

//     uint8_t tx_buf[ASYNC_test_w25q_page_SIZE];

//     for (size_t i = 0; i < ASYNC_test_w25q_page_SIZE; i++) {
//         tx_buf[i] = i;
//         // tx_buf[i] = ASYNC_test_w25q_page_SIZE - i;
//     }
//     memcpy(context->tx_buf, tx_buf, ASYNC_test_w25q_page_SIZE);
//     memset(context->rx_buf, 0, ASYNC_test_w25q_page_SIZE + ASYNC_test_w25q_page_ADDR);

//     context->state = ASYNC_test_w25q_page_START;
// }

// TASK_RETURN ASYNC_test_w25q_page(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_test_w25q_page_CONTEXT *context = (ASYNC_test_w25q_page_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     switch (context->state) {
//     case ASYNC_test_w25q_page_START: {
//         context->is_done = false;
//         // TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_EraseSector, false);
//         // ASYNC_W25Q_EraseSector_init(task, context->flash_chip, 0);
//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_EraseAll, false);
//         ASYNC_W25Q_EraseAll_init(task, machine.components->flash);
//         task->is_done = &(context->is_done);
//         context->state = ASYNC_test_w25q_page_WAIT_ERASE;
//         // context->state = ASYNC_test_w25q_page_WAIT_TX;
//         break; }
//     case ASYNC_test_w25q_page_WAIT_ERASE: {
//         if (context->is_done) {
//             context->is_done = false;
//             TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_PageProgram, false);
//             ASYNC_W25Q_PageProgram_init(task, context->flash_chip, context->tx_buf, ASYNC_test_w25q_page_SIZE, ASYNC_test_w25q_page_ADDR);
//             task->is_done = &(context->is_done);
//             context->state = ASYNC_test_w25q_page_WAIT_TX;
//         }
//         break; }
//     case ASYNC_test_w25q_page_WAIT_TX: {
//         if (context->is_done) {
//             context->is_done = false;
//             TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ReadData, false);
//             ASYNC_W25Q_ReadData_init(task, context->flash_chip, context->rx_buf, ASYNC_test_w25q_page_ADDR + ASYNC_test_w25q_page_SIZE, 0);
//             task->is_done = &(context->is_done);
//             context->state = ASYNC_test_w25q_page_WAIT_RX;
//         }
//         break; }
//     case ASYNC_test_w25q_page_WAIT_RX: {
//         if (context->is_done) {
//             for (size_t i = 0; i < ASYNC_test_w25q_page_SIZE; i++) {
//                 if (context->tx_buf[i] != context->rx_buf[i + ASYNC_test_w25q_page_ADDR]) {
//                     break;
//                 }
//             }

//             GMS_free(&GMS_memory, context->tx_buf);
//             GMS_free(&GMS_memory, context->rx_buf);
//             return TASK_RETURN_STOP;
//         }
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_test_w25q_write);

// void ASYNC_test_w25q_write_init(TASK *self) {
//     ASYNC_test_w25q_write_CONTEXT *context = (ASYNC_test_w25q_write_CONTEXT*)self->context;

//     context->flash_chip = machine.components->flash;

//     uint8_t tx_buf[ASYNC_test_w25q_write_SIZE];

//     for (size_t i = 0; i < ASYNC_test_w25q_write_SIZE; i++) {
//         tx_buf[i] = i;
//     }
    
//     context->tx_buf = GMS_alloc(&GMS_memory, ASYNC_test_w25q_write_SIZE);
//     context->rx_buf = GMS_alloc(&GMS_memory, ASYNC_test_w25q_write_SIZE + ASYNC_test_w25q_write_ADDR);

//     memcpy(context->tx_buf, tx_buf, ASYNC_test_w25q_write_SIZE);
//     memset(context->rx_buf, 0, ASYNC_test_w25q_write_SIZE + ASYNC_test_w25q_write_ADDR);

//     context->state = ASYNC_test_w25q_write_START;
// }

// TASK_RETURN ASYNC_test_w25q_write(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_test_w25q_write_CONTEXT *context = (ASYNC_test_w25q_write_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     switch (context->state) {
//     case ASYNC_test_w25q_write_START: {
//         context->is_done = false;
//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_EraseSector, false);
//         ASYNC_W25Q_EraseSector_init(task, context->flash_chip, 0);
//         task->is_done = &(context->is_done);
//         context->state = ASYNC_test_w25q_write_WAIT_ERASE;
//         break; }
//     case ASYNC_test_w25q_write_WAIT_ERASE: {
//         if (context->is_done) {
//             context->is_done = false;
//             TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WriteData, false);
//             ASYNC_W25Q_WriteData_init(task, context->flash_chip, context->tx_buf, ASYNC_test_w25q_write_SIZE, ASYNC_test_w25q_write_ADDR);
//             task->is_done = &(context->is_done);
//             context->state = ASYNC_test_w25q_write_WAIT_TX;
//         }
//         break; }
//     case ASYNC_test_w25q_write_WAIT_TX: {
//         if (context->is_done) {
//             context->is_done = false;
//             TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ReadData, false);
//             ASYNC_W25Q_ReadData_init(task, context->flash_chip, context->rx_buf, ASYNC_test_w25q_write_SIZE + ASYNC_test_w25q_write_ADDR, 0);
//             task->is_done = &(context->is_done);
//             context->state = ASYNC_test_w25q_write_WAIT_RX;
//         }
//         break; }
//     case ASYNC_test_w25q_write_WAIT_RX: {
//         if (context->is_done) {
//             for (size_t i = 0; i < ASYNC_test_w25q_write_SIZE; i++) {
//                 if (context->tx_buf[i] != context->rx_buf[i + ASYNC_test_w25q_write_ADDR]) {
//                     break;
//                 }
//             }

//             GMS_free(&GMS_memory, context->tx_buf);
//             GMS_free(&GMS_memory, context->rx_buf);
//             return TASK_RETURN_STOP;
//         }
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_test_fs);

// void ASYNC_test_fs_init(TASK *self, FLASH_STREAM *flash_stream) {
//     ASYNC_test_fs_CONTEXT *context = (ASYNC_test_fs_CONTEXT*)self->context;

//     context->flash_stream = flash_stream;

//     float tx[ASYNC_test_fs_float_SIZE];

//     for (int i = 0; i < ASYNC_test_fs_packet_SIZE; i++) {
//         tx[i + 0] = 31.4 + 1/((float)i + 2.0);
//         tx[i + 4] = 69.0 + 1/((float)i + 2.0);
//         tx[i + 8] = 42.0 + 1/((float)i + 2.0);
//         // ASYNC_test_fs_nb_packet == 3
//     }

//     context->tx_buf = GMS_alloc(&GMS_memory, ASYNC_test_fs_uint8_SIZE);
//     context->rx_buf = GMS_alloc(&GMS_memory, ASYNC_test_fs_uint8_SIZE);

//     memcpy(context->tx_buf, tx, ASYNC_test_fs_uint8_SIZE);

//     context->idx = 0;
//     context->state = ASYNC_test_fs_START;
// }

// TASK_RETURN ASYNC_test_fs(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_test_fs_CONTEXT *context = (ASYNC_test_fs_CONTEXT*)self->context;

//     switch (context->state) {
//     case ASYNC_test_fs_START: {
//         context->is_done = false;
//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_EraseSector, false);
//         ASYNC_W25Q_EraseSector_init(task, context->flash_stream->flash_chip, 0);
//         task->is_done = &(context->is_done);
//         context->state = ASYNC_test_fs_WAIT_ERASE;
//         break; }
//     case ASYNC_test_fs_WAIT_ERASE: {
//         if (context->is_done) {
//             context->state = ASYNC_test_fs_START_TX;
//         }
//         break; }
//     case ASYNC_test_fs_START_TX: {
//         context->is_done = false;

//         uint8_t *tx = context->tx_buf + context->idx * ASYNC_test_fs_packet_SIZE * sizeof(float);

//         TASK *task = SCHEDULER_add_task(scheduler, ASYNC_fs_write, false, (OBJ_POOL*)ASYNC_fs_read_write_POOL);
//         ASYNC_fs_read_write_init(task, context->flash_stream, tx, ASYNC_test_fs_packet_SIZE * sizeof(float));

//         task->is_done = &(context->is_done);
//         context->state = ASYNC_test_fs_WAIT_TX;
//         break; }
//     case ASYNC_test_fs_WAIT_TX: {
//         if (context->is_done) {
//             if (context->idx < ASYNC_test_fs_nb_packet) {
//                 context->idx += 1;
//                 context->state = ASYNC_test_fs_START_TX;
//             } else {
//                 context->idx = 0;
//                 context->state = ASYNC_test_fs_START_RX;
//             }
//         }
//         break; }
//     case ASYNC_test_fs_START_RX: {
//         context->is_done = false;

//         uint8_t *rx = context->rx_buf + context->idx * ASYNC_test_fs_packet_SIZE * sizeof(float);

//         TASK *task = SCHEDULER_add_task(scheduler, ASYNC_fs_read, false, (OBJ_POOL*)ASYNC_fs_read_write_POOL);
//         ASYNC_fs_read_write_init(task, context->flash_stream, rx, ASYNC_test_fs_packet_SIZE * sizeof(float));
        
//         task->is_done = &(context->is_done);
//         context->state = ASYNC_test_fs_WAIT_RX;
//         break; }
//     case ASYNC_test_fs_WAIT_RX: {
//         if (context->is_done) {
//             if (context->idx < ASYNC_test_fs_nb_packet) {
//                 context->idx += 1;
//                 context->state = ASYNC_test_fs_START_RX;
//             } else {
//                 context->state = ASYNC_test_fs_DONE_RX;
//             }
//         }
//         break; }
//     case ASYNC_test_fs_DONE_RX: {

//         for (size_t i = 0; i < ASYNC_test_fs_uint8_SIZE; i++) {
//             if (context->tx_buf[i] != context->rx_buf[i]) {
//                 break;
//             }
//         }

//         GMS_free(&GMS_memory, context->tx_buf);
//         GMS_free(&GMS_memory, context->rx_buf);
//         return TASK_RETURN_STOP;
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_save_IMU);

// void ASYNC_save_IMU_init(TASK *self) {
//     ASYNC_save_IMU_CONTEXT *context = (ASYNC_save_IMU_CONTEXT*)self->context;

//     context->max_delay = 30000;  // 30 seconds
//     context->imu_delay = 10;     // 10 ms
//     context->state = ASYNC_save_IMU_START_ERASE;

//     context->ready = false;
//     context->save_done = false;
// }

// TASK_RETURN ASYNC_save_IMU(SCHEDULER* scheduler, TASK* self) {
//     ASYNC_save_IMU_CONTEXT *context = (ASYNC_save_IMU_CONTEXT*)self->context;

//     uint32_t current_tick = HAL_GetTick();

//     switch (context->state) {
//     case ASYNC_save_IMU_START_ERASE: {
//         context->ready = false;
//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_EraseAll, false);
//         ASYNC_W25Q_EraseAll_init(task, machine.flash_stream->flash_chip);
//         task->is_done = &(context->ready);
//         context->state = ASYNC_save_IMU_START_WAITING;
//         break; }
//     case ASYNC_save_IMU_START_WAITING: {
//         if (context->ready) {
//             context->ready = false;
//             TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WaitForReady, false);
//             ASYNC_W25Q_WaitForReady_init(task, machine.flash_stream->flash_chip);
//             task->is_done = &(context->ready); 
//             context->state = ASYNC_save_IMU_WAIT_READY;
//         }
//         break; }
//     case ASYNC_save_IMU_WAIT_READY: {
//         if (context->ready) {
//             TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_BUZZER_play_note, false);
//             ASYNC_BUZZER_play_note_init(task, machine.components->buzzer,
//                                  buzzer_song_bank.beep_1_freqs,
//                                  buzzer_song_bank.beep_1_durations,
//                                  BUZZER_BEEP_SONG_SIZE);
            
//             CDC_Transmit_FS((uint8_t*)"accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,time\n", 50);
            
//             context->state = ASYNC_save_IMU_START_UPDATE;
//         }
//         break; }
//     case ASYNC_save_IMU_START_UPDATE: {
//         context->first_time = current_tick;
//         context->task_imu = SCHEDULER_add_task_macro(scheduler, ASYNC_update_IMU, true);
//         ASYNC_update_IMU_init(context->task_imu, context->imu_delay);

//         context->state = ASYNC_save_IMU_WAIT_UPDATE;
//         break; }
//     case ASYNC_save_IMU_WAIT_UPDATE: {
//         if (machine.components->bmi->new_acc_data && machine.components->bmi->new_gyr_data) {
//             machine.components->bmi->new_acc_data = false;
//             machine.components->bmi->new_gyr_data = false;

//             context->save_done = false;

//             context->datas_buff[0] = machine.components->bmi->acc_mps2.x;
//             context->datas_buff[1] = machine.components->bmi->acc_mps2.y;
//             context->datas_buff[2] = machine.components->bmi->acc_mps2.z;
//             context->datas_buff[3] = machine.components->bmi->gyr_rps.x;
//             context->datas_buff[4] = machine.components->bmi->gyr_rps.y;
//             context->datas_buff[5] = machine.components->bmi->gyr_rps.z;
//             context->datas_buff[6] = (float)current_tick;

//             TASK *task = SCHEDULER_add_task(scheduler, ASYNC_fs_write, false, (OBJ_POOL*)ASYNC_fs_read_write_POOL);
//             ASYNC_fs_read_write_init(task, machine.flash_stream, (uint8_t*)(context->datas_buff), ASYNC_save_IMU_DATA_NUMBER * sizeof(float));
//             task->is_done = &(context->save_done);

//             context->state = ASYNC_save_IMU_WAIT_SAVE;
//         }
//         break; }
//     case ASYNC_save_IMU_WAIT_SAVE: {
//         if (context->save_done) {
//             if (current_tick - context->first_time >= context->max_delay) {
//                 context->state = ASYNC_save_IMU_END_UPDATE;
//             } else {
//                 context->state = ASYNC_save_IMU_WAIT_UPDATE;
//             }
//         }
//         break; }
//     case ASYNC_save_IMU_END_UPDATE: {
//         ((ASYNC_update_IMU_CONTEXT*)(context->task_imu->context))->continue_update = false;

//         context->state = ASYNC_save_IMU_START_LOAD;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_BUZZER_play_note, false);
//         ASYNC_BUZZER_play_note_init(task, machine.components->buzzer,
//                              buzzer_song_bank.beep_2_freqs,
//                              buzzer_song_bank.beep_2_durations,
//                              BUZZER_BEEP_SONG_SIZE);
//         break; }
//     case ASYNC_save_IMU_START_LOAD: {
//         context->save_done = false;

//         TASK *task = SCHEDULER_add_task(scheduler, ASYNC_fs_read, false, (OBJ_POOL*)ASYNC_fs_read_write_POOL);
//         ASYNC_fs_read_write_init(task, machine.flash_stream, (uint8_t*)(context->datas_buff), ASYNC_save_IMU_DATA_NUMBER * sizeof(float));
//         task->is_done = &(context->save_done);

//         context->state = ASYNC_save_IMU_WAIT_LOAD;
//         break; }
//     case ASYNC_save_IMU_WAIT_LOAD: {
//         if (context->save_done) {
//             context->state = ASYNC_save_IMU_START_SEND;
//         }
//         break; }
//     case ASYNC_save_IMU_START_SEND: {
//         context->next_time = current_tick + 2;
//         char buff[256];
//         char acc_x[10], acc_y[10], acc_z[10];
//         char gyr_x[10], gyr_y[10], gyr_z[10];
//         char time[10];

//         float_format(acc_x, context->datas_buff[0], 5, 10);
//         float_format(acc_y, context->datas_buff[1], 5, 10);
//         float_format(acc_z, context->datas_buff[2], 5, 10);

//         float_format(gyr_x, context->datas_buff[3], 5, 10);
//         float_format(gyr_y, context->datas_buff[4], 5, 10);
//         float_format(gyr_z, context->datas_buff[5], 5, 10);

//         float_format(time, context->datas_buff[6], 1, 10);

//         sprintf(buff, "%s, %s, %s, %s, %s, %s, %s\r\n", acc_x, acc_y, acc_z,
//                                                         gyr_x, gyr_y, gyr_z, time);
//         CDC_Transmit_FS((uint8_t*)buff, strlen(buff));

//         context->state = ASYNC_save_IMU_WAIT_SEND;
//         break; }
//     case ASYNC_save_IMU_WAIT_SEND: {
//         if (current_tick >= context->next_time) {
//             if (machine.flash_stream->read_ptr < machine.flash_stream->write_ptr) {
//                 context->state =  ASYNC_save_IMU_START_LOAD;
//             } else {
//                 context->state = ASYNC_save_IMU_END_SEND;
//             }
//         }
//         break; }
//     case ASYNC_save_IMU_END_SEND: {
//         return TASK_RETURN_STOP;
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_filtred_IMU);

// void ASYNC_filtred_IMU_init(TASK *self) {
//     ASYNC_filtred_IMU_CONTEXT *context = (ASYNC_filtred_IMU_CONTEXT*)self->context;

//     context->acc_offset_num = 0;
//     context->gyr_offset_num = 0;

//     context->acc_offset_mean[0] = 0;
//     context->acc_offset_mean[1] = 0;
//     context->acc_offset_mean[2] = 0;

//     context->gyr_offset_mean[0] = 0;
//     context->gyr_offset_mean[1] = 0;
//     context->gyr_offset_mean[2] = 0;

//     context->state = ASYNC_filtred_IMU_START;    
// }

// TASK_RETURN ASYNC_filtred_IMU(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_filtred_IMU_CONTEXT *context = (ASYNC_filtred_IMU_CONTEXT*)self->context;

//     uint32_t current_tick = HAL_GetTick();

//     switch (context->state) {
//     case ASYNC_filtred_IMU_START: {
//         context->w25q_ready = false;
//         TASK *task;
//         task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_EraseAll, false);
//         ASYNC_W25Q_EraseAll_init(task, machine.flash_stream->flash_chip);
//         task->is_done = &(context->w25q_ready);

//         task = SCHEDULER_add_task(scheduler, ASYNC_BMI088_Accelero_Publisher, true, (OBJ_POOL*)ASYNC_BMI088_Sensor_Publisher_POOL);
//         ASYNC_BMI088_Sensor_Publisher_init(task, machine.components->bmi, &(context->acc_data), 10, 16);

//         task = SCHEDULER_add_task(scheduler, ASYNC_BMI088_Gyro_Publisher, true, (OBJ_POOL*)ASYNC_BMI088_Sensor_Publisher_POOL);
//         ASYNC_BMI088_Sensor_Publisher_init(task, machine.components->bmi, &(context->gyr_data), 10, 16);

//         context->state = ASYNC_filtred_IMU_START_WAITING_W25Q;
//         break; }
//     case ASYNC_filtred_IMU_START_WAITING_W25Q: {
//         if (context->w25q_ready) {
//             context->w25q_ready = false;
//             TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WaitForReady, false);
//             ASYNC_W25Q_WaitForReady_init(task, machine.flash_stream->flash_chip);
//             task->is_done = &(context->w25q_ready);
//             context->state = ASYNC_filtred_IMU_WAIT_OFFSET_BMI_AND_W25Q;
//         }
//         break; }
//     case ASYNC_filtred_IMU_WAIT_OFFSET_BMI_AND_W25Q: {
//         if (!context->w25q_ready || context->acc_offset_num < 100 || context->gyr_offset_num < 100) {
//             FLOAT3_TIMESTAMP data_time;
//             if (context->acc_offset_num < 100) {
//                 size_t new_acc_num = context->acc_data.new_data_num;
//                 for (size_t i = 0; i < new_acc_num; i++) {
//                     DATA_PUB_pop(&(context->acc_data), &data_time);
//                     context->acc_offset[0][context->acc_offset_num + i] = data_time.data.x;
//                     context->acc_offset[1][context->acc_offset_num + i] = data_time.data.y;
//                     context->acc_offset[2][context->acc_offset_num + i] = data_time.data.z;
//                 }
//                 context->acc_offset_num += new_acc_num;
//             }
//             if (context->gyr_offset_num < 100) {
//                 size_t new_gyr_num = context->gyr_data.new_data_num;
//                 for (size_t i = 0; i < new_gyr_num; i++) {
//                     DATA_PUB_pop(&(context->gyr_data), &data_time);
//                     context->gyr_offset[0][context->gyr_offset_num + i] = data_time.data.x;
//                     context->gyr_offset[1][context->gyr_offset_num + i] = data_time.data.y;
//                     context->gyr_offset[2][context->gyr_offset_num + i] = data_time.data.z;
//                 }
//                 context->gyr_offset_num += new_gyr_num;
//             }
//         } else {
//             context->state = ASYNC_filtred_IMU_OFFSET_CALCULUS;
//         }
//         break; }
//     case ASYNC_filtred_IMU_OFFSET_CALCULUS: {

//         for (size_t i = 0; i < context->acc_offset_num; i++) {
//             context->acc_offset_mean[0] += context->acc_offset[0][i];
//             context->acc_offset_mean[1] += context->acc_offset[1][i];
//             context->acc_offset_mean[2] += context->acc_offset[2][i];

//             context->gyr_offset_mean[0] += context->gyr_offset[0][i];
//             context->gyr_offset_mean[1] += context->gyr_offset[1][i];
//             context->gyr_offset_mean[2] += context->gyr_offset[2][i];
//         }

//         context->acc_offset_mean[0] /= context->acc_offset_num;
//         context->acc_offset_mean[1] /= context->acc_offset_num;
//         context->acc_offset_mean[2] /= context->acc_offset_num;

//         context->gyr_offset_mean[0] /= context->gyr_offset_num;
//         context->gyr_offset_mean[1] /= context->gyr_offset_num;
//         context->gyr_offset_mean[2] /= context->gyr_offset_num;

//         char acc_x[10], acc_y[10], acc_z[10];
//         char gyr_x[10], gyr_y[10], gyr_z[10];

//         float_format(acc_x, context->acc_offset_mean[0], 5, 10);
//         float_format(acc_y, context->acc_offset_mean[1], 5, 10);
//         float_format(acc_z, context->acc_offset_mean[2], 5, 10);
        
//         float_format(gyr_x, context->gyr_offset_mean[0], 5, 10);
//         float_format(gyr_y, context->gyr_offset_mean[1], 5, 10);
//         float_format(gyr_z, context->gyr_offset_mean[2], 5, 10);

//         char buff[256];

//         sprintf(buff, "acc_offset: %s, %s, %s\ngyr_offset: %s, %s, %s\n",
//                 acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z);
//         CDC_Transmit_FS((uint8_t*)buff, strlen(buff));

//         return TASK_RETURN_STOP;
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }



// TASK_POOL_CREATE(ASYNC_test_LSM303AGR);

// void ASYNC_test_LSM303AGR_init(TASK *self, LSM303AGR *lsm) {
//     ASYNC_test_LSM303AGR_CONTEXT *context = (ASYNC_test_LSM303AGR_CONTEXT*)self->context;

//     context->lsm = lsm;

//     context->is_ready = false;

//     // context->delay_us = 10000; // 0 second

//     context->state = ASYNC_test_LSM303AGR_READ_ACCELEROMETER;
// }

// TASK_RETURN ASYNC_test_LSM303AGR(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_test_LSM303AGR_CONTEXT *context = (ASYNC_test_LSM303AGR_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     switch (context->state) {
//     case ASYNC_test_LSM303AGR_WAIT_READY: {
//         if (context->is_ready) {
//             context->state = context->next_state;
//         }
//         break; }
//     case ASYNC_test_LSM303AGR_READ_ACCELEROMETER: {
//         context->is_ready = false;

//         TASK *task = SCHEDULER_add_task(scheduler, ASYNC_LSM303AGR_ReadAcc, true, (OBJ_POOL*)ASYNC_LSM303AGR_ReadSensor_POOL);
//         ASYNC_LSM303AGR_ReadSensor_init(task, context->lsm, &(context->acc));
//         task->is_done = &(context->is_ready);

//         context->next_state = ASYNC_test_LSM303AGR_READ_MAGNETOMETER;
//         context->state = ASYNC_test_LSM303AGR_WAIT_READY;
//         break; }
//     case ASYNC_test_LSM303AGR_READ_MAGNETOMETER: {
//         context->is_ready = false;

//         TASK *task = SCHEDULER_add_task(scheduler, ASYNC_LSM303AGR_ReadMag, true, (OBJ_POOL*)ASYNC_LSM303AGR_ReadSensor_POOL);
//         ASYNC_LSM303AGR_ReadSensor_init(task, context->lsm, &(context->mag));
//         task->is_done = &(context->is_ready);

//         context->next_state = ASYNC_test_LSM303AGR_PRINT;
//         context->state = ASYNC_test_LSM303AGR_WAIT_READY;
//         break; }
//     case ASYNC_test_LSM303AGR_PRINT: {
//         char buff[256];
//         char acc_x[10], acc_y[10], acc_z[10];
//         char mag_x[10], mag_y[10], mag_z[10];
//         uint32_t current_micro = GetMicros();

//         float_format(acc_x, context->acc.x, 5, 10);
//         float_format(acc_y, context->acc.y, 5, 10);
//         float_format(acc_z, context->acc.z, 5, 10);

//         float_format(mag_x, context->mag.x, 5, 10);
//         float_format(mag_y, context->mag.y, 5, 10);
//         float_format(mag_z, context->mag.z, 5, 10);

//         sprintf(buff, "==== Time ====\n%lu\n==== Acc ====\n x: %s\ny: %s\nz: %s\n==== Mag ====\nx: %s\ny: %s\nz: %s\n\n",
//                 current_micro,
//                 acc_x, acc_y, acc_z,
//                 mag_x, mag_y, mag_z);
//         CDC_Transmit_FS((uint8_t*)buff, strlen(buff));

//         context->state = ASYNC_test_LSM303AGR_DELAY;
//         break; }
//     case ASYNC_test_LSM303AGR_DELAY: {
//         context->is_ready = false;

//         TASK *task = SCHEDULER_add_task(scheduler, ASYNC_Delay_ms, false, (OBJ_POOL*)ASYNC_Delay_POOL);
//         ASYNC_Delay_ms_init(task, 5); // 5 ms delay
//         task->is_done = &(context->is_ready);

//         context->next_state = ASYNC_test_LSM303AGR_READ_ACCELEROMETER;
//         context->state = ASYNC_test_LSM303AGR_WAIT_READY;
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }





// TASK_POOL_CREATE(ASYNC_test_RFM96_Tx);

// void ASYNC_test_RFM96_Tx_init(TASK *self, RFM96_Chip *rfm96_chip) {
//     ASYNC_test_RFM96_Tx_CONTEXT *context = (ASYNC_test_RFM96_Tx_CONTEXT*)self->context;

//     context->rfm96_chip = rfm96_chip;

//     context->is_ready = false;

//     char buff[256] = "Ceci est un test de transmission RFM96\n";

//     memcpy(context->tx_buf, buff, strlen(buff) + 1);
//     context->tx_size = strlen(buff) + 1;

//     context->state = ASYNC_test_RFM96_Tx_SEND_PACKET;
// }

// TASK_RETURN ASYNC_test_RFM96_Tx(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_test_RFM96_Tx_CONTEXT *context = (ASYNC_test_RFM96_Tx_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     switch (context->state) {
//     case ASYNC_test_RFM96_Tx_WAIT_READY: {
//         if (context->is_ready) {
//             context->state = context->next_state;
//         }
//         break; }
//     case ASYNC_test_RFM96_Tx_SEND_PACKET: {
//         context->is_ready = false;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_RFM96_SendPacket, false);
//         ASYNC_RFM96_SendPacket_init(task, context->rfm96_chip, context->tx_buf, strlen(context->tx_buf) + 1);
//         task->is_done = &(context->is_ready);

//         context->next_state = ASYNC_test_RFM96_Tx_DELAY;
//         context->state = ASYNC_test_RFM96_Tx_WAIT_READY;
//         break; }
//     case ASYNC_test_RFM96_Tx_DELAY: {
//         context->is_ready = false;

//         TASK *task = SCHEDULER_add_task(scheduler, ASYNC_Delay_ms, false, (OBJ_POOL*)ASYNC_Delay_POOL);
//         ASYNC_Delay_ms_init(task, 1000); // 1 second delay
//         task->is_done = &(context->is_ready);

//         context->next_state = ASYNC_test_RFM96_Tx_SEND_PACKET;
//         context->state = ASYNC_test_RFM96_Tx_WAIT_READY;
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_test_RFM96_Rx);

// void ASYNC_test_RFM96_Rx_init(TASK *self, RFM96_Chip *rfm96_chip) {
//     ASYNC_test_RFM96_Rx_CONTEXT *context = (ASYNC_test_RFM96_Rx_CONTEXT*)self->context;

//     context->rfm96_chip = rfm96_chip;

//     context->is_ready = false;

//     memset(context->rx_buf, 0, 256);

//     context->state = ASYNC_test_RFM96_Rx_RECEIVE_PACKET;
// }

// TASK_RETURN ASYNC_test_RFM96_Rx(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_test_RFM96_Rx_CONTEXT *context = (ASYNC_test_RFM96_Rx_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     switch (context->state) {
//     case ASYNC_test_RFM96_Rx_WAIT_READY: {
//         if (context->is_ready) {
//             context->state = context->next_state;
//         }
//         break; }
//     case ASYNC_test_RFM96_Rx_SET_RX_MODE: {
//         context->is_ready = false;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_RFM96_SetRxMode, false);
//         ASYNC_RFM96_SetRxMode_init(task, context->rfm96_chip);
//         task->is_done = &(context->is_ready);

//         context->next_state = ASYNC_test_RFM96_Rx_RECEIVE_PACKET;
//         context->state = ASYNC_test_RFM96_Rx_WAIT_READY;
//         break;
//     }
//     case ASYNC_test_RFM96_Rx_RECEIVE_PACKET: {
//         context->is_ready = false;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_RFM96_ParsePacket, false);
//         ASYNC_RFM96_ParsePacket_init(task, context->rfm96_chip, context->rx_buf, &(context->rx_size));
//         task->is_done = &(context->is_ready);

//         context->next_state = ASYNC_test_RFM96_Rx_RECEIVE_PACKET_DONE;
//         context->state = ASYNC_test_RFM96_Rx_WAIT_READY;
//         break; }
//     case ASYNC_test_RFM96_Rx_RECEIVE_PACKET_DONE: {
//         if (context->rx_size > 0) {
//             CDC_Transmit_FS((uint8_t*)context->rx_buf, context->rx_size);
//             HAL_Delay(1);
//             context->rx_size = 0; // Reset size for next reception
//             memset(context->rx_buf, 0, 256); // Clear buffer for next reception
//             context->state = ASYNC_test_RFM96_Rx_SET_RX_MODE;
//         } else {
//             context->state = ASYNC_test_RFM96_Rx_RECEIVE_PACKET;
//         }
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }






// TASK_POOL_CREATE(ASYNC_test_GPS);

// void ASYNC_test_GPS_init(TASK *self, GPS_t *gps) {
//     ASYNC_test_GPS_CONTEXT *context = (ASYNC_test_GPS_CONTEXT*)self->context;

//     context->gps = gps;

//     context->is_ready = false;

//     context->state = ASYNC_test_GPS_READ;
// }

// TASK_RETURN ASYNC_test_GPS(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_test_GPS_CONTEXT *context = (ASYNC_test_GPS_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     switch (context->state) {
//     case ASYNC_test_GPS_WAIT_READY: {
//         if (context->is_ready) {
//             context->state = context->next_state;
//         }
//         break; }
//     case ASYNC_test_GPS_READ: {
//         context->is_ready = false;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_UART_RxToIdle_DMA, false);
//         ASYNC_UART_RxToIdle_DMA_init_GPS(task, machine.components->gps);
//         // ASYNC_GPS_ReadSensor_init(task, context->gps);
//         task->is_done = &(context->is_ready);

//         context->next_state = ASYNC_test_GPS_PRINT;
//         context->state = ASYNC_test_GPS_WAIT_READY;
//         break; }
//     case ASYNC_test_GPS_PRINT: {
//         char buff[256];
//         char lat[20], lon[20], alt[10];
//         uint32_t current_ms = HAL_GetTick();

//         float_format(lat, context->gps->dec_latitude, 5, 10);
//         float_format(lon, context->gps->dec_longitude, 5, 10);
//         float_format(alt, context->gps->altitude_ft, 5, 10);

//         sprintf(buff, "==== Time ====\n%lu\n==== GPS ====\nLatitude: %s\nLongitude: %s\nAltitude: %s\n\n",
//                 current_ms,
//                 lat, lon, alt);
//         CDC_Transmit_FS((uint8_t*)buff, strlen(buff));

//         context->state = ASYNC_test_GPS_DELAY;
//         break; }
//     case ASYNC_test_GPS_DELAY: {
//         context->is_ready = false;

//         TASK *task = SCHEDULER_add_task(scheduler, ASYNC_Delay_ms, false, (OBJ_POOL*)ASYNC_Delay_POOL);
//         ASYNC_Delay_ms_init(task, 1000); // 1 second delay
//         task->is_done = &(context->is_ready);

//         context->next_state = ASYNC_test_GPS_READ;
//         context->state = ASYNC_test_GPS_WAIT_READY;
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_test_MESURE_ALL_DATA);

// void ASYNC_test_MESURE_ALL_DATA_init(TASK *self, MACHINE *machine, DATA_PUB *data_pub) {
//     ASYNC_test_MESURE_ALL_DATA_CONTEXT *context = (ASYNC_test_MESURE_ALL_DATA_CONTEXT*)self->context;

//     context->state = ASYNC_test_MESURE_ALL_DATA_START_MEASURING;

//     context->components = machine->components;
//     context->last_data = &(machine->datas);

//     context->inner_data.dummy_start = 0xaa55;
//     context->inner_data.dummy_end = 0xdead;

//     context->bmi088_acc_ready = false;
//     context->bmi088_gyr_ready = false;  

//     context->outer_data_pub = data_pub;
//     }

// TASK_RETURN ASYNC_test_MESURE_ALL_DATA(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_test_MESURE_ALL_DATA_CONTEXT *context = (ASYNC_test_MESURE_ALL_DATA_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     uint32_t current_tick = HAL_GetTick();

//     switch (context->state) {
//     case ASYNC_test_MESURE_ALL_DATA_START_MEASURING: {

//         TASK *task;

//         task = SCHEDULER_add_task(scheduler, ASYNC_BMI088_Accelero_Publisher, true, (OBJ_POOL*)ASYNC_BMI088_Sensor_Publisher_POOL);
//         ASYNC_BMI088_Sensor_Publisher_init(task, context->components->bmi, &(context->inner_data_pub.bmi088_acc_data_pub), 10, 16);
//         task = SCHEDULER_add_task(scheduler, ASYNC_BMI088_Gyro_Publisher, true, (OBJ_POOL*)ASYNC_BMI088_Sensor_Publisher_POOL);
//         ASYNC_BMI088_Sensor_Publisher_init(task, context->components->bmi, &(context->inner_data_pub.bmi088_gyr_data_pub), 10, 16);

//         context->state = ASYNC_test_MESURE_ALL_DATA_WAIT_RECEIVE;
//         break;  }
//     case ASYNC_test_MESURE_ALL_DATA_WAIT_RECEIVE: {
//         if ((!context->bmi088_acc_ready) && context->inner_data_pub.bmi088_acc_data_pub.new_data_num > 0) {
//             DATA_PUB_pop(&(context->inner_data_pub.bmi088_acc_data_pub), &(context->inner_data.bmi088_acc));
//             context->bmi088_acc_ready = true;
//         }
//         if ((!context->bmi088_gyr_ready) && context->inner_data_pub.bmi088_gyr_data_pub.new_data_num > 0) {
//             DATA_PUB_pop(&(context->inner_data_pub.bmi088_gyr_data_pub), &(context->inner_data.bmi088_gyr));
//             context->bmi088_gyr_ready = true;
//         }
//         if (context->bmi088_acc_ready && context->bmi088_gyr_ready) {
//             context->bmi088_acc_ready = false;
//             context->bmi088_gyr_ready = false;

//             context->inner_data.longitude = context->components->gps->dec_longitude;
//             context->inner_data.latitude = context->components->gps->dec_latitude;
//             context->inner_data.altitude = context->components->gps->altitude_ft;

//             context->inner_data.timestamp = current_tick;
//             memcpy(context->last_data, &(context->inner_data), sizeof(DATA_ALL_TIMESTAMP));
//             DATA_PUB_push(context->outer_data_pub, &(context->inner_data));
            
//             __NOP();
//             // Do something with the data
//             // context->state = ASYNC_test_MESURE_ALL_DATA_WAIT_TIMER;
//         }
//         break; }
//     // case ASYNC_test_MESURE_ALL_DATA_WAIT_TIMER: {
//     //     if (context->timer_ready) {
//     //         context->timer_ready = false;
//     //         context->next_time = HAL_GetTick() + context->delay_ms;
//     //     }
//     //     if (!(context->timer_ready) && HAL_GetTick() >= context->next_time) {
//     //         context->timer_ready = true;
//     //         context->state = ASYNC_test_MESURE_ALL_DATA_WAIT_RECEIVE;
//     //     }
//     //     break; }
//     }
//     return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_test_SAVE_ALL_DATA);

// void ASYNC_test_SAVE_ALL_DATA_init(TASK *self, MACHINE *machine) {
//     ASYNC_test_SAVE_ALL_DATA_CONTEXT *context = (ASYNC_test_SAVE_ALL_DATA_CONTEXT*)self->context;

//     context->state = ASYNC_test_SAVE_ALL_DATA_GET_DATA;

//     context->components = machine->components;

//     flash_stream_init(&(context->flash_stream), context->components->flash, 0);

//     DATA_PUB_init(&(context->data_pub), (uint8_t*)context->data_pub_buffer, sizeof(DATA_ALL_TIMESTAMP), 16);
// }

// TASK_RETURN ASYNC_test_SAVE_ALL_DATA(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_test_SAVE_ALL_DATA_CONTEXT *context = (ASYNC_test_SAVE_ALL_DATA_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     // uint32_t current_tick = HAL_GetTick();

//     switch (context->state) {
//     case ASYNC_test_SAVE_ALL_DATA_GET_DATA: {
//         if (context->data_pub.new_data_num > 0) {
//             DATA_PUB_pop(&(context->data_pub), &(context->data_to_write));
//             // flash_stream_write(context->flash_stream, &(context->data_to_write), sizeof(DATA_ALL_TIMESTAMP));
//             if (context->flash_stream.write_ptr >= 0xb67) {
//                 __NOP();
//             }
//             TASK *task = SCHEDULER_add_task(scheduler, ASYNC_fs_write, false, (OBJ_POOL*)ASYNC_fs_read_write_POOL);
//             ASYNC_fs_read_write_init(task, &(context->flash_stream), (uint8_t*)(&(context->data_to_write)), sizeof(DATA_ALL_TIMESTAMP));
//             task->is_done = &(context->is_done);

//             context->state = ASYNC_test_SAVE_ALL_DATA_WAIT_WRITE;
//         }
//         break; }
//     case ASYNC_test_SAVE_ALL_DATA_WAIT_WRITE: {
//         if (context->is_done) {
//             context->state = ASYNC_test_SAVE_ALL_DATA_GET_DATA;
//         }
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }



// TASK_POOL_CREATE(ASYNC_Memory_Erase_Process);

// void ASYNC_Memory_Erase_Process_init(TASK *self, W25Q_Chip *w25q_chip) {
//     ASYNC_Memory_Erase_Process_CONTEXT *context = (ASYNC_Memory_Erase_Process_CONTEXT*)self->context;

//     context->w25q_chip = w25q_chip;

//     context->state = ASYNC_Memory_Erase_Process_WAIT_USB;
// }

// TASK_RETURN ASYNC_Memory_Erase_Process(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_Memory_Erase_Process_CONTEXT *context = (ASYNC_Memory_Erase_Process_CONTEXT*)self->context;

//     // TODO: bahhhh faut le faire quoi

//     switch (context->state) {
//     case ASYNC_SEND_ALL_DATA_USB_WAIT: {
//         if (context->is_done) {
//             context->state = context->next_state;
//         }
//         break; }
//     case ASYNC_Memory_Erase_Process_WAIT_USB: {
//         context->is_done = false;
        
//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_update_usb_watchdog, false);
//         ASYNC_update_usb_watchdog_init(task);
//         task->is_done = &(context->is_done);

//         context->state = ASYNC_Memory_Erase_Process_WAIT;
//         context->next_state = ASYNC_Memory_Erase_Process_START;
//         break; }
//     case ASYNC_Memory_Erase_Process_START: {
//         context->is_done = false;
//         TASK *task_mem, *task_buz;

//         task_buz = SCHEDULER_add_task_macro(scheduler, ASYNC_BUZZER_play_note, false);
//         ASYNC_BUZZER_play_note_init(task_buz, machine.components->buzzer,
//             buzzer_song_bank.beep_1_freqs,
//             buzzer_song_bank.beep_1_durations,
//             BUZZER_BEEP_SONG_SIZE
//         );

//         task_mem = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_EraseAll, false);
//         ASYNC_W25Q_EraseAll_init(task_mem, context->w25q_chip);
//         task_mem->is_done = &(context->is_done);
    
//         context->state = ASYNC_Memory_Erase_Process_WAIT;
//         context->next_state = ASYNC_Memory_Erase_Process_WAIT_ERASED;
//         break; }
//     case ASYNC_Memory_Erase_Process_WAIT_ERASED: {
//         context->is_done = false;

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_WaitForReady, false);
//         ASYNC_W25Q_WaitForReady_init(task, context->w25q_chip);
//         task->is_done = &(context->is_done);

//         context->state = ASYNC_Memory_Erase_Process_WAIT;
//         context->next_state = ASYNC_Memory_Erase_Process_END;
//         break; }
//     case ASYNC_Memory_Erase_Process_END: {
//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_BUZZER_play_note, false);
//         ASYNC_BUZZER_play_note_init(task, machine.components->buzzer,
//             buzzer_song_bank.beep_2_freqs,
//             buzzer_song_bank.beep_2_durations,
//             BUZZER_BEEP_SONG_SIZE
//         );

//         return TASK_RETURN_STOP;
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }






// TASK_POOL_CREATE(ASYNC_SEND_ALL_DATA_USB);

// void ASYNC_SEND_ALL_DATA_USB_init(TASK *self) {
//     ASYNC_SEND_ALL_DATA_USB_CONTEXT *context = (ASYNC_SEND_ALL_DATA_USB_CONTEXT*)self->context;
    
//     context->state = ASYNC_SEND_ALL_DATA_USB_WAIT_USB;
// }

// TASK_RETURN ASYNC_SEND_ALL_DATA_USB(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_SEND_ALL_DATA_USB_CONTEXT *context = (ASYNC_SEND_ALL_DATA_USB_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     switch (context->state) {
//     case ASYNC_SEND_ALL_DATA_USB_WAIT: {
//         if (context->is_done) {
//             context->state = context->next_state;
//         }
//         break; }
//     case ASYNC_SEND_ALL_DATA_USB_WAIT_USB: {
//         context->is_done = false;
        
//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_update_usb_watchdog, false);
//         ASYNC_update_usb_watchdog_init(task);
//         task->is_done = &(context->is_done);

//         context->state = ASYNC_SEND_ALL_DATA_USB_WAIT;
//         context->next_state = ASYNC_SEND_ALL_DATA_USB_SEND;
//         break; }
//     case ASYNC_SEND_ALL_DATA_USB_SEND: {
//         context->is_done = false;
    
//         TASK *task_buz, *task_mem;

//         task_buz = SCHEDULER_add_task_macro(scheduler, ASYNC_BUZZER_play_note, false);
//         ASYNC_BUZZER_play_note_init(task_buz, machine.components->buzzer,
//             buzzer_song_bank.beep_1_freqs,
//             buzzer_song_bank.beep_1_durations,
//             BUZZER_BEEP_SONG_SIZE
//         );

//         task_mem = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_USB_ScanMemory, false);
//         ASYNC_W25Q_USB_ScanMemory_init(task_mem, machine.flash_stream->flash_chip);
//         task_mem->is_done = &(context->is_done);

//         context->state = ASYNC_SEND_ALL_DATA_USB_WAIT;
//         context->next_state = ASYNC_SEND_ALL_DATA_USB_DONE_FLAG;
//         break; }
//     case ASYNC_SEND_ALL_DATA_USB_DONE_FLAG: {

//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_BUZZER_play_note, false);
//         ASYNC_BUZZER_play_note_init(task, machine.components->buzzer,
//             buzzer_song_bank.beep_2_freqs,
//             buzzer_song_bank.beep_2_durations,
//             BUZZER_BEEP_SONG_SIZE
//         );

//         return TASK_RETURN_STOP;
//         break; }
//     case ASYNC_SEND_ALL_DATA_USB_END: {
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_SCAN_IF_MEMORY_EMPTY);

// void ASYNC_SCAN_IF_MEMORY_EMPTY_init(TASK *self, W25Q_Chip *w25q_chip) {
//     ASYNC_SCAN_IF_MEMORY_EMPTY_CONTEXT *context = (ASYNC_SCAN_IF_MEMORY_EMPTY_CONTEXT*)self->context;

//     context->w25q_chip = w25q_chip;

//     context->state = ASYNC_SCAN_IF_MEMORY_EMPTY_START;
//     context->is_errased = false;

//     memset(context->blocks, 0, 1024);
// }

// TASK_RETURN ASYNC_SCAN_IF_MEMORY_EMPTY(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_SCAN_IF_MEMORY_EMPTY_CONTEXT *context = (ASYNC_SCAN_IF_MEMORY_EMPTY_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     switch (context->state) {
//     case ASYNC_SCAN_IF_MEMORY_EMPTY_START: {
//         context->is_done = false;

//         TASK *task_buzzer, *task_w25q;

//         task_buzzer = SCHEDULER_add_task_macro(scheduler, ASYNC_BUZZER_play_note, false);
//         ASYNC_BUZZER_play_note_init(task_buzzer, machine.components->buzzer,
//             buzzer_song_bank.beep_1_freqs,
//             buzzer_song_bank.beep_1_durations,
//             BUZZER_BEEP_SONG_SIZE
//         );

//         task_w25q = SCHEDULER_add_task_macro(scheduler, ASYNC_W25Q_ScanIfChipErased, false);
//         ASYNC_W25Q_ScanIfChipErased_init(task_w25q, context->w25q_chip, context->blocks, &(context->is_errased));
//         task_w25q->is_done = &(context->is_done);

//         context->state = ASYNC_SCAN_IF_MEMORY_EMPTY_WAIT_SCAN;
//         break; }
//     case ASYNC_SCAN_IF_MEMORY_EMPTY_WAIT_SCAN: {
//         if (context->is_done) {
//             context->state = ASYNC_SCAN_IF_MEMORY_EMPTY_DONE;
//         }
//         break; }
//     case ASYNC_SCAN_IF_MEMORY_EMPTY_DONE: {

//         break; }
//     case ASYNC_SCAN_IF_MEMORY_EMPTY_END: {
//         return TASK_RETURN_STOP;
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_telem_gps);

// void ASYNC_telem_gps_init(TASK *self, DATA_ALL_TIMESTAMP *last_data, RFM96_Chip *rfm96_chip) {
//     ASYNC_telem_gps_CONTEXT *context = (ASYNC_telem_gps_CONTEXT*)self->context;

//     context->rfm96_chip = rfm96_chip;
//     context->last_data = last_data;
//     context->is_done = true;
// }

// TASK_RETURN ASYNC_telem_gps(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_telem_gps_CONTEXT *context = (ASYNC_telem_gps_CONTEXT*)self->context;
//     UNUSED(scheduler);

//     if (context->is_done) {
//         context->is_done = false;
//         char lat[20], lon[20], alt[10];

//         float_format(lat, context->last_data->latitude, 5, 10);
//         float_format(lon, context->last_data->longitude, 5, 10);
//         float_format(alt, context->last_data->altitude, 5, 10);

//         sprintf(context->telem, "La: %s, Lo: %s, Alt: %s\n",
//                 lat, lon, alt);
        
//         TASK *task = SCHEDULER_add_task_macro(scheduler, ASYNC_RFM96_SendPacket, false);
//         ASYNC_RFM96_SendPacket_init(task, context->rfm96_chip, context->telem, strlen(context->telem) + 1);
//         task->is_done = &(context->is_done);
//     }
//     return TASK_RETURN_IDLE;
// }
