# ifndef SX127x_H
# define SX127x_H

#include "sx127x/sx127x_common.h"
#include "sx127x/sx127x_lora.h"
#include "sx127x/sx127x_fsk_ook.h"

#include "utils/scheduler.h"

typedef struct sx127x_chip_t {
    sx127x_base_chip_t base_chip;
    union mod_chip {
        sx127x_LORA_chip_t lora;
        sx127x_FSK_OOK_chip_t fsk_ook;
    } mod_chip;
} sx127x_chip_t;

typedef union sx127x_mod_config_t {
    sx127x_LORA_config_t lora;
    sx127x_FSK_OOK_config_t fsk_ook;
} sx127x_mod_config_t;

sx127x_status_t sx127x_Init(sx127x_chip_t *chip, sx127x_base_config_t base_config, sx127x_modulation_t modulation, sx127x_mod_config_t config);

sx127x_status_t sx127x_TxSend(sx127x_chip_t *chip, const uint8_t *data, uint16_t len);
sx127x_status_t sx127x_RxReceive(sx127x_chip_t *chip, uint8_t *data, uint16_t *len);




/* ============================== FreeRTOS ============================== */

sx127x_status_t sx127x_Init_RTOS(sx127x_chip_t *chip, sx127x_base_config_t base_config, sx127x_modulation_t modulation, sx127x_mod_config_t config);

sx127x_status_t sx127x_TxSend_RTOS_base(sx127x_chip_t *chip, const uint8_t *data, uint16_t len, bool lock_sem);
sx127x_status_t sx127x_RxReceive_RTOS_base(sx127x_chip_t *chip, uint8_t *data, uint16_t *len, bool lock_sem);

typedef struct TASK_sx127x_Init_ARGS{
    sx127x_chip_t               *chip;
    const sx127x_base_config_t  *base_config;
    sx127x_modulation_t          modulation;
    const sx127x_mod_config_t   *config;
    sx127x_status_t             *return_state;
	osEventFlagsId_t             done_flags;
} TASK_sx127x_Init_ARGS;
TASK_POOL_CONFIGURE(TASK_sx127x_Init, 1, 1024);
void TASK_sx127x_Init(void *argument);

typedef struct TASK_sx127x_TxSend_ARGS{
    sx127x_chip_t       *chip;
    const uint8_t       *data;
    uint16_t             len;
    sx127x_status_t     *return_state;
    osEventFlagsId_t     done_flags;
} TASK_sx127x_TxSend_ARGS;
TASK_POOL_CONFIGURE(TASK_sx127x_TxSend, 1, 1024);
void TASK_sx127x_TxSend(void *argument);

typedef struct TASK_sx127x_RxReceive_ARGS{
    sx127x_chip_t       *chip;
    uint8_t             *data;
    uint16_t            *len;
    sx127x_status_t     *return_state;
    osEventFlagsId_t     done_flags;
} TASK_sx127x_RxReceive_ARGS;
TASK_POOL_CONFIGURE(TASK_sx127x_RxReceive, 1, 2048);
void TASK_sx127x_RxReceive(void *argument);




// ================================== task test ==================================

#define IS_SYNC true
// #define IS_LORA true
typedef struct TASK_sx127x_Init_TxRx_ARGS {
    sx127x_chip_t *chip;
} TASK_sx127x_Init_TxRx_ARGS;
TASK_POOL_CONFIGURE(TASK_sx127x_Init_TxRx, 1, 2048);
void TASK_sx127x_Init_TxRx(void *argument);




# endif // SX127x_H