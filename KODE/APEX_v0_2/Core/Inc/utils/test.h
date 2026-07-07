#ifndef TEST_H
#define TEST_H

#include "stm32f4xx_hal.h"

#include "drivers/ADXL375.h"
#include "drivers/BMI088.h"
#include "drivers/BMP388.h"
#include "drivers/buzzer.h"
#include "drivers/gps.h"
#include "drivers/led.h"
#include "drivers/LSM303AGR.h"
#include "drivers/rfm96w.h"
#include "drivers/w25q_mem.h"

#include "utils/data_publisher.h"
#include "utils/flash_stream.h"
#include "utils/telem_format.h"


typedef struct COMPONENTS {
    ADXL375                 *adxl;
    BMI088                  *bmi;
    BMP388_HandleTypeDef    *bmp;
    BUZZER                  *buzzer;
    GPS_t                   *gps;
    LED_RGB                 *led_rgb_0;
    LED_RGB                 *led_rgb_1;
    LSM303AGR               *lsm;
    RFM96_LORA_Chip              *lora;
    W25Q_Chip               *flash;
} COMPONENTS;

typedef struct DATAS {
    // Sensors
    BMI088                  *imu;
    BMP388_HandleTypeDef    *bmp;
    GPS_t                   *gps;

    // Datas
    float accel[3];     // [m/s^2]
    float gyro[3];      // [rad/s]
    // float mag[3];       // ???
    float temp;         // [C]
    float pressure;     // [Pa]
    float longitude;    // [deg]
    float latitude;     // [deg]
    float time;         // [ms]
} DATAS;



typedef enum MACHINE_STATE {
    // ANALYSING_INFO,

    // WAIT_FOR_GPS_FIX,

    // SEND_GONOGO,
    // WAIT_RECIEVE_GONOGO,
    // RECIEVE_GONOGO,

    // BEGIN_FLASH_ERASING,
    // FLASH_ERASING,
    // END_FLASH_ERASING,

    // GETTING_DATA,

    // WAIT_FOR_SENDING_DATA, // <-- mode pour attendre et lire directement les données
    // BEGIN_SENDING_DATA,
    // SENDING_DATA,
    // END_SENDING_DATA,

    START,
    WAIT_USB,
    USB_READY,

    TEST_ASYNC,
    TEST_BMI,
    TEST_W25Q_PAGE,
    TEST_W25Q_WRITE,
    TEST_FLASH_STREAM,
    TEST_SAVE_IMU,
    TEST_FILTERED_IMU,
    TEST_LSM303AGR,
    TEST_RFM96_LORA_Tx,
    TEST_RFM96_LORA_Rx,
    TEST_GPS,
    W25Q_WAIT_ERASE,
    W25Q_writing_test,
    W25Q_WAIT,


    SCAN_IF_MEMORY_EMPTY,

    MEMORY_ERASE_PROCESS,
    MESURE_ALL_DATA,
    SEND_ALL_MEMORY,
    // IS_STOPPED,
    // STOP,

    DEFAULT_STATE = 0xff
} MACHINE_STATE;

typedef enum MACHINE_STATE_2 {
    MACHINE_STATE_2_START_TIMER,
    MACHINE_STATE_2_WAIT_TIMER,
    MACHINE_STATE_2_CHECK_TELEM,
    MACHINE_STATE_2_DO,
    MACHINE_STATE_2_WRIT_FLASH,
    MACHINE_STATE_2_DEFAULT_STATE = 0xff
} MACHINE_STATE_2;

typedef struct MACHINE {
    MACHINE_STATE  state;

    COMPONENTS* components;
    FLASH_STREAM* flash_stream;
    // DATAS* datas;
    DATA_ALL_TIMESTAMP datas;

    char           tx_buff[256];
    uint8_t        usb_watchdog_bfr[3];

    uint32_t       last_time;
    uint32_t       last_tick;

    bool is_done;
} MACHINE;

extern MACHINE machine;

void init_obj_pools(void);

void init_components(COMPONENTS             *components,
                     ADXL375                *adxl,
                     BMI088                 *bmi,
                     BMP388_HandleTypeDef   *bmp,
                     BUZZER                 *buzzer,
                     GPS_t                  *gps,
                     LED_RGB                *led_rgb_0,
                     LED_RGB                *led_rgb_1,
                     LSM303AGR              *lsm,
                     RFM96_LORA_Chip             *lora_chip,
                     W25Q_Chip              *flash_chip
);

void init_all_components(COMPONENTS *components);

void init_machine(MACHINE *machine, COMPONENTS *components,
                  FLASH_STREAM *flash_stream
                //   DATAS *datas
);

void add_task_if_flag(SCHEDULER *scheduler, MACHINE *machine);

void update_usb_watchdog(uint8_t *usb_watchdog_bfr);
bool is_connected_to_usb(uint8_t *usb_watchdog_bfr);

#define ASYNC_update_usb_watchdog_NUMBER 2

typedef struct ASYNC_update_usb_watchdog_CONTEXT {
    MACHINE_STATE* state;
    uint8_t usb_watchdog_bfr[3];
    uint32_t next_time;
} ASYNC_update_usb_watchdog_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_update_usb_watchdog);

void ASYNC_update_usb_watchdog_init(TASK *self);
TASK_RETURN ASYNC_update_usb_watchdog(SCHEDULER *scheduler, TASK *self);



#define ASYNC_test_usb_NUMBER 1

typedef struct ASYNC_test_usb_CONTEXT {
    char* buff;
    int len;
    int index;
    uint32_t delay;
    uint32_t next_time;
} ASYNC_test_usb_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_test_usb);

void ASYNC_test_usb_init(TASK *self, char *buff, int len, uint32_t delay);
TASK_RETURN ASYNC_test_usb(SCHEDULER *scheduler, TASK *self);

// =============================================

#define ASYNC_send_usb_NUMBER 1

typedef struct ASYNC_send_usb_CONTEXT {
    uint32_t delay;
    uint32_t next_time;
} ASYNC_send_usb_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_send_usb);

void ASYNC_send_usb_init(TASK *self, uint32_t delay);
TASK_RETURN ASYNC_send_usb(SCHEDULER *scheduler, TASK *self);

// =============================================

#define ASYNC_update_IMU_NUMBER 1

typedef struct ASYNC_update_IMU_CONTEXT {
    bool      continue_update;
    uint32_t  delay;
    uint32_t  next_time;
} ASYNC_update_IMU_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_update_IMU);

void ASYNC_update_IMU_init(TASK *self, uint32_t delay);
TASK_RETURN ASYNC_update_IMU(SCHEDULER *scheduler, TASK *self);

// =============================================

#define ASYNC_test_w25q_page_NUMBER 1

#define ASYNC_test_w25q_page_SIZE 24
#define ASYNC_test_w25q_page_ADDR 245
// #define ASYNC_test_w25q_page_ADDR 10

typedef enum ASYNC_test_w25q_page_STATE {
    ASYNC_test_w25q_page_START,
    ASYNC_test_w25q_page_WAIT_ERASE,
    ASYNC_test_w25q_page_WAIT_TX,
    ASYNC_test_w25q_page_WAIT_RX
} ASYNC_test_w25q_page_STATE;

typedef struct ASYNC_test_w25q_page_CONTEXT {
    W25Q_Chip* flash_chip;

    uint8_t *tx_buf;
    uint8_t *rx_buf;

    bool is_done;

    ASYNC_test_w25q_page_STATE state;
} ASYNC_test_w25q_page_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_test_w25q_page);

void ASYNC_test_w25q_page_init(TASK *self);
TASK_RETURN ASYNC_test_w25q_page(SCHEDULER *scheduler, TASK *self);

// =============================================

#define ASYNC_test_w25q_write_NUMBER 1

#define ASYNC_test_w25q_write_SIZE 0x300
#define ASYNC_test_w25q_write_ADDR 0x30
// #define ASYNC_test_w25q_write_SIZE 10 * sizeof(float)
// #define ASYNC_test_w25q_write_ADDR 3 * sizeof(float)

typedef enum ASYNC_test_w25q_write_STATE {
    ASYNC_test_w25q_write_START,
    ASYNC_test_w25q_write_WAIT_ERASE,
    ASYNC_test_w25q_write_WAIT_TX,
    ASYNC_test_w25q_write_WAIT_RX
} ASYNC_test_w25q_write_STATE;

typedef struct ASYNC_test_w25q_write_CONTEXT {
    W25Q_Chip* flash_chip;

    uint8_t *tx_buf;
    uint8_t *rx_buf;

    bool is_done;

    ASYNC_test_w25q_write_STATE state;
} ASYNC_test_w25q_write_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_test_w25q_write);

void ASYNC_test_w25q_write_init(TASK *self);
TASK_RETURN ASYNC_test_w25q_write(SCHEDULER *scheduler, TASK *self);

// =============================================

#define ASYNC_test_fs_NUMBER 1

#define ASYNC_test_fs_packet_SIZE 7
#define ASYNC_test_fs_nb_packet 3
#define ASYNC_test_fs_float_SIZE ASYNC_test_fs_packet_SIZE * ASYNC_test_fs_nb_packet
#define ASYNC_test_fs_uint8_SIZE ASYNC_test_fs_float_SIZE * sizeof(float)

typedef enum ASYNC_test_fs_STATE {
    ASYNC_test_fs_START,
    ASYNC_test_fs_WAIT_ERASE,
    ASYNC_test_fs_START_TX,
    ASYNC_test_fs_WAIT_TX,
    ASYNC_test_fs_START_RX,
    ASYNC_test_fs_WAIT_RX,
    ASYNC_test_fs_DONE_RX
} ASYNC_test_fs_STATE;

typedef struct ASYNC_test_fs_CONTEXT {
    FLASH_STREAM* flash_stream;

    uint8_t *tx_buf;
    uint8_t *rx_buf;
    
    bool is_done;

    uint8_t idx;
    ASYNC_test_fs_STATE state;
} ASYNC_test_fs_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_test_fs);

void ASYNC_test_fs_init(TASK *self, FLASH_STREAM *flash_stream);
TASK_RETURN ASYNC_test_fs(SCHEDULER *scheduler, TASK *self);

// =============================================

#define ASYNC_save_IMU_NUMBER 1

#define ASYNC_save_IMU_DATA_NUMBER 7

typedef enum ASYNC_save_IMU_STATE {
    ASYNC_save_IMU_START_ERASE,
    ASYNC_save_IMU_START_WAITING,
    ASYNC_save_IMU_WAIT_READY,
    ASYNC_save_IMU_START_UPDATE,
    ASYNC_save_IMU_WAIT_UPDATE,
    ASYNC_save_IMU_WAIT_SAVE,
    ASYNC_save_IMU_END_UPDATE,
    ASYNC_save_IMU_START_LOAD,
    ASYNC_save_IMU_WAIT_LOAD,
    ASYNC_save_IMU_START_SEND,
    ASYNC_save_IMU_WAIT_SEND,
    ASYNC_save_IMU_END_SEND,
} ASYNC_save_IMU_STATE;

typedef struct ASYNC_save_IMU_CONTEXT {
    TASK *task_imu;
    float datas_buff[7];

    bool ready;
    bool save_done;

    ASYNC_save_IMU_STATE state;
    uint32_t max_delay;
    uint32_t imu_delay;
    uint32_t first_time;
    uint32_t next_time;
} ASYNC_save_IMU_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_save_IMU);

void ASYNC_save_IMU_init(TASK *self);
TASK_RETURN ASYNC_save_IMU(SCHEDULER* scheduler, TASK *self);

// ============================================

#define ASYNC_filtred_IMU_NUMBER 1

#define ASYNC_filtred_IMU_DATA_NUMBER 7

typedef enum ASYNC_filtred_IMU_STATE {
    ASYNC_filtred_IMU_START,
    ASYNC_filtred_IMU_START_WAITING_W25Q,
    ASYNC_filtred_IMU_WAIT_OFFSET_BMI_AND_W25Q,
    ASYNC_filtred_IMU_OFFSET_CALCULUS,
    ASYNC_filtred_IMU_WAIT_UPDATE,
    ASYNC_filtred_IMU_END_UPDATE,
} ASYNC_filtred_IMU_STATE;

typedef struct ASYNC_filtred_IMU_CONTEXT {
    TASK *task_imu;
    float datas_buff[7];

    float acc_offset[3][100];
    float gyr_offset[3][100];

    size_t acc_offset_num;
    size_t gyr_offset_num;

    float acc_offset_mean[3];
    float gyr_offset_mean[3];

    DATA_PUB acc_data;
    DATA_PUB gyr_data;

    bool w25q_ready;

    ASYNC_filtred_IMU_STATE state;
    uint32_t max_delay;
    uint32_t imu_delay;
    uint32_t first_time;
    uint32_t next_time;
} ASYNC_filtred_IMU_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_filtred_IMU);

void ASYNC_filtred_IMU_init(TASK *self);
TASK_RETURN ASYNC_filtred_IMU(SCHEDULER* scheduler, TASK *self);



// ============================================

#define ASYNC_test_LSM303AGR_NUMBER 1

typedef enum ASYNC_test_LSM303AGR_STATE {
    ASYNC_test_LSM303AGR_WAIT_READY,
    ASYNC_test_LSM303AGR_READ_ACCELEROMETER,
    ASYNC_test_LSM303AGR_READ_MAGNETOMETER,
    ASYNC_test_LSM303AGR_PRINT,
    ASYNC_test_LSM303AGR_DELAY
} ASYNC_test_LSM303AGR_STATE;


typedef struct ASYNC_test_LSM303AGR_CONTEXT {
    LSM303AGR *lsm;

    FLOAT3 acc;
    FLOAT3 mag;

    bool is_ready;

    ASYNC_test_LSM303AGR_STATE state;
    ASYNC_test_LSM303AGR_STATE next_state;

    uint32_t delay_us;
} ASYNC_test_LSM303AGR_CONTEXT;


extern TASK_POOL_CREATE(ASYNC_test_LSM303AGR);

void ASYNC_test_LSM303AGR_init(TASK *self, LSM303AGR *lsm);
TASK_RETURN ASYNC_test_LSM303AGR(SCHEDULER *scheduler, TASK *self);


// ============================================


#define ASYNC_test_RFM96_LORA_Tx_NUMBER 1

typedef enum ASYNC_test_RFM96_LORA_Tx_STATE {
    ASYNC_test_RFM96_LORA_Tx_WAIT_READY,
    ASYNC_test_RFM96_LORA_Tx_SEND_PACKET,
    ASYNC_test_RFM96_LORA_Tx_DELAY
} ASYNC_test_RFM96_LORA_Tx_STATE;

typedef struct ASYNC_test_RFM96_LORA_Tx_CONTEXT {
    RFM96_LORA_Chip *RFM96_LORA_chip;

    uint8_t tx_buf[256];
    uint16_t tx_size;

    bool is_ready;

    ASYNC_test_RFM96_LORA_Tx_STATE state;
    ASYNC_test_RFM96_LORA_Tx_STATE next_state;

    uint32_t delay_ms;
} ASYNC_test_RFM96_LORA_Tx_CONTEXT;


extern TASK_POOL_CREATE(ASYNC_test_RFM96_LORA_Tx);

void ASYNC_test_RFM96_LORA_Tx_init(TASK *self, RFM96_LORA_Chip *RFM96_LORA_chip);
TASK_RETURN ASYNC_test_RFM96_LORA_Tx(SCHEDULER *scheduler, TASK *self);


// ============================================


#define ASYNC_test_RFM96_LORA_Rx_NUMBER 1

typedef enum ASYNC_test_RFM96_LORA_Rx_STATE {
    ASYNC_test_RFM96_LORA_Rx_WAIT_READY,
    ASYNC_test_RFM96_LORA_Rx_SET_RX_MODE,
    ASYNC_test_RFM96_LORA_Rx_RECEIVE_PACKET,
    ASYNC_test_RFM96_LORA_Rx_RECEIVE_PACKET_DONE,
} ASYNC_test_RFM96_LORA_Rx_STATE;

typedef struct ASYNC_test_RFM96_LORA_Rx_CONTEXT {
    RFM96_LORA_Chip *RFM96_LORA_chip;

    uint8_t rx_buf[256];
    uint8_t rx_size;

    bool is_ready;

    ASYNC_test_RFM96_LORA_Rx_STATE state;
    ASYNC_test_RFM96_LORA_Rx_STATE next_state;

    uint32_t delay_ms;
} ASYNC_test_RFM96_LORA_Rx_CONTEXT;


extern TASK_POOL_CREATE(ASYNC_test_RFM96_LORA_Rx);

void ASYNC_test_RFM96_LORA_Rx_init(TASK *self, RFM96_LORA_Chip *RFM96_LORA_chip);
TASK_RETURN ASYNC_test_RFM96_LORA_Rx(SCHEDULER *scheduler, TASK *self);


// ============================================


#define ASYNC_test_GPS_NUMBER 5

typedef enum ASYNC_test_GPS_STATE {
    ASYNC_test_GPS_WAIT_READY,
    ASYNC_test_GPS_READ,
    ASYNC_test_GPS_PRINT,
    ASYNC_test_GPS_DELAY
} ASYNC_test_GPS_STATE;

typedef struct ASYNC_test_GPS_CONTEXT {
    GPS_t *gps;

    bool is_ready;

    ASYNC_test_GPS_STATE state;
    ASYNC_test_GPS_STATE next_state;

    uint32_t delay_ms;
} ASYNC_test_GPS_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_test_GPS);

void ASYNC_test_GPS_init(TASK *self, GPS_t *gps);
TASK_RETURN ASYNC_test_GPS(SCHEDULER *scheduler, TASK *self);


// ============================================

#define ASYNC_test_MESURE_ALL_DATA_NUMBER 1

typedef enum ASYNC_test_MESURE_ALL_DATA_STATE {
    ASYNC_test_MESURE_ALL_DATA_START_MEASURING,
    ASYNC_test_MESURE_ALL_DATA_WAIT_RECEIVE,
    // ASYNC_test_MESURE_ALL_DATA_WAIT_TIMER,
    // ASYNC_test_MESURE_ALL_DATA_
} ASYNC_test_MESURE_ALL_DATA_STATE;

typedef struct ASYNC_test_MESURE_ALL_DATA_CONTEXT {
    COMPONENTS *components;

    TASK *bmi088_acc_task;
    TASK *bmi088_gyr_task;

    bool bmi088_acc_ready;
    bool bmi088_gyr_ready;

    DATA_ALL_PUB inner_data_pub;
    DATA_ALL_TIMESTAMP inner_data;
    DATA_ALL_TIMESTAMP *last_data;
    // DATA_ALL_TIMESTAMP *outer_data;

    // Data publisher buffer for outer data
    // This buffer is used to store the data that will be sent to the outer world
    // 16 data can be stored in this buffer
    DATA_ALL_TIMESTAMP outer_data[16];
    DATA_PUB *outer_data_pub;

    bool timer_ready;

    ASYNC_test_MESURE_ALL_DATA_STATE state;
    ASYNC_test_MESURE_ALL_DATA_STATE next_state;

    uint32_t delay_ms;
    uint32_t next_time;
} ASYNC_test_MESURE_ALL_DATA_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_test_MESURE_ALL_DATA);

void ASYNC_test_MESURE_ALL_DATA_init(TASK *self, MACHINE *machine, DATA_PUB *data_pub);
TASK_RETURN ASYNC_test_MESURE_ALL_DATA(SCHEDULER *scheduler, TASK *self);

#define ASYNC_test_SAVE_ALL_DATA_NUMBER 1

typedef enum ASYNC_test_SAVE_ALL_DATA_STATE {
    ASYNC_test_SAVE_ALL_DATA_GET_DATA,
    // ASYNC_test_SAVE_ALL_DATA_WRITE,
    ASYNC_test_SAVE_ALL_DATA_WAIT_WRITE,
    // ASYNC_test_SAVE_ALL_DATA_WAIT_TIMER,
} ASYNC_test_SAVE_ALL_DATA_STATE;

typedef struct ASYNC_test_SAVE_ALL_DATA_CONTEXT {
    COMPONENTS *components;

    FLASH_STREAM flash_stream;

    DATA_ALL_TIMESTAMP data_pub_buffer[16];
    DATA_PUB data_pub;
    DATA_ALL_TIMESTAMP data_to_write;

    size_t data_num;

    bool timer_ready;
    uint32_t delay_ms;
    uint32_t next_time;

    bool is_done;

    ASYNC_test_SAVE_ALL_DATA_STATE state;
} ASYNC_test_SAVE_ALL_DATA_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_test_SAVE_ALL_DATA);

void ASYNC_test_SAVE_ALL_DATA_init(TASK *self, MACHINE *machine);
TASK_RETURN ASYNC_test_SAVE_ALL_DATA(SCHEDULER *scheduler, TASK *self);


#define ASYNC_SEND_ALL_DATA_USB_NUMBER 1

typedef enum ASYNC_SEND_ALL_DATA_USB_STATE {
    ASYNC_SEND_ALL_DATA_USB_WAIT,
    ASYNC_SEND_ALL_DATA_USB_WAIT_USB,
    ASYNC_SEND_ALL_DATA_USB_SEND,
    ASYNC_SEND_ALL_DATA_USB_DONE_FLAG,
    ASYNC_SEND_ALL_DATA_USB_END,
} ASYNC_SEND_ALL_DATA_USB_STATE;

typedef struct ASYNC_SEND_ALL_DATA_USB_CONTEXT {
    bool is_done;
    uint32_t next_time;
    ASYNC_SEND_ALL_DATA_USB_STATE state;
    ASYNC_SEND_ALL_DATA_USB_STATE next_state;
} ASYNC_SEND_ALL_DATA_USB_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_SEND_ALL_DATA_USB);

void ASYNC_SEND_ALL_DATA_USB_init(TASK *self);
TASK_RETURN ASYNC_SEND_ALL_DATA_USB(SCHEDULER *scheduler, TASK *self);


#define ASYNC_SCAN_IF_MEMORY_EMPTY_NUMBER 1

typedef enum ASYNC_SCAN_IF_MEMORY_EMPTY_STATE {
    ASYNC_SCAN_IF_MEMORY_EMPTY_START,
    ASYNC_SCAN_IF_MEMORY_EMPTY_WAIT_SCAN,
    ASYNC_SCAN_IF_MEMORY_EMPTY_DONE,
    ASYNC_SCAN_IF_MEMORY_EMPTY_END
} ASYNC_SCAN_IF_MEMORY_EMPTY_STATE;

typedef struct ASYNC_SCAN_IF_MEMORY_EMPTY_CONTEXT {
    W25Q_Chip *w25q_chip;

    bool is_done;

    bool blocks[1024];
    bool is_errased;


    ASYNC_SCAN_IF_MEMORY_EMPTY_STATE state;
    uint32_t next_time;
} ASYNC_SCAN_IF_MEMORY_EMPTY_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_SCAN_IF_MEMORY_EMPTY);

void ASYNC_SCAN_IF_MEMORY_EMPTY_init(TASK *self, W25Q_Chip *w25q_chip);
TASK_RETURN ASYNC_SCAN_IF_MEMORY_EMPTY(SCHEDULER *scheduler, TASK *self);


#define ASYNC_Memory_Erase_Process_NUMBER 1

typedef enum ASYNC_Memory_Erase_Process_State {
    ASYNC_Memory_Erase_Process_WAIT,
    ASYNC_Memory_Erase_Process_WAIT_USB,
    ASYNC_Memory_Erase_Process_START,
    ASYNC_Memory_Erase_Process_WAIT_ERASED,
    ASYNC_Memory_Erase_Process_END
} ASYNC_Memory_Erase_Process_State;

typedef struct ASYNC_Memory_Erase_Process_CONTEXT {
    W25Q_Chip *w25q_chip;

    bool is_done;

    ASYNC_Memory_Erase_Process_State state;
    ASYNC_Memory_Erase_Process_State next_state;
} ASYNC_Memory_Erase_Process_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_Memory_Erase_Process);

void ASYNC_Memory_Erase_Process_init(TASK *self, W25Q_Chip *w25q_chip);
TASK_RETURN ASYNC_Memory_Erase_Process(SCHEDULER *scheduler, TASK *self);


#define ASYNC_telem_gps_NUMBER 1

typedef struct ASYNC_telem_gps_CONTEXT {
    bool is_done;
    RFM96_LORA_Chip *RFM96_LORA_chip;
    uint8_t telem[256];
    DATA_ALL_TIMESTAMP *last_data;
} ASYNC_telem_gps_CONTEXT;

extern TASK_POOL_CREATE(ASYNC_telem_gps);

void ASYNC_telem_gps_init(TASK *self, DATA_ALL_TIMESTAMP *last_data, RFM96_LORA_Chip *RFM96_LORA_chip);
TASK_RETURN ASYNC_telem_gps(SCHEDULER *scheduler, TASK *self);

// ============================================


#endif // TEST_H