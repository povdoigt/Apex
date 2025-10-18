#ifndef LSM303AGR_H
#define LSM303AGR_H

#include "stm32f4xx_hal.h"
#include "peripherals/i2c.h"

#include "utils/data_publisher.h"
#include "utils/scheduler.h"
#include "utils/tools.h"


// ============== LSM303AGR REGISTERS ADDRESSES ==============

// Auxiliary status register for accelerometer (to check temperature data ready)
#define LSM303AGR_REG_STATUS_REG_AUX_A 0x07

// Temperature output registers for accelerometer
#define LSM303AGR_REG_OUT_TEMP_L_A 0x0C
#define LSM303AGR_REG_OUT_TEMP_H_A 0x0D

// Interrupt count register for accelerometer
#define LSM303AGR_REG_INT_COUNTER_REG_A 0x0E

// Accelerometer device identification
#define LSM303AGR_REG_WHO_AM_I_A 0x0F

// Temperature configuration register
#define LSM303AGR_REG_TEMP_CFG_REG_A 0x1F

// Accelerometer control registers
#define LSM303AGR_REG_CTRL_REG1_A 0x20
#define LSM303AGR_REG_CTRL_REG2_A 0x21
#define LSM303AGR_REG_CTRL_REG3_A 0x22
#define LSM303AGR_REG_CTRL_REG4_A 0x23
#define LSM303AGR_REG_CTRL_REG5_A 0x24
#define LSM303AGR_REG_CTRL_REG6_A 0x25

// Accelerometer status register
#define LSM303AGR_REG_STATUS_REG_A 0x27

// Accelerometer output registers
#define LSM303AGR_REG_OUT_X_L_A 0x28
#define LSM303AGR_REG_OUT_X_H_A 0x29
#define LSM303AGR_REG_OUT_Y_L_A 0x2A
#define LSM303AGR_REG_OUT_Y_H_A 0x2B
#define LSM303AGR_REG_OUT_Z_L_A 0x2C
#define LSM303AGR_REG_OUT_Z_H_A 0x2D

// FIFO registers
#define LSM303AGR_REG_FIFO_CTRL_REG_A 0x2E
#define LSM303AGR_REG_FIFO_SRC_REG_A 0x2F

// Magnetometer hard-iron registers
#define LSM303AGR_REG_OFFSET_X_L_M 0x05
#define LSM303AGR_REG_OFFSET_X_H_M 0x06
#define LSM303AGR_REG_OFFSET_Y_L_M 0x07
#define LSM303AGR_REG_OFFSET_Y_H_M 0x08
#define LSM303AGR_REG_OFFSET_Z_L_M 0x09
#define LSM303AGR_REG_OFFSET_Z_H_M 0x0A

// Magnetometer device identification
#define LSM303AGR_WHO_AM_I_M 0x4F

// Magnetometer configuration registers
#define LSM303AGR_REG_CFG_REG_A_M 0x60
#define LSM303AGR_REG_CFG_REG_B_M 0x61
#define LSM303AGR_REG_CFG_REG_C_M 0x62

// Magnetometer status register
#define LSM303AGR_REG_STATUS_REG_M 0x67

// Magnetometer output registers
#define LSM303AGR_REG_OUTX_L_M 0x68
#define LSM303AGR_REG_OUTX_H_M 0x69
#define LSM303AGR_REG_OUTY_L_M 0x6A
#define LSM303AGR_REG_OUTY_H_M 0x6B
#define LSM303AGR_REG_OUTZ_L_M 0x6C
#define LSM303AGR_REG_OUTZ_H_M 0x6D

// ==============================================================


#define LSM303AGR_SAD_A 0x19 // I2C address for accelerometer
#define LSM303AGR_SAD_M 0x1E // I2C address for magnetometer

#define LSM303AGR_DEVICE_ID_A 0x33 // Device ID for accelerometer
#define LSM303AGR_DEVICE_ID_M 0x40 // Device ID for magnetometer


#define LSM303AGR_AUTO_INCREMENT 0x80 // Auto-increment bit for I2C addresses


// ========= LSM303AGR Linearly Acceleration Sensiivity ============

#define LSM303AGR_LIN_ACC_SO_2G_HR    0.98 // mg/LSB
#define LSM303AGR_LIN_ACC_SO_4G_HR    1.95 // mg/LSB
#define LSM303AGR_LIN_ACC_SO_8G_HR    3.90 // mg/LSB
#define LSM303AGR_LIN_ACC_SO_16G_HR   7.81 // mg/LSB

#define LSM303AGR_LIN_ACC_SO_2G_N     3.90 // mg/LSB
#define LSM303AGR_LIN_ACC_SO_4G_N     7.82 // mg/LSB
#define LSM303AGR_LIN_ACC_SO_8G_N    15.63 // mg/LSB
#define LSM303AGR_LIN_ACC_SO_16G_N   46.90 // mg/LSB

#define LSM303AGR_LIN_ACC_SO_2G_LP   15.63 // mg/LSB
#define LSM303AGR_LIN_ACC_SO_4G_LP   31.26 // mg/LSB
#define LSM303AGR_LIN_ACC_SO_8G_LP   62.52 // mg/LSB
#define LSM303AGR_LIN_ACC_SO_16G_LP 187.58 // mg/LSB


// =========== LSM303AGR Linearly Magnetometer Sensiivity ============

#define LSM303AGR_LIN_MAG_SO  1.5 // mGauss/LSB


// =========== LSM303AGR STATUS_REG_AUX_A mask ============

#define LSM303AGR_STATUS_REG_AUX_A_TOR_MASK 0x40 // Temperature data overrun
#define LSM303AGR_STATUS_REG_AUX_A_TDA_MASK 0x04 // Temperature data available


// =========== LSM303AGR TEMP_CFG_REG_A mask ============

#define LSM303AGR_TEMP_CFG_REG_A_TEMP_DIS_MASK 0x00 // Temperature sensor disabled
#define LSM303AGR_TEMP_CFG_REG_A_TEMP_EN_MASK 0xC0 // Temperature sensor enabled


// =========== LSM303AGR CTRL_REG1_A mask ============

#define LSM303AGR_CTRL_REG1_A_ODR_PWD_MASK      0x00 // Power-down mode
#define LSM303AGR_CTRL_REG1_A_ODR_1HZ_MASK      0x10 // Output data rate (HR/N/LP): 1Hz
#define LSM303AGR_CTRL_REG1_A_ODR_10HZ_MASK     0x20 // Output data rate (HR/N/LP): 10Hz
#define LSM303AGR_CTRL_REG1_A_ODR_25HZ_MASK     0x30 // Output data rate (HR/N/LP): 25Hz
#define LSM303AGR_CTRL_REG1_A_ODR_50HZ_MASK     0x40 // Output data rate (HR/N/LP): 50Hz
#define LSM303AGR_CTRL_REG1_A_ODR_100HZ_MASK    0x50 // Output data rate (HR/N/LP): 100Hz
#define LSM303AGR_CTRL_REG1_A_ODR_200HZ_MASK    0x60 // Output data rate (HR/N/LP): 200Hz
#define LSM303AGR_CTRL_REG1_A_ODR_400HZ_MASK    0x70 // Output data rate (HR/N/LP): 400Hz
#define LSM303AGR_CTRL_REG1_A_ODR_1620HZ_MASK   0x80 // Output data rate (LP)     : 1620Hz
#define LSM303AGR_CTRL_REG1_A_ODR_MAXHZ_MASK    0x90 // Output data rate (HR/N) : 1344Hz, (LP) : 5376Hz
#define LSM303AGR_CTRL_REG1_A_LPEN_MASK         0x08 // Low power mode: enabled
#define LSM303AGR_CTRL_REG1_A_ZEN_MASK          0x04 // Z-axis enabled
#define LSM303AGR_CTRL_REG1_A_YEN_MASK          0x02 // Y-axis enabled
#define LSM303AGR_CTRL_REG1_A_XEN_MASK          0x01 // X-axis enabled


// =========== LSM303AGR CTRL_REG2_A mask ============

#define LSM303AGR_CTRL_REG2_A_HPM_NORMAL_CAP_MASK  0x00 // High-pass filter mode: normal mode, reset by capture
#define LSM303AGR_CTRL_REG2_A_HPM_REF_SIGNAL_MASK  0x10 // High-pass filter mode: reference signal for high-pass filter
#define LSM303AGR_CTRL_REG2_A_HPM_NORMAL_MASK      0x20 // High-pass filter mode: normal mode
#define LSM303AGR_CTRL_REG2_A_HPM_AUTORESET_MASK   0x30 // High-pass filter mode: auto-reset by new data
#define LSM303AGR_CTRL_REG2_A_FDS_MASK             0x08 // Filtered data selection: 0 = bypass filter, 1 = high-pass filter
// HPFC - High-Pass Filter Cutoff, no details in datasheet...
#define LSM303AGR_CTRL_REG2_A_HPFCLICK_MASK        0x04 // High-pass filter enabled for click function
#define LSM303AGR_CTRL_REG2_A_HPISO2_MASK          0x02 // High-pass filter enabled for AOI function on interrupt 2
#define LSM303AGR_CTRL_REG2_A_HPISO1_MASK          0x01 // High-pass filter enabled for AOI function on interrupt 1


// =========== LSM303AGR CTRL_REG3_A mask ============

#define LSM303AGR_CTRL_REG3_A_I1_CLICK_MASK      0x80 // Interrupt 1: click function enabled
#define LSM303AGR_CTRL_REG3_A_I1_AOI1_MASK       0x40 // Interrupt 1: AOI1 function enabled
#define LSM303AGR_CTRL_REG3_A_I1_AOI2_MASK       0x20 // Interrupt 1: AOI2 function enabled
#define LSM303AGR_CTRL_REG3_A_I1_DRDY1_MASK      0x10 // Interrupt 1: DRDY1 function enabled
#define LSM303AGR_CTRL_REG3_A_I1_DRDY2_MASK      0x08 // Interrupt 1: DRDY2 function enabled
#define LSM303AGR_CTRL_REG3_A_I1_WTM_MASK        0x04 // Interrupt 1: FIFO watermark function enabled
#define LSM303AGR_CTRL_REG3_A_I1_OVERRUN_MASK    0x02 // Interrupt 1: FIFO overrun function enabled


// =========== LSM303AGR CTRL_REG4_A mask ============

#define LSM303AGR_CTRL_REG4_A_BDU_MASK          0x80 // Block data update: non-blocking mode
#define LSM303AGR_CTRL_REG4_A_BLE_MASK          0x40 // Big/Little Endian: 0 = LSB first, 1 = MSB first
#define LSM303AGR_CTRL_REG4_A_FS_2G_MASK        0x00 // +/-2g
#define LSM303AGR_CTRL_REG4_A_FS_4G_MASK        0x10 // +/-4g
#define LSM303AGR_CTRL_REG4_A_FS_8G_MASK        0x20 // +/-8g
#define LSM303AGR_CTRL_REG4_A_FS_16G_MASK       0x30 // +/-16g
#define LSM303AGR_CTRL_REG4_A_HR_EN_MASK        0x08 // High resolution: enabled
#define LSM303AGR_CTRL_REG4_A_ST_NORMAL_MASK    0x00 // Self-test: normal mode
#define LSM303AGR_CTRL_REG4_A_ST_0_MASK         0x02 // Self-test: self-test 0
#define LSM303AGR_CTRL_REG4_A_ST_1_MASK         0x04 // Self-test: self-test 1
#define LSM303AGR_CTRL_REG4_A_SPI_EN_MASK       0x01 // SPI 3-wire interface enabled


// =========== LSM303AGR CTRL_REG5_A mask ============

// TODO


// =========== LSM303AGR CTRL_REG6_A mask ============

// TODO


// =========== LSM303AGR STATUS_REG_A mask ============

#define LSM303AGR_STATUS_REG_A_ZYXOR_MASK 0x80 // ZYXOR: X, Y, Z axes data overrun
#define LSM303AGR_STATUS_REG_A_ZOR_MASK   0x40 // ZOR: Z axis data overrun
#define LSM303AGR_STATUS_REG_A_YOR_MASK   0x20 // YOR: Y axis data overrun
#define LSM303AGR_STATUS_REG_A_XOR_MASK   0x10 // XOR: X axis data overrun
#define LSM303AGR_STATUS_REG_A_ZYXDA_MASK 0x08 // ZYXDA: X, Y, Z axes data available
#define LSM303AGR_STATUS_REG_A_ZDA_MASK   0x04 // ZDA: Z axis data available
#define LSM303AGR_STATUS_REG_A_YDA_MASK   0x02 // YDA: Y axis data available
#define LSM303AGR_STATUS_REG_A_XDA_MASK   0x01 // XDA: X axis data available

// =========== LSM303AGR FIFO_CTRL_REG_A mask ============

// TODO


// =========== LSM303AGR FIFO_SRC_REG_A mask ============

// TODO


// =========== LSM303AGR CFG_REG_A_M mask ============

#define LSM303AGR_REG_CFG_REG_A_M_COMP_TEMP_MASK        0x80 // Temperature compensation enabled
#define LSM303AGR_REG_CFG_REG_A_M_REBOOT_MASK           0x40 // Reboot memory content
#define LSM303AGR_REG_CFG_REG_A_M_SOFT_RST_MASK         0x20 // Soft reset
#define LSM303AGR_REG_CFG_REG_A_M_LP_MASK               0x10 // Low power mode: enabled
#define LSM303AGR_REG_CFG_REG_A_M_ODR_10HZ_MASK         0x00 // Output data rate: 10Hz
#define LSM303AGR_REG_CFG_REG_A_M_ODR_20HZ_MASK         0x04 // Output data rate: 20Hz
#define LSM303AGR_REG_CFG_REG_A_M_ODR_50HZ_MASK         0x08 // Output data rate: 50Hz
#define LSM303AGR_REG_CFG_REG_A_M_ODR_100HZ_MASK        0x0C // Output data rate: 100Hz
#define LSM303AGR_REG_CFG_REG_A_M_MD_CONTINUOUS_MASK    0x00 // Mode: continuous
#define LSM303AGR_REG_CFG_REG_A_M_MD_SINGLE_MASK        0x01 // Mode: single
#define LSM303AGR_REG_CFG_REG_A_M_MD_IDLE0_MASK         0x02 // Mode: idle
#define LSM303AGR_REG_CFG_REG_A_M_MD_IDLE1_MASK         0x03 // Mode: idle


// =========== LSM303AGR CFG_REG_B_M mask ============

#define LSM303AGR_REG_CFG_REG_B_M_OFF_CANC_ONE_SHOT_MASK    0x10 // Offset cancellation on single measurement
#define LSM303AGR_REG_CFG_REG_B_M_INT_ON_DATA_OFF_MASK      0x08 // Interrupt on data recognition
#define LSM303AGR_REG_CFG_REG_B_M_SET_FREQ_MASK             0x04 // Set frequency of the set pulse
#define LSM303AGR_REG_CFG_REG_B_M_OFF_CANC_MASK             0x02 // Offset cancellation enabled
#define LSM303AGR_REG_CFG_REG_B_M_LPF_MASK                  0x01 // Low-pass filter enabled


// =========== LSM303AGR CFG_REG_C_M mask ============

#define LSM303AGR_REG_CFG_REG_C_M_INT_MAG_PIN_MASK  0x40 // Interrupt pin enabled
#define LSM303AGR_REG_CFG_REG_C_M_I2C_DIS_MASK      0x20 // I2C interface disabled
#define LSM303AGR_REG_CFG_REG_C_M_BDU_MASK          0x10 // Block data update: enabled
#define LSM303AGR_REG_CFG_REG_C_M_BLE_MASK          0x08 // Big/Little Endian: 0 = LSB first, 1 = MSB first
#define LSM303AGR_REG_CFG_REG_C_M_SELF_TEST_MASK    0x02 // Self-test: enabled
#define LSM303AGR_REG_CFG_REG_C_M_INT_MAG_MASK      0x01 // DRDY pin is configured as digital output


// =========== LSM303AGR STATUS_REG_M mask ============

#define LSM303AGR_REG_STATUS_REG_M_ZYXOR_MASK 0x80 // ZYXOR: X, Y, Z axes data overrun
#define LSM303AGR_REG_STATUS_REG_M_ZOR_MASK   0x40 // ZOR: Z axis data overrun
#define LSM303AGR_REG_STATUS_REG_M_YOR_MASK   0x20 // YOR: Y axis data overrun
#define LSM303AGR_REG_STATUS_REG_M_XOR_MASK   0x10 // XOR: X axis data overrun
#define LSM303AGR_REG_STATUS_REG_M_ZYXDA_MASK 0x08 // ZYXDA: X, Y, Z axes data available
#define LSM303AGR_REG_STATUS_REG_M_ZDA_MASK   0x04 // ZDA: Z axis data available
#define LSM303AGR_REG_STATUS_REG_M_YDA_MASK   0x02 // YDA: Y axis data available
#define LSM303AGR_REG_STATUS_REG_M_XDA_MASK   0x01 // XDA: X axis data available


// ================================================

typedef struct LSM303AGR {
    I2C_HandleTypeDef *hi2c; // I2C handle

    // float acc_data[3]; // Accelerometer data
    // float mag_data[3]; // Magnetometer data

    float acc_conv; // Accelerometer conversion factor
    float mag_conv; // Magnetometer conversion factor

    bool busy; // Flag to indicate if the sensor is busy
    // TODO: Need to include the busy flag in all methods that read data from the sensor

} LSM303AGR;


void LSM303AGR_Init(LSM303AGR *lsm, I2C_HandleTypeDef *hi2c);

void LSM303AGR_ReadAcc(LSM303AGR *lsm, float *acc_x, float *acc_y, float *acc_z);
void LSM303AGR_ReadMag(LSM303AGR *lsm, float *mag_x, float *mag_y, float *mag_z);





/*
	Macro that call the ASYNC_I2C_Mem_Tx_or_Rx_DMA_init function with the LSM303AGR
    accelerometer parameters.
	It requires:
		- SCHEUDLER struct named "scheduler"
		- TASK struct named "task"
		- ASYNC_LSM303AGR_..._CONTEXT struct named "context" witch is the context of the task
	
	... to be defined in the current scope.

	Parameters:
        - add: the memory address to read or write
        - buf: the buffer to read or write
        - size: the size of the buffer in bytes
*/
// #define ASYNC_I2C_Mem_Tx_or_Rx_Acc_LSM303AGR(add, buf, size) \
//     ASYNC_I2C_Mem_Tx_or_Rx_DMA_init( \
//         task, \
//         context->lsm->hi2c, \
//         LSM303AGR_SAD_A << 1, \
//         add, \
//         I2C_MEMADD_SIZE_8BIT, \
//         buf, \
//         size)

/*
    Macro that call the ASYNC_I2C_Mem_Tx_or_Rx_DMA_init function with the LSM303AGR
    magnetometer parameters.
    It requires:
        - SCHEUDLER struct named "scheduler"
        - TASK struct named "task"
        - ASYNC_LSM303AGR_..._CONTEXT struct named "context" witch is the context of the task
    
    ... to be defined in the current scope.

    Parameters:
        - add: the memory address to read or write
        - buf: the buffer to read or write
        - size: the size of the buffer in bytes
*/
// #define ASYNC_I2C_Mem_Tx_or_Rx_Mag_LSM303AGR(add, buf, size) \
//     ASYNC_I2C_Mem_Tx_or_Rx_DMA_init( \
//         task, \
//         context->lsm->hi2c, \
//         LSM303AGR_SAD_M << 1, \
//         add, \
//         I2C_MEMADD_SIZE_8BIT, \
//         buf, \
//         size)


// #define ASYNC_LSM303AGR_ReadSensor_NUMBER 5

// typedef enum ASYNC_LSM303AGR_ReadSensor_STATE {
//     ASYNC_LSM303AGR_ReadSensor_START,
//     ASYNC_LSM303AGR_ReadSensor_WAIT_READ,
//     ASYNC_LSM303AGR_ReadSensor_END
// } ASYNC_LSM303AGR_ReadSensor_STATE;

// typedef struct ASYNC_LSM303AGR_ReadSensor_CONTEXT {
//     LSM303AGR *lsm; // Pointer to LSM303AGR structure

//     bool is_done; // Flag to indicate if the read operation is done
    
//     uint8_t raw_sensor_data[6]; // Raw sensor data buffer (acceleration or magnetometer data)
//     FLOAT3 *sensor_data; // Sensor data buffer (acceleration or magnetometer data)
    
//     ASYNC_LSM303AGR_ReadSensor_STATE state; // Current state of the asynchronous operation
// } ASYNC_LSM303AGR_ReadSensor_CONTEXT;

// extern TASK_POOL_CREATE(ASYNC_LSM303AGR_ReadSensor);

// void ASYNC_LSM303AGR_ReadSensor_init(TASK *self, LSM303AGR *lsm, FLOAT3 *sensor_data);
// TASK_RETURN ASYNC_LSM303AGR_ReadAcc(SCHEDULER *scheduler, TASK *self);
// TASK_RETURN ASYNC_LSM303AGR_ReadMag(SCHEDULER *scheduler, TASK *self);





#endif // LSM303AGR_H