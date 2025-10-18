#include "drivers/LSM303AGR.h"
#include "stdbool.h"

void LSM303AGR_Init(LSM303AGR *lsm, I2C_HandleTypeDef *hi2c) {
    lsm->hi2c = hi2c;

    uint8_t data;

    // Accelerometer configuration
    HAL_I2C_Mem_Read(hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_WHO_AM_I_A, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    if (data != LSM303AGR_DEVICE_ID_A) {
        // Handle error: Device ID mismatch
        return;
    }

    // data = 0x57; // 0b01010111 // 100Hz, normal mode, all axes enabled
    data = 0x00;
    data |= LSM303AGR_CTRL_REG1_A_ODR_100HZ_MASK;   // Output data rate: 100Hz
    data &= ~LSM303AGR_CTRL_REG1_A_LPEN_MASK;       // Normal mode
    data |= LSM303AGR_CTRL_REG1_A_XEN_MASK;         // X axis enabled
    data |= LSM303AGR_CTRL_REG1_A_YEN_MASK;         // Y axis enabled
    data |= LSM303AGR_CTRL_REG1_A_ZEN_MASK;         // Z axis enabled
    HAL_I2C_Mem_Write(hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_CTRL_REG1_A, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    // data = 0x00;
    // data |= LSM303AGR_CTRL_REG2_A_HPM_NORMAL_MASK; // High-pass filter mode: normal
    // data &= ~LSM303AGR_CTRL_REG2_A_HPFCLICK_MASK; // High-pass filter click detection: disabled
    // data &= ~LSM303AGR_CTRL_REG2_A_HPISO2_MASK; // High-pass filter interrupt 2: disabled
    // data &= ~LSM303AGR_CTRL_REG2_A_HPISO1_MASK; // High-pass filter interrupt 1: disabled
    // HAL_I2C_Mem_Write(hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_CTRL_REG2_A, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    data = 0x00;
    data |= LSM303AGR_CTRL_REG4_A_BDU_MASK;         // Block data update: blocking mode
    data &= ~LSM303AGR_CTRL_REG4_A_BLE_MASK;        // LSB first
    data |= LSM303AGR_CTRL_REG4_A_FS_2G_MASK;       // +/-2g
    data |= LSM303AGR_CTRL_REG4_A_HR_EN_MASK;       // High resolution: enabled
    data |= LSM303AGR_CTRL_REG4_A_ST_NORMAL_MASK;   // Self-test: normal mode
    data &= ~LSM303AGR_CTRL_REG4_A_SPI_EN_MASK;     // SPI 3-wire interface disabled
    HAL_I2C_Mem_Write(hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_CTRL_REG4_A, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    
    lsm->acc_conv = LSM303AGR_LIN_ACC_SO_2G_HR / 16.0f * 9.81f / 1000.0f; // Set conversion factor for +/-2g range high resolution
    // (in hr mode, 10 bit resolution in left justified mode so data is divided by 16 to shift right 4 bits)
    // (1000 mg = 1 g, so we convert to m/s^2 by dividing by 1000 and multiplying by 9.81)

    // Magnetometer configuration
    HAL_I2C_Mem_Read(hi2c, LSM303AGR_SAD_M << 1, LSM303AGR_WHO_AM_I_M, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    if (data != LSM303AGR_DEVICE_ID_M) {
        // Handle error: Device ID mismatch
        return;
    }

    data = 0x00;
    data |= LSM303AGR_REG_CFG_REG_A_M_COMP_TEMP_MASK;       // Temperature compensation enabled
    data &= ~LSM303AGR_REG_CFG_REG_A_M_REBOOT_MASK;         // Reboot memory content disabled
    data &= ~LSM303AGR_REG_CFG_REG_A_M_SOFT_RST_MASK;       // Soft reset disabled
    data &= ~LSM303AGR_REG_CFG_REG_A_M_LP_MASK;             // Low power mode enabled
    data |= LSM303AGR_REG_CFG_REG_A_M_ODR_100HZ_MASK;       // Output data rate: 100Hz
    data |= LSM303AGR_REG_CFG_REG_A_M_MD_CONTINUOUS_MASK;   // Mode: continuous
    HAL_I2C_Mem_Write(hi2c, LSM303AGR_SAD_M << 1, LSM303AGR_REG_CFG_REG_A_M, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    data = 0x00;
    data &= ~LSM303AGR_REG_CFG_REG_B_M_OFF_CANC_ONE_SHOT_MASK;  // Offset cancellation on single measurement disabled
    data &= ~LSM303AGR_REG_CFG_REG_B_M_INT_ON_DATA_OFF_MASK;    // Interrupt on data recognition disabled
    data &= ~LSM303AGR_REG_CFG_REG_B_M_SET_FREQ_MASK;           // Set frequency of the set pulse disabled
    data &= ~LSM303AGR_REG_CFG_REG_B_M_OFF_CANC_MASK;           // Offset cancellation disabled
    data |= LSM303AGR_REG_CFG_REG_B_M_LPF_MASK;                 // Low-pass filter enabled
    HAL_I2C_Mem_Write(hi2c, LSM303AGR_SAD_M << 1, LSM303AGR_REG_CFG_REG_B_M, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

    data = 0x00;
    data &= ~LSM303AGR_REG_CFG_REG_C_M_INT_MAG_PIN_MASK;    // Interrupt pin disabled
    data &= ~LSM303AGR_REG_CFG_REG_C_M_I2C_DIS_MASK;        // I2C interface enabled
    data |= LSM303AGR_REG_CFG_REG_C_M_BDU_MASK;             // Block data update: enabled
    data &= ~LSM303AGR_REG_CFG_REG_C_M_BLE_MASK;            // Big/Little Endian: LSB first
    data &= ~LSM303AGR_REG_CFG_REG_C_M_SELF_TEST_MASK;      // Self-test: disabled
    data &= ~LSM303AGR_REG_CFG_REG_C_M_INT_MAG_MASK;        // DRDY pin disabled
    HAL_I2C_Mem_Write(hi2c, LSM303AGR_SAD_M << 1, LSM303AGR_REG_CFG_REG_C_M, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}



void LSM303AGR_SelfTestAcc(LSM303AGR *lsm) {

//     // Read current accelerometer configuration and store it
//     uint8_t data_reg_1, data_reg_2, data_reg_3, data_reg_4;
//     HAL_I2C_Mem_Read(lsm->hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_CTRL_REG1_A, I2C_MEMADD_SIZE_8BIT, &data_reg_1, 1, HAL_MAX_DELAY);
//     HAL_I2C_Mem_Read(lsm->hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_CTRL_REG2_A, I2C_MEMADD_SIZE_8BIT, &data_reg_2, 1, HAL_MAX_DELAY);
//     HAL_I2C_Mem_Read(lsm->hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_CTRL_REG3_A, I2C_MEMADD_SIZE_8BIT, &data_reg_3, 1, HAL_MAX_DELAY);
//     HAL_I2C_Mem_Read(lsm->hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_CTRL_REG4_A, I2C_MEMADD_SIZE_8BIT, &data_reg_4, 1, HAL_MAX_DELAY);

//     // Set accelerometer configuration for self-test
//     uint8_t data[6];
//     data[0] = 0x00;
//     HAL_I2C_Mem_Write(lsm->hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_CTRL_REG2_A, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
//     data[0] = 0x00;
//     HAL_I2C_Mem_Write(lsm->hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_CTRL_REG3_A, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
//     data[0] = 0x81;
//     HAL_I2C_Mem_Write(lsm->hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_CTRL_REG4_A, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
//     data[0] = 0x57; // 0b01010111 // 100Hz, normal mode, all axes enabled
//     HAL_I2C_Mem_Write(lsm->hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_CTRL_REG1_A, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

//     HAL_Delay(100); // Wait for configuration to take effect

//     // Wait for data ready
//     uint8_t status = 0;
//     while (!(status & 0x08)) {
//         HAL_I2C_Mem_Read(lsm->hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_STATUS_REG_A, I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY);
//     };

//     // Read and discard initial accelerometer data
//     HAL_I2C_Mem_Read(lsm->hi2c, LSM303AGR_SAD_A << 1,
//         LSM303AGR_REG_OUT_X_L_A | LSM303AGR_AUTO_INCREMENT,
//         I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

//     HAL_Delay(10); // Allow some time for data to stabilize

    
//     // Read accelerometer data in normal mode
//     int16_t OUT_X_NOST_sum = 0, OUT_Y_NOST_sum = 0, OUT_Z_NOST_sum = 0;
//     float OUT_X_NOST, OUT_Y_NOST, OUT_Z_NOST;
//     for (int i = 0; i < 5; i++) {
//         HAL_I2C_Mem_Read(lsm->hi2c, LSM303AGR_SAD_A << 1,
//             LSM303AGR_REG_OUT_X_L_A | LSM303AGR_AUTO_INCREMENT,
//             I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

//         OUT_X_NOST_sum += (int16_t)((data[1] << 8) | data[0]);
//         OUT_Y_NOST_sum += (int16_t)((data[3] << 8) | data[2]);
//         OUT_Z_NOST_sum += (int16_t)((data[5] << 8) | data[4]);
//     }
//     // Calculate average values
//     OUT_X_NOST = (float)OUT_X_NOST_sum / 5.0f;
//     OUT_Y_NOST = (float)OUT_Y_NOST_sum / 5.0f;
//     OUT_Z_NOST = (float)OUT_Z_NOST_sum / 5.0f;

//     data[0] = 0X85; // enable self-test mode
//     HAL_I2C_Mem_Write(lsm->hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_CTRL_REG4_A, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

//     HAL_Delay(100); // Wait for self-test configuration to take effect

//     // Wait for data ready
//     status = 0;
//     while (!(status & 0x08)) {
//         HAL_I2C_Mem_Read(lsm->hi2c, LSM303AGR_SAD_A << 1, LSM303AGR_REG_STATUS_REG_A, I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY);
//     };

//     // Read and discard initial accelerometer data
//     HAL_I2C_Mem_Read(lsm->hi2c, LSM303AGR_SAD_A << 1,
//         LSM303AGR_REG_OUT_X_L_A | LSM303AGR_AUTO_INCREMENT,
//         I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

//     HAL_Delay(10); // Allow some time for data to stabilize

//     // Read accelerometer data in self-test mode
//     int16_t OUT_X_ST_sum = 0, OUT_Y_ST_sum = 0, OUT_Z_ST_sum = 0;
//     float OUT_X_ST, OUT_Y_ST, OUT_Z_ST;
//     for (int i = 0; i < 5; i++) {
//         HAL_I2C_Mem_Read(lsm->hi2c, LSM303AGR_SAD_A << 1,
//             LSM303AGR_REG_OUT_X_L_A | LSM303AGR_AUTO_INCREMENT,
//             I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

//         OUT_X_ST_sum += (int16_t)((data[1] << 8) | data[0]);
//         OUT_Y_ST_sum += (int16_t)((data[3] << 8) | data[2]);
//         OUT_Z_ST_sum += (int16_t)((data[5] << 8) | data[4]);
//     }
//     // Calculate average values
//     OUT_X_ST = (float)OUT_X_ST_sum / 5.0f;
//     OUT_Y_ST = (float)OUT_Y_ST_sum / 5.0f;
//     OUT_Z_ST = (float)OUT_Z_ST_sum / 5.0f;
}




void LSM303AGR_ReadAccRaw(LSM303AGR *lsm, int16_t *acc_x, int16_t *acc_y, int16_t *acc_z) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(lsm->hi2c, LSM303AGR_SAD_A << 1,
        LSM303AGR_REG_OUT_X_L_A | LSM303AGR_AUTO_INCREMENT,
        I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

    *acc_x = (int16_t)((data[1] << 8) | data[0]);
    *acc_y = (int16_t)((data[3] << 8) | data[2]);
    *acc_z = (int16_t)((data[5] << 8) | data[4]);
}

void LSM303AGR_ReadAcc(LSM303AGR *lsm, float *acc_x, float *acc_y, float *acc_z) {
    int16_t raw_acc_x, raw_acc_y, raw_acc_z;
    LSM303AGR_ReadAccRaw(lsm, &raw_acc_x, &raw_acc_y, &raw_acc_z);

    *acc_x = raw_acc_x * lsm->acc_conv; // Convert to m/s^2
    *acc_y = raw_acc_y * lsm->acc_conv; // Convert to m/s^2
    *acc_z = raw_acc_z * lsm->acc_conv; // Convert to m/s^2
}

void LSM303AGR_ReadMagRaw(LSM303AGR *lsm, int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(lsm->hi2c, LSM303AGR_SAD_M << 1,
        LSM303AGR_REG_OUTX_L_M | LSM303AGR_AUTO_INCREMENT,
        I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

    *mag_x = (int16_t)((data[1] << 8) | data[0]);
    *mag_y = (int16_t)((data[3] << 8) | data[2]);
    *mag_z = (int16_t)((data[5] << 8) | data[4]);
}

void LSM303AGR_ReadMag(LSM303AGR *lsm, float *mag_x, float *mag_y, float *mag_z) {
    int16_t raw_mag_x, raw_mag_y, raw_mag_z;
    LSM303AGR_ReadMagRaw(lsm, &raw_mag_x, &raw_mag_y, &raw_mag_z);

    *mag_x = raw_mag_x * LSM303AGR_LIN_MAG_SO; // Convert to mGauss
    *mag_y = raw_mag_y * LSM303AGR_LIN_MAG_SO; // Convert to mGauss
    *mag_z = raw_mag_z * LSM303AGR_LIN_MAG_SO; // Convert to mGauss
}



// TASK_POOL_CREATE(ASYNC_LSM303AGR_ReadSensor);

// void ASYNC_LSM303AGR_ReadSensor_init(TASK *self, LSM303AGR *lsm, FLOAT3 *sensor_data) {
//     ASYNC_LSM303AGR_ReadSensor_CONTEXT *context = (ASYNC_LSM303AGR_ReadSensor_CONTEXT*)self->context;
//     context->lsm = lsm;
//     context->is_done = false;
//     context->raw_sensor_data[0] = 0; // Initialize raw sensor data
//     context->raw_sensor_data[1] = 0; // Initialize raw sensor data
//     context->raw_sensor_data[2] = 0; // Initialize raw sensor data
//     context->sensor_data = sensor_data;
//     context->state = ASYNC_LSM303AGR_ReadSensor_START;
// }

// TASK_RETURN ASYNC_LSM303AGR_ReadAcc(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_LSM303AGR_ReadSensor_CONTEXT *context = (ASYNC_LSM303AGR_ReadSensor_CONTEXT*)self->context;

//     switch (context->state) {
//     case ASYNC_LSM303AGR_ReadSensor_START: {
//         context->is_done = false;
//         // Read accelerometer data

//         TASK *task = SCHEDULER_add_task(scheduler, ASYNC_I2C_Mem_Rx_DMA, true, (OBJ_POOL*)ASYNC_I2C_Mem_Tx_or_Rx_DMA_POOL);
//         ASYNC_I2C_Mem_Tx_or_Rx_Acc_LSM303AGR(LSM303AGR_REG_OUT_X_L_A | LSM303AGR_AUTO_INCREMENT,
//                                              context->raw_sensor_data,
//                                              sizeof(context->raw_sensor_data));
//         task->is_done = &(context->is_done);

//         context->state = ASYNC_LSM303AGR_ReadSensor_WAIT_READ;
//         break; }
//     case ASYNC_LSM303AGR_ReadSensor_WAIT_READ: {
//         if (context->is_done) {
//             // Convert raw data to float values
//             int16_t raw_acc_x = (int16_t)((context->raw_sensor_data[1] << 8) | context->raw_sensor_data[0]);
//             int16_t raw_acc_y = (int16_t)((context->raw_sensor_data[3] << 8) | context->raw_sensor_data[2]);
//             int16_t raw_acc_z = (int16_t)((context->raw_sensor_data[5] << 8) | context->raw_sensor_data[4]);

//             context->sensor_data->x = raw_acc_x * context->lsm->acc_conv; // Convert to m/s^2
//             context->sensor_data->y = raw_acc_y * context->lsm->acc_conv; // Convert to m/s^2
//             context->sensor_data->z = raw_acc_z * context->lsm->acc_conv; // Convert to m/s^2

//             context->state = ASYNC_LSM303AGR_ReadSensor_END;
//         }
//         break; }
//     case ASYNC_LSM303AGR_ReadSensor_END: {
//         // Task is done, return stop signal
//         return TASK_RETURN_STOP;
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }

// TASK_RETURN ASYNC_LSM303AGR_ReadMag(SCHEDULER *scheduler, TASK *self) {
//     ASYNC_LSM303AGR_ReadSensor_CONTEXT *context = (ASYNC_LSM303AGR_ReadSensor_CONTEXT*)self->context;

//     switch (context->state) {
//     case ASYNC_LSM303AGR_ReadSensor_START: {
//         context->is_done = false;
//         // Read magnetometer data

//         TASK *task = SCHEDULER_add_task(scheduler, ASYNC_I2C_Mem_Rx_DMA, true, (OBJ_POOL*)ASYNC_I2C_Mem_Tx_or_Rx_DMA_POOL);
//         ASYNC_I2C_Mem_Tx_or_Rx_Mag_LSM303AGR(LSM303AGR_REG_OUTX_L_M | LSM303AGR_AUTO_INCREMENT,
//                                              context->raw_sensor_data,
//                                              sizeof(context->raw_sensor_data));
//         task->is_done = &(context->is_done);

//         context->state = ASYNC_LSM303AGR_ReadSensor_WAIT_READ;
//         break; }
//     case ASYNC_LSM303AGR_ReadSensor_WAIT_READ: {
//         if (context->is_done) {
//             // Convert raw data to float values
//             int16_t raw_mag_x = (int16_t)((context->raw_sensor_data[1] << 8) | context->raw_sensor_data[0]);
//             int16_t raw_mag_y = (int16_t)((context->raw_sensor_data[3] << 8) | context->raw_sensor_data[2]);
//             int16_t raw_mag_z = (int16_t)((context->raw_sensor_data[5] << 8) | context->raw_sensor_data[4]);

//             context->sensor_data->x = raw_mag_x * LSM303AGR_LIN_MAG_SO; // Convert to mGauss
//             context->sensor_data->y = raw_mag_y * LSM303AGR_LIN_MAG_SO; // Convert to mGauss
//             context->sensor_data->z = raw_mag_z * LSM303AGR_LIN_MAG_SO; // Convert to mGauss

//             context->state = ASYNC_LSM303AGR_ReadSensor_END;
//         }
//         break; }
//     case ASYNC_LSM303AGR_ReadSensor_END: {
//         // Task is done, return stop signal
//         return TASK_RETURN_STOP;
//         break; }
//     }
//     return TASK_RETURN_IDLE;
// }


