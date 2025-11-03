#include "drivers/BMI088.h"
#include "cmsis_os2.h"
#include "peripherals/spi.h"
#include "stm32f4xx_hal_spi.h"
#include "utils/circular_buffer.h"
#include "utils/data_topic.h"
#include "utils/scheduler.h"
#include "utils/tools.h"
#include "utils/types.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/*
 *
 * INITIALISATION
 *
 */
uint8_t BMI088_Init(BMI088 *imu,
				    SPI_HandleTypeDef *spiHandle,
				    GPIO_TypeDef *csAccPinBank, uint16_t csAccPin,
				    GPIO_TypeDef *csGyrPinBank, uint16_t csGyrPin) {

	/* Store interface parameters in struct */
	imu->spiHandle 		= spiHandle;
	imu->csAccPinBank 	= csAccPinBank;
	imu->csAccPin 		= csAccPin;
	imu->csGyrPinBank 	= csGyrPinBank;
	imu->csGyrPin 		= csGyrPin;

	imu->new_acc_data = false;
	imu->new_gyr_data = false;

	imu->ASYNC_busy = false;

	uint8_t status = 0;

	/*
	 *
	 * ACCELEROMETER
	 *
	 */

	uint8_t ACC_Range = BMI_ACC_RANGE_24G;

	/* Accelerometer requires rising edge on CSB at start-up to activate SPI */
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);
	HAL_Delay(50);

	/* Perform accelerometer soft reset */
	status += BMI088_WriteAccRegister(imu, BMI_ACC_SOFTRESET, 0xB6);
	HAL_Delay(50);

	/* Check chip ID */
	uint8_t chipID;
	status += BMI088_ReadAccRegister(imu, BMI_ACC_CHIP_ID, &chipID);

	if (chipID != 0x1E) {
		__NOP();
	//	return 0;

	}
	HAL_Delay(10);

	/* Configure accelerometer  */
	uint8_t accConf = BMI_ACC_BWP_NORMAL | BMI_ACC_ODR_100;
	status += BMI088_WriteAccRegister(imu, BMI_ACC_CONF, accConf); /* (no oversampling, ODR = 100 Hz, BW = 40 Hz) */
	HAL_Delay(10);

	status += BMI088_WriteAccRegister(imu, BMI_ACC_RANGE, ACC_Range); /* +- 3g range */
	HAL_Delay(10);

	/* Put accelerometer into active mode */
	status += BMI088_WriteAccRegister(imu, BMI_ACC_PWR_CONF, 0x00);
	HAL_Delay(10);

	/* Turn accelerometer on */
	status += BMI088_WriteAccRegister(imu, BMI_ACC_PWR_CTRL, 0x04);
	HAL_Delay(10);

	/* Pre-compute accelerometer conversion constant (raw to m/s^2) */
	imu->accConversion = (9.81f / 32768.0f) * pow(2.0, ACC_Range + 1) * 1.5f; /* Datasheet page 21-22 */

	/* Init accelerometer offset (m/s^2) */
	imu->acc_mps2_offset.x = 0.0f;
	imu->acc_mps2_offset.y = 0.0f;
	imu->acc_mps2_offset.z = 0.0f;

	/*
	 *
	 * GYROSCOPE
	 *
	 */

	uint8_t GYR_Range = BMI_GYR_RANGE_2000;

	
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

	/* Perform gyro soft reset */
	status += BMI088_WriteGyrRegister(imu, BMI_GYR_SOFTRESET, 0xB6);
	HAL_Delay(250);

	/* Check chip ID */
	status += BMI088_ReadGyrRegister(imu, BMI_GYR_CHIP_ID, &chipID);

	if (chipID != 0x0F) {
		__NOP();
		//return 0;

	}
	HAL_Delay(10);

	/* Configure gyroscope */
	status += BMI088_WriteGyrRegister(imu, BMI_GYR_RANGE, GYR_Range); /* +- 1000 deg/s */
	HAL_Delay(10);

	status += BMI088_WriteGyrRegister(imu, BMI_GYR_BANDWIDTH, BMI_GYR_BANDWIDTH_12);
	HAL_Delay(10);

	/* Pre-compute gyroscope conversion constant (raw to rad/s) */
	// imu->gyrConversion = (M_PI / 180.0f) * 2000 / (32768.0f * pow(2.0, GYR_Range)); /* Datasheet page 28 */
	imu->gyrConversion = 2000 / (32768.0f * pow(2.0, GYR_Range)); /* Datasheet page 28 */

	/* Init gyroscope offset (rad/s) */
	imu->gyr_rps_offset.x = 0.0f;
	imu->gyr_rps_offset.y = 0.0f;
	imu->gyr_rps_offset.z = 0.0f;

	return status;

}



/* ACCELEROMETER READS ARE DIFFERENT TO GYROSCOPE READS. SEND ONE BYTE ADDRESS, READ ONE DUMMY BYTE, READ TRUE DATA !!! */
uint8_t BMI088_ReadAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[3] = {regAddr | 0x80, 0x00, 0x00};
	uint8_t rxBuf[3];

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 3, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);

	if (status == 1) {

		*data = rxBuf[2];

	}

	return status;

}

uint8_t BMI088_ReadGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[2] = {regAddr | 0x80, 0x00};
	uint8_t rxBuf[2];

	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

	if (status == 1) {

		*data = rxBuf[1];

	}

	return status;

}

uint8_t BMI088_WriteAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);

	return status;

}

uint8_t BMI088_WriteGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	while(HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

	return status;

}


uint8_t BMI088_ReadAccelerometer(BMI088 *imu) {

	/* Read raw accelerometer data */
	uint8_t txBuf[8] = {(BMI_ACC_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 1 byte dummy, 6 bytes data */
	uint8_t rxBuf[8];

	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 8, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csAccPinBank, imu->csAccPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
	int16_t accX = (int16_t) ((rxBuf[3] << 8) | rxBuf[2]);
	int16_t accY = (int16_t) ((rxBuf[5] << 8) | rxBuf[4]);
	int16_t accZ = (int16_t) ((rxBuf[7] << 8) | rxBuf[6]);

	/* Convert to m/s^2 */
	imu->acc_mps2.x = imu->accConversion * accX - imu->acc_mps2_offset.x;
	imu->acc_mps2.y = imu->accConversion * accY - imu->acc_mps2_offset.y;
	imu->acc_mps2.z = imu->accConversion * accZ - imu->acc_mps2_offset.z;

	return status;

}

uint8_t BMI088_ReadGyroscope(BMI088 *imu) {

	/* Read raw gyroscope data */
	uint8_t txBuf[7] = {(BMI_GYR_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 6 bytes data */
	uint8_t rxBuf[7];

	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 7, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
	int16_t gyrX = (int16_t) ((rxBuf[2] << 8) | rxBuf[1]);
	int16_t gyrY = (int16_t) ((rxBuf[4] << 8) | rxBuf[3]);
	int16_t gyrZ = (int16_t) ((rxBuf[6] << 8) | rxBuf[5]);

	/* Convert to rad/s */
	imu->gyr_rps.x = imu->gyrConversion * gyrX - imu->gyr_rps_offset.x;
	imu->gyr_rps.y = imu->gyrConversion * gyrY - imu->gyr_rps_offset.y;
	imu->gyr_rps.z = imu->gyrConversion * gyrZ - imu->gyr_rps_offset.z;

	return status;

}


bool BMI088_SelfTestAccelerometer(BMI088 *imu) {
	float acc_x[256], acc_y[256], acc_z[256];
	float acc_x_pos = 0, acc_y_pos = 0, acc_z_pos = 0;
	float acc_x_neg = 0, acc_y_neg = 0, acc_z_neg = 0;
	float acc_x_res = 0, acc_y_res = 0, acc_z_res = 0;

	// Set to +-24g range
	BMI088_WriteAccRegister(imu, BMI_ACC_RANGE, BMI_ACC_RANGE_24G);
	BMI088_WriteAccRegister(imu, BMI_ACC_CONF, BMI_ACC_BWP_NORMAL | BMI_ACC_ODR_1600);
	HAL_Delay(5);

	BMI088_WriteAccRegister(imu, BMI_ACC_SELF_TEST, BMI_ACC_P_SELF_TEST);
	HAL_Delay(55);

	for (int i = 0; i < 256; i++) {
		BMI088_ReadAccelerometer(imu);
		acc_x[i] = imu->acc_mps2.x * 1000 / 9.81f;
		acc_y[i] = imu->acc_mps2.y * 1000 / 9.81f;
		acc_z[i] = imu->acc_mps2.z * 1000 / 9.81f;
		HAL_Delay(2);
	}

	for (int i = 0; i < 256; i++) {
		acc_x_pos += acc_x[i];
		acc_y_pos += acc_y[i];
		acc_z_pos += acc_z[i];
	}
	acc_x_pos /= 256.0f - 1.0f; // Remove 1 degree of freedom
	acc_y_pos /= 256.0f - 1.0f; // Remove 1 degree of freedom
	acc_z_pos /= 256.0f - 1.0f; // Remove 1 degree of freedom
	
	BMI088_WriteAccRegister(imu, BMI_ACC_SELF_TEST, BMI_ACC_N_SELF_TEST);
	HAL_Delay(55);

	for (int i = 0; i < 256; i++) {
		BMI088_ReadAccelerometer(imu);
		acc_x[i] = imu->acc_mps2.x * 1000 / 9.81f;
		acc_y[i] = imu->acc_mps2.y * 1000 / 9.81f;
		acc_z[i] = imu->acc_mps2.z * 1000 / 9.81f;
		HAL_Delay(10);
	}

	for (int i = 0; i < 256; i++) {
		acc_x_neg += acc_x[i];
		acc_y_neg += acc_y[i];
		acc_z_neg += acc_z[i];
	}

	acc_x_res = fabs(acc_x_pos - acc_x_neg);
	acc_y_res = fabs(acc_y_pos - acc_y_neg);
	acc_z_res = fabs(acc_z_pos - acc_z_neg);

	BMI088_WriteAccRegister(imu, BMI_ACC_SELF_TEST, BMI_ACC_NO_SELF_TEST);
	HAL_Delay(55);

	BMI088_Init(imu,imu->spiHandle,
		        imu->csAccPinBank, imu->csAccPin,
				imu->csGyrPinBank, imu->csGyrPin);

	return ((acc_x_res > 1000.0f) && (acc_y_res > 1000.0f) && (acc_z_res > 500.0f));
}

bool BMI088_SelfTestGyroscope(BMI088 *imu) {
	uint8_t st_reg;
	uint8_t st_reg_ready = 0;
	uint8_t st_reg_failed = 1;
	uint8_t st_reg_rate_ok = 0;

	BMI088_WriteGyrRegister(imu, BMI_GYR_SELF_TEST, (1 << BMI_GYR_ST_B_TRIGGER));
	while (!st_reg_ready) {
		BMI088_ReadGyrRegister(imu, BMI_GYR_SELF_TEST, &st_reg);
		st_reg_ready = (st_reg >> BMI_GYR_ST_B_READY) & 0x01;
		HAL_Delay(1);
	}
	st_reg_failed = (st_reg >> BMI_GYR_ST_B_FAILED) & 0x01;
	st_reg_rate_ok = (st_reg >> BMI_GYR_ST_B_RATE_OK) & 0x01;

	return ((!st_reg_failed) && (st_reg_rate_ok));
}


BMI088_OffsetData BMI088_OffsetAccelerometer(BMI088 *imu) {
	BMI088_OffsetData offsetData = {0};
	float acc_x[256], acc_y[256], acc_z[256];

	for (int i = 0; i < 256; i++) {
		BMI088_ReadAccelerometer(imu);
		acc_x[i] = imu->acc_mps2.x + imu->acc_mps2_offset.x;
		acc_y[i] = imu->acc_mps2.y + imu->acc_mps2_offset.y;
		acc_z[i] = imu->acc_mps2.z + imu->acc_mps2_offset.z;
		HAL_Delay(10);
	}

	for (int i = 0; i < 256; i++) {
		offsetData.avg.x += acc_x[i];
		offsetData.avg.y += acc_y[i];
		offsetData.avg.z += acc_z[i];
	}
	offsetData.avg.x /= 256.0f - 1.0f; // Remove 1 degree of freedom
	offsetData.avg.y /= 256.0f - 1.0f; // Remove 1 degree of freedom
	offsetData.avg.z /= 256.0f - 1.0f; // Remove 1 degree of freedom

	for (int i = 0; i < 256; i++) {
		offsetData.std.x += pow(acc_x[i] - offsetData.avg.x, 2.0f);
		offsetData.std.y += pow(acc_y[i] - offsetData.avg.y, 2.0f);
		offsetData.std.z += pow(acc_z[i] - offsetData.avg.z, 2.0f);
	}
	offsetData.std.x = sqrt(offsetData.std.x / (256.0f - 1.0f)); // Remove 1 degree of freedom
	offsetData.std.y = sqrt(offsetData.std.y / (256.0f - 1.0f)); // Remove 1 degree of freedom
	offsetData.std.z = sqrt(offsetData.std.z / (256.0f - 1.0f)); // Remove 1 degree of freedom

	return offsetData;
}

BMI088_OffsetData BMI088_OffsetGyroscope(BMI088 *imu) {
	BMI088_OffsetData offsetData = {0};
	float gyr_x[256], gyr_y[256], gyr_z[256];

	for (int i = 0; i < 256; i++) {
		BMI088_ReadGyroscope(imu);
		gyr_x[i] = imu->gyr_rps.x + imu->gyr_rps_offset.x;
		gyr_y[i] = imu->gyr_rps.y + imu->gyr_rps_offset.y;
		gyr_z[i] = imu->gyr_rps.z + imu->gyr_rps_offset.z;
		HAL_Delay(10);
	}

	for (int i = 0; i < 256; i++) {
		offsetData.avg.x += gyr_x[i];
		offsetData.avg.y += gyr_y[i];
		offsetData.avg.z += gyr_z[i];
	}
	offsetData.avg.x /= 256.0f - 1.0f; // Remove 1 degree of freedom
	offsetData.avg.y /= 256.0f - 1.0f; // Remove 1 degree of freedom
	offsetData.avg.z /= 256.0f - 1.0f; // Remove 1 degree of freedom

	for (int i = 0; i < 256; i++) {
		offsetData.std.x += pow(gyr_x[i] - offsetData.avg.x, 2.0f);
		offsetData.std.y += pow(gyr_y[i] - offsetData.avg.y, 2.0f);
		offsetData.std.z += pow(gyr_z[i] - offsetData.avg.z, 2.0f);
	}
	offsetData.std.x = sqrt(offsetData.std.x / (256.0f - 1.0f)); // Remove 1 degree of freedom
	offsetData.std.y = sqrt(offsetData.std.y / (256.0f - 1.0f)); // Remove 1 degree of freedom
	offsetData.std.z = sqrt(offsetData.std.z / (256.0f - 1.0f)); // Remove 1 degree of freedom

	return offsetData;
}

TASK_POOL_ALLOCATE(TASK_BMI088_ReadAcc);

void TASK_BMI088_ReadAcc(void *argument) {
	TASK_BMI088_ReadAcc_ARGS *args = (TASK_BMI088_ReadAcc_ARGS*)(argument);

	BMI088 *imu = args->imu;
	data_topic_t **dt_ptr = args->dt;
	uint32_t delay = args->delay;

	uint8_t storage[CIRCULAR_BUFFER_BYTES(FLOAT3, 16)];
	data_topic_t dt;
	*dt_ptr = &dt;

	data_topic_init(&dt, storage, sizeof(FLOAT3), 16, CB_OVERWRITE_OLDEST);

	FLOAT3 accData;

	for (;;) {
		uint8_t cmd = BMI_ACC_DATA | 0x80; /* Register addr, 1 byte dummy, 6 bytes data */
		uint8_t rxBuf[6];

		// TASK_HAL_SPI_TransmitReceive_DMA(imu->spiHandle, imu->csAccPinBank, imu->csAccPin,
		// 	                             txBuf, rxBuf, 8);
		SPI_Begin_DMA_RTOS(imu->spiHandle, imu->csAccPinBank, imu->csAccPin);
		SPI_Transmit_DMA_RTOS(imu->spiHandle, &cmd, 1);
		SPI_Receive_DMA_RTOS(imu->spiHandle, rxBuf, 1); // Dummy byte
		SPI_Receive_DMA_RTOS(imu->spiHandle, rxBuf, 6); // Data bytes
		SPI_End_DMA_RTOS(imu->spiHandle, imu->csAccPinBank, imu->csAccPin);

		/* Form signed 16-bit integers */
		int16_t accX = (int16_t) ((rxBuf[1] << 8) | rxBuf[0]);
		int16_t accY = (int16_t) ((rxBuf[3] << 8) | rxBuf[2]);
		int16_t accZ = (int16_t) ((rxBuf[5] << 8) | rxBuf[4]);

		/* Convert to m/s^2 and construct FLOAT3 */
		accData.x = imu->accConversion * accX - imu->acc_mps2_offset.x;
		accData.y = imu->accConversion * accY - imu->acc_mps2_offset.y;
		accData.z = imu->accConversion * accZ - imu->acc_mps2_offset.z;

		data_topic_publish(&dt, &accData);

		osDelay(delay);
	}
}

TASK_POOL_ALLOCATE(TASK_BMI088_ReadGyr);

void TASK_BMI088_ReadGyr(void *argument) {
	TASK_BMI088_ReadGyr_ARGS *args = (TASK_BMI088_ReadGyr_ARGS*)(argument);

	BMI088 *imu = args->imu;
	data_topic_t **dt_ptr = args->dt;
	uint32_t delay = args->delay;

	uint8_t storage[CIRCULAR_BUFFER_BYTES(FLOAT3, 16)];
	data_topic_t dt;
	*dt_ptr = &dt;

	data_topic_init(&dt, storage, sizeof(FLOAT3), 16, CB_OVERWRITE_OLDEST);

	FLOAT3 gyrData;

	for (;;) {
		uint8_t cmd = BMI_GYR_DATA | 0x80; /* Register addr */
		uint8_t rxBuf[6];

		// TASK_HAL_SPI_TransmitReceive_DMA(imu->spiHandle, imu->csGyrPinBank, imu->csGyrPin,
		// 	                             txBuf, rxBuf, 7);

		SPI_Begin_DMA_RTOS(imu->spiHandle, imu->csAccPinBank, imu->csAccPin);
		SPI_Transmit_DMA_RTOS(imu->spiHandle, &cmd, 1);
		SPI_Receive_DMA_RTOS(imu->spiHandle, rxBuf, 6);
		SPI_End_DMA_RTOS(imu->spiHandle, imu->csAccPinBank, imu->csAccPin);

		/* Form signed 16-bit integers */
		int16_t gyrX = (int16_t) ((rxBuf[1] << 8) | rxBuf[0]);
		int16_t gyrY = (int16_t) ((rxBuf[3] << 8) | rxBuf[2]);
		int16_t gyrZ = (int16_t) ((rxBuf[5] << 8) | rxBuf[4]);

		/* Convert to rad/s and construct FLOAT3 */
		gyrData.x = imu->gyrConversion * gyrX - imu->gyr_rps_offset.x;
		gyrData.y = imu->gyrConversion * gyrY - imu->gyr_rps_offset.y;
		gyrData.z = imu->gyrConversion * gyrZ - imu->gyr_rps_offset.z;

		data_topic_publish(&dt, &gyrData);

		osDelay(delay);
	}
}

// TASK_POOL_CREATE(ASYNC_BMI088_ReadSensorDMA);

// void ASYNC_BMI088_ReadSensorDMA_init(TASK *self, BMI088 *imu) {
// 	ASYNC_BMI088_ReadSensorDMA_CONTEXT *context = (ASYNC_BMI088_ReadSensorDMA_CONTEXT*)self->context;

// 	context->imu = imu;
// 	context->state = ASYNC_BMI088_ReadSensorDMA_WAIT_BMI088;

// 	// Can't be a problem because size of txBuf is BMI088_SPI_LEN (8)
// 	memset(context->txBuf, 0, BMI088_SPI_LEN);
// 	memset(context->rxBuf, 0, BMI088_SPI_LEN);
// }

// TASK_RETURN ASYNC_BMI088_ReadAccelerometerDMA(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_BMI088_ReadSensorDMA_CONTEXT *context = (ASYNC_BMI088_ReadSensorDMA_CONTEXT*)self->context;

// 	switch (context->state) {
// 	case ASYNC_BMI088_ReadSensorDMA_WAIT_BMI088: {
// 		if (!(context->imu->ASYNC_busy)) {
// 			context->state = ASYNC_BMI088_ReadSensorDMA_START;
// 			context->imu->ASYNC_busy = true;
// 		}
// 		break;}
// 	case ASYNC_BMI088_ReadSensorDMA_START: {
// 		context->txBuf[0] = BMI_ACC_DATA | 0x80;
// 		TASK *task = SCHEDULER_add_task(scheduler, ASYNC_SPI_TxRx_DMA, true, (OBJ_POOL*)ASYNC_SPI_TxRx_DMA_POOL);
// 		ASYNC_SPI_TxRx_DMA_init_Acc_BMI088(context->txBuf, context->rxBuf, 1, BMI088_SPI_LEN);
// 		task->is_done = &(context->dma_complete);
// 		context->state = ASYNC_BMI088_ReadSensorDMA_WAIT_DMA;
// 		break;}
// 	case ASYNC_BMI088_ReadSensorDMA_WAIT_DMA: {
// 		if (context->dma_complete) {
// 			context->state = ASYNC_BMI088_ReadSensorDMA_END;
// 		}
// 		break;}
// 	case ASYNC_BMI088_ReadSensorDMA_END: {
// 		uint8_t *rate_buf = context->rxBuf + 1; // Skip dummy byte

// 		/* Form signed 16-bit integers */
// 		int16_t accX = (int16_t) ((rate_buf[1] << 8) | rate_buf[0]);
// 		int16_t accY = (int16_t) ((rate_buf[3] << 8) | rate_buf[2]);
// 		int16_t accZ = (int16_t) ((rate_buf[5] << 8) | rate_buf[4]);

// 		/* Convert to m/s^2 */
// 		context->imu->acc_mps2.x = context->imu->accConversion * accX - context->imu->acc_mps2_offset.x;
// 		context->imu->acc_mps2.y = context->imu->accConversion * accY - context->imu->acc_mps2_offset.y;
// 		context->imu->acc_mps2.z = context->imu->accConversion * accZ - context->imu->acc_mps2_offset.z;

// 		context->imu->new_acc_data = true;

// 		context->imu->ASYNC_busy = false;
// 		return TASK_RETURN_STOP;
// 		break;}
// 	}
// 	return TASK_RETURN_IDLE;
// }

// TASK_RETURN ASYNC_BMI088_ReadGyroscopeDMA(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_BMI088_ReadSensorDMA_CONTEXT *context = (ASYNC_BMI088_ReadSensorDMA_CONTEXT*)self->context;

// 	switch (context->state) {
// 	case ASYNC_BMI088_ReadSensorDMA_WAIT_BMI088: {
// 		if (!(context->imu->ASYNC_busy)) {
// 			context->state = ASYNC_BMI088_ReadSensorDMA_START;
// 			context->imu->ASYNC_busy = true;
// 		}
// 		break;}
// 	case ASYNC_BMI088_ReadSensorDMA_START: {
// 		context->txBuf[0] = BMI_GYR_DATA | 0x80;
// 		context->dma_complete = false;
// 		TASK *task = SCHEDULER_add_task(scheduler, ASYNC_SPI_TxRx_DMA, true, (OBJ_POOL*)ASYNC_SPI_TxRx_DMA_POOL);
// 		ASYNC_SPI_TxRx_DMA_init_Gyr_BMI088(context->txBuf, context->rxBuf, 1, BMI088_SPI_LEN);
// 		task->is_done = &(context->dma_complete);
// 		context->state = ASYNC_BMI088_ReadSensorDMA_WAIT_DMA;
// 		break;}
// 	case ASYNC_BMI088_ReadSensorDMA_WAIT_DMA: {
// 		if (context->dma_complete) {
// 			context->state = ASYNC_BMI088_ReadSensorDMA_END;
// 		}
// 		break;}
// 	case ASYNC_BMI088_ReadSensorDMA_END: {
// 		uint8_t *rate_buf = context->rxBuf + 0; // No dummy byte

// 		/* Form signed 16-bit integers */
// 		int16_t gyrX = (int16_t) ((rate_buf[1] << 8) | rate_buf[0]);
// 		int16_t gyrY = (int16_t) ((rate_buf[3] << 8) | rate_buf[2]);
// 		int16_t gyrZ = (int16_t) ((rate_buf[5] << 8) | rate_buf[4]);

// 		/* Convert to rad/s */
// 		context->imu->gyr_rps.x = context->imu->gyrConversion * gyrX - context->imu->gyr_rps_offset.x;
// 		context->imu->gyr_rps.y = context->imu->gyrConversion * gyrY - context->imu->gyr_rps_offset.y;
// 		context->imu->gyr_rps.z = context->imu->gyrConversion * gyrZ - context->imu->gyr_rps_offset.z;

// 		context->imu->new_gyr_data = true;

// 		context->imu->ASYNC_busy = false;
// 		return TASK_RETURN_STOP;
// 		break;}
// 	}
// 	return TASK_RETURN_IDLE;
// }


// TASK_POOL_CREATE(ASYNC_BMI088_Sensor_Publisher);

// void ASYNC_BMI088_Sensor_Publisher_init(TASK *self, BMI088 *imu, DATA_PUB *data_pub, uint32_t delay, size_t buffer_len) {
// 	ASYNC_BMI088_Sensor_Publisher_CONTEXT *context = (ASYNC_BMI088_Sensor_Publisher_CONTEXT*)self->context;

// 	context->imu = imu;
// 	context->data_pub = data_pub;
// 	context->delay = delay;

// 	context->data_buffer = GMS_alloc(&GMS_memory, buffer_len * sizeof(FLOAT3_TIMESTAMP));
// 	DATA_PUB_init(data_pub, context->data_buffer, sizeof(FLOAT3_TIMESTAMP), buffer_len);

// 	context->state = ASYNC_BMI088_Sensor_Publisher_START_Timer;
// }

// TASK_RETURN ASYNC_BMI088_Accelero_Publisher(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_BMI088_Sensor_Publisher_CONTEXT *context = (ASYNC_BMI088_Sensor_Publisher_CONTEXT*)self->context;

// 	if (context->task_complet) {
// 		return TASK_RETURN_STOP;
// 	}

// 	switch (context->state) {
// 	case ASYNC_BMI088_Sensor_Publisher_START_Timer: {
// 		context->timer_start = HAL_GetTick();
// 		context->state = ASYNC_BMI088_Sensor_Publisher_WAIT_Timer;
// 		break;}
// 	case ASYNC_BMI088_Sensor_Publisher_WAIT_Timer: {
// 		if (HAL_GetTick() - context->timer_start >= context->delay) {
// 			context->state = ASYNC_BMI088_Sensor_Publisher_WAIT_BMI088;
// 			context->timer_start = HAL_GetTick();
// 		}
// 		break;}
// 	case ASYNC_BMI088_Sensor_Publisher_WAIT_BMI088: {
// 		if (!(context->imu->ASYNC_busy)) {
// 			context->state = ASYNC_BMI088_Sensor_Publisher_START;
// 			context->imu->ASYNC_busy = true;
// 		}
// 		break;}
// 	case ASYNC_BMI088_Sensor_Publisher_START: {
// 		context->txBuf[0] = BMI_ACC_DATA | 0x80;
// 		TASK *task = SCHEDULER_add_task(scheduler, ASYNC_SPI_TxRx_DMA, true, (OBJ_POOL*)ASYNC_SPI_TxRx_DMA_POOL);
// 		ASYNC_SPI_TxRx_DMA_init_Acc_BMI088(context->txBuf, context->rxBuf, 1, BMI088_SPI_LEN);
// 		task->is_done = &(context->dma_complete);
// 		context->state = ASYNC_BMI088_Sensor_Publisher_WAIT_DMA;
// 		break;}
// 	case ASYNC_BMI088_Sensor_Publisher_WAIT_DMA: {
// 		if (context->dma_complete) {
// 			context->state = ASYNC_BMI088_Sensor_Publisher_END;
// 		}
// 		}	// break is removed intentionally here to allow the next case to execute
// 	case ASYNC_BMI088_Sensor_Publisher_END: {
// 		uint8_t *rate_buf = context->rxBuf + 1; // Skip dummy byte

// 		/* Form signed 16-bit integers */
// 		int16_t accX = (int16_t) ((rate_buf[1] << 8) | rate_buf[0]);
// 		int16_t accY = (int16_t) ((rate_buf[3] << 8) | rate_buf[2]);
// 		int16_t accZ = (int16_t) ((rate_buf[5] << 8) | rate_buf[4]);

// 		/* Convert to m/s^2 and build data struct */
// 		FLOAT3 data = {
// 			.x = context->imu->accConversion * accX - context->imu->acc_mps2_offset.x,
// 			.y = context->imu->accConversion * accY - context->imu->acc_mps2_offset.y,
// 			.z = context->imu->accConversion * accZ - context->imu->acc_mps2_offset.z,
// 		};
// 		FLOAT3_TIMESTAMP data_time = {
// 			.data = data,
// 			.timestamp = HAL_GetTick()
// 		};
// 		DATA_PUB_push(context->data_pub, &data_time);

// 		context->imu->ASYNC_busy = false;
// 		context->state = ASYNC_BMI088_Sensor_Publisher_WAIT_Timer;
// 		break;}
// 	}
// 	return TASK_RETURN_IDLE;
// }

// TASK_RETURN ASYNC_BMI088_Gyro_Publisher(SCHEDULER *scheduler, TASK *self) {
// 	ASYNC_BMI088_Sensor_Publisher_CONTEXT *context = (ASYNC_BMI088_Sensor_Publisher_CONTEXT*)self->context;

// 	if (context->task_complet) {
// 		return TASK_RETURN_STOP;
// 	}

// 	switch (context->state) {
// 	case ASYNC_BMI088_Sensor_Publisher_START_Timer: {
// 		context->timer_start = HAL_GetTick();
// 		context->state = ASYNC_BMI088_Sensor_Publisher_WAIT_Timer;
// 		break;}
// 	case ASYNC_BMI088_Sensor_Publisher_WAIT_Timer: {
// 		if (HAL_GetTick() - context->timer_start >= context->delay) {
// 			context->state = ASYNC_BMI088_Sensor_Publisher_WAIT_BMI088;
// 			context->timer_start = HAL_GetTick();
// 		}
// 		break;}
// 	case ASYNC_BMI088_Sensor_Publisher_WAIT_BMI088: {
// 		if (!(context->imu->ASYNC_busy)) {
// 			context->state = ASYNC_BMI088_Sensor_Publisher_START;
// 			context->imu->ASYNC_busy = true;
// 		}
// 		break;}
// 	case ASYNC_BMI088_Sensor_Publisher_START: {
// 		context->txBuf[0] = BMI_GYR_DATA | 0x80;
// 		context->dma_complete = false;
// 		TASK *task = SCHEDULER_add_task(scheduler, ASYNC_SPI_TxRx_DMA, true, (OBJ_POOL*)ASYNC_SPI_TxRx_DMA_POOL);
// 		ASYNC_SPI_TxRx_DMA_init_Gyr_BMI088(context->txBuf, context->rxBuf, 1, BMI088_SPI_LEN);
// 		task->is_done = &(context->dma_complete);
// 		context->state = ASYNC_BMI088_Sensor_Publisher_WAIT_DMA;
// 		break;}
// 	case ASYNC_BMI088_Sensor_Publisher_WAIT_DMA: {
// 		if (context->dma_complete) {
// 			context->state = ASYNC_BMI088_Sensor_Publisher_END;
// 		}
// 		}	// break is removed, we can continue to END state
// 	case ASYNC_BMI088_Sensor_Publisher_END: {
// 		uint8_t *rate_buf = context->rxBuf + 0; // No dummy byte

// 		/* Form signed 16-bit integers */
// 		int16_t gyrX = (int16_t) ((rate_buf[1] << 8) | rate_buf[0]);
// 		int16_t gyrY = (int16_t) ((rate_buf[3] << 8) | rate_buf[2]);
// 		int16_t gyrZ = (int16_t) ((rate_buf[5] << 8) | rate_buf[4]);

// 		/* Convert to rad/s and build data struct */
// 		FLOAT3 data = {
// 			.x = context->imu->gyrConversion * gyrX - context->imu->gyr_rps_offset.x,
// 			.y = context->imu->gyrConversion * gyrY - context->imu->gyr_rps_offset.y,
// 			.z = context->imu->gyrConversion * gyrZ - context->imu->gyr_rps_offset.z,
// 		};
// 		FLOAT3_TIMESTAMP data_time = {
// 			.data = data,
// 			.timestamp = HAL_GetTick()
// 		};
// 		DATA_PUB_push(context->data_pub, &data_time);

// 		context->imu->ASYNC_busy = false;
// 		context->state = ASYNC_BMI088_Sensor_Publisher_WAIT_Timer;
// 		break;}
// 	}
// 	return TASK_RETURN_IDLE;
// }
