#ifndef BMI088_IMU_H
#define BMI088_IMU_H

#include "FreeRTOSConfig.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "peripherals/spi.h"

#include "utils/data_topic.h"
#include "utils/types.h"
#include "utils/scheduler.h"

/* Register defines */
#define BMI_ACC_CHIP_ID 		0x00
#define BMI_ACC_DATA 			0x12
#define BMI_TEMP_DATA 			0x22
#define BMI_ACC_CONF 			0x40
#define BMI_ACC_RANGE 			0x41
#define BMI_INT1_IO_CONF 	   	0x53
#define BMI_INT1_INT2_MAP_DATA 	0x58
#define BMI_ACC_SELF_TEST 		0x6D
#define BMI_ACC_PWR_CONF 		0x7C
#define BMI_ACC_PWR_CTRL 		0x7D
#define BMI_ACC_SOFTRESET 		0x7E

#define BMI_ACC_BWP_OSR4		0x80
#define BMI_ACC_BWP_OSR2		0x90
#define BMI_ACC_BWP_NORMAL		0xA0 

#define BMI_ACC_ODR_12_5		0x05
#define BMI_ACC_ODR_25			0x06
#define BMI_ACC_ODR_50			0x07
#define BMI_ACC_ODR_100			0x08
#define BMI_ACC_ODR_200			0x09
#define BMI_ACC_ODR_400			0x0A
#define BMI_ACC_ODR_800			0x0B
#define BMI_ACC_ODR_1600		0x0C

#define BMI_ACC_RANGE_3G		0x00
#define BMI_ACC_RANGE_6G		0x01
#define BMI_ACC_RANGE_12G		0x02
#define BMI_ACC_RANGE_24G		0x03

#define BMI_ACC_NO_SELF_TEST	0x00
#define BMI_ACC_P_SELF_TEST		0x0D
#define BMI_ACC_N_SELF_TEST		0x09


#define BMI_GYR_CHIP_ID			0x00
#define BMI_GYR_DATA			0x02
#define	BMI_GYR_RANGE			0x0F
#define	BMI_GYR_BANDWIDTH		0x10
#define	BMI_GYR_SOFTRESET		0x14
#define	BMI_GYR_INT_CTRL		0x15
#define	BMI_INT3_INT4_IO_CONF	0x16
#define BMI_INT3_INT4_IO_MAP	0x18
#define BMI_GYR_SELF_TEST		0x3C

#define BMI_GYR_RANGE_2000		0x00
#define BMI_GYR_RANGE_1000		0x01
#define BMI_GYR_RANGE_500		0x02
#define BMI_GYR_RANGE_250		0x03
#define BMI_GYR_RANGE_125		0x04

#define BMI_GYR_BANDWIDTH_512	0x00
#define BMI_GYR_BANDWIDTH_230	0x01
#define BMI_GYR_BANDWIDTH_116	0x02
#define BMI_GYR_BANDWIDTH_47	0x03
#define BMI_GYR_BANDWIDTH_23	0x04
#define BMI_GYR_BANDWIDTH_12	0x05
#define BMI_GYR_BANDWIDTH_64	0x06
#define BMI_GYR_BANDWIDTH_32	0x07

#define BMI_GYR_ST_B_TRIGGER	0x00
#define BMI_GYR_ST_B_READY		0x01
#define BMI_GYR_ST_B_FAILED		0x02
#define BMI_GYR_ST_B_RATE_OK	0x04

typedef struct {

	/* SPI */
	// SPI_HandleTypeDef_flag *spiHandle_flag;
	SPI_HandleTypeDef	   *spiHandle;
	GPIO_TypeDef 	  	   *csAccPinBank;
	GPIO_TypeDef 	  	   *csGyrPinBank;
	uint16_t 		  	    csAccPin;
	uint16_t 		  	    csGyrPin;

	/* DMA */
	// bool use_acc_spi;
	// bool use_gyr_spi;
	// bool *dma_complete;

	/* Conversion constants (raw to m/s^2 and raw to rad/s) */
	float accConversion;
	float gyrConversion;

	/* x-y-z measurements */
	FLOAT3 acc_mps2;
	FLOAT3 gyr_rps;

	FLOAT3 acc_mps2_offset;
	FLOAT3 gyr_rps_offset;

	bool new_acc_data;
	bool new_gyr_data;

	bool ASYNC_busy;
} BMI088;

typedef struct BMI088_OffsetData {
	FLOAT3 avg;
	FLOAT3 std;
} BMI088_OffsetData;


#define BMI088_SPI_LEN 7



/*
	Macro that call the ASYNC_SPI_TxRx_DMA_init function with the BMI088 ACC's parameters
	It requires:
		- SCHEUDLER struct named "scheduler"
		- TASK struct named "task"
		- ASYNC_BMI088_..._CONTEXT struct named "context" witch is the context of the task
	
	... to be defined in the current scope.

	Parameters:
		- (uint8_t*)txBuf: the buffer to transmit
		- (uint8_t*)rxBuf: the buffer to receive
		- (size_t)txLen: the length of the buffer to transmit
		- (size_t)rxLen: the length of the buffer to receive
*/
#define ASYNC_SPI_TxRx_DMA_init_Acc_BMI088(txBuf, rxBuf, txLen, rxLen) \
	ASYNC_SPI_TxRx_DMA_init(task, \
		                    context->imu->spiHandle, \
							context->imu->csAccPinBank, context->imu->csAccPin, \
							txBuf, rxBuf, txLen, rxLen, self);

/*
	Macro that call the ASYNC_SPI_TxRx_DMA_init function with the BMI088 GYR's parameters
	It requires:
		- SCHEUDLER struct named "scheduler"
		- TASK struct named "task"
		- ASYNC_BMI088_..._CONTEXT struct named "context" witch is the context of the task
	
	... to be defined in the current scope.

	Parameters:
		- (uint8_t*)txBuf: the buffer to transmit
		- (uint8_t*)rxBuf: the buffer to receive
		- (size_t)txLen: the length of the buffer to transmit
		- (size_t)rxLen: the length of the buffer to receive
*/
#define ASYNC_SPI_TxRx_DMA_init_Gyr_BMI088(txBuf, rxBuf, txLen, rxLen) \
	ASYNC_SPI_TxRx_DMA_init(task, \
		                    context->imu->spiHandle, \
							context->imu->csGyrPinBank, context->imu->csGyrPin, \
							txBuf, rxBuf, txLen, rxLen, self);




uint8_t BMI088_Init(BMI088 *imu,
				    SPI_HandleTypeDef *spiHandle,
				    GPIO_TypeDef *csAccPinBank, uint16_t csAccPin,
				    GPIO_TypeDef *csGyrPinBank, uint16_t csGyrPin);

uint8_t BMI088_ReadAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data);
uint8_t BMI088_ReadGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data);

uint8_t BMI088_WriteAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t data);
uint8_t BMI088_WriteGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t data);

uint8_t BMI088_ReadAccelerometer(BMI088 *imu);
uint8_t BMI088_ReadGyroscope(BMI088 *imu);

bool BMI088_SelfTestAccelerometer(BMI088 *imu);
bool BMI088_SelfTestGyroscope(BMI088 *imu);

BMI088_OffsetData BMI088_OffsetAccelerometer(BMI088 *imu);
BMI088_OffsetData BMI088_OffsetGyroscope(BMI088 *imu);



typedef struct TASK_BMI088_ReadAcc_ARGS {
	BMI088 *imu;
	// DATA_PUB *data_pub;
	uint32_t delay;

	uint8_t *data_buffer;

	uint32_t timer_start;
	uint32_t timer_delay;
} TASK_BMI088_ReadAcc_ARGS;

TASK_POOL_CONFIGURE(TASK_BMI088_ReadAcc, 10, 512);

void TASK_BMI088_ReadAcc(void *argument);




typedef struct TASK_BMI088_ReadGyr_ARGS {
  BMI088 *imu;
//   DATA_PUB *data_pub;
  uint32_t delay;

  uint8_t *data_buffer;

  uint32_t timer_start;
  uint32_t timer_delay;
} TASK_BMI088_ReadGyr_ARGS;

TASK_POOL_CONFIGURE(TASK_BMI088_ReadGyr, 10, 512);

void TASK_BMI088_ReadGyr(void *argument);





// #define ASYNC_BMI088_ReadSensorDMA_NUMBER 10

// typedef enum ASYNC_BMI088_ReadSensorDMA_State {
// 	ASYNC_BMI088_ReadSensorDMA_WAIT_BMI088,
// 	ASYNC_BMI088_ReadSensorDMA_START,
// 	ASYNC_BMI088_ReadSensorDMA_WAIT_DMA,
// 	ASYNC_BMI088_ReadSensorDMA_END,
// } ASYNC_BMI088_ReadSensorDMA_State;

// typedef struct ASYNC_BMI088_ReadSensorDMA_CONTEXT {
// 	BMI088 *imu;

// 	ASYNC_BMI088_ReadSensorDMA_State state;
// 	bool dma_complete;

// 	uint8_t txBuf[BMI088_SPI_LEN];
// 	uint8_t rxBuf[BMI088_SPI_LEN];

// } ASYNC_BMI088_ReadSensorDMA_CONTEXT;

// extern TASK_POOL_CREATE(ASYNC_BMI088_ReadSensorDMA);

// void ASYNC_BMI088_ReadSensorDMA_init(TASK *self, BMI088 *imu);
// TASK_RETURN ASYNC_BMI088_ReadAccelerometerDMA(SCHEDULER *scheduler, TASK *self);
// TASK_RETURN ASYNC_BMI088_ReadGyroscopeDMA(SCHEDULER *scheduler, TASK *self);


// #define ASYNC_BMI088_Sensor_Publisher_NUMBER 10

// typedef enum ASYNC_BMI088_Sensor_Publisher_State {
// 	ASYNC_BMI088_Sensor_Publisher_START_Timer,
// 	ASYNC_BMI088_Sensor_Publisher_WAIT_Timer,
// 	ASYNC_BMI088_Sensor_Publisher_WAIT_BMI088,
// 	ASYNC_BMI088_Sensor_Publisher_START,
// 	ASYNC_BMI088_Sensor_Publisher_WAIT_DMA,
// 	ASYNC_BMI088_Sensor_Publisher_END,
// } ASYNC_BMI088_Sensor_Publisher_State;

// typedef struct ASYNC_BMI088_Sensor_Publisher_CONTEXT {
// 	BMI088 *imu;

// 	uint8_t txBuf[BMI088_SPI_LEN];
// 	uint8_t rxBuf[BMI088_SPI_LEN];

// 	ASYNC_BMI088_Sensor_Publisher_State state;
// 	bool dma_complete;
// 	bool task_complet;

// 	DATA_PUB *data_pub;
// 	uint8_t *data_buffer;

// 	uint32_t timer_start;
// 	uint32_t delay;
// } ASYNC_BMI088_Sensor_Publisher_CONTEXT;

// extern TASK_POOL_CREATE(ASYNC_BMI088_Sensor_Publisher);

// void ASYNC_BMI088_Sensor_Publisher_init(TASK *self, BMI088 *imu, DATA_PUB *data_pub, uint32_t delay, size_t buffer_len);
// TASK_RETURN ASYNC_BMI088_Accelero_Publisher(SCHEDULER *scheduler, TASK *self);
// TASK_RETURN ASYNC_BMI088_Gyro_Publisher(SCHEDULER *scheduler, TASK *self);

#endif