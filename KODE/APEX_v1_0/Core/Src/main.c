/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"
#include "cmsis_gcc.h"
#include "cmsis_os2.h"
#include "crc.h"

#include <stdint.h>
#include <string.h>

#include "drivers/ADXL375.h"
#include "drivers/BMI088.h"
#include "drivers/led.h"
#include "drivers/w25q_mem.h"

#include "peripherals/adc.h"
#include "peripherals/dma.h"
#include "peripherals/gpio.h"
#include "peripherals/i2c.h"
#include "peripherals/spi.h"
#include "peripherals/tim.h"
#include "peripherals/usart.h"

#include "stm32f4xx_hal_conf.h"
#include "utils/data_topic.h"
#include "utils/scheduler.h"
#include "utils/tools.h"
#include "utils/usb.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_ll_adc.h"
#include "usb_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
const char TASK_Program_start_name[19] = "TASK_Program_start";
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
BMI088 BMI088_imu;
W25Q_Chip w25q_chip;

data_topic_t *data_topic_acc_ptr;
data_topic_t *data_topic_gyr_ptr;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C3_Init();
	MX_SPI2_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_TIM3_Init();
	MX_CRC_Init();
	MX_TIM11_Init();
	/* USER CODE BEGIN 2 */

	// MX_USB_DEVICE_Init();


	BMI088_Init(&BMI088_imu, &hspi1, CS_ACC0_GPIO_Port, CS_ACC0_Pin,
				CS_GRYO_GPIO_Port, CS_GRYO_Pin);

	W25Q_STATE state;
	state = W25Q_Init(&w25q_chip, &hspi2, CS_FLASH_GPIO_Port, CS_FLASH_Pin);
	assert_param(state == W25Q_OK);
	// W25Q_ReadWriteTest(&w25q_chip);



	
	TASK_POOL_CREATE(TASK_Program_start);

	TASK_POOL_CREATE(TASK_BMI088_ReadAcc);
	TASK_POOL_CREATE(TASK_BMI088_ReadGyr);

	TASK_POOL_CREATE(TASK_W25Q_SendCmd);
	TASK_POOL_CREATE(TASK_W25Q_SendCmdAddr);
	TASK_POOL_CREATE(TASK_W25Q_Init);
	TASK_POOL_CREATE(TASK_W25Q_WriteData);
	TASK_POOL_CREATE(TASK_W25Q_ReadData);
	TASK_POOL_CREATE(TASK_W25Q_ReadWriteTest);

	TASK_POOL_CREATE(TASK_USB_Transmit);
	TASK_POOL_CREATE(TASK_Data_USB_Transmit);

	Init_cleanup();
	Init_spi_semaphores();
	USB_Init();

	// osThreadAttr_t attr = {
	// 	.name = TASK_Program_start_name,
	// };
	// OS_THREAD_NEW_CSTM(TASK_Program_start, (TASK_Program_start_ARGS) {}, attr, osWaitForever);

	osThreadAttr_t attr = {
		.name = "TASK_W25Q_ReadWriteTest",
		.priority = (osPriority_t)osPriorityNormal,
	};
	TASK_W25Q_ReadWriteTest_ARGS rwtest_args = {
		.chip = &w25q_chip,
	};
	OS_THREAD_NEW_CSTM(TASK_W25Q_ReadWriteTest, rwtest_args, attr, osWaitForever);

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// const char TASK_Program_start_name[19] = "TASK_Program_start";

TASK_POOL_ALLOCATE(TASK_Program_start);

void TASK_Program_start(void *argument) {

  	osThreadAttr_t attr0 = {
		.name = "TASK_BMI088_ReadAcc",
		.priority = (osPriority_t)osPriorityNormal,
	};

	data_topic_acc_ptr = NULL;

  	TASK_BMI088_ReadAcc_ARGS args0 = {
		.imu = &BMI088_imu,
		.delay = 100,        // 100 ms delay
		.dt = &data_topic_acc_ptr,
		.timer_start = 0,
		.timer_delay = 0,
  	};
	OS_THREAD_NEW_CSTM(TASK_BMI088_ReadAcc, args0, attr0, osWaitForever);

	osThreadAttr_t attr1 = {
  		.name = "TASK_BMI088_ReadGyr",
		.priority = (osPriority_t)osPriorityNormal,
	};

	data_topic_gyr_ptr = NULL;

  	TASK_BMI088_ReadGyr_ARGS args1 = {
		.imu = &BMI088_imu,
		.delay = 1000,        // 100 ms delay
		.dt = &data_topic_gyr_ptr,
		.timer_start = 0,
		.timer_delay = 0,
  	};
  	OS_THREAD_NEW_CSTM(TASK_BMI088_ReadGyr, args1, attr1, osWaitForever);


	TASK_Data_USB_Transmit_ARGS usb_args = {
		.dt = &data_topic_acc_ptr,
		.delay = 200,
	};
	osThreadAttr_t usb_attr = {
		.name = "TASK_Data_USB_Transmit",
		.priority = (osPriority_t)osPriorityNormal,
	};
	OS_THREAD_NEW_CSTM(TASK_Data_USB_Transmit, usb_args, usb_attr, osWaitForever);


  	osThreadExit_Cstm();
	// for (;;) {
	// 	osDelay(osWaitForever);
	// }
}

TASK_POOL_ALLOCATE(TASK_Data_USB_Transmit);

void TASK_Data_USB_Transmit(void *argument) {
	TASK_Data_USB_Transmit_ARGS *args = (TASK_Data_USB_Transmit_ARGS *)argument;

	data_topic_t **dt = args->dt;
	uint32_t delay = args->delay;

	char buffer[64];
	char buffer_status[9];
	char buffer_x[10];
	char buffer_y[10];
	char buffer_z[10];
	FLOAT3 data;

	while (*dt == NULL) {
		osDelay(10);
	}
	data_sub_t sub = { 0 };
	data_sub_attach(&sub, *dt, DATA_ATTACH_FROM_NOW);

	osThreadAttr_t usb_attr = {
		.name = "TASK_USB_Transmit",
		.priority = (osPriority_t)osPriorityNormal,
	};
	TASK_USB_Transmit_ARGS usb_args = {
		.buff = (uint8_t*)buffer,
		.len = 0,
	};

	for (;;) {
		data_sub_wait_for_data(&sub, osWaitForever);
		data_status_t status = data_sub_read(&sub, &data);
		switch (status) {
		case DT_OK:
			strcpy(buffer_status, "OK");
			break;
		case DT_EMPTY:
			strcpy(buffer_status, "EMPTY");
			break;
		case DT_DATA_LOSS:
			strcpy(buffer_status, "LOSS");
			break;
		case DT_FULL:
			strcpy(buffer_status, "FULL");
			break;
		case DT_BAD_ARG:
			strcpy(buffer_status, "BAD_ARG");
			break;
		}
		if (status == DT_OK || status == DT_DATA_LOSS) {
			float_format(buffer_x, data.x, 4, 10);
			float_format(buffer_y, data.y, 4, 10);
			float_format(buffer_z, data.z, 4, 10);
		} else {
			float_format(buffer_x, +9999.9999, 4, 10);
			float_format(buffer_y, +9999.9999, 4, 10);
			float_format(buffer_z, +9999.9999, 4, 10);
		}


		// Format data into CSV string
		strcpy(buffer, "Status: ");
		strcat(buffer, buffer_status);
		strcat(buffer, ", Acc : ");
		strcat(buffer, buffer_x);
		strcat(buffer, ", ");
		strcat(buffer, buffer_y);
		strcat(buffer, ", ");
		strcat(buffer, buffer_z);
		strcat(buffer, "\n");

		// // Transmit data over USB
		usb_args.len = strlen(buffer);
		OS_THREAD_NEW_CSTM(TASK_USB_Transmit, usb_args, usb_attr, osWaitForever);

		osDelay(delay); // Adjust delay as needed
	}
}



/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
