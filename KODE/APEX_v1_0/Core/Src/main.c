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

#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include "peripherals/adc.h"
#include "peripherals/dma.h"
#include "peripherals/gpio.h"
#include "peripherals/i2c.h"
#include "peripherals/spi.h"
#include "peripherals/tim.h"
#include "peripherals/usart.h"
#include "crc.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "scheduler.h"
#include "stm32f4xx_hal.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "project.h"

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
	MX_TIM9_Init();
	MX_TIM11_Init();

	/* USER CODE BEGIN 2 */

	// MX_USB_DEVICE_Init();


#if (APEX_CFG_SCHED_SEQ == 1)
	DRIVERS_CONFIG_init_result_t Drivers_Init_Seq_result;
	DRIVERS_CONFIG_init_seq(&Drivers_Init_Seq_result);
	// Later on, we can implement more specific error handling based on the
	// type of failure. DRIVERS_CONFIG_init_seq could return different error codes for
	// different initialization failures or write on struct saying which module is
	// init and which isn't, etc. For now, we just do a generic error handler.

	setup();

#endif

#if (APEX_CFG_SCHED_RTOS == 1)
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
	MX_FREERTOS_Init();

  Init_spi_semaphores();
  Init_cleanup();

	setup();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
#endif

	while (1) {
#if (APEX_CFG_SCHED_SEQ == 1)
		loop();
#endif
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

// TASK_POOL_ALLOCATE(TASK_Program_start);

// void TASK_Program_start(void *argument) {

// 	// const bmi_config_t bmi_cfg = {
// 	// 	.acc_range  = BMI_ACC_RANGE_24G,
// 	// 	.acc_bwp    = BMI_ACC_CONF_BWP_NORMAL,
// 	// 	.acc_odr    = BMI_ACC_CONF_ODR_100_HZ,
// 	// 	.acc_pwr    = BMI_ACC_PWR_CONF_ACTIVE,
// 	// 	.acc_ctrl   = BMI_ACC_PWR_CTRL_ENABLE,

// 	// 	.gyr_range  = BMI_GYR_RANGE_2000,
// 	// 	.gyr_bw     = BMI_GYR_BANDWIDTH_BW_23_HZ,
// 	// 	.gyr_mode   = BMI_GYR_LPM1_MODE_NORMAL,
// 	// };
// 	// BMI_STATE st;
// 	// StaticEventGroup_t bmi_done_event_cb;
// 	// osEventFlagsId_t bmi_done_event_id = osEventFlagsNew(&(osEventFlagsAttr_t){
// 	// 	.name = "bmi_done_event",
// 	// 	.cb_mem = &bmi_done_event_cb,
// 	// 	.cb_size = sizeof(bmi_done_event_cb)
// 	// });
// 	// osEventFlagsClear(bmi_done_event_id, 0xFFFFFFFF);

// 	// osThreadAttr_t attr_init = {
// 	// 	.name = "TASK_BMI088_Init",
// 	// 	.priority = (osPriority_t)osPriorityNormal,
// 	// };

// 	// TASK_BMI088_Init_ARGS args_init = {
// 	// 	.imu			= &BMI088_imu,
// 	// 	.hspi			= &hspi1,
// 	// 	.cs_acc_bank	= CS_ACC0_GPIO_Port,
// 	// 	.cs_acc_pin		= CS_ACC0_Pin,
// 	// 	.cs_gyr_bank	= CS_GYRO_GPIO_Port,
// 	// 	.cs_gyr_pin		= CS_GYRO_Pin,
// 	// 	.cfg			= &bmi_cfg,
// 	// 	.return_state	= &st,
// 	// 	.done_flags		= bmi_done_event_id,
// 	// };
// 	// OS_THREAD_NEW_CSTM(TASK_BMI088_Init, args_init,  attr_init, osWaitForever);

// 	// osEventFlagsWait(bmi_done_event_id, 1, osFlagsWaitAll, osWaitForever);

	



//   	// osThreadAttr_t bmi_acc_attr = {
// 	// 	.name = "TASK_BMI_ACC",
// 	// 	.priority = (osPriority_t)osPriorityNormal,
// 	// };

// 	// TASK_BMI088_ReadAcc_ARGS bmi_acc_args = {
// 	// 	.imu			= &BMI088_imu,
// 	// 	.dt				= &data_topic_acc_ptr,
// 	// 	.return_state	= &st,
// 	// };

// 	// OS_THREAD_NEW_CSTM(TASK_BMI088_ReadAcc, bmi_acc_args, bmi_acc_attr, osWaitForever);

	


	
// 	// osThreadAttr_t bmi_gyr_attr = {
// 	// 	.name = "TASK_BMI_GYR",
// 	// 	.priority = (osPriority_t)osPriorityNormal,
// 	// };

// 	// TASK_BMI088_ReadGyr_ARGS bmi_gyr_args = {
// 	// 	.imu			= &BMI088_imu,
// 	// 	.dt				= &data_topic_gyr_ptr,
// 	// 	.return_state	= &st,
// 	// };

// 	// OS_THREAD_NEW_CSTM(TASK_BMI088_ReadGyr, bmi_gyr_args, bmi_gyr_attr, osWaitForever);

	


	
// 	// TASK_Data_USB_Transmit_ARGS usb_args = {
// 	// 	.dt = &data_topic_acc_ptr,
// 	// 	.delay = 10,
// 	// };
// 	// osThreadAttr_t usb_attr = {
// 	// 	.name = "TASK_Data_USB_Transmit",
// 	// 	.priority = (osPriority_t)osPriorityNormal,
// 	// };
// 	// OS_THREAD_NEW_CSTM(TASK_Data_USB_Transmit, usb_args, usb_attr, osWaitForever);

	
// 	TASK_sx127x_Init_TxRx_ARGS args = {
// 		.chip = &sx127x_1,
// 	};
// 	osThreadAttr_t sx127x_attr = {
// 		.name = "TASK_sx127x_Init_TxRx",
// 		.priority = (osPriority_t)osPriorityNormal,
// 	};

// 	OS_THREAD_NEW_CSTM(TASK_sx127x_Init_TxRx, args, sx127x_attr, osWaitForever);


//   	// osThreadExit_Cstm();
// 	for (;;) {
// 		osDelay(osWaitForever);
// 	}
// }

// TASK_POOL_ALLOCATE(TASK_Data_USB_Transmit);

// void TASK_Data_USB_Transmit(void *argument) {
// 	TASK_Data_USB_Transmit_ARGS *args = (TASK_Data_USB_Transmit_ARGS *)argument;

// 	data_topic_t **dt = args->dt;
// 	uint32_t delay = args->delay;

// 	char buffer[64];
// 	char buffer_status[9];
// 	char buffer_x[10];
// 	char buffer_y[10];
// 	char buffer_z[10];
// 	float3_t data;

// 	while (*dt == NULL) {
// 		osDelay(10);
// 	}
// 	data_sub_t sub = { 0 };
// 	data_sub_attach(&sub, *dt, DATA_ATTACH_FROM_NOW);

// 	osThreadAttr_t usb_attr = {
// 		.name = "TASK_USB_Transmit",
// 		.priority = (osPriority_t)osPriorityNormal,
// 	};
// 	TASK_USB_Transmit_ARGS usb_args = {
// 		.buff = (uint8_t*)buffer,
// 		.len = 0,
// 	};

// 	for (;;) {
// 		data_sub_wait_for_data(&sub, osWaitForever);
// 		data_status_t status = data_sub_read(&sub, &data);
// 		switch (status) {
// 		case DT_OK:
// 			strcpy(buffer_status, "OK");
// 			break;
// 		case DT_EMPTY:
// 			strcpy(buffer_status, "EMPTY");
// 			break;
// 		case DT_DATA_LOSS:
// 			strcpy(buffer_status, "LOSS");
// 			break;
// 		case DT_FULL:
// 			strcpy(buffer_status, "FULL");
// 			break;
// 		case DT_BAD_ARG:
// 			strcpy(buffer_status, "BAD_ARG");
// 			break;
// 		}
// 		if (status == DT_OK || status == DT_DATA_LOSS) {
// 			float_format(buffer_x, data.x, 4, 10);
// 			float_format(buffer_y, data.y, 4, 10);
// 			float_format(buffer_z, data.z, 4, 10);
// 		} else {
// 			float_format(buffer_x, +9999.9999, 4, 10);
// 			float_format(buffer_y, +9999.9999, 4, 10);
// 			float_format(buffer_z, +9999.9999, 4, 10);
// 		}


// 		// Format data into CSV string
// 		strcpy(buffer, "Status: ");
// 		strcat(buffer, buffer_status);
// 		strcat(buffer, ", Acc : ");
// 		strcat(buffer, buffer_x);
// 		strcat(buffer, ", ");
// 		strcat(buffer, buffer_y);
// 		strcat(buffer, ", ");
// 		strcat(buffer, buffer_z);
// 		strcat(buffer, "\n");

// 		// // Transmit data over USB
// 		usb_args.len = strlen(buffer);
// 		OS_THREAD_NEW_CSTM(TASK_USB_Transmit, usb_args, usb_attr, osWaitForever);

// 		osDelay(delay); // Adjust delay as needed
// 	}
// }



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

  HAL_GPIO_TogglePin(LED0B_GPIO_Port, LED0B_Pin);
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
	while (1);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
