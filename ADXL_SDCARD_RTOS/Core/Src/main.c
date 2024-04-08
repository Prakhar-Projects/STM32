/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include <stdarg.h>
#include "string.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SampleSize 2000
uint32_t adcVal[2];
uint32_t counter; // Counter for collected samples
double XanalogValues1;
double XanalogValues2;
FATFS fs;
FATFS *pfs;
FIL fil, fil2; // Declarations for both files
FRESULT fresult;
DWORD fre_clust;
uint32_t total, free_space;
UINT br,bw; 	//	file read/write count
char buffer[BufferSize];


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void bufclear (void) {// clear buffer
		for (int i = 0; i < 1024; i++) {
			buffer[i]='\0';
		}
		}
QueueHandle_t ADC_Semaphore;


xTaskHandle ADC1_Task_Handler;
xTaskHandle ADC2_Task_Handler;
xTaskHandle SDCARD_Task_Handler;


void ADC1_TASK (void *argument)
{
	while (1)
	{
		if (xSemaphoreTake(ADC_Semaphore, portMAX_DELAY) == pdTRUE)
		{
			 counter1++;
			 if (counter1<=SampleSize)
			 {
				 ADC_ChannelConfTypeDef sConfig = {0};
				 int Sensor1Values;
				 uint8_t str[50]="";
				 sConfig.Channel = ADC_CHANNEL_0;
				  sConfig.Rank = 1;
				  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
				  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
				  {
				    Error_Handler();
				  }
				  HAL_ADC_Start(&hadc1);
				  HAL_ADC_PollForConversion(&hadc1, 100);
				  Sensor1Values=(int)HAL_ADC_GetValue(&hadc1);
				  HAL_ADC_Stop(&hadc1);
				  sprintf(str, "\r\nSensor1 Value[%lu] = %d\r\n",(unsigned long)counter1,Sensor1Values);
				  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 200);
			 }else{
				 break;
			 }
			xSemaphoreGive(ADC_Semaphore); // Release the semaphore
		}
		vTaskDelay(500);
	}
}
void ADC2_TASK (void *argument)
{
	while (1)
	{
		if (xSemaphoreTake(ADC_Semaphore, portMAX_DELAY) == pdTRUE)
		{
			counter2++;
			if (counter1<=SampleSize)
			{
				ADC_ChannelConfTypeDef sConfig = {0};
				int Sensor1Values;
				uint8_t str[50]="";
				sConfig.Channel = ADC_CHANNEL_1;
				sConfig.Rank = 1;
				sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
				if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
				{
					Error_Handler();
				}
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 100);
				Sensor2Values=(int)HAL_ADC_GetValue(&hadc1);
				HAL_ADC_Stop(&hadc1);
				sprintf(str, "\r\nSensor1 Value[%lu] = %d\r\n",(unsigned long)counter2,Sensor2Values);
				HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 200);
			}else{
				break;
			}
			xSemaphoreGive(ADC_Semaphore); // Release the semaphore
		  }
		vTaskDelay(500);
	}
}
void SDCARD_TASK (void *argument)
{
	while(1)
	{
		if (xSemaphoreTake(ADC_Semaphore, portMAX_DELAY) == pdTRUE) {
		/* Mount SD Card  */
	fresult = f_mount(&fs, "", 0);
		if (fresult != FR_OK) {
	    send_uart("SD card MOUNT FAILED...\r\n");
		} else {
			send_uart("SD card MOUNTED...\r\n");
		}
	      // Open the file for writing (for sensor 1)
			fresult = f_open(&fil, "ADXL1_DATA.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	      if (fresult == FR_OK)
	      {
	        // Position the file pointer at the end of the file (append mode)
	        fresult = f_lseek(&fil, f_size(&fil));
	         snprintf(buffer, sizeof(buffer), "%.d\r\n", Sensor1Values); // 2 decimal places
	        // Write the string representation of the double value to the file
	        fresult = f_puts(buffer, &fil);
	        // Close the file when you're done writing
	        fresult = f_close(&fil);
	      }
	      // Open the file for writing (for sensor 2)
	    	fresult = f_open(&fil2, "ADXL2_DATA.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	    	   if (fresult == FR_OK)
	    	   {
	    	   // Position the file pointer at the end of the file (append mode)
	    		fresult = f_lseek(&fil2, f_size(&fil2));
	    	    snprintf(buffer, sizeof(buffer), "%.2f\r\n", XanalogValues2); // 2 decimal places
	    	   // Write the string representation of the double value to the file
	    	    fresult = f_puts(buffer, &fil2);
	    	   // Close the file when you're done writing
	    	    fresult = f_close(&fil2);
	    	   }
	            xSemaphoreGive(ADC_Semaphore);
	        }
	    	   vTaskDelay(500);
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  ADC_Semaphore = xSemaphoreCreateBinary();
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  fresult = f_mount(&fs, "", 1); // 1 = mount with format
  if (fresult == FR_OK) {
      send_uart("SD card FORMATTED and MOUNTED...\r\n");

      // Open the file for writing (for sensor 1)
      fresult = f_open(&fil, "ADXL1_DATA.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
      if (fresult == FR_OK) {
          send_uart("ADXL1.txt has been created!\r\n");
          f_close(&fil);
      } else {
          send_uart("Error opening ADXL1_DATA.txt...\r\n");
      }

      // Open the file for writing (for sensor 2)
      fresult = f_open(&fil2, "ADXL2_DATA.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
      if (fresult == FR_OK) {
          send_uart("ADXL2.txt has been created!\r\n");
          f_close(&fil2);
      } else {
          send_uart("Error opening ADXL2_DATA.txt...\r\n");
      }
  } else {
      send_uart("SD card FORMAT and MOUNT FAILED...\r\n");
  }
  // Unmount the SD card
  fresult = f_mount(NULL, "", 0);
  if (fresult == FR_OK) {
      send_uart("SD card UNMOUNTED...\r\n");
  } else {
      send_uart("Error unmounting SD card...\r\n");
  }

  xTaskCreate(ADC1_TASK, "ADC1", 128, NULL, 1, &ADC1_Task_Handler);
  xTaskCreate(ADC2_TASK, "ADC2", 128, NULL, 2, &ADC2_Task_Handler);
  xTaskCreate(SDCARD_TASK, "SDCARD", 128, NULL, 3, &SDCARD_Task_Handler);


   vTaskStartScheduler();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
