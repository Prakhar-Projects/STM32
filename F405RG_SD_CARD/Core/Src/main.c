/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "string.h"
#include <stdio.h>
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
FRESULT fresult;
char log_path[] = "/RAW.TXT";
char msg[] = "Hello SDIO!!\r\n";
UINT testByte;
FATFS fs;
FIL myFILE;
FATFS fs;
FATFS *pfs;
FIL fil;
DWORD fre_clust;
uint32_t total, free_space;
char buffer[1024];
UINT br,bw;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_tx;
DMA_HandleTypeDef hdma_sdio_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_CDC(char *string) {
    uint8_t len = strlen(string);
    CDC_Transmit_FS((uint8_t*)string, len);
}
/* to find the size of the buffer*/

int bufsize (char *buf){
	int i=0;
	while (*buf++ != '\0') i ++;
	return i;
}

void bufclear (void) {// clear buffer
for (int i = 0; i < 1024; i++) {
	buffer[i]='\0';
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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  if ( BSP_SD_Init() != MSD_OK ) {
       return FR_NOT_READY;
     }
     // Re-initialize FATFS
     if ( FATFS_UnLinkDriver(SDPath) != 0 ) {
       return FR_NOT_READY;
     }
     if ( FATFS_LinkDriver(&SD_Driver, SDPath) != 0 ) {
       return FR_NOT_READY;
     }
  fresult = f_mount(&fs, SDPath, 1);
  if (fresult != FR_OK) {
      // Handle SD card mount error
      f_mount(0, SDPath, 0);
      return fresult;
  }
  f_getfree(SDPath, &fre_clust, &pfs);
      	    if (fresult != FR_OK) {
      	    	send_CDC("SD card MOUNT FAILED...\r\n");
      	} else {

      	    /* Calculate total space only after mounting */
      	    total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);

      	   /* Print total space */
      	    sprintf(buffer, "Total SD card space: %lu \r\n", total);
      	    send_CDC(buffer);
      	    bufclear();
      	}
      	  if (fresult != FR_OK) {
      		send_CDC("SD card MOUNT FAILED...\r\n");
      	  }else if(fresult == FR_OK){
      	   /* Calculate available space */
      	    free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);

      	   /* Print available space */
      	    sprintf(buffer, "Available Space: %lu \r\n", free_space);
      	    send_CDC(buffer);
      	    bufclear();

      	  }
      	 //open a file to write, if it doesn't exist, it creates.
      	  fresult = f_open(&fil, "ADXL1_DATA.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

      	  //Write the text
      	  fresult = f_puts("ADXL1_DATA HERE!\r\n", &fil);

      	  // close file
      	  fresult = f_close(&fil);
      	  send_CDC("ADXL1_DATA.txt created & data is written\r\n");

      	  //open the file to read
      	  fresult = f_open(&fil,"ADXL1_DATA.txt", FA_READ);

    	  // Read the text from the file
      	  f_gets(buffer,sizeof(buffer),&fil);

      	  send_CDC(buffer);

      	  //close file
      	  f_close(&fil);
      	  bufclear();

      	//opening another file
            	fresult = f_open(&fil, "ADXL2_DATA.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

            	// using f_write to write inside this file
            	strcpy(buffer,"ADXL2_DATA WILL be stored here\r\n");

            	fresult = f_write(&fil, buffer,bufsize(buffer), &bw);

            	send_CDC("ADXL2_DATA data will be written here.\r\n");

            	//close file
            	f_close(&fil);

            	//clearing buffer to empty it before reading data
             	bufclear();

            	// open 2nd file to read
            	fresult = f_open(&fil,"ADXL2_DATA.txt", FA_READ);
            	f_read(&fil, buffer, sizeof(buffer), &br);
            	send_CDC(buffer);

            	//Close file
            	f_close(&fil);
            	bufclear();

            	// Update a file
            	fresult = f_open(&fil, "ADXL2_DATA.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
            	// Move offset toward the end of the line using the size of the file
            	fresult = f_lseek(&fil, f_size(&fil));

            	fresult= f_puts("UPDATE 1 to ADXL2_DATA",&fil);

            	//close file
            	f_close(&fil);
            	bufclear();
            	fresult = f_open(&fil,"ADXL2_DATA.txt", FA_READ);

              f_read(&fil, buffer, sizeof(buffer), &br);

            	send_CDC(buffer);

            	 //Close file
            	 f_close(&fil);
            	 bufclear();
  /* USER CODE END 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
