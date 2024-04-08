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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "fatfs_sd.h"
#include "string.h"
#define ARM_MATH_CM4
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SampleSize 4096

#define BufferSize 2048

uint32_t adcVal[2];
uint32_t counter =0; // Counter for collected samples
double Sensor1Values;
double Sensor2Values;
double buffer[BufferSize];


char read_flag;

FATFS fs;
FATFS *pfs;
FIL fil,fil2,filOut,filOut2; // Declarations for all files
FRESULT fresult;

int fftbufferIndex = 0;
#define FFT_BUFFER_SIZE 2048
#define SAMPLING_RATE 1000
arm_rfft_fast_instance_f32 fftHandler;
float32_t fftBuffIn[FFT_BUFFER_SIZE];
float32_t fftBuffOut[FFT_BUFFER_SIZE];
float32_t outputfft_mag[FFT_BUFFER_SIZE/2];
uint8_t fftFlag=0;

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

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void unlinkFile(const char *fileName) {
 FRESULT  fresult = f_unlink(fileName);
    if (fresult != FR_OK) {
    	send_uart("%s not found\r\n", fileName);
    }
}
double calculateAverage(const double *values, int size) {
    if (size <= 0) {
        return 0.0;  // Return 0 for an empty array or invalid size
    }

    double sum = 0.0;

    for (int i = 0; i < size; i++) {
        sum += values[i];
    }

    return sum / size;
}
void writeDoubleToFile(FIL* file, double value)
{
    FRESULT fresult;

    // Move the file pointer to the end of the file
    fresult = f_lseek(file, f_size(file));
    if (fresult != FR_OK)
    {
        return;
    }
    char buffer[20];  // Adjust the buffer size as needed
    snprintf(buffer, sizeof(buffer), "%.2f\n", value);

    // Write the string representation of the double value to the file
    fresult = f_puts(buffer, file);
    if (fresult != FR_OK)
    {
        // Handle error
        // You may want to add error handling specific to your application
        return;
    }
}
// Function to convert a double to a string
void ADC_Select_CH0(void){
	ADC_ChannelConfTypeDef sConfig = {0};

/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
*/
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
 {
   Error_Handler();
 }
}

void ADC_Select_CH1(void){
	ADC_ChannelConfTypeDef sConfig = {0};

/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
*/
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
 {
   Error_Handler();
 }
}

void send_uart(const char *format, ...) {
    va_list args;
    va_start(args, format);
    char buffer[1024];  // Assuming a maximum string length of 255 characters
    // Use vsnprintf to format the string and store it in the buffer
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    if (len >= 0) {
       HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 200);
    }
    va_end(args);
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
void processAndSaveData(const char *inputFileName, const char *outputFileName, double *buffer, int bufferSize, double *totalAverage) {
    FRESULT fresult;
    FIL inputFile, outputFile;
    char line[20];
    int bufferIndex = 0;
    int sampleCount = 0;
    int iterationCount = 0;
    double hanningWindow[SampleSize];

    // Calculate Hanning window
    calculateHanningWindow(hanningWindow, SampleSize);

    fresult = f_open(&inputFile, inputFileName, FA_READ);
    if (fresult != FR_OK) {
        send_uart("Error opening input file for reading.\r\n");

    }

    fresult = f_open(&outputFile, outputFileName, FA_CREATE_ALWAYS | FA_WRITE);
    if (fresult != FR_OK) {
        send_uart("Error opening output file for writing.\r\n");
        f_close(&inputFile);

    }

    while (f_gets(line, sizeof(line), &inputFile) != NULL) {
        double currentSample;
        if (sscanf(line, "%lf", &currentSample) == 1) {
            buffer[bufferIndex] = currentSample*hanningWindow[sampleCount];
            bufferIndex++;
            sampleCount++;


            if (bufferIndex == bufferSize) {
                *totalAverage += calculateAverage(buffer, bufferSize);
                bufferIndex = 0;
                iterationCount=iterationCount+1;
            }
        }
    }

    *totalAverage /= iterationCount;

    // Reset file pointer to the beginning for a second pass
    f_lseek(&inputFile, 0);

    while (f_gets(line, sizeof(line), &inputFile) != NULL) {
        double value;
        if (sscanf(line, "%lf", &value) == 1) {
            double diff = value - *totalAverage;
            snprintf(buffer, bufferSize, "%.2f\n", diff);
            f_puts(buffer, &outputFile);
        }
    }

    f_close(&inputFile);
    f_close(&outputFile);
}
void calculateHanningWindow(double* window, int size) {
    for (int i = 0; i < size; i++) {
        window[i] = 0.5 * (1 - cos(2.0 * M_PI * i / (size - 1)));
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
	  // Open the file for writing (for sensor 1)

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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  void calculateHanningWindow(double* window, int size);
 HAL_TIM_Base_Start(&htim3);
 arm_rfft_fast_init_f32(&fftHandler, FFT_BUFFER_SIZE);

  fresult = f_mount(&fs, "", 0);
  if (fresult != FR_OK) {
	  send_uart("\nSD card MOUNT FAILED...\r\n");
  } else {
	  send_uart("SD card MOUNTED...\r\n");
	  unlinkFile("/ADXL1_DATA.txt");
	  unlinkFile("/ADXL1_DATA_AVG_DIFF.txt");
	  unlinkFile("/ADXL2_DATA.txt");
	  unlinkFile("/ADXL2_DATA_AVG_DIFF.txt");

	 fresult = f_open(&fil, "ADXL1_DATA.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	 send_uart("ADXL1_DATA.txt created.\r\n");

	  //opening another file
	  fresult = f_open(&fil2, "ADXL2_DATA.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	  send_uart("ADXL2_DATA.txt created.\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  counter++;
	if (counter <= SampleSize)
	{
		ADC_Select_CH0();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
	 	adcVal[0] = (double)HAL_ADC_GetValue(&hadc1);
	 	Sensor1Values= adcVal[0];
	 	writeDoubleToFile(&fil, Sensor1Values);
		HAL_ADC_Stop(&hadc1);


		ADC_Select_CH1();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
		adcVal[1] = (double)HAL_ADC_GetValue(&hadc1);
		Sensor2Values= adcVal[1];
		writeDoubleToFile(&fil2, Sensor2Values);
		HAL_ADC_Stop(&hadc1);


	   }

	if (counter > SampleSize){
		HAL_TIM_Base_Stop(&htim3);
		fresult = f_close(&fil);
		fresult = f_close(&fil2);
		send_uart("Data acquisition complete.\r\n");

		double totalAverage1 = 0.0;
		processAndSaveData("ADXL1_DATA.txt", "ADXL1_DATA_AVG_DIFF.txt", buffer, BufferSize, &totalAverage1);

		double totalAverage2 = 0.0;
		processAndSaveData("ADXL2_DATA.txt", "ADXL2_DATA_AVG_DIFF.txt", buffer, BufferSize, &totalAverage2);
		send_uart("Programme complete!\r\n");
		fftFlag=1;

	}

	if(fftFlag)
	{
		 fresult = f_open(&fil, "ADXL1_DATA_AVG_DIFF.txt",FA_OPEN_ALWAYS | FA_READ);
		    if (fresult != FR_OK) {
		        send_uart("Error opening ADXL1_DATA_AVG_DIFF file for reading.\r\n");

		    }
		    char line[20];
		    while (f_gets(line, sizeof(line), &fil) != NULL) {

		    	double currentfftSample;
		        if (sscanf(line, "%lf", &currentfftSample) == 1) {
		        	fftBuffIn[fftbufferIndex] = currentfftSample;
		            fftbufferIndex++;
		    		 if (fftbufferIndex == FFT_BUFFER_SIZE) {
		    			 fftbufferIndex=0;
		    		 }
		        }
		    }
		    arm_rfft_fast_f32(&fftHandler,fftBuffIn,fftBuffOut,0);
		    arm_cmplx_mag_f32(fftBuffOut, outputfft_mag, FFT_BUFFER_SIZE/2);
			for (int k = 0;  k< FFT_BUFFER_SIZE/2; k++)
		 		  {
				//send_uart("");
				send_uart("frequency %f: %f\r\n", ((float32_t)(k * SAMPLING_RATE)/ FFT_BUFFER_SIZE),outputfft_mag[k]);
		 		  }
			fresult = f_close(&fil);
			break;

	}
	}
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 500-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 85-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
