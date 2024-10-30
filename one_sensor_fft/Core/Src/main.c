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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SampleSize 65536
#define BufferSize 2048

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define ADC_VAL_SIZE 64
uint32_t adcVal[ADC_VAL_SIZE];
uint32_t adcFlush[ADC_VAL_SIZE];
double buffer[BufferSize];
unsigned long count=0;

FATFS fs;
FATFS *pfs;
FIL fil1,fil2;
FRESULT fresult;

int bufferIndex = 0;
char read_flag=0;

int fftbufferIndex = 0;
#define FFT_BUFFER_SIZE 2048
#define SAMPLING_RATE 500
arm_rfft_fast_instance_f32 fftHandler;
float32_t fftBuffIn[FFT_BUFFER_SIZE];
float32_t fftBuffOut[FFT_BUFFER_SIZE];
float32_t outputfft_mag_Adxl1[FFT_BUFFER_SIZE/2];
float32_t outputfft_mag_Adxl2[FFT_BUFFER_SIZE/2];
uint8_t fftFlag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
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

void send_uart(const char *format, ...) {
    va_list args;
    va_start(args, format);
    char buffer[1024];
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    if (len >= 0) {
       HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 200);
    }
    va_end(args);
}


void writeToFile(FIL* file1, FIL* file2, uint32_t* buffer, int size) {
    char fileBuffer1[1024];
    char fileBuffer2[1024];
    char intBuffer[6];

    int counter1 = 0;
    int counter2 = 0;

    char* fbPtr1 = &fileBuffer1[0];
    char* fbPtr2 = &fileBuffer2[0];

    memset(fileBuffer1, 0, 1024);
    memset(fileBuffer2, 0, 1024);
    memset(intBuffer, 0, 6);

    for (int i = 0; i < size; i += 2) {
        int c = snprintf(intBuffer, sizeof(intBuffer), "%d\n", (int)buffer[i]);

        if (counter1 + c > 1024) {
            f_puts(fileBuffer1, file1);
            counter1 = 0;
            memset(fileBuffer1, 0, 1024);
        }
        memcpy(&fileBuffer1[counter1], intBuffer, c);
        counter1 += c;
        fbPtr1+=c;

        c = snprintf(intBuffer, sizeof(intBuffer), "%d\n", (int)buffer[i + 1]);
        if (counter2 + c > 1024) {
            f_puts(fileBuffer2, file2);
            counter2 = 0;
            memset(fileBuffer2, 0, 1024);
        }
        memcpy(&fileBuffer2[counter2], intBuffer, c);
        counter2 += c;
        fbPtr2+=c;
    }

    if (counter1 > 0) {
        f_puts(fileBuffer1, file1);
    }

    if (counter2 > 0) {
        f_puts(fileBuffer2, file2);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
 {
	read_flag = 1;
	memcpy(adcFlush,adcVal,ADC_VAL_SIZE*sizeof(uint32_t));
 }

void generate_resultant(const char *data_file1, const char *data_file2, const char *add_file) {
    FIL fil1, fil2, add_fil;
    FRESULT fresult;

    // Open the first data file for reading
    fresult = f_open(&fil1, data_file1, FA_OPEN_ALWAYS | FA_READ);
    if (fresult != FR_OK) {
        send_uart("Error opening ADXL1.\r\n");
        return;
    }

    // Open the second data file for reading
    fresult = f_open(&fil2, data_file2, FA_OPEN_ALWAYS | FA_READ);
    if (fresult != FR_OK) {
        send_uart("Error opening ADXL2.\r\n");
        f_close(&fil1);
        return;
    }

    // Open the difference file for writing
    fresult = f_open(&add_fil, add_file, FA_CREATE_ALWAYS | FA_WRITE);
    if (fresult != FR_OK) {
        send_uart("Error opening Resultant file for writing.\r\n");
        f_close(&fil1);
        f_close(&fil2);
        return;
    }

    char line1[50], line2[50];
    double mag1, mag2;

    while (f_gets(line1, sizeof(line1), &fil1) != NULL && f_gets(line2, sizeof(line2), &fil2) != NULL) {

        sscanf(line1, "%lf", &mag1);
        sscanf(line2, "%lf", &mag2);

        // Calculate and format sum of magnitudes
        double add_mag = sqrt(mag1*mag1 + mag2*mag2);

        char add_mag_str[20]; // Adjust size as needed to accommodate formatted output
        snprintf(add_mag_str, sizeof(add_mag_str), "%.2f\n", (double)add_mag); // Convert double to string with formatting
        f_puts(add_mag_str, &add_fil);
        if (fresult != FR_OK) {
            send_uart("Error writing to resultant.txt.\r\n");

        }
    }

    // Close all files
    f_close(&fil1);
    f_close(&fil2);
    f_close(&add_fil);
}
void processAndSaveData(const char *inputFileName, const char *outputFileName, double *buffer, int bufferSize, double *totalAverage) {
    FRESULT fresult;
    FIL inputFile, outputFile;
    char line[20];
    int bufferIndex = 0;
    int iterationCount = 0;

    fresult = f_open(&inputFile, inputFileName, FA_READ);
    if (fresult != FR_OK) {
        send_uart("Error opening input file for reading.\r\n");
        return;
    }

    fresult = f_open(&outputFile, outputFileName, FA_CREATE_ALWAYS | FA_WRITE);
    if (fresult != FR_OK) {
        send_uart("Error opening output file for writing.\r\n");
        f_close(&inputFile);
        return;
    }

    while (f_gets(line, sizeof(line), &inputFile) != NULL) {
        double currentSample;
        if (sscanf(line, "%lf", &currentSample) == 1) {
            buffer[bufferIndex] = currentSample;
            bufferIndex++;

            if (bufferIndex == bufferSize) {
                *totalAverage += calculateAverage(buffer, bufferSize);
                bufferIndex = 0;
                iterationCount=iterationCount+1;
            }
        }
    }

    *totalAverage /= iterationCount;

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
void fftResults(const char *data_file, const char *fft_file, float *outputfft_mag, float *fftBuffIn, float *fftBuffOut) {
    FIL fil1, fil2;
    FRESULT fresult;

    fresult = f_open(&fil1, data_file, FA_OPEN_ALWAYS | FA_READ);
    if (fresult != FR_OK) {
        send_uart("Error opening data file for reading.\r\n");
        return;
    }
    fresult = f_open(&fil2, fft_file, FA_OPEN_ALWAYS | FA_WRITE);
    if (fresult != FR_OK) {
        send_uart("Error opening fft file for writing.\r\n");
        return;
    }

    char line[20];
    int fftbufferIndex = 0;
    int dataCount = 0;

    while (f_gets(line, sizeof(line), &fil1) != NULL && dataCount < FFT_BUFFER_SIZE) {
        double currentfftSample;
        if (sscanf(line, "%lf", &currentfftSample) == 1) {
            fftBuffIn[fftbufferIndex] = currentfftSample;
            fftbufferIndex++;
            dataCount++;
            if (fftbufferIndex == FFT_BUFFER_SIZE) {
                arm_rfft_fast_f32(&fftHandler, fftBuffIn, fftBuffOut, 0);
                arm_cmplx_mag_f32(fftBuffOut, outputfft_mag, FFT_BUFFER_SIZE/2);

                char fftline[50];
                for (int i = 0; i < FFT_BUFFER_SIZE/2; i++) {
                    snprintf(fftline, sizeof(fftline), "frequency %.2f: %.2f\n", ((float32_t)(i*SAMPLING_RATE)/FFT_BUFFER_SIZE), outputfft_mag[i]);
                    f_puts(fftline, &fil2);
                }

                float peakVal = 0.0;
                int peakIndex = 0;
                for (int k = 0; k < FFT_BUFFER_SIZE/2; k++) {
                    float curVal = outputfft_mag[k];
                    if (curVal > peakVal) {
                        peakVal = curVal;
                        peakIndex = k;
                    }
                }
                float peakFrequency = (float32_t)(peakIndex * SAMPLING_RATE) / FFT_BUFFER_SIZE;
                send_uart("Peak Value: %.3f at Frequency: %.3f Hz\r\n", peakVal, peakFrequency);

                fftbufferIndex = 0;
            }
        }
    }

    // Close both files
    f_close(&fil1);
    f_close(&fil2);
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, adcVal, ADC_VAL_SIZE);
  HAL_TIM_Base_Start(&htim2);
  arm_rfft_fast_init_f32(&fftHandler, FFT_BUFFER_SIZE);

    fresult = f_mount(&fs, "", 0);
    	  if (fresult != FR_OK) {
    		  send_uart("SD card MOUNT FAILED...\r\n");
    	  } else {
    		  send_uart("\n\rSD card MOUNTED...\r\n");

    		  unlinkFile("/ADXL1_DATA.txt");
    		  unlinkFile("/Resultant_AVG_DIFF.txt");
    		  unlinkFile("/ADXL2_DATA.txt");
    		  unlinkFile("/Resultant.txt");


    		  fresult = f_open(&fil1, "ADXL1_DATA.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
    		  send_uart("ADXL1_DATA.txt created.\r\n");

    		  fresult = f_open(&fil2, "ADXL2_DATA.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
    		  send_uart("ADXL2_DATA.txt created.\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (read_flag)
	 	  {
	 		  writeToFile(&fil1, &fil2, adcFlush, ADC_VAL_SIZE);
	 		  count+=ADC_VAL_SIZE;
	 		  read_flag = 0;
	 	  }
	 	  if (count >= SampleSize)
	 	  {
	 		  HAL_ADC_Stop_DMA(&hadc1);
	 		  HAL_TIM_Base_Stop(&htim2);
	 		  f_close(&fil1);
	 		  f_close(&fil2);
	 		  send_uart("Data acquisition complete!\r\n");
	 		  generate_resultant("ADXL1_DATA.txt", "ADXL2_DATA.txt", "Resultant.txt");
	 		  double totalAverage1 = 0.0;
	 		  processAndSaveData("Resultant.txt", "Resultant_AVG_DIFF.txt", buffer, BufferSize, &totalAverage1);
	 		  send_uart("FFT preparation complete!\r\n");
	 		  fftFlag=1;
	 	  }
	 	  if(fftFlag){
	 		 fftResults("Resultant_AVG_DIFF.txt", "Resultant_FFT.txt", outputfft_mag_Adxl1, fftBuffIn, fftBuffOut);
	 		 send_uart("Programme Complete!\r\n");
	 		 break;
	 	  }
  	  	  }
    	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 840-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
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
