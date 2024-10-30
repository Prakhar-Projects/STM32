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
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "errorstringifyFRresult.h"
#include "sdTESTS.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SampleSize 24576
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  FRESULT fres;
  char log_path[] = "/RAW.TXT";
  char log_x[]="/XAXIS.TXT";
  char log_y[]="/YAXIS.TXT";
  char log_z[]="/ZAXIS.TXT";
  char log_r[]="/RESULTANT.TXT";
  char log_x_M[]="/XMFREE.TXT";
  char log_y_M[]="/YMFREE.TXT";
  char log_z_M[]="/ZMFREE.TXT";
  char log_r_M[]="/RMFREE.TXT";

  char log_x_F[]="/XFFT.TXT";
  char log_y_F[]="/YFFT.TXT";
  char log_z_F[]="/ZFFT.TXT";
  char log_r_F[]="/RFFT.TXT";


#define BufferSize 2048
#define ADC_VAL_SIZE 48
uint32_t adcVal[ADC_VAL_SIZE];
uint32_t adcFlush[ADC_VAL_SIZE];
double buffer[BufferSize];
volatile char read_flag=0;
unsigned long count=0;

#define ARM_MATH_CM4
#include "arm_math.h"
int fftbufferIndex = 0;
#define FFT_BUFFER_SIZE 2048
#define SAMPLING_RATE 935.15
arm_rfft_fast_instance_f32 fftHandler;
float32_t fftBuffIn[FFT_BUFFER_SIZE];
float32_t fftBuffOut[FFT_BUFFER_SIZE];
float32_t outputfft_mag_xAxis[FFT_BUFFER_SIZE/2];
float32_t outputfft_mag_yAxis[FFT_BUFFER_SIZE/2];
float32_t outputfft_mag_zAxis[FFT_BUFFER_SIZE/2];
float32_t outputfft_mag_R[FFT_BUFFER_SIZE/2];
uint8_t fftFlag=0;
  /* File read buffer */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
void processAndSaveData(const char *inputFileName, const char *outputFileName, double *buffer, int bufferSize, double *totalAverage) {
    FRESULT fresult;
    FIL inputFile, outputFile;
    char line[20];
    int bufferIndex = 0;
    int iterationCount = 0;

//    // Mount the SD card
//    fresult = f_mount(&SDFatFS, SDPath, 1);
//    if (fres != FR_OK) {
//        // Handle SD mount error
//        return;
//    }

    fresult = f_open(&inputFile, inputFileName, FA_READ);
    if (fresult != FR_OK) {
      return;
    }

    fresult = f_open(&outputFile, outputFileName, FA_OPEN_ALWAYS | FA_WRITE);
    if (fresult != FR_OK) {
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

void writeToFile(char* path1, char* path2, char* path3, uint32_t* buffer, int size) {
    char fileBuffer1[1024];
    char fileBuffer2[1024];
    char fileBuffer3[1024];
    char intBuffer[6];  // Buffer to hold the integer as a string

    int counter1 = 0;
    int counter2 = 0;
    int counter3 = 0;

    char* fbPtr1 = &fileBuffer1[0];
    char* fbPtr2 = &fileBuffer2[0];
    char* fbPtr3 = &fileBuffer3[0];

    // Initialize the buffers
    memset(fileBuffer1, 0, sizeof(fileBuffer1));
    memset(fileBuffer2, 0, sizeof(fileBuffer2));
    memset(fileBuffer3, 0, sizeof(fileBuffer3));
    memset(intBuffer, 0, sizeof(intBuffer));

    // Loop over the buffer and distribute the data into three files
    for (int i = 0; i < size; i += 3) {
        // Process for file 1
        int c = snprintf(intBuffer, sizeof(intBuffer), "%d\n", (int)buffer[i]);
        if (counter1 + c > 1024) {
            // When buffer is full, append to file
            AppendToFile(path1, strlen(path1), fileBuffer1, counter1);
            counter1 = 0;
            memset(fileBuffer1, 0, sizeof(fileBuffer1));
        }
        memcpy(&fileBuffer1[counter1], intBuffer, c);
        counter1 += c;
        fbPtr1+=c;

        // Process for file 2
        if (i + 1 < size) {
            c = snprintf(intBuffer, sizeof(intBuffer), "%d\n", (int)buffer[i + 1]);
            if (counter2 + c > 1024) {
                AppendToFile(path2, strlen(path2), fileBuffer2, counter2);
                counter2 = 0;
                memset(fileBuffer2, 0, sizeof(fileBuffer2));
            }
            memcpy(&fileBuffer2[counter2], intBuffer, c);
            counter2 += c;
            fbPtr2 += c;
        }

        // Process for file 3
        if (i + 2 < size) {
            c = snprintf(intBuffer, sizeof(intBuffer), "%d\n", (int)buffer[i + 2]);
            if (counter3 + c > 1024) {
                AppendToFile(path3, strlen(path3), fileBuffer3, counter3);
                counter3 = 0;
                memset(fileBuffer3, 0, sizeof(fileBuffer3));
            }
            memcpy(&fileBuffer3[counter3], intBuffer, c);
            counter3 += c;
            fbPtr3 += c;
        }
    }

    // After the loop, append any remaining data to the files
    if (counter1 > 0) {
        AppendToFile(path1, strlen(path1), fileBuffer1, counter1);
    }
    if (counter2 > 0) {
        AppendToFile(path2, strlen(path2), fileBuffer2, counter2);
    }
    if (counter3 > 0) {
        AppendToFile(path3, strlen(path3), fileBuffer3, counter3);
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
 {
	read_flag = 1;
	memcpy(adcFlush,adcVal,ADC_VAL_SIZE*sizeof(uint32_t));
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
 }
void calculateResultant(char* xPath, char* yPath, char* zPath, char* resultPath) {
    // Buffers to hold file data
    char xBuffer[32];
    char yBuffer[32];
    char zBuffer[32];
    char resultBuffer[1024];
    int resultCounter = 0;
    char resultStr[32];
    int xValue, yValue, zValue;

    // Initialize file structures
    FIL xFile, yFile, zFile, resultFile;
    FRESULT fres;

//    // Mount the SD card
//    fres = f_mount(&SDFatFS, SDPath, 1);
//    if (fres != FR_OK) {
//        // Handle SD mount error
//        return;
//    }

    // Open the X, Y, Z axis files for reading
    fres = f_open(&xFile, xPath, FA_READ);
    if (fres != FR_OK) {
        // Handle file open error
        return;
    }

    fres = f_open(&yFile, yPath, FA_READ);
    if (fres != FR_OK) {
        // Handle file open error
        f_close(&xFile);
        return;
    }

    fres = f_open(&zFile, zPath, FA_READ);
    if (fres != FR_OK) {
        // Handle file open error
        f_close(&xFile);
        f_close(&yFile);
        return;
    }

    // Open the result file for writing
    fres = f_open(&resultFile, resultPath, FA_WRITE | FA_CREATE_ALWAYS);
    if (fres != FR_OK) {
        // Handle file open error
        f_close(&xFile);
        f_close(&yFile);
        f_close(&zFile);
        return;
    }

    // Read lines from X, Y, Z files, compute resultant, and write to result file
    while ((f_gets(xBuffer, sizeof(xBuffer), &xFile) != NULL) &&
           (f_gets(yBuffer, sizeof(yBuffer), &yFile) != NULL) &&
           (f_gets(zBuffer, sizeof(zBuffer), &zFile) != NULL)) {

        // Convert strings to integers
        xValue = atoi(xBuffer);
        yValue = atoi(yBuffer);
        zValue = atoi(zBuffer);

        // Calculate the resultant vector: sqrt(x^2 + y^2 + z^2)
        float resultant = sqrtf((xValue * xValue) + (yValue * yValue) + (zValue * zValue));

        // Convert the resultant to string and append to buffer
        int len = snprintf(resultStr, sizeof(resultStr), "%.2f\n", resultant);
        if (resultCounter + len >= sizeof(resultBuffer)) {
            // If buffer is full, write to file and reset the counter
            f_write(&resultFile, resultBuffer, resultCounter, NULL);
            resultCounter = 0;
            memset(resultBuffer, 0, sizeof(resultBuffer));
        }
        memcpy(&resultBuffer[resultCounter], resultStr, len);
        resultCounter += len;
    }

    // Write any remaining data in the buffer
    if (resultCounter > 0) {
        f_write(&resultFile, resultBuffer, resultCounter, NULL);
    }

    // Close all files
    f_close(&xFile);
    f_close(&yFile);
    f_close(&zFile);
    f_close(&resultFile);

//    // Unmount the SD card
//    f_mount(NULL, SDPath, 1);
}
void fftResults(const char *data_file, const char *fft_file, float *outputfft_mag, float *fftBuffIn, float *fftBuffOut) {
    FIL fil1, fil2;
    FRESULT fresult;

//    // Mount the SD card
//    fresult = f_mount(&SDFatFS, SDPath, 1);
//    if (fresult != FR_OK) {
//        // Error mounting the SD card, return from function
//        return;
//    }

    // Open the data file for reading
    fresult = f_open(&fil1, data_file, FA_OPEN_ALWAYS | FA_READ);
    if (fresult != FR_OK) {
        // Error opening data file for reading, unmount SD and return
        f_mount(NULL, SDPath, 1);
        return;
    }

    // Open the FFT result file for writing
    fresult = f_open(&fil2, fft_file, FA_OPEN_ALWAYS | FA_WRITE);
    if (fresult != FR_OK) {
        // Error opening FFT file for writing, close input file, unmount SD and return
        f_close(&fil1);
        f_mount(NULL, SDPath, 1);
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

            // If buffer is full, perform FFT
            if (fftbufferIndex == FFT_BUFFER_SIZE) {
                // Perform FFT
                arm_rfft_fast_f32(&fftHandler, fftBuffIn, fftBuffOut, 0);
                arm_cmplx_mag_f32(fftBuffOut, outputfft_mag, FFT_BUFFER_SIZE / 2);

                // Write FFT magnitude results to file
                char fftline[50];
                for (int i = 0; i < FFT_BUFFER_SIZE / 2; i++) {
                    snprintf(fftline, sizeof(fftline), "frequency %.2f: %.2f\n", ((float32_t)(i * SAMPLING_RATE) / FFT_BUFFER_SIZE), outputfft_mag[i]);
                    f_puts(fftline, &fil2);
                }

                // Find the peak frequency and magnitude
                float peakVal = 0.0;
                int peakIndex = 0;
                for (int i = 0; i < FFT_BUFFER_SIZE / 2; i++) {
                    float curVal = outputfft_mag[i];
                    if (curVal > peakVal) {
                        peakVal = curVal;
                        peakIndex = i;
                    }
                }

                // Calculate peak frequency
                float peakFrequency = (float32_t)(peakIndex * SAMPLING_RATE) / FFT_BUFFER_SIZE;

                // Write peak frequency and magnitude to file
                snprintf(fftline, sizeof(fftline), "Peak Frequency: %.3f Hz, Peak Value: %.3f\n", peakFrequency, peakVal);
                f_puts(fftline, &fil2);

                // Reset FFT buffer index for the next chunk
                fftbufferIndex = 0;
            }
        }
    }

    // Close both files
    f_close(&fil1);
    f_close(&fil2);

//    // Unmount the SD card
//    f_mount(NULL, SDPath, 1);
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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, adcVal, ADC_VAL_SIZE);
  HAL_TIM_Base_Start(&htim2);
  arm_rfft_fast_init_f32(&fftHandler, FFT_BUFFER_SIZE);
  HAL_Delay(1000);
  FRESULT sd_status = SD_Setup();
  if (sd_status != FR_OK) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, SET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RESET);
	  HAL_Delay(500);
      }
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (read_flag)
	  {
		  writeToFile(log_x, log_y, log_z, adcFlush, ADC_VAL_SIZE);
		  count += ADC_VAL_SIZE;  // Update the count by the size of the data collected
		  read_flag = 0;
	  }
	  if (count >= SampleSize)
	    {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RESET);
		  HAL_ADC_Stop_DMA(&hadc1);
		  HAL_TIM_Base_Stop(&htim2);
          calculateResultant(log_x, log_y, log_z, log_r);
          double totalAverage1 = 0.0;
          processAndSaveData(log_x, log_x_M, buffer, BufferSize, &totalAverage1);
          double totalAverage2 = 0.0;
          processAndSaveData(log_y, log_y_M, buffer, BufferSize, &totalAverage2);
          double totalAverage3 = 0.0;
          processAndSaveData(log_z,log_z_M, buffer, BufferSize, &totalAverage3);
          double totalAverage4 = 0.0;
          processAndSaveData(log_r,log_r_M, buffer, BufferSize, &totalAverage4);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, SET);
          fftFlag=1;
	    }
	  if(fftFlag){
		  fftResults(log_x_M, log_x_F, outputfft_mag_xAxis, fftBuffIn, fftBuffOut);
		  fftResults(log_y_M, log_y_F, outputfft_mag_yAxis, fftBuffIn, fftBuffOut);
		  fftResults(log_z_M, log_z_F, outputfft_mag_zAxis, fftBuffIn, fftBuffOut);
		  fftResults(log_r_M, log_r_F, outputfft_mag_R, fftBuffIn, fftBuffOut);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RESET);
		  f_mount(NULL, SDPath, 1);
		  break;
	  }
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
