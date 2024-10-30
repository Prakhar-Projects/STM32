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
#include "fatfs_sd.h"
#include "string.h"
#include <stdio.h>
#include <stdarg.h>
#define ARM_MATH_CM7
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SampleSize 24576
#define BufferSize 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define ADC_VAL_SIZE 384
uint32_t adcVal[ADC_VAL_SIZE] __attribute__ ((section(".nocache")));;
uint32_t adcFlush[ADC_VAL_SIZE];
double buffer[BufferSize];
unsigned long count=0;

FATFS fs;
FATFS *pfs;
FIL fil1,fil2,fil3,fil4,fil5,fil6,rawDataFile;
FRESULT fresult;

int bufferIndex = 0;
volatile char read_flag=0;
volatile char dist_flag=0;

int fftbufferIndex = 0;
#define FFT_BUFFER_SIZE 4096
#define SAMPLING_RATE 1002.13
arm_rfft_fast_instance_f32 fftHandler;
float32_t fftBuffIn[FFT_BUFFER_SIZE];
float32_t fftBuffOut[FFT_BUFFER_SIZE];
float32_t outputfft_mag_xAxis_s1[FFT_BUFFER_SIZE/2];
float32_t outputfft_mag_yAxis_s1[FFT_BUFFER_SIZE/2];
float32_t outputfft_mag_zAxis_s1[FFT_BUFFER_SIZE/2];
float32_t outputfft_mag_xAxis_s2[FFT_BUFFER_SIZE/2];
float32_t outputfft_mag_yAxis_s2[FFT_BUFFER_SIZE/2];
float32_t outputfft_mag_zAxis_s2[FFT_BUFFER_SIZE/2];
float32_t outputfft_mag_R_s1[FFT_BUFFER_SIZE/2];
float32_t outputfft_mag_R_s2[FFT_BUFFER_SIZE/2];
uint8_t fftFlag=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
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

void send_uart(const char *format, ...) {
    va_list args;
    va_start(args, format);
    char buffer[1024];
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    if (len >= 0) {
       HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len, 200);
    }
    va_end(args);
}

void writeToFile(FIL* file, uint32_t* buffer, int size) {
    char fileBuffer[1024];
    char intBuffer[7];
    int counter = 0;
    char* fbPtr = &fileBuffer[0];
    memset(fileBuffer, 0, 1024);
    memset(intBuffer, 0, 7);

    for (int i = 0; i < size; i ++) {
        int c = snprintf(intBuffer, sizeof(intBuffer), "%d\n", (int)buffer[i]);
        if (counter + c > 1024) {
            f_puts(fileBuffer, file);
            counter = 0;
            memset(fileBuffer, 0, 1024);
        }
        memcpy(&fileBuffer[counter], intBuffer, c);
        counter += c;
        fbPtr += c;
    }
    if (counter > 0) {
        f_puts(fileBuffer, file);

 }
}
void distributeDataToFiles(FIL* rawDataFile, FIL* file1, FIL* file2, FIL* file3) {
    FIL* files[3] = {file1, file2, file3};
    char fileBuffers[3][1024];
    int counters[3] = {0, 0, 0};
    char intBuffer[7];

    for (int i = 0; i < 3; i++) {
        memset(fileBuffers[i], 0, 1024);
    }

    char line[12];
    int fileIndex = 0;

    while (f_gets(line, sizeof(line), rawDataFile)) {
        int c = snprintf(intBuffer, sizeof(intBuffer), "%s", line);
        if (counters[fileIndex] + c > 1024) {
            f_puts(fileBuffers[fileIndex], files[fileIndex]);
            counters[fileIndex] = 0;
            memset(fileBuffers[fileIndex], 0, 1024);
        }
        memcpy(&fileBuffers[fileIndex][counters[fileIndex]], intBuffer, c);
        counters[fileIndex] += c;

        fileIndex = (fileIndex + 1) % 3;
    }

    for (int i = 0; i < 3; i++) {
        if (counters[i] > 0) {
            f_puts(fileBuffers[i], files[i]);
        }
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
 {
	read_flag = 1;
	memcpy(adcFlush,adcVal,ADC_VAL_SIZE*sizeof(uint32_t));
	//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_12);
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

    fresult = f_open(&outputFile, outputFileName, FA_OPEN_ALWAYS | FA_WRITE);
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

void generate_resultant(const char *data_file1, const char *data_file2, const char *data_file3, const char *add_file) {
    FIL fil1, fil2, fil3, add_fil;
    FRESULT fresult;

    // Open the first data file for reading
    fresult = f_open(&fil1, data_file1, FA_OPEN_ALWAYS | FA_READ);
    if (fresult != FR_OK) {
        send_uart("Error opening X-Axis.\r\n");
        return;
    }

    // Open the second data file for reading
    fresult = f_open(&fil2, data_file2, FA_OPEN_ALWAYS | FA_READ);
    if (fresult != FR_OK) {
        send_uart("Error opening Y-Axis.\r\n");
        f_close(&fil1);
        return;
    }

    // Open the third data file for reading
    fresult = f_open(&fil3, data_file3, FA_OPEN_ALWAYS | FA_READ);
    if (fresult != FR_OK) {
        send_uart("Error opening Z-Axis.\r\n");
        f_close(&fil1);
        f_close(&fil2);
        return;
    }

    // Open the resultant file for writing
    fresult = f_open(&add_fil, add_file, FA_OPEN_ALWAYS | FA_WRITE);
    if (fresult != FR_OK) {
        send_uart("Error opening Resultant file for writing.\r\n");
        f_close(&fil1);
        f_close(&fil2);
        f_close(&fil3);
        return;
    }

    char line1[50], line2[50], line3[50];
    double mag1, mag2, mag3;

    while (f_gets(line1, sizeof(line1), &fil1) != NULL && f_gets(line2, sizeof(line2), &fil2) != NULL && f_gets(line3, sizeof(line3), &fil3) != NULL) {

        sscanf(line1, "%lf", &mag1);
        sscanf(line2, "%lf", &mag2);
        sscanf(line3, "%lf", &mag3);

        // Calculate and format sum of magnitudes
        double add_mag = sqrt(mag1*mag1 + mag2*mag2 + mag3*mag3);

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
    f_close(&fil3);
    f_close(&add_fil);
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
        send_uart("Error opening FFT file for writing.\r\n");
        f_close(&fil1);
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
                for (int i = 0; i < FFT_BUFFER_SIZE/2; i++) {
                    float curVal = outputfft_mag[i];
                    if (curVal > peakVal) {
                        peakVal = curVal;
                        peakIndex = i;
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

void generate_frequency_difference(const char *fft_file1, const char *fft_file2, const char *diff_file) {
    FIL fil1, fil2, diff_fil;
    FRESULT fresult;

    // Open the first FFT file for reading
    fresult = f_open(&fil1, fft_file1, FA_OPEN_ALWAYS | FA_READ);
    if (fresult != FR_OK) {
        send_uart("Error opening first FFT file for reading.\r\n");
        return;
    }

    // Open the second FFT file for reading
    fresult = f_open(&fil2, fft_file2, FA_OPEN_ALWAYS | FA_READ);
    if (fresult != FR_OK) {
        send_uart("Error opening second FFT file for reading.\r\n");
        f_close(&fil1);
        return;
    }

    // Open the difference file for writing
    fresult = f_open(&diff_fil, diff_file, FA_OPEN_ALWAYS | FA_WRITE);
    if (fresult != FR_OK) {
        send_uart("Error opening difference file for writing.\r\n");
        f_close(&fil1);
        f_close(&fil2);
        return;
    }

    char line1[50], line2[50];
    float freq1, mag1, freq2, mag2;
    float max_diff_mag = 0;
    float max_diff_freq = 0;

    while (f_gets(line1, sizeof(line1), &fil1) != NULL && f_gets(line2, sizeof(line2), &fil2) != NULL) {
        // Parse frequency and magnitude from each file
        sscanf(line1, "frequency %f: %f", &freq1, &mag1);
        sscanf(line2, "frequency %f: %f", &freq2, &mag2);

        // Calculate the difference in magnitudes
        float diff_mag = mag2-mag1;

        // Write the difference along with the frequency to the difference file
        char diff_line[50];
        snprintf(diff_line, sizeof(diff_line), "frequency %.2f: %.2f\n", freq1, diff_mag);
        f_puts(diff_line, &diff_fil);

        // Update the maximum difference and corresponding frequency
        if (fabs(diff_mag) > max_diff_mag) {
            max_diff_mag = fabs(diff_mag);
            max_diff_freq = freq1;
        }
    }

    // Close all files
    f_close(&fil1);
    f_close(&fil2);
    f_close(&diff_fil);
    // Print the maximum difference magnitude and corresponding frequency
    char max_diff_message[100];
    snprintf(max_diff_message, sizeof(max_diff_message), "Max difference magnitude: %.2f at frequency: %.2f\r\n", max_diff_mag, max_diff_freq);
    send_uart(max_diff_message);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET ,ADC_SINGLE_ENDED);

HAL_ADC_Start_DMA(&hadc1, adcVal, ADC_VAL_SIZE);

HAL_TIM_Base_Start(&htim3);

arm_rfft_fast_init_f32(&fftHandler, FFT_BUFFER_SIZE);
fresult = f_mount(&fs, "", 0);
   	  if (fresult != FR_OK) {
   		  send_uart("SD card MOUNT FAILED...\r\n");
   	  } else {
   		  send_uart("\n\rSD card MOUNTED...\r\n");

   		  fresult = f_open(&rawDataFile, "rawData.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
   		  send_uart("rawData.txt created.\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (read_flag)
	  	 	  {
	  	 		  writeToFile(&rawDataFile, adcFlush, ADC_VAL_SIZE);
	  	 		  count+=ADC_VAL_SIZE;
	  	 		  read_flag = 0;
	  	 	  }
	  	 	  if (count >= SampleSize)
	  	 	  {
	  	 		  HAL_ADC_Stop_DMA(&hadc1);
	  	 		  HAL_TIM_Base_Stop(&htim3);
	  	 		  f_close(&rawDataFile);
	  	 		  send_uart("Data acquisition complete!\r\n");
	  	 		  dist_flag = 1;
	  	 		  if (dist_flag){
	  	 		  fresult = f_open(&rawDataFile, "rawData.txt",FA_READ);

	  	 		  fresult = f_open(&fil1, "xAxisS1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	  	 		  send_uart("xAxis_sensor1.txt created.\r\n");

	  	 		  fresult = f_open(&fil2, "yAxisS1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	  	   		  send_uart("yAxis_sensor1.txt created.\r\n");

	  	   		  fresult = f_open(&fil3, "zAxisS1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	  	   		  send_uart("zAxis_sensor1.txt created.\r\n");

//	  	   		  fresult = f_open(&fil4, "xAxisS2.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
//	  	  		  send_uart("xAxis_sensor2.txt created.\r\n");
//
//	  	   		  fresult = f_open(&fil5, "yAxisS2.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
//	  	  		  send_uart("yAxis_sensor2.txt created.\r\n");
//
//	  	   		  fresult = f_open(&fil6, "zAxisS2.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
//	  	  		  send_uart("zAxis_sensor2.txt created.\r\n");

	  	  		  distributeDataToFiles(&rawDataFile, &fil1, &fil2, &fil3);
	  	  		  f_close(&rawDataFile);
	  	  		  f_close(&fil1);
	  	  		  f_close(&fil2);
	  	  		  f_close(&fil3);
	  	 		  }
	  	 		  dist_flag=0;
	  	 		  generate_resultant("xAxisS1.txt", "yAxisS1.txt", "zAxisS1.txt", "ResultantS1.txt");

//	  	 		  generate_resultant("xAxisS2.txt", "yAxisS2.txt", "zAxisS2.txt", "ResultantS2.txt");

	  	 		  double totalAverage1 = 0.0;
	  	 		  processAndSaveData("xAxisS1.txt", "xAxisMWFS1.txt", buffer, BufferSize, &totalAverage1);
	  	 		  double totalAverage2 = 0.0;
	  	 		  processAndSaveData("yAxisS1.txt", "yAxisMWFS1.txt", buffer, BufferSize, &totalAverage2);
	  	 		  double totalAverage3 = 0.0;
	  	 		  processAndSaveData("zAxisS1.txt", "zAxisMWFS1.txt", buffer, BufferSize, &totalAverage3);
//	  	 		  double totalAverage4 = 0.0;
//	  	 		  processAndSaveData("xAxisS2.txt", "xAxisMWFS2.txt", buffer, BufferSize, &totalAverage4);
//	  	 		  double totalAverage5 = 0.0;
//	  	 		  processAndSaveData("yAxisS2.txt", "yAxisMWFS2.txt", buffer, BufferSize, &totalAverage5);
//	  	 		  double totalAverage6 = 0.0;
//	  	 		  processAndSaveData("zAxisS2.txt", "zAxisMWFS2.txt", buffer, BufferSize, &totalAverage6);
	  	 		  double totalAverage7 = 0.0;
	  	 		  processAndSaveData("ResultantS1.txt", "ResultantMWFS1.txt", buffer, BufferSize, &totalAverage7);
//	  	 		  double totalAverage8 = 0.0;
//	  	 		  processAndSaveData("ResultantS2.txt", "ResultantMWFS2.txt", buffer, BufferSize, &totalAverage8);

	  	 		  send_uart("Text database created!!\r\n");
	  	 		  fftFlag=1;

	  	 	  	  	  }
	  	 	  if(fftFlag)
	  	 			{
	  		   send_uart("Performing FFT on X_Axis!\r\n");
	  		   fftResults("xAxisMWFS1.txt", "xAxisFFTS1.txt", outputfft_mag_xAxis_s1, fftBuffIn, fftBuffOut);

	  		   send_uart("Performing FFT on Y_Axis!\r\n");
	  		   fftResults("yAxisMWFS1.txt", "yAxisFFTS1.txt", outputfft_mag_yAxis_s1, fftBuffIn, fftBuffOut);

	  		   send_uart("Performing FFT on Z_Axis!\r\n");
	  		   fftResults("zAxisMWFS1.txt", "zAxisFFTS1.txt", outputfft_mag_zAxis_s1, fftBuffIn, fftBuffOut);

	  		   send_uart("Performing FFT on Resultant!\r\n");
	  		   fftResults("ResultantMWFS1.txt", "ResultantFFTS1.txt", outputfft_mag_R_s1, fftBuffIn, fftBuffOut);

//	  		   send_uart("Performing FFT on sensor2_X_Axis!\r\n");
//	  		   fftResults("xAxisMWFS2.txt", "xAxisFFTS2.txt", outputfft_mag_xAxis_s2, fftBuffIn, fftBuffOut);
//
//	  		   send_uart("Performing FFT on sensor2_Y_Axis!\r\n");
//	  		   fftResults("yAxisMWFS2.txt", "yAxisFFTS2.txt", outputfft_mag_yAxis_s2, fftBuffIn, fftBuffOut);
//
//	  		   send_uart("Performing FFT on sensor2_Z_Axis!\r\n");
//	  		   fftResults("zAxisMWFS2.txt", "zAxisFFTS2.txt", outputfft_mag_zAxis_s2, fftBuffIn, fftBuffOut);
//
//	  		   send_uart("Performing FFT on sensor2_Resultant!\r\n");
//	  		   fftResults("ResultantMWFS2.txt", "ResultantFFTS2.txt", outputfft_mag_R_s2, fftBuffIn, fftBuffOut);

//	  		   generate_frequency_difference("ResultantFFTS1.txt", "ResultantFFTS2.txt", "Frequency_Difference.txt");
	  		   send_uart("Programme Complete!\r\n");
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

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 35;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
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
  htim3.Init.Prescaler = 280-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
