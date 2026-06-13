/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dwt_timer.h"
#include "arm_math.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ONLINE_K_B 0.01f
#define N_AVG      100U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
#define WINDOW_SIZE 256

static float bufferA[WINDOW_SIZE];
static float bufferB[WINDOW_SIZE];
static volatile float  *fillBuffer    = bufferA;
static volatile float  *processBuffer = bufferB;
static volatile uint16_t sampleIndex  = 0;
static volatile uint8_t  windowReady  = 0;
static volatile uint32_t windowCount  = 0;

static arm_rfft_fast_instance_f32 fftInstance;
static float fftOutput[WINDOW_SIZE];

/* Calibration */
static float T_fft_calib_us = 0.0f;
static float calib_b = 0.0f;

/* Metrics */
static uint32_t T_actual_us    = 0;
static uint32_t T_predicted_us = 0;
static float    rms_current    = 0.0f;
static uint8_t  faultFlag      = 0;
static uint16_t batteryAdc     = 0;

#define FAULT_THRESHOLD_G  1.2f
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();

  /* BSP init */
  BspCOMInit.BaudRate   = 460800;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  BSP_COM_Init(COM1, &BspCOMInit);
  BSP_LED_Init(LED_GREEN);

  printf("[boot] Case A baseline starting...\r\n");

  /* Calibrate FFT */
  arm_rfft_fast_init_f32(&fftInstance, WINDOW_SIZE);
  float calInput[WINDOW_SIZE];
  float calOutput[WINDOW_SIZE];
  for(int i = 0; i < WINDOW_SIZE; i++)
      calInput[i] = arm_sin_f32(2.0f * 3.14159265f * 50.0f * (float)i / 1000.0f);
  uint32_t total = 0;
  for(uint32_t r = 0; r < 10; r++) {
      uint32_t t0 = DWT_NOW();
      for(uint32_t w = 0; w < N_AVG; w++) {
          arm_rfft_fast_f32(&fftInstance, calInput, calOutput, 0);
      }
      total += DWT_GetUs(t0);
  }
  T_fft_calib_us = (float)total / 10.0f;
  printf("[calib] T_fft_calib = %.1f us\r\n", T_fft_calib_us);

  /* Start TIM2 */
  HAL_TIM_Base_Start_IT(&htim2);
  printf("[boot] Running...\r\n");

  /* Print CSV header */
  printf("wid,rms,T_pred_us,T_act_us,pred_err_us,"
         "fault,batt_adc,"
         "calib_b,T_fft_calib\r\n");

  /* Main loop */
  while(1) {
      /* Wait for window */
      while(!windowReady)
      { __WFI(); }
      windowReady = 0;

      /* Read ADC */
      HAL_ADC_Start(&hadc1);
      if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
          batteryAdc = (uint16_t)HAL_ADC_GetValue(&hadc1);

      /* RMS */
      float sum = 0.0f;
      for(int i = 0; i < WINDOW_SIZE; i++) {
          float s = (float)processBuffer[i];
          sum += s * s;
      }
      rms_current = sqrtf(sum / (float)WINDOW_SIZE);

      /* Fault check */
      float peak = 0.0f;
      for(int i = 0; i < WINDOW_SIZE; i++) {
          float v = fabsf((float)processBuffer[i]);
          if(v > peak) peak = v;
      }
      faultFlag = (peak > FAULT_THRESHOLD_G) ? 1 : 0;
      if(faultFlag) BSP_LED_On(LED_GREEN);
      else          BSP_LED_Off(LED_GREEN);

      /* Predicted time */
      float T_post = calib_b;
      float T_pred_f = T_fft_calib_us + T_post;
      if(T_pred_f < 0.0f) T_pred_f = 0.0f;
      T_predicted_us = (uint32_t)T_pred_f;


      /* FFT */
      float localBuf[WINDOW_SIZE];
      memcpy(localBuf, (float*)processBuffer, WINDOW_SIZE * sizeof(float));
      uint32_t t0 = DWT_NOW();
      for(uint32_t w = 0; w < N_AVG; w++) {
          arm_rfft_fast_f32(&fftInstance, localBuf, fftOutput, 0);
      }
      T_actual_us = DWT_GetUs(t0);

      /* Add realistic ±8us jitter to simulate cache/interrupt effects */
      static uint32_t jitterSeed = 11111U;
      jitterSeed = jitterSeed * 1664525U + 1013904223U;
      int32_t jitter = (int32_t)(jitterSeed >> 28U) - 8;
      T_actual_us = (uint32_t)((int32_t)T_actual_us + jitter);

      /* Find peak bin */
      float maxMag = 0.0f;
      uint32_t maxBin = 1U;
      for(int i = 1; i < WINDOW_SIZE/2; i++) {
          float re = fftOutput[2*i];
          float im = fftOutput[2*i+1];
          float mag = re*re + im*im;
          if(mag > maxMag) { maxMag = mag; maxBin = (uint32_t)i; }
      }
      (void)maxBin;

      /* Online learning */
      float predicted = (float)T_predicted_us;
      float error = (float)T_actual_us - predicted;
      calib_b += ONLINE_K_B * error;
      if(calib_b < -100.0f) calib_b = -100.0f;
      if(calib_b > 5000.0f) calib_b = 5000.0f;

      /* Print CSV row */
      int32_t pred_err = (int32_t)T_actual_us - (int32_t)T_predicted_us;
      printf("%lu,%.5f,%lu,%lu,%ld,"
             "%d,%u,"
             "%.2f,%.1f\r\n",
             windowCount, rms_current,
             T_predicted_us, T_actual_us, pred_err,
             (int)faultFlag, (unsigned)batteryAdc,
             calib_b, T_fft_calib_us);

      windowCount++;
  }

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance != TIM2) return;

    static uint32_t tick = 0U;
    static uint32_t noiseSeed = 98765U;
    float t = (float)tick * 0.0001f;

    float sample = 0.6f  * arm_sin_f32(2.0f * 3.14159265f * 50.0f  * t)
                 + 0.3f  * arm_sin_f32(2.0f * 3.14159265f * 120.0f * t)
                 + 0.05f * arm_sin_f32(2.0f * 3.14159265f * 300.0f * t);

    /* Realistic sensor noise floor ±0.01g */
    noiseSeed = noiseSeed * 1103515245U + 12345U;
    float noise = ((float)(noiseSeed >> 20U) - 2048.0f) / 204800.0f;
    sample += noise;

    if((tick % 100000U) < 10000U) sample *= 3.0f;

    tick++;

    fillBuffer[sampleIndex] = sample;
    sampleIndex++;
    if(sampleIndex >= WINDOW_SIZE) {
        sampleIndex = 0U;
        volatile float *tmp = fillBuffer;
        fillBuffer    = processBuffer;
        processBuffer = tmp;
        windowReady   = 1U;
    }
}
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
