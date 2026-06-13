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
#include "cmsis_os.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_globals.h"
#include "dwt_timer.h"
#include "cost_model.h"
#include "arm_math.h"
#include <stdio.h>
#include <string.h>
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

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
#define MPU6050_ADDR    (0x69U << 1U)
#define REG_PWR_MGMT_1  0x6BU
#define REG_ACCEL_CFG   0x1CU
#define REG_ACCEL_XOUT  0x3BU
#define REG_WHO_AM_I    0x75U

void SafetyTaskFunc(void const *argument);
void ProcessingTaskFunc(void const *argument);
void DecisionTaskFunc(void const *argument);
void MetricsTaskFunc(void const *argument);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();

    BspCOMInit.BaudRate   = 460800;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
    BSP_COM_Init(COM1, &BspCOMInit);
    BSP_LED_Init(LED_GREEN);

    HAL_TIM_Base_Stop_IT(&htim2);
    CostModel_Calibrate();
    g_T_local_predicted_us = (uint32_t)g_T_fft_calib_us;
    printf("[calib] T_fft_calib = %.1f us\r\n", g_T_fft_calib_us);

    /* Seed RTT — realistic RX-only latency ~1ms */
    CostModel_UpdateRTT(500U);
    CostModel_UpdateRTT(400U);
    CostModel_UpdateRTT(600U);
    printf("[rtt] RTT seeded avg=%lu us\r\n",
           CostModel_GetRTT_mavg_us());

    osSemaphoreDef(winSem);
    windowReadySem = osSemaphoreCreate(osSemaphore(winSem), 1U);
    osSemaphoreWait(windowReadySem, 0U);

    osSemaphoreDef(safSem);
    safetyDoneSem = osSemaphoreCreate(osSemaphore(safSem), 1U);
    osSemaphoreWait(safetyDoneSem, 0U);
    windowReadySem_handle = (SemaphoreHandle_t)windowReadySem;

    osSemaphoreDef(metSem);
    metricsSem = osSemaphoreCreate(osSemaphore(metSem), 1U);
    osSemaphoreWait(metricsSem, 0U);

    osThreadDef(SafTask, SafetyTaskFunc,
                osPriorityHigh, 0U, 512U);
    osThreadCreate(osThread(SafTask), NULL);

    osThreadDef(PrcTask, ProcessingTaskFunc,
                osPriorityNormal, 0U, 2048U);
    osThreadCreate(osThread(PrcTask), NULL);

    osThreadDef(DecTask, DecisionTaskFunc,
                osPriorityNormal, 0U, 512U);
    osThreadCreate(osThread(DecTask), NULL);

    osThreadDef(MetTask, MetricsTaskFunc,
                osPriorityNormal, 0U, 1024U);
    metricsTaskHandle = osThreadCreate(osThread(MetTask), NULL);

    HAL_TIM_Base_Start_IT(&htim2);
    printf("[boot] Scheduler starting...\r\n");

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (460800, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 460800;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        g_rxDone = 1;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance != TIM2) return;

    static uint32_t tick = 0U;
    static uint32_t noiseSeed = 98765U;
    float t = (float)tick * 0.0001f;

    float sample = 0.6f  * arm_sin_f32(2.0f * 3.14159265f * 50.0f  * t)
                 + 0.3f  * arm_sin_f32(2.0f * 3.14159265f * 120.0f * t)
                 + 0.05f * arm_sin_f32(2.0f * 3.14159265f * 300.0f * t);

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
        g_windowCount++;
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(windowReadySem_handle, &woken);
        portYIELD_FROM_ISR(woken);
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
