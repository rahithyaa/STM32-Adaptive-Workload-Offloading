#ifndef APP_GLOBALS_H
#define APP_GLOBALS_H
#include "cmsis_os.h"
#include <stdint.h>
#define WINDOW_SIZE 256
extern float bufferA[WINDOW_SIZE];
extern float bufferB[WINDOW_SIZE];
extern volatile float  *fillBuffer;
extern volatile float  *processBuffer;
extern volatile uint16_t sampleIndex;
extern osSemaphoreId windowReadySem;
extern osSemaphoreId safetyDoneSem;
extern osThreadId metricsTaskHandle;
extern SemaphoreHandle_t windowReadySem_handle;
extern volatile uint8_t  g_faultFlag;
extern volatile float    g_peakAmplitude;
typedef enum { MODE_LOCAL=0, MODE_OFFLOAD=1, MODE_FORCED_LOCAL=2 } OffloadMode_t;
extern volatile OffloadMode_t g_mode;
extern volatile float    g_rms_current;
extern volatile uint32_t g_T_local_predicted_us;
extern volatile uint32_t g_T_local_actual_us;
extern volatile uint32_t g_T_tx_actual_us;
extern volatile uint32_t g_T_compute_python_us;
extern volatile uint32_t g_T_offload_actual_us;
extern volatile uint32_t g_lastRttUs;
extern volatile uint32_t g_missedDeadlines;
extern volatile uint32_t g_offloadAttempts;
extern volatile uint32_t g_offloadSuccesses;
extern volatile uint32_t g_windowCount;
extern volatile uint16_t g_batteryAdc;
extern volatile float    g_calib_b;
extern volatile float    g_T_fft_calib_us;
extern volatile uint32_t g_rttHistory[3];
extern volatile uint8_t  g_rttIdx;
extern volatile uint32_t g_lastLoggedWindow;
#endif
