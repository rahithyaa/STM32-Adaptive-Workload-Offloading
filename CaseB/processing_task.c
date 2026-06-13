#include "app_globals.h"
#include "cost_model.h"
#include "dwt_timer.h"
#include "main.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
extern UART_HandleTypeDef huart1;
#define START_BYTE 0xAAU
#define END_BYTE   0xBBU
#define RESULT_SIZE 11U
#define OFFLOAD_TIMEOUT 200U
static float s_fftOutput[WINDOW_SIZE];
#define N_AVG 100U

static float LocalFFT(void) {
    float localBuf[WINDOW_SIZE];
    memcpy(localBuf, (float*)processBuffer, WINDOW_SIZE * sizeof(float));
    uint32_t t0=DWT_NOW();
    for(uint32_t w = 0; w < N_AVG; w++) {
        arm_rfft_fast_f32(&g_fftInstance, localBuf, s_fftOutput, 0);
    }
    g_T_local_actual_us=DWT_GetUs(t0);

    float maxMag=0.0f; uint32_t maxBin=1U;
    for(int i=1;i<WINDOW_SIZE/2;i++){
        float re=s_fftOutput[2*i],im=s_fftOutput[2*i+1];
        float mag=re*re+im*im;
        if(mag>maxMag){maxMag=mag;maxBin=(uint32_t)i;}
    }
    return (float)maxBin*10000.0f/(float)WINDOW_SIZE;
}
static uint8_t OffloadWindow(float *freq_out) {
    uint8_t hdr=START_BYTE,ftr=END_BYTE;
    uint32_t t1=DWT_NOW();
    HAL_UART_Transmit(&huart1,&hdr,1U,10U);
    HAL_UART_Transmit(&huart1,(uint8_t*)processBuffer,WINDOW_SIZE*(uint32_t)sizeof(float),200U);
    HAL_UART_Transmit(&huart1,&ftr,1U,10U);
    g_T_tx_actual_us=DWT_GetUs(t1);
    g_offloadAttempts++;
    uint8_t rx[RESULT_SIZE]; memset(rx,0,sizeof(rx));
    HAL_StatusTypeDef st=HAL_UART_Receive(&huart1,rx,RESULT_SIZE,OFFLOAD_TIMEOUT);
    uint32_t rtt=DWT_GetUs(t1);
    if(st!=HAL_OK||rx[0]!=START_BYTE||rx[10]!=END_BYTE)return 0U;
    uint8_t csum=0; for(int i=1;i<=8;i++)csum^=rx[i];
    if(csum!=rx[9])return 0U;
    float freq,t_compute_ms;
    memcpy(&freq,&rx[1],4U); memcpy(&t_compute_ms,&rx[5],4U);
    if(freq<0.0f||freq>5000.0f)return 0U;
    *freq_out=freq;
    g_T_compute_python_us=(uint32_t)(t_compute_ms*1000.0f);
    g_T_offload_actual_us=rtt;
    CostModel_UpdateRTT(rtt);
    g_offloadSuccesses++;
    return 1U;
}
void ProcessingTaskFunc(void const *argument) {
    for(;;) {
        osSemaphoreWait(safetyDoneSem, osWaitForever);
        float sum=0.0f;
        for(int i=0;i<WINDOW_SIZE;i++){float s=(float)processBuffer[i];sum+=s*s;}
        g_rms_current=sqrtf(sum/(float)WINDOW_SIZE);
        float freq = 0.0f;
        uint8_t ranLocalFFT = 0;
        if(g_mode == MODE_OFFLOAD) {
            if(!OffloadWindow(&freq)) {
                freq = LocalFFT();
                g_missedDeadlines++;
                ranLocalFFT = 1;
            }
        } else {
            freq = LocalFFT();
            ranLocalFFT = 1;
        }

        if(ranLocalFFT) {
            CostModel_Update(g_rms_current, g_T_local_actual_us);
        }
        g_lastLoggedWindow = g_windowCount - 1U;
        osSignalSet(metricsTaskHandle,0x01U);
    }
}
