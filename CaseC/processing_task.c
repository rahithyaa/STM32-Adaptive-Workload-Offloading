#include "app_globals.h"
#include "cost_model.h"
#include "dwt_timer.h"
#include "main.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;
#define START_BYTE      0xAAU
#define END_BYTE        0xBBU
#define RESULT_SIZE     11U
#define OFFLOAD_TIMEOUT 800U
#define N_AVG           100U

static float s_fftOutput[WINDOW_SIZE];
static float s_powerSpectrum[WINDOW_SIZE/2];
static float s_localBuf[WINDOW_SIZE];    /* moved from stack to static */

static float LocalFFT(void) {
	//printf("[FFT] starting\r\n");
    memcpy(s_localBuf, (float*)processBuffer,
           WINDOW_SIZE * sizeof(float));

    memset(s_powerSpectrum, 0, sizeof(s_powerSpectrum));

    uint32_t t0 = DWT_NOW();
    for(uint32_t w = 0; w < N_AVG; w++) {
        arm_rfft_fast_f32(&g_fftInstance,
                          s_localBuf,
                          s_fftOutput, 0);
        for(int i = 1; i < WINDOW_SIZE/2; i++) {
            float re = s_fftOutput[2*i];
            float im = s_fftOutput[2*i+1];
            s_powerSpectrum[i] += re*re + im*im;
        }
    }
    g_T_local_actual_us = DWT_GetUs(t0);
    //printf("[FFT] loop done T=%lu us\r\n", g_T_local_actual_us);

    /*static uint32_t jitterSeed = 11111U;
    jitterSeed = jitterSeed * 1664525U + 1013904223U;
    int32_t jitter = (int32_t)(jitterSeed >> 28U) - 8;
    g_T_local_actual_us = (uint32_t)(
        (int32_t)g_T_local_actual_us + jitter);*/

    float maxMag = 0.0f;
    uint32_t maxBin = 1U;
    for(int i = 1; i < WINDOW_SIZE/2; i++) {
        s_powerSpectrum[i] /= (float)N_AVG;
        if(s_powerSpectrum[i] > maxMag) {
            maxMag = s_powerSpectrum[i];
            maxBin = (uint32_t)i;
        }
    }
    return (float)maxBin * 10000.0f / (float)WINDOW_SIZE;
}
static uint8_t OffloadWindow(float *freq_out) {
    static uint8_t txBuf[1 + WINDOW_SIZE*4 + 1];
    txBuf[0] = START_BYTE;
    memcpy(&txBuf[1], (float*)processBuffer,
           WINDOW_SIZE * sizeof(float));
    txBuf[WINDOW_SIZE*4 + 1] = END_BYTE;

    /* Start listening BEFORE transmitting */
    g_rxDone = 0;
    memset((uint8_t*)g_rxBuf, 0, RESULT_SIZE);
    HAL_UART_Receive_IT(&huart1,
                        (uint8_t*)g_rxBuf,
                        RESULT_SIZE);

    uint32_t t1 = DWT_NOW();
    HAL_UART_Transmit(&huart1, txBuf, sizeof(txBuf), 500U);
    g_T_tx_actual_us = DWT_GetUs(t1);
    g_offloadAttempts++;

    uint32_t tWait = DWT_NOW();
    while(!g_rxDone) {
        if(DWT_GetUs(tWait) > 500000U) break;
    }
    uint32_t rtt = DWT_GetUs(t1);   // ← measures TX + wait + RX together
    g_T_offload_actual_us = rtt;

    /*printf("RX done=%d: %02X %02X %02X %02X %02X "
           "%02X %02X %02X %02X %02X %02X\r\n",
           (int)g_rxDone,
           g_rxBuf[0],g_rxBuf[1],g_rxBuf[2],
           g_rxBuf[3],g_rxBuf[4],g_rxBuf[5],
           g_rxBuf[6],g_rxBuf[7],g_rxBuf[8],
           g_rxBuf[9],g_rxBuf[10]);*/

    if(!g_rxDone) return 0U;
    if(g_rxBuf[0] != START_BYTE ||
       g_rxBuf[10] != END_BYTE) return 0U;

    uint8_t csum = 0;
    for(int i = 1; i <= 8; i++) csum ^= g_rxBuf[i];
    if(csum != g_rxBuf[9]) return 0U;

    float freq, t_compute_ms;
    memcpy(&freq, (uint8_t*)&g_rxBuf[1], 4U);
    memcpy(&t_compute_ms, (uint8_t*)&g_rxBuf[5], 4U);
    if(freq < 0.0f || freq > 5000.0f) return 0U;

    *freq_out = freq;
    g_T_compute_python_us =
        (uint32_t)(t_compute_ms * 1000.0f);
    uint32_t clean_rtt = rtt > 24000U ? rtt - 24000U : rtt;
    g_lastRttUs = clean_rtt;
    CostModel_UpdateRTT(clean_rtt);
    g_offloadSuccesses++;
    return 1U;
}
void ProcessingTaskFunc(void const *argument) {
	//printf("[PRC] Processing task running\r\n");
    for(;;) {
        osSemaphoreWait(safetyDoneSem, osWaitForever);
        //printf("[PRC] safetyDone received\r\n");
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
        //printf("S%lu\r\n", g_windowCount);
        g_lastLoggedWindow = g_windowCount - 1U;
        osSemaphoreRelease(metricsSem);
    }
}
