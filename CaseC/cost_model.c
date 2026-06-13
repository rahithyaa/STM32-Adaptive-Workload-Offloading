#include "cost_model.h"
#include "app_globals.h"
#include "dwt_timer.h"
#include <string.h>
#define I_CPU_uA   26500UL
#define I_IDLE_uA   6450UL
#define I_UART_uA  26500UL
#define UART_BAUD 460800UL
#define DEADLINE_US 500000UL
#define CALIB_RUNS 10U
#define ONLINE_K_B  0.01f
#define N_AVG_CALIB 100U
arm_rfft_fast_instance_f32 g_fftInstance;
static float s_calInput[WINDOW_SIZE];
static float s_calOutput[WINDOW_SIZE];

void CostModel_Calibrate(void) {
    arm_rfft_fast_init_f32(&g_fftInstance, WINDOW_SIZE);
    for(int i = 0; i < WINDOW_SIZE; i++)
        s_calInput[i] = arm_sin_f32(
            2.0f*3.14159265f*50.0f*(float)i/8000.0f);

    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    uint32_t total = 0;
    for(uint32_t r = 0; r < 5U; r++) {
        static float calSpectrum[WINDOW_SIZE/2];
        memset(calSpectrum, 0, sizeof(calSpectrum));
        uint32_t t0 = DWT_NOW();
        for(uint32_t w = 0; w < N_AVG_CALIB; w++) {
            arm_rfft_fast_f32(&g_fftInstance,
                              s_calInput, s_calOutput, 0);
            for(int i=1; i<WINDOW_SIZE/2; i++) {
                float re = s_calOutput[2*i];
                float im = s_calOutput[2*i+1];
                calSpectrum[i] += re*re + im*im;
            }
        }
        total += DWT_GetUs(t0);
    }
    g_T_fft_calib_us = (float)total / 5.0f;
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}
uint32_t CostModel_TLocal_us(float rms, uint32_t overhead_us) {
	float T_pred_f = g_T_fft_calib_us + g_calib_b + (float)overhead_us;
	if(T_pred_f < 0.0f) T_pred_f = 0.0f;
	return (uint32_t)T_pred_f;
}
uint32_t CostModel_TOffload_us(void) {
    uint32_t bytes=(uint32_t)(WINDOW_SIZE*sizeof(float))+3U;
    uint32_t T_tx_us=(bytes*8UL*1000000UL)/UART_BAUD;
    uint32_t T_cloud=CostModel_GetRTT_mavg_us();
    if(T_cloud==0U)return DEADLINE_US+1U;
    return T_tx_us+T_cloud;
}
float CostModel_ELocal_nJ(uint32_t T_us) {
    return (float)T_us*(float)I_CPU_uA*3.3f/1000.0f;
}
float CostModel_EOffload_nJ(uint32_t T_tx_us, uint32_t T_cloud_us) {
    return (float)T_tx_us*(float)I_UART_uA*3.3f/1000.0f
          +(float)T_cloud_us*(float)I_IDLE_uA*3.3f/1000.0f;
}
void CostModel_Update(float rms, uint32_t T_actual_us) {
    float predicted=(float)CostModel_TLocal_us(rms,0U);
    float error=(float)T_actual_us-predicted;
    g_calib_b+=ONLINE_K_B*error;
    if(g_calib_b<-100.0f) g_calib_b=-100.0f;
    if(g_calib_b>5000.0f) g_calib_b=5000.0f;
}
void CostModel_UpdateRTT(uint32_t rtt_us) {
    g_rttHistory[g_rttIdx%3U]=rtt_us;
    g_rttIdx++;
    g_lastRttUs=rtt_us;
}
uint32_t CostModel_GetRTT_mavg_us(void) {
    uint8_t count=(g_rttIdx<3U)?(uint8_t)g_rttIdx:3U;
    if(count==0U)return 0U;
    uint32_t sum=0U;
    for(uint8_t i=0U;i<count;i++)sum+=g_rttHistory[i];
    return sum/(uint32_t)count;
}
