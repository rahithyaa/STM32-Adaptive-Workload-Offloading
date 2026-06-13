#ifndef COST_MODEL_H
#define COST_MODEL_H
#include <stdint.h>
#include "arm_math.h"
void     CostModel_Calibrate(void);
uint32_t CostModel_TLocal_us(float rms, uint32_t overhead_us);
uint32_t CostModel_TOffload_us(void);
float    CostModel_ELocal_nJ(uint32_t T_us);
float    CostModel_EOffload_nJ(uint32_t T_tx_us, uint32_t T_cloud_us);
void     CostModel_Update(float rms, uint32_t T_actual_us);
void     CostModel_UpdateRTT(uint32_t rtt_us);
uint32_t CostModel_GetRTT_mavg_us(void);
extern arm_rfft_fast_instance_f32 g_fftInstance;
#endif
