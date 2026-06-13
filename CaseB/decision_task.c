#include "app_globals.h"
#include "cost_model.h"
#define DEADLINE_US 500000UL
#define UART_BAUD   460800UL
void DecisionTaskFunc(void const *argument) {
    static uint32_t lastOverhead_us=0U;
    for(;;) {
        osDelay(27U);
        if(g_faultFlag){g_mode=MODE_FORCED_LOCAL;continue;}
        float rms=g_rms_current;
        uint32_t T_local=CostModel_TLocal_us(rms, 0U);
        uint32_t T_offload=CostModel_TOffload_us();
        uint32_t bytes=(uint32_t)(WINDOW_SIZE*sizeof(float))+3U;
        uint32_t T_tx_us=(bytes*8UL*1000000UL)/UART_BAUD;
        uint32_t T_cloud=(T_offload>T_tx_us)?T_offload-T_tx_us:0U;
        float E_local=CostModel_ELocal_nJ(T_local);
        float E_offload=CostModel_EOffload_nJ(T_tx_us,T_cloud);
        g_T_local_predicted_us=T_local;
        if(CostModel_GetRTT_mavg_us()==0U) g_mode=MODE_LOCAL;
        else if(T_offload>=DEADLINE_US) g_mode=MODE_LOCAL;
        else if(E_offload<E_local) g_mode=MODE_OFFLOAD;
        else g_mode=MODE_LOCAL;
    }
}
