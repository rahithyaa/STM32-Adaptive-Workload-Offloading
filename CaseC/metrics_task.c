#include "app_globals.h"
#include "main.h"
#include <stdio.h>
extern ADC_HandleTypeDef hadc1;
static uint16_t ReadBatteryADC(void) {
    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1,10U)==HAL_OK)
        return (uint16_t)HAL_ADC_GetValue(&hadc1);
    return 0U;
}
void MetricsTaskFunc(void const *argument) {
	//printf("[MET] Metrics task running\r\n");
    printf("wid,rms,T_pred_us,T_act_us,pred_err_us,"
           "T_tx_us,T_py_us,RTT_us,"
           "mode,fault,batt_adc,"
           "calib_b,T_fft_calib,"
           "missed,attempts,successes\r\n");
    for(;;) {
    	osSemaphoreWait(metricsSem, osWaitForever);
        g_batteryAdc=ReadBatteryADC();
        int32_t pred_err=(int32_t)g_T_local_actual_us-(int32_t)g_T_local_predicted_us;
        printf("%lu,%.5f,%lu,%lu,%ld,"
               "%lu,%lu,%lu,"
               "%d,%d,%u,"
               "%.2f,%.1f,"
               "%lu,%lu,%lu\r\n",
			   g_lastLoggedWindow,g_rms_current,
               g_T_local_predicted_us,g_T_local_actual_us,pred_err,
               g_T_tx_actual_us,g_T_compute_python_us,g_lastRttUs,
               (int)g_mode,(int)g_faultFlag,(unsigned)g_batteryAdc,
               g_calib_b,g_T_fft_calib_us,
               g_missedDeadlines,g_offloadAttempts,g_offloadSuccesses);
    }
}
