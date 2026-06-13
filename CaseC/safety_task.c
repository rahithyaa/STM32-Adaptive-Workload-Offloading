#include "app_globals.h"
#include "stm32g4xx_nucleo.h"
#include "tim.h"
#include <math.h>
#include <stdio.h>
#define FAULT_THRESHOLD_G 1.2f
void SafetyTaskFunc(void const *argument) {
	//printf("[SAF] Safety task running\r\n");
    for(;;) {
        osSemaphoreWait(windowReadySem, osWaitForever);
        //printf("[SAF] window received\r\n");
        float peak=0.0f;
        for(int i=0;i<WINDOW_SIZE;i++){
            float v=fabsf((float)processBuffer[i]);
            if(v>peak)peak=v;
        }
        g_peakAmplitude=peak;
        if(peak>FAULT_THRESHOLD_G){
            g_faultFlag=1;
            BSP_LED_On(LED_GREEN);
        } else {
            g_faultFlag=0;
            BSP_LED_Off(LED_GREEN);
        }
        osSemaphoreRelease(safetyDoneSem);
    }
}

