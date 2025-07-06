/*
 * buzzer_task.c
 *
 *  Created on: Jul 3, 2025
 *      Author: YI MING
 */

#include "buzzer_task.h"
#include "buzzer.h"
#include "cmsis_os.h"
#include "j60_10.h"
#include "j60_10motor_task.h"

// Private variables
static uint32_t heartbeat_counter = 0;
extern motor_t j60_motor[6];

/**
 * @brief Buzzer heartbeat beep function - plays a beep every 1 second
 * This function provides audible confirmation that the controller is alive
 */
void BuzzerTask_HeartbeatBeep(void) {
    // Simple heartbeat beep pattern
    Buzzer_PlayTone(NOTE_BEEP, 20);  // Short 50ms beep
    osDelay(100);                     // Wait for beep to finish
}

void J60_Motor_Offline_Beep(void) {
    // Simple heartbeat beep pattern
	if (j60_motor[0].para.online == 0){
		Buzzer_PlayTone(NOTE_ALERT, 30);
	}
      // Short 50ms beep
    osDelay(500);                     // Wait for beep to finish
}
/**
 * @brief Main buzzer task function (follows PC_MCU_UART_TASK pattern)
 * This function runs the buzzer task loop with 1Hz heartbeat
 */
void BUZZER_TASK(void) {
    // Initialize the buzzer
    Buzzer_Init();
    
    // Set a moderate volume
    Buzzer_SetVolume(30); // 30% volume to avoid being too loud
    
    // Startup sound - quick melody to indicate system start
    const uint32_t startup_freq[] = {NOTE_C4, NOTE_E4, NOTE_G4};
    const uint32_t startup_dur[] = {100, 100, 150};
    Buzzer_PlayMelody(startup_freq, startup_dur, 3);
    
    osDelay(500); // Wait before starting heartbeat
    
    // Infinite loop for heartbeat beep
    for (;;) {
        // Play heartbeat beep every 1 second
        BuzzerTask_HeartbeatBeep();
        J60_Motor_Offline_Beep();
        
        // Increment heartbeat counter
        heartbeat_counter++;
        
        // Wait 1 second before next heartbeat
        osDelay(2000);
    }
}


