/*
 * buzzer_task.h
 *
 *  Created on: Jul 3, 2025
 *      Author: YI MING
 */

#ifndef INC_BUZZER_TASK_H_
#define INC_BUZZER_TASK_H_

#include "cmsis_os.h"
#include "buzzer.h"

// Function prototypes
void BUZZER_TASK(void);
void BuzzerTask_HeartbeatBeep(void);

#endif /* INC_BUZZER_TASK_H_ */
