/*
 * supercap_comm_task.h
 *
 *  Created on: Jul 16, 2025
 *      Author: gskan
 */

#ifndef TASKS_INC_SUPERCAP_COMM_TASK_H_
#define TASKS_INC_SUPERCAP_COMM_TASK_H_

#include <stm32h7xx_hal.h>
#include <main.h>

// Energy threshold below which supercap usage is disabled (%)
#define SUPERCAP_DISABLE_THRESHOLD    20
// Energy threshold above which supercap usage is re-enabled (%)
#define SUPERCAP_ENABLE_THRESHOLD     40
#define SUPERCAP_TIMEOUT				5000 // ms

void supercap_comm_task(void *argument);
void txHeaderConfig(FDCAN_TxHeaderTypeDef* TxHeader);
void supercapISR(uint8_t* rxdata);

//extern SuperCapCommThread* SuperCapCommInstance;


#endif /* TASKS_INC_SUPERCAP_COMM_TASK_H_ */
