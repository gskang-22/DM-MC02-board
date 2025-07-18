/*
 * j60_10motor_task.h
 *
 *  Created on: Jul 4, 2025
 *      Author: YI MING
 */

#ifndef INC_J60_10MOTOR_TASK_H_
#define INC_J60_10MOTOR_TASK_H_
#include "j60_10.h"

void j60_10_TASK(void);
void J60_10_MOTOR_ONLINE_CHECK(void);

extern motor_t j60_motor[6];

#endif /* INC_J60_10MOTOR_TASK_H_ */
