/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pc_mcu_uart.h"
#include "usart.h"
#include "string.h"
#include "imu_task.h"
#include "buzzer_task.h"
#include "j60_10motor_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t tx_data[8] = {0};
int16_t current_cmd = 0; // example: 1000 units of current
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId PC_MCU_TASKHandle;
uint32_t PC_MCU_TASKBuffer[ 128 ];
osStaticThreadDef_t PC_MCU_TASKControlBlock;
osThreadId imuTaskHandle;
uint32_t imuTaskBuffer[ 128 ];
osStaticThreadDef_t imuTaskControlBlock;
osThreadId buzzerTaskHandle;
uint32_t buzzerTaskBuffer[ 128 ];
osStaticThreadDef_t buzzerTaskControlBlock;
osThreadId J60_10TaskHandle;
uint32_t J60_10TaskBuffer[ 128 ];
osStaticThreadDef_t J60_10TaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void PC_MCU_ENTRY(void const * argument);
void ImuTask_Entry(void const * argument);
void buzzerEntry(void const * argument);
void J60_10Entry(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of PC_MCU_TASK */
  osThreadStaticDef(PC_MCU_TASK, PC_MCU_ENTRY, osPriorityNormal, 0, 128, PC_MCU_TASKBuffer, &PC_MCU_TASKControlBlock);
  PC_MCU_TASKHandle = osThreadCreate(osThread(PC_MCU_TASK), NULL);

  /* definition and creation of imuTask */
  osThreadStaticDef(imuTask, ImuTask_Entry, osPriorityAboveNormal, 0, 128, imuTaskBuffer, &imuTaskControlBlock);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* definition and creation of buzzerTask */
  osThreadStaticDef(buzzerTask, buzzerEntry, osPriorityBelowNormal, 0, 128, buzzerTaskBuffer, &buzzerTaskControlBlock);
  buzzerTaskHandle = osThreadCreate(osThread(buzzerTask), NULL);

  /* definition and creation of J60_10Task */
  osThreadStaticDef(J60_10Task, J60_10Entry, osPriorityNormal, 0, 128, J60_10TaskBuffer, &J60_10TaskControlBlock);
  J60_10TaskHandle = osThreadCreate(osThread(J60_10Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */

//	Enable_J60_Motor(1); // enable motor with joint ID 2
//	Enable_J60_Motor(2);

  /* Infinite loop */
  for(;;)
  {


//	  uint16_t pos_u16 = MAP_F32_TO_U16(target_pos, -40.0f, 40.0f);
//	  uint16_t vel_u14 = MAP_F32_TO_U14(target_vel, -40.0f, 40.0f);
//	  uint16_t kp_u10  = MAP_F32_TO_U10(target_kp, 0.0f, 1023.0f);
//	  uint8_t  kd_u8   = MAP_F32_TO_U8(target_kd, 0.0f, 51.0f);
//	  uint16_t torque_u16 = MAP_F32_TO_U16(target_torque, -40.0f, 40.0f);
//
//	  uint8_t cmd_data[8] = {0};

	  // Pack bits into cmd_data[]
//	  cmd_data[0] = (pos_u16 >> 8) & 0xFF;
//	  cmd_data[1] = pos_u16 & 0xFF;
//
//	  cmd_data[2] = (vel_u14 >> 6) & 0xFF;
//	  cmd_data[3] = ((vel_u14 & 0x3F) << 2) | ((kp_u10 >> 8) & 0x03);
//	  cmd_data[4] = kp_u10 & 0xFF;
//
//	  cmd_data[5] = kd_u8;
//
//	  cmd_data[6] = (torque_u16 >> 8) & 0xFF;
//	  cmd_data[7] = torque_u16 & 0xFF;
//
//	  // Send CAN control command
//	  uint32_t motor_ctrl_id = (0x01 & 0x1F) | (4 << 5); // joint_id = 1, CMD = 4
//	  fdcanx_send_data(&hfdcan1, motor_ctrl_id, cmd_data, 8);
//	  tx_data[0] = (current_cmd >> 8) & 0xFF;
//	  tx_data[1] = current_cmd & 0xFF;
	  //	  tx_data[2] = current_cmd >> 8;
	  //	  tx_data[3] = current_cmd;
	  //	  tx_data[4] = current_cmd >> 8;
	  //	  tx_data[5] = current_cmd;
	  //	  tx_data[6] = current_cmd >> 8;
	  //	  tx_data[7] = current_cmd;
	  //	  tx_data[2] = 0;
	  //	  tx_data[3] = 0;
	  //	  tx_data[4] = 0;
	  //	  tx_data[5] = 0;
	  //	  tx_data[6] = 0;
	  //	  tx_data[7] = 0;

//	  uint32_t can_id = (0x01 & 0x1F) | (2 << 5); // CAN_CMD_MOTOR_ENABLE = 2
//	  uint8_t dummy_data[1] = {0}; // FDCAN requires non-null data pointer
//
//	  fdcanx_send_data(&hfdcan1, can_id, dummy_data, 0);
//	  fdcanx_send_data(&hfdcan1, 0x1FF, tx_data, 8);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_PC_MCU_ENTRY */
/**
* @brief Function implementing the PC_MCU_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PC_MCU_ENTRY */
void PC_MCU_ENTRY(void const * argument)
{
  /* USER CODE BEGIN PC_MCU_ENTRY */
  /* Infinite loop */
  for(;;)
  {
	PC_MCU_UART_TASK();
    osDelay(1);
  }
  /* USER CODE END PC_MCU_ENTRY */
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
__weak void ImuTask_Entry(void const * argument)
{
  /* USER CODE BEGIN ImuTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ImuTask_Entry */
}

/* USER CODE BEGIN Header_buzzerEntry */
/**
* @brief Function implementing the buzzerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_buzzerEntry */
void buzzerEntry(void const * argument)
{
  /* USER CODE BEGIN buzzerEntry */
  /* Infinite loop */
  for(;;)
  {
    BUZZER_TASK();
    osDelay(1);
  }
  /* USER CODE END buzzerEntry */
}

/* USER CODE BEGIN Header_J60_10Entry */
/**
* @brief Function implementing the J60_10Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_J60_10Entry */
void J60_10Entry(void const * argument)
{
  /* USER CODE BEGIN J60_10Entry */
  /* Infinite loop */
  for(;;)
  {
	j60_10_TASK();
    osDelay(1);
  }
  /* USER CODE END J60_10Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
