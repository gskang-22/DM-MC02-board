#ifndef BOARD_LIB_H_
#define BOARD_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx.h"
#include "stdint.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "usart.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#include "bsp_queue.h"
#include "board_settings.h"
#include "typedefs.h"
#include "bsp_dbus_input.h"
#include "bsp_usart.h"
#include "bsp_referee.h"
#include "bsp_can.h"
#include "bsp_led.h"
#include "bsp_oled.h"
#include "bsp_imu.h"
#include "bsp_buzzer.h"
#include "bsp_gpio.h"
//#include "usbd_cdc_if.h"
#include "bsp_usb_redir.h"
#include "bsp_micros_timer.h"
#include "bsp_damiao.h"
#include "CRC8_CRC16.h"


#ifdef __cplusplus
}
#endif

#endif
