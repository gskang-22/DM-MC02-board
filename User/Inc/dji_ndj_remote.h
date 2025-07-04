/*
 * dji_ndj_remote.h
 *
 *  Created on: Jun 4, 2025
 *      Author: YI MING
 */

#ifndef USER_INC_DJI_NDJ_REMOTE_H_
#define USER_INC_DJI_NDJ_REMOTE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "uart_bsp.h"
#include "string.h"
#include "usart.h"
#include "main.h"

#define KEY_OFFSET_W        ((uint16_t)0x01<<0)
#define KEY_OFFSET_S        ((uint16_t)0x01<<1)
#define KEY_OFFSET_A 		((uint16_t)0x01<<2)
#define KEY_OFFSET_D        ((uint16_t)0x01<<3)
#define KEY_OFFSET_Q        ((uint16_t)0x01<<6)
#define KEY_OFFSET_E        ((uint16_t)0x01<<7)
#define KEY_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define KEY_OFFSET_CTRL     ((uint16_t)0x01<<5)
#define KEY_OFFSET_T        ((uint16_t)0x01<<6)

typedef struct {
	/* Joysticks - Values range from -660 to 660 */
	    	int16_t right_x;
	    	int16_t right_y;
	    	int16_t left_x;
	    	int16_t left_y;
	    	/* Switches - Values range from 1 - 3 */
	    	int8_t left_switch;
	    	int8_t right_switch;
	    	/* Mouse movement - Values range from -32768 to 32767 */
	    	int16_t mouse_x;
	    	int16_t mouse_y;
	    	int16_t mouse_z;
	    	int32_t mouse_hori;
	    	int32_t mouse_vert;
	    	/* Mouse clicks - Values range from 0 to 1 */
	    	int8_t mouse_left;
	    	int8_t mouse_right;

	    	uint8_t W;
	    	uint8_t S;
	    	uint8_t A;
	    	uint8_t D;
	    	uint8_t Q;
	    	uint8_t E;
	    	uint8_t Shift;
	    	uint8_t Ctrl;
	    	/* Keyboard keys mapping
	    	 * Bit0 -- W 键
	    	 * Bit1 -- S 键
	    	 *	Bit2 -- A 键
	    	 *	Bit3 -- D 键
	    	 *	Bit4 -- Q 键
	    	 *	Bit5 -- E 键
	    	 *	Bit6 -- Shift 键
	    	 *	Bit7 -- Ctrl 键
	    	 *
	    	 */
	    	uint16_t keyboard_keys;
	    	int16_t side_dial;
	    	uint32_t last_time;
	    	uint8_t online;
} dji_ndj6;


void DJI_NDJ_REMOTE_PROCESS(dji_ndj6 *remoter, uint8_t *buf);
void DJI_NDJ_REMOTE_RESET(dji_ndj6 *remoter);
void DJI_NDJ_REMOTE_CHECK_ONLINE();

#ifdef __cplusplus
}
#endif

#endif /* USER_INC_DJI_NDJ_REMOTE_H_ */
