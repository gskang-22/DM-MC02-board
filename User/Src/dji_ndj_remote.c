/*
 * dji_ndj_remote.c
 *
 *  Created on: Jun 4, 2025
 *      Author: YI MING
 */
#include "dji_ndj_remote.h"

#define JOYSTICK_OFFSET 1024

dji_ndj6 dji_remote;

void DJI_NDJ_REMOTE_PROCESS(dji_ndj6 *remoter, uint8_t *buf)
{
	remoter->right_x = (buf[0] | buf[1] << 8) & 0x07FF;
	remoter->right_x -= JOYSTICK_OFFSET;
	remoter->right_y = (buf[1] >> 3 | buf[2] << 5) & 0x07FF;
	remoter->right_y -= JOYSTICK_OFFSET;
	remoter->left_x = (buf[2] >> 6 | buf[3] << 2
			| buf[4] << 10) & 0x07FF;
	remoter->left_x -= JOYSTICK_OFFSET;
	remoter->left_y = (buf[4] >> 1 | buf[5] << 7) & 0x07FF;
	remoter->left_y -= JOYSTICK_OFFSET;
	//Left switch position
	remoter->left_switch = ((buf[5] >> 4) & 0x000C) >> 2;
	remoter->right_switch = (buf[5] >> 4) & 0x0003;
	remoter->mouse_x = ((int16_t) buf[6] | ((int16_t) buf[7] << 8));
	remoter->mouse_y = ((int16_t) buf[8] | ((int16_t) buf[9] << 8));
	remoter->mouse_z = ((int16_t) buf[10] | ((int16_t) buf[11] << 8));
	remoter->mouse_hori += remoter->mouse_x;
	remoter->mouse_vert += remoter->mouse_y;
	remoter->mouse_left = (buf[12]);
	remoter->mouse_right = (buf[13]);
	remoter->keyboard_keys = (buf[14]);
	remoter->side_dial = ((int16_t) buf[16]) | ((int16_t) buf[17] << 8);
	remoter->side_dial -= JOYSTICK_OFFSET;
	remoter->last_time = 0;

	// Map individual keyboard keys
	remoter->W = !!(remoter->keyboard_keys & KEY_OFFSET_W);
	remoter->S = !!(remoter->keyboard_keys & KEY_OFFSET_S);
	remoter->A = !!(remoter->keyboard_keys & KEY_OFFSET_A);
	remoter->D = !!(remoter->keyboard_keys & KEY_OFFSET_D);
	remoter->Q = !!(remoter->keyboard_keys & KEY_OFFSET_Q);
	remoter->E = !!(remoter->keyboard_keys & KEY_OFFSET_E);
	remoter->Shift = !!(remoter->keyboard_keys & KEY_OFFSET_SHIFT);
	remoter->Ctrl = !!(remoter->keyboard_keys & KEY_OFFSET_CTRL);

//	if (remoter->keyboard_keys & KEY_OFFSET_T){
//			remoter->T = 1;
//		}else{
//			remoter->T = 0;
//		}
}

void DJI_NDJ_REMOTE_CHECK_ONLINE(){
	dji_remote.last_time += 1;

	if (dji_remote.last_time < 3){
		dji_remote.online = 1;
	}else{
		dji_remote.online = 0;
		DJI_NDJ_REMOTE_RESET(&dji_remote);
	}
}

void DJI_NDJ_REMOTE_RESET(dji_ndj6 *remoter){
	remoter->right_x = 0;
	remoter->right_y = 0;
	remoter->left_x = 0;
	remoter->left_y = 0;
	remoter->left_switch = 0;
	remoter->right_switch = 0;
	remoter->mouse_x = 0;
	remoter->mouse_y = 0;
	remoter->mouse_z = 0;
	remoter->mouse_hori = 0;
	remoter->mouse_vert = 0;
	remoter->mouse_left = 0;
	remoter->mouse_right = 0;
	remoter->keyboard_keys = 0;
	remoter->side_dial = 0;
	remoter->W = 0;
	remoter->S = 0;
	remoter->A = 0;
	remoter->D = 0;
	remoter->Q = 0;
	remoter->E = 0;
	remoter->Shift = 0;
	remoter->Ctrl = 0;
}
