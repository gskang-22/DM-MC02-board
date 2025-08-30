/*
 * bsp_dbus_input.c
 *
 * Created on: Mar 2 2020
 *     Author: Raghav Bhardwaj
 */
#include "board_lib.h"
#include "robot_config.h"
#include "bsp_dbus_input.h"

//#define JOYSTICK_OFFSET 1024
//extern TaskHandle_t control_input_task_handle;
//
//uint8_t remote_raw_data[REMOTE_DATA_SIZE] = {0};
//remote_cmd_t g_remote_cmd = { 0 };
//
//
//void dbus_remote_ISR(DMA_HandleTypeDef *hdma) {
//	g_remote_cmd.right_x = (remote_raw_data[0] | remote_raw_data[1] << 8) & 0x07FF;
//	g_remote_cmd.right_x -= JOYSTICK_OFFSET;
//	g_remote_cmd.right_y = (remote_raw_data[1] >> 3 | remote_raw_data[2] << 5) & 0x07FF;
//	g_remote_cmd.right_y -= JOYSTICK_OFFSET;
//	g_remote_cmd.left_x = (remote_raw_data[2] >> 6 | remote_raw_data[3] << 2
//			| remote_raw_data[4] << 10) & 0x07FF;
//	g_remote_cmd.left_x -= JOYSTICK_OFFSET;
//	g_remote_cmd.left_y = (remote_raw_data[4] >> 1 | remote_raw_data[5] << 7) & 0x07FF;
//	g_remote_cmd.left_y -= JOYSTICK_OFFSET;
//	//Left switch position
//	g_remote_cmd.left_switch = ((remote_raw_data[5] >> 4) & 0x000C) >> 2;
//	g_remote_cmd.right_switch = (remote_raw_data[5] >> 4) & 0x0003;
//	g_remote_cmd.mouse_x = ((int16_t) remote_raw_data[6] | ((int16_t) remote_raw_data[7] << 8));
//	g_remote_cmd.mouse_y = ((int16_t) remote_raw_data[8] | ((int16_t) remote_raw_data[9] << 8));
//	g_remote_cmd.mouse_z = ((int16_t) remote_raw_data[10] | ((int16_t) remote_raw_data[11] << 8));
//	g_remote_cmd.mouse_hori += g_remote_cmd.mouse_x;
//	g_remote_cmd.mouse_vert += g_remote_cmd.mouse_y;
//	g_remote_cmd.mouse_left = (remote_raw_data[12]);
//	g_remote_cmd.mouse_right = (remote_raw_data[13]);
//	g_remote_cmd.keyboard_keys = (remote_raw_data[14]);
//	g_remote_cmd.side_dial = ((int16_t) remote_raw_data[16]) | ((int16_t) remote_raw_data[17] << 8);
//	g_remote_cmd.side_dial -= JOYSTICK_OFFSET;
//	g_remote_cmd.last_time = HAL_GetTick();
//
//	BaseType_t xHigherPriorityTaskWoken;
//	xHigherPriorityTaskWoken = pdFALSE;
//	vTaskNotifyGiveFromISR(control_input_task_handle, &xHigherPriorityTaskWoken);
//	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}
//
///**
// * This function starts the circular DMA for receiving on a UART port. It is specifically
// * written for the UART1 port for DBUS interface from the controller.
// */
//HAL_StatusTypeDef dbus_remote_start()
//{
//	uint8_t *pData = remote_raw_data;
//	UART_HandleTypeDef *huart = DBUS_UART;
//	uint32_t *tmp;
//
//	/* Check that a Rx process is not already ongoing */
//	if (huart->RxState == HAL_UART_STATE_READY) {
//		if ((pData == NULL) || (REMOTE_DATA_SIZE == 0U)) {
//			return HAL_ERROR;
//		}
//
//		/* Process Locked */
//		__HAL_LOCK(huart);
//
//		huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
//		huart->pRxBuffPtr = pData;
//		huart->RxXferSize = REMOTE_DATA_SIZE;
//
//		huart->ErrorCode = HAL_UART_ERROR_NONE;
//		huart->RxState = HAL_UART_STATE_BUSY_RX;
//
//		/* Set the UART DMA transfer complete callback */
//		huart->hdmarx->XferCpltCallback = dbus_remote_ISR;
//
//		/* Set the DMA abort callback */
//		huart->hdmarx->XferAbortCallback = NULL;
//
//		/* Enable the DMA stream */
//		tmp = (uint32_t *)&pData;
//		HAL_DMA_Start_IT(huart->hdmarx, (uint32_t)&huart->Instance->RDR, (uint32_t)tmp, REMOTE_DATA_SIZE);
//
//		/* Clear the Overrun flag just before enabling the DMA Rx request: can be mandatory for the second transfer */
//		__HAL_UART_CLEAR_OREFLAG(huart);
//
//		/* Process Unlocked */
//		__HAL_UNLOCK(huart);
//
//		/* Enable the UART Parity Error Interrupt */
//		SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);
//
//		/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
//		SET_BIT(huart->Instance->CR3, USART_CR3_EIE);
//
//		/* Enable the DMA transfer for the receiver request by setting the DMAR bit
//		in the UART CR3 register */
//		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
//
//		if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
//		{
//			__HAL_UART_CLEAR_IDLEFLAG(huart);
//			ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
//		}
//		else
//		{
//			/* In case of errors already pending when reception is started,
//			   Interrupts may have already been raised and lead to reception abortion.
//			   (Overrun error for instance).
//			   In such case Reception Type has been reset to HAL_UART_RECEPTION_STANDARD. */
//			return HAL_ERROR;
//		}
//		return HAL_OK;
//	} else {
//		return HAL_BUSY;
//	}
//}

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdbool.h>
#include <string.h>

typedef struct {
	uint8_t sof_1;
	uint8_t sof_2;
	uint64_t ch_0 :11;
	uint64_t ch_1 :11;
	uint64_t ch_2 :11;
	uint64_t ch_3 :11;
	uint64_t mode_sw :2;
	uint64_t pause :1;
	uint64_t fn_1 :1;
	uint64_t fn_2 :1;
	uint64_t wheel :11;
	uint64_t trigger :1;

	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_left :2;
	uint8_t mouse_right :2;
	uint8_t mouse_middle :2;
	uint16_t key;
	uint16_t crc16;
} remote_data_t;

#define DJI_FRAME_SIZE 21  // DJI remote frame size (bytes)

//TaskHandle_t dji_processing_task_handle;  // task to notify when a new frame is ready
extern TaskHandle_t control_input_task_handle;

uint8_t remote_raw_data[REMOTE_DATA_SIZE] = { 0 };
remote_data_t g_remote_cmd = { 0 };

static uint16_t crc16_init = 0xffff;
static const uint16_t crc16_tab[256] = { 0x0000, 0x1189, 0x2312, 0x329b, 0x4624,
		0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5,
		0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7,
		0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
		0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a,
		0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a,
		0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9,
		0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f,
		0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868,
		0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528,
		0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb,
		0xaa72, 0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
		0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387,
		0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46,
		0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a,
		0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb,
		0x4e64, 0x5fed, 0x6d76, 0x7cff, 0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad,
		0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c,
		0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c,
		0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
		0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3,
		0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785,
		0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956,
		0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416,
		0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1,
		0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3,
		0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70,
		0x1ff9, 0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
		0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78 };

/**
 * @brief Get the crc16 checksum
 *
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len,
		uint16_t crc16) {
	uint8_t data;

	if (p_msg == NULL) {
		return 0xffff;
	}

	while (len--) {
		data = *p_msg++;
		(crc16) = ((uint16_t) (crc16) >> 8)
				^ crc16_tab[((uint16_t) (crc16) ^ (uint16_t) (data)) & 0x00ff];
	}

	return crc16;
}

/**
 * @brief crc16 verify function
 *
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len) {
	uint16_t w_expected = 0;

	if ((p_msg == NULL) || (len <= 2)) {
		return false;
	}
	w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

	return ((w_expected & 0xff) == p_msg[len - 2]
			&& ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}

// ---------------- DMA ISR ----------------
//void dji_half_cplt_isr(DMA_HandleTypeDef *hdma)
//{
//    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)hdma->Parent;
//    uint8_t *buf = huart->pRxBuffPtr;
//
//    // process first half of buffer
//    for(uint16_t i=0; i<huart->RxXferSize/2; i++)
//        queue_append_byte(dji_UART_queue, buf[i]);
//}

void dji_full_cplt_isr(DMA_HandleTypeDef *hdma) {
	UART_HandleTypeDef *huart = DBUS_UART;
	uint8_t *buf = huart->pRxBuffPtr;

	if (verify_crc16_check_sum(buf, huart->RxXferSize / 2)) {

		{
			// Assuming remote_raw_data points to DMA buffer (18 bytes for DJI remote)

			g_remote_cmd.ch_0 = (remote_raw_data[0] | remote_raw_data[1] << 8)
					& 0x07FF;
			g_remote_cmd.ch_1 = ((remote_raw_data[1] >> 3)
					| remote_raw_data[2] << 5) & 0x07FF;
			g_remote_cmd.ch_2 = ((remote_raw_data[2] >> 6)
					| remote_raw_data[3] << 2 | remote_raw_data[4] << 10)
					& 0x07FF;
			g_remote_cmd.ch_3 = (remote_raw_data[4] >> 1
					| remote_raw_data[5] << 7) & 0x07FF;

			// Mode switch and other 1-2 bit fields
			g_remote_cmd.mode_sw = (remote_raw_data[5] >> 4) & 0x0003;
			g_remote_cmd.pause = (remote_raw_data[5] >> 6) & 0x0001;
			g_remote_cmd.fn_1 = (remote_raw_data[5] >> 7) & 0x0001;
			g_remote_cmd.fn_2 = (remote_raw_data[6]) & 0x0001; // adjust depending on layout
			g_remote_cmd.wheel = ((remote_raw_data[6] >> 1)
					| remote_raw_data[7] << 7) & 0x07FF;
			g_remote_cmd.trigger = (remote_raw_data[8] & 0x01);

			// Mouse
			g_remote_cmd.mouse_x = (int16_t) (remote_raw_data[9]
					| (remote_raw_data[10] << 8));
			g_remote_cmd.mouse_y = (int16_t) (remote_raw_data[11]
					| (remote_raw_data[12] << 8));
			g_remote_cmd.mouse_z = (int16_t) (remote_raw_data[13]
					| (remote_raw_data[14] << 8));

			// Mouse buttons
			g_remote_cmd.mouse_left = remote_raw_data[15] & 0x03;
			g_remote_cmd.mouse_right = (remote_raw_data[15] >> 2) & 0x03;
			g_remote_cmd.mouse_middle = (remote_raw_data[15] >> 4) & 0x03;

			// Keyboard keys
			g_remote_cmd.key = remote_raw_data[16] | (remote_raw_data[17] << 8);

			// CRC16
			g_remote_cmd.crc16 = remote_raw_data[18]
					| (remote_raw_data[19] << 8);
		}

		// notify processing task
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(control_input_task_handle,
				&xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

//HAL_StatusTypeDef dbus_remote_start()

// ---------------- UART init ----------------
HAL_StatusTypeDef dbus_remote_start() {
	uint8_t *pData = remote_raw_data;
	UART_HandleTypeDef *huart = DBUS_UART;

	if (huart->RxState != HAL_UART_STATE_READY)
		return HAL_BUSY;

	if (!pData)
		return HAL_ERROR;

	__HAL_LOCK(huart);

	huart->pRxBuffPtr = pData;
	huart->RxXferSize = REMOTE_DATA_SIZE;
	huart->ErrorCode = HAL_UART_ERROR_NONE;
	huart->RxState = HAL_UART_STATE_BUSY_RX;

	huart->hdmarx->XferCpltCallback = dji_full_cplt_isr;
//    huart->hdmarx->XferHalfCpltCallback = dji_half_cplt_isr;
	huart->hdmarx->XferAbortCallback = NULL;

	HAL_DMA_Start_IT(huart->hdmarx, (uint32_t) &huart->Instance->RDR,
			(uint32_t) pData,
			REMOTE_DATA_SIZE);

	__HAL_UART_CLEAR_OREFLAG(huart);
	__HAL_UNLOCK(huart);

	SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);
	SET_BIT(huart->Instance->CR3, USART_CR3_EIE);
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

	return HAL_OK;
}

// ---------------- Frame parser ----------------
//bool dji_parse_frame(remote_data_t *frame)
//{
//    if(!verify_crc16_check_sum((uint8_t*)frame, DJI_FRAME_SIZE))
//        return false;
//
//    // optionally process channels, mouse, keyboard
//    // e.g. map ch_0 ~ ch_3 to [-1024,1024] etc.
//
//    return true;
//}

// ---------------- Processing task example ----------------
//void dji_task(void *arg)
//{
//    uint8_t buf[DJI_FRAME_SIZE];
//
//    while(1)
//    {
//        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//
//        // read full frame from queue
//        for(uint16_t i=0;i<DJI_FRAME_SIZE;i++)
//            buf[i] = queue_pop_byte(dji_UART_queue);
//
//        remote_data_t frame;
//        memcpy(&frame, buf, DJI_FRAME_SIZE);
//
//        if(dji_parse_frame(&frame))
//        {
//            // use frame.ch_0, frame.mouse_x, frame.key, etc.
//        }
//    }
//}

