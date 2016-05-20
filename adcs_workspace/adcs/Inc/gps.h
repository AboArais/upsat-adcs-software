/*
 * gps.h
 *
 *  Created on: May 18, 2016
 *      Author: azisi
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "stm32f4xx_hal.h"
#include "services.h"

#define GPS_BUF_SIZE 85

static uint8_t gps_buffer[GPS_BUF_SIZE];
static uint8_t gps_buff_size;

void
gps_init ();

SAT_returnState
HAL_gps_rx (TC_TM_app_id app_id, uint8_t *uart_buf);

void
HAL_GPS_UART_IRQHandler (UART_HandleTypeDef *huart);

void
UART_GPS_Receive_IT (UART_HandleTypeDef *huart);

#endif /* INC_GPS_H_ */
