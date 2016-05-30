/*
 * gps_pqNAV_L1.h
 *
 *  Created on: May 18, 2016
 *      Author: azisi
 */

#ifndef INC_GPS_PQNAV_L1_H_
#define INC_GPS_PQNAV_L1_H_

#include "services.h"

//#include "adcs.h"

#define GPS_BUF_SIZE 85
#define GPS_TEMP_BUFF 10


void
gps_init (uint8_t *uart_gps_buf);

SAT_returnState
HAL_gps_rx (TC_TM_app_id app_id, uint8_t *uart_buf);

void
HAL_GPS_UART_IRQHandler (UART_HandleTypeDef *huart);

void
UART_GPS_Receive_IT (UART_HandleTypeDef *huart);

void reset_gps_flag (const uint8_t i);

void get_gps_buff (uint8_t **buf, const uint8_t i, uint8_t *flag);

#endif /* INC_GPS_PQNAV_L1_H_ */
