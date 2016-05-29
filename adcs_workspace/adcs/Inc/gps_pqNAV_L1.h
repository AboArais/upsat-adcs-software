/*
 * gps_pqNAV_L1.h
 *
 *  Created on: May 18, 2016
 *      Author: azisi
 */

#ifndef INC_GPS_PQNAV_L1_H_
#define INC_GPS_PQNAV_L1_H_

#include "stm32f4xx_hal.h"
#include "services.h"

//#include "adcs.h"

#define GPS_BUF_SIZE 85

#define GGA 0
#define GSA 1
#define LSP 2
#define LSV 3

static uint8_t gps_buffer[GPS_BUF_SIZE];
static uint8_t gps_buff_size;

//extern _adcs_state adcs_state;

void
gps_init (uint8_t *uart_gps_buf);

SAT_returnState
HAL_gps_rx (TC_TM_app_id app_id, uint8_t *uart_buf);

void
HAL_GPS_UART_IRQHandler (UART_HandleTypeDef *huart);

void
UART_GPS_Receive_IT (UART_HandleTypeDef *huart);

uint8_t
get_gps_flag ();

void
get_gps_gga (uint8_t **gga);

void
get_gps_gsa (uint8_t **gsa);

void
get_gps_lsp (uint8_t **lsp);

void
get_gps_LSV (uint8_t **lsv);

#endif /* INC_GPS_PQNAV_L1_H_ */
