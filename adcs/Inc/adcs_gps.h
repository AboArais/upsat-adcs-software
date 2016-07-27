/*
 * adcs_gps.h
 *
 *  Created on: May 18, 2016
 *      Author: azisi
 */

#ifndef INC_ADCS_GPS_H_
#define INC_ADCS_GPS_H_

#include "services.h"
#include "time_management_service.h"
#include "gps.h"

#define GPS_ALARM_HOUR  5
#define GPS_ALARM_MIN   20

#define GPS_ALARM_BASE_ADDRESS   0x01F000
#define GPS_ALARM_OFFSET_ADDRESS 1

extern uint8_t gps_sentence[NMEA_MAX_LEN];

_gps_status gps_init(struct time_utc gps_utc);
void HAL_GPS_UART_IRQHandler(UART_HandleTypeDef *huart);
void UART_GPS_Receive_IT(UART_HandleTypeDef *huart);
void reset_gps_flag();
uint8_t get_gps_flag();
_gps_status gps_status_flash(uint8_t gps_status_value);
void HAL_GPS_Alarm_Handler(_gps_status gps_status_value);
_gps_status HAL_SetAlarm_GPS_ON(struct time_utc gps_utc);
_gps_status HAL_SetAlarm_GPS_LOCK(struct time_utc gps_utc);
_gps_status HAL_SetAlarm_GPS(uint8_t gps_alarm_hour, uint8_t gps_alarm_min, uint8_t gps_alarm_sec);

#endif /* INC_ADCS_GPS_H_ */
