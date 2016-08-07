/*
 * adcs_configuration.h
 *
 *  Created on: May 8, 2016
 *      Author: azisi
 */

#ifndef INC_ADCS_CONFIGURATION_H_
#define INC_ADCS_CONFIGURATION_H_

#include "stm32f4xx_hal.h"

#define ADCS_UART_BUF_LEN   2048
#define ADCS_UART_DBG_EN    0
#define UART_DBG_TIMEOUT    1000

#define TEST_EPS_PKT_TIME   120 // in sec

#define EPS_DELAY_READY     1000 // in ms

#define OBC_REQUEST_TIME_UPDATE 10000 // in ms

#define LOOP_TIME           1       // in sec
#define LOOP_TIME_TICKS     50000   // in ticks

#define GPS_ALARM_ON        5  // in hours
#define GPS_ALARM_UNLOCK    12 // in min
#define GPS_ALARM_RESET     8  // in min

#define WDG_REFRESH         5

typedef enum {
    TIMED_EVENT_SERVICED = 0,
    TIMED_EVENT_NOT_SERVICED,
    TIMED_EVENT_LAST_VALUE
}ADCS_timed_event_status;

extern TIM_HandleTypeDef htim7;
extern ADCS_timed_event_status ADCS_event_period_status;

/* Configure control loop timer */
HAL_StatusTypeDef kick_TIM7_timed_interrupt(uint32_t control_loop); // in ticks

#endif /* INC_ADCS_CONFIGURATION_H_ */
