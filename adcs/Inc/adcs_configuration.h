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
#define ADCS_UART_DBG_EN    1
#define UART_DBG_TIMEOUT    1000

#define LOOP_TIME           1 // in sec

/* Configure control loop timer */
extern TIM_HandleTypeDef htim7;
void kick_TIM7_timed_interrupt(uint32_t control_loop);

#endif /* INC_ADCS_CONFIGURATION_H_ */
