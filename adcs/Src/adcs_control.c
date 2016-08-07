/*
 * adcs_control.c
 *
 *  Created on: Jul 16, 2016
 *      Author: azisi
 */
#include "stm32f4xx_hal.h"

#include "adcs_control.h"

_adcs_control control = { .gain[0] = 0, .gain[1] = 0, .gain[2] = 0, .set_point[0] = 0,
                          .set_point[1] = 0, .set_point[2] = 0 };

