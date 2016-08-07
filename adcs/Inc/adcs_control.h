/*
 * adcs_control.h
 *
 *  Created on: Jul 16, 2016
 *      Author: azisi
 */

#ifndef ADCS_CONTROL_H_
#define ADCS_CONTROL_H_

typedef struct {
    uint16_t gain[3];
    int16_t set_point[3];
} _adcs_control;

extern _adcs_control control;

#endif /* ADCS_CONTROL_H_ */
