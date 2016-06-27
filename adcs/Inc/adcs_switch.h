/*
 * adcs_common.h
 *
 *  Created on: Jun 17, 2016
 *      Author: azisi
 */

#ifndef INC_ADCS_SWITCH_H_
#define INC_ADCS_SWITCH_H_

typedef enum {
	SWITCH_ON = 1, SWITCH_OFF
} _switch_state;

typedef enum {
	SENSORS = 1, GPS
} _adcs_switch;

typedef struct {
	_switch_state sens_sw;
	_switch_state gps_sw;
} _adcs_switch_state;

extern _adcs_switch_state adcs_switch_state;

/* ON-OFF Switches */
void adcs_pwr_switch(_switch_state switch_state, _adcs_switch adcs_switch);

#endif /* INC_ADCS_SWITCH_H_ */
