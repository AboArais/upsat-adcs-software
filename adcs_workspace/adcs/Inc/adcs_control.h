/*
 * adcs_control_module.h
 *
 *  Created on: May 21, 2016
 *      Author: azisi
 */

#ifndef INC_ADCS_CONTROL_H_
#define INC_ADCS_CONTROL_H_

#include "adcs_configuration.h"

/* Spin Torquer */
#define SPIN_ID 	0x03
#define SPIN_TIMEOUT	1000

/* Set Up PWM Start up Duty-Cycle */
#define MAGNETO_TORQUER_PERIOD		65535	/* 4095.9375 Hz */
#define MAGNETO_TORQUER_RESISTANCE	135	/* in Ohm */
#define MAX_VOLT_MAGNETO_TORQUER	5000	/* in mV */
#define MAX_CURR_MAGNETO_TORQUER	37	/* in mA */

typedef struct
{
  uint32_t flag;
  int32_t RPM;
  uint32_t rampTime;
  uint32_t crc;

  int32_t current_x; /* in mA */
  int32_t current_y;
  uint32_t duty_cycle_x;
  uint32_t duty_cycle_y;
} _adcs_actuator;

void
init_magneto_torquer (volatile _adcs_actuator *actuator);
/* Add init for spin_torquer */
void
update_magneto_torquer (volatile _adcs_actuator *actuator);
/* Add errors in magneto */
void
update_spin_torquer (volatile _adcs_actuator *actuator);
/* Flag of spin torquer for errors */

#endif /* INC_ADCS_CONTROL_H_ */
