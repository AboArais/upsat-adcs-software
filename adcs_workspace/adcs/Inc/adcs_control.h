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
#define MAX_RPM		40000
#define CNT2RPM(x)	48000000*15/(x)
#define RPM2CNT(x)	CNT2RPM(x)
// MOTOR In sync to bridge pulses, but not locked  yet to reference RPM
#define MOTOR_INSYNC 	1
// MOTOR In Synchronous mode to  reference RPM
#define MOTOR_LOCKED 	2
// MOTOR stalled (that is bad, unless RPMr <100)
#define MOTOR_STALL  	3

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
  int32_t m_RPM;

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
