/*
 * adcs_control_module.h
 *
 *  Created on: May 21, 2016
 *      Author: azisi
 */

#ifndef INC_ADCS_CONTROL_H_
#define INC_ADCS_CONTROL_H_

#include "adcs_configuration.h"

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
void
update_magneto_torquer (volatile _adcs_actuator *actuator);
void
update_spin_torquer (volatile _adcs_actuator *actuator);

#endif /* INC_ADCS_CONTROL_H_ */
