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

  uint32_t duty_cycle[4];
} _adcs_actuator;

void
init_magneto_torquer (volatile _adcs_actuator *actuator);
void
update_magneto_torquer (volatile _adcs_actuator *actuator);
void
update_spin_torquer (volatile _adcs_actuator *actuator);

#endif /* INC_ADCS_CONTROL_H_ */
