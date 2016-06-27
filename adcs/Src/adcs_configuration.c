/*
 * adcs_configuration.c
 *
 *  Created on: May 8, 2016
 *      Author: azisi
 */


#include "adcs_configuration.h"

/* TIM7 init function */
void kick_TIM7_timed_interrupt(uint32_t control_loop) {

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 32;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = control_loop;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

  /* Kick timer interrupt for timed threads */
  HAL_TIM_Base_Start_IT(&htim7);
}
