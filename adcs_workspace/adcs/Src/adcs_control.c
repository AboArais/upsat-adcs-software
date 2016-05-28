/*
 * adcs_control_module.c
 *
 *  Created on: May 21, 2016
 *      Author: azisi
 */

#include "adcs_control.h"

extern I2C_HandleTypeDef hi2c2;
extern CRC_HandleTypeDef hcrc;
extern TIM_HandleTypeDef htim4;

void
init_magneto_torquer (volatile _adcs_actuator *actuator)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  actuator->current_x = 0;
  actuator->current_y = 0;
  actuator->duty_cycle_x = 0;
  actuator->duty_cycle_y = 0;

  /* Set up period */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = MAGNETO_TORQUER_PERIOD;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init (&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization (&htim4, &sMasterConfig);

  HAL_TIM_Base_Start (&htim4);

  HAL_TIM_PWM_Start (&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start (&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start (&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start (&htim4, TIM_CHANNEL_4);
}

void
update_magneto_torquer (volatile _adcs_actuator *actuator)
{
  if (abs (actuator->current_x) > MAX_CURR_MAGNETO_TORQUER) {
    actuator->current_x = MAX_CURR_MAGNETO_TORQUER;
  }
  if (actuator->current_x >= 0) {
    actuator->duty_cycle_x = actuator->current_x * MAGNETO_TORQUER_RESISTANCE
	* MAGNETO_TORQUER_PERIOD / MAX_VOLT_MAGNETO_TORQUER;
    htim4.Instance->CCR3 = actuator->duty_cycle_x;
    htim4.Instance->CCR4 = MAGNETO_TORQUER_PERIOD;
  }
  else if (actuator->current_x < 0) {
    actuator->duty_cycle_x = -actuator->current_x * MAGNETO_TORQUER_RESISTANCE
	* MAGNETO_TORQUER_PERIOD / MAX_VOLT_MAGNETO_TORQUER;
    htim4.Instance->CCR3 = MAGNETO_TORQUER_PERIOD;
    htim4.Instance->CCR4 = actuator->duty_cycle_x;
  }
  else {
    /* error */
  }

  if (abs (actuator->current_y) > MAX_CURR_MAGNETO_TORQUER) {
    actuator->current_y = MAX_CURR_MAGNETO_TORQUER;
  }
  if (actuator->current_y >= 0) {
    actuator->duty_cycle_y = actuator->current_y * MAGNETO_TORQUER_RESISTANCE
	* MAGNETO_TORQUER_PERIOD / MAX_VOLT_MAGNETO_TORQUER;
    htim4.Instance->CCR3 = actuator->duty_cycle_y;
    htim4.Instance->CCR4 = MAGNETO_TORQUER_PERIOD;
  }
  else if (actuator->current_y < 0) {
    actuator->duty_cycle_y = -actuator->current_y * MAGNETO_TORQUER_RESISTANCE
	* MAGNETO_TORQUER_PERIOD / MAX_VOLT_MAGNETO_TORQUER;
    htim4.Instance->CCR3 = MAGNETO_TORQUER_PERIOD;
    htim4.Instance->CCR4 = actuator->duty_cycle_y;
  }
  else {
    /* error */
  }
}

void
update_spin_torquer (volatile _adcs_actuator *actuator)
{
  uint8_t sendbuf[16];

  /* Hardware Calculation or CRC */
  uint32_t cbuf[3] =
    { actuator->flag, actuator->RPM, actuator->rampTime };
  actuator->crc = HAL_CRC_Calculate (&hcrc, cbuf, 3);

  sendbuf[3] = (actuator->flag >> 24) & 0x000000FF;
  sendbuf[2] = (actuator->flag >> 16) & 0x000000FF;
  sendbuf[1] = (actuator->flag >> 8) & 0x000000FF;
  sendbuf[0] = (actuator->flag) & 0x000000FF;

  sendbuf[7] = (actuator->RPM >> 24) & 0x000000FF;
  sendbuf[6] = (actuator->RPM >> 16) & 0x000000FF;
  sendbuf[5] = (actuator->RPM >> 8) & 0x000000FF;
  sendbuf[4] = (actuator->RPM) & 0x000000FF;

  sendbuf[11] = (actuator->rampTime >> 24) & 0x000000FF;
  sendbuf[10] = (actuator->rampTime >> 16) & 0x000000FF;
  sendbuf[9] = (actuator->rampTime >> 8) & 0x000000FF;
  sendbuf[8] = (actuator->rampTime) & 0x000000FF;

  sendbuf[15] = (actuator->crc >> 24) & 0x000000FF;
  sendbuf[14] = (actuator->crc >> 16) & 0x000000FF;
  sendbuf[13] = (actuator->crc >> 8) & 0x000000FF;
  sendbuf[12] = (actuator->crc) & 0x000000FF;

  if (HAL_I2C_Mem_Write (&hi2c2, SPIN_ID, sendbuf[0], 1, &sendbuf[1], 15,
			 SPIN_TIMEOUT) != HAL_OK) {
    ; /* ERROR */
  }

}
