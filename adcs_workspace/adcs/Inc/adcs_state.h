/*
 * adcs_state.h
 *
 *  Created on: May 8, 2016
 *      Author: azisi
 */

#ifndef INC_ADCS_STATE_H_
#define INC_ADCS_STATE_H_

#include "adcs_configuration.h"

extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart2;

typedef struct{
	int16_t gyr_raw[3];
	float gyr[3];
	float calib_gyr[3];
	int32_t rm_raw[3];
	float rm_mag[3]; /* in uΤ */
	float rm_gain;
	uint16_t temp_raw;
	float temp_c;
	uint16_t v_sun_raw[5];
	float v_sun[5];
	/* add geomag in X,Y,Z */
	/* add sun sensor in X,Y,Z */

}adcs_state;

/* Init ADCS state */
void adcs_init_state(volatile adcs_state *state);
void init_lsm9ds0(volatile adcs_state *state);
void init_rm3100(volatile adcs_state *state);
void init_adt7420(volatile adcs_state *state);
void init_sun_sensor(volatile adcs_state *state);

/* Update ADCS state */
void adcs_update_state(volatile adcs_state *state);
void update_lsm9ds0(volatile adcs_state *state);
void update_rm3100(volatile adcs_state *state);
void update_adt7420(volatile adcs_state *state);
uint16_t update_ad7682(uint8_t ch);

/* SGP4 orbit propagator */
#include "sgdp4h.h"

/* Geomagnetic Field Model */
#include "geomag.h"

#endif /* INC_ADCS_STATE_H_ */
