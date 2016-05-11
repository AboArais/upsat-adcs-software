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
	signed long rm_raw[3];
	float rm_mag[3]; /* in uΤ */
	float rm_gain;
	/* add geomag in X,Y,Z */
	/* add temperature */
	/* add sun sensor in X,Y,Z */

}adcs_state;

/* Init ADCS state */
void adcs_init_state(volatile adcs_state *state);
void init_lsm9ds0(volatile adcs_state *state);
void init_rm3100(volatile adcs_state *state);

/* Update ADCS state */
void adcs_update_state(volatile adcs_state *state);
void update_lsm9ds0(volatile adcs_state *state);
void update_rm3100(volatile adcs_state *state);

/* SGP4 orbit propagator */
#include "sgdp4h.h"

/* Geomagnetic Field Model */
#include "geomag.h"

#endif /* INC_ADCS_STATE_H_ */
