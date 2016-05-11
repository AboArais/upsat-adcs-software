/*
 * adcs_state.c
 *
 *  Created on: May 8, 2016
 *      Author: azisi
 */

#include "adcs_state.h"

void adcs_init_state(volatile adcs_state *state) {
    /* Set to 0 all state values */
	state->gyr_raw[0] = 0;
	state->gyr_raw[1] = 0;
	state->gyr_raw[2] = 0;
	state->gyr[0] = 0;
	state->gyr[1] = 0;
	state->gyr[2] = 0;
	state->calib_gyr[0] = 0;
	state->calib_gyr[1] = 0;
	state->calib_gyr[2] = 0;
	state->rm_raw[0] = 0;
	state->rm_raw[1] = 0;
	state->rm_raw[2] = 0;
	state->rm_mag[0] = 0;
	state->rm_mag[1] = 0;
	state->rm_mag[2] = 0;
	state->rm_gain = 0;
	/* Enable sensors */
	HAL_GPIO_WritePin(SENS_EN_GPIO_Port, SENS_EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	/* Init IMU LSM9DS0 */
	init_lsm9ds0(state);
	/* Init Magnetometer */
	init_rm3100(state);
}

void adcs_update_state (volatile adcs_state *state) {
	/* Update gyro values */
	update_lsm9ds0(state);
	/* Update RM3100 values */
	update_rm3100(state);
}
/* Init LSM9DS0 for gyro */
void init_lsm9ds0(volatile adcs_state *state) {

	uint8_t i2c_temp[2];
	/* Set offset */
	state->calib_gyr[0] = GYRO_OFFSET_X;
	state->calib_gyr[1] = GYRO_OFFSET_Y;
	state->calib_gyr[2] = GYRO_OFFSET_Z;
	HAL_I2C_Mem_Write(&hi2c2, (GYRO_ADDR << 1), 0x20|0x80, 1, i2c_temp, 1, 100);
	HAL_I2C_Mem_Read(&hi2c2, (GYRO_ADDR << 1), 0x0f|0x80, 1, i2c_temp, 1, 100);
	/* 24h ->0b00000000 disables LPF2, Enable 0b00000010 */
	uint8_t GyroCTRreg[5] = {0b01111111, 0b00100101, 0b00000000, 0b10010000, 0b00000000};
	HAL_I2C_Mem_Write(&hi2c2, (GYRO_ADDR << 1), 0x20|0x80, 1, GyroCTRreg, 5, 100);
}
/* Get ID and set the Cycle Count Registers */
void init_rm3100(volatile adcs_state *state) {
	uint8_t spi_in_temp[10];
	uint8_t spi_out_temp[10];
	/* Get ID */
	spi_in_temp[0] = PNI_REVID|STATUS_MASK;
	spi_in_temp[1] = 0xFF;
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, spi_in_temp, spi_out_temp, 2, 1000);
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_SET);
	if(spi_out_temp[1] == 0x22)  {
		;//return true;
	} else {
		;//return false;
	}
	/* LSB/μT */
	state->rm_gain = 75/1e6;
}
/* Update values for RM3100 */
void update_rm3100(volatile adcs_state *state) {

	uint8_t spi_in_temp[9];
	uint8_t spi_out_temp[9];
	int32_t tmp = 0;
	char *ptr;
	/* Write POLL 0x00 register and followed 0x70 */
	spi_in_temp[0] = PNI_POLL;
	spi_in_temp[1] = SM_ALL_AXIS;
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, spi_in_temp, 2, 1000);
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_SET);
	/* Wait for LOW MISO */
	HAL_Delay(10);
	/* Read raw data */
	spi_in_temp[0] = PNI_MX|STATUS_MASK;
	spi_in_temp[1] = 0x00;
	spi_in_temp[2] = 0x00;
	spi_in_temp[3] = 0x00;
	spi_in_temp[4] = 0x00;
	spi_in_temp[5] = 0x00;
	spi_in_temp[6] = 0x00;
	spi_in_temp[7] = 0x00;
	spi_in_temp[8] = 0x00;

	spi_out_temp[0] = 0x00;
	spi_out_temp[1] = 0x00;
	spi_out_temp[2] = 0x00;
	spi_out_temp[3] = 0x00;
	spi_out_temp[4] = 0x00;
	spi_out_temp[5] = 0x00;
	spi_out_temp[6] = 0x00;
	spi_out_temp[7] = 0x00;
	spi_out_temp[8] = 0x00;

	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, spi_in_temp, spi_out_temp, 9, 1000);
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_SET);
	/* X Axis */
	ptr = (char*)(&tmp);
	ptr = ptr +3;
	*ptr --= spi_out_temp[0];
	*ptr --= spi_out_temp[1];
	*ptr --= spi_out_temp[2];
	state->rm_raw[0] = tmp>>8;
	state->rm_mag[0] = (float)state->rm_raw[0]*state->rm_gain;
	/* Y Axis */
	ptr = (char*)(&tmp);
	ptr = ptr +3;
	*ptr --= spi_out_temp[3];
	*ptr --= spi_out_temp[4];
	*ptr --= spi_out_temp[5];
	state->rm_raw[1] = tmp>>8;
	state->rm_mag[1] = (float)state->rm_raw[1]*state->rm_gain;
	/* Z Axis */
	ptr = (char*)(&tmp);
	ptr = ptr +3;
	*ptr --= spi_out_temp[6];
	*ptr --= spi_out_temp[7];
	*ptr --= spi_out_temp[8];
	state->rm_raw[2] = tmp>>8;
	state->rm_mag[2] = (float)state->rm_raw[2]*state->rm_gain;
}

void update_lsm9ds0(volatile adcs_state *state) {
	int i;

	/* IMU, Gyro measure */
	/* Takes ~0.21ms when no error */
	HAL_I2C_Mem_Read(&hi2c2, (GYRO_ADDR << 1), 0x28|0x80, 1, (uint8_t *)state->gyr_raw, 6, 300);
	/* Handle return of I2C */
	for (i = 0; i<3; i++) {
		state->gyr[i] = ((float)state->gyr_raw[i]-state->calib_gyr[i])*17.5/1e3;/* 8.75/1e3 */
	}
}
