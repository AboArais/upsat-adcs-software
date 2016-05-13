/*
 * adcs_state.c
 *
 *  Created on: May 8, 2016
 *      Author: azisi
 */

#include "adcs_state.h"

void adcs_init_state(volatile adcs_state *state) {
	/* Enable sensors */
	HAL_GPIO_WritePin(SENS_EN_GPIO_Port, SENS_EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	/* Init IMU LSM9DS0 */
	init_lsm9ds0(state);
	/* Init Magnetometer */
	init_rm3100(state);
	/* Init ADT7420 */
	init_adt7420(state);
	/* Init sun sensor */
	init_sun_sensor(state);
}

void adcs_update_state (volatile adcs_state *state) {
	/* Update gyro values */
	update_lsm9ds0(state);
	/* Update RM3100 values */
	update_rm3100(state);
	/* Update ADT7420 values */
	update_adt7420(state);
	/* Update sun sensor values */
	update_sun_sensor(state);
}

/* Init LSM9DS0 for gyro */
void init_lsm9ds0(volatile adcs_state *state) {

	uint8_t i2c_temp[2];
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
	/* Set offset */
	state->calib_gyr[0] = GYRO_OFFSET_X;
	state->calib_gyr[1] = GYRO_OFFSET_Y;
	state->calib_gyr[2] = GYRO_OFFSET_Z;
	HAL_I2C_Mem_Read(&hi2c2, (GYRO_ADDR << 1), WHO_AM_I_G|GYRO_MASK, 1, i2c_temp, 1, 100);
	if (i2c_temp[0] != GYRO_ID) {
		;
	}
	uint8_t GyroCTRreg[5] = {0b01111111, 0b00100101, 0b00000000, 0b10010000, 0b00000000};
	HAL_I2C_Mem_Write(&hi2c2, (GYRO_ADDR << 1), CTRL_REG1_G|GYRO_MASK, 1, GyroCTRreg, 5, 100);
}

/* Get ID and set the Cycle Count Registers */
void init_rm3100(volatile adcs_state *state) {
	uint8_t spi_in_temp[9];
	uint8_t spi_out_temp[9];
    /* Set to 0 all state values */
	state->rm_raw[0] = 0;
	state->rm_raw[1] = 0;
	state->rm_raw[2] = 0;
	state->rm_mag[0] = 0;
	state->rm_mag[1] = 0;
	state->rm_mag[2] = 0;
	state->rm_gain = 0;
	/* Get ID */
	spi_in_temp[0] = PNI_REVID|STATUS_MASK;
	spi_in_temp[1] = 0xFF;
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, spi_in_temp, spi_out_temp, 2, 1000);
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_SET);
	if(spi_out_temp[1] != 0x22)  {
		;//return true;
	}
	HAL_Delay(10);
	/* Set Cycle Count Register */
	spi_in_temp[0] = PNI_CCX;
	spi_in_temp[1] = PNI_CyclesMSB;
	spi_in_temp[2] = PNI_CyclesLSB;
	spi_in_temp[3] = PNI_CyclesMSB;
	spi_in_temp[4] = PNI_CyclesLSB;
	spi_in_temp[5] = PNI_CyclesMSB;
	spi_in_temp[6] = PNI_CyclesLSB;
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, spi_in_temp, 7, 1000);
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_SET);
	/* LSB/μT */
	state->rm_gain = RM_GAIN;
}

/* Init ADT7420 */
void init_adt7420(volatile adcs_state *state) {
	uint8_t i2c_temp[2];

	/* Set to 0 all state values */
	state->temp_raw = 0;
	state->temp_c = 0;

	i2c_temp[0] = 0;
	i2c_temp[1] = 0;
	HAL_I2C_Mem_Read(&hi2c2, ( ADT7420_ADDRESS  << 1 ), ADT7420_REG_ID, 1, i2c_temp, 1, 100);
	if(i2c_temp[0] != ADT7420_DEFAULT_ID)  {
		;
	}
	HAL_Delay(10);
	/* Set operation mode */
	i2c_temp[0] = ADT7420_16BIT|ADT7420_OP_MODE_1_SPS;
	i2c_temp[1] = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, ( ADT7420_ADDRESS  << 1 ), ADT7420_REG_CONFIG, 1, i2c_temp, 1, 100);
}

/* Init sun sensor */
void init_sun_sensor(volatile adcs_state *state) {
	uint8_t spi_in_tmp[4], spi_out_tmp[4];

	state->v_sun_raw[0] = 0;
	state->v_sun_raw[1] = 0;
	state->v_sun_raw[2] = 0;
	state->v_sun_raw[3] = 0;
	state->v_sun_raw[4] = 0;

	state->v_sun[0] = 0;
	state->v_sun[1] = 0;
	state->v_sun[2] = 0;
	state->v_sun[3] = 0;
	state->v_sun[4] = 0;

	spi_in_tmp[0] = AD7682_CFG | AD7682_INCC | AD7682_CH1 | AD7682_BW;
	spi_in_tmp[1] = AD7682_REF | AD7682_SEQ | AD7682_RB;
	spi_in_tmp[2] = 0x00;
	spi_in_tmp[3] = 0x00;

	spi_out_tmp[0] = 0x00;
	spi_out_tmp[1] = 0x00;
	spi_out_tmp[2] = 0x00;
	spi_out_tmp[3] = 0x00;

	HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_SET);
	HAL_Delay(6);
	HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspi1, spi_in_tmp, spi_out_tmp, 5, 100);

	HAL_Delay(1);

	HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_SET);
	HAL_Delay(6);
	HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspi1, spi_in_tmp, spi_out_tmp, 5, 100);

	if ((spi_out_tmp[2] != AD7682_CFG | AD7682_INCC | AD7682_CH1 | AD7682_BW) | (spi_out_tmp[3] != AD7682_REF | AD7682_SEQ | AD7682_RB)) {
		;
	}
}

/* Update values for RM3100 */
void update_rm3100(volatile adcs_state *state) {

	uint8_t spi_in_temp[10];
	uint8_t spi_out_temp[10];
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
	spi_in_temp[9] = 0x00;

	spi_out_temp[0] = 0x00;
	spi_out_temp[1] = 0x00;
	spi_out_temp[2] = 0x00;
	spi_out_temp[3] = 0x00;
	spi_out_temp[4] = 0x00;
	spi_out_temp[5] = 0x00;
	spi_out_temp[6] = 0x00;
	spi_out_temp[7] = 0x00;
	spi_out_temp[8] = 0x00;
	spi_out_temp[9] = 0x00;

	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, spi_in_temp, spi_out_temp, 10, 1000);
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_SET);

	/* X Axis */
	ptr = (char*)(&tmp);
	ptr = ptr +3;
	*ptr --= spi_out_temp[1];
	*ptr --= spi_out_temp[2];
	*ptr --= spi_out_temp[3];
	state->rm_raw[0] = tmp>>8;
	state->rm_mag[0] = (float)state->rm_raw[0]*state->rm_gain;
	/* Y Axis */
	ptr = (char*)(&tmp);
	ptr = ptr +3;
	*ptr --= spi_out_temp[4];
	*ptr --= spi_out_temp[5];
	*ptr --= spi_out_temp[6];
	state->rm_raw[1] = tmp>>8;
	state->rm_mag[1] = (float)state->rm_raw[1]*state->rm_gain;
	/* Z Axis */
	ptr = (char*)(&tmp);
	ptr = ptr +3;
	*ptr --= spi_out_temp[7];
	*ptr --= spi_out_temp[8];
	*ptr --= spi_out_temp[9];
	state->rm_raw[2] = tmp>>8;
	state->rm_mag[2] = (float)state->rm_raw[2]*state->rm_gain;
}

/* Update values for lsm9ds0 */
void update_lsm9ds0(volatile adcs_state *state) {
	int i;
	/* IMU, Gyro measure */
	/* Takes ~0.21ms when no error */
	HAL_I2C_Mem_Read(&hi2c2, (GYRO_ADDR << 1), GYRO_VAL|GYRO_MASK, 1, (uint8_t *)state->gyr_raw, 6, 300);
	/* Handle return of I2C */
	for (i = 0; i<3; i++) {
		state->gyr[i] = ((float)state->gyr_raw[i]-state->calib_gyr[i])*17.5/1e3;/* 8.75/1e3 */
	}
}

/* Update values for adt7420 */
void update_adt7420(volatile adcs_state *state) {
	uint8_t lsb, msb;
	uint8_t i2c_temp[2];
    /* Get Temperature */

	i2c_temp[0] = 0;
	i2c_temp[1] = 0;
	HAL_Delay(10);
	HAL_I2C_Mem_Read(&hi2c2, ( ADT7420_ADDRESS  << 1 ), ADT7420_REG_TEMP_MSB, 1, i2c_temp, 1, 100);
	msb = i2c_temp[0];
	HAL_I2C_Mem_Read(&hi2c2, ( ADT7420_ADDRESS  << 1 ), ADT7420_REG_TEMP_LSB, 1, i2c_temp, 1, 100);
	lsb = i2c_temp[0];
	state->temp_raw = msb<<8;
	state->temp_raw |= lsb;
	if((state->temp_raw >> 15 & 1) == 0)
	{
		state->temp_c = (float)state->temp_raw / 128;
	}
	else
	{
		state->temp_c = (float)(state->temp_raw - 65536) / 128;
	}
}

/* Update sun sensor values */
void update_sun_sensor(volatile adcs_state *state) {

	/* AD7682 Response n-2 channel */
	state->v_sun_raw[3] = update_ad7682(AD7682_CH1);
	state->v_sun_raw[4] = update_ad7682(AD7682_CH2);
	state->v_sun_raw[0] = update_ad7682(AD7682_CH3);
	state->v_sun_raw[1] = update_ad7682(AD7682_CH4);
	state->v_sun_raw[2] = update_ad7682(AD7682_CH5);
}

/* Update values for AD7682 */
uint16_t update_ad7682(uint8_t ch) {
	uint8_t spi_in_temp[4], spi_out_temp[4];
	uint16_t v = 0;

	spi_in_temp[0] = AD7682_CFG | AD7682_INCC | ch | AD7682_BW;
	spi_in_temp[1] = AD7682_REF | AD7682_SEQ | AD7682_RB;
	spi_in_temp[2] = 0x00;
	spi_in_temp[3] = 0x00;

	spi_out_temp[0] = 0x00;
	spi_out_temp[1] = 0x00;
	spi_out_temp[2] = 0x00;
	spi_out_temp[3] = 0x00;

	HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_SET);
	HAL_Delay(6);
	HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspi1, spi_in_temp, spi_out_temp, 5, 100);
	HAL_Delay(1);

	if ((spi_out_temp[2] == AD7682_CFG | AD7682_INCC | ch | AD7682_BW)& (spi_out_temp[3] == AD7682_REF | AD7682_SEQ | AD7682_RB)) {
		v = spi_out_temp[0]<<8;
		v |= spi_out_temp[1];
	}
	else {
		;
	}
	return v;
}
