/*
 * adcs_sensors.c
 *
 *  Created on: Jun 9, 2016
 *      Author: azisi
 */

#include "stm32f4xx_hal.h"

#include "adcs_sensors.h"
#include <math.h>
#include "log.h"
UART_HandleTypeDef huart2;

extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;

_adcs_sensors adcs_sensors;

static float SUN_SENSOR_K[] = { 4.866511, -4.265312, 4.731704, -4.130505,
		-1.202398 };
static float SUN_SENSOR_N[] = { 4.542984, -4.321169, -4.456006, 4.677820,
		-0.443630 };
static float SUN_SENSOR_M[] = { -1.498061, -1.689346, -1.500885, -1.686522,
		6.374813 };
static float SUN_SENSOR_I[5][5] = { { 46.565823, -75.714898, 10.063492,
		7.353256, -34.833495 }, { 0, 39.719279, 3.216948, 0.506712, -7.447319 },
		{ 0, 0, 44.497669, -75.714898, -26.560880 }, { 0, 0, 0, 41.787433,
				-15.719935 }, { 0, 0, 0, 0, 42.280814 } };
static float SUN_SENSOR_P[6][6] = { { -9.63448e-1, 1.59660, -5.59010e-2,
		1.96049e-3, -3.11760e-5, 1.92557e-7 }, { -6.13404e-3, 4.17313e-4,
		-3.55805e-5, 8.80495e-7, -7.29800E-9, 0 }, { -2.52682e-5, 2.46440e-7,
		-4.67540e-8, 5.16398e-10, 0, 0 }, { 3.77094e-7, 3.46851e-9, 1.50630e-11,
		0, 0, 0 }, { -5.10700e-10, 5.04247e-11, 0, 0, 0, 0 }, { -9.76909e-12, 0,
		0, 0, 0, 0 } };
static float SUN_SENSOR_Q[6][6] = { { -1.38461, 5.03967e-2, -1.81857e-3,
		4.32029e-5, -7.41240e-7, 3.86804e-9 }, { 9.31514e-1, 4.40996e-3,
		-1.27887e-4, 1.88475e-6, -1.07913e-8, 0 }, { 1.16345E-4, 7.17591e-7,
		-5.32212e-8, 1.35780e-9, 0, 0 }, { 1.77366E-6, -5.65011e-8, 5.00120e-10,
		0, 0, 0 }, { -1.49555e-9, -3.97058e-11, 0, 0, 0, 0 }, { -9.13224e-12, 0,
		0, 0, 0, 0 } };

void init_sens(_adcs_sensors *sensors) {
	/* Initialize IMU LSM9DS0 */
	init_lsm9ds0_gyro(sensors);
	/* Set offset */
//	calib_lsm9ds0_gyro(sensors);
	/* Initialize IMU LSM9DS0 magnetometer */
	init_lsm9ds0_xm(sensors);
	/* Initialize Magnetometer */
	init_rm3100(sensors);
	/* Initialize ADT7420 */
	init_adt7420(sensors);
	/* Initialize sun sensor */
	init_sun_sensor(sensors);
}

void update_sens(_adcs_sensors *sensors) {
	uint8_t i = 0;

	int16_t prev_gyr_raw[3] = { 0, 0, 0 };
	int32_t prev_rm_raw[3] = { 0, 0, 0 };

	/* Update LSM9DS0 magnetometer */
	update_lsm9ds0_xm(sensors);

//	for (i = 0; i < N_FILTER; i++) {
	/* Update gyroscope values and convert to Body frame*/
	update_lsm9ds0_gyro(sensors);
//		if ( == DEVICE_NORMAL) {
//			for (uint8_t cnt = 0; cnt < 3; cnt++) {
//				adcs_sensors->lsm9ds0_sensor.gyr_raw_filt[cnt] = A_FILTER
//						* adcs_sensors->lsm9ds0_sensor.gyr_raw[cnt]
//						+ (1 - A_FILTER) * prev_gyr_raw[cnt];
//				prev_gyr_raw[cnt] =
//						adcs_sensors->lsm9ds0_sensor.gyr_raw_filt[cnt];
//			}
//		}
	/* Update RM3100 values and convert to Body frame*/
	update_rm3100(sensors);
//		if ( == DEVICE_NORMAL) {
//			for (uint8_t cnt = 0; cnt < 3; cnt++) {
//				adcs_sensors->magn_sensor.rm_raw_filt[cnt] = A_FILTER
//						* adcs_sensors->magn_sensor.rm_raw[cnt]
//						+ (1 - A_FILTER) * prev_rm_raw[cnt];
//				prev_rm_raw[cnt] = adcs_sensors->magn_sensor.rm_raw_filt[cnt];
//			}
//		}
//	}

	/* Update ADT7420 values */
	update_adt7420(sensors);
	/* Update sun sensor values and convert to Body frame*/
	update_sun_sensor(sensors);
}

/* Initialize LSM9DS0 for gyroscope */
void init_lsm9ds0_gyro(_adcs_sensors *sensors) {
	uint8_t i2c_temp[2];
	/* Set to 0 all state values */
	sensors->lsm9ds0_sensor.gyr_raw[0] = 0;
	sensors->lsm9ds0_sensor.gyr_raw[1] = 0;
	sensors->lsm9ds0_sensor.gyr_raw[2] = 0;
	sensors->lsm9ds0_sensor.gyr_raw_filt[0] = 0;
	sensors->lsm9ds0_sensor.gyr_raw_filt[1] = 0;
	sensors->lsm9ds0_sensor.gyr_raw_filt[2] = 0;
	sensors->lsm9ds0_sensor.gyr[0] = 0;
	sensors->lsm9ds0_sensor.gyr[1] = 0;
	sensors->lsm9ds0_sensor.gyr[2] = 0;
	sensors->lsm9ds0_sensor.calib_gyr[0] = 0;
	sensors->lsm9ds0_sensor.calib_gyr[1] = 0;
	sensors->lsm9ds0_sensor.calib_gyr[2] = 0;
	sensors->lsm9ds0_sensor.gyr_status = DEVICE_NORMAL;

	if (HAL_I2C_Mem_Read(&hi2c2, (GYRO_ADDR << 1), WHO_AM_I_G | LSM9DS0_MASK, 1,
			i2c_temp, 1, LSM9DS0_TIMEOUT) != HAL_OK) {
		sensors->lsm9ds0_sensor.gyr_status = DEVICE_ERROR;
		return;
	}
	if (i2c_temp[0] != GYRO_ID) {
		sensors->lsm9ds0_sensor.gyr_status = DEVICE_ERROR;
		return;
	}

	uint8_t GyroCTRreg[5] = { 0b01111111, 0b00100101, 0b00000000, 0b10010000,
			0b00000000 };
	if (HAL_I2C_Mem_Write(&hi2c2, (GYRO_ADDR << 1),
	CTRL_REG1_G | LSM9DS0_MASK, 1, GyroCTRreg, 5, LSM9DS0_TIMEOUT) != HAL_OK) {
		sensors->lsm9ds0_sensor.gyr_status = DEVICE_ERROR;
		return;
	}

}

/* Calibration of lsm9ds0 gyroscope */
void calib_lsm9ds0_gyro(_adcs_sensors *sensors) {
	int i = GYRO_N, d, cnt;
	cnt = 0;

	while (i--) {
		update_lsm9ds0_gyro(sensors);
		if (sensors->lsm9ds0_sensor.gyr_status == DEVICE_ERROR) {
			return;
		}
		cnt++;
		for (d = 0; d < 3; d++) {
			sensors->lsm9ds0_sensor.calib_gyr[d] +=
					(float) sensors->lsm9ds0_sensor.gyr_raw[d];
			HAL_Delay(10);
		}
	}
	for (d = 0; d < 3; d++) {
		sensors->lsm9ds0_sensor.calib_gyr[d] /= (float) cnt;
	}
}

/* Update values for lsm9ds0 gyroscope */
void update_lsm9ds0_gyro(_adcs_sensors *sensors) {
	uint8_t i;
	sensors->lsm9ds0_sensor.gyr_raw[0] = 0;
	sensors->lsm9ds0_sensor.gyr_raw[1] = 0;
	sensors->lsm9ds0_sensor.gyr_raw[2] = 0;
	/* IMU, Gyroscope measure */
	if (HAL_I2C_Mem_Read(&hi2c2, (GYRO_ADDR << 1), GYRO_VAL | LSM9DS0_MASK, 1,
			(uint8_t *) sensors->lsm9ds0_sensor.gyr_raw, 6,
			LSM9DS0_TIMEOUT) != HAL_OK) {
		sensors->lsm9ds0_sensor.gyr_status = DEVICE_ERROR;
		return;
	}

	sensors->lsm9ds0_sensor.gyr_status = DEVICE_NORMAL;

	for (i = 0; i < 3; i++) {
		sensors->lsm9ds0_sensor.gyr[i] =
				((float) sensors->lsm9ds0_sensor.gyr_raw[i]
						- sensors->lsm9ds0_sensor.calib_gyr[i]) * GYRO_GAIN;
	}
}

/* Initialize LSM9DS0 for magnetometer */
void init_lsm9ds0_xm(_adcs_sensors *sensors) {
	uint8_t i2c_temp[2];
	/* Set to 0 all state values */
	sensors->lsm9ds0_sensor.xm_raw[0] = 0;
	sensors->lsm9ds0_sensor.xm_raw[1] = 0;
	sensors->lsm9ds0_sensor.xm_raw[2] = 0;
	sensors->lsm9ds0_sensor.xm_raw_filt[0] = 0;
	sensors->lsm9ds0_sensor.xm_raw_filt[1] = 0;
	sensors->lsm9ds0_sensor.xm_raw_filt[2] = 0;
	sensors->lsm9ds0_sensor.xm[0] = 0;
	sensors->lsm9ds0_sensor.xm[1] = 0;
	sensors->lsm9ds0_sensor.xm[2] = 0;
	sensors->lsm9ds0_sensor.xm_status = DEVICE_NORMAL;

	if (HAL_I2C_Mem_Read(&hi2c2, (XM_ADDR << 1), WHO_AM_I_XM | LSM9DS0_MASK, 1,
			i2c_temp, 1, LSM9DS0_TIMEOUT) != HAL_OK) {
		sensors->lsm9ds0_sensor.xm_status = DEVICE_ERROR;
		return;
	}
	if (i2c_temp[0] != XM_ID) {
		sensors->lsm9ds0_sensor.xm_status = DEVICE_ERROR;
		return;
	}
	// HP filter bypassed
	// 200Hz, 50HzBW, 4G, Magn:100Hz,2Gauss
	uint8_t XM_CTRreg[8] = { 0b00000000, 0b01111111, 0b11001000, 0b00000000,
			0b00000000, 0b11110100, 0b00000000, 0b00000000 };

	if (HAL_I2C_Mem_Write(&hi2c2, (XM_ADDR << 1),
	XM_CTR_REG | LSM9DS0_MASK, 1, XM_CTRreg, 8, LSM9DS0_TIMEOUT) != HAL_OK) {
		sensors->lsm9ds0_sensor.xm_status = DEVICE_ERROR;
		return;
	}

}

/* Update values for lsm9ds0 magnetometer*/
void update_lsm9ds0_xm(_adcs_sensors *sensors) {
	sensors->lsm9ds0_sensor.xm_raw[0] = 0;
	sensors->lsm9ds0_sensor.xm_raw[1] = 0;
	sensors->lsm9ds0_sensor.xm_raw[2] = 0;
	/* IMU, Magnetometer measure */
	if (HAL_I2C_Mem_Read(&hi2c2, (XM_ADDR << 1), XM_VAL | LSM9DS0_MASK, 1,
			(uint8_t *) sensors->lsm9ds0_sensor.xm_raw, 6, LSM9DS0_TIMEOUT)
			!= HAL_OK) {
		sensors->lsm9ds0_sensor.xm_status = DEVICE_ERROR;
		return;
	}

	sensors->lsm9ds0_sensor.xm_status = DEVICE_NORMAL;

	sensors->lsm9ds0_sensor.xm[0] = (float) sensors->lsm9ds0_sensor.xm_raw[0]
			* XM_GAIN;
	sensors->lsm9ds0_sensor.xm[1] = (float) sensors->lsm9ds0_sensor.xm_raw[1]
			* XM_GAIN;
	sensors->lsm9ds0_sensor.xm[2] = -(float) sensors->lsm9ds0_sensor.xm_raw[2]
			* XM_GAIN;

	return;

}

/* Get ID and set the Cycle Count Registers */
void init_rm3100(_adcs_sensors *sensors) {
	uint8_t spi_in_temp[9];
	uint8_t spi_out_temp[9];

	/* Set to 0 all adcs_sensors values */
	sensors->magn_sensor.rm_raw[0] = 0;
	sensors->magn_sensor.rm_raw[1] = 0;
	sensors->magn_sensor.rm_raw[2] = 0;
	sensors->magn_sensor.rm_mag[0] = 0;
	sensors->magn_sensor.rm_mag[1] = 0;
	sensors->magn_sensor.rm_mag[2] = 0;
	sensors->magn_sensor.rm_status = DEVICE_NORMAL;

	/* Get ID */
	spi_in_temp[0] = PNI_REVID | STATUS_MASK;
	spi_in_temp[1] = 0xFF;
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive(&hspi1, spi_in_temp, spi_out_temp, 2,
	PNI_TIMEOUT) != HAL_OK) {
		sensors->magn_sensor.rm_status = DEVICE_ERROR;
		return;
	}
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_SET);
	if (spi_out_temp[1] != 0x22) {
		sensors->magn_sensor.rm_status = DEVICE_ERROR;
		return;
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
	if (HAL_SPI_Transmit(&hspi1, spi_in_temp, 7, PNI_TIMEOUT) != HAL_OK) {
		sensors->magn_sensor.rm_status = DEVICE_ERROR;
		return;
	}
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_SET);

}

/* Update values for RM3100 */
void update_rm3100(_adcs_sensors *sensors) {
	uint8_t spi_in_temp[10];
	uint8_t spi_out_temp[10];
	int32_t tmp = 0;
	char *ptr;

	/* Write POLL 0x00 register and followed 0x70 */
	spi_in_temp[0] = PNI_POLL;
	spi_in_temp[1] = SM_ALL_AXIS;
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&hspi1, spi_in_temp, 2, PNI_TIMEOUT) != HAL_OK) {
		sensors->magn_sensor.rm_status = DEVICE_ERROR;
		return;
	}
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_SET);
	/* Wait for LOW MISO */
	HAL_Delay(10);
	/* Read raw data */
	spi_in_temp[0] = PNI_MX | STATUS_MASK;
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
	if (HAL_SPI_TransmitReceive(&hspi1, spi_in_temp, spi_out_temp, 10,
	PNI_TIMEOUT) != HAL_OK) {
		sensors->magn_sensor.rm_status = DEVICE_ERROR;
		return;
	}
	HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_SET);

	sensors->magn_sensor.rm_status = DEVICE_NORMAL;
	/* X Axis */
	ptr = (char*) (&tmp);
	ptr = ptr + 3;
	*ptr-- = spi_out_temp[1];
	*ptr-- = spi_out_temp[2];
	*ptr-- = spi_out_temp[3];
	sensors->magn_sensor.rm_raw[0] = tmp >> 8;
	sensors->magn_sensor.rm_mag[0] = (float) sensors->magn_sensor.rm_raw[0]
			* PNI_GAIN;
	/* Y Axis */
	ptr = (char*) (&tmp);
	ptr = ptr + 3;
	*ptr-- = spi_out_temp[4];
	*ptr-- = spi_out_temp[5];
	*ptr-- = spi_out_temp[6];
	sensors->magn_sensor.rm_raw[1] = tmp >> 8;
	sensors->magn_sensor.rm_mag[1] = (float) sensors->magn_sensor.rm_raw[1]
			* PNI_GAIN;
	/* Z Axis */
	ptr = (char*) (&tmp);
	ptr = ptr + 3;
	*ptr-- = spi_out_temp[7];
	*ptr-- = spi_out_temp[8];
	*ptr-- = spi_out_temp[9];
	sensors->magn_sensor.rm_raw[2] = tmp >> 8;
	sensors->magn_sensor.rm_mag[2] = (float) sensors->magn_sensor.rm_raw[2]
			* PNI_GAIN;

}

/* Initialize ADT7420 */
void init_adt7420(_adcs_sensors *sensors) {
	uint8_t i2c_temp[2];

	/* Set to 0 all state values */
	sensors->temp_sensor.temp_raw = 0;
	sensors->temp_sensor.temp_c = 0;
	sensors->temp_sensor.temp_status = DEVICE_NORMAL;

	i2c_temp[0] = 0;
	i2c_temp[1] = 0;
	if (HAL_I2C_Mem_Read(&hi2c2, ( ADT7420_ADDRESS << 1), ADT7420_REG_ID, 1,
			i2c_temp, 1, ADT7420_TIMEOUT) != HAL_OK) {
		sensors->temp_sensor.temp_status = DEVICE_ERROR;
		return;
	}
	if (i2c_temp[0] != ADT7420_DEFAULT_ID) {
		sensors->temp_sensor.temp_status = DEVICE_ERROR;
		return;
	}
	HAL_Delay(10);
	/* Set operation mode */
	i2c_temp[0] = ADT7420_16BIT | ADT7420_OP_MODE_1_SPS;
	i2c_temp[1] = 0x00;
	if (HAL_I2C_Mem_Write(&hi2c2, ( ADT7420_ADDRESS << 1),
	ADT7420_REG_CONFIG, 1, i2c_temp, 1, ADT7420_TIMEOUT) != HAL_OK) {
		sensors->temp_sensor.temp_status = DEVICE_ERROR;
		return;
	}
}

/* Update values for adt7420 */
void update_adt7420(_adcs_sensors *sensors) {
	uint8_t lsb, msb;
	uint8_t i2c_temp[2];

	/* Get Temperature */
	i2c_temp[0] = 0;
	i2c_temp[1] = 0;
	if (HAL_I2C_Mem_Read(&hi2c2, ( ADT7420_ADDRESS << 1),
	ADT7420_REG_TEMP_MSB, 1, i2c_temp, 1, ADT7420_TIMEOUT) != HAL_OK) {
		sensors->temp_sensor.temp_status = DEVICE_ERROR;
		return;
	}
	msb = i2c_temp[0];
	if (HAL_I2C_Mem_Read(&hi2c2, ( ADT7420_ADDRESS << 1),
	ADT7420_REG_TEMP_LSB, 1, i2c_temp, 1, ADT7420_TIMEOUT) != HAL_OK) {
		sensors->temp_sensor.temp_status = DEVICE_ERROR;
		return;
	}

	sensors->temp_sensor.temp_status = DEVICE_NORMAL;
	lsb = i2c_temp[0];
	sensors->temp_sensor.temp_raw = msb << 8;
	sensors->temp_sensor.temp_raw |= lsb;
	if ((sensors->temp_sensor.temp_raw >> 15 & 1) == 0) {
		sensors->temp_sensor.temp_c = (float) sensors->temp_sensor.temp_raw
				/ 128;
	} else {
		sensors->temp_sensor.temp_c = (float) (sensors->temp_sensor.temp_raw
				- 65536) / 128;
	}

}

/* Initialize sun sensor */
void init_sun_sensor(_adcs_sensors *sensors) {
	uint8_t spi_in_tmp[4], spi_out_tmp[4];

	sensors->sun_sensor.v_sun_raw[0] = 0;
	sensors->sun_sensor.v_sun_raw[1] = 0;
	sensors->sun_sensor.v_sun_raw[2] = 0;
	sensors->sun_sensor.v_sun_raw[3] = 0;
	sensors->sun_sensor.v_sun_raw[4] = 0;

	sensors->sun_sensor.v_sun[0] = 0;
	sensors->sun_sensor.v_sun[1] = 0;
	sensors->sun_sensor.v_sun[2] = 0;
	sensors->sun_sensor.v_sun[3] = 0;
	sensors->sun_sensor.v_sun[4] = 0;

	sensors->sun_sensor.long_rough = 0;
	sensors->sun_sensor.lat_rough = 0;
	sensors->sun_sensor.long_sun = 0;
	sensors->sun_sensor.lat_sun = 0;

	sensors->sun_sensor.sun_status = DEVICE_NORMAL;

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
	if (HAL_SPI_TransmitReceive(&hspi1, spi_in_tmp, spi_out_tmp, 5,
	AD7682_TIMEOUT) != HAL_OK) {
		sensors->sun_sensor.sun_status = DEVICE_ERROR;
		return;
	}

	HAL_Delay(1);

	HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_SET);
	HAL_Delay(6);
	HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	if (HAL_SPI_TransmitReceive(&hspi1, spi_in_tmp, spi_out_tmp, 5,
	AD7682_TIMEOUT) != HAL_OK) {
		sensors->sun_sensor.sun_status = DEVICE_ERROR;
		return;
	}

	if ((spi_out_tmp[2] != (AD7682_CFG | AD7682_INCC | AD7682_CH1 | AD7682_BW))
			| (spi_out_tmp[3] != (AD7682_REF | AD7682_SEQ | AD7682_RB))) {
		sensors->sun_sensor.sun_status = DEVICE_ERROR;
		return;
	}
}

/* Update sun sensor values */
void update_sun_sensor(_adcs_sensors *sensors) {
	uint8_t i = 0;
	uint8_t j = 0;
	float long_rough_numerator = 0;
	float long_rough_demominator = 0;
	float lat_rough_numerator = 0;
	float lat_rough_demominator = 0;
	float meas_Valid = 0;

	/* AD7682 Response to n-2 channel */
	if (update_ad7682(AD7682_CH1, &sensors->sun_sensor.v_sun_raw[3])
			== DEVICE_ERROR) {
		sensors->sun_sensor.sun_status = DEVICE_ERROR;
		return;
	}
	if (update_ad7682(AD7682_CH2, &sensors->sun_sensor.v_sun_raw[4])
			== DEVICE_ERROR) {
		sensors->sun_sensor.sun_status = DEVICE_ERROR;
		return;
	}
	if (update_ad7682(AD7682_CH3, &sensors->sun_sensor.v_sun_raw[0])
			== DEVICE_ERROR) {
		sensors->sun_sensor.sun_status = DEVICE_ERROR;
		return;
	}
	if (update_ad7682(AD7682_CH4, &sensors->sun_sensor.v_sun_raw[1])
			== DEVICE_ERROR) {
		sensors->sun_sensor.sun_status = DEVICE_ERROR;
		return;
	}
	if (update_ad7682(AD7682_CH5, &sensors->sun_sensor.v_sun_raw[2])
			== DEVICE_ERROR) {
		sensors->sun_sensor.sun_status = DEVICE_ERROR;
		return;
	}

	sensors->sun_sensor.sun_status = DEVICE_NORMAL;
	/* Convert to V */
	for (i = 0; i < 5; i++) {
		sensors->sun_sensor.v_sun[i] = (float) sensors->sun_sensor.v_sun_raw[i]
				* AD7682_COEF;
	}
	/* Measure from sun sensor only if: 4*V5 − V1 − V2 − V3 − V4 ≥ 0.7 */
	meas_Valid = 4 * sensors->sun_sensor.v_sun[4] - sensors->sun_sensor.v_sun[0]
			- sensors->sun_sensor.v_sun[1] - sensors->sun_sensor.v_sun[2]
			- sensors->sun_sensor.v_sun[3];
	if (meas_Valid >= SUN_SENSOR_VALID) {
		sensors->sun_sensor.sun_status = DEVICE_ENABLE;
		/* Calculate Rough Measures */
		for (i = 0; i < 5; i++) {
			long_rough_numerator += SUN_SENSOR_K[i]
					* sensors->sun_sensor.v_sun[i];
			long_rough_demominator += SUN_SENSOR_N[i]
					* sensors->sun_sensor.v_sun[i];
			lat_rough_numerator += SUN_SENSOR_M[i]
					* sensors->sun_sensor.v_sun[i];
			for (j = i; j < 5; j++) {
				long_rough_demominator += SUN_SENSOR_I[i][j]
						* sensors->sun_sensor.v_sun[i]
						* sensors->sun_sensor.v_sun[j];
			}
		}
		/* Calculate Rough Measures */
		if ((long_rough_demominator != 0) & (lat_rough_demominator != 0)) {
			sensors->sun_sensor.long_rough = atan2f(long_rough_numerator,
					long_rough_demominator);
			sensors->sun_sensor.lat_rough = acosf(
					lat_rough_numerator / sqrtf(lat_rough_demominator));
			/* Calculate Fine Measures */
			for (i = 0; i < S_SUN_SENSOR; i++) {
				for (j = 0; j < S_SUN_SENSOR - i; j++) {
					sensors->sun_sensor.long_sun += SUN_SENSOR_P[i][j]
							* sensors->sun_sensor.long_rough
							* sensors->sun_sensor.lat_rough;
					sensors->sun_sensor.lat_sun += SUN_SENSOR_Q[i][j]
							* sensors->sun_sensor.long_rough
							* sensors->sun_sensor.lat_rough;
				}
			}
		}
	} else {
		sensors->sun_sensor.sun_status = DEVICE_DISABLE;
	}

}

/* Update values for AD7682 */
_adcs_sensor_status update_ad7682(uint8_t ch, uint16_t *v_raw) {
	uint8_t spi_in_temp[4], spi_out_temp[4];

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
	if (HAL_SPI_TransmitReceive(&hspi1, spi_in_temp, spi_out_temp, 5,
	AD7682_TIMEOUT) != HAL_OK) {
		return DEVICE_ERROR;
	}
	if ((spi_out_temp[2] != (AD7682_CFG | AD7682_INCC | AD7682_CH1 | AD7682_BW))
			| (spi_out_temp[3] != (AD7682_REF | AD7682_SEQ | AD7682_RB))) {
		return DEVICE_ERROR;
	} else {
		*v_raw = spi_out_temp[0] << 8;
		*v_raw |= spi_out_temp[1];
		return DEVICE_NORMAL;
	}
}
