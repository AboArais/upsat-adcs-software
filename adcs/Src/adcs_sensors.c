/*
 * adcs_sensors.c
 *
 *  Created on: Jun 9, 2016
 *      Author: azisi
 */

#include "stm32f4xx_hal.h"

#include "adcs_sensors.h"
#include "adcs_frame.h"
#include <math.h>
#include <string.h>

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

static float RM_OFFSET[] = { 727.5142, -76.0090, 75.7189 };
static float RM_SCALE[] = { 0.2991e-3, 0.0139e-3, -0.0008e-3, 0, 0.3142e-3, -0.0026e-3, 0, 0, 0.3081e-3 };
static float XM_OFFSET[] = { 610.3734, -192.9480, 831.2022 };
static float XM_SCALE[] = { 0.1943e-3, -0.0059e-3, -0.0008e-3, 0, 0.1930e-3, -0.0052e-3, 0, 0, 0.1901e-3 };

/* Initialize LSM9DS0 for gyroscope */
void init_lsm9ds0_gyro(_adcs_sensors *sensors) {
    uint8_t i2c_temp[2];
    /* Set to 0 all state values */
    memset(sensors->imu.gyr_raw, 0, 3);
    memset(sensors->imu.gyr, 0, 3);
    memset(sensors->imu.calib_gyr, 0, 3);
    sensors->imu.gyr_status = DEVICE_NORMAL;

    if (HAL_I2C_Mem_Read(&hi2c2, (GYRO_ADDR << 1), WHO_AM_I_G | LSM9DS0_MASK, 1,
            i2c_temp, 1, LSM9DS0_TIMEOUT) != HAL_OK) {
        sensors->imu.gyr_status = DEVICE_ERROR;
        return;
    }
    if (i2c_temp[0] != GYRO_ID) {
        sensors->imu.gyr_status = DEVICE_ERROR;
        return;
    }

    // 0x20 --> 0b01111111, 190Hz, LPF Cut off 70Hz
    // 0x21 --> 0b00100101, HPF cut off 0.45Hz
    // 0x22 --> 0b00000000
    // 0x23 --> 0b10000000, 245 DPS
    // 0x24 --> 0b00000000
    uint8_t GyroCTRreg[5] = { 0x7F, 0x25, 0x00, 0x80, 0x00 };
    if (HAL_I2C_Mem_Write(&hi2c2, (GYRO_ADDR << 1),
    CTRL_REG1_G | LSM9DS0_MASK, 1, GyroCTRreg, 5, LSM9DS0_TIMEOUT) != HAL_OK) {
        sensors->imu.gyr_status = DEVICE_ERROR;
        return;
    }

}

/* Calibration of lsm9ds0 gyroscope */
void calib_lsm9ds0_gyro(_adcs_sensors *sensors) {
    int i = GYRO_N, d, cnt;
    cnt = 0;

    while (i--) {
        update_lsm9ds0_gyro(sensors);
        if (sensors->imu.gyr_status == DEVICE_ERROR) {
            return;
        }
        cnt++;
        for (d = 0; d < 3; d++) {
            sensors->imu.calib_gyr[d] += (float) sensors->imu.gyr_raw[d];
            HAL_Delay(10);
        }
    }
    for (d = 0; d < 3; d++) {
        sensors->imu.calib_gyr[d] /= (float) cnt;
    }

//    sensors->imu.calib_gyr[0] = GYRO_OFFSET_X;
//    sensors->imu.calib_gyr[1] = GYRO_OFFSET_Y;
//    sensors->imu.calib_gyr[2] = GYRO_OFFSET_Z;
}

/* Update values for lsm9ds0 gyroscope */
void update_lsm9ds0_gyro(_adcs_sensors *sensors) {

    /* IMU, Gyroscope measure */
    if (HAL_I2C_Mem_Read(&hi2c2, (GYRO_ADDR << 1), GYRO_VAL | LSM9DS0_MASK, 1,
            (uint8_t *) sensors->imu.gyr_raw, 6,
            LSM9DS0_TIMEOUT) != HAL_OK) {
        sensors->imu.gyr_status = DEVICE_ERROR;
        return;
    }

    sensors->imu.gyr_status = DEVICE_NORMAL;

    sensors->imu.gyr[0] = -((float) sensors->imu.gyr_raw[2]
            - sensors->imu.calib_gyr[2]) * GYRO_GAIN; // -Zg
    sensors->imu.gyr[1] = -((float) sensors->imu.gyr_raw[1]
            - sensors->imu.calib_gyr[1]) * GYRO_GAIN; // -Yg
    sensors->imu.gyr[2] = -((float) sensors->imu.gyr_raw[0]
            - sensors->imu.calib_gyr[0]) * GYRO_GAIN; // -Xb
}

/* Initialize LSM9DS0 for magnetometer */
void init_lsm9ds0_xm(_adcs_sensors *sensors) {
    uint8_t i2c_temp[2];
    /* Set to 0 all state values */
    memset(sensors->imu.xm_raw, 0, 3);
    memset(sensors->imu.xm, 0, 3);
    memset(sensors->imu.xm_f, 0, 3);
    sensors->imu.xm_norm = 0;
    sensors->imu.xm_status = DEVICE_NORMAL;

    if (HAL_I2C_Mem_Read(&hi2c2, (XM_ADDR << 1), WHO_AM_I_XM | LSM9DS0_MASK, 1,
            i2c_temp, 1, LSM9DS0_TIMEOUT) != HAL_OK) {
        sensors->imu.xm_status = DEVICE_ERROR;
        return;
    }
    if (i2c_temp[0] != XM_ID) {
        sensors->imu.xm_status = DEVICE_ERROR;
        return;
    }

    // 0x1F --> 0b00000000
    // 0x20 --> 0b00001000
    // 0x21 --> 0b00000000
    // 0x22 --> 0b00000000
    // 0x23 --> 0b00000000
    // 0x24 --> 0b01110100, 100Hz Data rate
    // 0x25 --> 0b00000000, 2Gauss
    // 0x26 --> 0b00000000
    uint8_t XM_CTRreg[8] = { 0x00, 0x08, 0x00, 0x00, 0x0, 0x74, 0x00, 0x00 };

    if (HAL_I2C_Mem_Write(&hi2c2, (XM_ADDR << 1),
    XM_CTR_REG | LSM9DS0_MASK, 1, XM_CTRreg, 8, LSM9DS0_TIMEOUT) != HAL_OK) {
        sensors->imu.xm_status = DEVICE_ERROR;
        return;
    }

}

/* Update values for lsm9ds0 magnetometer*/
void update_lsm9ds0_xm(_adcs_sensors *sensors) {

    float offset[3] = { 0 };

    /* IMU, Magnetometer measure */
    if (HAL_I2C_Mem_Read(&hi2c2, (XM_ADDR << 1), XM_VAL | LSM9DS0_MASK, 1,
            (uint8_t *) sensors->imu.xm_raw, 6, LSM9DS0_TIMEOUT) != HAL_OK) {
        sensors->imu.xm_status = DEVICE_ERROR;
        return;
    }

    sensors->imu.xm_status = DEVICE_NORMAL;
    offset[0] = ((float) sensors->imu.xm_raw[0] - XM_OFFSET[0]);
    offset[1] = ((float) sensors->imu.xm_raw[1] - XM_OFFSET[1]);
    offset[2] = ((float) sensors->imu.xm_raw[2] - XM_OFFSET[2]);

    sensors->imu.xm[0] = offset[0] * XM_SCALE[6] + offset[1] * XM_SCALE[7]
            + offset[2] * XM_SCALE[8]; // Zxm
    sensors->imu.xm[1] = -(offset[0] * XM_SCALE[3] + offset[1] * XM_SCALE[4]
            + offset[2] * XM_SCALE[5]); // -Yxm
    sensors->imu.xm[2] = -(offset[0] * XM_SCALE[0] + offset[1] * XM_SCALE[1]
            + offset[2] * XM_SCALE[2]); // -Xxm

    sensors->imu.xm_norm = (float) norm(sensors->imu.xm[0], sensors->imu.xm[1],
            sensors->imu.xm[2]);
}

/* Get ID and set the Cycle Count Registers */
void init_rm3100(_adcs_sensors *sensors) {
    uint8_t spi_in_temp[9];
    uint8_t spi_out_temp[9];

    /* Set to 0 all adcs_sensors values */
    memset(sensors->mgn.rm_raw, 0, 3);
    memset(sensors->mgn.rm, 0, 3);
    memset(sensors->mgn.rm_f, 0, 3);
    sensors->mgn.rm_norm = 0;
    sensors->mgn.rm_status = DEVICE_NORMAL;

    /* Get ID */
    spi_in_temp[0] = PNI_REVID | STATUS_MASK;
    spi_in_temp[1] = 0xFF;
    HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(&hspi1, spi_in_temp, spi_out_temp, 2,
    PNI_TIMEOUT) != HAL_OK) {
        sensors->mgn.rm_status = DEVICE_ERROR;
        return;
    }
    HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_SET);
    if (spi_out_temp[1] != PNI_DEFAULT_ID) {
        sensors->mgn.rm_status = DEVICE_ERROR;
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
        sensors->mgn.rm_status = DEVICE_ERROR;
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
    float offset[3] = { 0 };

    /* Write POLL 0x00 register and followed 0x70 */
    spi_in_temp[0] = PNI_POLL;
    spi_in_temp[1] = SM_ALL_AXIS;
    HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi1, spi_in_temp, 2, PNI_TIMEOUT) != HAL_OK) {
        sensors->mgn.rm_status = DEVICE_ERROR;
        return;
    }
    HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_SET);
    /* Wait for LOW MISO */
    HAL_Delay(10);
    /* Read raw data */
    memset(spi_in_temp, 0, 10);
    spi_in_temp[0] = PNI_MX | STATUS_MASK;

    memset(spi_out_temp, 0, 10);

    HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(&hspi1, spi_in_temp, spi_out_temp, 10,
    PNI_TIMEOUT) != HAL_OK) {
        sensors->mgn.rm_status = DEVICE_ERROR;
        return;
    }
    HAL_GPIO_WritePin(RM_CS_GPIO_Port, RM_CS_Pin, GPIO_PIN_SET);

    sensors->mgn.rm_status = DEVICE_NORMAL;
    /* X Axis */
    ptr = (char*) (&tmp);
    ptr = ptr + 3;
    *ptr-- = spi_out_temp[1];
    *ptr-- = spi_out_temp[2];
    *ptr-- = spi_out_temp[3];
    sensors->mgn.rm_raw[0] = tmp >> 8;
    /* Y Axis */
    ptr = (char*) (&tmp);
    ptr = ptr + 3;
    *ptr-- = spi_out_temp[4];
    *ptr-- = spi_out_temp[5];
    *ptr-- = spi_out_temp[6];
    sensors->mgn.rm_raw[1] = tmp >> 8;
    /* Z Axis */
    ptr = (char*) (&tmp);
    ptr = ptr + 3;
    *ptr-- = spi_out_temp[7];
    *ptr-- = spi_out_temp[8];
    *ptr-- = spi_out_temp[9];
    sensors->mgn.rm_raw[2] = tmp >> 8;

    if (sensors->mgn.rm_raw[0] == 0 && sensors->mgn.rm_raw[1] == 0
            && sensors->mgn.rm_raw[1] == 0) {
        sensors->mgn.rm_status = DEVICE_ERROR;
        return;
    }

    offset[0] = ((float) sensors->mgn.rm_raw[0] - RM_OFFSET[0]);
    offset[1] = ((float) sensors->mgn.rm_raw[1] - RM_OFFSET[1]);
    offset[2] = ((float) sensors->mgn.rm_raw[2] - RM_OFFSET[2]);

    sensors->mgn.rm[0] = offset[0] * RM_SCALE[6] + offset[1] * RM_SCALE[7]
            + offset[2] * RM_SCALE[8]; // Zm
    sensors->mgn.rm[1] = offset[0] * RM_SCALE[0] + offset[1] * RM_SCALE[1]
            + offset[2] * RM_SCALE[2]; // Xm
    sensors->mgn.rm[2] = offset[0] * RM_SCALE[3] + offset[1] * RM_SCALE[4]
            + offset[2] * RM_SCALE[5]; // Ym

    sensors->mgn.rm_norm = (float) norm(sensors->mgn.rm[0], sensors->mgn.rm[1],
            sensors->mgn.rm[2]);

}

/* Initialize ADT7420 */
void init_adt7420(_adcs_sensors *sensors) {
    uint8_t i2c_temp[2];

    /* Set to 0 all state values */
    sensors->temp.temp_raw = 0;
    sensors->temp.temp_c = 0;
    sensors->temp.temp_status = DEVICE_NORMAL;

    i2c_temp[0] = 0;
    i2c_temp[1] = 0;
    if (HAL_I2C_Mem_Read(&hi2c2, ( ADT7420_ADDRESS << 1), ADT7420_REG_ID, 1,
            i2c_temp, 1, ADT7420_TIMEOUT) != HAL_OK) {
        sensors->temp.temp_status = DEVICE_ERROR;
        return;
    }
    if (i2c_temp[0] != ADT7420_DEFAULT_ID) {
        sensors->temp.temp_status = DEVICE_ERROR;
        return;
    }
    /* Set operation mode */
    i2c_temp[0] = ADT7420_16BIT | ADT7420_OP_MODE_1_SPS;
    i2c_temp[1] = 0x00;
    if (HAL_I2C_Mem_Write(&hi2c2, ( ADT7420_ADDRESS << 1),
    ADT7420_REG_CONFIG, 1, i2c_temp, 1, ADT7420_TIMEOUT) != HAL_OK) {
        sensors->temp.temp_status = DEVICE_ERROR;
        return;
    }
}

/* Update values for ADT7420 */
void update_adt7420(_adcs_sensors *sensors) {
    uint8_t lsb, msb;
    uint8_t i2c_temp[2];

    /* Get Temperature */
    i2c_temp[0] = 0;
    i2c_temp[1] = 0;
    if (HAL_I2C_Mem_Read(&hi2c2, ( ADT7420_ADDRESS << 1),
    ADT7420_REG_TEMP_MSB, 1, i2c_temp, 1, ADT7420_TIMEOUT) != HAL_OK) {
        sensors->temp.temp_status = DEVICE_ERROR;
        return;
    }
    msb = i2c_temp[0];
    if (HAL_I2C_Mem_Read(&hi2c2, ( ADT7420_ADDRESS << 1),
    ADT7420_REG_TEMP_LSB, 1, i2c_temp, 1, ADT7420_TIMEOUT) != HAL_OK) {
        sensors->temp.temp_status = DEVICE_ERROR;
        return;
    }

    sensors->temp.temp_status = DEVICE_NORMAL;
    lsb = i2c_temp[0];
    sensors->temp.temp_raw = msb << 8;
    sensors->temp.temp_raw |= lsb;

    if ((sensors->temp.temp_raw >> 15 & 1) == 0) {
        sensors->temp.temp_c = (float) sensors->temp.temp_raw / 128;
    } else {
        sensors->temp.temp_c = (float) (sensors->temp.temp_raw - 65536) / 128;
    }

}

/* Initialize sun sensor */
void init_sun_sensor(_adcs_sensors *sensors) {
    uint8_t spi_in_tmp[4], spi_out_tmp[4];

    memset(sensors->sun.v_sun_raw, 0, 4);
    memset(sensors->sun.v_sun, 0, 4);
    memset(sensors->sun.sun_rough, 0, 3);
    memset(sensors->sun.sun_fine, 0, 3);
    memset(sensors->sun.sun_xyz, 0, 3);
    sensors->sun.sun_status = DEVICE_NORMAL;

    memset(spi_in_tmp, 0, 4);
    spi_in_tmp[0] = AD7682_CFG | AD7682_INCC | AD7682_CH1 | AD7682_BW;
    spi_in_tmp[1] = AD7682_REF | AD7682_SEQ | AD7682_RB;
    memset(spi_out_tmp, 0, 4);

    HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_SET);
    HAL_Delay(6);
    HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    if (HAL_SPI_TransmitReceive(&hspi1, spi_in_tmp, spi_out_tmp, 5,
    AD7682_TIMEOUT) != HAL_OK) {
        sensors->sun.sun_status = DEVICE_ERROR;
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
        sensors->sun.sun_status = DEVICE_ERROR;
        return;
    }

    if (spi_out_tmp[3] != (AD7682_REF | AD7682_SEQ | AD7682_RB)) {
        sensors->sun.sun_status = DEVICE_ERROR;
        return;
    }
}

/* Update sun sensor values */
void update_sun_sensor(_adcs_sensors *sensors) {
    uint8_t i = 0;
    uint8_t j = 0;
    float long_rough_numerator[5] = { 0 };
    float long_rough_demominator[5] = { 0 };
    float lat_rough_numerator[5] = { 0 };
    float lat_rough_demominator[5] = { 0 };

    float long_rough_numerator_sum = 0;
    float long_rough_demominator_sum = 0;
    float lat_rough_numerator_sum = 0;
    float lat_rough_demominator_sum = 0;

    float meas_Valid = 0;
    float sun_rough[3][5] = { 0 };

    /* AD7682 Response to n-2 channel */
    if (update_ad7682(AD7682_CH1, &sensors->sun.v_sun_raw[3]) == DEVICE_ERROR) {
        sensors->sun.sun_status = DEVICE_ERROR;
        return;
    }
    if (update_ad7682(AD7682_CH2, &sensors->sun.v_sun_raw[4]) == DEVICE_ERROR) {
        sensors->sun.sun_status = DEVICE_ERROR;
        return;
    }
    if (update_ad7682(AD7682_CH3, &sensors->sun.v_sun_raw[0]) == DEVICE_ERROR) {
        sensors->sun.sun_status = DEVICE_ERROR;
        return;
    }
    if (update_ad7682(AD7682_CH4, &sensors->sun.v_sun_raw[1]) == DEVICE_ERROR) {
        sensors->sun.sun_status = DEVICE_ERROR;
        return;
    }
    if (update_ad7682(AD7682_CH5, &sensors->sun.v_sun_raw[2]) == DEVICE_ERROR) {
        sensors->sun.sun_status = DEVICE_ERROR;
        return;
    }

    sensors->sun.sun_status = DEVICE_NORMAL;
    /* Convert to V */
    for (i = 0; i < 5; i++) {
        sensors->sun.v_sun[i] = (float) sensors->sun.v_sun_raw[i] * AD7682_COEF;
    }

    /* For Testing */
//    sensors->sun.v_sun[4] = 3.3;
//    sensors->sun.v_sun[3] = 3.3;
//    sensors->sun.v_sun[2] = 1;
//    sensors->sun.v_sun[1] = 3.3;
//    sensors->sun.v_sun[0] = 1;
    /* Measure from sun sensor only if: 4*V5 − V1 − V2 − V3 − V4 ≥ 0.7
     * V5 is reference voltage */
    meas_Valid = 4 * sensors->sun.v_sun[4] - sensors->sun.v_sun[0]
            - sensors->sun.v_sun[1] - sensors->sun.v_sun[2]
            - sensors->sun.v_sun[3];
    if (meas_Valid >= SUN_SENSOR_VALID) {
        sensors->sun.sun_status = DEVICE_ENABLE;
        /* Calculate Rough numerator and demominator */
        for (i = 0; i < 5; i++) {
            long_rough_numerator[i] = SUN_SENSOR_K[i] * sensors->sun.v_sun[i];
            long_rough_numerator_sum += long_rough_numerator[i];

            long_rough_demominator[i] = SUN_SENSOR_N[i] * sensors->sun.v_sun[i];
            long_rough_demominator_sum += long_rough_demominator[i];

            lat_rough_numerator[i] = SUN_SENSOR_M[i] * sensors->sun.v_sun[i];
            lat_rough_numerator_sum += lat_rough_numerator[i];

            for (j = i; j < 5; j++) {
                lat_rough_demominator[i] += SUN_SENSOR_I[i][j] * sensors->sun.v_sun[i] * sensors->sun.v_sun[j];
            }
            lat_rough_demominator_sum += lat_rough_demominator[i];

            if ((long_rough_demominator[i] != 0) & (lat_rough_demominator[i] != 0)) {
                sun_rough[0][i] = atan2f(long_rough_numerator[i], long_rough_demominator[i]); // Lon
                sun_rough[1][i] = acosf(lat_rough_numerator[i] / sqrtf(lat_rough_demominator[i])); // Lat
                sun_rough[2][i] = 1; // Alt
            } else {
                sensors->sun.sun_status = DEVICE_ERROR;
                return;
            }
        }

        /* Calculate Rough Measure */
        if ((long_rough_demominator_sum != 0)
                & (lat_rough_demominator_sum != 0)) {
            sensors->sun.sun_rough[0] = atan2f(long_rough_numerator_sum, long_rough_demominator_sum);
            sensors->sun.sun_rough[1] = acosf(lat_rough_numerator_sum / sqrtf(lat_rough_demominator_sum));
            sensors->sun.sun_rough[2] = 1;
        } else {
            sensors->sun.sun_status = DEVICE_ERROR;
            return;
        }

        /* Calculate Fine Measures */
        for (i = 0; i < S_SUN_SENSOR; i++) {
            for (j = 0; j < S_SUN_SENSOR - i; j++) {
                sensors->sun.sun_fine[0] += SUN_SENSOR_P[i][j] * sun_rough[0][i]
                        * sun_rough[1][j]; // Lon
                sensors->sun.sun_fine[1] += SUN_SENSOR_Q[i][j] * sun_rough[0][i]
                        * sun_rough[1][j]; // Lat
                sensors->sun.sun_fine[2] = 1; // Alt
            }
        }

        /* Convert to XYZ */
        sensors->sun.sun_xyz[1] = -sinf(sensors->sun.sun_rough[1])
                * cosf(sensors->sun.sun_rough[0]); // Yb
        sensors->sun.sun_xyz[0] = sinf(sensors->sun.sun_rough[1])
                * sinf(sensors->sun.sun_rough[0]); // -Xb
        sensors->sun.sun_xyz[2] = -cosf(sensors->sun.sun_rough[1]); // -Zb
    } else {
        sensors->sun.sun_status = DEVICE_DISABLE;
    }

}

/* Update values for AD7682 */
_adcs_sensor_status update_ad7682(uint8_t ch, uint16_t *v_raw) {
    uint8_t spi_in_temp[4], spi_out_temp[4];

    memset(spi_in_temp, 0, 4);
    spi_in_temp[0] = AD7682_CFG | AD7682_INCC | ch | AD7682_BW;
    spi_in_temp[1] = AD7682_REF | AD7682_SEQ | AD7682_RB;
    memset(spi_out_temp, 0, 4);

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
    if (spi_out_temp[3] != (AD7682_REF | AD7682_SEQ | AD7682_RB)) {
        return DEVICE_ERROR;
    } else {
        *v_raw = spi_out_temp[0] << 8;
        *v_raw |= spi_out_temp[1];
        return DEVICE_NORMAL;
    }
}
