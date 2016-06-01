/*
 * adcs_state.h
 *
 *  Created on: May 8, 2016
 *      Author: azisi
 */

#ifndef INC_ADCS_STATE_H_
#define INC_ADCS_STATE_H_

#include <gps_pqNAV_L1.h>
#include "adcs_configuration.h"
#include "sgdp4h.h"
#include "sgp4ext.h"
/* Geomagnetic Field Model */
#include "geomag.h"
/* Sun Position */
#include "sun_pos.h"

/* IMU LSM9DS0,  I2C */
#define GYRO_ADDR	0x6B
#define GYRO_VAL	0x28
#define GYRO_MASK	0x80
#define CTRL_REG1_G	0x20
#define WHO_AM_I_G	0x0F
#define GYRO_ID		0xD4
#define GYRO_TIMEOUT	300
/* Gyro offsets */
#define GYRO_OFFSET_X	47
#define GYRO_OFFSET_Y	57
#define GYRO_OFFSET_Z	-55

/* RM3100 Magnerometer, SPI1 */
#define PNI_POLL	0x0	/* Polls for a Single Measurement */
#define PNI_CMM		0x1	/* Initiates Continuous Measurement Mode */
#define PNI_CCX		0x04	/* Cycle Count Register – X Axis */
#define PNI_CCY		0x06	/* Cycle Count Register – Y Axis */
#define PNI_CCZ		0x089	/* Cycle Count Register – Z Axis */
#define PNI_TMRC	0x0B	/* Sets Continuous Measurement Mode Data Rate */
#define PNI_ALLX	0x0C	/* Alarm Lower Limit – X Axis */
#define PNI_AULX	0x0F	/* Alarm Upper Limit – X Axis */
#define PNI_ALLY	0x12	/* Alarm Lower Limit – Y Axis */
#define PNI_AULY	0x15	/* Alarm Upper Limit – Y Axis */
#define PNI_ALLZ	0x18	/* Alarm Lower Limit – Z Axis */
#define PNI_AULZ	0x1B	/* Alarm Upper Limit – Z Axis */
#define PNI_ADLX	0x1E	/* Alarm Hysteresis Value – X Axis */
#define PNI_ADLY	0x20	/* Alarm Hysteresis Value – Y Axis */
#define PNI_ADLZ	0x22	/* Alarm Hysteresis Value – Z Axis */
#define PNI_MX		0x24	/* Measurement Results – X Axis */
#define PNI_MY		0x27	/* Measurement Results – Y Axis */
#define PNI_MZ		0x2A	/* Measurement Results – Z Axis */
#define PNI_BIST	0x33	/* Built-In Self Test */
#define PNI_STATUS	0x34	/* Status of DRDY */
#define PNI_HSHAKE	0x35	/* Handshake Register */
#define PNI_REVID	0x36	/* MagI2C Revision Identification */
#define SM_ALL_AXIS	0x70	/* Single measurment mode */
#define STATUS_MASK	0x80	/* To get status of data ready */
#define PNI_CyclesMSB	0x00
#define PNI_CyclesLSB	0xC8
#define PNI_GAIN	1e3/75	/* 75/1e6 Sensitivity LSB/uT to convert nT 1e3/75 */
#define PNI_TIMEOUT	1000

/* ADT7420 address, IC2*/
#define ADT7420_ADDRESS		0x48
/* ADT7420 default ID */
#define ADT7420_DEFAULT_ID	0xCB
/* ADT7420 registers */
#define ADT7420_REG_TEMP_MSB	0x00	/* Temperature value MSB */
#define ADT7420_REG_TEMP_LSB	0x01	/* Temperature value LSB */
#define ADT7420_REG_STATUS	0x02	/* Status */
#define ADT7420_REG_CONFIG	0x03	/* Configuration */
#define ADT7420_REG_T_HIGH_MSB	0x04	/* Temperature HIGH setpoint MSB */
#define ADT7420_REG_T_HIGH_LSB	0x05	/* Temperature HIGH setpoint LSB */
#define ADT7420_REG_T_LOW_MSB	0x06	/* Temperature LOW setpoint MSB */
#define ADT7420_REG_T_LOW_LSB	0x07	/* Temperature LOW setpoint LSB */
#define ADT7420_REG_T_CRIT_MSB	0x08	/* Temperature CRIT setpoint MSB */
#define ADT7420_REG_T_CRIT_LSB	0x09	/* Temperature CRIT setpoint LSB */
#define ADT7420_REG_HIST	0x0A	/* Temperature HYST setpoint */
#define ADT7420_REG_ID		0x0B	/* ID */
#define ADT7420_REG_RESET	0x2F	/* Software reset */
#define ADT7420_TIMEOUT		100
/* ADT7420 configure */
#define ADT7420_16BIT		0x80
#define ADT7420_OP_MODE_1_SPS	0x40
#define ADT7420_OP_MODE_CONT_CONV 0x00

/* AD7689 */
#define AD7682_CFG	0x80		/* Configuration update */
#define AD7682_INCC	0x70		/* Input channel configuration */
#define AD7682_BW	0x01		/* Bandwidth for low-pass filter */
#define AD7682_REF	0x20		/* Reference/buffer selection */
#define AD7682_SEQ	0x00		/* Channel sequencer */
#define AD7682_RB	0x00		/* Read back the CFG register */
#define AD7682_CH5	0x0A		/* Channel 5 */
#define AD7682_CH4	0x08		/* Channel 4 */
#define AD7682_CH3	0x06		/* Channel 3 */
#define AD7682_CH2	0x04		/* Channel 2 */
#define AD7682_CH1	0x02		/* Channel 1 */
#define AD7682_COEF	0.0000625	/* Convert digital measure to analog */
#define AD7682_TIMEOUT	100
/* TLE update */
#define CD	2.3		/* Drag Coefficient clear number */
#define RHO	5.75E-13	/* Atmospheric Density Kg/m^3 in 400 km */
#define M	2.3		/* Mass of cubesat in Kg */
#define A	0.1*0.1	/* Cross-sectional effective Area m^2*/

typedef enum
{
  SWITCH_ON = 1, SWITCH_OFF
} _switch_state;

typedef enum
{
  SENSORS = 1, GPS
} _adcs_switch;

typedef struct
{
  /* Sensor Switch */
  _switch_state sens_sw;
  /* GPS Switch */
  _switch_state gps_sw;
  /* LSSM9DS0 - Gyroscope */
  int16_t gyr_raw[3];
  float gyr[3];
  float calib_gyr[3];
  /* RM3100 Magnetometer */
  int32_t rm_raw[3];
  float rm_mag[3]; /* in uΤ */
  float rm_gain;
  /* ADT7420 Temperature */
  uint16_t temp_raw;
  float temp_c;
  /* Sun Sensor */
  uint16_t v_sun_raw[5];
  float v_sun[5];
  float long_rough;
  float lat_rough;
  float long_sun;
  float lat_sun;
  /* GPS */
  uint8_t gps_buf[500];
  /* SGDP4 */
  xyz_t p_ECI, v_ECI;
  orbit_t orb_tle;
  xyz_t p_ECEF;
  llh_t p_ECEF_LLH;
  double jd;
  /* Geomag */
  geomagStruct mag_ECEF;
  gTime gen_time;
  /* Sun Position */
  double p_sun_ECI[3];
} _adcs_state;

/* ON-OFF Switches */
void
adcs_switch (_switch_state switch_state, _adcs_switch adcs_switch,
	     volatile _adcs_state *state);

/* Init ADCS state */
void
init_sens (volatile _adcs_state *state);
void
init_lsm9ds0 (volatile _adcs_state *state);
void
init_rm3100 (volatile _adcs_state *state);
void
init_adt7420 (volatile _adcs_state *state);
void
init_sun_sensor (volatile _adcs_state *state);
void
calib_lsm9ds0_gyro (volatile _adcs_state *state);

/* Update ADCS state */
void
update_sens (volatile _adcs_state *state);
void
update_lsm9ds0 (volatile _adcs_state *state);
void
update_rm3100 (volatile _adcs_state *state);
void
update_adt7420 (volatile _adcs_state *state);
uint16_t
update_ad7682 (uint8_t ch);
void
update_sun_sensor (volatile _adcs_state *state);

/* TLE Update sgp4 propagator */
void
calculate_tle (volatile _adcs_state *state);
void
update_tle (volatile _adcs_state *state);
void
update_sgdp4 (volatile _adcs_state *state);

/* */
void
update_geomag (volatile _adcs_state *state);
/* */
void
update_sun_pos (volatile _adcs_state *state);

#endif /* INC_ADCS_STATE_H_ */
