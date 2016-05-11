/*
 * eps_configuration.h
 *
 *  Created on: May 8, 2016
 *      Author: azisi
 */

#ifndef INC_ADCS_CONFIGURATION_H_
#define INC_ADCS_CONFIGURATION_H_

#include "stm32f4xx_hal.h"

/* IMU LSM9DS0,  I2C */
#define GYRO_ADDR 0x6B
#define GYRO_OFFSET_X 47
#define GYRO_OFFSET_Y 57
#define GYRO_OFFSET_Z -55

/* RM3100 Magnerometer, SPI1 */
/* REGISTER MAP          R/W	  Default	Format	Description */
#define PNI_POLL	0x0		/* RW	0		7:0]	Polls for a Single Measurement */
#define PNI_CMM		0x1		/* RW	0		[7:0]	Initiates Continuous Measurement Mode */
#define PNI_CCX		0x04	/* RW	00C8	UInt16	Cycle Count Register – X Axis */
#define PNI_CCY		0x06	/* RW	00C8	UInt16	Cycle Count Register – Y Axis */
#define PNI_CCZ		0x089	/* RW	00C8	Uint16	Cycle Count Register – Z Axis */
#define PNI_TMRC	0x0B	/* RW	96		[7:0]	Sets Continuous Measurement Mode Data Rate */
#define PNI_ALLX	0x0C	/* RW	0		Uint24	Alarm Lower Limit – X Axis */
#define PNI_AULX	0x0F	/* RW	0		Uint24	Alarm Upper Limit – X Axis */
#define PNI_ALLY	0x12	/* RW	0		Uint24	Alarm Lower Limit – Y Axis */
#define PNI_AULY	0x15	/* RW	0		Uint24	Alarm Upper Limit – Y Axis */
#define PNI_ALLZ	0x18	/* RW	0		Uint24	Alarm Lower Limit – Z Axis */
#define PNI_AULZ	0x1B	/* RW	0		Uint24	Alarm Upper Limit – Z Axis */
#define PNI_ADLX	0x1E	/* RW	0		UInt16	Alarm Hysteresis Value – X Axis */
#define PNI_ADLY	0x20	/* RW	0		UInt16	Alarm Hysteresis Value – Y Axis */
#define PNI_ADLZ	0x22	/* RW	0		UInt16	Alarm Hysteresis Value – Z Axis */
#define PNI_MX		0x24	/* R		0		Uint24	Measurement Results – X Axis */
#define PNI_MY		0x27	/* R		0		Uint24	Measurement Results – Y Axis */
#define PNI_MZ		0x2A	/* R		0		Uint24	Measurement Results – Z Axis */
#define PNI_BIST	0x33	/* RW	0		[7:0]	Built-In Self Test */
#define PNI_STATUS	0x34	/* R		0		[7:0]	Status of DRDY */
#define PNI_HSHAKE	0x35	/* RW	1B		[7:0]	Handshake Register */
#define PNI_REVID	0x36	/* R		--		Unit8	MagI2C Revision Identification */
#define SM_ALL_AXIS 0x70    /* Single measurment mode */
#define STATUS_MASK 0x80    /* To get status of data ready */

/* Set Up PWM Start up Duty-Cycle */
#define MAGN_STARTUP_PWM_DUTYCYCLE ((uint32_t) 0)

/* Configure timer */
extern TIM_HandleTypeDef htim7;
#define TIMED_EVENT_PERIOD ((uint32_t)50000)
void kick_TIM7_timed_interrupt(uint32_t control_loop);

#endif /* INC_ADCS_CONFIGURATION_H_ */
