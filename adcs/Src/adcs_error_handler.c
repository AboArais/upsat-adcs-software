/*
 * adcs_error_handler.c
 *
 *  Created on: Jul 16, 2016
 *      Author: azisi
 */

#include "stm32f4xx_hal.h"
#include "adcs_error_handler.h"
#include "adcs_switch.h"

#define TR_ERROR_OK         0x05
#define TR_ERROR_USRESOLVED 0x10
#define TR_ERROR_SGP4       0x15
#define TR_ERROR_TLE        0x20
#define TR_ERROR_TIME       0x25
#define TR_ERROR_SENSOR     0x30
#define TR_ERROR_FLASH      0x35
#define TR_ERROR_ACTUATOR   0x40
#define TR_ERROR_GPS        0x45
#define TR_ERROR_HAL_INIT   0x50

extern IWDG_HandleTypeDef hiwdg;

adcs_error_status error_status;
uint8_t trasmit_error_status;

/**
 * @brief Checks the status, if it is a known issue from the enumeration, tries
 * to solve it in software else a soft reset is trigerred. if the error status
 * is ok, the watchdog is updated.
 * @param error
 */
void error_handler(adcs_error_status error) {

    switch (error) {
    case ERROR_OK:
        HAL_IWDG_Refresh(&hiwdg);
        trasmit_error_status = TR_ERROR_OK;
        break;
    case ERROR_SGP4:
        /* Send from the ground new TLE or update the TLE from GPS */
        HAL_IWDG_Refresh(&hiwdg);
        trasmit_error_status = TR_ERROR_SGP4;
        break;
    case ERROR_TLE:
        /* Send from the ground new TLE or update the TLE from GPS */
        HAL_IWDG_Refresh(&hiwdg);
        trasmit_error_status = TR_ERROR_TLE;
        break;
    case ERROR_TIME:
        /* Resolve in main loop, request time from OBC or from GPS */
        HAL_IWDG_Refresh(&hiwdg);
        trasmit_error_status = TR_ERROR_TIME;
        break;
    case ERROR_SENSOR:
        /* Close the power of sensors */
        HAL_IWDG_Refresh(&hiwdg);
        adcs_pwr_switch(SWITCH_OFF, SENSORS);
        trasmit_error_status = TR_ERROR_SENSOR;
        HAL_Delay(10);
        adcs_pwr_switch(SWITCH_ON, SENSORS);
        break;
    case ERROR_ACTUATOR:
        HAL_IWDG_Refresh(&hiwdg);
        /* Software reset of I2C */
        GPIOB->BSRR = GPIO_PIN_10 | GPIO_PIN_11;
        uint8_t i = 8;
        while ((i--) && ((GPIOB->IDR & GPIO_PIN_11) == 0)) {
            HAL_Delay(1);
            GPIOB->BSRR = (uint32_t)GPIO_PIN_10 << 16U;
            HAL_Delay(1);
            GPIOB->BSRR = GPIO_PIN_10;
        }
        trasmit_error_status = TR_ERROR_ACTUATOR;
        break;
    case ERROR_FLASH:
        HAL_IWDG_Refresh(&hiwdg);
        trasmit_error_status = TR_ERROR_FLASH;
        // Send to OBC command to open GPS
        break;
    case ERROR_GPS:
        HAL_IWDG_Refresh(&hiwdg);
        trasmit_error_status = TR_ERROR_GPS;
        adcs_pwr_switch(SWITCH_OFF, GPS);
        // Send to OBC command to open GPS
        break;
    case ERROR_UNRESOLVED:
        trasmit_error_status = TR_ERROR_USRESOLVED;
        break;
    default:
        break;
    }

}
