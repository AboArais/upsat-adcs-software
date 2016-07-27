/*
 * adcs_gps.c
 *
 *  Created on: May 18, 2016
 *      Author: azisi
 */

#include "stm32f4xx_hal.h"

#include "adcs_gps.h"
#include "adcs_flash.h"
#include "adcs_switch.h"

static uint8_t gps[NMEA_MAX_LEN];
static uint8_t gps_flag = false;

uint8_t gps_sentence[NMEA_MAX_LEN];

extern UART_HandleTypeDef huart4;
extern RTC_HandleTypeDef hrtc;

/**
 * @brief Initialize GPS serial and alarm.
 * @param uart_gps_buf
 * @param gps_utc
 */
_gps_status gps_init(struct time_utc gps_utc) {

    /* Set up UART4 for GPS */
    uint8_t uart_gps_buf[NMEA_MAX_LEN];
    memset(uart_gps_buf, 0, NMEA_MAX_LEN);
    memset(gps_sentence, 0, NMEA_MAX_LEN);
    HAL_UART_Receive_IT(&huart4, &uart_gps_buf, NMEA_MAX_LEN);

    uint32_t flash_read_address = GPS_ALARM_BASE_ADDRESS;
    uint8_t gps_flash[4];
    flash_status gps_flash_status = FLASH_NORMAL;

    memset(gps_flash, 0, 4);
    for (uint8_t i = 0; i < 4; i++) {
        if (flash_read_byte(&gps_flash[i], flash_read_address) == FLASH_NORMAL) {
            flash_read_address = flash_read_address + GPS_ALARM_OFFSET_ADDRESS;
        } else {
            gps_flash_status = FLASH_ERROR;
            break;
        }
    }
    /* Is not the first boot and the time is set up from OBC and no error in flash */
    if (adcs_boot_cnt > 1 && gps_utc.year != 0 && gps_flash_status == FLASH_NORMAL) {
        if (HAL_SetAlarm_GPS(gps_flash[0], gps_flash[1], gps_flash[2]) == GPS_NORMAL) {
            return gps_status_flash(gps_flash[3]);
        } else {
            return GPS_ERROR;
        }
    /* First time that ADCS boot or the time is not set up from OBC or error with flash */
    } else {
        if (gps_flash_status == FLASH_ERROR) {
            return GPS_ERROR;
        } else {
            return HAL_SetAlarm_GPS_ON(gps_utc);
        }
    }

}

/**
 * @brief  This function handles UART interrupt request.
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_GPS_UART_IRQHandler(UART_HandleTypeDef *huart) {
    uint32_t tmp1 = 0U, tmp2 = 0U;

    tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE);
    tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE);
    /* UART in mode Receiver ---------------------------------------------------*/
    if ((tmp1 != RESET) && (tmp2 != RESET)) {
        UART_GPS_Receive_IT(huart);
    }
}

/**
 * @brief  Receives an amount of data in non blocking mode
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval HAL status
 */
void UART_GPS_Receive_IT(UART_HandleTypeDef *huart) {
    uint8_t c;

    c = (uint8_t) (huart->Instance->DR & (uint8_t) 0x00FFU);
    if (huart->RxXferSize == huart->RxXferCount && c == '$') {
        *huart->pRxBuffPtr++ = c;
        huart->RxXferCount--;
        //start timeout
    } else if (c == '$') {
        huart->RxXferCount = huart->RxXferSize;

        *huart->pRxBuffPtr++ = c;
        huart->RxXferCount--;
        //error
        //event log
        //reset buffers & pointers
        //start timeout
    } else if (c == 0xa) {
        *huart->pRxBuffPtr++ = c;
        huart->RxXferCount--;
        *huart->pRxBuffPtr++ = 0;
        gps_flag = true;
        huart->pRxBuffPtr = &gps;
        memset(gps_sentence, 0 , NMEA_MAX_LEN);
        for (uint8_t i = 0 ; i < NMEA_MAX_LEN; i++){
            gps_sentence[i] = gps[i];
        }
        huart->RxXferCount = huart->RxXferSize;
    } else if (huart->RxXferSize > huart->RxXferCount) {
        *huart->pRxBuffPtr++ = c;
        huart->RxXferCount--;
    }
    /* Errror */
    if (huart->RxXferCount == 0U) {
        //huart->pRxBuffPtr = &gps;
        huart->RxXferCount = huart->RxXferSize;
    }

}

void reset_gps_flag() {

    gps_flag = false;
}

uint8_t get_gps_flag() {

    return gps_flag;
}

void HAL_GPS_Alarm_Handler(_gps_status gps_status_value) {
    struct time_utc gps_utc_handler;
    switch (gps_status_value) {
    case GPS_OFF:
        adcs_pwr_switch(SWITCH_ON, GPS);
        get_time_UTC(&gps_utc_handler);
        gps_state.status = HAL_SetAlarm_GPS_LOCK(gps_utc_handler);
        break;
    case GPS_UNLOCK:
        gps_state.status = GPS_ERROR;
        break;
    case GPS_LOCK:
        gps_state.status = GPS_OFF;
        break;
    default:
        break;
    }
}

/**
 * @brief Set up GPS alarm, to open GPS every 5 hours and save in flash the
 *        alarm time and GPS status.
 * @param gps_utc
 */
_gps_status HAL_SetAlarm_GPS_ON(struct time_utc gps_utc) {

    uint8_t gps_flash[3];
    memset(gps_flash, 0, 3);

    gps_flash[0] = (gps_utc.hour + GPS_ALARM_HOUR) % 24;
    gps_flash[1] = gps_utc.min;
    gps_flash[2] = gps_utc.sec;
    HAL_SetAlarm_GPS(gps_flash[0], gps_flash[1], gps_flash[2]);

    /* Write to flash GPS alarm to recover after reset */
    if (flash_erase_block4K(GPS_ALARM_BASE_ADDRESS) == FLASH_NORMAL) {
        flash_write_page(gps_flash, GPS_ALARM_BASE_ADDRESS);
    } else {
        return GPS_ERROR;
    }

    return GPS_OFF;

}

/**
 * @brief Set up GPS alarm, to wait 20 minutes for cold start and save in flash
 *        the alarm and GPS status.
 * @param gps_utc
 */
_gps_status HAL_SetAlarm_GPS_LOCK(struct time_utc gps_utc) {

    uint8_t gps_flash[3];

    memset(gps_flash, 0, 3);

    if (gps_utc.min + GPS_ALARM_MIN > 60) {
        gps_flash[0] = gps_utc.hour + 1;
    } else {
        gps_flash[0] = gps_utc.hour;
    }

    gps_flash[1] = (gps_utc.min + GPS_ALARM_MIN) % 60;
    gps_flash[2] = gps_utc.sec;
    HAL_SetAlarm_GPS(gps_flash[0], gps_flash[1], gps_flash[2]);

    /* Write to flash GPS alarm to recover after reset */
    if (flash_erase_block4K(GPS_ALARM_BASE_ADDRESS) == FLASH_NORMAL) {
        flash_write_page(gps_flash, GPS_ALARM_BASE_ADDRESS);
    } else {
        return GPS_ERROR;
    }

    return GPS_ON;
}

/**
 * @brief Set GPS alarm to interrupt every day at specific time.
 * @param gps_alarm_hour
 * @param gps_alarm_min
 * @param gps_alarm_sec
 */
_gps_status HAL_SetAlarm_GPS(uint8_t gps_alarm_hour, uint8_t gps_alarm_min, uint8_t gps_alarm_sec) {

    RTC_AlarmTypeDef gps_alarm;

    gps_alarm.AlarmTime.Hours = gps_alarm_hour;
    gps_alarm.AlarmTime.Minutes = gps_alarm_min;
    gps_alarm.AlarmTime.Seconds = gps_alarm_sec;
    gps_alarm.AlarmTime.SubSeconds = 0;
    gps_alarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    gps_alarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    gps_alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
    gps_alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    gps_alarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    gps_alarm.AlarmDateWeekDay = 1;
    gps_alarm.Alarm = RTC_ALARM_A;
    if (HAL_RTC_SetAlarm_IT(&hrtc, &gps_alarm, RTC_FORMAT_BIN) != HAL_OK) {
        return GPS_ERROR;
    }

    return GPS_NORMAL;

}

_gps_status gps_status_flash(uint8_t gps_status_value) {
    switch (gps_status_value) {
    case (0):
        return GPS_ERROR;
    case (1):
        return GPS_NORMAL;
    case (2):
        return GPS_ON;
    case (3):
        return GPS_OFF;
    case (4):
        return GPS_LOCK;
    case (5):
        return GPS_UNLOCK;
    default:
        return GPS_ERROR;
    }
}
