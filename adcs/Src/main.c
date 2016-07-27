/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include "adcs_configuration.h"
#include "adcs_sensors.h"
#include "adcs_gps.h"
#include "adcs_actuators.h"
#include "adcs_switch.h"
#include "adcs_time.h"
#include "adcs_frame.h"
#include "adcs_flash.h"
#include "WahbaRotM.h"
#include "adcs_error_handler.h"

#include "sgp4.h"
#include "geomag.h"
#include "sun_pos.h"

#include "adcs.h"
#include "service_utilities.h"
#include "event_reporting_service.h"

#include "log.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t uart_temp[500];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

    /* USER CODE BEGIN 1 */

    /* Reset error status */
    error_status = ERROR_OK;

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C2_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_UART4_Init();
    MX_USART2_UART_Init();
    MX_TIM7_Init();
    MX_CRC_Init();
    MX_RTC_Init();

    /* USER CODE BEGIN 2 */

    /* Initialize ~3.6s */
    HAL_Delay(100);

    /* Initialize flash memory */
    uint8_t flash_id = 0;
    flash_init();
    if (flash_readID(&flash_id) != FLASH_NORMAL) {
        error_status = ERROR_FLASH;
    }

    /* Calculate Boot counter */
    if (flash_increment_boot_counter() != FLASH_NORMAL) {
        error_status = ERROR_FLASH;
    }

    /* ecss */
    uint8_t rsrc = 0;
    HAL_reset_source(&rsrc);
    set_reset_source(rsrc);
    pkt_pool_INIT();
    event_boot(rsrc, adcs_boot_cnt);
    HAL_UART_Receive_IT(&huart2, adcs_data.obc_uart.uart_buf, UART_BUF_SIZE);

    /* Get time from OBC */
    adcs_time.utc.year = 16;
    adcs_time.utc.month = 7;
    adcs_time.utc.day = 23;
    adcs_time.utc.weekday = RTC_WEEKDAY_MONDAY;
    adcs_time.utc.hour = 23;
    adcs_time.utc.min = 0;
    adcs_time.utc.sec = 0;
    set_time_UTC(adcs_time.utc);

    for (uint8_t i = 0; i < NUM_REQUEST_TIME_OBC; i++) {
        time_management_request_time_in_utc(OBC_APP_ID);
        export_pkt(OBC_APP_ID, &adcs_data.obc_uart);
//        import_pkt(OBC_APP_ID, &adcs_data.obc_uart);
        get_time_UTC(&adcs_time.utc);
        if (adcs_time.utc.year == 0) {
            error_status = ERROR_TIME;
        } else {
            error_status = ERROR_OK;
            break;
        }
    }

    /* Initialize GPS */
//    uint8_t gps_sentence[NMEA_MAX_LEN];
//    memset(gps_sentence, 0, NMEA_MAX_LEN);
    get_time_UTC(&adcs_time.utc);
    gps_state.status = gps_init(adcs_time.utc);
    if (gps_state.status == GPS_ERROR) {
        error_status = ERROR_GPS;
    }

    /* Initialize actuators */
    if (init_spin_torquer(&adcs_actuator) == MOTOR_ERROR) {
        error_status = ERROR_ACTUATOR;
    }
    init_magneto_torquer(&adcs_actuator);

    /* Initialize SGP4 and TLE read */
    sgp4_status satpos_status = SGP4_ERROR;
    orbit_t temp_tle;
    init_satpos_xyz();
    if (flash_read_tle(&temp_tle) == FLASH_ERROR) {
        error_status = ERROR_FLASH;
    }
    if (update_tle(&upsat_tle, temp_tle) == TLE_ERROR) {
        error_status = ERROR_TLE;
    }

    /* Initialize IGRF model */
    xyz_t p_ecef;
    llh_t p_ecef_llh;
    p_ecef.x = 0; p_ecef.y = 0; p_ecef.z = 0;
    p_ecef_llh.lat = 0; p_ecef_llh.lon = 0; p_ecef_llh.alt = 0;
    init_geomag(&igrf_vector);

    /* Initialize Sun position model */
    init_sun(&sun_vector);
    xyz_t sun_ned_vector;
    sun_ned_vector.x = 0; sun_ned_vector.y = 0; sun_ned_vector.z = 0;

    /* Initialize sensors */
//  adcs_pwr_switch(SWITCH_ON, SENSORS);
//  HAL_Delay(100);
    if (init_lsm9ds0_gyro(&adcs_sensors) == DEVICE_ERROR) {
        error_status = ERROR_SENSOR;
    }
    calib_lsm9ds0_gyro(&adcs_sensors);
    if (init_lsm9ds0_xm(&adcs_sensors) == DEVICE_ERROR) {
        error_status = ERROR_SENSOR;
    }
    if (init_rm3100(&adcs_sensors) == DEVICE_ERROR) {
        error_status = ERROR_SENSOR;
    }
    if (init_sun_sensor(&adcs_sensors) == DEVICE_ERROR) {
        error_status = ERROR_SENSOR;
    }
    if (init_adt7420(&adcs_sensors) == DEVICE_ERROR) {
        error_status = ERROR_SENSOR;
    }
    float xm_prev[3] = { 0 };
    float rm_prev[3] = { 0 };
    float gyr_prev[3] = { 0 };
    uint8_t sens_cnt = 0;

    /* Initialize Attitude determination */
    initWahbaStruct(&WahbaRot, LOOP_TIME);

    /* Kick timer interrupt for timed threads */
    ADCS_event_period_status = TIMED_EVENT_SERVICED;
    if (kick_TIM7_timed_interrupt(LOOP_TIME_TICKS) == HAL_ERROR) {
        ADCS_event_period_status = TIMED_EVENT_NOT_SERVICED;
    }

    /* Initialize and Refresh WDG timer */
    MX_IWDG_Init();
    HAL_IWDG_Refresh(&hiwdg);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {

        import_pkt(OBC_APP_ID, &adcs_data.obc_uart);
        export_pkt(OBC_APP_ID, &adcs_data.obc_uart);

        /* GPS update */
        if (gps_state.status == GPS_ON || gps_state.status == GPS_UNLOCK) {

//            for (gps_cnt = 0; gps_cnt < NMEA_NUM_SENTENCES; gps_cnt++) {
                if (get_gps_flag() == true) {
                    reset_gps_flag();
                }
//            }
            //gps_parse_fields(gps_sent, const uint8_t size, uint8_t (*res)[NMEA_MAX_FIELDS][NMEA_MAX_FIELD_SIZE]);

            if (gps_state.d3fix == 3 && gps_state.num_sat >= 3) {
                /* Update time and set ADCS RTC  */
                adcs_time.gps_time = gps_state.time;
                adcs_time.gps_week = gps_state.week;
                gps2utc(&adcs_time);
                set_time_UTC(adcs_time.utc);
                /* Update time of OBC */
                time_management_force_time_update(OBC_APP_ID);
                /* Calculate new TLE */
                tle_epoch(&adcs_time);
                temp_tle = calculate_tle(gps_state.p_gps_ecef,
                        gps_state.v_gps_ecef, adcs_time.tle_epoch);
                if (update_tle(&upsat_tle, temp_tle) == TLE_NORMAL) {
                    if (flash_write_tle(&upsat_tle) == FLASH_ERROR) {
                        error_status = ERROR_FLASH;
                    }
                } else {
                    error_status = ERROR_TLE;
                }
                gps_state.status = GPS_LOCK;
            } else {
                gps_state.status = GPS_UNLOCK;
            }
        } else if (gps_state.status == GPS_LOCK) {
            adcs_pwr_switch(SWITCH_OFF, GPS);
            gps_state.status = GPS_OFF;
        } else if (gps_state.status == GPS_ERROR) {
            error_status = ERROR_GPS;
        }

        /* Control loop runs at 168ms, interrupt runs every ~1.048s, WDG at ~2s */
        if (ADCS_event_period_status == TIMED_EVENT_NOT_SERVICED) {

            /* Send to EPS a packet */
            update_eps_pkt();

            /* Get time for RTC*/
            get_time_UTC(&adcs_time.utc);
            if (adcs_time.utc.year != 0) {
                /* When set time with OBC RTC update GPS_ALARM only for the first time*/
                if (error_status == ERROR_TIME) {
                    HAL_SetAlarm_GPS_ON(adcs_time.utc);
                    error_status = ERROR_OK;
                }
                /* Calculate decimal year and Julian day */
                decyear(&adcs_time);
                julday(&adcs_time);

                /* Update position and velocity of satellite */
                satpos_status = satpos_xyz(adcs_time.jd, &p_eci, &v_eci);

                if (satpos_status == SGP4_ZERO_ECC || satpos_status == SGP4_NEAR_SIMP || satpos_status == SGP4_NEAR_NORM) {
                    /* Calculate IGRF reference vectors */
                    ECI2ECEF(adcs_time.jd, p_eci, &p_ecef);
                    cart2spher(p_ecef, &p_ecef_llh);
                    igrf_vector.sdate = adcs_time.decyear;
                    igrf_vector.latitude = p_ecef_llh.lat;
                    igrf_vector.longitude = p_ecef_llh.lon;
                    igrf_vector.alt = p_ecef_llh.alt;
                    geomag(&igrf_vector); // Return igrf status, Xm,Ym,Zm in NED
                    igrf_vector.norm = norm(igrf_vector.Xm, igrf_vector.Ym,
                            igrf_vector.Zm);

                    /* Calculate sun position reference vectors */
                    sun_vector.JD_epoch = adcs_time.jd;
                    sun(&sun_vector); // Sun position in ECI
                    ECI2NED(sun_vector.sun_pos, &sun_ned_vector, upsat_tle.ascn,
                            upsat_tle.eqinc, upsat_tle.argp + upsat_tle.mnan); // Convert to NED
                    sun_vector.norm = norm(sun_ned_vector.x, sun_ned_vector.y,
                            sun_ned_vector.z);

                    /* Close Magneto-torquer for 56ms */
                    adcs_actuator.magneto_torquer.current_z = 0;
                    adcs_actuator.magneto_torquer.current_y = 0;
                    update_magneto_torquer(&adcs_actuator);

                    /* Calculate measurement vectors and filter */
                    if (update_adt7420(&adcs_sensors) == DEVICE_ERROR) {
                        error_status = ERROR_SENSOR;
                    }
                    if (update_sun_sensor(&adcs_sensors) == DEVICE_ERROR) {
                        error_status = ERROR_SENSOR;
                    }
                    if (update_lsm9ds0_gyro(&adcs_sensors) == DEVICE_ERROR) {
                        error_status = ERROR_SENSOR;
                    }
                    if (update_rm3100(&adcs_sensors) == DEVICE_ERROR) {
                        error_status = ERROR_SENSOR;
                    }
                    if (update_lsm9ds0_xm(&adcs_sensors) == DEVICE_ERROR) {
                        error_status = ERROR_SENSOR;
                    }
                    for (sens_cnt = 0; sens_cnt < 3; sens_cnt++) {
                        adcs_sensors.imu.xm_f[sens_cnt] = A_FILTER
                                * adcs_sensors.imu.xm[sens_cnt]
                                + (1 - A_FILTER) * xm_prev[sens_cnt];
                        xm_prev[sens_cnt] = adcs_sensors.imu.xm[sens_cnt];
                        adcs_sensors.imu.gyr_f[sens_cnt] = A_FILTER
                                * adcs_sensors.imu.gyr[sens_cnt]
                                + (1 - A_FILTER) * gyr_prev[sens_cnt];
                        gyr_prev[sens_cnt] = adcs_sensors.imu.gyr[sens_cnt];
                        adcs_sensors.mgn.rm_f[sens_cnt] = A_FILTER
                                * adcs_sensors.mgn.rm[sens_cnt]
                                + (1 - A_FILTER) * rm_prev[sens_cnt];
                        rm_prev[sens_cnt] = adcs_sensors.mgn.rm[sens_cnt];
                    }

                    /* Attitude determination */
                    /* Set IGRF reference vectors */
                    WahbaRot.w_m[0] = igrf_vector.Xm / igrf_vector.norm;
                    WahbaRot.w_m[1] = igrf_vector.Ym / igrf_vector.norm;
                    WahbaRot.w_m[2] = igrf_vector.Zm / igrf_vector.norm;
                    /* Set Sun position reference vectors */
                    WahbaRot.w_a[0] = sun_ned_vector.x / sun_vector.norm;
                    WahbaRot.w_a[1] = sun_ned_vector.y / sun_vector.norm;
                    WahbaRot.w_a[2] = sun_ned_vector.z / sun_vector.norm;

                    WahbaRotM(adcs_sensors.sun.sun_xyz, adcs_sensors.imu.gyr_f,
                            adcs_sensors.mgn.rm_f, &WahbaRot);
                } else {
                    error_status = ERROR_SGP4;
                }

                /* Control Law */
                adcs_actuator.magneto_torquer.current_z = 37;
                adcs_actuator.magneto_torquer.current_y = 0;
                adcs_actuator.spin_torquer.RPM = 0;
                update_magneto_torquer(&adcs_actuator);
                if (update_spin_torquer(&adcs_actuator) == MOTOR_ERROR) {
                    //error_status = ERROR_ACTUATOR;
                }
            } else {
                error_status = ERROR_TIME;
                time_management_request_time_in_utc(OBC_APP_ID);
            }

            /* Update flag */
            ADCS_event_period_status = TIMED_EVENT_SERVICED;
        }

        /* Software error handler */
        error_handler(error_status);
        error_status = ERROR_OK;

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

    }
    /* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
            | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function */
static void MX_CRC_Init(void) {

    hcrc.Instance = CRC;
    if (HAL_CRC_Init(&hcrc) != HAL_OK) {
        Error_Handler();
    }

}

/* I2C2 init function */
static void MX_I2C2_Init(void) {

    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }

}

/* IWDG init function */
static void MX_IWDG_Init(void) {

    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Reload = 250;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }

}

/* RTC init function */
static void MX_RTC_Init(void) {

    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC and set the Time and Date 
     */
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        Error_Handler();
    }

    sTime.Hours = 0;
    sTime.Minutes = 0;
    sTime.Seconds = 0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }

    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 1;
    sDate.Year = 0;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }

    /**Enable the Alarm A 
     */
    sAlarm.AlarmTime.Hours = 0;
    sAlarm.AlarmTime.Minutes = 0;
    sAlarm.AlarmTime.Seconds = 0;
    sAlarm.AlarmTime.SubSeconds = 0;
    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay = 1;
    sAlarm.Alarm = RTC_ALARM_A;
    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }

}

/* SPI1 init function */
static void MX_SPI1_Init(void) {

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }

}

/* SPI2 init function */
static void MX_SPI2_Init(void) {

    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }

}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
            != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
            != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
            != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void) {

    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
            != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
            != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
            != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
            != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4)
            != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim4);

}

/* TIM7 init function */
static void MX_TIM7_Init(void) {

    TIM_MasterConfigTypeDef sMasterConfig;

    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 2048;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 50000;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig)
            != HAL_OK) {
        Error_Handler();
    }

}

/* UART4 init function */
static void MX_UART4_Init(void) {

    huart4.Instance = UART4;
    huart4.Init.BaudRate = 9600;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(SENS_EN_GPIO_Port, SENS_EN_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(FM_nHLD_GPIO_Port, FM_nHLD_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(CNV_GPIO_Port, CNV_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, RM_CS_Pin | FM_nCE_Pin | FM_nWP_Pin,
            GPIO_PIN_RESET);

    /*Configure GPIO pins : GPS_EN_Pin RM_CS_Pin FM_nCE_Pin FM_nWP_Pin */
    GPIO_InitStruct.Pin = GPS_EN_Pin | RM_CS_Pin | FM_nCE_Pin | FM_nWP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : SENS_EN_Pin FM_nHLD_Pin */
    GPIO_InitStruct.Pin = SENS_EN_Pin | FM_nHLD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : CNV_Pin */
    GPIO_InitStruct.Pin = CNV_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CNV_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler */
    error_status = ERROR_HAL_INIT;
    /* User can add his own implementation to report the HAL error return state */
    while (1) {
        return;
    }
    /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
