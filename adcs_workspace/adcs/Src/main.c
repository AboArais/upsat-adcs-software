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
#include "adcs_state.h"
#include "adcs.h"
#include "service_utilities.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

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
adcs_state adcs_board_state;

uint8_t uart_temp[200];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void
SystemClock_Config (void);
static void
MX_GPIO_Init (void);
static void
MX_DMA_Init (void);
static void
MX_I2C2_Init (void);
static void
MX_SDIO_SD_Init (void);
static void
MX_SPI1_Init (void);
static void
MX_SPI2_Init (void);
static void
MX_TIM3_Init (void);
static void
MX_TIM4_Init (void);
static void
MX_UART4_Init (void);
static void
MX_USART2_UART_Init (void);
static void
MX_TIM7_Init (void);

void
HAL_TIM_MspPostInit (TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int
main (void)
{

  /* USER CODE BEGIN 1 */
  /* For Debug */
  char uart_tmp[20];

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init ();

  /* Configure the system clock */
  SystemClock_Config ();

  /* Initialize all configured peripherals */
  MX_GPIO_Init ();
  MX_DMA_Init ();
  MX_I2C2_Init ();
  MX_SDIO_SD_Init ();
  MX_SPI1_Init ();
  MX_SPI2_Init ();
  MX_TIM3_Init ();
  MX_TIM4_Init ();
  MX_UART4_Init ();
  MX_USART2_UART_Init ();
  MX_TIM7_Init ();

  /* USER CODE BEGIN 2 */

  /* Kick timer interrupt for timed threads */
  /*kick_TIM7_timed_interrupt(TIMED_EVENT_PERIOD);*/

  adcs_init_state (&adcs_board_state);

  HAL_reset_source (&sys_data.rsrc);
  update_boot_counter ();
  pkt_pool_INIT ();
  uint16_t size = 0;
  event_crt_pkt_api (uart_temp, "ADCS STARTED", 666, 666, "", &size, SATR_OK);
  HAL_uart_tx (DBG_APP_ID, (uint8_t *) uart_temp, size);
  HAL_UART_Receive_IT (&huart2, adcs_data.obc_uart.uart_buf, UART_BUF_SIZE);

  extern double SGDP4_jd0;
  geomagStruct gStr;
  gTime gT;
  orbit_t orb;
  double jd;
  xyz_t pos, pos_ecef;
  xyz_t vel;
  float llh_ecef[3];
  float gst, Cgst, Sgst;
  uint16_t cnt;
  int8_t imode;
  /*TLE
   ISS (ZARYA)
   1 25544U 98067A   16118.56765286  .00004329  00000-0  72029-4 0  9994
   2 25544  51.6443 310.5306 0002107  73.7950  28.9464 15.54350419997100
   Generate in satpos_xyz() NaN

   ISS (ZARYA)
   1 25544U 98067A   16124.14033565  .00005548  00000-0  90051-4 0  9992
   2 25544  51.6441 282.7446 0001865  87.1449 259.7301 15.54414600997966
   OK
   */
  orb.ep_year = 16; /* Year of epoch, e.g. 94 for 1994, 100 for 2000AD */
  orb.ep_day = 124.14033565; /* Day of epoch from 00:00 Jan 1st ( = 1.0 ) */
  orb.rev = 15.54414600; /* Mean motion, revolutions per day */
  orb.bstar = 9.0051 * powf (10.0, -5); /* Drag term .*/
  orb.eqinc = RAD(51.6441); /* Equatorial inclination, radians */
  orb.ecc = 0.0001865; /* Eccentricity 0.0002107*/
  orb.mnan = RAD(259.7301); /* Mean anomaly at epoch from elements, radians */
  orb.argp = RAD(87.1449); /* Argument of perigee, radians */
  orb.ascn = RAD(282.7446); /* Right ascension (ascending node), radians */
  orb.norb = 99796; /* Orbit number, for elements */
  orb.satno = 25544; /* Satellite number. */
  imode = init_sgdp4 (&orb);

  switch (imode)
    {
    case SGDP4_ERROR:
      /*printf ("# SGDP error\n");*/
      break;
    case SGDP4_NOT_INIT:
      /*printf ("# SGDP not init\n");*/
      break;
    case SGDP4_ZERO_ECC:
      /*printf ("# SGDP zero ecc\n");*/
      break;
    case SGDP4_NEAR_SIMP:
      /*printf ("# SGP4 simple\n");*/
      break;
    case SGDP4_NEAR_NORM:
      /*printf ("# SGP4 normal\n");*/
      break;
    case SGDP4_DEEP_NORM:
      /*printf ("# SDP4 normal\n");*/
      break;
    case SGDP4_DEEP_RESN:
      /*printf ("# SDP4 resonant\n");*/
      break;
    case SGDP4_DEEP_SYNC:
      /*printf ("# SDP4 synchronous\n");*/
      break;
    default:
      /*printf ("# SGDP mode not recognised!\n");*/
      break;
    }

  jd = SGDP4_jd0;
  cnt = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    import_pkt (OBC_APP_ID, &adcs_data.obc_uart);

    adcs_update_state (&adcs_board_state);

    /* sgdp4-geomag test */
    if (cnt <= 100) {
      if (satpos_xyz (jd, &pos, &vel) != SGDP4_ERROR) {
	/* ECI to ECEF  correct?*/
	gst = gha_aries (jd);
	Cgst = cosf (gst);
	Sgst = sinf (gst);
	pos_ecef.x = pos.x * Cgst + pos.y * Sgst;
	pos_ecef.y = -pos.x * Sgst + pos.y * Cgst;
	pos_ecef.z = pos.z;
	/* If get values from GPS */
	/* ECEF, Cartesian to Spherical */
	/* Altitude */
	llh_ecef[2] = sqrtf (
	    pos_ecef.x * pos_ecef.x + pos_ecef.y * pos_ecef.y
		+ pos_ecef.z * pos_ecef.z);
	/* Latitude [-90, 90] */
	llh_ecef[0] = asinf (pos_ecef.z / llh_ecef[2]) * 180.0 / M_PI;
	/* Longitude [-180, 180] */
	llh_ecef[1] = atan2f (pos_ecef.y, pos_ecef.x) * 180.0 / M_PI;

	//gStr.latitude = llh_ecef[0];
	//gStr.longitude = llh_ecef[1];
	//gStr.alt = llh_ecef[2];
	//JD2Greg (jd, &gT);
	//JD2Greg (jd, &gT);
	//gT.decyear = decyear (gT);
	//gStr.sdate = gT.decyear;

	//geomag (&gStr);
	//NED2ECEF(&gStr);
	jd = jd + 1.0 / 1440.0; //every minute
	snprintf (uart_temp, 100, "%.3f \t", jd);
	HAL_UART_Transmit (&huart2, uart_temp, strlen (uart_temp), 1000);

	snprintf (uart_temp, 100, "%.3f \t", llh_ecef[2]);
	HAL_UART_Transmit (&huart2, uart_temp, strlen (uart_temp), 1000);

	snprintf (uart_temp, 100, "%.3f \t", llh_ecef[0]);
	HAL_UART_Transmit (&huart2, uart_temp, strlen (uart_temp), 1000);

	snprintf (uart_temp, 100, "%.3f \n", llh_ecef[1]);
	HAL_UART_Transmit (&huart2, uart_temp, strlen (uart_temp), 1000);

	cnt++;
      }
      /* Update TLE from GPS */
    }

    /* Serial debug */
    /*sprintf(uart_tmp, "T:%.3f \n", adcs_board_state.temp_c );
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);

     sprintf(uart_tmp, "Xm:%.3f \t", adcs_board_state.rm_mag[0]);
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);
     sprintf(uart_tmp, "Ym:%.3f \t", adcs_board_state.rm_mag[1]);
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);
     sprintf(uart_tmp, "Zm:%.3f \n", adcs_board_state.rm_mag[2]);
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);

     sprintf(uart_tmp, "Vx:%.3f \t", adcs_board_state.gyr[0]);
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);
     sprintf(uart_tmp, "Vy:%.3f \t", adcs_board_state.gyr[1]);
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);
     sprintf(uart_tmp, "Vz:%.3f \n", adcs_board_state.gyr[2]);
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);

     sprintf(uart_tmp, "V1:%.3f \t", adcs_board_state.v_sun[0]);
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);
     sprintf(uart_tmp, "V2:%.3f \t", adcs_board_state.v_sun[1]);
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);
     sprintf(uart_tmp, "V3:%.3f \t", adcs_board_state.v_sun[2]);
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);
     sprintf(uart_tmp, "V4:%.3f \t", adcs_board_state.v_sun[3]);
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);
     sprintf(uart_tmp, "V5:%.3f \n", adcs_board_state.v_sun[4]);
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);

     sprintf(uart_tmp, "Longitude:%.3f \t", adcs_board_state.long_sun);
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);
     sprintf(uart_tmp, "Latitude:%.3f \n", adcs_board_state.lat_sun);
     HAL_UART_Transmit(&huart2, uart_tmp, strlen(uart_tmp), 100);*/
    /****************/
    HAL_Delay (100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
 */
void
SystemClock_Config (void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE()
  ;

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig (&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config (HAL_RCC_GetHCLKFreq () / 1000);

  HAL_SYSTICK_CLKSourceConfig (SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
void
MX_I2C2_Init (void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init (&hi2c2);

}

/* SDIO init function */
void
MX_SDIO_SD_Init (void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  HAL_SD_Init (&hsd, &SDCardInfo);

}

/* SPI1 init function */
void
MX_SPI1_Init (void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init (&hspi1);

}

/* SPI2 init function */
void
MX_SPI2_Init (void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init (&hspi2);

}

/* TIM3 init function */
void
MX_TIM3_Init (void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init (&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization (&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel (&htim3, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel (&htim3, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_MspPostInit (&htim3);

}

/* TIM4 init function */
void
MX_TIM4_Init (void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init (&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization (&htim4, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel (&htim4, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel (&htim4, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel (&htim4, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel (&htim4, &sConfigOC, TIM_CHANNEL_4);

  HAL_TIM_MspPostInit (&htim4);

}

/* TIM7 init function */
void
MX_TIM7_Init (void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 32;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = ControlLoop;
  HAL_TIM_Base_Init (&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization (&htim7, &sMasterConfig);

}

/* UART4 init function */
void
MX_UART4_Init (void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init (&huart4);

}

/* USART2 init function */
void
MX_USART2_UART_Init (void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init (&huart2);

}

/** 
 * Enable DMA controller clock
 */
void
MX_DMA_Init (void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE()
  ;

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (DMA1_Stream6_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
void
MX_GPIO_Init (void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE()
  ;
  __HAL_RCC_GPIOH_CLK_ENABLE()
  ;
  __HAL_RCC_GPIOA_CLK_ENABLE()
  ;
  __HAL_RCC_GPIOB_CLK_ENABLE()
  ;
  __HAL_RCC_GPIOD_CLK_ENABLE()
  ;

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (GPIOC, SD_EN_Pin | CNV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (GPIOA, GPS_EN_Pin | RM_CS_Pin | FM_nCE_Pin | FM_nWP_Pin,
		     GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (GPIOB, SENS_EN_Pin | FM_nHLD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SD_EN_Pin CNV_Pin */
  GPIO_InitStruct.Pin = SD_EN_Pin | CNV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_EN_Pin RM_CS_Pin FM_nCE_Pin FM_nWP_Pin */
  GPIO_InitStruct.Pin = GPS_EN_Pin | RM_CS_Pin | FM_nCE_Pin | FM_nWP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SENS_EN_Pin FM_nHLD_Pin */
  GPIO_InitStruct.Pin = SENS_EN_Pin | FM_nHLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
