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
#include "adcs_control.h"
#include "adcs.h"
#include "service_utilities.h"
#include "log.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

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

uint8_t uart_temp[500];
_adcs_state adcs_state;
_adcs_actuator adcs_actuator;
uint8_t dbg_msg = 0;

#pragma location = 0x0800C000
const uint32_t key[(16 * 1000) / 4];

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
static void
MX_CRC_Init (void);
static void
MX_IWDG_Init (void);

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
  MX_CRC_Init ();
  MX_IWDG_Init ();

  /* USER CODE BEGIN 2 */

  /* Kick timer interrupt for timed threads */
  /*kick_TIM7_timed_interrupt(TIMED_EVENT_PERIOD);*/

  /* Switch ON sensors-GPS */
  adcs_switch (SWITCH_ON, SENSORS, &adcs_state);
  adcs_switch (SWITCH_ON, GPS, &adcs_state);

  /* GPS delay */
  //HAL_Delay(30000);
  init_sens (&adcs_state);
  init_magneto_torquer (&adcs_actuator);
  gps_init (adcs_state.gps_buf);

  /* ecss */
  uint8_t rsrc = 0;
  HAL_reset_source (&rsrc);
  set_reset_source (rsrc);
  pkt_pool_INIT ();
  uint16_t size = 0;
  event_crt_pkt_api (uart_temp, "ADCS STARTED", 666, 666, "", &size, SATR_OK);
  HAL_uart_tx (DBG_APP_ID, (uint8_t *) uart_temp, size);
  event_dbg_api (uart_temp, "ADCS STARTED\n", &size);
  HAL_uart_tx (DBG_APP_ID, (uint8_t *) uart_temp, size);
  HAL_UART_Receive_IT (&huart2, adcs_data.obc_uart.uart_buf, UART_BUF_SIZE);

  /*
   TLE

   ISS (ZARYA)
   1 25544U 98067A   16118.56765286  .00004329  00000-0  72029-4 0  9994
   2 25544  51.6443 310.5306 0002107  73.7950  28.9464 15.54350419997100
   NaN

   ISS (ZARYA)
   1 25544U 98067A   16124.14033565  .00005548  00000-0  90051-4 0  9992
   2 25544  51.6441 282.7446 0001865  87.1449 259.7301 15.54414600997966
   OK

   ISS (ZARYA)
   1 25544U 98067A   16137.55001157  .00005721  00000-0  91983-4 0  9991
   2 25544  51.6440 215.8562 0001995 108.3968  92.4187 15.54614828    61
   OK
   */
  adcs_state.orb_tle.ep_year = 16;
  adcs_state.orb_tle.ep_day = 137.55001157;
  adcs_state.orb_tle.rev = 15.54614828;
  adcs_state.orb_tle.bstar = 0.00175395 * powf (10.0, -5);
  adcs_state.orb_tle.eqinc = RAD(51.6440);
  adcs_state.orb_tle.ecc = 0.0001995;
  adcs_state.orb_tle.mnan = RAD(92.4187);
  adcs_state.orb_tle.argp = RAD(108.3968);
  adcs_state.orb_tle.ascn = RAD(215.8562);
  adcs_state.orb_tle.norb = 0;
  adcs_state.orb_tle.satno = 13;
  update_tle (&adcs_state);
  uint8_t tleup = 0;

  uint8_t gps_cnt = 0;
  uint8_t gps_flag = 0;
  uint8_t *gps_buf;
  uint8_t test_buf[100];

  adcs_actuator.RPM = 10;
  adcs_actuator.rampTime = 0;
  adcs_actuator.current_x = 0;
  adcs_actuator.current_y = 0;

  //flash_write_trasmit(0xffaf0f6412);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    import_pkt (OBC_APP_ID, &adcs_data.obc_uart);

    update_spin_torquer (&adcs_actuator);
    update_magneto_torquer (&adcs_actuator);

    /* ADCS tests */
    switch (dbg_msg)
      {
      case 0:
	break;
      case 1:
	snprintf (test_buf, 100, "m %.3f %.3f %.3f\n", adcs_state.rm_mag[0],
		  adcs_state.rm_mag[1], adcs_state.rm_mag[2]);
	event_dbg_api (uart_temp, test_buf, &size);
	HAL_uart_tx (DBG_APP_ID, (uint8_t *) uart_temp, size);
	break;
      case 2:
	snprintf (test_buf, 100, "g %.3f %.3f %.3f\n", adcs_state.gyr[0],
		  adcs_state.gyr[1], adcs_state.gyr[2]);
	event_dbg_api (uart_temp, test_buf, &size);
	HAL_uart_tx (DBG_APP_ID, (uint8_t *) uart_temp, size);
	break;
      case 3:
	snprintf (test_buf, 100, "v %.3f %.3f %.3f %.3f %.3f\n",
		  adcs_state.v_sun[0], adcs_state.v_sun[1], adcs_state.v_sun[2],
		  adcs_state.v_sun[3], adcs_state.v_sun[4],
		  adcs_state.v_sun[5]);
	event_dbg_api (uart_temp, test_buf, &size);
	HAL_uart_tx (DBG_APP_ID, (uint8_t *) uart_temp, size);
	break;
      case 4:
	snprintf (test_buf, 100, "l %.3f %.3f\n", adcs_state.long_sun,
		  adcs_state.lat_sun);
	event_dbg_api (uart_temp, test_buf, &size);
	HAL_uart_tx (DBG_APP_ID, (uint8_t *) uart_temp, size);
	break;
      case 5:
	snprintf (test_buf, 100, "s %d, %d\n", adcs_actuator.m_RPM,
		  adcs_actuator.flag);
	event_dbg_api (uart_temp, test_buf, &size);
	HAL_uart_tx (DBG_APP_ID, (uint8_t *) uart_temp, size);
	break;
      case 6:
	snprintf (test_buf, 100, "t %.3f\n", adcs_state.temp_c);
	event_dbg_api (uart_temp, test_buf, &size);
	HAL_uart_tx (DBG_APP_ID, (uint8_t *) uart_temp, size);
	break;
      case 8:
	for (gps_cnt = 0; gps_cnt < 10; gps_cnt++) {
	  gps_buf = get_gps_buff (gps_cnt, &gps_flag);

	  if (gps_flag == 1) {
	    reset_gps_flag (gps_cnt);
	    size = 0;
	    event_dbg_api (uart_temp, gps_buf, &size);
	    HAL_uart_tx (DBG_APP_ID, (uint8_t *) uart_temp, size);
	    HAL_Delay (10);
	  }
	}
	break;
      default:
	break;
      }
    HAL_Delay (100);

    /* CubeSat propagator */
    LOG_UART_FILE(&huart2, "%.5f \t", adcs_state.jd);

    update_sgdp4 (&adcs_state);
    update_geomag (&adcs_state);
    update_sun_pos (&adcs_state);
    if (tleup == 1) {
      calculate_tle (&adcs_state);
      update_tle (&adcs_state);
      tleup = 0;
    }

    /* Update ADCS state */
    update_sens (&adcs_state);

    LOG_UART_FILE(&huart2, "%.3f \t %.3f \t %.3f \n", adcs_state.p_ECEF_LLH.alt,
		  adcs_state.p_ECEF_LLH.lat, adcs_state.p_ECEF_LLH.lon);

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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
      | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig (&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config (HAL_RCC_GetHCLKFreq () / 1000);

  HAL_SYSTICK_CLKSourceConfig (SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (SysTick_IRQn, 0, 0);
}

/* CRC init function */
void
MX_CRC_Init (void)
{

  hcrc.Instance = CRC;
  HAL_CRC_Init (&hcrc);

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

/* IWDG init function */
void
MX_IWDG_Init (void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  HAL_IWDG_Init (&hiwdg);

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
