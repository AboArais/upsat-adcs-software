/*
 * gps_pqNAV_L1.c
 *
 *  Created on: May 18, 2016
 *      Author: azisi
 */

#include "gps_pqNAV_L1.h"
#include "stm32f4xx_hal.h"

static uint8_t gps[GPS_TEMP_BUFF][GPS_BUF_SIZE];
static uint8_t gps_flag[GPS_TEMP_BUFF] = {false};
static uint8_t gps_pointer = 0;

extern UART_HandleTypeDef huart4;

void
gps_init (uint8_t *uart_gps_buf)
{
  HAL_UART_Receive_IT (&huart4, uart_gps_buf, GPS_BUF_SIZE);
}

SAT_returnState
HAL_gps_rx (TC_TM_app_id app_id, uint8_t *uart_gps_buf)
{
  UART_HandleTypeDef *huart;

  huart = &huart4;

  if (huart->RxState == HAL_UART_STATE_READY) {
    uint8_t gps_buffer_size = huart->RxXferSize - huart->RxXferCount;
    uart_gps_buf[gps_buffer_size] = 0;
    //LOG_UART_DBG(&huart2, "%c\n%s\n%c", 0x7E, adcs_state.gps_buf, 0x7E);
    //HAL_UART_Receive_IT (huart, uart_gps_buf, GPS_BUF_SIZE);
    return SATR_EOT;
  }
  return SATR_OK;
}

/**
 * @brief  This function handles UART interrupt request.
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void
HAL_GPS_UART_IRQHandler (UART_HandleTypeDef *huart)
{
  uint32_t tmp1 = 0U, tmp2 = 0U;

  tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE);
  tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE);
  /* UART in mode Receiver ---------------------------------------------------*/
  if ((tmp1 != RESET) && (tmp2 != RESET)) {
    UART_GPS_Receive_IT (huart);
  }
}

/**
 * @brief  Receives an amount of data in non blocking mode
 * @param  huart: pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval HAL status
 */
void
UART_GPS_Receive_IT (UART_HandleTypeDef *huart)
{
  uint8_t c;

  c = (uint8_t) (huart->Instance->DR & (uint8_t) 0x00FFU);
  if (huart->RxXferSize == huart->RxXferCount && c == '$') {
    *huart->pRxBuffPtr++ = c;
    huart->RxXferCount--;
    //start timeout
  }
  else if (c == '$') {
    huart->RxXferCount = huart->RxXferSize;

    *huart->pRxBuffPtr++ = c;
    huart->RxXferCount--;
    //error
    //event log
    //reset buffers & pointers
    //start timeout
  }
  else if (c == '\n') {
    *huart->pRxBuffPtr++ = c;
    huart->RxXferCount--;

    *huart->pRxBuffPtr++ = 0;

    gps_flag[gps_pointer] = true;
    if(++gps_pointer < GPS_TEMP_BUFF) { gps_pointer = 0; }
    huart->pRxBuffPtr = &gps[gps_pointer];

    huart->RxXferCount = huart->RxXferSize;

    //__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);

    /* Disable the UART Parity Error Interrupt */
    //__HAL_UART_DISABLE_IT(huart, UART_IT_PE);
    /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    //__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
    /* Rx process is completed, restore huart->RxState to Ready */
    huart->RxState = HAL_UART_STATE_READY;
  }
  else if (huart->RxXferSize > huart->RxXferCount) {
    *huart->pRxBuffPtr++ = c;
    huart->RxXferCount--;
  }

  if (huart->RxXferCount == 0U) // errror
  {
        uart->pRxBuffPtr = &gps[gps_pointer];
        huart->RxXferCount = huart->RxXferSize;

  }

}

void reset_gps_flag (const uint8_t i) {

  gps_flag[i] = false;
}

uint8_t * get_gps_buff (const uint8_t i, uint8_t *flag) {

  *flag = gps_flag[i];
  return gps[i];
}
