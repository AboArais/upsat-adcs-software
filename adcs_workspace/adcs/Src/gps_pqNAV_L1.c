/*
 * gps.c
 *
 *  Created on: May 18, 2016
 *      Author: azisi
 */

#include "gps_pqNAV_L1.h"

extern UART_HandleTypeDef huart4;

void
gps_init ()
{
  HAL_UART_Receive_IT (&huart4, gps_buffer, GPS_BUF_SIZE);
}

SAT_returnState
HAL_gps_rx (TC_TM_app_id app_id, uint8_t *uart_buf)
{
  UART_HandleTypeDef *huart;

  huart = &huart4;

  if (huart->RxState == HAL_UART_STATE_READY) {
    gps_buff_size = huart->RxXferSize - huart->RxXferCount;
    HAL_UART_Receive_IT (huart, uart_buf, GPS_BUF_SIZE);
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

    __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);

    /* Disable the UART Parity Error Interrupt */
    __HAL_UART_DISABLE_IT(huart, UART_IT_PE);

    /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

    /* Rx process is completed, restore huart->RxState to Ready */
    huart->RxState = HAL_UART_STATE_READY;
  }
  else if (huart->RxXferSize > huart->RxXferCount) {
    *huart->pRxBuffPtr++ = c;
    huart->RxXferCount--;
  }

  if (huart->RxXferCount == 0U) // errror
      {

  }

}
