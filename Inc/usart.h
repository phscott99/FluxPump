/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
extern DMA_HandleTypeDef hdma_lpuart1_rx; // Make this an external handle so it can be easily referenced by CLI code
/* USER CODE END Includes */

extern UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN Private defines */
#define RX_BUFFER_SIZE 100 // Max. 100 characters for RX Buffer (could be reduced if commands stay short)
#define TX_BUFFER_SIZE 100 // Max. 100 characters for TX Buffer (could be reduced)

extern char LPUART1_rxBuffer[RX_BUFFER_SIZE]; // Make UART RX Buffer external to be easily referenced by CLI code
extern char LPUART1_txBuffer[TX_BUFFER_SIZE]; // Make UART TX Buffer external for easy access
/* USER CODE END Private defines */

void MX_LPUART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

void configureUART(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
