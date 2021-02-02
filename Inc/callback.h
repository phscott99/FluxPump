/**
  ******************************************************************************
  * @file    callback.h
  * @brief   This file contains all the function prototypes for
  *          the callback.c file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CALLBACK_H
#define __CALLBACK_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Variables -----------------------------------------------------------------*/
extern volatile uint8_t transHalfCplt_Flag;
extern volatile uint8_t transFullCplt_Flag;
extern volatile uint8_t switch1Cplt_Flag;
extern volatile uint8_t switch2Cplt_Flag;
extern volatile uint8_t commandReceived_Flag;

/* Function Prototypes -------------------------------------------------------*/
void transHalfCpltCallback(DAC_HandleTypeDef *hdac);
void transFullCpltCallback(DAC_HandleTypeDef *hdac);
void preCycleStartCallback(TIM_HandleTypeDef *htim);
void s1BurstCpltCallback(TIM_HandleTypeDef *htim);
void s2BurstCpltCallback(TIM_HandleTypeDef *htim);
void commandReceivedCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /*__ CALLBACK_H */

/****END OF FILE****/