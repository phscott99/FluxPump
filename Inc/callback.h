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
extern volatile uint8_t transHalfDMA_Flag;
extern volatile uint8_t transFullDMA_Flag;
extern volatile uint8_t switch1Cplt_Flag;
extern volatile uint8_t switch2Cplt_Flag;
extern volatile uint8_t currentHalfDMA_Flag;
extern volatile uint8_t currentFullDMA_Flag;
extern volatile uint8_t commandReceived_Flag;

/* Function Prototypes -------------------------------------------------------*/
void transHalfDMACallback(DAC_HandleTypeDef *hdac);
void transFullDMACallback(DAC_HandleTypeDef *hdac);
void preCycleStartCallback(TIM_HandleTypeDef *htim);
void s1BurstCpltCallback(TIM_HandleTypeDef *htim);
void s2BurstCpltCallback(TIM_HandleTypeDef *htim);
void currentHalfDMACallback(ADC_HandleTypeDef* hadc);
void currentFullDMACallback(ADC_HandleTypeDef* hadc);
void commandReceivedCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /*__ CALLBACK_H */

/****END OF FILE****/