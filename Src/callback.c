/**
  ******************************************************************************
  * File Name          : callback.c
  * Description        : This file provides code for various
  *                      callback routines
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "callback.h"
#include "signal.h"
#include "dac.h"

/* Variables -----------------------------------------------------------------*/
volatile uint8_t transHalfDMA_Flag = 0;
volatile uint8_t transFullDMA_Flag = 0;
volatile uint8_t switch1Cplt_Flag = 0;
volatile uint8_t switch2Cplt_Flag = 0;
volatile uint8_t currentHalfDMA_Flag = 0;
volatile uint8_t currentFullDMA_Flag = 0;
volatile uint8_t currentReadFlag = 0;
volatile uint8_t commandReceived_Flag = 0;

/* Functions -----------------------------------------------------------------*/
void transHalfDMACallback(DAC_HandleTypeDef *hdac)
{
  transHalfDMA_Flag = 1; 
}

void transFullDMACallback(DAC_HandleTypeDef *hdac)
{
  transFullDMA_Flag = 1;
}

void preCycleStartCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) & (transformer.state == Waveform_Request_On))
  {
    HAL_DAC_Start_DMA_NB(transformer.DAC_Handle, transformer.DAC_Channel, (uint32_t*)transformer.LUTLocation, transformer.LUTSize, DAC_ALIGN_12B_R);
    transformer.state = Waveform_On;
  }
}

void s1BurstCpltCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) switch1Cplt_Flag = 1;
}

void s2BurstCpltCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) switch2Cplt_Flag = 1;
}

void currentHalfDMACallback(ADC_HandleTypeDef* hadc) 
{  
  currentHalfDMA_Flag = 1;
}

void currentFullDMACallback(ADC_HandleTypeDef* hadc)
{
  currentFullDMA_Flag = 1;
}

void commandReceivedCallback(UART_HandleTypeDef *huart)
{
  commandReceived_Flag = 1;
}

/****END OF FILE****/