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
volatile uint8_t transHalfCplt_Flag = 0;
volatile uint8_t transFullCplt_Flag = 0;
volatile uint8_t switch1Cplt_Flag = 0;
volatile uint8_t switch2Cplt_Flag = 0;
volatile uint8_t commandReceived_Flag = 0;

/* Functions -----------------------------------------------------------------*/
void transHalfCpltCallback(DAC_HandleTypeDef *hdac)
{
  transHalfCplt_Flag = 1; 
}

void transFullCpltCallback(DAC_HandleTypeDef *hdac)
{
  transFullCplt_Flag = 1;
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

void commandReceivedCallback(UART_HandleTypeDef *huart)
{
  commandReceived_Flag = 1;
}

/****END OF FILE****/