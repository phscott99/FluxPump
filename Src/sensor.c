/**
  ******************************************************************************
  * @file    sensor.c
  * @brief   This file provides code for the configuration of
  *          Flux Pump sensors and their clocks, ADCs, Buffers etc.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sensor.h"
#include "adc.h"
#include "callback.h"
#include "defaults.h"
#include "opamp.h"
#include "signal.h"

/* Variables -----------------------------------------------------------------*/
q15_t currentBuffer[TRANSFORMERLUT_MAXSIZE];

Sensor_HandleTypeDef hallCurrent = {
  .type = Current,
  .sensitivity = DEF_HALLSENSITIVITY,
  .BUFLocation = currentBuffer,
  .ADC_Handle = &hadc1,
  .ADC_Channel = ADC_CHANNEL_1,
  .OPAMP_Handle = &hopamp3,
  .halfDMAFlag = &currentHalfDMA_Flag,
  .fullDMAFlag = &currentFullDMA_Flag
};

/* Functions -----------------------------------------------------------------*/
/**
  * @brief  Initialises Sensor Handle type and registers neccessary callback functions
  * @param  sen       Sensor Handle to be initialised
  * @param  callback1 Callback function to be carried out halfway through a Buffer pass
  * @param  callback2 Callback function to be carried out at the end of a Buffer pass
  * @return HAL Status
  */
HAL_StatusTypeDef initSensor(Sensor_HandleTypeDef *sen, void *callback1, void *callback2)
{
  if (sen->ADC_Handle != NULL) // If an ADC is used
  {
    // If ADC is not yet calibrated, run a calibration
    if (READ_REG(sen->ADC_Handle->Instance->CALFACT) == 0) HAL_ADCEx_Calibration_Start(sen->ADC_Handle, ADC_SINGLE_ENDED);
    // Assign callback functions if they are not already assigned
    if (sen->ADC_Handle->ConvHalfCpltCallback == HAL_ADC_ConvHalfCpltCallback) HAL_ADC_RegisterCallback(sen->ADC_Handle, HAL_ADC_CONVERSION_HALF_CB_ID, callback1);
    if (sen->ADC_Handle->ConvCpltCallback == HAL_ADC_ConvCpltCallback) HAL_ADC_RegisterCallback(sen->ADC_Handle, HAL_ADC_CONVERSION_COMPLETE_CB_ID, callback2);
  } 
  // If an OPAMP is used, run a calibration
  if (sen->OPAMP_Handle != NULL) HAL_OPAMP_SelfCalibrate(sen->OPAMP_Handle);
  return HAL_OK;
}

/**
  * @brief  Configures Sensor Buffer size using transformer LUT size
  * @param  sen   Sensor Handle to be configured
  * @param  sig   Signal handle for transformer
  * @retval HAL Status
  */
HAL_StatusTypeDef configSensor(Sensor_HandleTypeDef *sen, Signal_HandleTypeDef *sig)
{
  sen->BUFSize = sig->LUTSize; // For the moment, assume you want to measure as often as the transformer signal updates

  HAL_OPAMP_Start(sen->OPAMP_Handle); // Turn the opamp on
  HAL_ADC_Start_DMA(sen->ADC_Handle, (uint32_t*)sen->BUFLocation, sen->BUFSize); // Start reading signals into buffer with DMA
 
  return HAL_OK;
}

/**
  * @brief  Calibrates sensor by measuring DC offset and setting conversion factor
  * @param  sen   Sensor Handle to be configured
  * @retval HAL Status
  */
HAL_StatusTypeDef calibSensor(Sensor_HandleTypeDef *sen)
{
  /* Determine the resolution and oversampling settings */
  sen->calib.resolution = 0x1 << (12 - (sen->ADC_Handle->Init.Resolution >> 2));
  sen->calib.ratio = 0x2 << (hadc1.Init.Oversampling.Ratio >> 2);
  sen->calib.shift = 0x1 << (hadc1.Init.Oversampling.RightBitShift >> 5);

  /* What do you multiply the ADC reading by to get to a measurement in SI units? */
  sen->conversion = (sen->calib.shift*3.3)/(sen->calib.ratio*sen->calib.resolution*sen->sensitivity);

  /* Before anything happens, take a measurement of the DC offset of the signal */
  while(!*(sen->halfDMAFlag));
  *(sen->halfDMAFlag) = 0;
  arm_mean_q15(sen->BUFLocation, sen->BUFSize/2, &sen->calcs.mean1);
  while(!*(sen->fullDMAFlag));
  *(sen->fullDMAFlag) = 0;
  arm_mean_q15(&sen->BUFLocation[sen->BUFSize/2], sen->BUFSize/2, &sen->calcs.mean2);
  sen->calib.offset = (sen->calcs.mean1 + sen->calcs.mean2)/2; // Weighted average of two half-buffers
 
  return HAL_OK;
}

/****END OF FILE****/