/**
  ******************************************************************************
  * @file    dac.h
  * @brief   This file contains all the function prototypes for
  *          the dac.c file
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
#ifndef __DAC_H__
#define __DAC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac2;

/* USER CODE BEGIN Private defines */
#define DAC_MAXSAMPLERATE 1000000.0 // Maximum External DAC Sample Rate of 1MS/s
#define DAC_FULLSCALE 4095.0 // 12 Bit DAC
/* USER CODE END Private defines */

void MX_DAC1_Init(void);
void MX_DAC2_Init(void);

/* USER CODE BEGIN Prototypes */
HAL_StatusTypeDef forceDACOutput(DAC_HandleTypeDef *hdac, uint32_t Channel, float32_t value); // Based on HAL_DAC_Start and doesn't use HAL_Delay
HAL_StatusTypeDef HAL_DAC_Start_DMA_NB(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t *pData, uint32_t Length, uint32_t Alignment); // Based on HAL_DAC_Start_DMA and doesn't use HAL_Delay
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DAC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
