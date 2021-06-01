/**
  ******************************************************************************
  * @file    command.h
  * @brief   This file contains all the function prototypes for
  *          the command.c file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMAND_H
#define __COMMAND_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Defines -------------------------------------------------------------------*/
#define CMD_STR_MAXLEN 20 // Maximum string length of a single CLI command
extern HRTIM_HandleTypeDef hhrtim1; // This is a bodge so the current PWM duty cycle can be queried

/* Function Prototypes -------------------------------------------------------*/
void parseCommand(char *commandString, uint8_t length);
HAL_StatusTypeDef tryParsingFloat(char *string, float32_t *answer);
HAL_StatusTypeDef tryParsingInt(char *string, uint8_t *answer);
HAL_StatusTypeDef startWaveform(Signal_HandleTypeDef *sig);
HAL_StatusTypeDef stopWaveform(Signal_HandleTypeDef *sig);
HAL_StatusTypeDef changeShape(Signal_HandleTypeDef *sig, uint8_t shape);
HAL_StatusTypeDef changeAmplitude(Signal_HandleTypeDef *sig, float32_t amp);
HAL_StatusTypeDef changeFrequency(Signal_HandleTypeDef *sig, float32_t freq);
HAL_StatusTypeDef changeWidth(Signal_HandleTypeDef *sig, float32_t width);
HAL_StatusTypeDef changeCenter(Signal_HandleTypeDef *sig, float32_t center);

#ifdef __cplusplus
}
#endif

#endif /*__ COMMAND_H */

/****END OF FILE****/