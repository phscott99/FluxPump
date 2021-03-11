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

extern volatile uint8_t currentReadFlag;
extern q15_t average;

/* Function Prototypes -------------------------------------------------------*/
void parseCommand(char *commandString, uint8_t length);
HAL_StatusTypeDef startWaveform(Signal_HandleTypeDef *sig);
HAL_StatusTypeDef stopWaveform(Signal_HandleTypeDef *sig);

#ifdef __cplusplus
}
#endif

#endif /*__ COMMAND_H */

/****END OF FILE****/