/**
  ******************************************************************************
  * @file    PID.h
  * @brief   This file contains all the function prototypes for
  *          the PID.c file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"

/* Variables -----------------------------------------------------------------*/
extern arm_pid_instance_q15 PID;

/* Function Prototypes -------------------------------------------------------*/
extern void PID_Init(q15_t Kp, q15_t Ki, q15_t Kd);

#ifdef __cplusplus
}
#endif

#endif /*__ PID_H */

/****END OF FILE****/