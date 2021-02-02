/**
  ******************************************************************************
  * File Name          : PID.c
  * Description        : This file provides code for the configuration
  *                      and running of the PID control Loop.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "arm_math.h"

/* Variables -----------------------------------------------------------------*/
arm_pid_instance_q15 PID;

/* Functions -----------------------------------------------------------------*/

void PID_Init(q15_t Kp, q15_t Ki, q15_t Kd)
{
  PID.Kp = Kp;
  PID.Ki = Ki;
  PID.Kd = Kd;

  arm_pid_init_q15(&PID, 1);
}

/****END OF FILE****/