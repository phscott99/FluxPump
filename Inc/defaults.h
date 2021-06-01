/**
  ******************************************************************************
  * @file    defaults.h
  * @brief   This file contains default parameters for the Flux Pump
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEFAULTS_H
#define __DEFAULTS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Defines -------------------------------------------------------------------*/

#define TRANSFORMERLUT_MAXSIZE 4000
#define SWITCHLUT_MAXSIZE 10000
#define SWITCH_PADDING 2

/* Default Settings for Transformer Signal */
#define DEF_TRANSFREQUENCY 10
#define DEF_TRANSAMPLITUDE 6
#define DEF_TRANSSHAPE Triangle
#define DEF_TRANSMAXAMPLITUDE (3.3/(2*0.11))

/* Default Settings for Switch 1 Signal */
#define DEF_SW1FREQUENCY 1000
#define DEF_SW1AMPLITUDE 0.9
#define DEF_SW1WIDTH 0.012
#define DEF_SW1CENTER 0.65
#define DEF_SW1SHAPE Sine
#define DEF_SW1MAXAMPLITUDE 1.0

/* Default Settings for Switch 2 Signal */
#define DEF_SW2FREQUENCY 500
#define DEF_SW2AMPLITUDE 0.6
#define DEF_SW2WIDTH 0.005
#define DEF_SW2CENTER 0.15
#define DEF_SW2SHAPE Sine
#define DEF_SW2MAXAMPLITUDE 1.0

/* Default Settings for ACS 711 Hall Current Sensor */
#define DEF_HALLSENSITIVITY 0.11 // 110mV/A

/* Default Settings for PID Controller */
#define KP_DEFAULT 0
#define KI_DEFAULT 0
#define KD_DEFAULT 0

#ifdef __cplusplus
}
#endif

#endif /*__ DEFAULTS_H */

/****END OF FILE****/