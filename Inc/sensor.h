/**
  ******************************************************************************
  * @file    sensor.h
  * @brief   This file contains all the function prototypes and
  *          structures for the sensor.c file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSOR_H__
#define __SENSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"
#include "signal.h"

/* typedefs ------------------------------------------------------------------*/
/**
 * @brief Flux Pump Sensor Type
 */
typedef enum{
  Current,    // Expected to be a constant waveform
  Temperature // Expected to be a waveform burst
} SensorType;

/**
  * @brief  Flux Pump Sensor Calibration Data
  */
typedef struct
{
  q15_t resolution; // Fullscale output from ADC measurement ie. 2^b
  q15_t ratio;      // Oversampling ratio
  q15_t shift;      // Effective division due to right bit shift of ADC data
  q15_t offset;     // Measurement of DC offset for AC signals
} Calib_TypeDef;

/**
  * @brief  Flux Pump Sensor Interim Calculation Variables
  */
typedef struct
{
  q15_t   mean1, mean2; // Mean measurements of first and second half of buffer
  q15_t   rms1, rms2;   // RMS measurements of first and second half of buffer
  q15_t   min1, min2;   // Minimum values of first and second half of buffer
  q15_t   max1, max2;   // Maximum values of first and second half of buffer
} Calcs_TypeDef;

/**
  * @brief  Flux Pump Sensor Statistics
  */
typedef struct
{
  float32_t   rms;    // RMS of sensor measurement over one transformer cycle
  float32_t   peak;   // Half of the range of sensor measurement over one transformer cycle
} Stats_TypeDef;

/**
  * @brief  Flux Pump Sensor Handle Structure definition
  */
typedef struct
{
  SensorType            type;         // Whether the handle is for a transformer or switch signal
  float32_t             sensitivity;  // Scale factor between voltage and SI unit (eg volts per amp)
  float32_t             conversion;   // What an ADC reading should be multiplied by for an SI result
  q15_t                *BUFLocation;  // Pointer to sensor measurement buffer location
  uint16_t              BUFSize;      // Number of sample in the measurement buffer
  Calib_TypeDef         calib;        // Calibration data for ADC setup
  Calcs_TypeDef         calcs;        // Interim calculation variables 
  Stats_TypeDef         stats;        // Calculated statistics of sensor measurements
  ADC_HandleTypeDef    *ADC_Handle;   // Pointer to ADC peripheral responsible for sensor input
  uint32_t              ADC_Channel;  // Which channel is responsible for snesor input
  OPAMP_HandleTypeDef  *OPAMP_Handle; // Pointer to OPAMP peripheral used by the sensor input
  volatile uint8_t     *halfDMAFlag;  // Pointer to half DMA flag
  volatile uint8_t     *fullDMAFlag;  // Pointer to full DMA flag
  volatile uint8_t      readFlag;     // Indicates whether the next full buffer should be sent over UART
} Sensor_HandleTypeDef;

/* Flux Pump Signal handles --------------------------------------------------*/
extern Sensor_HandleTypeDef hallCurrent;

/* Function Prototypes -------------------------------------------------------*/
HAL_StatusTypeDef initSensor(Sensor_HandleTypeDef *sen, void *callback1, void *callback2);
HAL_StatusTypeDef configSensor(Sensor_HandleTypeDef *sen, Signal_HandleTypeDef *sig);
HAL_StatusTypeDef calibSensor(Sensor_HandleTypeDef *sen);

#ifdef __cplusplus
}
#endif

#endif /* __SIGNAL_H__ */

/****END OF FILE****/