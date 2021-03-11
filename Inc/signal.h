/**
  ******************************************************************************
  * @file    signal.h
  * @brief   This file contains all the function prototypes and
  *          structures for the signal.c file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SIGNAL_H__
#define __SIGNAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"

/* typedefs ------------------------------------------------------------------*/
/**
 * @brief Flux Pump Signal Type
 */
typedef enum{
  FluxPump_Transformer, // Expected to be a constant waveform
  FluxPump_Switch // Expected to be a waveform burst
} SignalType;

/**
 * @brief LUT Waveform Options
 */
typedef enum
{
  Constant,     // Sets all of LUT to same value
  Sine,         // Uses standard (slow) sin function to calculate sine
  Cosine,       // Uses standard (slow) cos function to calculate cosine
  Triangle,     // Uses a continuous method using the standard (slow) trig functions
  Square,       // Uses a discrete method to produce a square wave
  Sine_Fast,    // Uses CMSIS DSP arm_sin_q15 function to calculate sine
  Cosine_Fast,  // Uses CMSIS DSP arm_cos_q15 function to calculate cosine
  Triangle_Fast // Uses a discrete method to produce a triangle wave
} WaveShape;

/**
 * @brief LUT Sections for Ping Pong Buffer functions
 */
typedef enum
{
  LUT_All,
  LUT_FirstHalf,
  LUT_SecondHalf
} LUTSection;

/**
 * @brief Waveform States
 */
typedef enum
{
  Waveform_Off,         // No signal being produced
  Waveform_Request_Off, // Signal should be stopped before next cycle
  Waveform_On,          // Signal being produced
  Waveform_Request_On   // Signal should start in next cycle
} WaveformState;

/**
  * @brief  Flux Pump Waveform Profile Structure definition
  */
typedef struct
{
  float32_t   frequency;  // Frequency of signal (Hz)
  float32_t   amplitude;  // Amplitude of signal (currently defined as a fraction of the DAC fullscale output)
  WaveShape   shape;      // Wave Shape of the signal (enum)
  float32_t   width;      // Width of burst (s)
  float32_t   center;     // Fraction of cycle length from start of transformer cycle to center of burst (s)
} Profile_TypeDef;

/**
  * @brief  Flux Pump Transformer Timing Configuration Structure definition
  */
typedef struct
{
  uint16_t divider;         // How many samples equals one transformer lookup table value
  uint32_t samplesPerCycle; // How many samples equals a whole transformer cycle
  uint16_t numHalfCycles;   // Number of half cycles to closest match the burst width required
  uint32_t startPoint;      // Number of samples into a transformer cycle to start the burst
} Config_TypeDef;

/**
  * @brief  Flux Pump Switch Handle Structure definition
  */
typedef struct
{
  SignalType              type;           // Whether the handle is for a transformer or switch signal
  Profile_TypeDef         profile;        // Current Burst Profile parameters
  Config_TypeDef          config;         // Timer configuration
  q15_t                  *LUTLocation;    // Pointer to switch lookup table location
  uint16_t                LUTSize;        // Number of samples in the lookup table
  TIM_HandleTypeDef      *gateTIM_Handle; // Pointer to timer responsible for gate signal
  TIM_HandleTypeDef      *trigTIM_Handle; // Pointer to timer responsible for trigger signal
  DAC_HandleTypeDef      *DAC_Handle;     // Pointer to DAC peripheral responsible for switch signal
  uint32_t                DAC_Channel;    // Which channel is responsible for switch signal output
  volatile WaveformState  state;          // Waveform State (running or not running)
  volatile uint8_t        pendingRecalc;  // True when profile has changed and LUT/TIMs need updating
} Signal_HandleTypeDef;

/* Flux Pump Signal handles --------------------------------------------------*/
extern Signal_HandleTypeDef transformer;
extern Signal_HandleTypeDef switch1;
extern Signal_HandleTypeDef switch2;

/* Defines -------------------------------------------------------------------*/
#define TRANSFORMERLUT_MAXSIZE 8000
#define SWITCH1LUT_MAXSIZE 2000
#define SWITCH2LUT_MAXSIZE 2000
#define SWITCH_PADDING 2

/* Function Prototypes -------------------------------------------------------*/
HAL_StatusTypeDef initSignal(SignalType type, Signal_HandleTypeDef *sig, void *callback1, void *callback2);
HAL_StatusTypeDef configSignal(SignalType type, Signal_HandleTypeDef *trans, Signal_HandleTypeDef *sw1, Signal_HandleTypeDef *sw2);
HAL_StatusTypeDef calcSignal(Signal_HandleTypeDef *sig, LUTSection section);

#ifdef __cplusplus
}
#endif

#endif /* __SIGNAL_H__ */

/****END OF FILE****/