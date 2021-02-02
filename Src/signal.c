/**
  ******************************************************************************
  * @file    signal.c
  * @brief   This file provides code for the configuration of
  *          Flux Pump signals and their clocks, DACs, LUTs etc.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "signal.h"
#include "callback.h"
#include "dac.h"

/* Variables -----------------------------------------------------------------*/
Signal_HandleTypeDef transformer;
Signal_HandleTypeDef switch1;
Signal_HandleTypeDef switch2;

volatile uint8_t fluxpumpRunning; // True if synchronised clocks for DMA are running

/* Functions -----------------------------------------------------------------*/

/**
  * @brief  Initialises Signal Handle type and registers neccessary callback functions for signals
  * @param  type      Defines whether the signal is a transformer or switch type
  * @param  sig       Signal Handle to be initialised
  * @param  callback1 Callback function to be carried out halfway through a LUT pass
  * @param  callback2 Callback function to be carried out at the end of a LUT pass
  * @return HAL Status
  */
HAL_StatusTypeDef initSignal(SignalType type, Signal_HandleTypeDef *sig, void *callback1, void *callback2)
{
  switch(type)
  {
    case FluxPump_Transformer:
      sig->type = FluxPump_Transformer; // Set type in Signal Handle Structure
      HAL_DAC_RegisterCallback(sig->DAC_Handle, HAL_DAC_CH1_HALF_COMPLETE_CB_ID, (pDAC_CallbackTypeDef)callback1); // DAC DMA Half Transfer Complete Flag
      HAL_DAC_RegisterCallback(sig->DAC_Handle, HAL_DAC_CH1_COMPLETE_CB_ID, (pDAC_CallbackTypeDef)callback2); // DAC DMA Transfer Complete Flag
      HAL_TIM_RegisterCallback(sig->gateTIM_Handle, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, preCycleStartCallback); // Callback used for signal start as DMA not running
      break;
    case FluxPump_Switch:
      sig->type = FluxPump_Switch;  // Set type in Signal Handle Structure
      HAL_TIM_RegisterCallback(sig->gateTIM_Handle, HAL_TIM_PWM_PULSE_FINISHED_CB_ID, (pTIM_CallbackTypeDef)callback2); // DAC DMA Transfer Complete Flag
      break;
    default:
      return HAL_ERROR;
      break;
  }
  return HAL_OK;
}

/**
  * @brief  Calculates Signal LUT Size and Configuration for Gate and Trigger Timers
  * @param  type  Defines whether the signal to configure is a transformer or switch type
  * @param  trans Transformer Handle
  * @param  sw1   Switch Handle 1 (configured if type == FluxPump_Switch)
  * @param  sw2   Switch Handle 2
  * @retval HAL Status
  */
HAL_StatusTypeDef configSignal(SignalType type, Signal_HandleTypeDef *trans, Signal_HandleTypeDef *sw1, Signal_HandleTypeDef *sw2)
{
  TIM_OC_InitTypeDef sConfigOC = {0}; // Empty Timer Output Compare configuration structure definition
  switch(type) // Perform different tasks, depending on whether the signal to configure is a transformer or switch
  {
    case FluxPump_Transformer:
      trans->config.divider = ceil(DAC_MAXSAMPLERATE/(trans->profile.frequency*TRANSFORMERLUT_MAXSIZE)); // What prescale from 1MHz is needed to fit one transformer cycle in LUT
      switch(trans->config.divider % 4)
      { // We want transformerDivider * transformerLUTSize to have 4 as a factor
        case 0: // transformerDivider is a multiple of 4, just round transformerLUTSize
          trans->LUTSize = round(DAC_MAXSAMPLERATE/(trans->profile.frequency*trans->config.divider));
          break;
        case 2: // transformerDivider is a multiple of 2, transformerLUTSize must be a multiple of 2
          trans->LUTSize = 2*round(DAC_MAXSAMPLERATE/(trans->profile.frequency*trans->config.divider*2));
          break;
        default:  // transformerDivider is not a multiple of 2 or 4, transformerLUTSize must be a multiple of 4
          trans->LUTSize = 4*round(DAC_MAXSAMPLERATE/(trans->profile.frequency*trans->config.divider*4));
          break;
      }
      trans->trigTIM_Handle->Init.Period = trans->config.divider - 1; // Set Transformer DAC Trigger Timer Period
      if (HAL_TIM_Base_Init(trans->trigTIM_Handle) != HAL_OK) Error_Handler();
      if(!fluxpumpRunning)
      { // If the flux pump isn't running, Timer must be disabled and the counter set to 0 
        CLEAR_BIT(trans->trigTIM_Handle->Instance->CR1, TIM_CR1_CEN);
        CLEAR_REG(trans->trigTIM_Handle->Instance->CNT);
      } 
      
      trans->config.samplesPerCycle = trans->LUTSize * trans->config.divider; // How many samples at 1MSPS for one transformer cycle
      sw1->gateTIM_Handle->Init.Period = trans->config.samplesPerCycle - 1; // Set the period of the first switch gate timer
      if (HAL_TIM_Base_Init(sw1->gateTIM_Handle) != HAL_OK) Error_Handler();
      if(!fluxpumpRunning)
      { // If the flux pump isn't running, Timer must be disabled and the counter set to 0
        CLEAR_BIT(sw1->gateTIM_Handle->Instance->CR1, TIM_CR1_CEN);
        CLEAR_REG(sw1->gateTIM_Handle->Instance->CNT); 
      }
      sw2->gateTIM_Handle->Init.Period = trans->config.samplesPerCycle - 1; // Set the period of the second switch gate timer
      if (HAL_TIM_Base_Init(sw2->gateTIM_Handle) != HAL_OK) Error_Handler();
      if(!fluxpumpRunning)
      { // If the flux pump isn't running, Timer must be disabled and the counter set to 0
        CLEAR_BIT(sw2->gateTIM_Handle->Instance->CR1, TIM_CR1_CEN);
        CLEAR_REG(sw2->gateTIM_Handle->Instance->CNT); 
      }

      /* Configure Output Compare on first switch to act as start trigger for transformer signal when DMA isn't running */
      sConfigOC.OCMode = TIM_OCMODE_TOGGLE; // Useful for debugging
      sConfigOC.Pulse = trans->config.samplesPerCycle - 1; // Trigger just before the start of the cycle
      sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
      if (HAL_TIM_OC_ConfigChannel(trans->gateTIM_Handle, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
      
      trans->profile.frequency = (float32_t)DAC_MAXSAMPLERATE/trans->config.samplesPerCycle; // new, adjusted transformer frequency
      break;
    case FluxPump_Switch:
      sw1->config.numHalfCycles = floor(sw1->profile.width*sw1->profile.frequency*2); // number of half cycles to fit in desired burst length
      sw1->profile.width = sw1->config.numHalfCycles/(2*sw1->profile.frequency);  // new, adjusted burst length
      sw1->LUTSize = 2*floor(sw1->profile.width * DAC_MAXSAMPLERATE/2); // LUT size must be cleanly divisible by 2
      
      sw1->config.startPoint = round((trans->config.samplesPerCycle*sw1->profile.center - sw1->LUTSize/2));// + (trans->config.divider - 2));      // Number of 1MHz samples from start of cycle to start of burst
      
      sConfigOC.OCMode = TIM_OCMODE_COMBINED_PWM2; // OC Channels 1 and 2 of gate clocks are configured in assymetric PWM mode
      sConfigOC.Pulse = sw1->config.startPoint; // Gate signal goes high at start of burst
      sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
      sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
      if (HAL_TIM_PWM_ConfigChannel(sw1->gateTIM_Handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
      sConfigOC.OCMode = TIM_OCMODE_PWM1;
      sConfigOC.Pulse = sw1->config.startPoint + sw1->LUTSize + SWITCH_PADDING; // Gate signal goes low at end of burst
      if (HAL_TIM_PWM_ConfigChannel(sw1->gateTIM_Handle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

      break;
    default:
      return HAL_ERROR;
      break;
  }
  return HAL_OK;
}

/**
  * @brief  Fill LUT with a signal of the chosen waveform, amplitude, shape etc.
  * @param  sig     Signal Handle of Waveform LUT to be calculated
  * @param  section Defines what sections of the LUT to calculate (ping pong buffer)
  * @retval HAL Status
  */
HAL_StatusTypeDef calcSignal(Signal_HandleTypeDef *sig, LUTSection section)
{
  /* Temporary variables for various chosen parameters */
  q15_t* loc = sig->LUTLocation;
  uint16_t size = sig->LUTSize;
  uint16_t numHalfCycles = sig->config.numHalfCycles;
  float32_t freq = sig->profile.frequency;
  float32_t amp = sig->profile.amplitude;

  /* Extra variables for pre-calculation (helps with speed)*/
  float32_t T;
  float32_t a;
  q15_t o;
  uint16_t m;
  uint16_t start = 0, end = 0;

  switch(section)
  { // Depending on which sections are required to be calculated, set the start and end points
    case LUT_All:
      start = 0;
      end = size;
      break;
    case LUT_FirstHalf:
      start = 0;
      end = size/2;
      break;
    case LUT_SecondHalf:
      start = size/2;
      end = size;
      break;
  }

  switch(sig->type)
  { // Perform different calculation approaches depending on whether the signal is for a tranformer or switch
    case FluxPump_Transformer:
      switch(sig->profile.shape)
      { // Chose from a range of different wave shapes (see WaveShape enum for brief description)
        case Constant:
          a = DAC_FULLSCALE/2 + amp*DAC_FULLSCALE/2;
          for(int i = start; i < end; i++) loc[i] = (int)a;
          break;
        case Sine:
          for(int i = start; i < end; i++) loc[i] = lroundf(DAC_FULLSCALE/2 + amp*DAC_FULLSCALE/2*sin(2*M_PI*i/size));
          break;
        case Cosine:
          for(int i = start; i < end; i++) loc[i] = lroundf(DAC_FULLSCALE/2 + amp*DAC_FULLSCALE/2*cos(2*M_PI*i/size));
          break;
        case Triangle: // Continuous equation for calculation of a triangle wave
          for(int i = start; i < end; i++) loc[i] = lroundf(DAC_FULLSCALE/2 + amp*DAC_FULLSCALE/M_PI*asin(sin(2*M_PI*i/size)));
          break;
        case Square:
          a = DAC_FULLSCALE/2 - amp*DAC_FULLSCALE/2;
          T = DAC_FULLSCALE/2 + amp*DAC_FULLSCALE/2;
          if((section == LUT_All) | (section == LUT_FirstHalf))
          {
            for(int i = 0; i < size/4; i++) loc[i] = (int)a;
            for(int i = size/4; i < size/2; i++) loc[i] = (int)T;
          }
          if((section == LUT_All) | (section == LUT_SecondHalf))
          {
            for(int i = size/2; i < (size*3)/4; i++) loc[i] = (int)T;
            for(int i = (size*3)/4; i < size; i++) loc[i] = (int)a;
          }
          break;
        case Sine_Fast:
          T = (float32_t)32768/size;
          a = amp * (DAC_FULLSCALE/2);
          o = DAC_FULLSCALE/2;

          for(int i = start; i < end; i++) loc[i] = arm_sin_q15(i*T);
          arm_scale_q15(loc + start, a, 0, loc, (end-start));
          arm_offset_q15(loc + start, o, loc, (end-start));
          break;
        case Cosine_Fast:
          T = (float32_t)32768/size;
          a = amp * (DAC_FULLSCALE/2);
          o = DAC_FULLSCALE/2;

          for(int i = start; i < end; i++) loc[i] = arm_cos_q15(i*T);
          arm_scale_q15(loc + start, a, 0, loc, (end-start));
          arm_offset_q15(loc + start, o, loc, (end-start));
          break;
        case Triangle_Fast:
          T = (2*DAC_FULLSCALE*amp)/size;
          if((section == LUT_All) | (section == LUT_FirstHalf))
          {
            a = DAC_FULLSCALE/2;
            m =  0;
            for(int i = 0; i < size/4; i++) loc[i] = (int)(a + (i-m)*T);

            a = DAC_FULLSCALE/2 + (T*size)/4;
            m = size/4;
            for(int i = size/4; i < size/2; i++) loc[i] = (int)(a - (i-m)*T);
          }
          if((section == LUT_All) | (section == LUT_SecondHalf))
          {
            a = DAC_FULLSCALE/2 + (T*size)/4;
            m = size/4;
            for(int i = size/2; i < (size*3)/4; i++) loc[i] = (int)(a - (i-m)*T);

            a = DAC_FULLSCALE/2 - (T*size)/4;
            m = (size*3)/4;      
            for(int i = (size*3)/4; i < size; i++) loc[i] = (int)(a + (i-m)*T);
          }
          break;
        default:
          return HAL_ERROR;
          break;
      }
      break;
    case FluxPump_Switch:
      switch(sig->profile.shape)
      {
        case Constant:
          a = DAC_FULLSCALE/2 + amp*DAC_FULLSCALE/2;
          for(int i = 0; i < size; i++) loc[i] = (int)a;
          break;
        case Sine: // Fits the appropriate number of sinusoidal half cycles into the desired burst width
          // If even number of half cycles, use sine
          if (numHalfCycles % 2 == 0) for(int i = -size/2; i < size/2; i++) loc[i + size/2] = lroundf(DAC_FULLSCALE/2 + amp*DAC_FULLSCALE/2*sin(i*2*M_PI*freq/DAC_MAXSAMPLERATE));
          // Else odd number of half cycles, use cosine
          else for(int i = -size/2; i < size/2; i++) loc[i + size/2] = lroundf(DAC_FULLSCALE/2 + amp*DAC_FULLSCALE/2*cos(i*2*M_PI*freq/DAC_MAXSAMPLERATE));
          break;
        case Sine_Fast: // Fits the appropriate number of sinusoidal half cycles into the desired burst width (using fast trig functions)
          T = (32768*freq)/DAC_MAXSAMPLERATE;
          a = amp * (DAC_FULLSCALE/2);
          o = DAC_FULLSCALE/2;
          m = size/2;
          
          // If even number of half cycles, use sine
          if (numHalfCycles % 2 == 0) for(int i = 0; i <= m; i++) {
            loc[m + i] = arm_sin_q15((uint16_t)(i*T) % 32768);
            loc[m - i] = -(arm_sin_q15((uint16_t)(i*T) % 32768)+1);
          }
          // Else odd number of half cycles, use cosine
          else for(int i = 0; i <= m; i++) {
            loc[m + i] = arm_cos_q15((uint16_t)(i*T) % 32768);
            loc[m - i] = arm_cos_q15((uint16_t)(i*T) % 32768);
          }

          arm_scale_q15(loc, a, 0, loc, size);
          arm_offset_q15(loc, o, loc, size);
          break;
        default:
          return HAL_ERROR;
        break;
      }
      
      // Padding values at end of LUT are needed to ensure there is no DC between bursts
      for (int i = 0; i < SWITCH_PADDING; i++) loc[size + i] = lroundf(DAC_FULLSCALE/2);
      break;
    default:
      return HAL_ERROR;
      break;
  }
  return HAL_OK;
}

/****END OF FILE****/