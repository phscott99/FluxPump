/**
  ******************************************************************************
  * File Name          : command.c
  * Description        : This file provides code for parsing commands
  *                      and changing parameters/settings
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "signal.h"
#include "sensor.h"
#include "command.h"

#include "dac.h"
#include "usart.h"

#include "arm_math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Functions -----------------------------------------------------------------*/
/**
  * @brief  Takes a command string and searches for specific commands, then acts upon them
  * @param  command Pointer to the start of a command to be parsed
  * @param  length  Length of command string
  * @return None
  */
void parseCommand(char *command, uint8_t length)
{
  float32_t temp_float;
  uint8_t temp_int;
  if     (command == NULL); // Ignore any commands of zero length
  else if((strncmp(command, "#echo", length) == 0))                                                    printf("#echo echo\r\n");
  else if((strncmp(command, "#T On", length) == 0) & (transformer.state == Waveform_Off))              startWaveform(&transformer);
  else if((strncmp(command, "#T Off", length) == 0) & (transformer.state == Waveform_On))              stopWaveform(&transformer);
  else if((strncmp(command, "#T s", 4) == 0) & (tryParsingInt(&command[4], &temp_int) == HAL_OK))      changeShape(&transformer, temp_int);
  else if((strncmp(command, "#T a", 4) == 0) & (tryParsingFloat(&command[4], &temp_float) == HAL_OK))  changeAmplitude(&transformer, temp_float);
  else if((strncmp(command, "?T div", length) == 0))                                                   HAL_UART_Transmit(&hlpuart1, (void *)&transformer.config.divider, sizeof(transformer.config.divider), 1);
  else if((strncmp(command, "?T spc", length) == 0))                                                   HAL_UART_Transmit(&hlpuart1, (void *)&transformer.config.samplesPerCycle, sizeof(transformer.config.samplesPerCycle), 1);
  else if((strncmp(command, "#S1 On", length) == 0) & (transformer.state == Waveform_On))              startWaveform(&switch1);
  else if((strncmp(command, "#S1 Off", length) == 0) & (transformer.state == Waveform_On))             stopWaveform(&switch1);
  else if((strncmp(command, "#S1 s", 5) == 0) & (tryParsingInt(&command[5], &temp_int) == HAL_OK))     changeShape(&switch1, temp_int);
  else if((strncmp(command, "#S1 a", 5) == 0) & (tryParsingFloat(&command[5], &temp_float) == HAL_OK)) changeAmplitude(&switch1, temp_float);
  else if((strncmp(command, "#S1 f", 5) == 0) & (tryParsingFloat(&command[5], &temp_float) == HAL_OK)) changeFrequency(&switch1, temp_float);
  else if((strncmp(command, "#S1 w", 5) == 0) & (tryParsingFloat(&command[5], &temp_float) == HAL_OK)) changeWidth(&switch1, temp_float);
  else if((strncmp(command, "#S1 c", 5) == 0) & (tryParsingFloat(&command[5], &temp_float) == HAL_OK)) changeCenter(&switch1, temp_float);
  else if((strncmp(command, "#S2 On", length) == 0) & (transformer.state == Waveform_On))              startWaveform(&switch2);
  else if((strncmp(command, "#S2 Off", length) == 0) & (transformer.state == Waveform_On))             stopWaveform(&switch2);
  else if((strncmp(command, "#S2 s", 5) == 0) & (tryParsingInt(&command[5], &temp_int) == HAL_OK))     changeShape(&switch2, temp_int);
  else if((strncmp(command, "#S2 a", 5) == 0) & (tryParsingFloat(&command[5], &temp_float) == HAL_OK)) changeAmplitude(&switch2, temp_float);
  else if((strncmp(command, "#S2 f", 5) == 0) & (tryParsingFloat(&command[5], &temp_float) == HAL_OK)) changeFrequency(&switch2, temp_float);
  else if((strncmp(command, "#S2 w", 5) == 0) & (tryParsingFloat(&command[5], &temp_float) == HAL_OK)) changeWidth(&switch2, temp_float);
  else if((strncmp(command, "#S2 c", 5) == 0) & (tryParsingFloat(&command[5], &temp_float) == HAL_OK)) changeCenter(&switch2, temp_float);
  else if((strncmp(command, "#I read", length) == 0))                                                  hallCurrent.readFlag = 1;
  else if((strncmp(command, "?I offset", length) == 0))                                                HAL_UART_Transmit(&hlpuart1, (void *)&hallCurrent.calib.offset, sizeof(hallCurrent.calib.offset), 1);
  else if((strncmp(command, "?I conv", length) == 0))                                                  HAL_UART_Transmit(&hlpuart1, (void *)&hallCurrent.conversion, sizeof(hallCurrent.conversion),1);
  else if((strncmp(command, "?I pk", length) == 0))                                                    HAL_UART_Transmit(&hlpuart1, (void *)&hallCurrent.stats.peak, sizeof(hallCurrent.stats.peak), 1);
  else if((strncmp(command, "?I rms", length) == 0))                                                   HAL_UART_Transmit(&hlpuart1, (void *)&hallCurrent.stats.rms, sizeof(hallCurrent.stats.rms), 1);
  else if((strncmp(command, "?P duty", length) == 0))                                                  HAL_UART_Transmit(&hlpuart1, (void *)&hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR, sizeof(hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR), 1);
  free(command); // Make sure to free the memory used for the string to stop memory leaks
}

/**
  * @brief  Tries to parse given string as a float variable
  * @param  string pointer to start of float string
  * @param  answer pointer to where to store result
  * @return HAL_OK if successful, else HAL_ERROR if unsuccessful
  */
HAL_StatusTypeDef tryParsingFloat(char *string, float32_t *answer)
{
  char *foo;
  *answer = strtod(string, &foo);
  if (foo == string) return HAL_ERROR;
  else if (foo[strspn(foo, " \t\r\n")] != '\0') return HAL_ERROR;
  return HAL_OK;
}

/**
  * @brief  Tries to parse given string as a uint8_t variable
  * @param  string pointer to start of float string
  * @param  answer pointer to where to store result
  * @return HAL_OK if successful, else HAL_ERROR if unsuccessful
  */
HAL_StatusTypeDef tryParsingInt(char *string, uint8_t *answer)
{
  char *foo;
  *answer = strtol(string, &foo, 10);
  if (foo == string) return HAL_ERROR;
  else if (foo[strspn(foo, " \t\r\n")] != '\0') return HAL_ERROR;
  return HAL_OK;
}

/**
  * @brief  Starts a waveform by the end of the next cycle at the latest
  * @param  sig Signal Handle
  * @return None
  */
HAL_StatusTypeDef startWaveform(Signal_HandleTypeDef *sig)
{
  uint32_t counter;
  switch(sig->type)
  { // Different approaches depending on the type of signal
    case FluxPump_Transformer:
      sig->state = Waveform_Request_On; // Set Waveform On request to be picked up by an interrupt
      break;
    case FluxPump_Switch:
      counter = __HAL_TIM_GET_COUNTER(sig->gateTIM_Handle); // Find the current counter value of the gate timer
      if((counter < sig->config.startPoint) | (counter > (sig->config.startPoint + sig->LUTSize + SWITCH_PADDING)))
      { // If the counter value does not correspond to time when a burst would be occuring, then start the DMA channel
        HAL_DAC_Start_DMA(sig->DAC_Handle, sig->DAC_Channel, (uint32_t*)sig->LUTLocation, sig->LUTSize + SWITCH_PADDING, DAC_ALIGN_12B_L);
        sig->state = Waveform_On;
      }  
      else
      { // Otherwise counter is within burst period and would not be a good time to start waveform
        sig->state = Waveform_Request_On; // Set Waveform On request to start waveform at another time
      }
      break;
    default:
      return HAL_ERROR;
      break;
  }
  return HAL_OK;
}

/**
  * @brief  Stops a waveform by the end of the next cycle at the latest
  * @param  sig Signal Handle
  * @return None
  */
HAL_StatusTypeDef stopWaveform(Signal_HandleTypeDef *sig)
{
  HAL_StatusTypeDef status;
  uint32_t counter;
  switch(sig->type)
  { // Different approaches depending on the type of signal
    case FluxPump_Transformer:
      counter = __HAL_DMA_GET_COUNTER(sig->DAC_Handle->DMA_Handle1); // Find the current counter value of the gate timer
      if((counter == 0) | (counter == sig->LUTSize/2))
      { // This is a good time to stop the waveform
        break; // Break out of the switch statement to stop waveform
      }
      else
      { // This isnt a good time to stop waveform as it would cause discontinuity
        sig->state = Waveform_Request_Off; // Set Waveform Off request to stop waveform at another time
        return HAL_OK;
      }
      break;
    case FluxPump_Switch:
      counter = __HAL_TIM_GET_COUNTER(sig->gateTIM_Handle); // Find the current value of the gate timer
      if((counter < sig->config.startPoint) | (counter > (sig->config.startPoint + sig->LUTSize + SWITCH_PADDING)))
      { // If the counter value does not correspond to time when a burst would be occuring
        break; // Break out of the switch statement to stop waveform
      }
      else
      { // Otherwise counter is within burst period and would not be a good time to stop waveform
        sig->state = Waveform_Request_Off; // Set Waveform Off request to stop waveform at another time
        return HAL_OK;
      }
      break;
    default:
      return HAL_ERROR;
      break;
  }

  //** IT IS SAFE TO STOP THE WAVEFORM **//
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(sig->DAC_Handle->Instance, sig->DAC_Channel));

  /* Disable the selected DAC channel DMA request */
  sig->DAC_Handle->Instance->CR &= ~(DAC_CR_DMAEN1 << (sig->DAC_Channel & 0x10UL));

  /* Disable the DMA channel */

  /* Channel1 is used */
  if (sig->DAC_Channel == DAC_CHANNEL_1)
  {
    /* Disable the DMA channel */
    status = HAL_DMA_Abort(sig->DAC_Handle->DMA_Handle1);

    /* Disable the DAC DMA underrun interrupt */
    __HAL_DAC_DISABLE_IT(sig->DAC_Handle, DAC_IT_DMAUDR1);
  }
  else /* Channel2 is used for */
  {
    /* Disable the DMA channel */
    status = HAL_DMA_Abort(sig->DAC_Handle->DMA_Handle2);

    /* Disable the DAC DMA underrun interrupt */
    __HAL_DAC_DISABLE_IT(sig->DAC_Handle, DAC_IT_DMAUDR2);
  }

  /* Check if DMA Channel effectively disabled */
  if (status != HAL_OK)
  {
    /* Update DAC state machine to error */
    sig->DAC_Handle->State = HAL_DAC_STATE_ERROR;
  }
  else
  {
    /* Change DAC state */
    sig->DAC_Handle->State = HAL_DAC_STATE_READY;
  }

  /* Force Output to 0 to prevent any DC offset */
  forceDACOutput(sig->DAC_Handle, sig->DAC_Channel, 0);

  sig->state = Waveform_Off;

  /* Return function status */
  return status;
}

/**
  * @brief  Changes the shape of the waveform and forces recalculation by the end of the next cycle at the latest
  * @param  sig Signal Handle
  * @param  shape New waveform shape for signal
  * @return None
  */
HAL_StatusTypeDef changeShape(Signal_HandleTypeDef *sig, uint8_t shape)
{
  if(shape < 0) return HAL_ERROR; // Not a valid shape enum
  else if(shape > Sine_Slow) return HAL_ERROR; // Not a valid shape enum
  
  sig->profile.shape = shape;
  sig->updateRequest = Update_FirstHalfLUT; // Change of shape requires a LUT recalculation
  
  return HAL_OK;
}

/**
  * @brief  Changes the amplitude of the waveform and forces recalculation by the end of the next cycle at the latest
  * @param  sig Signal Handle
  * @param  amp New amplitude for signal
  * @return None
  */
HAL_StatusTypeDef changeAmplitude(Signal_HandleTypeDef *sig, float32_t amp)
{
  /* Constrain amplitude to +- maximum amplitude */
  float32_t max = sig->maxAmplitude;
  if(amp > max) amp = max;
  else if(amp < -max) amp = -max;
  
  sig->profile.amplitude = amp;
  sig->updateRequest = Update_FirstHalfLUT; // Change of amplitude requires a LUT recalculation
  
  return HAL_OK;
}

/**
  * @brief  Changes the frequency of the waveform and forces recalculation by the end of the next cycle at the latest
  * @param  sig Signal Handle
  * @param  freq New frequency for signal
  * @return None
  */
HAL_StatusTypeDef changeFrequency(Signal_HandleTypeDef *sig, float32_t freq)
{
  if(freq < 0) return HAL_ERROR; // Please let me know if you find proof of negative frequencies
  
  sig->profile.frequency = freq;
  sig->updateRequest = Update_Clocks; // Changing the frequency will require a reconfiguration of the clocks
  
  return HAL_OK;
}

/**
  * @brief  Changes the width of the burst and forces recalculation by the end of the next cycle at the latest
  * @param  sig Signal Handle
  * @param  width New burst width for signal
  * @return None
  */
HAL_StatusTypeDef changeWidth(Signal_HandleTypeDef *sig, float32_t width)
{
  if(width < 0) return HAL_ERROR; // I know I don't understand flux pumps but a negative width sounds like a bad idea
  
  sig->profile.width = width;
  sig->updateRequest = Update_Clocks; // Changing burst width will require a change in the clock configuration
  
  return HAL_OK;
 }

 /**
  * @brief  Changes the center of the burst and forces recalculation by the end of the next cycle at the latest
  * @param  sig Signal Handle
  * @param  center New center for burst
  * @return None
  */
HAL_StatusTypeDef changeCenter(Signal_HandleTypeDef *sig, float32_t center)
{
  /* Burst must be within the bounds of the cycle, this does not catch all errors */
  if(center < 0) return HAL_ERROR;
  else if(center > 1) return HAL_ERROR;
  
  sig->profile.center = center;
  sig->updateRequest = Update_Clocks; // Changing the center of the burst will require clock recalculation
  
  return HAL_OK;
 }

/****END OF FILE****/