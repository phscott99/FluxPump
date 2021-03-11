/**
  ******************************************************************************
  * File Name          : command.c
  * Description        : This file provides code for parsing commands
  *                      and changing parameters/settings
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "signal.h"
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
  if     (command == NULL); // Ignore any commands of zero length
  else if((strncmp(command, "#echo", length) == 0)) printf("#echo echo\r\n");
  else if((strncmp(command, "#T On", length) == 0) & (transformer.state == Waveform_Off))   startWaveform(&transformer);
  else if((strncmp(command, "#T Off", length) == 0) & (transformer.state == Waveform_On))   stopWaveform(&transformer);
  else if((strncmp(command, "?T conf", length) == 0)) printf("%d, %d, %ld, %ld \r\n", transformer.config.divider, transformer.config.numHalfCycles, transformer.config.samplesPerCycle, transformer.config.startPoint);
  else if((strncmp(command, "#S1 On", length) == 0) & (transformer.state == Waveform_On))   startWaveform(&switch1);
  else if((strncmp(command, "#S1 Off", length) == 0) & (transformer.state == Waveform_On))  stopWaveform(&switch1);
  else if((strncmp(command, "#S2 On", length) == 0) & (transformer.state == Waveform_On))   startWaveform(&switch2);
  else if((strncmp(command, "#S2 Off", length) == 0) & (transformer.state == Waveform_On))  stopWaveform(&switch2);
  else if((strncmp(command, "#I read", length) == 0)) currentReadFlag = 1;
  else if((strncmp(command, "?I zero", length) == 0)) printf("%d \r\n", average);

  free(command); // Make sure to free the memory used for the string to stop memory leaks
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
        HAL_DAC_Start_DMA(sig->DAC_Handle, sig->DAC_Channel, (uint32_t*)sig->LUTLocation, sig->LUTSize + SWITCH_PADDING, DAC_ALIGN_12B_R);
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

  /* Force Output to Half of Fullscale to prevent any DC offset */
  forceDACOutput(sig->DAC_Handle, sig->DAC_Channel, DAC_FULLSCALE/2);

  sig->state = Waveform_Off;

  /* Return function status */
  return status;
}

/****END OF FILE****/