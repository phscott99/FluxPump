/**
  ******************************************************************************
  * @file    dac.c
  * @brief   This file provides code for the configuration
  *          of the DAC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dac.h"

/* USER CODE BEGIN 0 */
#include "arm_math.h"
/* USER CODE END 0 */

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac2;
DMA_HandleTypeDef hdma_dac1_ch1;
DMA_HandleTypeDef hdma_dac1_ch2;
DMA_HandleTypeDef hdma_dac2_ch1;

/* DAC1 init function */
void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = ENABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T3_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}
/* DAC2 init function */
void MX_DAC2_Init(void)
{

  /* USER CODE BEGIN DAC2_Init 0 */

  /* USER CODE END DAC2_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC2_Init 1 */

  /* USER CODE END DAC2_Init 1 */
  /** DAC Initialization
  */
  hdac2.Instance = DAC2;
  if (HAL_DAC_Init(&hdac2) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = ENABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T15_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC2_Init 2 */

  /* USER CODE END DAC2_Init 2 */

}

void HAL_DAC_MspInit(DAC_HandleTypeDef* dacHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(dacHandle->Instance==DAC1)
  {
  /* USER CODE BEGIN DAC1_MspInit 0 */

  /* USER CODE END DAC1_MspInit 0 */
    /* DAC1 clock enable */
    __HAL_RCC_DAC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1
    PA5     ------> DAC1_OUT2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* DAC1 DMA Init */
    /* DAC1_CH1 Init */
    hdma_dac1_ch1.Instance = DMA1_Channel1;
    hdma_dac1_ch1.Init.Request = DMA_REQUEST_DAC1_CHANNEL1;
    hdma_dac1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac1_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dac1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_dac1_ch1.Init.Mode = DMA_CIRCULAR;
    hdma_dac1_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_dac1_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(dacHandle,DMA_Handle1,hdma_dac1_ch1);

    /* DAC1_CH2 Init */
    hdma_dac1_ch2.Instance = DMA1_Channel2;
    hdma_dac1_ch2.Init.Request = DMA_REQUEST_DAC1_CHANNEL2;
    hdma_dac1_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac1_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac1_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac1_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dac1_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_dac1_ch2.Init.Mode = DMA_CIRCULAR;
    hdma_dac1_ch2.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_dac1_ch2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(dacHandle,DMA_Handle2,hdma_dac1_ch2);

  /* USER CODE BEGIN DAC1_MspInit 1 */

  /* USER CODE END DAC1_MspInit 1 */
  }
  else if(dacHandle->Instance==DAC2)
  {
  /* USER CODE BEGIN DAC2_MspInit 0 */

  /* USER CODE END DAC2_MspInit 0 */
    /* DAC2 clock enable */
    __HAL_RCC_DAC2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DAC2 GPIO Configuration
    PA6     ------> DAC2_OUT1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* DAC2 DMA Init */
    /* DAC2_CH1 Init */
    hdma_dac2_ch1.Instance = DMA1_Channel3;
    hdma_dac2_ch1.Init.Request = DMA_REQUEST_DAC2_CHANNEL1;
    hdma_dac2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac2_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dac2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_dac2_ch1.Init.Mode = DMA_CIRCULAR;
    hdma_dac2_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_dac2_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(dacHandle,DMA_Handle1,hdma_dac2_ch1);

  /* USER CODE BEGIN DAC2_MspInit 1 */

  /* USER CODE END DAC2_MspInit 1 */
  }
}

void HAL_DAC_MspDeInit(DAC_HandleTypeDef* dacHandle)
{

  if(dacHandle->Instance==DAC1)
  {
  /* USER CODE BEGIN DAC1_MspDeInit 0 */

  /* USER CODE END DAC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC1_CLK_DISABLE();

    /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1
    PA5     ------> DAC1_OUT2
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5);

    /* DAC1 DMA DeInit */
    HAL_DMA_DeInit(dacHandle->DMA_Handle1);
    HAL_DMA_DeInit(dacHandle->DMA_Handle2);
  /* USER CODE BEGIN DAC1_MspDeInit 1 */

  /* USER CODE END DAC1_MspDeInit 1 */
  }
  else if(dacHandle->Instance==DAC2)
  {
  /* USER CODE BEGIN DAC2_MspDeInit 0 */

  /* USER CODE END DAC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC2_CLK_DISABLE();

    /**DAC2 GPIO Configuration
    PA6     ------> DAC2_OUT1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

    /* DAC2 DMA DeInit */
    HAL_DMA_DeInit(dacHandle->DMA_Handle1);
  /* USER CODE BEGIN DAC2_MspDeInit 1 */

  /* USER CODE END DAC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief  Enables DAC, loads value into DHR12R1 register and starts conversion of channel.
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected (1)
  *
  *         (1) On this STM32 series, parameter not available on all instances.
  *             Refer to device datasheet for channels availability.
  * @param  value What to load into DHR12R1 register to output from DAC
  * @retval HAL status
  */
HAL_StatusTypeDef forceDACOutput(DAC_HandleTypeDef *hdac, uint32_t Channel, float32_t value)
{
  /* Round value to nearest integer */
  value = round(value);

  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(hdac->Instance, Channel));

  /* Process locked */
  __HAL_LOCK(hdac);

  /* Change DAC state */
  hdac->State = HAL_DAC_STATE_BUSY;

  /* Enable the Peripheral */
  __HAL_DAC_ENABLE(hdac, Channel);
  /* Ensure minimum wait before using peripheral after enabling it */
  //HAL_Delay(1);

  /* Store Current configuration */
  uint32_t temp = hdac->Instance->CR;

  if (Channel == DAC_CHANNEL_1)
  {
    MODIFY_REG(hdac->Instance->CR, DAC_CR_TSEL1, DAC_CR_TEN1);
    WRITE_REG(hdac->Instance->DHR12L1, value);
    SET_BIT(hdac->Instance->SWTRIGR, DAC_SWTRIGR_SWTRIG1);
  }
  else
  {
    MODIFY_REG(hdac->Instance->CR, DAC_CR_TSEL2, DAC_CR_TEN2);
    WRITE_REG(hdac->Instance->DHR12L2, value);
    SET_BIT(hdac->Instance->SWTRIGR, DAC_SWTRIGR_SWTRIG2);
  }
  //HAL_Delay(1);

  /* Restore previous configuration */
  WRITE_REG(hdac->Instance->CR, temp);

  /* Change DAC state */
  hdac->State = HAL_DAC_STATE_READY;

  /* Process unlocked */
  __HAL_UNLOCK(hdac);

  /* Return function status */
  return HAL_OK;
}

/* Non-Blocking version of HAL_DAC_Start_DMA for use within interrupts */
HAL_StatusTypeDef HAL_DAC_Start_DMA_NB(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t *pData, uint32_t Length,
                                    uint32_t Alignment)
{
  HAL_StatusTypeDef status;
  uint32_t tmpreg = 0U;

  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(hdac->Instance, Channel));
  assert_param(IS_DAC_ALIGN(Alignment));

  /* Process locked */
  __HAL_LOCK(hdac);

  /* Change DAC state */
  hdac->State = HAL_DAC_STATE_BUSY;

  if (Channel == DAC_CHANNEL_1)
  {
    /* Set the DMA transfer complete callback for channel1 */
    hdac->DMA_Handle1->XferCpltCallback = DAC_DMAConvCpltCh1;

    /* Set the DMA half transfer complete callback for channel1 */
    hdac->DMA_Handle1->XferHalfCpltCallback = DAC_DMAHalfConvCpltCh1;

    /* Set the DMA error callback for channel1 */
    hdac->DMA_Handle1->XferErrorCallback = DAC_DMAErrorCh1;

    /* Enable the selected DAC channel1 DMA request */
    SET_BIT(hdac->Instance->CR, DAC_CR_DMAEN1);

    /* Case of use of channel 1 */
    switch (Alignment)
    {
      case DAC_ALIGN_12B_R:
        /* Get DHR12R1 address */
        tmpreg = (uint32_t)&hdac->Instance->DHR12R1;
        break;
      case DAC_ALIGN_12B_L:
        /* Get DHR12L1 address */
        tmpreg = (uint32_t)&hdac->Instance->DHR12L1;
        break;
      case DAC_ALIGN_8B_R:
        /* Get DHR8R1 address */
        tmpreg = (uint32_t)&hdac->Instance->DHR8R1;
        break;
      default:
        break;
    }
  }
  else
  {
    /* Set the DMA transfer complete callback for channel2 */
    hdac->DMA_Handle2->XferCpltCallback = DAC_DMAConvCpltCh2;

    /* Set the DMA half transfer complete callback for channel2 */
    hdac->DMA_Handle2->XferHalfCpltCallback = DAC_DMAHalfConvCpltCh2;

    /* Set the DMA error callback for channel2 */
    hdac->DMA_Handle2->XferErrorCallback = DAC_DMAErrorCh2;

    /* Enable the selected DAC channel2 DMA request */
    SET_BIT(hdac->Instance->CR, DAC_CR_DMAEN2);

    /* Case of use of channel 2 */
    switch (Alignment)
    {
      case DAC_ALIGN_12B_R:
        /* Get DHR12R2 address */
        tmpreg = (uint32_t)&hdac->Instance->DHR12R2;
        break;
      case DAC_ALIGN_12B_L:
        /* Get DHR12L2 address */
        tmpreg = (uint32_t)&hdac->Instance->DHR12L2;
        break;
      case DAC_ALIGN_8B_R:
        /* Get DHR8R2 address */
        tmpreg = (uint32_t)&hdac->Instance->DHR8R2;
        break;
      default:
        break;
    }
  }

  /* Enable the DMA channel */
  if (Channel == DAC_CHANNEL_1)
  {
    /* Enable the DAC DMA underrun interrupt */
    __HAL_DAC_ENABLE_IT(hdac, DAC_IT_DMAUDR1);

    /* Enable the DMA channel */
    status = HAL_DMA_Start_IT(hdac->DMA_Handle1, (uint32_t)pData, tmpreg, Length);
  }
  else
  {
    /* Enable the DAC DMA underrun interrupt */
    __HAL_DAC_ENABLE_IT(hdac, DAC_IT_DMAUDR2);

    /* Enable the DMA channel */
    status = HAL_DMA_Start_IT(hdac->DMA_Handle2, (uint32_t)pData, tmpreg, Length);
  }

  /* Process Unlocked */
  __HAL_UNLOCK(hdac);

  if (status == HAL_OK)
  {
    /* Enable the Peripheral */
    __HAL_DAC_ENABLE(hdac, Channel);
    /* Ensure minimum wait before using peripheral after enabling it */
    //HAL_Delay(1);
  }
  else
  {
    hdac->ErrorCode |= HAL_DAC_ERROR_DMA;
  }

  /* Return function status */
  return status;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
