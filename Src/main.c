/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "hrtim.h"
#include "hrtim.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Custom Libraries for Flux Pump*/
#include "signal.h"
#include "callback.h"
#include "command.h"
#include "pid.h"

/* 3rd Party and Standard Libraries */
#include "arm_math.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARM_MATH_CM4 // Declare Cortex-M4 Architecture for ARM CMSIS Library
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Allocate SRAM for Waveform LUTs */
q15_t transformerLUT[TRANSFORMERLUT_MAXSIZE];
q15_t switch1LUT[SWITCH1LUT_MAXSIZE + SWITCH_PADDING];
q15_t switch2LUT[SWITCH2LUT_MAXSIZE + SWITCH_PADDING];

int32_t duty = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void callback_function(HRTIM_HandleTypeDef *hhrtim, uint32_t TimerIdx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* Default Settings for Transformer Signal */
  transformer.LUTLocation = transformerLUT;
  transformer.gateTIM_Handle = &htim2; // used for starting DMA waveform
  transformer.trigTIM_Handle = &htim15;
  transformer.DAC_Handle = &hdac2;
  transformer.DAC_Channel = DAC_CHANNEL_1;
  transformer.profile.frequency = 10;
  transformer.profile.amplitude = 0.9;
  transformer.profile.shape = Triangle_Fast;

  /* Default Settings for Switch 1 Signal */
  switch1.LUTLocation = switch1LUT;
  switch1.gateTIM_Handle = &htim2;
  switch1.trigTIM_Handle = &htim3;
  switch1.DAC_Handle = &hdac1;
  switch1.DAC_Channel = DAC_CHANNEL_1;
  switch1.profile.frequency = 1000;
  switch1.profile.amplitude = 0.9;
  switch1.profile.width = 0.02;
  switch1.profile.center = 0.25;
  switch1.profile.shape = Sine_Fast;

  /* Default Settings for Switch 2 Signal */
  switch2.LUTLocation = switch2LUT;
  switch2.gateTIM_Handle = &htim5;
  switch2.trigTIM_Handle = &htim4;
  switch2.DAC_Handle = &hdac1;
  switch2.DAC_Channel = DAC_CHANNEL_2;
  switch2.profile.frequency = 2000;
  switch2.profile.amplitude = 0.9;
  switch2.profile.width = 0.00125;
  switch2.profile.center = 0.75;
  switch2.profile.shape = Sine_Fast;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM15_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_DAC2_Init();
  MX_ADC1_Init();
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_HRTIM1_Init();
  /* USER CODE BEGIN 2 */
  resetTimers(); // Disables all the timers until we're ready for them

  /* Set all DAC outputs to half of fullscale so amplifiers can be turned on */
  forceDACOutput(&hdac1, DAC_CHANNEL_1, DAC_FULLSCALE/2);
  forceDACOutput(&hdac1, DAC_CHANNEL_2, DAC_FULLSCALE/2);
  forceDACOutput(&hdac2, DAC_CHANNEL_1, DAC_FULLSCALE/2);

  configureUART();
  UARTsprintf("\r\nSTART");

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // Start calibration of ADC channels

  /* Initialise Signals */
  initSignal(FluxPump_Transformer, &transformer, transHalfCpltCallback, transFullCpltCallback);
  initSignal(FluxPump_Switch, &switch1, NULL, s1BurstCpltCallback);
  initSignal(FluxPump_Switch, &switch2, NULL, s2BurstCpltCallback);

  /* Configure Signals */
  configSignal(FluxPump_Transformer, &transformer, &switch1, &switch2);
  configSignal(FluxPump_Switch, &transformer, &switch1, NULL);
  configSignal(FluxPump_Switch, &transformer, &switch2, NULL);
  
  /* Calculate Waveform LUTs */
  calcSignal(&transformer, LUT_All);
  calcSignal(&switch1, LUT_All);
  calcSignal(&switch2, LUT_All);

  updateTimers(); // Send update command to load preloaded values into registers
  startTimers();  // Start all timers required for flux pump signals
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);

  /* Start all Flux Pump Signals */
  startWaveform(&transformer);
  startWaveform(&switch1);
  startWaveform(&switch2);

  HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A);
  HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B);
  HAL_HRTIM_SimpleBaseStart_IT(&hhrtim1, HRTIM_TIMERINDEX_MASTER);
  HAL_HRTIM_TIMxRegisterCallback(&hhrtim1, HAL_HRTIM_REPETITIONEVENTCALLBACK_CB_ID, callback_function);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(transHalfCplt_Flag)
    { // DMA has passed the midpoint of the Transformer LUT
      transHalfCplt_Flag = 0; // Clear Flag
      if(transformer.pendingRecalc) calcSignal(&transformer, LUT_FirstHalf); // Recalculate first half of transformer LUT if requested
      if(transformer.state == Waveform_Request_Off) stopWaveform(&transformer); // Stop the transformer waveform if requested
    }
    if(transFullCplt_Flag)
    { // DMA has passed the end of the Transformer LUT and has wrapped around to the start
      transFullCplt_Flag = 0;
      if(transformer.pendingRecalc) calcSignal(&transformer, LUT_SecondHalf); // Recalculate second half of transformer LUT if requested
      if(transformer.state == Waveform_Request_Off) stopWaveform(&transformer); // Stop the transformer waveform if requested
    }
    if(switch1Cplt_Flag)
    { // DMA has finished switch 1 burst
      switch1Cplt_Flag = 0; // Clear Flag
      if(switch1.pendingRecalc) calcSignal(&switch1, LUT_All); // Recalculate Switch 1 LUT if requested
      if(switch1.state == Waveform_Request_On)  startWaveform(&switch1); // Start switch 1 waveform if requested
      if(switch1.state == Waveform_Request_Off) stopWaveform(&switch1); // Stop switch 1 waveform if requested
    }
    if(switch2Cplt_Flag)
    { // DMA has finished switch 2 burst
      switch2Cplt_Flag = 0; // Clear Flag
      if(switch2.pendingRecalc) calcSignal(&switch2, LUT_All); // Recalculate Switch 1 LUT if requested
      if(switch2.state == Waveform_Request_On)  startWaveform(&switch2); // Start switch 2 waveform if requested
      if(switch2.state == Waveform_Request_Off) stopWaveform(&switch2); // Stop switch 2 waveform if requested
    }
    if(commandReceived_Flag)
    { // LPUART1 has recieved new data
      commandReceived_Flag = 0; // Clear Flag
      static size_t old_pos; // Point in circular buffer where the new data starts
      size_t pos;

      static size_t commandStart = 0;
      static size_t commandEnd = 0;

      pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_lpuart1_rx); // Calculate where the new data ends
      while(old_pos != pos)
      { // This loop will run for every new character in turn, until the end of new data has been reached
        if(((LPUART1_rxBuffer[old_pos] == '#') | (LPUART1_rxBuffer[old_pos] == '\r')) & (old_pos != commandStart)) 
        { // This code runs if the special command character (#) or a return character is found
          commandStart = commandEnd + 1; // The new command starts immediately after the old command end
          commandEnd = old_pos; // The new command ends at the point determined by old_pos
          if(commandEnd>commandStart)
          { // Then the whole command is in a continuous section of the RX buffer
            uint8_t size = commandEnd - commandStart + 1; // Size of buffer needed to store the command
            char *string = (char*)calloc(size, sizeof(char)); // Allocate SRAM for command buffer
            strncat(string, &LPUART1_rxBuffer[commandStart], commandEnd - commandStart); // Copy command from RX buffer to command buffer
            parseCommand(string, size); // Send this command to be parsed
          }
          else
          { // The command string wraps around to the start of the circular RX buffer
            uint8_t size = RX_BUFFER_SIZE - commandStart + commandEnd + 1; // Size of buffer needed to store the command
            char *string = (char*)calloc(size, sizeof(char)); // Allocate SRAM for command buffer
            strncat(string, &LPUART1_rxBuffer[commandStart], RX_BUFFER_SIZE - commandStart); // Copy first section of command from RX buffer to command buffer
            strncat(string, &LPUART1_rxBuffer[0], commandEnd); // Copy second section of command from RX buffer to command buffer
            parseCommand(string, size); // Send this command to be parsed
          }
        }
        old_pos = (old_pos + 1) % RX_BUFFER_SIZE; // Move to next character in the new data and wrap around to the beginning if necessary
      }   
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 81;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void callback_function(HRTIM_HandleTypeDef * hhrtim, uint32_t TimerIdx)
{
  duty = (duty + 1);
  if (duty >= 0xFD20) duty = duty - (2*0xFD20 - 1);

  if((duty > -HRTIM_CMP_MIN) & (duty < HRTIM_CMP_MIN)) hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = 0;
  else hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = abs(duty);
  
  if (duty < 0) hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = 0x0000;
  else hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = HRTIM_CMP_MAX;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
