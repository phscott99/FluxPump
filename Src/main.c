/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * This software was developed by Peter Scott (PHScott99@outlook.com) for their
  * fourth year project at Cambridge University Engineering Department:
  *     B-TAC1000-2 - An Embedded System for Flux Pump Control 
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
#include "opamp.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Custom Libraries for Flux Pump*/
#include "signal.h"
#include "sensor.h"
#include "callback.h"
#include "command.h"
#include "pid.h"

/* 3rd Party and Standard Libraries */
#include "arm_math.h"
#include "retarget.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void callback_function(TIM_HandleTypeDef *htim); // Temporary function for prototyping PID control
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4); // Use this for debugging!
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_OPAMP3_Init();
  /* USER CODE BEGIN 2 */
  Retarget_Init(&hlpuart1); // Printf retargeting for LPUART1
  PID_Init(KP_DEFAULT, KI_DEFAULT, KD_DEFAULT); // Initialises the PID instance with default gains
  resetTimers(); // Disables all the timers until we're ready for them

  /* Set all DAC outputs to half of fullscale */
  forceDACOutput(&hdac1, DAC_CHANNEL_1, 0);
  forceDACOutput(&hdac1, DAC_CHANNEL_2, 0);
  forceDACOutput(&hdac2, DAC_CHANNEL_1, 0);

  configureUART(); // Configure character match interrupts to recognise incoming commands
  printf("\r\n START \r\n"); // Sanity check

  /* Initialise Signals */
  initSignal(FluxPump_Transformer, &transformer, transHalfDMACallback, transFullDMACallback);
  initSignal(FluxPump_Switch, &switch1, NULL, s1BurstCpltCallback);
  initSignal(FluxPump_Switch, &switch2, NULL, s2BurstCpltCallback);

  /* Initialise Sensors */
  initSensor(&hallCurrent, currentHalfDMACallback, currentFullDMACallback);

  /* Configure Signals */
  configSignal(FluxPump_Transformer, &transformer, &switch1, &switch2);
  configSignal(FluxPump_Switch, &transformer, &switch1, NULL);
  configSignal(FluxPump_Switch, &transformer, &switch2, NULL);

  /* Configure Sensors */
  configSensor(&hallCurrent, &transformer);
  
  /* Calculate Waveform LUTs */
  calcSignal(&transformer, LUT_All);
  calcSignal(&switch1, LUT_All);
  calcSignal(&switch2, LUT_All);

  /* Configure PID prototyping function to be called for every sample of the transformer LUT */
  HAL_TIM_RegisterCallback(transformer.trigTIM_Handle, HAL_TIM_OC_DELAY_ELAPSED_CB_ID, callback_function);

  updateTimers(); // Send update command to load preloaded values into registers
  startTimers();  // Start all timers required for flux pump signals

  /* Calibrate Sensors */
  calibSensor(&hallCurrent);

  /* Start all Flux Pump Signals */
  startWaveform(&transformer);
  startWaveform(&switch1);
  startWaveform(&switch2);

  /* Initialise HRTIM for PWM control of H-Bridge - this should be packaged up into a function at some point */
  HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A);
  HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B);
  HAL_HRTIM_SimpleBaseStart_IT(&hhrtim1, HRTIM_TIMERINDEX_MASTER);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(transHalfDMA_Flag)
    { // DMA has passed the midpoint of the Transformer LUT
      transHalfDMA_Flag = 0; // Clear Flag
      if(transformer.updateRequest == Update_FirstHalfLUT) calcSignal(&transformer, LUT_FirstHalf); // Recalculate first half of transformer LUT if requested
      if(transformer.state == Waveform_Request_Off) stopWaveform(&transformer); // Stop the transformer waveform if requested
    }
    if(transFullDMA_Flag)
    { // DMA has passed the end of the Transformer LUT and has wrapped around to the start
      transFullDMA_Flag = 0; // Clear flag
      if(transformer.updateRequest == Update_SecondHalfLUT) calcSignal(&transformer, LUT_SecondHalf); // Recalculate second half of transformer LUT if requested
      if(transformer.state == Waveform_Request_Off) stopWaveform(&transformer); // Stop the transformer waveform if requested
    }
    if(switch1Cplt_Flag)
    { // DMA has finished switch 1 burst
      switch1Cplt_Flag = 0; // Clear Flag
      if(switch1.updateRequest == Update_Clocks)
      { // Reconfigure Switch 1 clocks if requested
        stopWaveform(&switch1); // Dangerous to mess with clocks while signal is running
        configSignal(FluxPump_Switch, &transformer, &switch1, NULL); 
        startWaveform(&switch1);
      }
      if(switch1.updateRequest == Update_FirstHalfLUT) calcSignal(&switch1, LUT_All); // Recalculate Switch 1 LUT if requested
      if(switch1.state == Waveform_Request_On)  startWaveform(&switch1); // Start switch 1 waveform if requested
      if(switch1.state == Waveform_Request_Off) stopWaveform(&switch1); // Stop switch 1 waveform if requested
    }
    if(switch2Cplt_Flag)
    { // DMA has finished switch 2 burst
      switch2Cplt_Flag = 0; // Clear Flag
      if(switch2.updateRequest == Update_Clocks)
      { // Reconfigure Switch 2 clocks if requested
        stopWaveform(&switch2); // Dangerous to mess with clocks while signal is running
        configSignal(FluxPump_Switch, &transformer, &switch2, NULL);
        startWaveform(&switch2);
      }
      if(switch2.updateRequest == 2) calcSignal(&switch2, LUT_All); // Recalculate Switch 2 LUT if requested
      if(switch2.state == Waveform_Request_On)  startWaveform(&switch2); // Start switch 2 waveform if requested
      if(switch2.state == Waveform_Request_Off) stopWaveform(&switch2); // Stop switch 2 waveform if requested
    }
    if(currentHalfDMA_Flag)
    {
      currentHalfDMA_Flag = 0; // Clear flag
      /* Calculate some statistics of the measurement over first half of cycle */
      arm_offset_q15(hallCurrent.BUFLocation, -hallCurrent.calib.offset, hallCurrent.BUFLocation, hallCurrent.BUFSize/2); // Remove DC offset
      arm_min_q15(hallCurrent.BUFLocation, hallCurrent.BUFSize/2, &hallCurrent.calcs.min1, NULL); // Find minimum value
      arm_max_q15(hallCurrent.BUFLocation, hallCurrent.BUFSize/2, &hallCurrent.calcs.max1, NULL); // Find maximum value
      arm_rms_q15(hallCurrent.BUFLocation, hallCurrent.BUFSize/2, &hallCurrent.calcs.rms1); // Calculate RMS
      if(hallCurrent.readFlag == 1)
      { // If there is a request to read the measurement data, send the first half via DMA
        hallCurrent.readFlag = 2; // Set flag so second half is also transmitted when done
        HAL_UART_Transmit_DMA(&hlpuart1, (void *)hallCurrent.BUFLocation, sizeof(hallCurrent.BUFLocation[1])*hallCurrent.BUFSize/2);
      }
    }
    if(currentFullDMA_Flag)
    {
      currentFullDMA_Flag = 0; // Clear flag
      /* Calculate some statistics of the measurement over first half of cycle */
      arm_offset_q15(&hallCurrent.BUFLocation[hallCurrent.BUFSize/2], -hallCurrent.calib.offset, &hallCurrent.BUFLocation[hallCurrent.BUFSize/2], hallCurrent.BUFSize/2); // Remove DC offset
      arm_min_q15(&hallCurrent.BUFLocation[hallCurrent.BUFSize/2], hallCurrent.BUFSize/2, &hallCurrent.calcs.min2, NULL); // Find minimum value
      arm_max_q15(&hallCurrent.BUFLocation[hallCurrent.BUFSize/2], hallCurrent.BUFSize/2, &hallCurrent.calcs.max2, NULL); // Find maximum value
      arm_rms_q15(&hallCurrent.BUFLocation[hallCurrent.BUFSize/2], hallCurrent.BUFSize/2, &hallCurrent.calcs.rms2); // Calculate RMS
      /* Combine stats for first and second half and save to sensor structure */
      hallCurrent.stats.peak = (fmax(hallCurrent.calcs.max1, hallCurrent.calcs.max2) - fmin(hallCurrent.calcs.min1, hallCurrent.calcs.min2)) / 2 * hallCurrent.conversion; // Peak amplitude is range/2
      hallCurrent.stats.rms = sqrt(((hallCurrent.calcs.rms1*hallCurrent.calcs.rms1) + (hallCurrent.calcs.rms2*hallCurrent.calcs.rms2))/2) * hallCurrent.conversion; // Calculate overall RMS from two halves (sorta like a weighted average)
      if(hallCurrent.readFlag == 2)
      { // If there is a request to read the measurement data and the first half has been sent, send the second half via DMA
        hallCurrent.readFlag = 0; // Reset read request to false
        HAL_UART_Transmit_DMA(&hlpuart1, (void *)&hallCurrent.BUFLocation[hallCurrent.BUFSize/2], sizeof(hallCurrent.BUFLocation[hallCurrent.BUFSize/2])*hallCurrent.BUFSize/2);
      }
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
        if(((LPUART1_rxBuffer[old_pos] == '#') | (LPUART1_rxBuffer[old_pos] == '?') | (LPUART1_rxBuffer[old_pos] == '\r')) & (old_pos != commandStart)) 
        { // This code runs if the special command character (#) or a return character is found
          commandStart = commandEnd; // The new command starts immediately after the old command end
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
  RCC_OscInitStruct.PLL.PLLN = 85;
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
void callback_function(TIM_HandleTypeDef * htim)
{ 
  if(transformer.state == Waveform_On) // Only calculate duty cycle if the transformer waveform should be on
  {
    q15_t demand = transformer.LUTLocation[transformer.LUTSize - __HAL_DMA_GET_COUNTER(transformer.DAC_Handle->DMA_Handle1)]; // Determine what the current should be
    q15_t measure = hallCurrent.BUFLocation[hallCurrent.BUFSize - __HAL_DMA_GET_COUNTER((&hadc1)->DMA_Handle)] * 2; // Measure what the most recent current measurement is
    q15_t error =  demand - measure; // The error is the difference

    // volatile q15_t duty = arm_pid_q15(&PID, error); // Havent tried this yet, good luck
    volatile int32_t duty = (demand + 0*error)/DAC_FULLSCALE * hhrtim1.Instance->sMasterRegs.MPER; // Really simple proportional control, but only if you change the 0 to something else

    /* Determine the correct signals for the H-Bridge you are using */
    if (duty > 0) hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = 0x0000; 
    else hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = HRTIM_CMP_MAX; 

    if((duty <= HRTIM_CMP_MIN) & (duty >= -HRTIM_CMP_MIN)) duty = 0; // There is a minimum allowable duty cycle, set to zero if duty is lower than this
    else if (duty >=  (int32_t)(hhrtim1.Instance->sMasterRegs.MPER - HRTIM_CMP_MIN)) duty =  (hhrtim1.Instance->sMasterRegs.MPER - HRTIM_CMP_MIN); // Don't let compare register be larger than period
    else if (duty <= -(int32_t)(hhrtim1.Instance->sMasterRegs.MPER - HRTIM_CMP_MIN)) duty = -(hhrtim1.Instance->sMasterRegs.MPER - HRTIM_CMP_MIN); // Don't let compare register be larger than period
    
    hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = abs(duty); // Take the magnitude of the duty cycle for unipolar modulation
  }
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
