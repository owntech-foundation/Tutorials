/*
 * Copyright (c) 2021 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGLPV2.1
 */

/**
 * @author Farah Hassan Ayoub <ayoub.farah-hassan@laas.fr>
 */



// include 

#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_hrtim.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32_ll_comp.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"


// PID parameters

static double p;
static double i;

static double integrator_mem = 0;

static double integrator_mem_leg2 = 0;

static double pid_period;
static double reset_value;

// 2p2z for 4ms step response

const double Az[3] = {1.000000000000000,  -0.6566,  -0.3434};
const double Bz[3] = {-0.001687,   0.004444,  0.006131};

// 2p2z for 2ms step response

// const double Az[3] = {1.000000000000000,  - 0.6566,  -0.3434};
// const double Bz[3] = {0.2763,   0.02747,  - 0.2488};


const double Ts = 100.0e-6;


// Comparator initialization for leg 2

void MX_COMP3_Init(void)
{

  /* USER CODE BEGIN COMP3_Init 0 */

  /* USER CODE END COMP3_Init 0 */

  LL_COMP_InitTypeDef COMP_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /**COMP3 GPIO Configuration
  PC1   ------> COMP3_INP
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN COMP3_Init 1 */

  /* USER CODE END COMP3_Init 1 */
  COMP_InitStruct.InputPlus = LL_COMP_INPUT_PLUS_IO2;
  COMP_InitStruct.InputMinus = LL_COMP_INPUT_MINUS_DAC1_CH1;
  COMP_InitStruct.InputHysteresis = LL_COMP_HYSTERESIS_NONE;
  COMP_InitStruct.OutputPolarity = LL_COMP_OUTPUTPOL_INVERTED;
  COMP_InitStruct.OutputBlankingSource = LL_COMP_BLANKINGSRC_NONE;
  LL_COMP_Init(COMP3, &COMP_InitStruct);

  /* Wait loop initialization and execution */
  /* Note: Variable divided by 2 to compensate partially CPU processing cycles */
  __IO uint32_t wait_loop_index = 0;
  wait_loop_index = (LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US * (SystemCoreClock / (1000000 * 2)));
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
  LL_EXTI_DisableEvent_0_31(LL_EXTI_LINE_29);
  LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_29);
  /* USER CODE BEGIN COMP3_Init 2 */

  /* USER CODE END COMP3_Init 2 */

}


// Comparator initialization for leg 1

static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  LL_COMP_InitTypeDef COMP_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**COMP1 GPIO Configuration
  PA1   ------> COMP1_INP
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  COMP_InitStruct.InputPlus = LL_COMP_INPUT_PLUS_IO1;
  COMP_InitStruct.InputMinus = LL_COMP_INPUT_MINUS_DAC3_CH1;
  COMP_InitStruct.InputHysteresis = LL_COMP_HYSTERESIS_NONE;
  COMP_InitStruct.OutputPolarity = LL_COMP_OUTPUTPOL_INVERTED;
  COMP_InitStruct.OutputBlankingSource = LL_COMP_BLANKINGSRC_NONE;
  LL_COMP_Init(COMP1, &COMP_InitStruct);

  /* Wait loop initialization and execution */
  /* Note: Variable divided by 2 to compensate partially CPU processing cycles */
  __IO uint32_t wait_loop_index = 0;
  wait_loop_index = (LL_COMP_DELAY_VOLTAGE_SCALER_STAB_US * (SystemCoreClock / (1000000 * 2)));
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
  LL_EXTI_DisableEvent_0_31(LL_EXTI_LINE_21);
  LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_21);
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

// DAC initialization for leg 1

static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */

  /* USER CODE END DAC3_Init 0 */

  LL_DAC_InitTypeDef DAC_InitStruct = {0};

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC3);

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */

  /** DAC channel OUT1 config
  */
  LL_DAC_SetSignedFormat(DAC3, LL_DAC_CHANNEL_1, LL_DAC_SIGNED_FORMAT_DISABLE);
  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_EXT_HRTIM_RST_TRG1;
  DAC_InitStruct.TriggerSource2 = LL_DAC_TRIG_EXT_HRTIM_STEP_TRG1;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_SAWTOOTH;
  DAC_InitStruct.WaveAutoGenerationConfig = __LL_DAC_FORMAT_SAWTOOTHWAVECONFIG(LL_DAC_SAWTOOTH_POLARITY_DECREMENT, 3300, 264);
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_DISABLE;
  DAC_InitStruct.OutputConnection = LL_DAC_OUTPUT_CONNECT_INTERNAL;
  DAC_InitStruct.OutputMode = LL_DAC_OUTPUT_MODE_NORMAL;
  LL_DAC_Init(DAC3, LL_DAC_CHANNEL_1, &DAC_InitStruct);
  LL_DAC_EnableTrigger(DAC3, LL_DAC_CHANNEL_1);
  LL_DAC_DisableDMADoubleDataMode(DAC3, LL_DAC_CHANNEL_1);
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

}

// DAC initialization for leg 1

void MX_DAC1_Init(void)
{

    /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  LL_DAC_InitTypeDef DAC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**DAC1 GPIO Configuration
  PA4   ------> DAC1_OUT1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC channel OUT1 config
  */
  LL_DAC_SetSignedFormat(DAC1, LL_DAC_CHANNEL_1, LL_DAC_SIGNED_FORMAT_DISABLE);
  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_EXT_HRTIM_RST_TRG3;
  DAC_InitStruct.TriggerSource2 = LL_DAC_TRIG_EXT_HRTIM_STEP_TRG3;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_SAWTOOTH;
  DAC_InitStruct.WaveAutoGenerationConfig = __LL_DAC_FORMAT_SAWTOOTHWAVECONFIG(LL_DAC_SAWTOOTH_POLARITY_DECREMENT, 3000, 264);
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  DAC_InitStruct.OutputConnection = LL_DAC_OUTPUT_CONNECT_INTERNAL;
  DAC_InitStruct.OutputMode = LL_DAC_OUTPUT_MODE_NORMAL;
  LL_DAC_Init(DAC1, LL_DAC_CHANNEL_1, &DAC_InitStruct);
  LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_1);
  LL_DAC_DisableDMADoubleDataMode(DAC1, LL_DAC_CHANNEL_1);
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}


void MX_DAC2_Init(void)
{

    /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  LL_DAC_InitTypeDef DAC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC2);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**DAC1 GPIO Configuration
  PA6   ------> DAC2_OUT1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN DAC2_Init 1 */

  /* USER CODE END DAC2_Init 1 */

  /** DAC channel OUT1 config
  */

  LL_DAC_SetWaveAutoGeneration(DAC2, LL_DAC_CHANNEL_1, LL_DAC_WAVE_AUTO_GENERATION_NONE);
	LL_DAC_DisableTrigger(DAC2, LL_DAC_CHANNEL_1);
	LL_DAC_DisableDMADoubleDataMode(DAC2, LL_DAC_CHANNEL_1);

	LL_DAC_ConfigOutput(DAC2, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_MODE_NORMAL, LL_DAC_OUTPUT_BUFFER_ENABLE, LL_DAC_OUTPUT_CONNECT_GPIO);
	LL_DAC_Enable(DAC2, LL_DAC_CHANNEL_1);


 	LL_DAC_SetSignedFormat(DAC2, LL_DAC_CHANNEL_1, LL_DAC_SIGNED_FORMAT_DISABLE);
  LL_DAC_SetWaveAutoGeneration(DAC2, LL_DAC_CHANNEL_1, LL_DAC_WAVE_AUTO_GENERATION_NONE);
	LL_DAC_DisableTrigger(DAC2, LL_DAC_CHANNEL_1);
	LL_DAC_DisableDMADoubleDataMode(DAC2, LL_DAC_CHANNEL_1);

	LL_DAC_ConfigOutput(DAC2, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_MODE_NORMAL, LL_DAC_OUTPUT_BUFFER_ENABLE, LL_DAC_OUTPUT_CONNECT_GPIO);
	LL_DAC_Enable(DAC2, LL_DAC_CHANNEL_1);
  // DAC_InitStruct.TriggerSource = LL_DAC_TRIG_EXT_HRTIM_RST_TRG3;
  // DAC_InitStruct.TriggerSource2 = LL_DAC_TRIG_EXT_HRTIM_STEP_TRG3;
  // DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  // DAC_InitStruct.WaveAutoGenerationConfig = __LL_DAC_FORMAT_SAWTOOTHWAVECONFIG(LL_DAC_SAWTOOTH_POLARITY_DECREMENT, 3000, 264);
  // DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  // DAC_InitStruct.OutputConnection = LL_DAC_OUTPUT_CONNECT_INTERNAL;
  // DAC_InitStruct.OutputMode = LL_DAC_OUTPUT_MODE_NORMAL;
  // LL_DAC_Init(DAC1, LL_DAC_CHANNEL_1, &DAC_InitStruct);
  // LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_1);
  // LL_DAC_DisableDMADoubleDataMode(DAC1, LL_DAC_CHANNEL_1);
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

// HRTIM initialization 


static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_HRTIM1);

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  LL_HRTIM_ConfigDLLCalibration(HRTIM1, LL_HRTIM_DLLCALIBRATION_MODE_CONTINUOUS, LL_HRTIM_DLLCALIBRATION_RATE_3);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_EE_SetPrescaler(HRTIM1, LL_HRTIM_EE_PRESCALER_DIV1);
  LL_HRTIM_EE_SetSrc(HRTIM1, LL_HRTIM_EVENT_4, LL_HRTIM_EEV4SRC_COMP1_OUT);
  LL_HRTIM_EE_SetPolarity(HRTIM1, LL_HRTIM_EVENT_4, LL_HRTIM_EE_POLARITY_HIGH);
  LL_HRTIM_EE_SetSensitivity(HRTIM1, LL_HRTIM_EVENT_4, LL_HRTIM_EE_SENSITIVITY_LEVEL);
  LL_HRTIM_EE_SetFastMode(HRTIM1, LL_HRTIM_EVENT_4, LL_HRTIM_EE_FASTMODE_DISABLE);
  LL_HRTIM_EE_SetSrc(HRTIM1, LL_HRTIM_EVENT_5, LL_HRTIM_EEV5SRC_COMP3_OUT);
  LL_HRTIM_EE_SetPolarity(HRTIM1, LL_HRTIM_EVENT_5, LL_HRTIM_EE_POLARITY_HIGH);
  LL_HRTIM_EE_SetSensitivity(HRTIM1, LL_HRTIM_EVENT_5, LL_HRTIM_EE_SENSITIVITY_LEVEL);
  LL_HRTIM_EE_SetFastMode(HRTIM1, LL_HRTIM_EVENT_5, LL_HRTIM_EE_FASTMODE_DISABLE);

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_PRESCALERRATIO_MUL32);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_MODE_CONTINUOUS);
	LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_MASTER, 27200);
	LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_MASTER, 0x00);
	LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_MASTER);
	LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
	LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_MASTER);
	LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_MASTER);
	LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_DACTRIG_NONE);
	LL_HRTIM_TIM_DisablePreload(HRTIM1, LL_HRTIM_TIMER_MASTER);
	LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_UPDATEGATING_INDEPENDENT);
	LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_UPDATETRIG_NONE);
	LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
	LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_MASTER);
	LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_MASTER, 13600);

  LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_MASTER);

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_PRESCALERRATIO_MUL32);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_A, 27200);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_A, 0x00);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetCountingMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_COUNTING_MODE_UP);
  LL_HRTIM_TIM_SetTriggeredHalfMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_TRIGHALF_DISABLED);
  LL_HRTIM_TIM_SetComp1Mode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_GTCMP1_EQUAL);
  LL_HRTIM_TIM_SetComp3Mode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_GTCMP3_EQUAL);
  LL_HRTIM_TIM_SetDualDacResetTrigger(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DCDR_COUNTER);
  LL_HRTIM_TIM_SetDualDacStepTrigger(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DCDS_CMP2);
  LL_HRTIM_TIM_EnableDualDacTrigger(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_DisablePreload(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_DisableResyncUpdate(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_UPDATETRIG_NONE|LL_HRTIM_UPDATETRIG_NONE);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_RESETTRIG_MASTER_PER); //
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_EnableDeadTime(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_EnableResyncUpdate(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, 24480);
  LL_HRTIM_TIM_SetCompareMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_COMPAREUNIT_2, LL_HRTIM_COMPAREMODE_REGULAR);
  LL_HRTIM_TIM_SetCompare2(HRTIM1, LL_HRTIM_TIMER_A, 272);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_A, 2720);
  LL_HRTIM_TIM_SetCompare4(HRTIM1, LL_HRTIM_TIMER_A, 1088);
  LL_HRTIM_TIM_SetCompareMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_COMPAREUNIT_4, LL_HRTIM_COMPAREMODE_REGULAR);
  LL_HRTIM_TIM_SetEventFilter(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_EVENT_4, LL_HRTIM_EEFLTR_BLANKINGCMP3);
  LL_HRTIM_TIM_SetEventLatchStatus(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_EVENT_4, LL_HRTIM_EELATCH_DISABLED);
  LL_HRTIM_DT_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DT_PRESCALER_MUL8);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_A, 136);
  LL_HRTIM_DT_SetRisingSign(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DT_RISING_POSITIVE);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_A, 136);
  LL_HRTIM_DT_SetFallingSign(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DT_FALLING_POSITIVE);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUTPUTSET_TIMCMP1|LL_HRTIM_OUTPUTSET_EEV_4);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUTPUTRESET_TIMCMP4);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUTPUTSET_NONE);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUTPUTRESET_NONE);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);

  LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_A);

  LL_HRTIM_SetADCTrigSrc(HRTIM1, LL_HRTIM_ADCTRIG_1, LL_HRTIM_ADCTRIG_SRC13_TIMACMP3);
  LL_HRTIM_SetADCTrigUpdate(HRTIM1, LL_HRTIM_ADCTRIG_1, LL_HRTIM_ADCTRIG_UPDATE_TIMER_A);


  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_PRESCALERRATIO_MUL32);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_C, 27200);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_C, 0x00);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetCountingMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_COUNTING_MODE_UP);
  LL_HRTIM_TIM_SetTriggeredHalfMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_TRIGHALF_DISABLED);
  LL_HRTIM_TIM_SetComp1Mode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_GTCMP1_EQUAL);
  LL_HRTIM_TIM_SetComp3Mode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_GTCMP3_EQUAL);
  LL_HRTIM_TIM_SetDualDacResetTrigger(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DCDR_COUNTER);
  LL_HRTIM_TIM_SetDualDacStepTrigger(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DCDS_CMP2);
  LL_HRTIM_TIM_EnableDualDacTrigger(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetInterleavedMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_INTERLEAVED_MODE_DISABLED);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_DisablePreload(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_DisableResyncUpdate(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_UPDATETRIG_NONE|LL_HRTIM_UPDATETRIG_NONE);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_RESETTRIG_MASTER_CMP1); //
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_EnableDeadTime(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_EnableResyncUpdate(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, 24480);
  LL_HRTIM_TIM_SetCompareMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_COMPAREUNIT_2, LL_HRTIM_COMPAREMODE_REGULAR);
  LL_HRTIM_TIM_SetCompare2(HRTIM1, LL_HRTIM_TIMER_C, 272);
  LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_C, 2720);
  LL_HRTIM_TIM_SetCompare4(HRTIM1, LL_HRTIM_TIMER_C, 1088);
  LL_HRTIM_TIM_SetCompareMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_COMPAREUNIT_4, LL_HRTIM_COMPAREMODE_REGULAR);
  LL_HRTIM_TIM_SetEventFilter(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_EVENT_5, LL_HRTIM_EEFLTR_BLANKINGCMP3);
  LL_HRTIM_TIM_SetEventLatchStatus(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_EVENT_5, LL_HRTIM_EELATCH_DISABLED);
  LL_HRTIM_DT_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DT_PRESCALER_MUL8);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_C, 136);
  LL_HRTIM_DT_SetRisingSign(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DT_RISING_POSITIVE);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_C, 136);
  LL_HRTIM_DT_SetFallingSign(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DT_FALLING_POSITIVE);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUTPUTSET_TIMCMP4);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUTPUTRESET_TIMCMP1|LL_HRTIM_OUTPUTRESET_EEV_5);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_FAULTSTATE_INACTIVE);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUTPUTSET_NONE);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUTPUTRESET_NONE);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);

  LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_C);

  LL_HRTIM_SetADCTrigSrc(HRTIM1, LL_HRTIM_ADCTRIG_3, LL_HRTIM_ADCTRIG_SRC13_TIMCCMP3);
  LL_HRTIM_SetADCTrigUpdate(HRTIM1, LL_HRTIM_ADCTRIG_3, LL_HRTIM_ADCTRIG_UPDATE_TIMER_C);

  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    /**HRTIM1 GPIO Configuration
    PB12     ------> HRTIM1_CHC1
    PB13     ------> HRTIM1_CHC2
    PA8     ------> HRTIM1_CHA1
    PA9     ------> HRTIM1_CHA2
    */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

// OpAmp configured in follower to watch DAC3 output

static void MX_OPAMP6_Init(void)
{

  /* USER CODE BEGIN OPAMP6_Init 0 */

  /* USER CODE END OPAMP6_Init 0 */

  LL_OPAMP_InitTypeDef OPAMP_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**OPAMP6 GPIO Configuration
  PB11   ------> OPAMP6_VOUT
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP6_Init 1 */

  /* USER CODE END OPAMP6_Init 1 */
  OPAMP_InitStruct.PowerMode = LL_OPAMP_POWERMODE_NORMALSPEED;
  OPAMP_InitStruct.FunctionalMode = LL_OPAMP_MODE_FOLLOWER;
  OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_DAC;
  LL_OPAMP_Init(OPAMP6, &OPAMP_InitStruct);
  LL_OPAMP_SetInputsMuxMode(OPAMP6, LL_OPAMP_INPUT_MUX_DISABLE);
  LL_OPAMP_SetInternalOutput(OPAMP6, LL_OPAMP_INTERNAL_OUPUT_DISABLED);
  LL_OPAMP_SetTrimmingMode(OPAMP6, LL_OPAMP_TRIMMING_FACTORY);
  /* USER CODE BEGIN OPAMP6_Init 2 */

  /* USER CODE END OPAMP6_Init 2 */

}

// Initialization of current mode peripheral 

void Init_CurrentMode_peripheral()
{   

    // init for leg1 and leg2 drivers
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

    LL_GPIO_SetPinMode      (GPIOC, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
	  LL_GPIO_SetPinSpeed     (GPIOC, LL_GPIO_PIN_12, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_12, LL_GPIO_OUTPUT_PUSHPULL);
	  LL_GPIO_SetPinPull      (GPIOC, LL_GPIO_PIN_12, LL_GPIO_PULL_NO);
    

    LL_GPIO_SetPinMode      (GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
	  LL_GPIO_SetPinSpeed     (GPIOC, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	  LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);
	  LL_GPIO_SetPinPull      (GPIOC, LL_GPIO_PIN_13, LL_GPIO_PULL_NO);
   

    // leg1 peripheral initialization
	  MX_COMP1_Init();
    MX_DAC3_Init();
    MX_HRTIM1_Init();
    MX_OPAMP6_Init();

    // leg2 peripheral initialization
    MX_COMP3_Init();
    MX_DAC1_Init();

    MX_DAC2_Init();


}

// current mode PID settings initialization

void Init_CurrentMode_PID(double p_i, double i_i, uint32_t pid_period_us )
{
      float million = 1000000;
	    p = p_i;
	    i = i_i;
      pid_period = pid_period_us/ million;

}

// Enabling the peripheral for leg 1

void Enable_CurrentMode()
{
  // LEG1 PERIPHERAL 
  LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1);
  LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA2);
  LL_OPAMP_Enable(OPAMP6);
  LL_COMP_Enable(COMP1);
  LL_DAC_Enable(DAC3, LL_DAC_CHANNEL_1);
  LL_GPIO_SetOutputPin    (GPIOC, LL_GPIO_PIN_12);

}

// Enabling peripheral for leg 2

void Enable_CurrentMode_leg2()
{
   // LEG2 PERIPHERAL

  LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TC1);
  LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TC2);
  LL_COMP_Enable(COMP3);
  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
   LL_GPIO_SetOutputPin    (GPIOC, LL_GPIO_PIN_13);

}

// Disabling peripheral for leg1

void Disable_CurrentMode()
{
  // LEG1 PERIPHERAL
  LL_HRTIM_DisableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1);
  LL_HRTIM_DisableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA2);
  LL_GPIO_ResetOutputPin    (GPIOC, LL_GPIO_PIN_12);

}


// Disabling peripheral for leg2

void Disable_CurrentMode_leg2()
{
    // LEG2 PERIPHERAL 

  LL_HRTIM_DisableOutput(HRTIM1, LL_HRTIM_OUTPUT_TC1);
  LL_HRTIM_DisableOutput(HRTIM1, LL_HRTIM_OUTPUT_TC2);
  LL_GPIO_ResetOutputPin    (GPIOC, LL_GPIO_PIN_13);


}


// Set the sawtooth peak voltage (set_voltage) and low point voltage (reset_voltage) to set the current reference for leg1 

void set_satwtooth(double set_voltage, double reset_voltage)
{

  double Dv = set_voltage - reset_voltage; 

  if(Dv < 0) Dv = 0;

  uint32_t set_data = (uint32_t) (4096*set_voltage)/(2.048);

  if(set_data > 4096) set_data = 4000;
  LL_DAC_SetWaveSawtoothResetData(DAC3, 1, set_data);

  uint32_t reset_data = (uint32_t) (Dv*65536)/(2.048*100);
  LL_DAC_SetWaveSawtoothStepData(DAC3, 1, reset_data); 


}

void set_DAC2_value(uint32_t dac_value)
{
  LL_DAC_ConvertData12RightAligned(DAC2, LL_DAC_CHANNEL_1, dac_value);
}


// Set the sawtooth peak voltage (set_voltage) and low point voltage (reset_voltage) to set the current reference for leg2 


void set_satwtooth_leg2(double set_voltage, double reset_voltage)
{

  double Dv = set_voltage - reset_voltage; 

  if(Dv < 0) Dv = 0;

  uint32_t set_data = (uint32_t) (4096*set_voltage)/(2.048);

  if(set_data > 4096) set_data = 4000;

  LL_DAC_SetWaveSawtoothResetData(DAC1, 1, set_data);

  uint32_t reset_data = (uint32_t) (Dv*65536)/(2.048*100);
  
  LL_DAC_SetWaveSawtoothStepData(DAC1, 1, reset_data); 


}

// Control of the ouput voltage in leg1 with a PID controller

void Update_DutyCycle_CM(double reference, double measurement)
{
	/////
	// Compute error

	double error = reference - measurement;

	/////
	// Compute derivative term

	double sum = (p * error) + integrator_mem;
	
  /////
	// Compute reset value

  reset_value = sum*0.100 + 1.053;

  if(reset_value > 2.35) reset_value = 2.35;

  set_satwtooth(reset_value, reset_value - 0.3);
  set_satwtooth_leg2(reset_value, reset_value - 0.3);
	/////
	// Compute integral term

	integrator_mem += i * error * pid_period;


}

// Control of the ouput voltage in leg2 with a PID controller

void Update_DutyCycle_CM_leg2(double reference, double measurement)
{
	/////
	// Compute error

	double error = reference - measurement;

	/////
	// Compute derivative term

	double sum = (p * error) + integrator_mem_leg2;
	
  /////
	// Compute reset value

  reset_value = sum*0.100 + 1.053;

  if(reset_value > 2.35) reset_value = 2.35;

  set_satwtooth_leg2(reset_value, reset_value - 0.3);

	/////
	// Compute integral term

	integrator_mem_leg2 += i * error * pid_period;


}

// Control of the output voltage in leg1 with a 2p2z controller

double p2z2_control(double yref, double y)
{
static double e[3] = {0.0, 0.0, 0.0};
static double u[3] = {0.0, 0.0, 0.0};
int kLoop;
double reset_data; 


e[0] = yref - y;

u[0] = Bz[0] * e[0] + Bz[1] * e[1]  + Bz[2] * e[2] - Az[1] * u[1] - Az[2] * u[2];

//if (u[0] < 0.1) u[0] = 0.1;
//if (u[0] > 20.0) u[0] = 20.0;
for (kLoop = 2;kLoop>0;--kLoop) {
    u[kLoop] = u[kLoop-1];
    e[kLoop] = e[kLoop-1];
}

reset_data = u[0]/10;

if(reset_data > 2.35) reset_data = 2.35;

set_satwtooth(reset_data, reset_data - 0.3);

return u[1];
}