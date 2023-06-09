/*
 *******************************************************************************
 * Copyright (c) 2017, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */

#include "pins_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

// Pin number
const PinName digitalPin[] = {
  PA_0,  //D6
  PA_1,  //D7
  PA_4,  //D10 / A0
  PA_6,  //D12 / A2
  PA_8,  //D18 
  PB_4,  //D27
  PB_5,  //D28
  PB_6,  //D29
  PB_7,  //D30

  //NC,    //D15
  //NC,    //D16
  PA_11, //D21 - USB_D-
  PA_12, //D22 - USB_D+
  PA_13, //D23 - SWD
  PA_14, //D24 - SWD
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  7, //A0
  10, //A1
  12, //A2
  18, //A3
};

#ifdef __cplusplus
}
#endif

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief  System Clock Configuration
 * @param  None
 * @retval None
 */
WEAK 	void SystemClock_Config(void)	
{	
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};	
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};	
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};	
  /** Configure the main internal regulator output voltage	
  */	
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);	
  /** Initializes the RCC Oscillators according to the specified parameters	
  * in the RCC_OscInitTypeDef structure.	
  */	
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;	
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;	
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;	
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;	
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;	
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;	
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;	
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;	
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
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)	
  {	
    Error_Handler();	
  }	
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C3	
                              |RCC_PERIPHCLK_USB;	
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;	
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;	
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;	
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)	
  {	
    Error_Handler();	
  }	
}

#ifdef __cplusplus
}
#endif