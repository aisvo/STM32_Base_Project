/**
 ******************************************************************************
 * @file    main.c
 * @brief   Example main file
 *          contains the main code for blinking an LED
 @verbatim
  ==============================================================================
                       ##### Title of the project  #####
  ==============================================================================

 @endverbatim
 ******************************************************************************
 * @attention
 * 
 * LICENSE
 * 
 * The MIT License (MIT)
 * 
 * Copyright (c) 2020 Rohit Gujarathi
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include "utilities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32l1xx_hal.h"

#include "simple_module.h"

void SystemClock_Config(void);

static GPIO_InitTypeDef  GPIO_InitStruct;

/** @brief   LED pin */
#define LED1_PIN GPIO_PIN_13
/** @brief   LED port */
#define LED1_PORT GPIOB

/**
* @brief    Task for blinking an LED every second
* 
* @param    pvParameters void pointer to task parameters
* 
* @retval   void
*/
void LedBlinky_Task(void *pvParameters) {
    while (1) {
        HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

int main ( void ) {
#if SEMIHOSTING
    initialise_monitor_handles();
    setbuf(stdout, NULL);
    //~ setvbuf(stdout, NULL, _IONBF, 0);
    INFO("Main program start");
#endif
    HAL_Init();
    SystemClock_Config();
    GPIO_InitStruct.Pin   = LED1_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(LED1_PORT, &GPIO_InitStruct);
    // calling something from the modules
    SomethingSimple(2, 7);
    xTaskCreate( LedBlinky_Task,						/* The function that implements the task. */
                 "LedBlinky", 							/* The text name assigned to the task - for debug only as it is not used by the kernel. */
                 configMINIMAL_STACK_SIZE, 				/* The size of the stack to allocate to the task. */
                 NULL, 									/* The parameter passed to the task - just to check the functionality. */
                 3, 										/* The priority assigned to the task. */
                 NULL );									/* The task handle is not required, so NULL is passed. */
    vTaskStartScheduler();
    while (1) {
    }
    return 0;
}

/**
* @brief    Setup the system clock
* 
* @note     This function is taken from STM32CubeMX
* 
* @retval   void
*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
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
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LCD;
    PeriphClkInit.LCDClockSelection = RCC_RTCCLKSOURCE_LSE;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}
