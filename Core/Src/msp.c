/*
 * msp.c
 *
 *  Created on: Oct 9, 2024
 *      Author: Rahul
 */

#include "main.h"

/**
  * @brief  Initialize the MSP.
  * @retval None
  */
void HAL_MspInit(void)
{
  //Here will do low level processor specific inits.
  //1. Set up the priority grouping of the arm cortex mx processor
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  //2. Enable the required system exceptions of the arm cortex mx processor
  SCB->SHCSR |= 0x7 << 16; //usage fault, memory fault and bus fault system exceptions

  //3. configure the priority for the system exceptions
  HAL_NVIC_SetPriority(MemoryManagement_IRQn,0,0);
  HAL_NVIC_SetPriority(BusFault_IRQn,0,0);
  HAL_NVIC_SetPriority(UsageFault_IRQn,0,0);
}

/**
  * @brief  Initializes the TIM Base MSP.
  * @param  htim TIM Base handle
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htimer)
{
  //1. enable the clock for the TIM6 peripheral
  __HAL_RCC_TIM6_CLK_ENABLE();

  //2. Enable the IRQ of TIM6
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  //3. setup the priority for TIM6_DAC_IRQn
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn,0,0);
}

/**
  * @brief  UART MSP Init.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_uart;

    // Check if the USART instance is USART2
    if(huart->Instance == USART2)
    {
        // 1. Enable the clock for USART2 and GPIOA
        __HAL_RCC_USART2_CLK_ENABLE();   // Enable USART2 clock
        __HAL_RCC_GPIOA_CLK_ENABLE();    // Enable GPIOA clock

        // 2. Pin muxing configurations for PA2 (USART2_TX) and PA3 (USART2_RX)
        gpio_uart.Pin = GPIO_PIN_2;                      // PA2 as TX
        gpio_uart.Mode = GPIO_MODE_AF_PP;                // Alternate function push-pull
        gpio_uart.Pull = GPIO_PULLUP;                    // Pull-up
        gpio_uart.Speed = GPIO_SPEED_FREQ_LOW;           // Low speed
        gpio_uart.Alternate = GPIO_AF7_USART2;           // Alternate function for USART2
        HAL_GPIO_Init(GPIOA, &gpio_uart);                // Initialize PA2

        // Configure PA3 for USART2_RX
        gpio_uart.Pin = GPIO_PIN_3;                      // PA3 as RX
        HAL_GPIO_Init(GPIOA, &gpio_uart);                // Initialize PA3

        // 3. Enable the IRQ and set up the priority (NVIC settings)
        HAL_NVIC_SetPriority(USART2_IRQn, 15, 0);        // Set priority
        HAL_NVIC_EnableIRQ(USART2_IRQn);                // Enable USART2 interrupt in NVIC
    }
}
