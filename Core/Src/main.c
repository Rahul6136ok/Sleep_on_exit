/*
 * main.c
 *
 *  Created on: Oct 9, 2024
 *      Author: Rahul
 *  Write an application in which TIM6 triggers update interrupt
 *  for every 10ms and in the ISR of TIM6 send data over UART2
 *  1.Measure the Current Consuption without sleep mode
 *  2.Measure the current Consuption with Sleep Mode (Eter Sleep mode using SLEEPONEXIT feature
 * you can get data in the UART Tera term measure current through multimeter then apply sleep mode
 * how to find SLEEP on Exit feature
 * Generic User manual of the ARM cortex M4 device and search System Control block
 * here u find the register SCR .. on first bit you will find SLEEPONEXIT on first bit
 * then again measure the current
 * the current consuption before and after greatly reduced
 * now processor is only awake during entering to ecxiting ISR
 *
 * */


#include <string.h>
#include "stm32f4xx_hal.h"
#include "main.h"

void GPIO_Init(void);
void Error_handler(void);
void TIMER6_Init(void);
void UART2_Init(void);
void SystemClock_Config_HSE(uint8_t clock_freq);

TIM_HandleTypeDef htimer6;
UART_HandleTypeDef huart2;
extern uint8_t some_data[];

int main(void)
{
	HAL_Init();

	GPIO_Init();

	//HAL Suspend tick
	UART2_Init();

	TIMER6_Init();

	//lets start SLEEPONEXIT feature
	//SCB->SCR |= (1 << 1); instead of this you can use API in stm32f4xx_hal_pwr
	//HAL_PWR_EnableSleepOnExit();

	//lets start with fresh Status register of the timer to avoid any
	//spurious timer interrupts
	TIM6->SR = 0;

	//Lets start the timer in interrupt mode
	HAL_TIM_Base_Start_IT(&htimer6);

	while(1);

	return 0;

}

void SystemClock_Config_HSE(uint8_t clock_freq)
{
    RCC_OscInitTypeDef osc_init;
    RCC_ClkInitTypeDef clk_init;

    uint32_t Flatency = 0;

    osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc_init.HSEState = RCC_HSE_BYPASS;
    osc_init.PLL.PLLState = RCC_PLL_ON;
    osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

    switch(clock_freq)
    {
        case SYS_CLOCK_FREQ_50MHZ:
        {
            osc_init.PLL.PLLM = 8;
            osc_init.PLL.PLLN = 100;
            osc_init.PLL.PLLP = 2;
            osc_init.PLL.PLLQ = 2;

            clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |\
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
            clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
            clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
            clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
            clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

            Flatency = FLASH_ACR_LATENCY_1WS;
            break;
        }
        case SYS_CLOCK_FREQ_84MHZ:
        {
            osc_init.PLL.PLLM = 8;
            osc_init.PLL.PLLN = 168;
            osc_init.PLL.PLLP = 2;
            osc_init.PLL.PLLQ = 2;

            clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |\
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
            clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
            clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
            clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
            clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

            Flatency = FLASH_ACR_LATENCY_2WS;
            break;
        }
        case SYS_CLOCK_FREQ_120MHZ:
        {
        	osc_init.PLL.PLLM = 8;
            osc_init.PLL.PLLN = 240;
            osc_init.PLL.PLLP = 2;
            osc_init.PLL.PLLQ = 2;

            clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |\
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
            clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
            clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
            clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
            clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

            Flatency = FLASH_ACR_LATENCY_3WS;
            break;
        case SYS_CLOCK_FREQ_180MHZ:
        {
        	//Enable the clock for the power controller
        	 __HAL_RCC_PWR_CLK_ENABLE();

        	 //set regulator voltage scale as 1
        	 __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);


         	osc_init.PLL.PLLM = 8;
            osc_init.PLL.PLLN =360;
            osc_init.PLL.PLLP = 2;
            osc_init.PLL.PLLQ = 2;

            clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |\
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
            clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
            clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
            clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
            clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

            Flatency = FLASH_ACR_LATENCY_3WS;
        }
        }
        default:
            return;
    }

    if (HAL_RCC_OscConfig(&osc_init) != HAL_OK)
    {
        Error_handler();
    }

    if (HAL_RCC_ClockConfig(&clk_init, Flatency) != HAL_OK)
    {
        Error_handler();
    }

    // Systick configuration
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
/**Configure the Systick
*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void GPIO_Init(void)
{
	// Enable clock for GPIOD (LEDs on PD12 to PD15)
	__HAL_RCC_GPIOD_CLK_ENABLE();

	// Configure PD12 as output push-pull (onboard LED)
	GPIO_InitTypeDef ledgpio;
	ledgpio.Pin = GPIO_PIN_12;   // Onboard LED (Green LED on PD12)
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	ledgpio.Speed = GPIO_SPEED_FREQ_LOW;  // Set speed (optional)
	HAL_GPIO_Init(GPIOD, &ledgpio);

	// Configure PD13 as output push-pull (additional LED)
	ledgpio.Pin = GPIO_PIN_13;   // Onboard Orange LED on PD13
	HAL_GPIO_Init(GPIOD, &ledgpio);

	// You can also configure other LEDs (PD14 - Red, PD15 - Blue) in a similar way if needed
}

void UART2_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate =115200;
	huart2.Init.WordLength =UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX;

	if (HAL_UART_Init(&huart2)!= HAL_OK)
	{
		//there is a problem
		Error_handler();

	}

}

void TIMER6_Init(void)
{
	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler = 4999;
	htimer6.Init.Period = 32 - 1;
	if(HAL_TIM_Base_Init(&htimer6)!= HAL_OK)
	{
		Error_handler();
	}
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if( HAL_UART_Transmit(&huart2,(uint8_t*)some_data,(uint16_t)strlen((char*)some_data),HAL_MAX_DELAY) != HAL_OK)
  {
    Error_handler();
  }
}

/**
  * @brief  Tx Transfer completed callbacks. only for Interrupt mode
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_handler(void)
{
	while(1);
}

