/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"

#include "chromatic_scale.h"
#include "color.h"
#include "controllfc.h"
#include "custom_init.h"

#include <stdlib.h>   /* For EXIT_SUCCESS */
#include <stdbool.h>  /* For true  */
#include <ctype.h> /* For toupper */

void SystemClock_Config(void);

GPIO_TypeDef *GPIOA_handle = (GPIO_TypeDef*) GPIOA_BASE;
GPIO_TypeDef *GPIOB_handle = (GPIO_TypeDef*) GPIOB_BASE;
GPIO_TypeDef *GPIOC_handle = (GPIO_TypeDef*) GPIOC_BASE;

//Function Defs

void led_Handler(const uint8_t colors[NUMBER_OF_CHANELS]);
void USART_ReadByte(USART_TypeDef *USARTx, uint8_t *buffer);
void USART_Write(USART_TypeDef *USARTx, uint8_t *buffer, uint32_t msgLength);
void startNote(unsigned char c);
void stopNote();

int main(void) {
	/* System Configuration */
	HAL_Init();
	SystemClock_Config();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_TIM1_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_TIM7_CLK_ENABLE();
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();

	//Inits
	custom_USART2_Init(USART2);
	custom_NVIC_Init();
	custom_DigitalOutput_Init();

	custom_TIM1_Init();
	custom_TIM2_Init();
	custom_TIM3_Init();
	custom_TIM7_Init();

	//Enable interrupt Routine for USART
	SET_BIT(USART2->CR1, USART_CR1_RXNEIE_Msk);

	while (true) {

	}

	return EXIT_SUCCESS;
}

void led_Handler(const uint8_t colors[NUMBER_OF_CHANELS]) {
	TIM3->CCR1 = colors[RED]; // R
	TIM3->CCR2 = colors[GREEN]; // G
	TIM1->CCR2 = colors[BLUE]; // B
}

void USART_Write(USART_TypeDef *USARTx, uint8_t *buffer, uint32_t msgLength) {
	for (uint32_t index = 0; index < msgLength; ++index) {
		// Wait for transfer from DR to to shift register
		while (!(USARTx->SR & USART_SR_TXE)) {
			;
		}
		USARTx->DR = buffer[index] & 0xFF;
	}
	// Wait for transmission completion
	while (!(USARTx->SR & USART_SR_TC)) {
		;
	}
}

void USART_ReadByte(USART_TypeDef *USARTx, uint8_t *buffer) {
	if (buffer) {
		//Wait until data receives in the DR
		while (!(USARTx->SR & USART_SR_RXNE)) {
			;
		}
		*buffer = USARTx->DR;
	}
}

void USART2_IRQHandler(void) {
	unsigned char c;

	USART_ReadByte(USART2, &c);
	startNote(c);
	c = toupper(c);
	USART_Write(USART2, &c, 1);
}

void TIM7_IRQHandler(void) {
	if(READ_BIT(TIM7->SR, TIM_SR_UIF) != 0) {
		CLEAR_BIT(TIM7->SR, TIM_SR_UIF);
		stopNote();
	}
}

void startNote(unsigned char c) {
	int8_t note = keyToNote(c);
	static int8_t octave = 8;

	if (note >= 0) {
		// Convert Notes from 0 to 15 to index
		uint8_t noteIndex = A0_OFFSET + note + octave * 12;
		// Set Frequency
		WRITE_REG(TIM2->PSC, NOTES_PSC_16MHz[noteIndex]);

		//For testing turn on the red LED
		CLEAR_BIT(GPIOB_handle->ODR, GPIO_ODR_ODR_4);

		//Start Timers
		SET_BIT(TIM1->CR1, TIM_CR1_CEN);
		SET_BIT(TIM2->CR1, TIM_CR1_CEN);
		SET_BIT(TIM3->CR1, TIM_CR1_CEN);

		led_Handler(COLORS[(noteIndex + 4) % NUMBER_OF_COLORS]);

		//Start TIM7
		SET_BIT(TIM7->CR1, TIM_CR1_CEN);
	}
	else {
		pianoController(c, &octave);
	}
}

void stopNote(void) {
	SET_BIT(GPIOB_handle->ODR, GPIO_ODR_ODR_4);
	CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);
	CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN);
	CLEAR_BIT(TIM3->CR1, TIM_CR1_CEN);
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
