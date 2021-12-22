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

#include <stdlib.h>   /* For EXIT_SUCCESS */
#include <stdbool.h>  /* For true  */
#include <ctype.h> /* For toupper */

void SystemClock_Config(void);

GPIO_TypeDef *GPIOA_handle = (GPIO_TypeDef*) GPIOA_BASE;
GPIO_TypeDef *GPIOB_handle = (GPIO_TypeDef*) GPIOB_BASE;
GPIO_TypeDef *GPIOC_handle = (GPIO_TypeDef*) GPIOC_BASE;

//Function Defs
void custom_GPIO(void);
void custom_TIM1_Init(void);
void custom_TIM2_Init(void);
void custom_TIM3_Init(void);
void custom_TIM7_Init(void);
void led_Handler(const uint8_t colors[NUMBER_OF_CHANELS]);
void TIM_LED_PWM_Init(TIM_TypeDef *t, int ccmr_num);
void custom_USART2_Init(USART_TypeDef *USARTx);
void custom_NVIC_Init(void);
void custom_DigitalOutput_Init(void);
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
	custom_GPIO();
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

void custom_TIM1_Init(void) {
	SET_BIT(TIM1->CCMR1, TIM_CCMR1_OC2M_0);
	SET_BIT(TIM1->CCMR1, TIM_CCMR1_OC2M_1);
	SET_BIT(TIM1->CCMR1, TIM_CCMR1_OC2M_2);
	SET_BIT(TIM1->CCMR1, TIM_CCMR1_OC2PE);
	SET_BIT(TIM1->CCER, TIM_CCER_CC2E);
	SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
	WRITE_REG(TIM1->ARR, 1 << 8);
	// Transfer Initialization to Shadow Register
	SET_BIT(TIM1->EGR, TIM_EGR_UG);
}

void custom_TIM2_Init(void) {
	SET_BIT(TIM2->CCMR2, TIM_CCMR2_OC3M_0);
	SET_BIT(TIM2->CCMR2, TIM_CCMR2_OC3M_1);
	SET_BIT(TIM2->CCMR2, TIM_CCMR2_OC3M_2);
	SET_BIT(TIM2->CCMR2, TIM_CCMR2_OC3PE);
	SET_BIT(TIM2->CCER, TIM_CCER_CC3E);
	WRITE_REG(TIM2->PSC, NOTES_PSC_16MHz[CONCERT_PITCH_INDEX]);
	WRITE_REG(TIM2->ARR, 1 << 9);
	WRITE_REG(TIM2->CCR3, 1 << 7);
	// Transfer Initialization to Shadow Register
	SET_BIT(TIM2->EGR, TIM_EGR_UG);
}

void custom_TIM3_Init(void) {
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1M_0);
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1M_1);
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1M_2);
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC2M_0);
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC2M_1);
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC2M_2);
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1PE);
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC2PE);
	SET_BIT(TIM3->CCER, TIM_CCER_CC1E);
	SET_BIT(TIM3->CCER, TIM_CCER_CC2E);
	WRITE_REG(TIM3->ARR, 1 << 8);
	// Transfer Initialization to Shadow Register
	SET_BIT(TIM3->EGR, TIM_EGR_UG);
}

void custom_TIM7_Init(void) {
	SET_BIT(TIM7->CR1, TIM_CR1_ARPE);
	SET_BIT(TIM7->CR1, TIM_CR1_URS);
	SET_BIT(TIM7->CR1, TIM_CR1_OPM);
	SET_BIT(TIM7->DIER, TIM_DIER_UIE);
	CLEAR_BIT(TIM7->SR, TIM_SR_UIF);
	WRITE_REG(TIM7->CNT, 0);
	WRITE_REG(TIM7->PSC, 62500 - 1);
	WRITE_REG(TIM7->ARR, 50); //Set initial tone duration to roughly 0.2s

	//Set initial volume in the middle
	WRITE_REG(TIM2->CCR3, 1 << (4 - 1));
}

void TIM_LED_PWM_Init(TIM_TypeDef *t, int ccmr_num) {
	// PWM Mode 1

	// Output compare 2, mode 0b0111 : S.568
	uint16_t flags = TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2
			| TIM_CCMR1_OC1PE;
	t->CCMR1 |= flags << (8 * (ccmr_num - 1));
	// Compare register output enable
	t->CCER |= (ccmr_num == 1) ? TIM_CCER_CC1E : TIM_CCER_CC2E;

	t->BDTR |= TIM_BDTR_MOE; // Main Output Enable (S.514)

	t->ARR = 256;     // Up to 256 levels
	t->PSC = 800 - 1; // Optional

	t->EGR |= TIM_EGR_UG; // Clean start + Shadow Register update
}

void custom_DigitalOutput_Init() {
	//Init GPIOs for PWM
	//Red LED
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE4_0);
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE4_1);

	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL4_Msk;
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL4_0 * 0b0010;
	TIM_LED_PWM_Init(TIM3, 1);

	//Green
	CLEAR_BIT(GPIOC->MODER, GPIO_MODER_MODE7_0);
	SET_BIT(GPIOC->MODER, GPIO_MODER_MODE7_1);

	GPIOC->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;
	GPIOC->AFR[0] |= GPIO_AFRL_AFSEL7_0 * 0b0010;
	TIM_LED_PWM_Init(TIM3, 2);

	//Blue
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE9_0);
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE9_1);

	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9_Msk;
	GPIOA->AFR[1] |= GPIO_AFRH_AFSEL9_0 * 0b0001;
	TIM_LED_PWM_Init(TIM1, 2);

	//Speaker
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE10_0);
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE10_1);
	SET_BIT(GPIOB->AFR[1], GPIO_AFRH_AFRH2_0);
	CLEAR_BIT(GPIOB->AFR[1], GPIO_AFRH_AFRH2_1);
	CLEAR_BIT(GPIOB->AFR[1], GPIO_AFRH_AFRH2_2);
	CLEAR_BIT(GPIOB->AFR[1], GPIO_AFRH_AFRH2_3);
}

void custom_GPIO(void) {
	CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE10_0);
	SET_BIT(GPIOB->MODER, GPIO_MODER_MODE10_1);
	SET_BIT(GPIOB->AFR[1], GPIO_AFRH_AFRH2_0);
	CLEAR_BIT(GPIOB->AFR[1], GPIO_AFRH_AFRH2_1);
	CLEAR_BIT(GPIOB->AFR[1], GPIO_AFRH_AFRH2_2);
	CLEAR_BIT(GPIOB->AFR[1], GPIO_AFRH_AFRH2_3);
	//Initialize the Red LED, with removed AF
	GPIOB->MODER &= ~GPIO_MODER_MODE4_1;
	SET_BIT(GPIOB_handle->MODER, GPIO_MODER_MODE4_0);
	CLEAR_BIT(GPIOB_handle->MODER, GPIO_MODER_MODE4_1);

}

void custom_USART2_Init(USART_TypeDef *USARTx) {
	// PA2 USART2
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE2_0);
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE2_1);
	// PA3 USART2
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE3_0);
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE3_1);

	// Alternate function AF7 PA2 and PA3
	MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL2_Msk, 0x7 << GPIO_AFRL_AFSEL2_Pos);
	MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL3_Msk, 0x7 << GPIO_AFRL_AFSEL3_Pos);

	// Set Speed high on PA2 and PA3
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED2_Msk, 0x3 << GPIO_OSPEEDR_OSPEED2_Pos);
	MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED3_Msk, 0x3 << GPIO_OSPEEDR_OSPEED3_Pos);

	// PA2 00 := No pull−up/ pull−down 01 := pull−up
	SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD2_0);
	CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD2_1);

	// PA3 00 := No pull−up/ pull−down
	SET_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD3_0);
	CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD3_1);

	// Output type PA2 open−drain
	SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT2);

	// Output type PA3 open−drain
	SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT3);

	//Disable Usart
	CLEAR_BIT(USARTx->CR1, USART_CR1_UE);
	//SET data length to 8
	CLEAR_BIT(USARTx->CR1, USART_CR1_M);
	//Select 1 stop bit
	CLEAR_BIT(USARTx->CR2, USART_CR2_STOP_0);
	CLEAR_BIT(USARTx->CR2, USART_CR2_STOP_1);
	//Set Parity control to no parity
	CLEAR_BIT(USARTx->CR1, USART_CR1_PCE);
	//Set oversampling to 16
	CLEAR_BIT(USARTx->CR1, USART_CR1_OVER8);
	//Set Baud RAte 115200 on a 16MHz ref clock
	USART2->BRR = 0x45;
	//Enable Transmisson and reception
	SET_BIT(USARTx->CR1, (USART_CR1_TE | USART_CR1_RE));
	//Enable USART
	SET_BIT(USARTx->CR1, USART_CR1_UE);
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

void custom_NVIC_Init(void) {
	//Interrupt USART2
	NVIC_SetPriority(USART2_IRQn, 2);
	NVIC_ClearPendingIRQ(USART2_IRQn);
	NVIC_EnableIRQ(USART2_IRQn);

	//Interrupt TIM7
	NVIC_SetPriority(TIM7_IRQn, 1);
	NVIC_ClearPendingIRQ(TIM7_IRQn);
	NVIC_EnableIRQ(TIM7_IRQn);
}

void startNote(unsigned char c) {
#define A0_OFFSET (21)
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
