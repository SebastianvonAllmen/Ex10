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
#include "digitalInput.h"
#include "chromatic_scale.h"
#include "color.h"

#include <stdlib.h>   /* For EXIT_SUCCESS */
#include <stdbool.h>  /* For true  */

void SystemClock_Config(void);

struct DigitalInput Up;
struct DigitalInput Down;
struct DigitalInput Left;
struct DigitalInput Right;

GPIO_TypeDef *GPIOA_handle = (GPIO_TypeDef*) GPIOA_BASE;
GPIO_TypeDef *GPIOB_handle = (GPIO_TypeDef*) GPIOB_BASE;
GPIO_TypeDef *GPIOC_handle = (GPIO_TypeDef*) GPIOC_BASE;

//Function Defs
void custom_DigitalInput_Init();
void custom_TIM6_Init(void);
void custom_NVIC_Init(void);
void custom_GPIO(void);
void toggle_RED_LED(struct DigitalInput *self);
void custom_TIM2_Init();
void note_Handler(struct DigitalInput *self);
void volume_Handler(struct DigitalInput *self);
void custom_TIM1_Init(void);
void custom_TIM3_Init(void);
void led_Handler(const uint8_t colors[NUMBER_OF_CHANELS]);
void TIM_LED_PWM_Init(TIM_TypeDef *t, int ccmr_num);

#define MAX_VOLUME 8
#define MIN_VOLUME 0

int main(void) {
	/* System Config */
	HAL_Init();
	SystemClock_Config();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_TIM6_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM1_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_SYSCFG_CLK_ENABLE();

	//Inits
	custom_DigitalInput_Init();
	custom_TIM6_Init();
	custom_NVIC_Init();
	custom_GPIO();
	custom_TIM2_Init();
	custom_TIM3_Init();
	custom_TIM1_Init();


	SET_BIT(TIM6->CR1, TIM_CR1_CEN);
	SET_BIT(TIM2->CR1, TIM_CR1_CEN);
	SET_BIT(TIM1->CR1, TIM_CR1_CEN);
	SET_BIT(TIM3->CR1, TIM_CR1_CEN);

	while (true) {

	}

	return EXIT_SUCCESS;
}



void led_Handler(const uint8_t colors[NUMBER_OF_CHANELS]){
	TIM3->CCR1 = colors[RED]; // R
	TIM3->CCR2 = colors[GREEN]; // G
	TIM1->CCR2 = colors[BLUE]; // B
}

void note_Handler(struct DigitalInput *self) {
	const uint32_t length =  sizeof(NOTES_PSC_16MHz)/sizeof(uint16_t);
	static uint32_t note = length/2;

	if (self == &Right) {
		if (note < length)
			++note;
	}

	if (self == &Left) {
		if (note > 0)
			--note;
	}

	WRITE_REG(TIM2->PSC, NOTES_PSC_16MHz[note]);
	led_Handler(COLORS[note % 12]);
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

void volume_Handler(struct DigitalInput *self) {
	static uint32_t volume = 0;

	if (self == &Up) {
		if (volume < MAX_VOLUME )
			++volume;
	}

	if(self == &Down) {
		if (volume > MIN_VOLUME )
			--volume;
	}

	WRITE_REG(TIM2->CCR3, 1 << (volume - 1));
}

void TIM6_DAC_IRQHandler(void) {
	DigitalInput_Sample(&Up);
	DigitalInput_Sample(&Down);
	DigitalInput_Sample(&Left);
	DigitalInput_Sample(&Right);
}

void toggle_RED_LED(struct DigitalInput *self) {
	if (READ_BIT(GPIOB->ODR, GPIO_ODR_OD4))
		CLEAR_BIT(GPIOB->ODR, GPIO_ODR_OD4);
	else
		SET_BIT(GPIOB->ODR, GPIO_ODR_OD4);
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

void custom_DigitalInput_Init() {
	DigitalInput_Init(&Up, GPIOA_handle, 4);
	DigitalInput_Init(&Down, GPIOB_handle, 0);
	DigitalInput_Init(&Left, GPIOC_handle, 1);
	DigitalInput_Init(&Right, GPIOC_handle, 0);
	Up.callbackRisingEdge = volume_Handler;
	Down.callbackRisingEdge = volume_Handler;
	Right.callbackRisingEdge = note_Handler;
	Left.callbackRisingEdge = note_Handler;

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
}

void TIM_LED_PWM_Init(TIM_TypeDef *t, int ccmr_num)
{
	// PWM Mode 1

	// Output compare 2, mode 0b0111 : S.568
	uint16_t flags = TIM_CCMR1_OC1M_0
			| TIM_CCMR1_OC1M_1
			| TIM_CCMR1_OC1M_2
			| TIM_CCMR1_OC1PE;
	t->CCMR1 |= flags << (8 * (ccmr_num - 1));
	// Compare register output enable
	t->CCER |= (ccmr_num == 1) ? TIM_CCER_CC1E : TIM_CCER_CC2E;

	t->BDTR |= TIM_BDTR_MOE; // Main Output Enable (S.514)

	t->ARR = 256;     // Up to 256 levels
	t->PSC = 800 - 1; // Optional

	t->EGR |= TIM_EGR_UG; // Clean start + Shadow Register update
}

void custom_TIM6_Init(void) {
	SET_BIT(TIM6->CR1, TIM_CR1_ARPE);
	SET_BIT(TIM6->CR1, TIM_CR1_URS);
	SET_BIT(TIM6->DIER, TIM_DIER_UIE);
	CLEAR_BIT(TIM6->SR, TIM_SR_UIF);
	WRITE_REG(TIM6->CNT, 0);
	WRITE_REG(TIM6->PSC, 160 - 1);
	WRITE_REG(TIM6->ARR, 1000 - 1);
}

void custom_NVIC_Init(void) {
	NVIC_SetPriority(TIM6_DAC_IRQn, 2);
	NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
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
