/*
 * custom_init.h
 *
 *  Created on: 22.12.2021
 *      Author: sebas
 */

#ifndef INC_CUSTOM_INIT_H_
#define INC_CUSTOM_INIT_H_

#include "main.h"

#include <stdlib.h>   /* For EXIT_SUCCESS */
#include <stdbool.h>  /* For true  */
#include <ctype.h> /* For toupper */

void custom_GPIO(void);
void custom_TIM1_Init(void);
void custom_TIM2_Init(void);
void custom_TIM3_Init(void);
void custom_TIM7_Init(void);
void custom_USART2_Init(USART_TypeDef *USARTx);
void custom_NVIC_Init(void);
void custom_DigitalOutput_Init(void);
void TIM_LED_PWM_Init(TIM_TypeDef *t, int ccmr_num);



#endif /* INC_CUSTOM_INIT_H_ */
