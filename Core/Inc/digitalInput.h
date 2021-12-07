/**
 * @file digitalInput.h
 * @author wlm2
 * @date 22.11.2021
 * @brief This header file defines the 'struct DigitalInput' and several
 * functions and macros to access 'struct DigitalInput'.
 */

#ifndef INC_DIGITALINPUT_H_
#define INC_DIGITALINPUT_H_

#include "stm32f4xx.h"
#include "stm32f446xx.h"

/**
 * @struct DigitalInput digitalInput.h
 * @brief Struct to hold filter data for a GPIO-Pin.
 *
 */
struct DigitalInput {
	GPIO_TypeDef *port;
	uint32_t pin;
	uint32_t status;
	uint32_t filterThreshold;
	uint32_t filterCount;
	void (*callbackRisingEdge)(struct DigitalInput *self);
	void (*callbackFallingEdge)(struct DigitalInput *self);
};

/**
 * @brief Initialization of 'struct DigitalInput'.
 *
 * Initialize all members of 'struct DigitalInput' with its default value.
 */
void DigitalInput_Init(struct DigitalInput *self, GPIO_TypeDef *port, uint32_t pin);


/**
 * @brief Samples the GPIO-Pin and executes callback function on an edge event.
 *
 * A filter allows to wait N samples after a change of the pin state to accept
 * the new state represented by 'status'.
 * On change of 'status' a callback function is triggered depending on the kind
 * of edge event.
 * If a callback function is initialized with NULL, it is not called.
 */
void DigitalInput_Sample(struct DigitalInput *self);

#endif /* INC_DIGITALINPUT_H_ */
