/**
 * @file digitalInput.c
 * @author wlm2
 * @date 22.11.2021
 * @brief This header file defines the 'struct DigitalInput' and several
 * functions and macros to access 'struct DigitalInput'.
 */

#include "digitalInput.h"

void DigitalInput_Init(struct DigitalInput *self, GPIO_TypeDef *port,
		uint32_t pin) {
	self->port = port;
	self->pin = 1 << pin;
	self->status = READ_BIT(self->port->IDR, self->pin);
	self->filterThreshold = 0;
	self->filterCount = 0;
	self->callbackRisingEdge = NULL;
	self->callbackFallingEdge = NULL;
}

void DigitalInput_Sample(struct DigitalInput *self) {
	if ((! READ_BIT(self ->port ->IDR , self ->pin)) != self ->status) {
		self->filterCount++;
		if (self ->filterCount > self ->filterThreshold) {
			if (self->status) {
				self->status = 0;
				if (self->callbackFallingEdge)
					self->callbackFallingEdge(self);
			} else {
				self->status = 1;
				if (self->callbackRisingEdge)
					self->callbackRisingEdge(self);
			}
			self->filterCount = 0;
		}
	}
}
