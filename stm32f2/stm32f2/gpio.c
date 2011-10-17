/*
 * Copyright (c) 2011, ShareBrained Technology, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * o Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * o Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "gpio.h"
#include "bitband.h"

void gpio_set_mode(gpio_pin_t* const pin, const gpio_moder_t mode) {
	const uint_fast8_t base_bit = pin->number * 2;
	const uint32_t mask = ((1 << 2) - 1) << base_bit;
	pin->port->MODER = (pin->port->MODER & ~mask) | (mode << base_bit);
}

void gpio_set_output_type(gpio_pin_t* const pin, const gpio_otyper_t type) {
	bitband_set(BITBAND_PERIPHERAL, &pin->port->OTYPER, pin->number, type);
}

void gpio_set_output_speed(gpio_pin_t* const pin, const gpio_ospeedr_t speed) {
	const uint_fast8_t base_bit = pin->number * 2;
	const uint32_t mask = ((1 << 2) - 1) << base_bit;
	pin->port->OSPEEDR = (pin->port->OSPEEDR & ~mask) | (speed << base_bit);
}

void gpio_set_alternate_function(gpio_pin_t* const pin, const gpio_af_t alternate_function) {
	const uint_fast8_t base_bit = (pin->number & 0x7) * 4;
	const uint32_t mask = ((1 << 4) - 1) << base_bit;
	volatile uint32_t* const p = (pin->number & 0x8) ? &pin->port->AFRH : &pin->port->AFRL;
	*p = (*p & ~mask) | (alternate_function << base_bit);
}

void gpio_set_output(gpio_pin_t* const pin, const uint_fast8_t value) {
	bitband_set(BITBAND_PERIPHERAL, &pin->port->ODR, pin->number, value);
}
