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

#ifndef STM32F2_I2C_H_
#define STM32F2_I2C_H_

#include <stdint.h>
#include "hw/i2c_hw.h"

typedef enum i2c_interface_mode_t {
	I2C_INTERFACE_MODE_I2C 		= I2C_CR1_SMBUS_I2C_MODE,
	I2C_INTERFACE_MODE_SMBUS	= I2C_CR1_SMBUS_SMBUS_MODE,
} i2c_interface_mode_t;

void i2c_enable(i2c_t* const i2c);
void i2c_disable(i2c_t* const i2c);

void i2c_set_peripheral_clock_frequency(i2c_t* const i2c, const uint_fast8_t frequency);
void i2c_set_interface_mode(i2c_t* const i2c, const i2c_interface_mode_t interface_mode);
void i2c_disable_packet_error_checking(i2c_t* const i2c);
void i2c_disable_general_call(i2c_t* const i2c);
void i2c_enter_standard_mode(i2c_t* const i2c, const uint_fast16_t ccr, const uint_fast8_t trise);

void i2c_start(i2c_t* const i2c);
void i2c_wait_for_start(i2c_t* const i2c);
void i2c_send(i2c_t* const i2c, const uint_fast8_t data);
void i2c_wait_for_address(i2c_t* const i2c);
void i2c_wait_for_transmit_empty(i2c_t* const i2c);
void i2c_stop(i2c_t* const i2c);

#endif /* STM32F2_I2C_H_ */
