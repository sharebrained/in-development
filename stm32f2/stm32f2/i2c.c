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

#include "i2c.h"
#include "bitband.h"

void i2c_enable(i2c_t* const i2c) {
	bitband_set(BITBAND_PERIPHERAL, &i2c->CR1, I2C_CR1_PE_BIT, 1);
}

void i2c_disable(i2c_t* const i2c) {
	bitband_set(BITBAND_PERIPHERAL, &i2c->CR1, I2C_CR1_PE_BIT, 0);
}

void i2c_set_peripheral_clock_frequency(i2c_t* const i2c, const uint_fast8_t frequency) {
	const uint32_t mask = ((1 << I2C_CR2_FREQ_WIDTH) - 1) << I2C_CR2_FREQ_BASE;
	i2c->CR2 = (i2c->CR2 & ~mask) | (frequency << I2C_CR2_FREQ_BASE);
}

void i2c_set_interface_mode(i2c_t* const i2c, const i2c_interface_mode_t interface_mode) {
	bitband_set(BITBAND_PERIPHERAL, &i2c->CR1, I2C_CR1_SMBUS_BIT, interface_mode);
}

void i2c_disable_packet_error_checking(i2c_t* const i2c) {
	bitband_set(BITBAND_PERIPHERAL, &i2c->CR1, I2C_CR1_ENPEC_BIT, 0);
}

void i2c_disable_general_call(i2c_t* const i2c) {
	bitband_set(BITBAND_PERIPHERAL, &i2c->CR1, I2C_CR1_ENGC_BIT, 0);
}

void i2c_enter_standard_mode(i2c_t* const i2c, const uint_fast16_t ccr, const uint_fast8_t trise) {
	i2c->CCR = (I2C_CCR_F_S_STANDARD_MODE << I2C_CCR_F_S_BIT) |
	           (I2C_CCR_DUTY_FAST_MODE_2_1 << I2C_CCR_DUTY_BIT) |
			   (ccr << I2C_CCR_CCR_BASE);
	i2c->TRISE = trise;
}

void i2c_start(i2c_t* const i2c) {
	bitband_set(BITBAND_PERIPHERAL, &i2c->CR1, I2C_CR1_START_BIT, 1);
}

void i2c_wait_for_start(i2c_t* const i2c) {
	while( bitband_get(BITBAND_PERIPHERAL, &i2c->SR1, I2C_SR1_SB_BIT) != I2C_SR1_SB_START_CONDITION_GENERATED );
}

void i2c_send(i2c_t* const i2c, const uint_fast8_t data) {
	i2c->DR = data;
}

void i2c_wait_for_address(i2c_t* const i2c) {
	while( bitband_get(BITBAND_PERIPHERAL, &i2c->SR1, I2C_SR1_ADDR_BIT) != I2C_SR1_ADDR_SENT_OR_MATCHED );
	volatile uint32_t dummy = i2c->SR2;
}

void i2c_wait_for_transmit_empty(i2c_t* const i2c) {
	while( bitband_get(BITBAND_PERIPHERAL, &i2c->SR1, I2C_SR1_TXE_BIT) != I2C_SR1_TXE_DATA_REGISTER_EMPTY );
}

void i2c_stop(i2c_t* const i2c) {
	bitband_set(BITBAND_PERIPHERAL, &i2c->CR1, I2C_CR1_STOP_BIT, 1);
}
