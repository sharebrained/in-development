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

#include "wm8731.h"

void wm8731_write(wm8731_t* const codec, const uint_fast8_t b1, const uint_fast8_t b2) {
	i2c_start(codec->i2c);
	i2c_wait_for_start(codec->i2c);

	i2c_send(codec->i2c, 0x34);
	i2c_wait_for_address(codec->i2c);

	////while( i2c_audio_codec->SR1.TXE != 1 );

	i2c_send(codec->i2c, b1);
	i2c_wait_for_transmit_empty(codec->i2c);

	i2c_send(codec->i2c, b2);
	i2c_wait_for_transmit_empty(codec->i2c);

	i2c_stop(codec->i2c);
}

void wm8731_write_register(wm8731_t* const codec, const uint_fast8_t register_address, const uint_fast16_t value) {
	const uint_fast8_t b1 = (register_address << 1) | (value >> 8);
	const uint_fast8_t b2 = value & 0xFF;
	wm8731_write(codec, b1, b2);
}

void wm8731_set_line_in(wm8731_t* const codec, const uint_fast8_t mute, const uint_fast8_t volume) {
	const uint_fast16_t write_value = (1 << 8) | (mute << 7) | (volume << 0);
	wm8731_write_register(codec, 0b0000000, write_value);
}

void wm8731_set_headphone_out(wm8731_t* const codec, const uint_fast8_t zero_cross_detect,
		                                  const uint_fast8_t volume) {
	const uint_fast16_t write_value = (1 << 8) | (zero_cross_detect << 7) |
			                          (volume << 0);
	wm8731_write_register(codec, 0b0000010, write_value);
}

void wm8731_set_analog_audio_path_control(wm8731_t* const codec, const uint_fast8_t sideatt,
											  const uint_fast8_t sidetone,
											  const uint_fast8_t dacsel,
											  const uint_fast8_t bypass,
											  const uint_fast8_t insel,
											  const uint_fast8_t mutemic,
											  const uint_fast8_t micboost) {
	const uint_fast16_t write_value = (sideatt << 6) | (sidetone << 5) |
									  (dacsel << 4) | (bypass << 3) |
									  (insel << 2) | (mutemic << 1) |
									  (micboost << 0);
	wm8731_write_register(codec, 0b0000100, write_value);
}
void wm8731_set_digital_audio_path_control(wm8731_t* const codec, const uint_fast8_t hpor,
													   const uint_fast8_t dacmu,
													   const uint_fast8_t deemp,
													   const uint_fast8_t adchpd) {
	const uint_fast16_t write_value = (hpor << 4) | (dacmu << 3) | (deemp << 1) | (adchpd << 0);
	wm8731_write_register(codec, 0b0000101, write_value);
}

void wm8731_set_power_down_control(wm8731_t* const codec, const uint_fast8_t poweroff,
											   const uint_fast8_t clkoutpd,
											   const uint_fast8_t oscpd,
											   const uint_fast8_t outpd,
											   const uint_fast8_t dacpd,
											   const uint_fast8_t adcpd,
											   const uint_fast8_t micpd,
											   const uint_fast8_t lineinpd) {
	const uint_fast16_t write_value = (poweroff << 7) | (clkoutpd << 6) | (oscpd << 5) |
			                          (outpd << 4) | (dacpd << 3) | (adcpd << 2) |
			                          (micpd << 1) | (lineinpd << 0);
	wm8731_write_register(codec, 0b0000110, write_value);
}

void wm8731_set_digital_audio_interface_format(wm8731_t* const codec, const uint_fast8_t bclkinv,
														   const uint_fast8_t ms,
														   const uint_fast8_t lrswap,
														   const uint_fast8_t lrp,
														   const uint_fast8_t iwl,
														   const uint_fast8_t format) {
	const uint_fast16_t write_value = (bclkinv << 7) | (ms << 6) | (lrswap << 5) |
			                          (lrp << 4) | (iwl << 2) | (format << 0);
	wm8731_write_register(codec, 0b0000111, write_value);
}

void wm8731_set_sampling_control(wm8731_t* const codec, const uint_fast8_t clkodiv2,
											 const uint_fast8_t clkidiv2,
											 const uint_fast8_t sr,
											 const uint_fast8_t bosr,
											 const uint_fast8_t usb_normal) {
	const uint_fast16_t write_value = (clkodiv2 << 7) | (clkidiv2 << 6) |
			                          (sr << 2) | (bosr << 1) | (usb_normal << 0);
	wm8731_write_register(codec, 0b0001000, write_value);
}

void wm8731_set_active_control(wm8731_t* const codec, const uint_fast8_t active) {
	wm8731_write_register(codec, 0b0001001, active << 0);
}

void wm8731_reset(wm8731_t* const codec) {
	wm8731_write_register(codec, 0b0001111, 0b000000000);
}

wm8731_t wm8731 = {
	.i2c = I2C1_BASE
};
