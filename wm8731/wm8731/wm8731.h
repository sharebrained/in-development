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

#ifndef WM8731_H_
#define WM8731_H_

#include <stdint.h>
#include <stm32f2/i2c.h>

typedef struct wm8731_t {
	i2c_t* const i2c;
} wm8731_t;

void wm8731_write(wm8731_t* const codec, const uint_fast8_t b1, const uint_fast8_t b2);
void wm8731_write_register(wm8731_t* const codec, const uint_fast8_t register_address, const uint_fast16_t value);
void wm8731_set_line_in(wm8731_t* const codec, const uint_fast8_t mute, const uint_fast8_t volume);
void wm8731_set_headphone_out(wm8731_t* const codec, const uint_fast8_t zero_cross_detect,
		                                  const uint_fast8_t volume);
void wm8731_set_analog_audio_path_control(wm8731_t* const codec, const uint_fast8_t sideatt,
											  const uint_fast8_t sidetone,
											  const uint_fast8_t dacsel,
											  const uint_fast8_t bypass,
											  const uint_fast8_t insel,
											  const uint_fast8_t mutemic,
											  const uint_fast8_t micboost);
void wm8731_set_digital_audio_path_control(wm8731_t* const codec, const uint_fast8_t hpor,
													   const uint_fast8_t dacmu,
													   const uint_fast8_t deemp,
													   const uint_fast8_t adchpd);
void wm8731_set_power_down_control(wm8731_t* const codec, const uint_fast8_t poweroff,
											   const uint_fast8_t clkoutpd,
											   const uint_fast8_t oscpd,
											   const uint_fast8_t outpd,
											   const uint_fast8_t dacpd,
											   const uint_fast8_t adcpd,
											   const uint_fast8_t micpd,
											   const uint_fast8_t lineinpd);
void wm8731_set_digital_audio_interface_format(wm8731_t* const codec, const uint_fast8_t bclkinv,
														   const uint_fast8_t ms,
														   const uint_fast8_t lrswap,
														   const uint_fast8_t lrp,
														   const uint_fast8_t iwl,
														   const uint_fast8_t format);
void wm8731_set_sampling_control(wm8731_t* const codec, const uint_fast8_t clkodiv2,
											 const uint_fast8_t clkidiv2,
											 const uint_fast8_t sr,
											 const uint_fast8_t bosr,
											 const uint_fast8_t usb_normal);
void wm8731_set_active_control(wm8731_t* const codec, const uint_fast8_t active);
void wm8731_reset(wm8731_t* const codec);

extern wm8731_t wm8731;

#endif /* WM8731_H_ */
