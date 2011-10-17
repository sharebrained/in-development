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

#include "bitband.h"

typedef struct bitband_table_entry_t {
	const uint32_t region_base;
	const uint32_t bit_band_base;
} bitband_table_entry_t;

static const bitband_table_entry_t bitband_table[] = {
	[BITBAND_PERIPHERAL] = { .region_base = 0x40000000, .bit_band_base = 0x42000000 },
	[BITBAND_SRAM]       = { .region_base = 0x20000000, .bit_band_base = 0x22000000 },
};

static volatile uint32_t* bitband_word_address(const bitband_t bitband, volatile void* const address, const uint_fast8_t bit_number) {
	const uint32_t byte_offset = (uint32_t)address - bitband_table[bitband].region_base;
	// From STM32F2xx reference manual, section 2.3.2 "Bit banding"
	return (volatile uint32_t*)(bitband_table[bitband].bit_band_base + (byte_offset * 32) + (bit_number * 4));
}

void bitband_set(const bitband_t bitband, volatile void* const address, const uint_fast8_t bit_number, const uint_fast8_t value) {
	*bitband_word_address(bitband, address, bit_number) = value;
}

uint_fast8_t bitband_get(const bitband_t bitband, volatile void* const address, const uint_fast8_t bit_number) {
	return *bitband_word_address(bitband, address, bit_number);
}
