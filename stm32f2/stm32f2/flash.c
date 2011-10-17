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

#include "flash.h"
#include "hw/flash_hw.h"

#include "bitband.h"

void flash_enable_prefetch() {
	bitband_set(BITBAND_PERIPHERAL, &FLASH->ACR, FLASH_ACR_PRFTEN_BIT, 1);
}

void flash_enable_instruction_cache() {
	bitband_set(BITBAND_PERIPHERAL, &FLASH->ACR, FLASH_ACR_ICEN_BIT, 1);
}

void flash_enable_data_cache() {
	bitband_set(BITBAND_PERIPHERAL, &FLASH->ACR, FLASH_ACR_DCEN_BIT, 1);
}

void flash_set_latency(const uint_fast8_t wait_states) {
	const uint32_t mask = ((1 << 3) - 1) << FLASH_ACR_LATENCY_BASE;
	FLASH->ACR = (FLASH->ACR & ~mask) | (wait_states << FLASH_ACR_LATENCY_BASE);
}
