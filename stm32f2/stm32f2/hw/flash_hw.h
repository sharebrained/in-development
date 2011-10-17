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

#ifndef STM32F2_FLASH_HW_H_
#define STM32F2_FLASH_HW_H_

#include <stdint.h>

typedef struct flash_t {
	volatile uint32_t ACR;
} flash_t;

///////////////////////////////////////////////////////////////////

typedef enum flash_acr_base_t {
	FLASH_ACR_LATENCY_BASE		=  0,
} flash_acr_base_t;

typedef enum flash_acr_bit_t {
	FLASH_ACR_PRFTEN_BIT		=  8,
	FLASH_ACR_ICEN_BIT			=  9,
	FLASH_ACR_DCEN_BIT			= 10,
	FLASH_ACR_ICRST_BIT			= 11,
	FLASH_ACR_DCRST_BIT			= 12,
} flash_acr_bit_t;

///////////////////////////////////////////////////////////////////

#define FLASH_BASE ((flash_t*)0x40023C00)
extern flash_t* const FLASH;

#endif /* STM32F2_FLASH_HW_H_ */
