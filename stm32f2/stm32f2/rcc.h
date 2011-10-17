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

#ifndef STM32F2_RCC_H_
#define STM32F2_RCC_H_

#include "hw/rcc_hw.h"

typedef enum rcc_oscillator_t {
	RCC_OSCILLATOR_PLL,
	RCC_OSCILLATOR_HSE,
	RCC_OSCILLATOR_HSI,
	RCC_OSCILLATOR_LSE,
	RCC_OSCILLATOR_LSI,
	RCC_OSCILLATOR_PLLI2S,
} rcc_oscillator_t;

typedef struct rcc_oscillator_table_entry_t {
	volatile uint32_t* const CR;
	const uint_least8_t on_bit;
	const uint_least8_t ready_bit;
} rcc_oscillator_table_entry_t;

typedef enum rcc_system_oscillator_t {
	RCC_SYSTEM_OSCILLATOR_HSI = RCC_CFGR_SW_HSI,
	RCC_SYSTEM_OSCILLATOR_HSE = RCC_CFGR_SW_HSE,
	RCC_SYSTEM_OSCILLATOR_PLL = RCC_CFGR_SW_PLL,
} rcc_system_oscillator_t;

typedef enum bus_t {
	BUS_AHB1,
	BUS_AHB2,
	BUS_AHB3,
	BUS_APB1,
	BUS_APB2,
} bus_t;

typedef struct bus_table_entry_t {
	volatile uint32_t* const ENR;
	volatile uint32_t* const RSTR;
} bus_table_entry_t;

typedef enum peripheral_t {
	PERIPHERAL_GPIOA,
	PERIPHERAL_GPIOB,
	PERIPHERAL_GPIOC,
	PERIPHERAL_GPIOD,
	PERIPHERAL_GPIOE,
	PERIPHERAL_GPIOF,
	PERIPHERAL_GPIOG,
	PERIPHERAL_GPIOH,
	PERIPHERAL_GPIOI,
	PERIPHERAL_SPI2,
	PERIPHERAL_SPI3,
	PERIPHERAL_I2C1,
	PERIPHERAL_I2C2,
	PERIPHERAL_I2C3,
	PERIPHERAL_DMA1,
	PERIPHERAL_DMA2,
	PERIPHERAL_SDIO,
} peripheral_t;

typedef struct peripheral_table_entry_t {
	const bus_t bus;
	const uint_least8_t enable_bit;
	const uint_least8_t reset_bit;
} peripheral_table_entry_t;

/*
	void set_rtc_prescaler(const uint_fast8_t value) {
		CFGR = (CFGR & ~(CFGR_RTCPRE_MASK)) | (value << CFGR_RTCPRE_BASE);
	}
*/

void rcc_set_oscillator_on(const rcc_oscillator_t oscillator, const uint_fast8_t value);
void rcc_enable_oscillator(const rcc_oscillator_t oscillator);
void rcc_disable_oscillator(const rcc_oscillator_t oscillator);
void rcc_wait_for_oscillator_ready(const rcc_oscillator_t oscillator);

void rcc_set_system_clock(const rcc_system_oscillator_t oscillator);
void rcc_wait_for_system_clock_switch(const rcc_system_oscillator_t oscillator);

void rcc_hse_bypass_off();
void rcc_css_disable();

void rcc_set_peripheral_enable(const peripheral_t peripheral, const uint_fast8_t value);
void rcc_enable_peripheral(const peripheral_t peripheral);
void rcc_disable_peripheral(const peripheral_t peripheral);

void rcc_set_peripheral_reset(const peripheral_t peripheral, const uint_fast8_t value);
void rcc_reset_peripheral(const peripheral_t peripheral);

void rcc_set_cfgr(const rcc_cfgr_mco2_t mco2,
				  const rcc_cfgr_mco2pre_t mco2pre,
				  const rcc_cfgr_mco1pre_t mco1pre,
				  const rcc_cfgr_i2ssrc_t i2ssrc,
				  const rcc_cfgr_mco1_t mco1,
				  const uint_fast8_t rtcpre,
				  const uint_fast8_t ppre2,
				  const uint_fast8_t ppre1,
				  const rcc_cfgr_hpre_t hpre,
				  const rcc_cfgr_sw_t sw);
void rcc_set_pllcfgr(const uint_fast8_t pllq,
					 const rcc_pllcfgr_pllsrc_t pllsrc,
					 const rcc_pllcfgr_pllp_t pllp,
					 const uint_fast16_t plln,
					 const uint_fast8_t pllm);
void rcc_set_plli2scfgr(const uint_fast8_t plli2sr,
						const uint_fast16_t plli2sn);

#endif /* STM32F2_RCC_H_ */
