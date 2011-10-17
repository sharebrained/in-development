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

#include "rcc.h"
#include "bitband.h"

static const bus_table_entry_t bus_table[] = {
	[BUS_AHB1]	= { .ENR = &RCC_BASE->AHB1ENR, .RSTR = &RCC_BASE->AHB1RSTR },
	[BUS_AHB2]	= { .ENR = &RCC_BASE->AHB2ENR, .RSTR = &RCC_BASE->AHB2RSTR },
	[BUS_AHB3]	= { .ENR = &RCC_BASE->AHB3ENR, .RSTR = &RCC_BASE->AHB3RSTR },
	[BUS_APB1]	= { .ENR = &RCC_BASE->APB1ENR, .RSTR = &RCC_BASE->APB1RSTR },
	[BUS_APB2]	= { .ENR = &RCC_BASE->APB2ENR, .RSTR = &RCC_BASE->APB2RSTR },
};

static const peripheral_table_entry_t peripheral_table[] = {
	[PERIPHERAL_GPIOA]	= { .bus = BUS_AHB1, .enable_bit = RCC_AHB1ENR_GPIOAEN_BIT, .reset_bit = RCC_AHB1RSTR_GPIOARST_BIT },
	[PERIPHERAL_GPIOB]	= { .bus = BUS_AHB1, .enable_bit = RCC_AHB1ENR_GPIOBEN_BIT, .reset_bit = RCC_AHB1RSTR_GPIOBRST_BIT },
	[PERIPHERAL_GPIOC]	= { .bus = BUS_AHB1, .enable_bit = RCC_AHB1ENR_GPIOCEN_BIT, .reset_bit = RCC_AHB1RSTR_GPIOCRST_BIT },
	[PERIPHERAL_GPIOD]	= { .bus = BUS_AHB1, .enable_bit = RCC_AHB1ENR_GPIODEN_BIT, .reset_bit = RCC_AHB1RSTR_GPIODRST_BIT },
	[PERIPHERAL_GPIOE]	= { .bus = BUS_AHB1, .enable_bit = RCC_AHB1ENR_GPIOEEN_BIT, .reset_bit = RCC_AHB1RSTR_GPIOERST_BIT },
	[PERIPHERAL_GPIOF]	= { .bus = BUS_AHB1, .enable_bit = RCC_AHB1ENR_GPIOFEN_BIT, .reset_bit = RCC_AHB1RSTR_GPIOFRST_BIT },
	[PERIPHERAL_GPIOG]	= { .bus = BUS_AHB1, .enable_bit = RCC_AHB1ENR_GPIOGEN_BIT, .reset_bit = RCC_AHB1RSTR_GPIOGRST_BIT },
	[PERIPHERAL_GPIOH]	= { .bus = BUS_AHB1, .enable_bit = RCC_AHB1ENR_GPIOHEN_BIT, .reset_bit = RCC_AHB1RSTR_GPIOHRST_BIT },
	[PERIPHERAL_GPIOI]	= { .bus = BUS_AHB1, .enable_bit = RCC_AHB1ENR_GPIOIEN_BIT, .reset_bit = RCC_AHB1RSTR_GPIOIRST_BIT },
	[PERIPHERAL_SPI2]   = { .bus = BUS_APB1, .enable_bit = RCC_APB1ENR_SPI2EN_BIT,  .reset_bit = RCC_APB1RSTR_SPI2RST_BIT  },
	[PERIPHERAL_SPI3]   = { .bus = BUS_APB1, .enable_bit = RCC_APB1ENR_SPI3EN_BIT,  .reset_bit = RCC_APB1RSTR_SPI3RST_BIT  },
	[PERIPHERAL_I2C1]   = { .bus = BUS_APB1, .enable_bit = RCC_APB1ENR_I2C1EN_BIT,  .reset_bit = RCC_APB1RSTR_I2C1RST_BIT  },
	[PERIPHERAL_I2C2]   = { .bus = BUS_APB1, .enable_bit = RCC_APB1ENR_I2C2EN_BIT,  .reset_bit = RCC_APB1RSTR_I2C2RST_BIT  },
	[PERIPHERAL_I2C3]   = { .bus = BUS_APB1, .enable_bit = RCC_APB1ENR_I2C3EN_BIT,  .reset_bit = RCC_APB1RSTR_I2C3RST_BIT  },
	[PERIPHERAL_DMA1]   = { .bus = BUS_AHB1, .enable_bit = RCC_AHB1ENR_DMA1EN_BIT,  .reset_bit = RCC_AHB1RSTR_DMA1RST_BIT  },
	[PERIPHERAL_DMA2]   = { .bus = BUS_AHB1, .enable_bit = RCC_AHB1ENR_DMA2EN_BIT,  .reset_bit = RCC_AHB1RSTR_DMA2RST_BIT  },
	[PERIPHERAL_SDIO]	= { .bus = BUS_APB2, .enable_bit = RCC_APB2ENR_SDIOEN_BIT,  .reset_bit = RCC_APB2RSTR_SDIORST_BIT  },
};

static const rcc_oscillator_table_entry_t rcc_oscillator_table[] = {
	[RCC_OSCILLATOR_PLL]	= { .CR = &RCC_BASE->CR,   .on_bit = RCC_CR_PLLON_BIT,    .ready_bit = RCC_CR_PLLRDY_BIT    },
	[RCC_OSCILLATOR_HSE]	= { .CR = &RCC_BASE->CR,   .on_bit = RCC_CR_HSEON_BIT,    .ready_bit = RCC_CR_HSERDY_BIT    },
	[RCC_OSCILLATOR_HSI]	= { .CR = &RCC_BASE->CR,   .on_bit = RCC_CR_HSION_BIT,    .ready_bit = RCC_CR_HSIRDY_BIT    },
	[RCC_OSCILLATOR_LSE]	= { .CR = &RCC_BASE->BDCR, .on_bit = RCC_BDCR_LSEON_BIT,  .ready_bit = RCC_BDCR_LSERDY_BIT  },
	[RCC_OSCILLATOR_LSI]	= { .CR = &RCC_BASE->CSR,  .on_bit = RCC_CSR_LSION_BIT,   .ready_bit = RCC_CSR_LSIRDY_BIT   },
	[RCC_OSCILLATOR_PLLI2S]	= { .CR = &RCC_BASE->CR,   .on_bit = RCC_CR_PLLI2SON_BIT, .ready_bit = RCC_CR_PLLI2SRDY_BIT },
};

static volatile uint32_t* rcc_bus_ENR(const bus_t bus) {
	return bus_table[bus].ENR;
}

static volatile uint32_t* rcc_bus_RSTR(const bus_t bus) {
	return bus_table[bus].RSTR;
}

static bus_t rcc_peripheral_bus(const peripheral_t peripheral) {
	return peripheral_table[peripheral].bus;
}

static uint_least8_t rcc_peripheral_enable_bit(const peripheral_t peripheral) {
	return peripheral_table[peripheral].enable_bit;
}

static uint_least8_t rcc_peripheral_reset_bit(const peripheral_t peripheral) {
	return peripheral_table[peripheral].reset_bit;
}

static volatile uint32_t* rcc_oscillator_CR(const rcc_oscillator_t oscillator) {
	return rcc_oscillator_table[oscillator].CR;
}

static uint_least8_t rcc_oscillator_on_bit(const rcc_oscillator_t oscillator) {
	return rcc_oscillator_table[oscillator].on_bit;
}

static uint_least8_t rcc_oscillator_ready_bit(const rcc_oscillator_t oscillator) {
	return rcc_oscillator_table[oscillator].ready_bit;
}

void rcc_set_oscillator_on(const rcc_oscillator_t oscillator, const uint_fast8_t value) {
	bitband_set(BITBAND_PERIPHERAL, rcc_oscillator_CR(oscillator), rcc_oscillator_on_bit(oscillator), value);
}

void rcc_enable_oscillator(const rcc_oscillator_t oscillator) {
	rcc_set_oscillator_on(oscillator, 1);
}

void rcc_disable_oscillator(const rcc_oscillator_t oscillator) {
	rcc_set_oscillator_on(oscillator, 0);
}

void rcc_wait_for_oscillator_ready(const rcc_oscillator_t oscillator) {
	while( bitband_get(BITBAND_PERIPHERAL, rcc_oscillator_CR(oscillator), rcc_oscillator_ready_bit(oscillator)) == 0 );
}

void rcc_set_system_clock(const rcc_system_oscillator_t oscillator) {
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_MASK) | (oscillator << RCC_CFGR_SW_BASE);
}

void rcc_wait_for_system_clock_switch(const rcc_system_oscillator_t oscillator) {
	while( ((RCC->CFGR >> RCC_CFGR_SWS_BASE) & 0x3) != oscillator );
}

void rcc_hse_bypass_off() {
	bitband_set(BITBAND_PERIPHERAL, &RCC->CR, RCC_CR_HSEBYP_BIT, 0);
}

void rcc_css_disable() {
	bitband_set(BITBAND_PERIPHERAL, &RCC->CR, RCC_CR_CSSON_BIT, 0);
}

void rcc_set_peripheral_enable(const peripheral_t peripheral, const uint_fast8_t value) {
	bitband_set(BITBAND_PERIPHERAL, rcc_bus_ENR(rcc_peripheral_bus(peripheral)), rcc_peripheral_enable_bit(peripheral), value);
}

void rcc_enable_peripheral(const peripheral_t peripheral) {
	rcc_set_peripheral_enable(peripheral, 1);
}

void rcc_disable_peripheral(const peripheral_t peripheral) {
	rcc_set_peripheral_enable(peripheral, 0);
}

void rcc_set_peripheral_reset(const peripheral_t peripheral, const uint_fast8_t value) {
	bitband_set(BITBAND_PERIPHERAL, rcc_bus_RSTR(rcc_peripheral_bus(peripheral)), rcc_peripheral_reset_bit(peripheral), value);
}

void rcc_reset_peripheral(const peripheral_t peripheral) {
	rcc_set_peripheral_reset(peripheral, 1);
	rcc_set_peripheral_reset(peripheral, 0);
}

void rcc_set_cfgr(const rcc_cfgr_mco2_t mco2,
				  const rcc_cfgr_mco2pre_t mco2pre,
				  const rcc_cfgr_mco1pre_t mco1pre,
				  const rcc_cfgr_i2ssrc_t i2ssrc,
				  const rcc_cfgr_mco1_t mco1,
				  const uint_fast8_t rtcpre,
				  const uint_fast8_t ppre2,
				  const uint_fast8_t ppre1,
				  const rcc_cfgr_hpre_t hpre,
				  const rcc_cfgr_sw_t sw) {
	RCC->CFGR = (mco2 << RCC_CFGR_MCO2_BASE) |
				(mco2pre << RCC_CFGR_MCO2PRE_BASE) |
				(mco1pre << RCC_CFGR_MCO1PRE_BASE) |
				(i2ssrc << RCC_CFGR_I2SSRC_BIT) |
				(mco1 << RCC_CFGR_MCO1_BASE) |
			    (rtcpre << RCC_CFGR_RTCPRE_BASE) |
			    (ppre2  << RCC_CFGR_PPRE2_BASE) |
			    (ppre1  << RCC_CFGR_PPRE1_BASE) |
			    (hpre << RCC_CFGR_HPRE_BASE) |
			    (sw << RCC_CFGR_SW_BASE);
}

void rcc_set_pllcfgr(const uint_fast8_t pllq,
					 const rcc_pllcfgr_pllsrc_t pllsrc,
					 const rcc_pllcfgr_pllp_t pllp,
					 const uint_fast16_t plln,
					 const uint_fast8_t pllm) {
	RCC->PLLCFGR = (pllq << RCC_PLLCFGR_PLLQ_BASE) |
				   (pllsrc << RCC_PLLCFGR_PLLSRC_BIT) |
			  	   (pllp << RCC_PLLCFGR_PLLP_BASE) |
			  	   (plln << RCC_PLLCFGR_PLLN_BASE) |
			  	   (pllm << RCC_PLLCFGR_PLLM_BASE);
}

void rcc_set_plli2scfgr(const uint_fast8_t plli2sr,
						const uint_fast16_t plli2sn) {
	RCC->PLLI2SCFGR = (plli2sr << RCC_PLLI2SCFGR_PLLI2SR_BASE) |
					  (plli2sn << RCC_PLLI2SCFGR_PLLI2SN_BASE);
}
