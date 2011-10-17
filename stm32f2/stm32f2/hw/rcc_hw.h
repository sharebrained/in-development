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

#ifndef STM32F2_HW_RCC_H_
#define STM32F2_HW_RCC_H_

#include <stdint.h>

typedef struct rcc_t {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t _reserved_0x1C;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t _reserved_0x28;
	volatile uint32_t _reserved_0x2C;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t _reserved_0x3C;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t _reserved_0x48;
	volatile uint32_t _reserved_0x4C;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t _reserved_0x5C;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t _reserved_0x68;
	volatile uint32_t _reserved_0x6C;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t _reserved_0x78;
	volatile uint32_t _reserved_0x7C;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
} rcc_t;

///////////////////////////////////////////////////////////////////
// 0x00 CR

typedef enum rcc_cr_bit_t {
	RCC_CR_HSION_BIT		=  0,
	RCC_CR_HSIRDY_BIT		=  1,
	RCC_CR_HSEON_BIT		= 16,
	RCC_CR_HSERDY_BIT		= 17,
	RCC_CR_HSEBYP_BIT		= 18,
	RCC_CR_CSSON_BIT		= 19,
	RCC_CR_PLLON_BIT		= 24,
	RCC_CR_PLLRDY_BIT		= 25,
	RCC_CR_PLLI2SON_BIT		= 26,
	RCC_CR_PLLI2SRDY_BIT	= 27,
} rcc_cr_bit_t;

///////////////////////////////////////////////////////////////////
// 0x04 PLLCFGR

typedef enum rcc_pllcfgr_bit_t {
	RCC_PLLCFGR_PLLSRC_BIT	= 22,
} rcc_pllcfgr_bit_t;

typedef enum rcc_pllcfgr_base_t {
	RCC_PLLCFGR_PLLM_BASE	=  0,
	RCC_PLLCFGR_PLLN_BASE	=  6,
	RCC_PLLCFGR_PLLP_BASE	= 16,
	RCC_PLLCFGR_PLLQ_BASE	= 24,
} rcc_pllcfgr_base_t;

typedef enum rcc_pllcfgr_pllp_t {
	RCC_PLLCFGR_PLLP_2		= 0b00,
	RCC_PLLCFGR_PLLP_4		= 0b01,
	RCC_PLLCFGR_PLLP_6		= 0b10,
	RCC_PLLCFGR_PLLP_8		= 0b11,
} rcc_pllcfgr_pllp_t;

typedef enum rcc_pllcfgr_pllsrc_t {
	RCC_PLLCFGR_PLLSRC_HSI	= 0,
	RCC_PLLCFGR_PLLSRC_HSE	= 1,
} rcc_pllcfgr_pllsrc_t;

///////////////////////////////////////////////////////////////////
// 0x08 RCC_CFGR

typedef enum rcc_cfgr_bit_t {
	RCC_CFGR_I2SSRC_BIT		= 23,
} rcc_cfgr_bit_t;

typedef enum rcc_cfgr_base_t {
	RCC_CFGR_SW_BASE		=  0,
	RCC_CFGR_SWS_BASE		=  2,
	RCC_CFGR_HPRE_BASE 		=  4,
	RCC_CFGR_PPRE1_BASE 	= 10,
	RCC_CFGR_PPRE2_BASE 	= 13,
	RCC_CFGR_RTCPRE_BASE 	= 16,
	RCC_CFGR_MCO1_BASE 		= 21,
	RCC_CFGR_MCO1PRE_BASE 	= 24,
	RCC_CFGR_MCO2PRE_BASE 	= 27,
	RCC_CFGR_MCO2_BASE 		= 30,
} rcc_cfgr_base_t;

typedef enum rcc_cfgr_sw_t {
	RCC_CFGR_SW_HSI		= 0b00,
	RCC_CFGR_SW_HSE		= 0b01,
	RCC_CFGR_SW_PLL		= 0b10,
	RCC_CFGR_SW_MASK	= 0b11,
} rcc_cfgr_sw_t;

typedef enum rcc_cfgr_sws_t {
	RCC_CFGR_SWS_HSI	= 0b00,
	RCC_CFGR_SWS_HSE	= 0b01,
	RCC_CFGR_SWS_PLL	= 0b10,
	RCC_CFGR_SWS_MASK	= 0b11,
} rcc_cfgr_sws_t;

typedef enum rcc_cfgr_hpre_t {
	RCC_CFGR_HPRE_NOT_DIVIDED		= 0b0000,
	RCC_CFGR_HPRE_DIVIDE_BY_2		= 0b1000,
	RCC_CFGR_HPRE_DIVIDE_BY_4		= 0b1001,
	RCC_CFGR_HPRE_DIVIDE_BY_8		= 0b1010,
	RCC_CFGR_HPRE_DIVIDE_BY_16		= 0b1011,
	RCC_CFGR_HPRE_DIVIDE_BY_64		= 0b1100,
	RCC_CFGR_HPRE_DIVIDE_BY_128		= 0b1101,
	RCC_CFGR_HPRE_DIVIDE_BY_256		= 0b1110,
	RCC_CFGR_HPRE_DIVIDE_BY_512		= 0b1111,
	RCC_CFGR_HPRE_MASK				= 0b1111,
} rcc_cfgr_hpre_t;

typedef enum rcc_cfgr_ppre1_t {
	RCC_CFGR_PPRE1_NOT_DIVIDED		= 0b000,
	RCC_CFGR_PPRE1_DIVIDE_BY_2		= 0b100,
	RCC_CFGR_PPRE1_DIVIDE_BY_4		= 0b101,
	RCC_CFGR_PPRE1_DIVIDE_BY_8		= 0b110,
	RCC_CFGR_PPRE1_DIVIDE_BY_16		= 0b111,
	RCC_CFGR_PPRE1_MASK				= 0b111,
} rcc_cfgr_ppre1_t;

typedef enum rcc_cfgr_ppre2_t {
	RCC_CFGR_PPRE2_NOT_DIVIDED		= 0b000,
	RCC_CFGR_PPRE2_DIVIDE_BY_2		= 0b100,
	RCC_CFGR_PPRE2_DIVIDE_BY_4		= 0b101,
	RCC_CFGR_PPRE2_DIVIDE_BY_8		= 0b110,
	RCC_CFGR_PPRE2_DIVIDE_BY_16		= 0b111,
	RCC_CFGR_PPRE2_MASK				= 0b111,
} rcc_cfgr_ppre2_t;

typedef enum rcc_cfgr_rtcpre_t {
	RCC_CFGR_RTCPRE_NO_CLOCK		= 0b00000,
	RCC_CFGR_RTCPRE_MASK			= 0b11111,
} rcc_cfgr_rtcpre_t;

typedef enum rcc_cfgr_mco1_t {
	RCC_CFGR_MCO1_HSI				= 0b00,
	RCC_CFGR_MCO1_LSE				= 0b01,
	RCC_CFGR_MCO1_HSE				= 0b10,
	RCC_CFGR_MCO1_PLL				= 0b11,
	RCC_CFGR_MCO1_MASK				= 0b11,
} rcc_cfgr_mco1_t;

typedef enum rcc_cfgr_i2ssrc_t {
	RCC_CFGR_I2SSRC_PLLI2S			= 0,
	RCC_CFGR_I2SSRC_EXTERNAL_CKIN	= 1,
} rcc_cfgr_i2ssrc_t;

typedef enum rcc_cfgr_mco1pre_t {
	RCC_CFGR_MCO1PRE_NO_DIVISION	= 0b000,
	RCC_CFGR_MCO1PRE_DIVIDE_BY_2	= 0b100,
	RCC_CFGR_MCO1PRE_DIVIDE_BY_3	= 0b101,
	RCC_CFGR_MCO1PRE_DIVIDE_BY_4	= 0b110,
	RCC_CFGR_MCO1PRE_DIVIDE_BY_5	= 0b111,
	RCC_CFGR_MCO1PRE_MASK			= 0b111,
} rcc_cfgr_mco1pre_t;

typedef enum rcc_cfgr_mco2pre_t {
	RCC_CFGR_MCO2PRE_NO_DIVISION	= 0b000,
	RCC_CFGR_MCO2PRE_DIVIDE_BY_2	= 0b100,
	RCC_CFGR_MCO2PRE_DIVIDE_BY_3	= 0b101,
	RCC_CFGR_MCO2PRE_DIVIDE_BY_4	= 0b110,
	RCC_CFGR_MCO2PRE_DIVIDE_BY_5	= 0b111,
	RCC_CFGR_MCO2PRE_MASK			= 0b111,
} rcc_cfgr_mco2pre_t;

typedef enum rcc_cfgr_mco2_t {
	RCC_CFGR_MCO2_SYSCLK			= 0b00,
	RCC_CFGR_MCO2_PLLI2S			= 0b01,
	RCC_CFGR_MCO2_HSE				= 0b10,
	RCC_CFGR_MCO2_PLL				= 0b11,
	RCC_CFGR_MCO2_MASK				= 0b11,
} rcc_cfgr_mco2_t;

///////////////////////////////////////////////////////////////////
// 0x10 RCC_AHB1RSTR

typedef enum rcc_ahb1rstr_bit_t {
	RCC_AHB1RSTR_GPIOARST_BIT		=  0,
	RCC_AHB1RSTR_GPIOBRST_BIT		=  1,
	RCC_AHB1RSTR_GPIOCRST_BIT		=  2,
	RCC_AHB1RSTR_GPIODRST_BIT		=  3,
	RCC_AHB1RSTR_GPIOERST_BIT		=  4,
	RCC_AHB1RSTR_GPIOFRST_BIT		=  5,
	RCC_AHB1RSTR_GPIOGRST_BIT		=  6,
	RCC_AHB1RSTR_GPIOHRST_BIT		=  7,
	RCC_AHB1RSTR_GPIOIRST_BIT		=  8,
	RCC_AHB1RSTR_CRCRST_BIT			= 12,
	RCC_AHB1RSTR_DMA1RST_BIT		= 21,
	RCC_AHB1RSTR_DMA2RST_BIT		= 22,
	RCC_AHB1RSTR_ETHMACRST_BIT		= 25,
	RCC_AHB1RSTR_OTGHSRST_BIT		= 29,
} rcc_ahb1rstr_bit_t;

///////////////////////////////////////////////////////////////////
// 0x14 RCC_AHB2RSTR

typedef enum rcc_ahb2rstr_bit_t {
	RCC_AHB2RSTR_DCMIRST_BIT		=  0,
	RCC_AHB2RSTR_CRYPRST_BIT		=  4,
	RCC_AHB2RSTR_HSAHRST_BIT		=  5,
	RCC_AHB2RSTR_RNGRST_BIT			=  6,
	RCC_AHB2RSTR_OTGFSRST_BIT		=  7,
} rcc_ahb2rstr_bit_t;

///////////////////////////////////////////////////////////////////
// 0x18 RCC_AHB3RSTR

typedef enum rcc_ahb3rstr_bit_t {
	RCC_AHB3RSTR_FSMCRST_BIT		= 0,
} rcc_ahb3rstr_bit_t;

///////////////////////////////////////////////////////////////////
// 0x20 RCC_APB1ENR

typedef enum rcc_apb1rstr_bit_t {
	RCC_APB1RSTR_TIM2RST_BIT		=  0,
	RCC_APB1RSTR_TIM3RST_BIT		=  1,
	RCC_APB1RSTR_TIM4RST_BIT		=  2,
	RCC_APB1RSTR_TIM5RST_BIT		=  3,
	RCC_APB1RSTR_TIM6RST_BIT		=  4,
	RCC_APB1RSTR_TIM7RST_BIT		=  5,
	RCC_APB1RSTR_TIM12RST_BIT		=  6,
	RCC_APB1RSTR_TIM13RST_BIT		=  7,
	RCC_APB1RSTR_TIM14RST_BIT		=  8,
	RCC_APB1RSTR_WWDGRST_BIT		= 11,
	RCC_APB1RSTR_SPI2RST_BIT		= 14,
	RCC_APB1RSTR_SPI3RST_BIT		= 15,
	RCC_APB1RSTR_USART2RST_BIT		= 17,
	RCC_APB1RSTR_USART3RST_BIT		= 18,
	RCC_APB1RSTR_UART4RST_BIT		= 19,
	RCC_APB1RSTR_UART5RST_BIT		= 20,
	RCC_APB1RSTR_I2C1RST_BIT		= 21,
	RCC_APB1RSTR_I2C2RST_BIT		= 22,
	RCC_APB1RSTR_I2C3RST_BIT		= 23,
	RCC_APB1RSTR_CAN1RST_BIT		= 25,
	RCC_APB1RSTR_CAN2RST_BIT		= 26,
	RCC_APB1RSTR_PWRRST_BIT			= 28,
	RCC_APB1RSTR_DACRST_BIT			= 29,
} rcc_apb1rstr_bit_t;

///////////////////////////////////////////////////////////////////
// 0x24 RCC_APB2RSTR

typedef enum rcc_apb2rstr_bit_t {
	RCC_APB2RSTR_TIM1RST_BIT	=  0,
	RCC_APB2RSTR_TIM8RST_BIT	=  1,
	RCC_APB2RSTR_USART1RST_BIT	=  4,
	RCC_APB2RSTR_USART6RST_BIT	=  5,
	RCC_APB2RSTR_ADCRST_BIT		=  8,
	RCC_APB2RSTR_SDIORST_BIT	= 11,
	RCC_APB2RSTR_SPI1RST_BIT	= 12,
	RCC_APB2RSTR_SYSCFGRST_BIT	= 14,
	RCC_APB2RSTR_TIM9RST_BIT	= 16,
	RCC_APB2RSTR_TIM10RST_BIT	= 17,
	RCC_APB2RSTR_TIM11RST_BIT	= 18,
} rcc_apb2rstr_bit_t;

///////////////////////////////////////////////////////////////////
// 0x30 RCC_AHB1ENR

typedef enum rcc_ahb1enr_bit_t {
	RCC_AHB1ENR_GPIOAEN_BIT			=  0,
	RCC_AHB1ENR_GPIOBEN_BIT			=  1,
	RCC_AHB1ENR_GPIOCEN_BIT			=  2,
	RCC_AHB1ENR_GPIODEN_BIT			=  3,
	RCC_AHB1ENR_GPIOEEN_BIT			=  4,
	RCC_AHB1ENR_GPIOFEN_BIT			=  5,
	RCC_AHB1ENR_GPIOGEN_BIT			=  6,
	RCC_AHB1ENR_GPIOHEN_BIT			=  7,
	RCC_AHB1ENR_GPIOIEN_BIT			=  8,
	RCC_AHB1ENR_CRCEN_BIT			= 12,
	RCC_AHB1ENR_BKPSRAMEN_BIT		= 18,
	RCC_AHB1ENR_DMA1EN_BIT			= 21,
	RCC_AHB1ENR_DMA2EN_BIT			= 22,
	RCC_AHB1ENR_ETHMACEN_BIT		= 25,
	RCC_AHB1ENR_ETHMACTXEN_BIT		= 26,
	RCC_AHB1ENR_ETHMACRXEN_BIT		= 27,
	RCC_AHB1ENR_ETHMACPTPEN_BIT		= 28,
	RCC_AHB1ENR_OTGHSEN_BIT			= 29,
	RCC_AHB1ENR_OTGHSULPIEN_BIT		= 30,
} rcc_ahb1enr_bit_t;

///////////////////////////////////////////////////////////////////
// 0x34 RCC_AHB2ENR

typedef enum rcc_ahb2enr_bit_t {
	RCC_AHB2ENR_DCMIEN_BIT		=  0,
	RCC_AHB2ENR_CRYPEN_BIT		=  4,
	RCC_AHB2ENR_HASHEN_BIT		=  5,
	RCC_AHB2ENR_RNGEN_BIT		=  6,
	RCC_AHB2ENR_OTGFSEN_BIT		=  7,
} rcc_ahb2enr_bit_t;

///////////////////////////////////////////////////////////////////
// 0x38 RCC_AHB3ENR

typedef enum rcc_ahb3enr_bit_t {
	RCC_AHB3ENR_FSMCEN_BIT		= 0,
} rcc_ahb3enr_bit_t;

///////////////////////////////////////////////////////////////////
// 0x40 RCC_APB1ENR

typedef enum rcc_apb1enr_bit_t {
	RCC_APB1ENR_TIM2EN_BIT			=  0,
	RCC_APB1ENR_TIM3EN_BIT			=  1,
	RCC_APB1ENR_TIM4EN_BIT			=  2,
	RCC_APB1ENR_TIM5EN_BIT			=  3,
	RCC_APB1ENR_TIM6EN_BIT			=  4,
	RCC_APB1ENR_TIM7EN_BIT			=  5,
	RCC_APB1ENR_TIM12EN_BIT			=  6,
	RCC_APB1ENR_TIM13EN_BIT			=  7,
	RCC_APB1ENR_TIM14EN_BIT			=  8,
	RCC_APB1ENR_WWDGEN_BIT			= 11,
	RCC_APB1ENR_SPI2EN_BIT			= 14,
	RCC_APB1ENR_SPI3EN_BIT			= 15,
	RCC_APB1ENR_USART2EN_BIT		= 17,
	RCC_APB1ENR_USART3EN_BIT		= 18,
	RCC_APB1ENR_UART4EN_BIT			= 19,
	RCC_APB1ENR_UART5EN_BIT			= 20,
	RCC_APB1ENR_I2C1EN_BIT			= 21,
	RCC_APB1ENR_I2C2EN_BIT			= 22,
	RCC_APB1ENR_I2C3EN_BIT			= 23,
	RCC_APB1ENR_CAN1EN_BIT			= 25,
	RCC_APB1ENR_CAN2EN_BIT			= 26,
	RCC_APB1ENR_PWREN_BIT			= 28,
	RCC_APB1ENR_DACEN_BIT			= 29,
} rcc_apb1enr_bit_t;

///////////////////////////////////////////////////////////////////
// 0x44 RCC_APB2ENR

typedef enum rcc_apb2en_bit_t {
	RCC_APB2ENR_TIM1EN_BIT		=  0,
	RCC_APB2ENR_TIM8EN_BIT		=  1,
	RCC_APB2ENR_USART1EN_BIT	=  4,
	RCC_APB2ENR_USART6EN_BIT	=  5,
	RCC_APB2ENR_ADC1EN_BIT		=  8,
	RCC_APB2ENR_ADC2EN_BIT		=  9,
	RCC_APB2ENR_ADC3EN_BIT		= 10,
	RCC_APB2ENR_SDIOEN_BIT		= 11,
	RCC_APB2ENR_SPI1EN_BIT		= 12,
	RCC_APB2ENR_SYSCFGEN_BIT	= 14,
	RCC_APB2ENR_TIM9EN_BIT		= 16,
	RCC_APB2ENR_TIM10EN_BIT		= 17,
	RCC_APB2ENR_TIM11EN_BIT		= 18,
} rcc_apb2en_bit_t;

///////////////////////////////////////////////////////////////////
// 0x70 RCC_BDCR

typedef enum rcc_bdcr_bit_t {
	RCC_BDCR_LSEON_BIT		=  0,
	RCC_BDCR_LSERDY_BIT		=  1,
	RCC_BDCR_LSEBYP_BIT		=  2,
	RCC_BDCR_RTCEN_BIT		= 15,
	RCC_BDCR_BDRST_BIT		= 16,
} rcc_bdcr_bit_t;

typedef enum rcc_bdcr_base_t {
	RCC_BDCR_RTCSEL_BASE	=  8,
} rcc_bdcr_base_t;

///////////////////////////////////////////////////////////////////
// 0x74 RCC_CSR

typedef enum rcc_csr_bit_t {
	RCC_CSR_LSION_BIT		=  0,
	RCC_CSR_LSIRDY_BIT		=  1,
	RCC_CSR_RMVF_BIT		= 24,
	RCC_CSR_BORRSTF_BIT		= 25,
	RCC_CSR_PINRSTF_BIT		= 26,
	RCC_CSR_PORRSTF_BIT		= 27,
	RCC_CSR_SFTRSTF_BIT		= 28,
	RCC_CSR_IWDGRSTF_BIT	= 29,
	RCC_CSR_WWDGRSTF_BIT	= 30,
	RCC_CSR_LPWRRSTF_BIT	= 31,
} rcc_csr_bit_t;

///////////////////////////////////////////////////////////////////
// 0x84 PLLI2SCFGR

typedef enum rcc_plli2scfgr_base_t {
	RCC_PLLI2SCFGR_PLLI2SN_BASE	=  6,
	RCC_PLLI2SCFGR_PLLI2SR_BASE = 28,
} rcc_plli2scfgr_base_t;

#define RCC_BASE ((rcc_t*)0x40023800)
extern rcc_t* const RCC;

#endif /* STM32F2_HW_RCC_H_ */
