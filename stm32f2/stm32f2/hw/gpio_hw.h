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

#ifndef STM32F2_HW_GPIO_H_
#define STM32F2_HW_GPIO_H_

#include <stdint.h>

typedef struct gpio_t {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
} gpio_t;

///////////////////////////////////////////////////////////////////

typedef enum gpio_moder_t {
	GPIO_MODER_INPUT				= 0b00,
	GPIO_MODER_OUTPUT				= 0b01,
	GPIO_MODER_ALTERNATE_FUNCTION	= 0b10,
	GPIO_MODER_ANALOG				= 0b11,
} gpio_moder_t;

///////////////////////////////////////////////////////////////////

typedef enum gpio_otyper_width_t {
	GPIO_OTYPER_WIDTH				= 1,
} gpio_otyper_width_t;

typedef enum gpio_otyper_base_t {
	GPIO_OTYPER_BASE				= 0,
} gpio_otyper_base_t;

typedef enum gpio_otyper_t {
	GPIO_OTYPER_PUSH_PULL			= 0,
	GPIO_OTYPER_OPEN_DRAIN			= 1,
} gpio_otyper_t;

///////////////////////////////////////////////////////////////////

typedef enum gpio_ospeedr_width_t {
	GPIO_OSPEEDR_WIDTH				= 2,
} gpio_ospeedr_width_t;

typedef enum gpio_ospeedr_base_t {
	GPIO_OSPEEDR_BASE				= 0,
} gpio_ospeedr_base_t;

typedef enum gpio_ospeedr_t {
	GPIO_OSPEEDR_2MHZ				= 0b00,
	GPIO_OSPEEDR_25MHZ				= 0b01,
	GPIO_OSPEEDR_50MHZ				= 0b10,
	GPIO_OSPEEDR_100MHZ				= 0b11,
} gpio_ospeedr_t;

///////////////////////////////////////////////////////////////////

typedef enum gpio_pupdr_width_t {
	GPIO_PUPDR_WIDTH				= 2,
} gpio_pupdr_width_t;

typedef enum gpio_pupdr_base_t {
	GPIO_PUPDR_BASE					= 0,
} gpio_pupdr_base_t;

typedef enum gpio_pupdr_t {
	GPIO_PUPDR_NO_PULL				= 0b00,
	GPIO_PUPDR_PULL_UP				= 0b01,
	GPIO_PUPDR_PULL_DOWN			= 0b10,
} gpio_pupdr_t;

///////////////////////////////////////////////////////////////////

typedef enum gpio_idr_width_t {
	GPIO_IDR_WIDTH					= 1,
} gpio_idr_width_t;

typedef enum gpio_idr_base_t {
	GPIO_IDR_BASE					= 0,
} gpio_idr_base_t;

///////////////////////////////////////////////////////////////////

typedef enum gpio_odr_width_t {
	GPIO_ODR_WIDTH					= 1,
} gpio_odr_width_t;

typedef enum gpio_odr_base_t {
	GPIO_ODR_BASE					= 0,
} gpio_odr_base_t;

///////////////////////////////////////////////////////////////////

typedef enum gpio_bsrr_width_t {
	GPIO_BSRR_BS_WIDTH				= 1,
	GPIO_BSRR_BR_WIDTH				= 1,
} gpio_bsrr_width_t;

typedef enum gpio_bsrr_base_t {
	GPIO_BSRR_BS_BASE				=  0,
	GPIO_BSRR_BR_BASE				= 16,
} gpio_bsrr_base_t;

///////////////////////////////////////////////////////////////////

typedef enum gpio_lckr_width_t {
	GPIO_LCKR_LCK_WIDTH				= 1,
} gpio_lckr_width_t;

typedef enum gpio_lckr_base_t {
	GPIO_LCKR_LCK_BASE				=  0,
} gpio_lckr_base_t;

typedef enum gpio_lckr_bit_t {
	GPIO_LCKR_LCKK_BIT				= 16,
} gpio_lckr_bit_t;

typedef enum gpio_lckr_lck_t {
	GPIO_LCKR_LCK_NOT_LOCKED		= 0,
	GPIO_LCKR_LCK_LOCKED			= 1,
} gpio_lckr_lck_t;

///////////////////////////////////////////////////////////////////

typedef enum gpio_af_width_t {
	GPIO_AF_WIDTH					= 4,
} gpio_af_width_t;

typedef enum gpio_af_t {
	GPIO_AF_SYS						=  0,
	GPIO_AF_TIM1_2					=  1,
	GPIO_AF_TIM3_4_5				=  2,
	GPIO_AF_TIM8_9_10_11			=  3,
	GPIO_AF_I2C1_2_3				=  4,
	GPIO_AF_SPI1_2_I2S2				=  5,
	GPIO_AF_SPI3_I2S3				=  6,
	GPIO_AF_USART1_2_3				=  7,
	GPIO_AF_UART4_5_6				=  8,
	GPIO_AF_CAN1_2_TIM12_13_14		=  9,
	GPIO_AF_OTGFS_OTGHS				= 10,
	GPIO_AF_ETH						= 11,
	GPIO_AF_FSMC_SDIO_OTGFS			= 12,
	GPIO_AF_DCMI					= 13,
	GPIO_AF_14						= 14,
	GPIO_AF_15						= 15,
} gpio_af_t;

///////////////////////////////////////////////////////////////////

#define GPIOA_BASE ((gpio_t*)0x40020000)
#define GPIOB_BASE ((gpio_t*)0x40020400)
#define GPIOC_BASE ((gpio_t*)0x40020800)
#define GPIOD_BASE ((gpio_t*)0x40020C00)
#define GPIOE_BASE ((gpio_t*)0x40021000)
#define GPIOF_BASE ((gpio_t*)0x40021400)
#define GPIOG_BASE ((gpio_t*)0x40021800)
#define GPIOH_BASE ((gpio_t*)0x40021C00)
#define GPIOI_BASE ((gpio_t*)0x40022000)

extern gpio_t* const GPIOA;
extern gpio_t* const GPIOB;
extern gpio_t* const GPIOC;
extern gpio_t* const GPIOD;
extern gpio_t* const GPIOE;
extern gpio_t* const GPIOF;
extern gpio_t* const GPIOG;
extern gpio_t* const GPIOH;
extern gpio_t* const GPIOI;

#endif /* STM32F2_HW_GPIO_H_ */
