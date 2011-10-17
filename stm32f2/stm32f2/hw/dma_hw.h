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

#ifndef STM32F2_DMA_HW_H_
#define STM32F2_DMA_HW_H_

#include <stdint.h>

typedef struct dma_stream_t {
	volatile uint32_t CR;
	volatile uint32_t NDTR;
	volatile uint32_t PAR;
	volatile uint32_t M0AR;
	volatile uint32_t M1AR;
	volatile uint32_t FCR;
} dma_stream_t;

typedef struct dma_t {
	volatile uint32_t LISR;
	volatile uint32_t HISR;
	volatile uint32_t LIFCR;
	volatile uint32_t HIFCR;
	dma_stream_t S[8];
} dma_t;

///////////////////////////////////////////////////////////////////

typedef enum dma_stream_cr_base_t {
	DMA_STREAM_CR_DIR_BASE		=  6,
	DMA_STREAM_CR_PSIZE_BASE	= 11,
	DMA_STREAM_CR_MSIZE_BASE	= 13,
	DMA_STREAM_CR_PL_BASE		= 16,
	DMA_STREAM_CR_PBURST_BASE	= 21,
	DMA_STREAM_CR_MBURST_BASE	= 23,
	DMA_STREAM_CR_CHSEL_BASE	= 25,
} dma_stream_cr_base_t;

typedef enum dma_stream_cr_bit_t {
	DMA_STREAM_CR_EN_BIT		=  0,
	DMA_STREAM_CR_DMEIE_BIT		=  1,
	DMA_STREAM_CR_TEIE_BIT		=  2,
	DMA_STREAM_CR_HTIE_BIT		=  3,
	DMA_STREAM_CR_TCIE_BIT		=  4,
	DMA_STREAM_CR_PFCTRL_BIT	=  5,
	DMA_STREAM_CR_CIRC_BIT		=  8,
	DMA_STREAM_CR_PINC_BIT		=  9,
	DMA_STREAM_CR_MINC_BIT		= 10,
	DMA_STREAM_CR_PINCOS_BIT	= 15,
	DMA_STREAM_CR_DBM_BIT		= 18,
	DMA_STREAM_CR_CT_BIT		= 19,
} dma_stream_cr_bit_t;

typedef enum dma_stream_cr_mburst_t {
	DMA_STREAM_CR_MBURST_SINGLE_TRANSFER	= 0b00,
	DMA_STREAM_CR_MBURST_INCR4				= 0b01,
	DMA_STREAM_CR_MBURST_INCR8				= 0b10,
	DMA_STREAM_CR_MBURST_INCR16				= 0b11,
} dma_stream_cr_mburst_t;

typedef enum dma_stream_cr_pburst_t {
	DMA_STREAM_CR_PBURST_SINGLE_TRANSFER	= 0b00,
	DMA_STREAM_CR_PBURST_INCR4				= 0b01,
	DMA_STREAM_CR_PBURST_INCR8				= 0b10,
	DMA_STREAM_CR_PBURST_INCR16				= 0b11,
} dma_stream_cr_pburst_t;

typedef enum dma_stream_cr_ct_t {
	DMA_STREAM_CR_CT_MEMORY_0				= 0,
	DMA_STREAM_CR_CT_MEMORY_1				= 1,
} dma_stream_cr_ct_t;

typedef enum dma_stream_cr_dbm_t {
	DMA_STREAM_CR_DBM_OFF					= 0,
	DMA_STREAM_CR_DBM_ON					= 1,
} dma_stream_cr_dbm_t;

typedef enum dma_stream_cr_pl_t {
	DMA_STREAM_CR_PL_LOW					= 0b00,
	DMA_STREAM_CR_PL_MEDIUM					= 0b01,
	DMA_STREAM_CR_PL_HIGH					= 0b10,
	DMA_STREAM_CR_PL_VERY_HIGH				= 0b11,
} dma_stream_cr_pl_t;

typedef enum dma_stream_cr_pincos_t {
	DMA_STREAM_CR_PINCOS_LINKED_TO_PSIZE	= 0,
	DMA_STREAM_CR_PINCOS_FIXED_TO_4			= 1,
} dma_stream_cr_pincos_t;

typedef enum dma_stream_cr_msize_t {
	DMA_STREAM_CR_MSIZE_BYTE				= 0b00,
	DMA_STREAM_CR_MSIZE_HALF_WORD			= 0b01,
	DMA_STREAM_CR_MSIZE_WORD				= 0b10,
} dma_stream_cr_msize_t;

typedef enum dma_stream_cr_psize_t {
	DMA_STREAM_CR_PSIZE_BYTE				= 0b00,
	DMA_STREAM_CR_PSIZE_HALF_WORD			= 0b01,
	DMA_STREAM_CR_PSIZE_WORD				= 0b10,
} dma_stream_cr_psize_t;

typedef enum dma_stream_cr_minc_t {
	DMA_STREAM_CR_MINC_FIXED				= 0,
	DMA_STREAM_CR_MINC_INCREMENT			= 1,
} dma_stream_cr_minc_t;

typedef enum dma_stream_cr_pinc_t {
	DMA_STREAM_CR_PINC_FIXED				= 0,
	DMA_STREAM_CR_PINC_INCREMENT			= 1,
} dma_stream_cr_pinc_t;

typedef enum dma_stream_cr_circ_t {
	DMA_STREAM_CR_CIRC_DISABLED				= 0,
	DMA_STREAM_CR_CIRC_ENABLED				= 1,
} dma_stream_cr_circ_t;

typedef enum dma_stream_cr_dir_t {
	DMA_STREAM_CR_DIR_PERIPHERAL_TO_MEMORY	= 0b00,
	DMA_STREAM_CR_DIR_MEMORY_TO_PERIPHERAL	= 0b01,
	DMA_STREAM_CR_DIR_MEMORY_TO_MEMORY		= 0b10,
} dma_stream_cr_dir_t;

typedef enum dma_stream_cr_pfctrl_t {
	DMA_STREAM_CR_PFCTRL_DMA				= 0,
	DMA_STREAM_CR_PFCTRL_PERIPHERAL			= 1,
} dma_stream_cr_pfctrl_t;

typedef enum dma_stream_cr_en_t {
	DMA_STREAM_CR_EN_DISABLED				= 0,
	DMA_STREAM_CR_EN_ENABLED				= 1,
} dma_stream_cr_en_t;

///////////////////////////////////////////////////////////////////

typedef enum dma_stream_fcr_base_t {
	DMA_STREAM_FCR_FTH_BASE		=  0,
	DMA_STREAM_FCR_FS_BASE		=  3,
} dma_stream_fcr_base_t;

typedef enum dma_stream_fcr_bit_t {
	DMA_STREAM_FCR_DMDIS_BIT	=  2,
	DMA_STREAM_FCR_FEIE_BIT		=  7,
} dma_stream_fcr_bit_t;

typedef enum dma_stream_fcr_dmdis_t {
	DMA_STREAM_FCR_DMDIS_DIRECT_MODE_ENABLED	= 0,
	DMA_STREAM_FCR_DMDIS_DIRECT_MODE_DISABLED	= 1,
} dma_stream_fcr_dmdis_t;

typedef enum dma_stream_fcr_fth_t {
	DMA_STREAM_FCR_FTH_1_4_FULL_FIFO	= 0b00 << DMA_STREAM_FCR_FTH_BASE,
	DMA_STREAM_FCR_FTH_1_2_FULL_FIFO	= 0b01 << DMA_STREAM_FCR_FTH_BASE,
	DMA_STREAM_FCR_FTH_3_4_FULL_FIFO	= 0b10 << DMA_STREAM_FCR_FTH_BASE,
	DMA_STREAM_FCR_FTH_FULL_FIFO		= 0b11 << DMA_STREAM_FCR_FTH_BASE,
} dma_stream_fcr_fth_t;

///////////////////////////////////////////////////////////////////

#define DMA1_BASE ((dma_t*)0x40026000)
#define DMA2_BASE ((dma_t*)0x40026400)

extern dma_t* const DMA1;
extern dma_t* const DMA2;

#endif /* STM32F2_DMA_HW_H_ */
