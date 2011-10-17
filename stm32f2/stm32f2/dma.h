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

#ifndef STM32F2_DMA_H_
#define STM32F2_DMA_H_

#include "hw/dma_hw.h"

typedef enum dma_stream_memory_burst_t {
	DMA_STREAM_MEMORY_BURST_SINGLE_TRANSFER 			= DMA_STREAM_CR_MBURST_SINGLE_TRANSFER,
	DMA_STREAM_MEMORY_BURST_INCREMENT_4					= DMA_STREAM_CR_MBURST_INCR4,
	DMA_STREAM_MEMORY_BURST_INCREMENT_8					= DMA_STREAM_CR_MBURST_INCR8,
	DMA_STREAM_MEMORY_BURST_INCREMENT_16				= DMA_STREAM_CR_MBURST_INCR16,
} dma_stream_memory_burst_t;

typedef enum dma_stream_peripheral_burst_t {
	DMA_STREAM_PERIPHERAL_BURST_SINGLE_TRANSFER 		= DMA_STREAM_CR_PBURST_SINGLE_TRANSFER,
	DMA_STREAM_PERIPHERAL_BURST_INCREMENT_4				= DMA_STREAM_CR_PBURST_INCR4,
	DMA_STREAM_PERIPHERAL_BURST_INCREMENT_8				= DMA_STREAM_CR_PBURST_INCR8,
	DMA_STREAM_PERIPHERAL_BURST_INCREMENT_16			= DMA_STREAM_CR_PBURST_INCR16,
} dma_stream_peripheral_burst_t;

typedef enum dma_stream_current_target_t {
	DMA_STREAM_CURRENT_TARGET_MEMORY_0					= DMA_STREAM_CR_CT_MEMORY_0,
	DMA_STREAM_CURRENT_TARGET_MEMORY_1					= DMA_STREAM_CR_CT_MEMORY_1,
} dma_stream_current_target_t;

typedef enum dma_stream_double_buffer_t {
	DMA_STREAM_DOUBLE_BUFFER_OFF						= DMA_STREAM_CR_DBM_OFF,
	DMA_STREAM_DOUBLE_BUFFER_ON							= DMA_STREAM_CR_DBM_ON,
} dma_stream_double_buffer_t;

typedef enum dma_stream_priority_t {
	DMA_STREAM_PRIORITY_LOW								= DMA_STREAM_CR_PL_LOW,
	DMA_STREAM_PRIORITY_MEDIUM							= DMA_STREAM_CR_PL_MEDIUM,
	DMA_STREAM_PRIORITY_HIGH							= DMA_STREAM_CR_PL_HIGH,
	DMA_STREAM_PRIORITY_VERY_HIGH						= DMA_STREAM_CR_PL_VERY_HIGH,
} dma_stream_priority_t;

typedef enum dma_stream_pincos_t {
	DMA_STREAM_PINCOS_LINKED_TO_PSIZE					= DMA_STREAM_CR_PINCOS_LINKED_TO_PSIZE,
	DMA_STREAM_PINCOS_FIXED_TO_4						= DMA_STREAM_CR_PINCOS_FIXED_TO_4,
} dma_stream_pincos_t;

typedef enum dma_stream_memory_data_size_t {
	DMA_STREAM_MEMORY_DATA_SIZE_BYTE					= DMA_STREAM_CR_MSIZE_BYTE,
	DMA_STREAM_MEMORY_DATA_SIZE_HALF_WORD				= DMA_STREAM_CR_MSIZE_HALF_WORD,
	DMA_STREAM_MEMORY_DATA_SIZE_WORD					= DMA_STREAM_CR_MSIZE_WORD,
} dma_stream_memory_data_size_t;

typedef enum dma_stream_peripheral_data_size_t {
	DMA_STREAM_PERIPHERAL_DATA_SIZE_BYTE				= DMA_STREAM_CR_PSIZE_BYTE,
	DMA_STREAM_PERIPHERAL_DATA_SIZE_HALF_WORD			= DMA_STREAM_CR_PSIZE_HALF_WORD,
	DMA_STREAM_PERIPHERAL_DATA_SIZE_WORD				= DMA_STREAM_CR_PSIZE_WORD,
} dma_stream_peripheral_data_size_t;

typedef enum dma_stream_memory_address_mode_t {
	DMA_STREAM_MEMORY_ADDRESS_FIXED						= DMA_STREAM_CR_MINC_FIXED,
	DMA_STREAM_MEMORY_ADDRESS_INCREMENT					= DMA_STREAM_CR_MINC_INCREMENT,
} dma_stream_memory_address_mode_t;

typedef enum dma_stream_peripheral_address_mode_t {
	DMA_STREAM_PERIPHERAL_ADDRESS_FIXED					= DMA_STREAM_CR_PINC_FIXED,
	DMA_STREAM_PERIPHERAL_ADDRESS_INCREMENT				= DMA_STREAM_CR_PINC_INCREMENT,
} dma_stream_peripheral_address_mode_t;

typedef enum dma_stream_circular_mode_t {
	DMA_STREAM_CIRCULAR_MODE_DISABLED					= DMA_STREAM_CR_CIRC_DISABLED,
	DMA_STREAM_CIRCULAR_MODE_ENABLED					= DMA_STREAM_CR_CIRC_ENABLED,
} dma_stream_circular_mode_t;

typedef enum dma_stream_direction_t {
	DMA_STREAM_DIRECTION_PERIPHERAL_TO_MEMORY			= DMA_STREAM_CR_DIR_PERIPHERAL_TO_MEMORY,
	DMA_STREAM_DIRECTION_MEMORY_TO_PERIPHERAL			= DMA_STREAM_CR_DIR_MEMORY_TO_PERIPHERAL,
	DMA_STREAM_DIRECTION_MEMORY_TO_MEMORY				= DMA_STREAM_CR_DIR_MEMORY_TO_MEMORY,
} dma_stream_direction_t;

typedef enum dma_stream_peripheral_flow_controller_t {
	DMA_STREAM_PERIPHERAL_FLOW_CONTROLLER_DMA			= DMA_STREAM_CR_PFCTRL_DMA,
	DMA_STREAM_PERIPHERAL_FLOW_CONTROLLER_PERIPHERAL	= DMA_STREAM_CR_PFCTRL_PERIPHERAL,
} dma_stream_peripheral_flow_controller_t;

typedef enum dma_stream_enable_t {
	DMA_STREAM_DISABLED									= DMA_STREAM_CR_EN_DISABLED,
	DMA_STREAM_ENABLED									= DMA_STREAM_CR_EN_ENABLED,
} dma_stream_enable_t;

typedef enum dma_stream_direct_mode_t {
	DMA_STREAM_DIRECT_MODE_DISABLED						= DMA_STREAM_FCR_DMDIS_DIRECT_MODE_DISABLED,
	DMA_STREAM_DIRECT_MODE_ENABLED						= DMA_STREAM_FCR_DMDIS_DIRECT_MODE_ENABLED,
} dma_stream_direct_mode_t;

typedef enum dma_stream_fifo_threshold_t {
	DMA_STREAM_FIFO_THRESHOLD_1_4_FULL 					= DMA_STREAM_FCR_FTH_1_4_FULL_FIFO,
	DMA_STREAM_FIFO_THRESHOLD_1_2_FULL 					= DMA_STREAM_FCR_FTH_1_2_FULL_FIFO,
	DMA_STREAM_FIFO_THRESHOLD_3_4_FULL 					= DMA_STREAM_FCR_FTH_3_4_FULL_FIFO,
	DMA_STREAM_FIFO_THRESHOLD_FULL						= DMA_STREAM_FCR_FTH_FULL_FIFO,
} dma_stream_fifo_threshold_t;

void dma_stream_enable(dma_stream_t* const stream);
void dma_stream_disable(dma_stream_t* const stream);
void dma_stream_wait_until_disabled(dma_stream_t* const stream);

void dma_stream_set_peripheral_address(dma_stream_t* const stream, void* const address);
void dma_stream_set_memory_0_address(dma_stream_t* const stream, void* const address);
void dma_stream_set_memory_1_address(dma_stream_t* const stream, void* const address);
void dma_stream_set_item_count(dma_stream_t* const stream, const uint32_t count);

dma_stream_current_target_t dma_stream_get_current_target(dma_stream_t* const stream);

void dma_stream_set_cr(dma_stream_t* const stream,
					   const uint_fast8_t channel,
					   const dma_stream_memory_burst_t memory_burst,
					   const dma_stream_peripheral_burst_t peripheral_burst,
					   const dma_stream_current_target_t current_target,
					   const dma_stream_double_buffer_t double_buffer,
					   const dma_stream_priority_t priority,
					   const dma_stream_pincos_t peripheral_increment_offset_size,
					   const dma_stream_memory_data_size_t memory_data_size,
					   const dma_stream_peripheral_data_size_t peripheral_data_size,
					   const dma_stream_memory_address_mode_t memory_increment_mode,
					   const dma_stream_peripheral_address_mode_t peripheral_increment_mode,
					   const dma_stream_circular_mode_t circular_mode,
					   const dma_stream_direction_t data_transfer_direction,
					   const dma_stream_peripheral_flow_controller_t peripheral_flow_controller,
					   const uint_fast8_t tcie,
					   const uint_fast8_t htie,
					   const uint_fast8_t teie,
					   const uint_fast8_t dmeie,
					   const dma_stream_enable_t en
					   );
void dma_stream_set_fcr(dma_stream_t* const stream,
						const uint_fast8_t fifo_error_interrupt_enable,
						const dma_stream_direct_mode_t direct_mode_disable,
						const dma_stream_fifo_threshold_t fifo_threshold
						);

#endif /* STM32F2_DMA_H_ */
