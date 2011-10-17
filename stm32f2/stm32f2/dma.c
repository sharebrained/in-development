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

#include "dma.h"
#include "bitband.h"

void dma_stream_enable(dma_stream_t* const stream) {
	bitband_set(BITBAND_PERIPHERAL, &stream->CR, DMA_STREAM_CR_EN_BIT, 1);
}

void dma_stream_disable(dma_stream_t* const stream) {
	bitband_set(BITBAND_PERIPHERAL, &stream->CR, DMA_STREAM_CR_EN_BIT, 0);
}

void dma_stream_wait_until_disabled(dma_stream_t* const stream) {
	while( bitband_get(BITBAND_PERIPHERAL, &stream->CR, DMA_STREAM_CR_EN_BIT) != 0 );
}

void dma_stream_set_peripheral_address(dma_stream_t* const stream, void* const address) {
	stream->PAR = (uint32_t)address;
}

void dma_stream_set_memory_0_address(dma_stream_t* const stream, void* const address) {
	stream->M0AR = (uint32_t)address;
}

void dma_stream_set_memory_1_address(dma_stream_t* const stream, void* const address) {
	stream->M1AR = (uint32_t)address;
}

void dma_stream_set_item_count(dma_stream_t* const stream, const uint32_t count) {
	stream->NDTR = count;
}

dma_stream_current_target_t dma_stream_get_current_target(dma_stream_t* const stream) {
	return bitband_get(BITBAND_PERIPHERAL, &stream->CR, DMA_STREAM_CR_CT_BIT);
}

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
					   ) {
	stream->CR = (channel << DMA_STREAM_CR_CHSEL_BASE) |
			  	 (memory_burst << DMA_STREAM_CR_MBURST_BASE) |
			  	 (peripheral_burst << DMA_STREAM_CR_PBURST_BASE) |
			  	 (current_target << DMA_STREAM_CR_CT_BIT) |
			  	 (double_buffer << DMA_STREAM_CR_DBM_BIT) |
			  	 (priority << DMA_STREAM_CR_PL_BASE) |
			  	 (peripheral_increment_offset_size << DMA_STREAM_CR_PINCOS_BIT) |
			  	 (memory_data_size << DMA_STREAM_CR_MSIZE_BASE) |
			  	 (peripheral_data_size << DMA_STREAM_CR_PSIZE_BASE) |
			  	 (memory_increment_mode << DMA_STREAM_CR_MINC_BIT) |
			  	 (peripheral_increment_mode << DMA_STREAM_CR_PINC_BIT) |
			  	 (circular_mode << DMA_STREAM_CR_CIRC_BIT) |
			  	 (data_transfer_direction << DMA_STREAM_CR_DIR_BASE) |
			  	 (peripheral_flow_controller << DMA_STREAM_CR_PFCTRL_BIT) |
			  	 (tcie << DMA_STREAM_CR_TCIE_BIT) |
			  	 (htie << DMA_STREAM_CR_HTIE_BIT) |
			  	 (teie << DMA_STREAM_CR_TEIE_BIT) |
			  	 (dmeie << DMA_STREAM_CR_DMEIE_BIT) |
			  	 (en << DMA_STREAM_CR_EN_BIT) ;
}

void dma_stream_set_fcr(dma_stream_t* const stream,
						const uint_fast8_t fifo_error_interrupt_enable,
						const dma_stream_direct_mode_t direct_mode_disable,
						const dma_stream_fifo_threshold_t fifo_threshold
						) {
	stream->FCR = (fifo_error_interrupt_enable << DMA_STREAM_FCR_FEIE_BIT) |
			  	  (direct_mode_disable << DMA_STREAM_FCR_DMDIS_BIT) |
			  	  (fifo_threshold << DMA_STREAM_FCR_FTH_BASE);
}
