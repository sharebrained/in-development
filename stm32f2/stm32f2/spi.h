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

#ifndef STM32F2_SPI_H_
#define STM32F2_SPI_H_

#include "hw/spi_hw.h"

typedef enum spi_master_clock_output_t {
	SPI_MASTER_CLOCK_OUTPUT_DISABLED	= SPI_I2SPR_MCKOE_DISABLED,
	SPI_MASTER_CLOCK_OUTPUT_ENABLED		= SPI_I2SPR_MCKOE_ENABLED,
} spi_master_clock_output_t;

typedef enum spi_prescaler_factor_t {
	SPI_PRESCALER_FACTOR_EVEN	= SPI_I2SPR_ODD_NO,
	SPI_PRESCALER_FACTOR_ODD	= SPI_I2SPR_ODD_YES,
} spi_prescaler_factor_t;

typedef enum spi_mode_t {
	SPI_MODE_SPI	= SPI_I2SCFGR_I2SMOD_SPI_MODE,
	SPI_MODE_I2S	= SPI_I2SCFGR_I2SMOD_I2S_MODE,
} spi_mode_t;

typedef enum i2s_enable_t {
	I2S_DISABLED	= SPI_I2SCFGR_I2SE_DISABLED,
	I2S_ENABLED		= SPI_I2SCFGR_I2SE_ENABLED,
} i2s_enable_t;

typedef enum i2s_config_t {
	I2S_CONFIG_SLAVE_TRANSMIT	= SPI_I2SCFGR_I2SCFG_SLAVE_TRANSMIT,
	I2S_CONFIG_SLAVE_RECEIVE	= SPI_I2SCFGR_I2SCFG_SLAVE_RECEIVE,
	I2S_CONFIG_MASTER_TRANSMIT	= SPI_I2SCFGR_I2SCFG_MASTER_TRANSMIT,
	I2S_CONFIG_MASTER_RECEIVE	= SPI_I2SCFGR_I2SCFG_MASTER_RECEIVE,
} i2s_config_t;

typedef enum i2s_pcmsync_t {
	I2S_PCMSYNC_SHORT_FRAME		= SPI_I2SCFGR_PCMSYNC_SHORT_FRAME_SYNC,
	I2S_PCMSYNC_LONG_FRAME		= SPI_I2SCFGR_PCMSYNC_LONG_FRAME_SYNC,
} i2s_pcmsync_t;

typedef enum i2s_standard_t {
	I2S_STANDARD_PHILIPS		= SPI_I2SCFGR_I2SSTD_PHILIPS,
	I2S_STANDARD_MSB_JUSTIFIED	= SPI_I2SCFGR_I2SSTD_MSB_JUSTIFIED,
	I2S_STANDARD_LSB_JUSTIFIED	= SPI_I2SCFGR_I2SSTD_LSB_JUSTIFIED,
	I2S_STANDARD_PCM			= SPI_I2SCFGR_I2SSTD_PCM,
} i2s_standard_t;

typedef enum i2s_clock_steady_state_t {
	I2S_CLOCK_STEADY_STATE_LOW	= SPI_I2SCFGR_CKPOL_STEADY_STATE_LOW,
	I2S_CLOCK_STEADY_STATE_HIGH	= SPI_I2SCFGR_CKPOL_STEADY_STATE_HIGH,
} i2s_clock_steady_state_t;

typedef enum i2s_data_transfer_length_t {
	I2S_DATA_TRANSFER_LENGTH_16_BITS	= SPI_I2SCFGR_DATLEN_16_BIT_LENGTH,
	I2S_DATA_TRANSFER_LENGTH_24_BITS	= SPI_I2SCFGR_DATLEN_24_BIT_LENGTH,
	I2S_DATA_TRANSFER_LENGTH_32_BITS	= SPI_I2SCFGR_DATLEN_32_BIT_LENGTH,
} i2s_data_transfer_length_t;

typedef enum i2s_channel_length_t {
	I2S_CHANNEL_LENGTH_16_BITS	= SPI_I2SCFGR_CHLEN_16_BIT_WIDTH,
	I2S_CHANNEL_LENGTH_32_BITS	= SPI_I2SCFGR_CHLEN_32_BIT_WIDTH,
} i2s_channel_length_t;

void spi_disable(spi_t* const spi);
void spi_enable(spi_t* const spi);

void i2s_disable(spi_t* const spi);
void i2s_enable(spi_t* const spi);

void spi_enable_rx_dma(spi_t* const spi);
void spi_enable_tx_dma(spi_t* const spi);

void spi_set_i2spr(spi_t* const spi,
				   const spi_master_clock_output_t master_clock_output,
				   const spi_prescaler_factor_t prescaler_factor,
				   const uint_fast8_t i2sdiv
				   );

void spi_set_i2scfgr(spi_t* const spi,
					 const spi_mode_t mode,
					 const i2s_enable_t enable,
					 const i2s_config_t config,
					 const i2s_pcmsync_t pcmsync,
					 const i2s_standard_t standard,
					 const i2s_clock_steady_state_t clock_steady_state,
					 const i2s_data_transfer_length_t data_transfer_length,
					 const i2s_channel_length_t channel_length
					 );

#endif /* STM32F2_SPI_H_ */
