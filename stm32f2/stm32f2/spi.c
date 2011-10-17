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

#include "spi.h"
#include "bitband.h"

void spi_enable(spi_t* const spi) {
	bitband_set(BITBAND_PERIPHERAL, &spi->CR1, SPI_CR1_SPE_BIT, 1);
}

void spi_disable(spi_t* const spi) {
	bitband_set(BITBAND_PERIPHERAL, &spi->CR1, SPI_CR1_SPE_BIT, 0);
}

void i2s_enable(spi_t* const spi) {
	bitband_set(BITBAND_PERIPHERAL, &spi->I2SCFGR, SPI_I2SCFGR_I2SE_BIT, 1);
}

void i2s_disable(spi_t* const spi) {
	bitband_set(BITBAND_PERIPHERAL, &spi->I2SCFGR, SPI_I2SCFGR_I2SE_BIT, 0);
}

void spi_enable_rx_dma(spi_t* const spi) {
	bitband_set(BITBAND_PERIPHERAL, &spi->CR2, SPI_CR2_RXDMAEN_BIT, 1);
}

void spi_enable_tx_dma(spi_t* const spi) {
	bitband_set(BITBAND_PERIPHERAL, &spi->CR2, SPI_CR2_TXDMAEN_BIT, 1);
}

void spi_set_i2spr(spi_t* const spi,
				   const spi_master_clock_output_t master_clock_output,
				   const spi_prescaler_factor_t prescaler_factor,
				   const uint_fast8_t i2sdiv) {
	spi->I2SPR = (master_clock_output << SPI_I2SPR_MCKOE_BIT) |
			     (prescaler_factor << SPI_I2SPR_ODD_BIT) |
			     (i2sdiv << SPI_I2SPR_I2SDIV_BASE);
}

void spi_set_i2scfgr(spi_t* const spi,
					 const spi_mode_t mode,
					 const i2s_enable_t enable,
					 const i2s_config_t config,
					 const i2s_pcmsync_t pcmsync,
					 const i2s_standard_t standard,
					 const i2s_clock_steady_state_t clock_steady_state,
					 const i2s_data_transfer_length_t data_transfer_length,
					 const i2s_channel_length_t channel_length
					 ) {
	spi->I2SCFGR = (mode << SPI_I2SCFGR_I2SMOD_BIT) |
				   (enable << SPI_I2SCFGR_I2SE_BIT) |
				   (config << SPI_I2SCFGR_I2SCFG_BASE) |
				   (pcmsync << SPI_I2SCFGR_PCMSYNC_BIT) |
				   (standard << SPI_I2SCFGR_I2SSTD_BASE) |
				   (clock_steady_state << SPI_I2SCFGR_CKPOL_BIT) |
				   (data_transfer_length << SPI_I2SCFGR_DATLEN_BASE) |
				   (channel_length << SPI_I2SCFGR_CHLEN_BIT);
}
