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

#ifndef STM32F2_SPI_HW_H_
#define STM32F2_SPI_HW_H_

#include <stdint.h>

typedef struct spi_t {
	volatile uint32_t	CR1;
	volatile uint32_t	CR2;
	volatile uint32_t	SR;
	volatile uint32_t	DR;
	volatile uint32_t	CRCPR;
	volatile uint32_t	RXCRCR;
	volatile uint32_t	TXCRCR;
	volatile uint32_t	I2SCFGR;
	volatile uint32_t	I2SPR;
} spi_t;

///////////////////////////////////////////////////////////////////
// 0x00 CR1

typedef enum spi_cr1_bit_t {
	SPI_CR1_CPHA_BIT		=  0,
	SPI_CR1_CPOL_BIT		=  1,
	SPI_CR1_MSTR_BIT		=  2,
	SPI_CR1_SPE_BIT			=  6,
	SPI_CR1_LSBFIRST_BIT	=  7,
	SPI_CR1_SSI_BIT			=  8,
	SPI_CR1_SSM_BIT			=  9,
	SPI_CR1_RXONLY_BIT		= 10,
	SPI_CR1_DFF_BIT			= 11,
	SPI_CR1_CRCNEXT_BIT		= 12,
	SPI_CR1_CRCEN_BIT		= 13,
	SPI_CR1_BIDIOE_BIT		= 14,
	SPI_CR1_BIDIMODE_BIT	= 15,
} spi_cr1_bit_t;

typedef enum spi_cr1_base_t {
	SPI_CR1_BR_BASE			=  3,
} spi_cr1_base_t;

typedef enum spi_cr1_cpha_t {
	SPI_CR1_CPHA_FIRST_CLOCK_EDGE_CAPTURE	= 0,
	SPI_CR1_CPHA_SECOND_CLOCK_EDGE_CAPTURE	= 1,
} spi_cr1_cpha_t;

typedef enum spi_cr1_cpol_t {
	SPI_CR1_CPOL_CK_IDLE_0	= 0,
	SPI_CR1_CPOL_CK_IDLE_1	= 1,
} spi_cr1_cpol_t;

typedef enum spi_cr1_mstr_t {
	SPI_CR1_MSTR_SLAVE	= 0,
	SPI_CR1_MSTR_MASTER	= 1,
} spi_cr1_mstr_t;

typedef enum spi_cr1_br_t {
	SPI_CR1_BR_FPCLK_DIV_2		= 0b000,
	SPI_CR1_BR_FPCLK_DIV_4		= 0b001,
	SPI_CR1_BR_FPCLK_DIV_8		= 0b010,
	SPI_CR1_BR_FPCLK_DIV_16		= 0b011,
	SPI_CR1_BR_FPCLK_DIV_32		= 0b100,
	SPI_CR1_BR_FPCLK_DIV_64		= 0b101,
	SPI_CR1_BR_FPCLK_DIV_128	= 0b110,
	SPI_CR1_BR_FPCLK_DIV_256	= 0b111,
} spi_cr1_br_t;

typedef enum spi_cr1_spe_t {
	SPI_CR1_SPE_PERIPHERAL_DISABLED = 0,
	SPI_CR1_SPE_PERIPHERAL_ENABLED  = 1,
} spi_cr1_spe_t;

typedef enum spi_cr1_lsbfirst_t {
	SPI_CR1_LSBFIRST_MSB_FIRST = 0,
	SPI_CR1_LSBFIRST_LSB_FIRST = 1,
} spi_cr1_lsbfirst_t;

typedef enum spi_cr1_ssi_t {
	SPI_CR1_SSI_0		= 0,
	SPI_CR1_SSI_1		= 1,
} spi_cr1_ssi_t;

typedef enum spi_cr1_ssm_t {
	SPI_CR1_SSM_SOFTWARE_SLAVE_MGMT_DISABLED	= 0,
	SPI_CR1_SSM_SOFTWARE_SLAVE_MGMT_ENABLED		= 1,
} spi_cr1_ssm_t;

typedef enum spi_cr1_rxonly_t {
	SPI_CR1_RXONLY_FULL_DUPLEX		= 0,
	SPI_CR1_RXONLY_OUTPUT_DISABLED	= 1,
} spi_cr1_rxonly_t;

typedef enum spi_cr1_dff_t {
	SPI_CR1_DFF_8_BIT_DATA_FRAME	= 0,
	SPI_CR1_DFF_16_BIT_DATA_FRAME	= 1,
} spi_cr1_dff_t;

typedef enum spi_cr1_crcnext_t {
	SPI_CR1_CRCNEXT_DATA_PHASE		= 0,
	SPI_CR1_CRCNEXT_CRC				= 1,
} spi_cr1_crcnext_t;

typedef enum spi_cr1_crcen_t {
	SPI_CR1_CRCEN_CRC_CALCULATION_DISABLED	= 0,
	SPI_CR1_CRCEN_CRC_CALCULATION_ENABLED	= 1,
} spi_cr1_crcen_t;

typedef enum spi_cr1_bidioe_t {
	SPI_CR1_BIDIOE_OUTPUT_DISABLED	= 0,
	SPI_CR1_BIDIOE_OUTPUT_ENABLED	= 1,
} spi_cr1_bidioe_t;

typedef enum spi_cr1_bidimode_t {
	SPI_CR1_BIDIMODE_2_LINE_UNIDIRECTIONAL	= 0,
	SPI_CR1_BIDIMODE_1_LINE_BIDIRECTIONAL	= 1,
} spi_cr1_bidimode_t;

///////////////////////////////////////////////////////////////////
// 0x04 CR2

typedef enum spi_cr2_bit_t {
	SPI_CR2_RXDMAEN_BIT		= 0,
	SPI_CR2_TXDMAEN_BIT		= 1,
	SPI_CR2_SSOE_BIT		= 2,
	SPI_CR2_FRF_BIT			= 4,
	SPI_CR2_ERRIE_BIT		= 5,
	SPI_CR2_RXNEIE_BIT		= 6,
	SPI_CR2_TXEIE_BIT		= 7,
} spi_cr2_bit_t;

typedef enum spi_cr2_rxdmaen_t {
	SPI_CR2_RXDMAEN_DMA_DISABLED	= 0,
	SPI_CR2_RXDMAEN_DMA_ENABLED		= 1,
} spi_cr2_rxdmaen_t;

typedef enum spi_cr2_txdmaen_t {
	SPI_CR2_TXDMAEN_DMA_DISABLED	= 0,
	SPI_CR2_TXDMAEN_DMA_ENABLED		= 1,
} spi_cr2_txdmaen_t;

typedef enum spi_cr2_ssoe_t {
	SPI_CR2_SSOE_SS_OUTPUT_DISABLED	= 0,
	SPI_CR2_SSOE_SS_OUTPUT_ENABLED	= 1,
} spi_cr2_ssoe_t;

typedef enum spi_cr2_frf_t {
	SPI_CR2_FRF_SPI_MOTOROLA	= 0,
	SPI_CR2_FRF_SPI_TI			= 1,
} spi_cr2_frf_t;

typedef enum spi_cr2_errie_t {
	SPI_CR2_ERRIE_MASKED	= 0,
	SPI_CR2_ERRIE_ENABLED	= 1,
} spi_cr2_errie_t;

typedef enum spi_cr2_rxneie_t {
	SPI_CR2_RXNEIE_MASKED		= 0,
	SPI_CR2_RXNEIE_NOT_MASKED	= 1,
} spi_cr2_rxneie_t;

typedef enum spi_cr2_txeie_t {
	SPI_CR2_TXEIE_MASKED		= 0,
	SPI_CR2_TXEIE_NOT_MASKED	= 1,
} spi_cr2_txeie_t;

///////////////////////////////////////////////////////////////////
// 0x08 SR

typedef enum spi_sr_bit_t {
	SPI_SR_RXNE_BIT		= 0,
	SPI_SR_TXE_BIT		= 1,
	SPI_SR_CHSIDE_BIT	= 2,
	SPI_SR_UDR_BIT		= 3,
	SPI_SR_CRCERR_BIT	= 4,
	SPI_SR_MODF_BIT		= 5,
	SPI_SR_OVR_BIT		= 6,
	SPI_SR_BSY_BIT		= 7,
	SPI_SR_TIFRFE_BIT	= 8,
} spi_sr_bit_t;

typedef enum spi_sr_rxne_t {
	SPI_SR_RXNE_RX_BUFFER_EMPTY		= 0,
	SPI_SR_RXNE_RX_BUFFER_NOT_EMPTY	= 1,
} spi_sr_rxne_t;

typedef enum spi_sr_txe_t {
	SPI_SR_TXE_TX_BUFFER_NOT_EMPTY	= 0,
	SPI_SR_TXE_TX_BUFFER_EMPTY		= 1,
} spi_sr_txe_t;

typedef enum spi_sr_chside_t {
	SPI_SR_CHSIDE_LEFT	= 0,
	SPI_SR_CHSIDE_RIGHT	= 1,
} spi_sr_chside_t;

typedef enum spi_sr_udr_t {
	SPI_SR_UDR_NO_UNDERRUN	= 0,
	SPI_SR_UDR_UNDERRUN		= 1,
} spi_sr_udr_t;

typedef enum spi_sr_crcerr_t {
	SPI_SR_CRCERR_MATCHES			= 0,
	SPI_SR_CRCERR_DOES_NOT_MATCH	= 1,
} spi_sr_crcerr_t;

typedef enum spi_sr_modf_t {
	SPI_SR_MODF_NO_MODE_FAULT	= 0,
	SPI_SR_MODF_MODE_FAULT		= 1,
} spi_sr_modf_t;

typedef enum spi_sr_ovr_t {
	SPI_SR_OVR_NO_OVERRUN		= 0,
	SPI_SR_OVR_OVERRUN			= 1,
} spi_sr_ovr_t;

typedef enum spi_sr_bsy_t {
	SPI_SR_BSY_NOT_BUSY		= 0,
	SPI_SR_BSY_BUSY_OR_TXNE	= 1,
} spi_sr_bsy_t;

typedef enum spi_sr_tifrfe_t {
	SPI_SR_TIFRFE_NO_FRAME_FORMAT_ERROR	= 0,
	SPI_SR_TIFRFE_FRAME_FORMAT_ERROR	= 1,
} spi_sr_tifrfe_t;

///////////////////////////////////////////////////////////////////
// 0x1C I2SCFGR

typedef enum spi_i2scfgr_bit_t {
	SPI_I2SCFGR_CHLEN_BIT	=  0,
	SPI_I2SCFGR_CKPOL_BIT	=  3,
	SPI_I2SCFGR_PCMSYNC_BIT	=  7,
	SPI_I2SCFGR_I2SE_BIT	= 10,
	SPI_I2SCFGR_I2SMOD_BIT	= 11,
} spi_i2scfgr_bit_t;

typedef enum spi_i2scfgr_base_t {
	SPI_I2SCFGR_DATLEN_BASE	= 1,
	SPI_I2SCFGR_I2SSTD_BASE	= 4,
	SPI_I2SCFGR_I2SCFG_BASE	= 8,
} spi_i2scfgr_base_t;

typedef enum spi_i2scfgr_chlen_t {
	SPI_I2SCFGR_CHLEN_16_BIT_WIDTH	= 0,
	SPI_I2SCFGR_CHLEN_32_BIT_WIDTH	= 1,
} spi_i2scfgr_chlen_t;

typedef enum spi_i2scfgr_datlen_t {
	SPI_I2SCFGR_DATLEN_16_BIT_LENGTH	= 0b00,
	SPI_I2SCFGR_DATLEN_24_BIT_LENGTH	= 0b01,
	SPI_I2SCFGR_DATLEN_32_BIT_LENGTH	= 0b10,
} spi_i2scfgr_datlen_t;

typedef enum spi_i2scfgr_ckpol_t {
	SPI_I2SCFGR_CKPOL_STEADY_STATE_LOW	= 0,
	SPI_I2SCFGR_CKPOL_STEADY_STATE_HIGH	= 1,
} spi_i2scfgr_ckpol_t;

typedef enum spi_i2scfgr_i2sstd_t {
	SPI_I2SCFGR_I2SSTD_PHILIPS			= 0b00,
	SPI_I2SCFGR_I2SSTD_MSB_JUSTIFIED	= 0b01,
	SPI_I2SCFGR_I2SSTD_LSB_JUSTIFIED	= 0b10,
	SPI_I2SCFGR_I2SSTD_PCM				= 0b11,
} spi_i2scfgr_i2sstd_t;

typedef enum spi_i2scfgr_pcmsync_t {
	SPI_I2SCFGR_PCMSYNC_SHORT_FRAME_SYNC	= 0,
	SPI_I2SCFGR_PCMSYNC_LONG_FRAME_SYNC		= 1,
} spi_i2scfgr_pcmsync_t;

typedef enum spi_i2scfgr_i2scfg_t {
	SPI_I2SCFGR_I2SCFG_SLAVE_TRANSMIT	= 0b00,
	SPI_I2SCFGR_I2SCFG_SLAVE_RECEIVE	= 0b01,
	SPI_I2SCFGR_I2SCFG_MASTER_TRANSMIT	= 0b10,
	SPI_I2SCFGR_I2SCFG_MASTER_RECEIVE	= 0b11,
} spi_i2scfgr_i2scfg_t;

typedef enum spi_i2scfgr_i2se_t {
	SPI_I2SCFGR_I2SE_DISABLED	= 0,
	SPI_I2SCFGR_I2SE_ENABLED	= 1,
} spi_i2scfgr_i2se_t;

typedef enum spi_i2scfgr_i2smod_t {
	SPI_I2SCFGR_I2SMOD_SPI_MODE	= 0,
	SPI_I2SCFGR_I2SMOD_I2S_MODE	= 1,
} spi_i2scfgr_i2smod_t;

///////////////////////////////////////////////////////////////////
// 0x20 I2SPR

typedef enum spi_i2spr_bit_t {
	SPI_I2SPR_ODD_BIT		= 8,
	SPI_I2SPR_MCKOE_BIT		= 9,
} spi_i2spr_bit_t;

typedef enum spi_i2spr_base_t {
	SPI_I2SPR_I2SDIV_BASE	= 0,
} spi_i2spr_base_t;

typedef enum spi_i2spr_odd_t {
	SPI_I2SPR_ODD_NO	= 0,
	SPI_I2SPR_ODD_YES	= 1,
} spi_i2spr_odd_t;

typedef enum spi_i2spr_mckoe_t {
	SPI_I2SPR_MCKOE_DISABLED 	= 0,
	SPI_I2SPR_MCKOE_ENABLED		= 1,
} spi_i2spr_mckoe_t;

///////////////////////////////////////////////////////////////////

#define SPI2_BASE ((spi_t*)0x40003800)
#define SPI3_BASE ((spi_t*)0x40003C00)

extern spi_t* const SPI2;
extern spi_t* const SPI3;

#endif /* STM32F2_SPI_HW_H_ */
