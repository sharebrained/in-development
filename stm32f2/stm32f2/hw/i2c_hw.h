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

#ifndef STM32F2_I2C_HW_H_
#define STM32F2_I2C_HW_H_

#include <stdint.h>

typedef struct i2c_t {
	volatile uint32_t		CR1;
	volatile uint32_t		CR2;
	volatile uint32_t		OAR1;
	volatile uint32_t		OAR2;
	volatile uint32_t		DR;
	volatile uint32_t		SR1;
	volatile uint32_t		SR2;
	volatile uint32_t		CCR;
	volatile uint32_t		TRISE;
} i2c_t;

///////////////////////////////////////////////////////////////////
// 0x00 CR1

typedef enum i2c_cr1_bit_t {
	I2C_CR1_PE_BIT			=  0,
	I2C_CR1_SMBUS_BIT		=  1,
	I2C_CR1_SMBTYPE_BIT		=  3,
	I2C_CR1_ENARP_BIT		=  4,
	I2C_CR1_ENPEC_BIT		=  5,
	I2C_CR1_ENGC_BIT		=  6,
	I2C_CR1_NOSTRETCH_BIT	=  7,
	I2C_CR1_START_BIT		=  8,
	I2C_CR1_STOP_BIT		=  9,
	I2C_CR1_ACK_BIT			= 10,
	I2C_CR1_POS_BIT			= 11,
	I2C_CR1_PEC_BIT			= 12,
	I2C_CR1_ALERT_BIT		= 13,
	I2C_CR1_SWRST_BIT		= 15,
} i2c_cr1_bit_t;

typedef enum i2c_cr1_pe_t {
	I2C_CR1_PE_PERIPHERAL_DISABLE	= 0,
	I2C_CR1_PE_PERIPHERAL_ENABLE	= 1,
} i2c_cr1_pe_t;

typedef enum i2c_cr1_smbus_t {
	I2C_CR1_SMBUS_I2C_MODE		= 0,
	I2C_CR1_SMBUS_SMBUS_MODE	= 1,
} i2c_cr1_smbus_t;

typedef enum i2c_cr1_smbtype_t {
	I2C_CR1_SMBTYPE_DEVICE	= 0,
	I2C_CR1_SMBTYPE_HOST	= 1,
} i2c_cr1_smbtype_t;

typedef enum i2c_cr1_enarp_t {
	I2C_CR1_ENARP_ARP_DISABLE	= 0,
	I2C_CR1_ENARP_ARP_ENABLE	= 1,
} i2c_cr1_enarp_t;

typedef enum i2c_cr1_enpec_t {
	I2C_CR1_ENPEC_PEC_CALCULATION_DISABLED	= 0,
	I2C_CR1_ENPEC_PEC_CALCULATION_ENABLED	= 1,
} i2c_cr1_enpec_t;

typedef enum i2c_cr1_engc_t {
	I2C_CR1_ENGC_GENERAL_CALL_DISABLED	= 0,
	I2C_CR1_ENGC_GENERAL_CALL_ENABLED	= 1,
} i2c_cr1_engc_t;

typedef enum i2c_cr1_nostretch_t {
	I2C_CR1_NOSTRETCH_CLOCK_STRETCHING_ENABLED	= 0,
	I2C_CR1_NOSTRETCH_CLOCK_STRETCHING_DISABLED	= 1,
} i2c_cr1_nostretch_t;

typedef enum i2c_cr1_start_t {
	I2C_CR1_START_MASTER_NO_START_GENERATION			= 0,
	I2C_CR1_START_MASTER_REPEATED_START_GENERATION		= 1,
	I2C_CR1_START_SLAVE_NO_START_GENERATION				= 0,
	I2C_CR1_START_SLAVE_START_GENERATION_WHEN_BUS_FREE	= 1,
} i2c_cr1_start_t;

typedef enum i2c_cr1_stop_t {
	I2C_CR1_STOP_MASTER_NO_STOP_GENERATION					= 0,
	I2C_CR1_STOP_MASTER_STOP_AFTER_CURRENT_BYTE_OR_START	= 1,
	I2C_CR1_STOP_SLAVE_NO_STOP_GENERATION					= 0,
	I2C_CR1_STOP_SLAVE_RELEASE_SCL_SDA_AFTER_CURRENT_BYTE	= 1,
} i2c_cr1_stop_t;

typedef enum i2c_cr1_ack_t {
	I2C_CR1_ACK_NO_ACK_RETURNED				= 0,
	I2C_CR1_ACK_ACK_RETURNED_AFTER_BYTE_RX	= 1,
} i2c_cr1_ack_t;

typedef enum i2c_cr1_pos_t {
	I2C_CR1_POS_ACK_CONTROLS_CURRENT		= 0,
	I2C_CR1_POS_ACK_CONTROLS_NEXT			= 1,
} i2c_cr1_pos_t;

typedef enum i2c_cr1_pec_t {
	I2C_CR1_PEC_NO_PEC_TRANSFER			= 0,
	I2C_CR1_PEC_TRANSFER_IN_TX_RX_MODE	= 1,
} i2c_cr1_pec_t;

typedef enum i2c_cr1_alert_t {
	I2C_CR1_ALERT_SMBA_HIGH_AND_NACK	= 0,
	I2C_CR1_ALERT_SMBA_LOW_AND_ACK		= 1,
} i2c_cr1_alert_t;

typedef enum i2c_cr1_swrst_t {
	I2C_CR1_SWRST_STATE_NORMAL	= 0,
	I2C_CR1_SWRST_STATE_RESET	= 1,
} i2c_cr1_swrst_t;

///////////////////////////////////////////////////////////////////
// 0x04 CR2

typedef enum i2c_cr2_bit_t {
	I2C_CR2_ITERREN_BIT	=  8,
	I2C_CR2_ITEVTEN_BIT	=  9,
	I2C_CR2_ITBUFEN_BIT	= 10,
	I2C_CR2_DMAEN_BIT	= 11,
	I2C_CR2_LAST_BIT	= 12,
} i2c_cr2_bit_t;

typedef enum i2c_cr2_base_t {
	I2C_CR2_FREQ_BASE	= 0,
} i2c_cr2_base_t;

typedef enum i2c_cr2_width_t {
	I2C_CR2_FREQ_WIDTH	= 6,
} i2c_cr2_width_t;
		
typedef enum i2c_cr2_iterren_t {
	I2C_CR2_ITERREN_ERROR_INTERRUPT_DISABLED	= 0,
	I2C_CR2_ITERREN_ERROR_INTERRUPT_ENABLED		= 1,
} i2c_cr2_iterren_t;

typedef enum i2c_cr2_itevten_t {
	I2C_CR2_ITEVTEN_EVENT_INTERRUPT_DISABLED	= 0,
	I2C_CR2_ITEVTEN_EVENT_INTERRUPT_ENABLED		= 1,
} i2c_cr2_itevten_t;

typedef enum i2c_cr2_itbufen_t {
	I2C_CR2_ITBUFEN_TXE_RXNE_NO_INTERRUPT		= 0,
	I2C_CR2_ITBUFEN_TXE_RXNE_INTERRUPT			= 1,
} i2c_cr2_itbufen_t;

typedef enum i2c_cr2_dmaen_t {
	I2C_CR2_DMAEN_REQUESTS_DISABLED		 			= 0,
	I2C_CR2_DMAEN_REQUESTS_ENABLED_WHEN_TXE_OR_RXNE	= 1,
} i2c_cr2_dmaen_t;

typedef enum i2c_cr2_last_t {
	I2C_CR2_LAST_NEXT_DMA_EOT_IS_NOT_LAST_TRANSFER	= 0,
	I2C_CR2_LAST_NEXT_DMA_EOT_IS_LAST_TRANSFER		= 1,
} i2c_cr2_last_t;

///////////////////////////////////////////////////////////////////
// 0x08 OAR1

typedef enum i2c_oar1_bit_t {
	I2C_OAR1_ADDMODE_BIT	= 15,
} i2c_oar1_bit_t;

typedef enum i2c_oar1_base_t {
	I2C_OAR1_ADD_BASE		= 0,
} i2c_oar1_base_t;

///////////////////////////////////////////////////////////////////
// 0x0C OAR2

typedef enum i2c_oar2_bit_t {
	I2C_OAR2_ENDUAL_BIT		= 0,
} i2c_oar2_bit_t;

typedef enum i2c_oar2_base_t {
	I2C_OAR2_ADD2_BASE		= 1,
} i2c_oar2_base_t;

///////////////////////////////////////////////////////////////////
// 0x14 SR1

typedef enum i2c_sr1_bit_t {
	I2C_SR1_SB_BIT			=  0,
	I2C_SR1_ADDR_BIT		=  1,
	I2C_SR1_BTF_BIT			=  2,
	I2C_SR1_ADD10_BIT		=  3,
	I2C_SR1_STOPF_BIT		=  4,
	I2C_SR1_RXNE_BIT		=  6,
	I2C_SR1_TXE_BIT			=  7,
	I2C_SR1_BERR_BIT		=  8,
	I2C_SR1_ARLO_BIT		=  9,
	I2C_SR1_AF_BIT			= 10,
	I2C_SR1_OVR_BIT			= 11,
	I2C_SR1_PECERR_BIT		= 12,
	I2C_SR1_TIMEOUT_BIT		= 14,
	I2C_SR1_SMBALERT_BIT	= 15,
} i2c_sr1_bit_t;

typedef enum i2c_sr1_sb_t {
	I2C_SR1_SB_NO_START_CONDITION			= 0,
	I2C_SR1_SB_START_CONDITION_GENERATED	= 1,
} i2c_sr1_sb_t;

typedef enum i2c_sr1_addr_t {
	I2C_SR1_ADDR_NOT_SENT_OR_MATCHED		= 0,
	I2C_SR1_ADDR_SENT_OR_MATCHED			= 1,
} i2c_sr1_addr_t;

typedef enum i2c_sr1_btf_t {
	I2C_SR1_BTF_DATA_BYTE_TRANSFER_NOT_DONE		= 0,
	I2C_SR1_BTF_DATA_BYTE_TRANSFER_SUCCEEDED	= 1,
} i2c_sr1_btf_t;

typedef enum i2c_sr1_add10_t {
	I2C_SR1_ADD10_NO_EVENT_OCCURRED					= 0,
	I2C_SR1_ADD10_MASTER_SENT_FIRST_ADDRESS_BYTE	= 1,
} i2c_sr1_add10_t;

typedef enum i2c_sr1_stopf_t {
	I2C_SR1_STOPF_NO_STOP_CONDITION_DETECTED	= 0,
	I2C_SR1_STOPF_STOP_CONDITION_DETECTED		= 1,
} i2c_sr1_stopf_t;

typedef enum i2c_sr1_rxne_t {
	I2C_SR1_RXNE_DATA_REGISTER_EMPTY		= 0,
	I2C_SR1_RXNE_DATA_REGISTER_NOT_EMPTY	= 1,
} i2c_sr1_rxne_t;

typedef enum i2c_sr1_txe_t {
	I2C_SR1_TXE_DATA_REGISTER_NOT_EMPTY	= 0,
	I2C_SR1_TXE_DATA_REGISTER_EMPTY		= 1,
} i2c_sr1_txe_t;

typedef enum i2c_sr1_berr_t {
	I2C_SR1_BERR_NO_MISPLACED_START_OR_STOP_CONDITION	= 0,
	I2C_SR1_BERR_MISPLACED_START_OR_STOP_CONDITION		= 1,
} i2c_sr1_berr_t;

typedef enum i2c_sr1_arlo_t {
	I2C_SR1_ARLO_NO_ARBITRATION_LOSS_DETECTED	= 0,
	I2C_SR1_ARLO_ARBITRATION_LOSS_DETECTED		= 1,
} i2c_sr1_arlo_t;

typedef enum i2c_sr1_af_t {
	I2C_SR1_AF_NO_ACKNOWLEDGE_FAILURE	= 0,
	I2C_SR1_AF_ACKNOWLEDGE_FAILURE		= 1,
} i2c_sr1_af_t;

typedef enum i2c_sr1_ovr_t {
	I2C_SR1_OVR_NO_OVERRUN_OR_UNDERRUN	= 0,
	I2C_SR1_OVR_OVERRUN_OR_UNDERRUN = 1,
} i2c_sr1_ovr_t;

typedef enum i2c_sr1_pecerr_t {
	I2C_SR1_PECERR_NO_PEC_ERROR	= 0,
	I2C_SR1_PECERR_PEC_ERROR	= 1,
} i2c_sr1_pecerr_t;

typedef enum i2c_sr1_timeout_t {
	I2C_SR1_TIMEOUT_NO_TIMEOUT_ERROR	= 0,
	I2C_SR1_TIMEOUT_TIMEOUT_ERROR		= 1,
} i2c_sr1_timeout_t;

typedef enum i2c_sr1_smbalert_t {
	I2C_SR1_SMBALERT_HOST_NO_SMBALERT						= 0,
	I2C_SR1_SMBALERT_HOST_EVENT_OCCURRED_ON_PIN				= 1,
	I2C_SR1_SMBALERT_SLAVE_NO_RESPONSE_ADDRESS_HEADER		= 0,
	I2C_SR1_SMBALERT_SLAVE_RESPONSE_ADDRESS_HEADER_TO_LOW	= 1,
} i2c_sr1_smbalert_t;

///////////////////////////////////////////////////////////////////
// 0x18 SR2

typedef enum i2c_sr2_bit_t {
	I2C_SR2_MSL_BIT			= 0,
	I2C_SR2_BUSY_BIT		= 1,
	I2C_SR2_TRA_BIT			= 2,
	I2C_SR2_GENCALL_BIT		= 4,
	I2C_SR2_SMBDEFAULT_BIT	= 5,
	I2C_SR2_SMBHOST_BIT		= 6,
	I2C_SR2_DUALF_BIT		= 7,
} i2c_sr2_bit_t;

typedef enum i2c_sr2_base_t {
	I2C_SR2_PEC_BASE	= 7,
} i2c_sr2_base_t;

typedef enum i2c_sr2_msl_t {
	I2C_SR2_MSL_SLAVE	= 0,
	I2C_SR2_MSL_MASTER	= 1,
} i2c_sr2_msl_t;

typedef enum i2c_sr2_busy_t {
	I2C_SR2_BUSY_NO_BUS_ACTIVITY	= 0,
	I2C_SR2_BUSY_BUS_ACTIVITY		= 1,
} i2c_sr2_busy_t;

typedef enum i2c_sr2_tra_t {
	I2C_SR2_TRA_DATA_BYTES_RECEIVED		= 0,
	I2C_SR2_TRA_DATA_BYTES_TRANSMITTED	= 1,
} i2c_sr2_tra_t;

typedef enum i2c_sr2_gencall_t {
	I2C_SR2_GENCALL_NOT_RECEIVED	= 0,
	I2C_SR2_GENCALL_RECEIVED		= 1,
} i2c_sr2_gencall_t;

typedef enum i2c_sr2_smbdefault_t {
	I2C_SR2_SMBDEFAULT_NOT_RECEIVED	= 0,
	I2C_SR2_SMBDEFAULT_RECEIVED 	= 1,
} i2c_sr2_smbdefault_t;

typedef enum i2c_sr2_smbhost_t {
	I2C_SR2_SMBHOST_NOT_RECEIVED	= 0,
	I2C_SR2_SMBHOST_RECEIVED		= 1,
} i2c_sr2_smbhost_t;

typedef enum i2c_sr2_dualf_t {
	I2C_SR2_DUALF_RECEIVED_ADDRESS_MATCH_OAR1	= 0,
	I2C_SR2_DUALF_RECEIVED_ADDRESS_MATCH_OAR2	= 1,
} i2c_sr2_dualf_t;

///////////////////////////////////////////////////////////////////
// 0x1C CCR

typedef enum i2c_ccr_bit_t {
	I2C_CCR_DUTY_BIT	= 14,
	I2C_CCR_F_S_BIT		= 15,
} i2c_ccr_bit_t;

typedef enum i2c_ccr_base_t {
	I2C_CCR_CCR_BASE	= 0,
} i2c_ccr_base_t;

typedef enum i2c_ccr_duty_t {
	I2C_CCR_DUTY_FAST_MODE_2_1	= 0,
	I2C_CCR_DUTY_FAST_MODE_16_9 = 1,
} i2c_ccr_duty_t;

typedef enum i2c_ccr_f_s_t {
	I2C_CCR_F_S_STANDARD_MODE	= 0,
	I2C_CCR_F_S_FAST_MODE		= 1,
} i2c_ccr_f_s_t;

///////////////////////////////////////////////////////////////////

#define I2C1_BASE ((i2c_t*)0x40005400)
#define I2C2_BASE ((i2c_t*)0x40005800)
#define I2C3_BASE ((i2c_t*)0x40005C00)

extern i2c_t* const I2C1;
extern i2c_t* const I2C2;
extern i2c_t* const I2C3;

#endif /* STM32F2_I2C_HW_H_ */
