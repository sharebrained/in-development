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

#ifndef STM32F2_SDIO_HW_H_
#define STM32F2_SDIO_HW_H_

#include <stdint.h>

typedef struct sdio_t {
	volatile uint32_t POWER;
	volatile uint32_t CLKCR;
	volatile uint32_t ARG;
	volatile uint32_t CMD;
	volatile uint32_t RESPCMD;
	volatile uint32_t RESP1;
	volatile uint32_t RESP2;
	volatile uint32_t RESP3;
	volatile uint32_t RESP4;
	volatile uint32_t DTIMER;
	volatile uint32_t DLEN;
	volatile uint32_t DCTRL;
	volatile uint32_t DCOUNT;
	volatile uint32_t STA;
	volatile uint32_t ICR;
	volatile uint32_t MASK;
	volatile uint32_t _reserved_0x40;
	volatile uint32_t _reserved_0x44;
	volatile uint32_t FIFOCNT;
} sdio_t;

///////////////////////////////////////////////////////////////////
// 0x00 POWER

typedef enum sdio_power_base_t {
	SDIO_POWER_PWRCTRL_BASE = 0,
} sdio_power_pwrctrl_base_t;

///////////////////////////////////////////////////////////////////
// 0x04 CLKCR

typedef enum sdio_clkcr_bit_t {
	SDIO_CLKCR_CLKEN_BIT 	=  8,
	SDIO_CLKCR_PWRSAV_BIT	=  9,
	SDIO_CLKCR_BYPASS_BIT 	= 10,
	SDIO_CLKCR_NEGEDGE_BIT 	= 13,
	SDIO_CLKCR_HWFC_EN_BIT 	= 14,
} sdio_clkcr_bit_t;

typedef enum sdio_clkcr_base_t {
	SDIO_CLKCR_CLKDIV_BASE =  0,
	SDIO_CLKCR_WIDBUS_BASE = 11,
} sdio_clkcr_base_t;

typedef enum sdio_clkcr_clken_t {
	SDIO_CLKCR_CLKEN_DISABLED	= 0,
	SDIO_CLKCR_CLKEN_ENABLED	= 1,
} sdio_clkcr_clken_t;

typedef enum sdio_clkcr_pwrsav_t {
	SDIO_CLKCR_PWRSAVE_CK_ALWAYS	= 0,
	SDIO_CLKCR_PWRSAVE_CK_ACTIVE	= 1,
} sdio_clkcr_pwrsav_t;

typedef enum sdio_clkcr_bypass_t {
	SDIO_CLKCR_BYPASS_DISABLE	= 0,
	SDIO_CLKCR_BYPASS_ENABLE	= 1,
} sdio_clkcr_bypass_t;

typedef enum sdio_clkcr_widbus_t {
	SDIO_CLKCR_WIDBUS_DEFAULT	= 0b00,
	SDIO_CLKCR_WIDBUS_4_WIDE	= 0b01,
	SDIO_CLKCR_WIDBUS_8_WIDE	= 0b10,
} sdio_clkcr_widbus_t;

typedef enum sdio_clkcr_negedge_t {
	SDIO_CLKCR_NEGEDGE_RISING	= 0,
	SDIO_CLKCR_NEGEDGE_FALLING	= 1,
} sdio_clkcr_negedge_t;

typedef enum sdio_clkcr_hwfc_en_t {
	SDIO_CLKCR_HWFC_EN_DISABLED	= 0,
	SDIO_CLKCR_HWFC_EN_ENABLED	= 1,
} sdio_clkcr_hwfc_en_t;

///////////////////////////////////////////////////////////////////
// 0x0C CMD

typedef enum sdio_cmd_bit_t {
	SDIO_CMD_WAITINT_BIT 		=  8,
	SDIO_CMD_WAITPEND_BIT 		=  9,
	SDIO_CMD_CPSMEN_BIT 		= 10,
	SDIO_CMD_SDIOSUSPEND_BIT 	= 11,
	SDIO_CMD_ENCMDCOMPL_BIT 	= 12,
	SDIO_CMD_NIEN_BIT 			= 13,
	SDIO_CMD_CEATACMD_BIT		= 14,
} sdio_cmd_bit_t;

typedef enum sdio_cmd_base_t {
	SDIO_CMD_CMDINDEX_BASE		=  0,
	SDIO_CMD_WAITRESP_BASE		=  6,
} sdio_cmd_base_t;

///////////////////////////////////////////////////////////////////
// 0x10 RESPCMD

typedef enum sdio_respcmd_base_t {
	SDIO_RESPCMD_RESPCMD_BASE	= 0,
} sdio_respcmd_base_t;

///////////////////////////////////////////////////////////////////
// 0x2C DCOUNT

typedef enum sdio_dctrl_bit_t {
	SDIO_DCTRL_DTEN_BIT		=  0,
	SDIO_DCTRL_DTDIR_BIT	=  1,
	SDIO_DCTRL_DTMODE_BIT	=  2,
	SDIO_DCTRL_DMAEN_BIT	=  3,
	SDIO_DCTRL_RWSTART_BIT	=  8,
	SDIO_DCTRL_RWSTOP_BIT	=  9,
	SDIO_DCTRL_RWMOD_BIT	= 10,
	SDIO_DCTRL_SDIOEN_BIT	= 11,
} sdio_dctrl_bit_t;

typedef enum sdio_dctrl_base_t {
	SDIO_DCTRL_DBLOCKSIZE_BASE	= 4,
} sdio_dctrl_base_t;

///////////////////////////////////////////////////////////////////
// 0x34 STA

typedef enum sdio_sta_bit_t {
	SDIO_STA_CCRCFAIL_BIT	=  0,
	SDIO_STA_DCRCFAIL_BIT	=  1,
	SDIO_STA_CTIMEOUT_BIT	=  2,
	SDIO_STA_DTIMEOUT_BIT	=  3,
	SDIO_STA_TXUNDERR_BIT	=  4,
	SDIO_STA_RXOVERR_BIT	=  5,
	SDIO_STA_CMDREND_BIT	=  6,
	SDIO_STA_CMDSENT_BIT	=  7,
	SDIO_STA_DATAEND_BIT	=  8,
	SDIO_STA_STBITERR_BIT	=  9,
	SDIO_STA_DBCKEND_BIT	= 10,
	SDIO_STA_CMDACT_BIT		= 11,
	SDIO_STA_TXACT_BIT		= 12,
	SDIO_STA_RXACT_BIT		= 13,
	SDIO_STA_TXFIFOHE_BIT	= 14,
	SDIO_STA_RXFIFOHF_BIT	= 15,
	SDIO_STA_TXFIFOF_BIT	= 16,
	SDIO_STA_RXFIFOF_BIT	= 17,
	SDIO_STA_TXFIFOE_BIT	= 18,
	SDIO_STA_RXFIFOE_BIT	= 19,
	SDIO_STA_TXDAVL_BIT		= 20,
	SDIO_STA_RXDAVL_BIT		= 21,
	SDIO_STA_SDIOIT_BIT		= 22,
	SDIO_STA_CEATAEND_BIT	= 23,
} sdio_sta_bit_t;

typedef enum sdio_sta_t {
	SDIO_STA_CCRCFAIL 		= 1 << SDIO_STA_CCRCFAIL_BIT,
	SDIO_STA_DCRCFAIL 		= 1 << SDIO_STA_DCRCFAIL_BIT,
	SDIO_STA_CTIMEOUT		= 1 << SDIO_STA_CTIMEOUT_BIT,
	SDIO_STA_DTIMEOUT		= 1 << SDIO_STA_DTIMEOUT_BIT,
	SDIO_STA_TXUNDERR		= 1 << SDIO_STA_TXUNDERR_BIT,
	SDIO_STA_RXOVERR		= 1 << SDIO_STA_RXOVERR_BIT,
	SDIO_STA_CMDREND		= 1 << SDIO_STA_CMDREND_BIT,
	SDIO_STA_CMDSENT		= 1 << SDIO_STA_CMDSENT_BIT,
	SDIO_STA_DATAEND		= 1 << SDIO_STA_DATAEND_BIT,
	SDIO_STA_STBITERR		= 1 << SDIO_STA_STBITERR_BIT,
	SDIO_STA_DBCKEND		= 1 << SDIO_STA_DBCKEND_BIT,
	SDIO_STA_CMDACT			= 1 << SDIO_STA_CMDACT_BIT,
	SDIO_STA_TXACT			= 1 << SDIO_STA_TXACT_BIT,
	SDIO_STA_RXACT			= 1 << SDIO_STA_RXACT_BIT,
	SDIO_STA_TXFIFOHE		= 1 << SDIO_STA_TXFIFOHE_BIT,
	SDIO_STA_RXFIFOHF		= 1 << SDIO_STA_RXFIFOHF_BIT,
	SDIO_STA_TXFIFOF		= 1 << SDIO_STA_TXFIFOF_BIT,
	SDIO_STA_RXFIFOF		= 1 << SDIO_STA_RXFIFOF_BIT,
	SDIO_STA_TXFIFOE		= 1 << SDIO_STA_TXFIFOE_BIT,
	SDIO_STA_RXFIFOE		= 1 << SDIO_STA_RXFIFOE_BIT,
	SDIO_STA_TXDAVL			= 1 << SDIO_STA_TXDAVL_BIT,
	SDIO_STA_RXDAVL			= 1 << SDIO_STA_RXDAVL_BIT,
	SDIO_STA_SDIOIT			= 1 << SDIO_STA_SDIOIT_BIT,
	SDIO_STA_CEATAEND		= 1 << SDIO_STA_CEATAEND_BIT,
} sdio_sta_t;

///////////////////////////////////////////////////////////////////
// 0x38 ICR

typedef enum sdio_icr_bit_t {
	SDIO_ICR_CCRCFAILC_BIT	=  0,
	SDIO_ICR_DCRCFAILC_BIT	=  1,
	SDIO_ICR_CTIMEOUTC_BIT	=  2,
	SDIO_ICR_DTIMEOUTC_BIT	=  3,
	SDIO_ICR_TXUNDERRC_BIT	=  4,
	SDIO_ICR_RXOVERRC_BIT	=  5,
	SDIO_ICR_CMDRENDC_BIT	=  6,
	SDIO_ICR_CMDSENTC_BIT	=  7,
	SDIO_ICR_DATAENDC_BIT	=  8,
	SDIO_ICR_STBITERRC_BIT	=  9,
	SDIO_ICR_DBCKENDC_BIT	= 10,
	SDIO_ICR_SDIOITC_BIT	= 22,
	SDIO_ICR_CEATAENDC_BIT	= 23,
} sdio_icr_bit_t;

///////////////////////////////////////////////////////////////////
// 0x3C MASK

typedef enum sdio_mask_bit_t {
	SDIO_MASK_CCRCFAILIE_BIT	=  0,
	SDIO_MASK_DCRCFAILIE_BIT	=  1,
	SDIO_MASK_CTIMEOUTIE_BIT	=  2,
	SDIO_MASK_DTIMEOUTIE_BIT	=  3,
	SDIO_MASK_TXUNDERRIE_BIT	=  4,
	SDIO_MASK_RXOVERRIE_BIT		=  5,
	SDIO_MASK_CMDRENDIE_BIT		=  6,
	SDIO_MASK_CMDSENTIE_BIT		=  7,
	SDIO_MASK_DATAENDIE_BIT		=  8,
	SDIO_MASK_STBITERRIE_BIT	=  9,
	SDIO_MASK_DBCKENDIE_BIT		= 10,
	SDIO_MASK_CMDACTIE_BIT		= 11,
	SDIO_MASK_TXACTIE_BIT		= 12,
	SDIO_MASK_RXACTIE_BIT		= 13,
	SDIO_MASK_TXFIFOHEIE_BIT	= 14,
	SDIO_MASK_RXFIFOHFIE_BIT	= 15,
	SDIO_MASK_TXFIFOFIE_BIT		= 16,
	SDIO_MASK_RXFIFOFIE_BIT		= 17,
	SDIO_MASK_TXFIFOEIE_BIT		= 18,
	SDIO_MASK_RXFIFOEIE_BIT		= 19,
	SDIO_MASK_TXDAVLIE_BIT		= 20,
	SDIO_MASK_RXDAVLIE_BIT		= 21,
	SDIO_MASK_SDIOITIE_BIT		= 22,
	SDIO_MASK_CEATAENDIE_BIT	= 23,
} sdio_mask_bit_t;

///////////////////////////////////////////////////////////////////

#define SDIO_BASE ((sdio_t*)0x40012C00)

extern sdio_t* const SDIO;

#endif /* STM32F2_SDIO_HW_H_ */
