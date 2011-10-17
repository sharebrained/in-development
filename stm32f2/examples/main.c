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

#include <stdint.h>

#include "stm32f2/rcc.h"
#include "stm32f2/gpio.h"
#include "stm32f2/flash.h"
#include "stm32f2/dma.h"
#include "stm32f2/spi.h"
#include "stm32f2/i2c.h"
#include "stm32f2/sdio.h"

#include "wm8731/wm8731.h"

#include <math.h>
#include <string.h>

/* General notes:
 *
 * Avoid accesses to APB registers that are not 32-bit accesses. From the
 * reference manual (ST Doc ID 15403 Rev 3), section 2.1.9 "AHB/APB bridges
 * (APB)":
 *
 * "When a 16- or an 8-bit access is performed on an APB register, the
 * access is transformed into a 32-bit access: the bridge duplicates the
 * 16- or 8-bit data to feed the 32-bit vector."
 *
 * Therefore, depending on compiler behavior, some really weird stuff could
 * happen when using C bit fields. It's probably best to avoid them.
 */

#define SAMPLE_RATE 44100

gpio_pin_t gpio_a4  = { GPIOA_BASE,  4 };
gpio_pin_t gpio_a8  = { GPIOA_BASE,  8 };
gpio_pin_t gpio_a15 = { GPIOA_BASE, 15 };

gpio_pin_t gpio_b3  = { GPIOB_BASE,  3 };
gpio_pin_t gpio_b4  = { GPIOB_BASE,  4 };
gpio_pin_t gpio_b5  = { GPIOB_BASE,  5 };
gpio_pin_t gpio_b8  = { GPIOB_BASE,  8 };
gpio_pin_t gpio_b9  = { GPIOB_BASE,  9 };
gpio_pin_t gpio_b12 = { GPIOB_BASE, 12 };
gpio_pin_t gpio_b13 = { GPIOB_BASE, 13 };
gpio_pin_t gpio_b15 = { GPIOB_BASE, 15 };

gpio_pin_t gpio_c6  = { GPIOC_BASE,  6 };
gpio_pin_t gpio_c8  = { GPIOC_BASE,  8 };
gpio_pin_t gpio_c9  = { GPIOC_BASE,  9 };
gpio_pin_t gpio_c10 = { GPIOC_BASE, 10 };
gpio_pin_t gpio_c11 = { GPIOC_BASE, 11 };
gpio_pin_t gpio_c12 = { GPIOC_BASE, 12 };

gpio_pin_t gpio_d2  = { GPIOD_BASE,  2 };

gpio_pin_t* const i2s2_ws_pin = &gpio_b12;
gpio_pin_t* const i2s2_sck_pin = &gpio_b13;
gpio_pin_t* const i2s2_sd_pin = &gpio_b15;
gpio_pin_t* const i2s2_mck_pin = &gpio_c6;

gpio_pin_t* const i2s3_ws_pin = &gpio_a4;
gpio_pin_t* const i2s3_sck_pin = &gpio_b3;
gpio_pin_t* const i2s3_sd_pin = &gpio_b5;

gpio_pin_t* const i2c1_scl_pin = &gpio_b8;
gpio_pin_t* const i2c1_sda_pin = &gpio_b9;

gpio_pin_t* const sdio_cmd_pin = &gpio_d2;
gpio_pin_t* const sdio_ck_pin = &gpio_c12;
gpio_pin_t* const sdio_d0_pin = &gpio_c8;
gpio_pin_t* const sdio_d1_pin = &gpio_c9;
gpio_pin_t* const sdio_d2_pin = &gpio_c10;
gpio_pin_t* const sdio_d3_pin = &gpio_c11;

gpio_pin_t* const core_activity_pin = &gpio_a8;

spi_t* const i2s_input = SPI2_BASE;
spi_t* const i2s_output = SPI3_BASE;
i2c_t* const i2c_audio_codec = I2C1_BASE;
wm8731_t* const audio_codec = &wm8731;
dma_stream_t* const audio_in_dma_stream = &DMA1_BASE->S[3];
dma_stream_t* const audio_out_dma_stream = &DMA1_BASE->S[7];
sdio_t* const sdio = SDIO_BASE;

static void sdio_signal_test() {
	gpio_set_mode(sdio_cmd_pin, GPIO_MODER_OUTPUT);
	gpio_set_mode(sdio_ck_pin, GPIO_MODER_OUTPUT);
	gpio_set_mode(sdio_d0_pin, GPIO_MODER_OUTPUT);
	gpio_set_mode(sdio_d1_pin, GPIO_MODER_OUTPUT);
	gpio_set_mode(sdio_d2_pin, GPIO_MODER_OUTPUT);
	gpio_set_mode(sdio_d3_pin, GPIO_MODER_OUTPUT);

	while(1) {
		gpio_set_output(sdio_cmd_pin, 1);
		gpio_set_output(sdio_ck_pin, 1);
		gpio_set_output(sdio_d0_pin, 1);
		gpio_set_output(sdio_d1_pin, 1);
		gpio_set_output(sdio_d2_pin, 1);
		gpio_set_output(sdio_d3_pin, 1);

		gpio_set_output(sdio_cmd_pin, 0);
		gpio_set_output(sdio_ck_pin, 0);
		gpio_set_output(sdio_d0_pin, 0);
		gpio_set_output(sdio_d1_pin, 0);
		gpio_set_output(sdio_d2_pin, 0);
		gpio_set_output(sdio_d3_pin, 0);
	}
}

typedef struct sdio_response_r1_t {
	uint32_t value;
} sdio_response_r1_t;

typedef struct sdio_response_r2_t {
	uint32_t value[4];
} sdio_response_r2_t;

typedef struct sdio_response_r3_t {
	uint32_t value;
} sdio_response_r3_t;

typedef struct sdio_response_r6_t {
	uint32_t value;
} sdio_response_r6_t;

typedef struct sdio_response_r7_t {
	uint32_t value;
} sdio_response_r7_t;

typedef struct sdio_card_status_t {
	uint32_t value;
} sdio_card_status_t;

typedef struct sdio_ocr_t {
	uint32_t value;
} sdio_ocr_t;

typedef struct sdio_cid_t {
	uint32_t value[4];
} sdio_cid_t;

typedef struct sdio_csd_t {
	uint32_t value[4];
} sdio_csd_t;

uint_fast8_t sdio_card_status_get_app_cmd(const sdio_card_status_t* const card_status) {
	return (card_status->value >> 5) & 1;
}

void sdio_get_response_r1(sdio_t* const sdio, sdio_response_r1_t* const r1) {
	r1->value = sdio->RESP1;
}

void sdio_response_r1_get_card_status(const sdio_response_r1_t* const r1, sdio_card_status_t* const card_status) {
	card_status->value = r1->value;
}

void sdio_get_response_r2(sdio_t* const sdio, sdio_response_r2_t* const r2) {
	r2->value[0] = sdio->RESP1;
	r2->value[1] = sdio->RESP2;
	r2->value[2] = sdio->RESP3;
	r2->value[3] = sdio->RESP4;
}

void sdio_response_r2_get_cid(const sdio_response_r2_t* const r2, sdio_cid_t* const cid) {
	cid->value[0] = r2->value[0];
	cid->value[1] = r2->value[1];
	cid->value[2] = r2->value[2];
	cid->value[3] = r2->value[3];
}

void sdio_response_r2_get_csd(const sdio_response_r2_t* const r2, sdio_csd_t* const csd) {
	csd->value[0] = r2->value[0];
	csd->value[1] = r2->value[1];
	csd->value[2] = r2->value[2];
	csd->value[3] = r2->value[3];
}

uint32_t sdio_cid_get_product_serial_number(const sdio_cid_t* const cid) {
	return ((cid->value[2] & 0xFFFFFF) << 8) | ((cid->value[3] >> 24) & 0xFF);
}

uint_fast16_t sdio_cid_get_oem_application_id(const sdio_cid_t* const cid) {
	return (cid->value[0] >>  8) & 0xFFFF;
}

uint_fast16_t sdio_cid_get_manufacturing_date(const sdio_cid_t* const cid) {
	return (cid->value[3] >> 8) & 0xFFF;
}

void sdio_cid_get_product_name(const sdio_cid_t* const cid, uint8_t* const product_name) {
	product_name[0] = (cid->value[0] >>  0) & 0xFF;
	product_name[1] = (cid->value[1] >> 24) & 0xFF;
	product_name[2] = (cid->value[1] >> 16) & 0xFF;
	product_name[3] = (cid->value[1] >>  8) & 0xFF;
	product_name[4] = (cid->value[1] >>  0) & 0xFF;
}

uint_fast8_t sdio_cid_get_manufacturer_id(const sdio_cid_t* const cid) {
	return (cid->value[0] >> 24) & 0xFF;
}

uint_fast8_t sdio_cid_get_product_revision(const sdio_cid_t* const cid) {
	return (cid->value[2] >> 24) & 0xFF;
}

uint_fast8_t sdio_cid_get_crc7_checksum(const sdio_cid_t* const cid) {
	return (cid->value[3] >> 1) & 0x7F;
}

void sdio_get_response_r3(sdio_t* const sdio, sdio_response_r3_t* const r3) {
	r3->value = sdio->RESP3;
}

void sdio_response_r3_get_ocr(const sdio_response_r3_t* const r3, sdio_ocr_t* const ocr) {
	ocr->value = r3->value;
}

uint_fast8_t sdio_ocr_get_card_power_up_status(const sdio_ocr_t* const ocr) {
	return (ocr->value >> 31) & 1;
}

uint_fast8_t sdio_ocr_get_card_capacity_status(const sdio_ocr_t* const ocr) {
	return (ocr->value >> 30) & 1;
}

uint_fast8_t sdio_ocr_get_card_switching_to_1v8_accepted(const sdio_ocr_t* const ocr) {
	return (ocr->value >> 24) & 1;
}

uint_fast32_t sdio_ocr_get_vdd_voltage_window(const sdio_ocr_t* const ocr) {
	return (ocr->value >> 8) & 0xFFFF;
}

void sdio_get_response_r6(sdio_t* const sdio, sdio_response_r6_t* const r6) {
	r6->value = sdio->RESP1;
}

uint_fast16_t sdio_response_r6_get_rca(const sdio_response_r6_t* const r6) {
	return (r6->value >> 16) & 0xFFFF;
}

uint_fast8_t sdio_response_r6_get_card_status_com_crc_error(const sdio_response_r6_t* const r6) {
	return (r6->value >> 15) & 1;
}

uint_fast8_t sdio_response_r6_get_card_status_illegal_command(const sdio_response_r6_t* const r6) {
	return (r6->value >> 14) & 1;
}

uint_fast8_t sdio_response_r6_get_card_status_error(const sdio_response_r6_t* const r6) {
	return (r6->value >> 13) & 1;
}

uint_fast8_t sdio_response_r6_get_card_status_current_state(const sdio_response_r6_t* const r6) {
	return (r6->value >> 9) & 0xF;
}

uint_fast8_t sdio_response_r6_get_card_status_ready_for_data(const sdio_response_r6_t* const r6) {
	return (r6->value >> 8) & 1;
}

uint_fast8_t sdio_response_r6_get_card_status_app_cmd(const sdio_response_r6_t* const r6) {
	return (r6->value >> 5) & 1;
}

uint_fast8_t sdio_response_r6_get_card_status_ake_seq_error(const sdio_response_r6_t* const r6) {
	return (r6->value >> 3) & 1;
}

void sdio_get_response_r7(sdio_t* const sdio, sdio_response_r7_t* const r7) {
	r7->value = sdio->RESP1;
}

uint_fast8_t sdio_response_r7_get_voltage_accepted(const sdio_response_r7_t* const r7) {
	return (r7->value >> 8) & 0xF;
}

uint_fast8_t sdio_response_r7_get_echo_back_of_check_pattern(const sdio_response_r7_t* const r7) {
	return (r7->value >> 0) & 0xFF;
}

sdio_sta_t sdio_wait_for_status(sdio_t* const sdio, const sdio_sta_t flags) {
	while( 1 ) {
		const sdio_sta_t sta_masked = sdio_status(sdio, flags);
		if( sta_masked != 0 ) {
			sdio_clear_static_flags(sdio);
			return sta_masked;
		}
	}
}

typedef enum sdio_command_status_t {
	SDIO_COMMAND_STATUS_OK,
	SDIO_COMMAND_STATUS_COMMAND_CRC_FAILED,
	SDIO_COMMAND_STATUS_COMMAND_TIMEOUT,
	SDIO_COMMAND_STATUS_ERROR,
} sdio_command_status_t;

static sdio_command_status_t get_command_status(const sdio_sta_t flags) {
	sdio_command_status_t command_status = SDIO_COMMAND_STATUS_ERROR;

	if( flags & SDIO_STA_CMDREND ) {
		command_status = SDIO_COMMAND_STATUS_OK;
	}
	if( flags & SDIO_STA_CCRCFAIL ) {
		command_status = SDIO_COMMAND_STATUS_COMMAND_CRC_FAILED;
	}
	if( flags & SDIO_STA_CTIMEOUT ) {
		command_status = SDIO_COMMAND_STATUS_COMMAND_TIMEOUT;
	}

	return command_status;
}

sdio_command_status_t sdio_send_no_response_command(sdio_t* const sdio, const uint_fast8_t command_index, const uint32_t argument) {
	// TODO: Implement timeout counter -- equivalent to CTIMEOUT status or not?
	sdio->ARG = argument;
	sdio->CMD = (sdio->CMD & 0xFFFFF800) |
				(1 << SDIO_CMD_CPSMEN_BIT) |
				(0 << SDIO_CMD_WAITPEND_BIT) |
				(0 << SDIO_CMD_WAITINT_BIT) |
			    (0b00 << SDIO_CMD_WAITRESP_BASE) |
			    (command_index << SDIO_CMD_CMDINDEX_BASE);
	const sdio_sta_t sta_masked = sdio_wait_for_status(sdio, SDIO_STA_CMDSENT);
	return get_command_status(sta_masked);
}

sdio_command_status_t sdio_send_short_command(sdio_t* const sdio, const uint_fast8_t command_index, const uint32_t argument) {
	// TODO: Implement timeout counter -- equivalent to CTIMEOUT status or not?
	sdio->ARG = argument;
	sdio->CMD = (sdio->CMD & 0xFFFFF800) |
				(1 << SDIO_CMD_CPSMEN_BIT) |
				(0 << SDIO_CMD_WAITPEND_BIT) |
				(0 << SDIO_CMD_WAITINT_BIT) |
			    (0b01 << SDIO_CMD_WAITRESP_BASE) |
			    (command_index << SDIO_CMD_CMDINDEX_BASE);
	const sdio_sta_t sta_masked = sdio_wait_for_status(sdio, SDIO_STA_CCRCFAIL |
															 SDIO_STA_CMDREND |
															 SDIO_STA_CTIMEOUT);
	return get_command_status(sta_masked);
}

sdio_command_status_t sdio_send_long_command(sdio_t* const sdio, const uint_fast8_t command_index, const uint32_t argument) {
	// TODO: Implement timeout counter -- equivalent to CTIMEOUT status or not?
	sdio->ARG = argument;
	sdio->CMD = (sdio->CMD & 0xFFFFF800) |
				(1 << SDIO_CMD_CPSMEN_BIT) |		// Enable command path state machine
				(0 << SDIO_CMD_WAITPEND_BIT) |		// Don't wait for data transfer end before sending command
				(0 << SDIO_CMD_WAITINT_BIT) |		// Use command timeout
				(0b11 << SDIO_CMD_WAITRESP_BASE) |	// Long response
				(command_index << SDIO_CMD_CMDINDEX_BASE);
	const sdio_sta_t sta_masked = sdio_wait_for_status(sdio, SDIO_STA_CCRCFAIL |
			                 	 	 	 	 	 	 	 	 SDIO_STA_CMDREND |
			                 	 	 	 	 	 	 	 	 SDIO_STA_CTIMEOUT);
	return get_command_status(sta_masked);
}

sdio_command_status_t sdio_send_cmd0_go_idle_state(sdio_t* const sdio) {
	// TODO: (and choose bus interface type with CS signal? SPI or SD?)
	return sdio_send_no_response_command(sdio, 0, 0);
}

sdio_command_status_t sdio_send_cmd8_send_interface_condition(sdio_t* const sdio) {
	bool card_present = 0;
	bool card_unusable = 0;
	bool card_supports_2_0 = 0;

	// Send Interface Condition Command (CMD8)
	// Physical Layer simplified spec v3.01, section 4.3.13
	// and figure 4-2.
	// CMD8 enables additional functionality in newer cards,
	// including support for ACMD41.
	const sdio_command_status_t command_status = sdio_send_short_command(sdio, 0b001000, (0x00000    << 12) |	// reserved
																					   	 (0b0001     <<  8) |	// VHS = 2.7V to 3.6V
																					     (0b10101010 <<  0));

	/* The card checks whether it can operate on the host's supply voltage.
	 * The card that accepted the supplied voltage returns R7 response.
	 * In the response, the card echoes back both the voltage range and check
	 * pattern set in the argument. If the card does not support the host
	 * supply voltage, it shall not return response and stays in Idle state.
	 */
	if( command_status == SDIO_COMMAND_STATUS_COMMAND_TIMEOUT ) {
		/* Version 2.00 or later SD memory card (VHS field voltage mismatch),
		 * Version 1.X SD memory card, or
		 * Not SD memory card.
		 */
		card_present = 1;
		card_supports_2_0 = 0;
		// Need to perform ACMD41 to see if card is usable.
	}

	if( command_status == SDIO_COMMAND_STATUS_OK ) {
		/* Version 2.00 or later SD memory card
		 */
		card_present = 1;
		card_supports_2_0 = 1;
		sdio_response_r7_t response_r7;
		sdio_get_response_r7(sdio, &response_r7);
		if( sdio_response_r7_get_voltage_accepted(&response_r7) != 0b0001 ) {
			// VHS value changed, which should not happen.
			card_unusable = 1;
			return SDIO_COMMAND_STATUS_ERROR;
		}
		if( sdio_response_r7_get_echo_back_of_check_pattern(&response_r7) != 0b10101010 ) {
			// Check pattern incorrect.
			card_unusable = 1;
			return SDIO_COMMAND_STATUS_ERROR;
		}
	}
	// Valid response should contain same command index and arg value.
	// No response if device does not support host voltage.
	// TODO: CMD8 can be rejected (illegal command error 0x05)
	// indicating card is SD v1 or MMC.

	return command_status;
}

sdio_command_status_t sdio_send_cmd55_app_cmd(sdio_t* const sdio) {
	const sdio_command_status_t command_status = sdio_send_short_command(sdio, 55, 0);

	if( command_status == SDIO_COMMAND_STATUS_OK ) {
		// CMD55: response type 1
		sdio_response_r1_t response_r1;
		sdio_get_response_r1(sdio, &response_r1);
		sdio_card_status_t card_status;
		sdio_response_r1_get_card_status(&response_r1, &card_status);
		if( sdio_card_status_get_app_cmd(&card_status) != 1 ) {
			// APP_CMD bit is not set, so something is wrong.
			return SDIO_COMMAND_STATUS_ERROR;
		}
	}

	return command_status;
}

sdio_command_status_t sdio_send_acmd41_set_operational_conditions(sdio_t* const sdio, sdio_ocr_t* const ocr) {
	sdio_command_status_t command_status = sdio_send_cmd55_app_cmd(sdio);
	if( command_status == SDIO_COMMAND_STATUS_OK ) {
		command_status = sdio_send_short_command(sdio, 41, (0 << 31) |	// Busy
														   (0 << 30) |	// HCS (host capacity support)
														   (0 << 29) |	// FB
														   (0 << 28) |	// XPC (SDXC power control)
														   (0b000 << 25) |	// reserved
														   (0 << 24) |	// S18R (switching to 1.8V request)
														   (0b001100000 << 15)	// OCR: 3.3-3.4, 3.2-3.3)
														   );
		// TODO: CCRCFAIL is OK here? Why? It seems CRC7 is forced to 0b1111111.
		if( command_status == SDIO_COMMAND_STATUS_OK ) {
			sdio_response_r3_t response_r3;
			sdio_get_response_r3(sdio, &response_r3);
			sdio_response_r3_get_ocr(&response_r3, ocr);
		}
	}

	return command_status;
}

sdio_command_status_t sdio_send_cmd2_all_send_cid(sdio_t* const sdio, sdio_response_r2_t* const response_r2) {
	const sdio_command_status_t command_status = sdio_send_long_command(sdio, 2, 0);
	if( command_status == SDIO_COMMAND_STATUS_OK ) {
		sdio_get_response_r2(sdio, &response_r2);
	}

	return command_status;
}

sdio_command_status_t sdio_send_cmd3_send_relative_address(sdio_t* const sdio, sdio_response_r6_t* const response_r6) {
	const sdio_command_status_t command_status = sdio_send_short_command(sdio, 3, 0);
	if( command_status == SDIO_COMMAND_STATUS_OK ) {
		if( (sdio->RESPCMD & 0x3F) != 3 ) {
			return SDIO_COMMAND_STATUS_ERROR;
		}
		sdio_get_response_r6(sdio, &response_r6);
	}

	return command_status;
}

sdio_command_status_t sdio_send_cmd9_send_csd(sdio_t* const sdio, const uint_fast16_t card_rca, sdio_response_r2_t* const response_r2) {
	const sdio_command_status_t command_status = sdio_send_long_command(sdio, 9, card_rca << 16);
	if( command_status == SDIO_COMMAND_STATUS_OK ) {
		sdio_get_response_r2(sdio, &response_r2);
	}

	return command_status;
}

sdio_command_status_t 	sdio_send_cmd17_read_single_block(sdio_t* const sdio, const uint_fast32_t read_address, uint8_t* const target, sdio_response_r1_t* const response_r1) {
	const sdio_command_status_t command_status = sdio_send_short_command(sdio, 17, read_address);
	if( command_status == SDIO_COMMAND_STATUS_OK ) {

	}
	return command_status;
}

static sdio_command_status_t initialize_sdio(sdio_t* const sdio) {
	sdio_command_status_t command_status = SDIO_COMMAND_STATUS_ERROR;

	command_status = sdio_send_cmd0_go_idle_state(sdio);
	if( command_status != SDIO_COMMAND_STATUS_OK ) {
		return command_status;
	}

	command_status = sdio_send_cmd8_send_interface_condition(sdio);
	if( command_status != SDIO_COMMAND_STATUS_OK ) {
		return command_status;
	}

	sdio_ocr_t ocr;
	bool card_initialization_complete = 0;
	while( card_initialization_complete != 1 ) {
		/* Physical spec 3.01 section 4.2.3 Application Notes:
		 * The host shall set ACMD41 timeout more than 1 second to abort
		 * repeat of issuing ACMD41 when the card does not indicate ready.
		 * The timeout count starts from the first ACMD41 which is set
		 * voltage window in the argument. */
		// TODO: implement one-second timeout.
		/*if( timeout ) {
			return SDIO_COMMAND_STATUS_ERROR;
		}*/

		command_status = sdio_send_acmd41_set_operational_conditions(sdio, &ocr);
		if( command_status != SDIO_COMMAND_STATUS_OK ) {
			return command_status;
		}

		card_initialization_complete = sdio_ocr_get_card_power_up_status(&ocr);
	}
	const uint_fast8_t card_capacity_status = sdio_ocr_get_card_capacity_status(&ocr);
	const uint_fast8_t card_switching_to_1v8_accepted = sdio_ocr_get_card_switching_to_1v8_accepted(&ocr);
	const uint_fast32_t card_vdd_voltage_window = sdio_ocr_get_vdd_voltage_window(&ocr);

	//SD_InitializeCards

	sdio_response_r2_t response_r2;
	command_status = sdio_send_cmd2_all_send_cid(sdio, &response_r2);
	if( command_status != SDIO_COMMAND_STATUS_OK ) {
		return command_status;
	}
	// Response[127:1]0b:
	// 0x02544d53	00000010 01010100 01001101 01010011
	// 0x44303247	01000100 00110000 00110010 01000111
	// 0x60a15b15	01100000 10100001 01011011 00010101
	// 0x8e00a464	10001110 00000000 10100100 0110010X

	sdio_response_r6_t response_r6;
	command_status = sdio_send_cmd3_send_relative_address(sdio, &response_r6);
	if( command_status != SDIO_COMMAND_STATUS_OK ) {
		return command_status;
	}
	// 01010011 01100000 00000101 00100000
	const uint32_t card_rca = sdio_response_r6_get_rca(&response_r6);
	const uint_fast8_t card_status_com_crc_error = sdio_response_r6_get_card_status_com_crc_error(&response_r6);
	const uint_fast8_t card_status_illegal_command = sdio_response_r6_get_card_status_illegal_command(&response_r6);
	const uint_fast8_t card_status_error = sdio_response_r6_get_card_status_error(&response_r6);
	const uint_fast8_t card_status_current_state = sdio_response_r6_get_card_status_current_state(&response_r6);
	const uint_fast8_t card_status_ready_for_data = sdio_response_r6_get_card_status_ready_for_data(&response_r6);
	const uint_fast8_t card_status_app_cmd = sdio_response_r6_get_card_status_app_cmd(&response_r6);
	const uint_fast8_t card_status_ake_seq_error = sdio_response_r6_get_card_status_ake_seq_error(&response_r6);
	if( card_status_com_crc_error | card_status_illegal_command | card_status_error ) {
		return SDIO_COMMAND_STATUS_ERROR;
	}

	command_status = sdio_send_cmd9_send_csd(sdio, card_rca, &response_r2);
	if( command_status != SDIO_COMMAND_STATUS_OK ) {
		return command_status;
	}
	sdio_csd_t csd;
	sdio_response_r2_get_csd(&response_r2, &csd);
	// Response[127:1]0b:
	// 0x002e0032	00000000 00101110 00000000 00110010
	// 0x5b5a83a9	01011011 01011010 10000011 10101001
	// 0xffffff80	11111111 11111111 11111111 10000000
	// 0x16800090	00010110 10000000 00000000 10010000

	//SDIO_Init
	//SD_GetCardInfo
	//SD_SelectDeselect
	//SD_EnableWideBusOperation

// NOTE: TESTING TESTING TESTING
	sdio_send_cmd17_read_single_block(sdio, read_address, target);

	return SDIO_COMMAND_STATUS_OK;
}

int32_t cos_lut_q24_x[256];
void fill_cos_lut_q24() {
	uint_fast16_t i;
	for(i=0; i<256; i++) {
		cos_lut_q24_x[i] = round(cos(2.0 * M_PI * (double)i / 256.0) * 16777216.0);
	}
}

static int32_t cos_lut_interp_q24(const uint32_t theta_q32) {
	const uint32_t frac_q24 = theta_q32 & 0xFFFFFF;
	const uint32_t index = theta_q32 >> 24;
	const int32_t v1_q24 = cos_lut_q24_x[index];
	const int32_t v2_q24 = cos_lut_q24_x[(index + 1) & 0xFF];
	const int32_t vdiff_q24 = v2_q24 - v1_q24;
	const int64_t vfrac_q48 = (int64_t)vdiff_q24 * (int64_t)frac_q24;
	const int32_t vfrac_q24 = vfrac_q48 >> 24; // / 16777216;
	const int32_t v_q24 = v1_q24 + vfrac_q24;
	return v_q24;
}

static int32_t cos_q24(const uint32_t theta_q32) {
	return cos_lut_interp_q24(theta_q32);
}

typedef struct {
	uint32_t phase_q32;
	uint32_t phase_increment_q32;
} oscillator_t;

static uint32_t phasor(oscillator_t* const o) {
	const uint32_t phase_q32 = o->phase_q32;
	o->phase_q32 = phase_q32 + o->phase_increment_q32;
	return phase_q32;
}

static int32_t osc(oscillator_t* const o) {
	const uint32_t phase_q32 = o->phase_q32;
	o->phase_q32 = phase_q32 + o->phase_increment_q32;
	return cos_q24(phase_q32);
}
/*
int32_t sine_table_interp(oscillator_t* const o) {
	const uint32_t phase_q32 = o->phase_q32;
	o->phase_q32 = phase_q32 + o->phase_increment_q32;

	const uint32_t phase_frac_q16 = (phase_q32 >> 8) & 0xFFFF;
	const uint32_t index_lo = phase_q32 >> 24;
	const uint32_t index_hi = (index_lo + 1) & 0xFF;
	const int32_t y1 = sine_lut_q24[index_lo];
	const int32_t y2 = sine_lut_q24[index_hi];
	const int32_t dy12 = y2 - y1;
	const int32_t dy = (dy12 * phase_frac_q16) / 65536;
	return y1 + dy;
}
*/

static oscillator_t osc1 = {
	.phase_q32 = 0,
	.phase_increment_q32 = 42852281,	// 440Hz
};

static oscillator_t osc2 = {
	.phase_q32 = 0,
	.phase_increment_q32 = 42852281 / 2,	// 220Hz
};

static oscillator_t osc3 = {
	.phase_q32 = 0,
	.phase_increment_q32 = 64278422 / 2,	// 3300Hz
};

static oscillator_t osc4 = {
	.phase_q32 = 0,
	.phase_increment_q32 = 24348		// 0.25Hz
};

static oscillator_t osc5 = {
	.phase_q32 = 0,
	.phase_increment_q32 = 13913		// 0.142Hz
};

static int32_t generate_sample() {
	const uint32_t theta_q32 = phasor(&osc1);

	const int32_t osc4_q24 = osc(&osc4);					// -2^24 to +2^24
	const uint32_t osc4_offset_q25 = osc4_q24 + (1 << 24);	//     0 to +2^25

	const int32_t osc2_q24 = osc(&osc2);
	const int32_t theta_osc2_q32 = osc2_q24 * (osc4_offset_q25 / 131072);

	const int32_t osc5_q24 = osc(&osc5);					// -2^24 to +2^24
	const uint32_t osc5_offset_q25 = osc5_q24 + (1 << 24);	//     0 to +2^25

	const int32_t osc3_q24 = osc(&osc3);
	const int32_t theta_osc3_q32 = osc3_q24 * (osc5_offset_q25 / 131072);

	const int32_t v_q24 = cos_q24(theta_q32 + theta_osc2_q32 + theta_osc3_q32);
	const int32_t result = v_q24 * 127;
	return result;
}

static int16_t adc_buffer[512];
static int16_t dac_buffer[512];

int main() {
	///////////////////////////////////////////////////////////////////
	// Clock config
	///////////////////////////////////////////////////////////////////

	// Turn on high-speed internal oscillator (16MHz)
	rcc_enable_oscillator(RCC_OSCILLATOR_HSI);
	rcc_wait_for_oscillator_ready(RCC_OSCILLATOR_HSI);

	// Switch to HSI for system, host, peripheral, RTC clocks
	rcc_set_system_clock(RCC_SYSTEM_OSCILLATOR_HSI);
	rcc_wait_for_system_clock_switch(RCC_SYSTEM_OSCILLATOR_HSI);

	// Make sure we're using crystal oscillator circuit, not just external input.
	rcc_hse_bypass_off();

	rcc_css_disable();

	// Turn off PLLs for imminent configuration
	rcc_disable_oscillator(RCC_OSCILLATOR_PLL);
	rcc_disable_oscillator(RCC_OSCILLATOR_PLLI2S);

	// Turn on high-speed external oscillator (25MHz)
	rcc_enable_oscillator(RCC_OSCILLATOR_HSE);
	rcc_wait_for_oscillator_ready(RCC_OSCILLATOR_HSE);

	//RCC->CIR = 0;

	// TODO: Set RTCSEL

	// Configure and start system VCO and peripheral clocks

	//  m   n  q usb_clock     n r div o    fs
	// 20 192  5  48000000   289 2   4 0 44097.900
	// 20 192  5  48000000   289 4   2 0 44097.900
	// PLL clock input = HSE = 25MHz
	// VCO = clock input * N / M = 25MHz * 192 / 20 = 240MHz
	// general clock output = VCO / P = 240MHz / 2 = 120MHz
	// USB/SDIO/RNG clock = VCO / Q = 240MHz / 5 = 48MHz
	// HPRE = 0 (divide by 1 = 120MHz)
	// PPRE1 = 5 (divide by 4 = 30MHz)
	// PPRE2 = 4 (divide by 2 = 60MHz)
	rcc_set_cfgr(RCC_CFGR_MCO2_SYSCLK,
			 	 RCC_CFGR_MCO2PRE_NO_DIVISION,
			 	 RCC_CFGR_MCO1PRE_NO_DIVISION,
			 	 RCC_CFGR_I2SSRC_PLLI2S,
			 	 RCC_CFGR_MCO1_HSI,
			 	 25, 4, 5,
			 	 RCC_CFGR_HPRE_NOT_DIVIDED,
			 	 RCC_CFGR_SW_HSI);

	rcc_set_pllcfgr(5, RCC_PLLCFGR_PLLSRC_HSE, RCC_PLLCFGR_PLLP_2, 192, 20);

	rcc_enable_oscillator(RCC_OSCILLATOR_PLL);
	rcc_wait_for_oscillator_ready(RCC_OSCILLATOR_PLL);

	// Reconfigure flash wait states to accommodate new CPU clock speed
	// Procedure from reference manual 2.3.4 "Flash memory read interface"
	flash_set_latency(3);
	// TODO: Check FLASH_ACR has correct latency?
	flash_enable_prefetch();
	flash_enable_instruction_cache();
	flash_enable_data_cache();

	// Switch CPU to use PLL clock
	rcc_set_system_clock(RCC_SYSTEM_OSCILLATOR_PLL);
	rcc_wait_for_system_clock_switch(RCC_SYSTEM_OSCILLATOR_PLL);

	///////////////////////////////////////////////////////////////////
	// I2S VCO
	///////////////////////////////////////////////////////////////////

	// I2S VCO = clock input / M * I2S_N = 25MHz / 20 * 289 = 361.25MHz
	// I2S clock output = I2S_VCO / I2S_R = 361.25MHz / 4 = 90.3125MHz
	rcc_set_plli2scfgr(4, 289);

	rcc_enable_oscillator(RCC_OSCILLATOR_PLLI2S);
	rcc_wait_for_oscillator_ready(RCC_OSCILLATOR_PLLI2S);

	///////////////////////////////////////////////////////////////////
	// GPIO
	///////////////////////////////////////////////////////////////////

	rcc_enable_peripheral(PERIPHERAL_GPIOA);
	rcc_reset_peripheral(PERIPHERAL_GPIOA);

	rcc_enable_peripheral(PERIPHERAL_GPIOB);
	rcc_reset_peripheral(PERIPHERAL_GPIOB);

	rcc_enable_peripheral(PERIPHERAL_GPIOC);
	rcc_reset_peripheral(PERIPHERAL_GPIOC);

	rcc_enable_peripheral(PERIPHERAL_GPIOD);
	rcc_reset_peripheral(PERIPHERAL_GPIOD);

	// TODO: Repurpose unused JTAG pins. Leave A13 (SWDIO) and A14 (SWCLK) alone.
	/*
	gpio_set_mode(&gpio_a15, GPIO_MODER_INPUT);
	gpio_set_mode(&gpio_b3, GPIO_MODER_INPUT);
	gpio_set_mode(&gpio_b4, GPIO_MODER_INPUT);
	*/

	// TODO: Disable unused 32K oscillator pins?

	// TODO: Enable I/O compensation cell?

	///////////////////////////////////////////////////////////////////
	// SPI2/I2S2
	///////////////////////////////////////////////////////////////////

	rcc_enable_peripheral(PERIPHERAL_SPI2);
	rcc_reset_peripheral(PERIPHERAL_SPI2);

	// I2S clock = 90.3125MHz
	// I2S2: ADC (receive) interface, master
	spi_disable(i2s_input);
	i2s_disable(i2s_input);

	spi_set_i2spr(i2s_input,
				  SPI_MASTER_CLOCK_OUTPUT_ENABLED,
				  SPI_PRESCALER_FACTOR_EVEN,
				  4);

	spi_set_i2scfgr(i2s_input,
					SPI_MODE_I2S,
			        I2S_DISABLED,
			        I2S_CONFIG_MASTER_RECEIVE,
			        I2S_PCMSYNC_SHORT_FRAME,
			        I2S_STANDARD_PHILIPS,
			        I2S_CLOCK_STEADY_STATE_LOW,
			        I2S_DATA_TRANSFER_LENGTH_32_BITS,
			        I2S_CHANNEL_LENGTH_32_BITS
			        );

	// PB12: I2S2_WS: ADCLRC
	gpio_set_mode(i2s2_ws_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(i2s2_ws_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(i2s2_ws_pin, GPIO_OSPEEDR_25MHZ);
	gpio_set_alternate_function(i2s2_ws_pin, GPIO_AF_SPI1_2_I2S2);

	// PB13: I2S2_SCK: BCLK (out)
	gpio_set_mode(i2s2_sck_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(i2s2_sck_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(i2s2_sck_pin, GPIO_OSPEEDR_25MHZ);
	gpio_set_alternate_function(i2s2_sck_pin, GPIO_AF_SPI1_2_I2S2);

	// PB15: I2S2_SD: ADCDAT (in)
	gpio_set_mode(i2s2_sd_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(i2s2_sd_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(i2s2_sd_pin, GPIO_OSPEEDR_25MHZ);
	gpio_set_alternate_function(i2s2_sd_pin, GPIO_AF_SPI1_2_I2S2);

	// PC6: I2S2_MCK: MCLK (out)
	gpio_set_mode(i2s2_mck_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(i2s2_mck_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(i2s2_mck_pin, GPIO_OSPEEDR_25MHZ);
	gpio_set_alternate_function(i2s2_mck_pin, GPIO_AF_SPI1_2_I2S2);

	///////////////////////////////////////////////////////////////////
	// SPI3/I2S3
	///////////////////////////////////////////////////////////////////

	rcc_enable_peripheral(PERIPHERAL_SPI3);
	rcc_reset_peripheral(PERIPHERAL_SPI3);

	// I2S clock = 90.3125MHz
	// I2S3: DAC (transmit) interface, slaved to I2S2 interface
	spi_disable(i2s_output);
	i2s_disable(i2s_output);

	spi_set_i2spr(i2s_output,
				  SPI_MASTER_CLOCK_OUTPUT_DISABLED,
				  SPI_PRESCALER_FACTOR_EVEN,
				  4);

	spi_set_i2scfgr(i2s_output,
					SPI_MODE_I2S,
			        I2S_DISABLED,
			        I2S_CONFIG_SLAVE_TRANSMIT,
			        I2S_PCMSYNC_SHORT_FRAME,
			        I2S_STANDARD_PHILIPS,
			        I2S_CLOCK_STEADY_STATE_LOW,
			        I2S_DATA_TRANSFER_LENGTH_32_BITS,
			        I2S_CHANNEL_LENGTH_32_BITS
			        );

	// PA4: I2S3_WS: DACLRC
	gpio_set_mode(i2s3_ws_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(i2s3_ws_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(i2s3_ws_pin, GPIO_OSPEEDR_25MHZ);
	gpio_set_alternate_function(i2s3_ws_pin, GPIO_AF_SPI3_I2S3);

	// PB3: I2S3_SCK: BCLK (in)
	gpio_set_mode(i2s3_sck_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(i2s3_sck_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(i2s3_sck_pin, GPIO_OSPEEDR_25MHZ);
	gpio_set_alternate_function(i2s3_sck_pin, GPIO_AF_SPI3_I2S3);

	// PB5: I2S3_SD: DACDAT
	gpio_set_mode(i2s3_sd_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(i2s3_sd_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(i2s3_sd_pin, GPIO_OSPEEDR_25MHZ);
	gpio_set_alternate_function(i2s3_sd_pin, GPIO_AF_SPI3_I2S3);

	///////////////////////////////////////////////////////////////////
	// I2C1
	///////////////////////////////////////////////////////////////////

	rcc_enable_peripheral(PERIPHERAL_I2C1);
	rcc_reset_peripheral(PERIPHERAL_I2C1);

	i2c_disable(i2c_audio_codec);

	// APB clock = 30MHz
	i2c_set_peripheral_clock_frequency(i2c_audio_codec, 30);
	i2c_set_interface_mode(i2c_audio_codec, I2C_INTERFACE_MODE_I2C);
	i2c_disable_packet_error_checking(i2c_audio_codec);
	i2c_disable_general_call(i2c_audio_codec);
	i2c_enter_standard_mode(i2c_audio_codec, 0x096, 31);

	gpio_set_mode(i2c1_scl_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(i2c1_scl_pin, GPIO_OTYPER_OPEN_DRAIN);
	gpio_set_output_speed(i2c1_scl_pin, GPIO_OSPEEDR_2MHZ);
	gpio_set_alternate_function(i2c1_scl_pin, GPIO_AF_I2C1_2_3);

	gpio_set_mode(i2c1_sda_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(i2c1_sda_pin, GPIO_OTYPER_OPEN_DRAIN);
	gpio_set_output_speed(i2c1_sda_pin, GPIO_OSPEEDR_2MHZ);
	gpio_set_alternate_function(i2c1_sda_pin, GPIO_AF_I2C1_2_3);

	i2c_enable(i2c_audio_codec);

	///////////////////////////////////////////////////////////////////
	// Audio codec
	///////////////////////////////////////////////////////////////////

	wm8731_reset(audio_codec);
	wm8731_set_digital_audio_interface_format(audio_codec, 0, 0, 0, 0, 0b11, 0b10);
	wm8731_set_sampling_control(audio_codec, 0, 0, 0b1000, 0, 0);
	wm8731_set_analog_audio_path_control(audio_codec, 0, 0, 1, 0, 0, 1, 0);
	wm8731_set_power_down_control(audio_codec, 0, 1, 1, 0, 0, 0, 1, 0);
	wm8731_set_line_in(audio_codec, 0, 0b10111);
	wm8731_set_headphone_out(audio_codec, 1, 0b111001);
	wm8731_set_digital_audio_path_control(audio_codec, 0, 0, 0, 0);
	wm8731_set_active_control(audio_codec, 1);

	///////////////////////////////////////////////////////////////////
	// SDIO
	///////////////////////////////////////////////////////////////////

	//SD_Init
	//SD_LowLevel_Init
	//SDIO_DeInit

	rcc_enable_peripheral(PERIPHERAL_SDIO);
	rcc_reset_peripheral(PERIPHERAL_SDIO);

	gpio_set_mode(sdio_cmd_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(sdio_cmd_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(sdio_cmd_pin, GPIO_OSPEEDR_50MHZ);
	gpio_set_alternate_function(sdio_cmd_pin, GPIO_AF_FSMC_SDIO_OTGFS);

	gpio_set_mode(sdio_ck_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(sdio_ck_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(sdio_ck_pin, GPIO_OSPEEDR_50MHZ);
	gpio_set_alternate_function(sdio_ck_pin, GPIO_AF_FSMC_SDIO_OTGFS);

	gpio_set_mode(sdio_d0_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(sdio_d0_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(sdio_d0_pin, GPIO_OSPEEDR_50MHZ);
	gpio_set_alternate_function(sdio_d0_pin, GPIO_AF_FSMC_SDIO_OTGFS);

	gpio_set_mode(sdio_d1_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(sdio_d1_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(sdio_d1_pin, GPIO_OSPEEDR_50MHZ);
	gpio_set_alternate_function(sdio_d1_pin, GPIO_AF_FSMC_SDIO_OTGFS);

	gpio_set_mode(sdio_d2_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(sdio_d2_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(sdio_d2_pin, GPIO_OSPEEDR_50MHZ);
	gpio_set_alternate_function(sdio_d2_pin, GPIO_AF_FSMC_SDIO_OTGFS);

	gpio_set_mode(sdio_d3_pin, GPIO_MODER_ALTERNATE_FUNCTION);
	gpio_set_output_type(sdio_d3_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(sdio_d3_pin, GPIO_OSPEEDR_50MHZ);
	gpio_set_alternate_function(sdio_d3_pin, GPIO_AF_FSMC_SDIO_OTGFS);

	// TODO: Add GPIO configuration for SDIO presence detect pin

	//SD_PowerON

	// SDIOCLK = 48MHz
	// Configure SDIO to run at 400kHz, initially.
	// SDIO_Init
	sdio_set_clkcr(sdio,
			       SDIO_HARDWARE_FLOW_CONTROL_DISABLED,
				   SDIO_CLOCK_PHASE_SDIOCLK_RISING_EDGE,
				   SDIO_BUS_WIDTH_1_BIT,
				   SDIO_CLOCK_DIVIDER_BYPASS_DISABLED,
				   SDIO_POWER_SAVING_CLOCK_ALWAYS_ENABLED,
				   SDIO_CLOCK_DISABLED,
				   118
			       );

	// SDIO_SetPowerState
	sdio_set_power_control(sdio, SDIO_POWER_ON);

	// SDIO_ClockCmd
	sdio_enable_clock(sdio);

	sdio_command_status_t command_status = initialize_sdio(sdio);

	///////////////////////////////////////////////////////////////////
	// Audio buffers
	///////////////////////////////////////////////////////////////////

	memset(adc_buffer, 0, sizeof(adc_buffer));
	memset(dac_buffer, 0, sizeof(dac_buffer));

	fill_cos_lut_q24();

	///////////////////////////////////////////////////////////////////
	// DMA
	///////////////////////////////////////////////////////////////////

	rcc_enable_peripheral(PERIPHERAL_DMA1);
	rcc_reset_peripheral(PERIPHERAL_DMA1);

	// Audio receive DMA

	// DMA configuration order from reference manual
	// 9.3.17 "Stream configuration procedure"
	dma_stream_disable(audio_in_dma_stream);
	dma_stream_wait_until_disabled(audio_in_dma_stream);

	dma_stream_set_peripheral_address(audio_in_dma_stream, &SPI2->DR);
	dma_stream_set_memory_0_address(audio_in_dma_stream, &adc_buffer[  0]);
	dma_stream_set_memory_1_address(audio_in_dma_stream, &adc_buffer[256]);
	dma_stream_set_item_count(audio_in_dma_stream, 64 * 2 * 2);

	dma_stream_set_cr(audio_in_dma_stream,
					  0,
					  DMA_STREAM_MEMORY_BURST_SINGLE_TRANSFER,
					  DMA_STREAM_PERIPHERAL_BURST_SINGLE_TRANSFER,
					  DMA_STREAM_CURRENT_TARGET_MEMORY_0,
					  DMA_STREAM_DOUBLE_BUFFER_ON,
					  DMA_STREAM_PRIORITY_HIGH,
					  DMA_STREAM_PINCOS_LINKED_TO_PSIZE,
					  DMA_STREAM_MEMORY_DATA_SIZE_HALF_WORD,
					  DMA_STREAM_PERIPHERAL_DATA_SIZE_HALF_WORD,
					  DMA_STREAM_MEMORY_ADDRESS_INCREMENT,
					  DMA_STREAM_PERIPHERAL_ADDRESS_FIXED,
					  DMA_STREAM_CIRCULAR_MODE_ENABLED,
					  DMA_STREAM_DIRECTION_PERIPHERAL_TO_MEMORY,
					  DMA_STREAM_PERIPHERAL_FLOW_CONTROLLER_DMA,
					  0, 0, 0, 0,
					  DMA_STREAM_ENABLED
					  );

	dma_stream_set_fcr(audio_in_dma_stream,
					   0,
					   DMA_STREAM_DIRECT_MODE_ENABLED,
					   DMA_STREAM_FIFO_THRESHOLD_1_2_FULL
					   );

	dma_stream_enable(audio_in_dma_stream);
	spi_enable_rx_dma(i2s_input);

	// Audio transmit DMA

	// DMA configuration order from reference manual
	// 9.3.17 "Stream configuration procedure"
	dma_stream_disable(audio_out_dma_stream);
	dma_stream_wait_until_disabled(audio_out_dma_stream);

	dma_stream_set_peripheral_address(audio_out_dma_stream, &SPI3->DR);
	dma_stream_set_memory_0_address(audio_out_dma_stream, &dac_buffer[  0]);
	dma_stream_set_memory_1_address(audio_out_dma_stream, &dac_buffer[256]);
	dma_stream_set_item_count(audio_out_dma_stream, 64 * 2 * 2);

	dma_stream_set_cr(audio_out_dma_stream,
					  0,
					  DMA_STREAM_MEMORY_BURST_SINGLE_TRANSFER,
					  DMA_STREAM_PERIPHERAL_BURST_SINGLE_TRANSFER,
					  DMA_STREAM_CURRENT_TARGET_MEMORY_0,
					  DMA_STREAM_DOUBLE_BUFFER_ON,
					  DMA_STREAM_PRIORITY_HIGH,
					  DMA_STREAM_PINCOS_LINKED_TO_PSIZE,
					  DMA_STREAM_MEMORY_DATA_SIZE_HALF_WORD,
					  DMA_STREAM_PERIPHERAL_DATA_SIZE_HALF_WORD,
					  DMA_STREAM_MEMORY_ADDRESS_INCREMENT,
					  DMA_STREAM_PERIPHERAL_ADDRESS_FIXED,
					  DMA_STREAM_CIRCULAR_MODE_ENABLED,
					  DMA_STREAM_DIRECTION_MEMORY_TO_PERIPHERAL,
					  DMA_STREAM_PERIPHERAL_FLOW_CONTROLLER_DMA,
					  0, 0, 0, 0,
					  DMA_STREAM_ENABLED
					  );

	dma_stream_set_fcr(audio_out_dma_stream,
					   0,
					   DMA_STREAM_DIRECT_MODE_ENABLED,
					   DMA_STREAM_FIFO_THRESHOLD_1_2_FULL
					   );

	dma_stream_enable(audio_out_dma_stream);
	spi_enable_tx_dma(i2s_output);

	// Wiggler pin
	gpio_set_mode(core_activity_pin, GPIO_MODER_OUTPUT);
	gpio_set_output_type(core_activity_pin, GPIO_OTYPER_PUSH_PULL);
	gpio_set_output_speed(core_activity_pin, GPIO_OSPEEDR_2MHZ);

	// Section 25.4.5: Start slave first, then master.
	// The data register needs to be loaded before the
	// master begins signaling.
	i2s_enable(i2s_output);
	i2s_enable(i2s_input);

	uint_fast8_t buffer = 0;
	while(1) {
		// Wait for desired buffer to become free.
		gpio_set_output(core_activity_pin, 1);
		while( dma_stream_get_current_target(audio_out_dma_stream) == buffer );
		while( dma_stream_get_current_target(audio_in_dma_stream) == buffer );
		gpio_set_output(core_activity_pin, 0);

		int16_t* audio_in = &adc_buffer[buffer * 256];
		int16_t* audio_out = &dac_buffer[buffer * 256];
		/*
		uint_fast16_t i;
		for(i=0; i<64; ++i) {
			const int32_t x = generate_sample();
			*(audio_out++) = x >> 16;
			*(audio_out++) = x & 65535;
			*(audio_out++) = x >> 16;
			*(audio_out++) = x & 65535;
		}
		*/
		memcpy(audio_out, audio_in, sizeof(dac_buffer[0]) * 256);

		buffer = (buffer == 0) ? 1 : 0;
	}

	return 0;
}
