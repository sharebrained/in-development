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

#include "sdio.h"
#include "bitband.h"

void sdio_set_clkcr(sdio_t* const sdio,
				    const sdio_hardware_flow_control_t hardware_flow_control,
					const sdio_clock_phase_t clock_phase,
					const sdio_bus_width_t bus_width,
					const sdio_clock_divider_bypass_t clock_divider_bypass,
					const sdio_power_saving_t power_saving,
					const sdio_clock_enable_t clock_enable,
					const uint_fast8_t clock_divide_factor) {
	sdio->CLKCR = (hardware_flow_control << SDIO_CLKCR_HWFC_EN_BIT) |
			      (clock_phase << SDIO_CLKCR_NEGEDGE_BIT) |
			      (bus_width << SDIO_CLKCR_WIDBUS_BASE) |
			      (clock_divider_bypass << SDIO_CLKCR_BYPASS_BIT) |
			      (power_saving << SDIO_CLKCR_PWRSAV_BIT) |
			      (clock_enable << SDIO_CLKCR_CLKEN_BIT) |
			      (clock_divide_factor << SDIO_CLKCR_CLKDIV_BASE);
}

void sdio_set_power_control(sdio_t* const sdio,
						    const sdio_power_control_t power_control) {
	sdio->POWER = (sdio->POWER & ~(3 << SDIO_POWER_PWRCTRL_BASE)) | (power_control << SDIO_POWER_PWRCTRL_BASE);
}

void sdio_enable_clock(sdio_t* const sdio) {
	bitband_set(BITBAND_PERIPHERAL, &sdio->CLKCR, SDIO_CLKCR_CLKEN_BIT, 1);
}
/*
bool sdio_is_command_sent(sdio_t* const sdio) {
	return bitband_get(BITBAND_PERIPHERAL, &sdio->STA, SDIO_STA_CMDSENT_BIT);
}

void sdio_clear_command_sent(sdio_t* const sdio) {
	bitband_set(BITBAND_PERIPHERAL, &sdio->ICR, SDIO_ICR_CMDSENTC_BIT, 1);
}

bool sdio_is_command_response_timeout(sdio_t* const sdio) {
	return bitband_get(BITBAND_PERIPHERAL, &sdio->STA, SDIO_STA_CTIMEOUT_BIT);
}
*/
uint32_t sdio_status(sdio_t* const sdio, const sdio_sta_t flags) {
	return sdio->STA & flags;
}

void sdio_clear_static_flags(sdio_t* const sdio) {
	sdio->ICR = (1 << SDIO_ICR_CEATAENDC_BIT) |
				(1 << SDIO_ICR_SDIOITC_BIT) |
				(1 << SDIO_ICR_DBCKENDC_BIT) |
				(1 << SDIO_ICR_STBITERRC_BIT) |
				(1 << SDIO_ICR_DATAENDC_BIT) |
				(1 << SDIO_ICR_CMDSENTC_BIT) |
				(1 << SDIO_ICR_CMDRENDC_BIT) |
				(1 << SDIO_ICR_RXOVERRC_BIT) |
				(1 << SDIO_ICR_TXUNDERRC_BIT) |
				(1 << SDIO_ICR_DTIMEOUTC_BIT) |
				(1 << SDIO_ICR_CTIMEOUTC_BIT) |
				(1 << SDIO_ICR_DCRCFAILC_BIT) |
				(1 << SDIO_ICR_CCRCFAILC_BIT);
}
