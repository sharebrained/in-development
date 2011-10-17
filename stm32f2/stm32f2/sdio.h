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

#ifndef STM32F2_SDIO_H_
#define STM32F2_SDIO_H_

#include "hw/sdio_hw.h"

typedef uint_fast8_t bool;

typedef enum sdio_power_control_t {
	SDIO_POWER_OFF	= 0b00,
	SDIO_POWER_ON	= 0b11,
} sdio_power_control_t;

typedef enum sdio_hardware_flow_control_t {
	SDIO_HARDWARE_FLOW_CONTROL_DISABLED = SDIO_CLKCR_HWFC_EN_DISABLED,
	SDIO_HARDWARE_FLOW_CONTROL_ENABLED	= SDIO_CLKCR_HWFC_EN_ENABLED,
} sdio_hardware_flow_control_t;

typedef enum sdio_clock_phase_t {
	SDIO_CLOCK_PHASE_SDIOCLK_RISING_EDGE	= SDIO_CLKCR_NEGEDGE_RISING,
	SDIO_CLOCK_PHASE_SDIOCLK_FALLING_EDGE	= SDIO_CLKCR_NEGEDGE_FALLING,
} sdio_clock_phase_t;

typedef enum sdio_bus_width_t {
	SDIO_BUS_WIDTH_1_BIT	= SDIO_CLKCR_WIDBUS_DEFAULT,
	SDIO_BUS_WIDTH_4_BIT	= SDIO_CLKCR_WIDBUS_4_WIDE,
	SDIO_BUS_WIDTH_8_BIT	= SDIO_CLKCR_WIDBUS_8_WIDE,
} sdio_bus_width_t;

typedef enum sdio_clock_divider_bypass_t {
	SDIO_CLOCK_DIVIDER_BYPASS_DISABLED	= SDIO_CLKCR_BYPASS_DISABLE,
	SDIO_CLOCK_DIVIDER_BYPASS_ENABLED	= SDIO_CLKCR_BYPASS_ENABLE,
} sdio_clock_divider_bypass_t;

typedef enum sdio_power_saving_t {
	SDIO_POWER_SAVING_CLOCK_ALWAYS_ENABLED	= SDIO_CLKCR_PWRSAVE_CK_ALWAYS,
	SDIO_POWER_SAVING_CLOCK_WHEN_ACTIVE		= SDIO_CLKCR_PWRSAVE_CK_ACTIVE,
} sdio_power_saving_t;

typedef enum sdio_clock_enable_t {
	SDIO_CLOCK_DISABLED	= SDIO_CLKCR_CLKEN_DISABLED,
	SDIO_CLOCK_ENABLED	= SDIO_CLKCR_CLKEN_ENABLED,
} sdio_clock_enable_t;

void sdio_set_clkcr(sdio_t* const sdio,
					const sdio_hardware_flow_control_t hardware_flow_control,
					const sdio_clock_phase_t clock_phase,
					const sdio_bus_width_t bus_width,
					const sdio_clock_divider_bypass_t clock_divider_bypass,
					const sdio_power_saving_t power_saving,
					const sdio_clock_enable_t clock_enable,
					const uint_fast8_t clock_divide_factor);

void sdio_set_power_control(sdio_t* const sdio,
						    const sdio_power_control_t power_control);

void sdio_enable_clock(sdio_t* const sdio);
/*
bool sdio_is_command_sent(sdio_t* const sdio);
void sdio_clear_command_sent(sdio_t* const sdio);
bool sdio_is_command_response_timeout(sdio_t* const sdio);
*/
uint32_t sdio_status(sdio_t* const sdio, const sdio_sta_t flags);

void sdio_clear_static_flags(sdio_t* const sdio);

#endif /* STM32F2_SDIO_H_ */
