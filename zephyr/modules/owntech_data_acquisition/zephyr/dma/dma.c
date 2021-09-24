/*
 * Copyright (c) 2021 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGLPV2.1
 */

/**
 * @brief  DMA configuration for OwnTech application.
 * One DMA channel is assigned per ADC. For each ADC, the DMA
 * has a buffer sized 2*(number of enabled channels in ADC),
 * which is subdivised in two half-buffers, so that when
 * the DMA is filling one half-buffer, the other one
 * is available to data dispatch.
 * DMA 1 is used for all acquisitions, with channel i
 * acquiring values from ADC i.
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 */


// Stdlib
#include <stdint.h>

// Zephyr
#include <zephyr.h>
#include <drivers/dma.h>

// STM32
#include <stm32g4xx_ll_dma.h>

// OwnTech API
#include "../data_dispatch/data_dispatch.h"
#include "../adc/adc_channels.h"


/////
// DT definitions
#define DMA1_NODELABEL DT_NODELABEL(dma1)
#define DMA1_LABEL     DT_PROP(DMA1_NODELABEL, label)

const struct device* _dma1;

/////
// Arrays of buffers:
// half_buffer_*[i][] is an array whose size matches
// the number of enabled channels in ADC(i+1).
static uint16_t* half_buffer_1[3];
static uint16_t* half_buffer_2[3];


/////
// Private API

/**
 * DMA callback
 * This callback is called twice per buffer filling:
 * when buffer is half-filled and when buffer is filled.
 */
static void _dma_callback(const struct device* dev, void* user_data, uint32_t channel, int status)
{
	static uint8_t current_half_buffer[3] = {0};

	if (current_half_buffer[channel] == 0)
	{
		data_dispatch_do_dispatch(channel+1, half_buffer_1[channel]);
		current_half_buffer[channel] = 1;
	}
	else
	{
		data_dispatch_do_dispatch(channel+1, half_buffer_2[channel]);
		current_half_buffer[channel] = 0;
	}
}

/**
 * DMA init function for one channel
 */
static void _dma_channel_init(uint8_t adc_num, uint32_t source_address, uint32_t source_trigger)
{
	// Prepare buffers
	uint8_t enabled_channels = adc_channels_get_enabled_channels_count(adc_num);
	size_t dma_buffer_size = enabled_channels * sizeof(uint16_t) * 2;
	uint8_t adc_index = adc_num - 1;

	uint16_t* dma_buffer = k_malloc(dma_buffer_size);
	half_buffer_1[adc_index] = dma_buffer;
	half_buffer_2[adc_index] = dma_buffer + enabled_channels;

	// Configure DMA
	struct dma_block_config dma_block_config_s = {0};
	struct dma_config       dma_config_s       = {0};

	dma_block_config_s.source_address   = source_address;         // Source: ADC DR register
	dma_block_config_s.dest_address     = (uint32_t)dma_buffer;   // Dest: buffer in memory
	dma_block_config_s.block_size       = dma_buffer_size;        // Buffer size in bytes
	dma_block_config_s.source_addr_adj  = DMA_ADDR_ADJ_NO_CHANGE; // Source: no increment in ADC register
	dma_block_config_s.dest_addr_adj    = DMA_ADDR_ADJ_INCREMENT; // Dest: increment in memory
	dma_block_config_s.source_reload_en = 1;                      // Reload initial address after block; Enables Half-transfer interrupt
	dma_block_config_s.dest_reload_en   = 1;                      // Reload initial address after block

	dma_config_s.dma_slot            = source_trigger;       // Source: triggered from ADC
	dma_config_s.channel_direction   = PERIPHERAL_TO_MEMORY; // From periph to mem
	dma_config_s.source_data_size    = 2;                    // 2 bytes
	dma_config_s.dest_data_size      = 2;                    // 2 bytes
	dma_config_s.source_burst_length = 1;                    // No burst
	dma_config_s.dest_burst_length   = 1;                    // No burst
	dma_config_s.block_count         = 1;                    // 1 block
	dma_config_s.head_block          = &dma_block_config_s;  // Above block config
	dma_config_s.dma_callback        = _dma_callback;        // Callback

	dma_config(_dma1, adc_num, &dma_config_s);
}


/////
// Public API

void dma_configure_and_start()
{
	_dma1 = device_get_binding(DMA1_LABEL);

	// ADC 1
	if (adc_channels_get_enabled_channels_count(1) > 0)
	{
		_dma_channel_init(1, (uint32_t)(&(ADC1->DR)), LL_DMAMUX_REQ_ADC1);
		dma_start(_dma1, 1);
	}

	// ADC 2
	if (adc_channels_get_enabled_channels_count(2) > 0)
	{
		_dma_channel_init(2, (uint32_t)(&(ADC2->DR)), LL_DMAMUX_REQ_ADC2);
		dma_start(_dma1, 2);
	}

	// ADC 3
	if (adc_channels_get_enabled_channels_count(3) > 0)
	{
		_dma_channel_init(3, (uint32_t)(&(ADC3->DR)), LL_DMAMUX_REQ_ADC3);
		dma_start(_dma1, 3);
	}
}


uint16_t* dma_get_dma1_buffer()
{
	return half_buffer_1[0];
}

uint16_t* dma_get_dma2_buffer()
{
	return half_buffer_1[1];
}
