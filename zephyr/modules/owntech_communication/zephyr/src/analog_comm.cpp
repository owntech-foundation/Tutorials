/*
 * Copyright (c) 2022 LAAS-CNRS
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
 * @date   2022
 *
 * @author Luiz Villa <luiz.villa@laas.fr>
 */


#include "analog_comm.h"

// OwnTech Power API
#include "HardwareConfiguration.h"
#include "DataAcquisition.h"

// OwnTech peripherals used in the analog communication
uint8_t analog_comm_adc_number = 4;
uint8_t analog_comm_dac_number = 2;
uint8_t analog_comm_dac_channel = 1;


void analog_comm_init()
{
	const char* analog_comm_channels[] =
	{
		"ANALOG_COMM"
	};

	hwConfig.configureAdcChannels(analog_comm_adc_number, analog_comm_channels, 1);
    hwConfig.initDacConstValue(analog_comm_dac_number);
}

void analog_comm_trigger()
{
    adc_software_trigger_conversion(analog_comm_adc_number);
}

float32_t analog_comm_get_value()
{
	return dataAcquisition.getAnalogComm();
}

void analog_comm_set_value(uint32_t analog_comm_value)
{
    hwConfig.setDacConstValue(analog_comm_dac_number,analog_comm_dac_channel,analog_comm_value);
}
