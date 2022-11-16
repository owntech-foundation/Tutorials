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
 * @author Clément Foucher <clement.foucher@laas.fr>
 */


#include "../src/data_objects.h"
#include "../src/analog_comm.h"

#include "CommunicationBus.h"

/////
// Public object to interact with the class

CommunicationBus commBus;


/////
// Extern variable defined in this module

extern uint16_t broadcast_time;
extern uint16_t control_time;


uint16_t CommunicationBus::getCanNodeAddr()
{
    return can_node_addr;
}

bool CommunicationBus::getCtrlEnable()
{
    return ctrl_enable;
}

float32_t CommunicationBus::getCtrlReference()
{
    return reference_value;
}

uint16_t CommunicationBus::getBroadcastPeriod()
{
    return broadcast_time;
}

uint16_t CommunicationBus::getControlPeriod()
{
    return control_time;
}


void CommunicationBus::setCanNodeAddr(uint16_t addr)
{
    can_node_addr = addr;
}

void CommunicationBus::setCtrlEnable(bool enable)
{
    ctrl_enable = enable;
}

void CommunicationBus::setCtrlReference(float32_t reference)
{
    reference_value = reference;
}

void CommunicationBus::setBroadcastPeriod(uint16_t time_100_ms)
{
    broadcast_time = time_100_ms;
}

void CommunicationBus::setControlPeriod(uint16_t time_100_ms)
{
    control_time = time_100_ms;
}

void CommunicationBus::initAnalogComm()
{
    analog_comm_init();
}

void CommunicationBus::triggerAnalogComm()
{
    analog_comm_trigger();
}

float32_t CommunicationBus::getAnalogCommValue()
{
    analog_comm_get_value();
}

void CommunicationBus::setAnalogCommValue(uint32_t analog_bus_value)
{
    analog_comm_set_value(analog_bus_value);
}

