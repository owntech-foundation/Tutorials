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


#ifndef COMMUNICATIONBUS_H_
#define COMMUNICATIONBUS_H_

#include <stdint.h>
#include <arm_math.h>


/////
// Static class definition

class CommunicationBus
{

public:
	static uint16_t  getCanNodeAddr();
	static bool      getCtrlEnable();
	static float32_t getCtrlReference();
	static uint16_t  getBroadcastPeriod();
	static uint16_t  getControlPeriod();

	static void setCanNodeAddr(uint16_t addr);
	static void setCtrlEnable(bool enable);
	static void setCtrlReference(float32_t reference);
	static void setBroadcastPeriod(uint16_t time_100_ms);
	static void setControlPeriod(uint16_t time_100_ms);

	static void initAnalogComm();
	static void triggerAnalogComm();
	static float32_t getAnalogCommValue();
	static void setAnalogCommValue(uint32_t analog_bus_value);
};


/////
// Public object to interact with the class

extern CommunicationBus commBus;


#endif // COMMUNICATIONBUS_H_
