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
 * @author Clément Foucher <clement.foucher@laas.fr>
 */

#ifndef OPALIB_PID_CURRENT_MODE_H_
#define OPALIB_PID_CURRENT_MODE_H_

#ifdef __cplusplus
extern "C" {
#endif

void Init_CurrentMode_peripheral();
void Init_CurrentMode_PID(double p_i, double i_i, uint32_t pid_period_us );
void Enable_CurrentMode();
void Enable_CurrentMode_leg2();
void Disable_CurrentMode();
void Disable_CurrentMode_leg2();
void set_satwtooth(double set_voltage, double reset_voltage);
void set_satwtooth_leg2(double set_voltage, double reset_voltage);
void Update_DutyCycle_CM(double reference, double measurement);
void Update_DutyCycle_CM_leg2(double reference, double measurement);
double p2z2_control(double yref, double y);
void set_DAC2_value(uint32_t dac_value);

#ifdef __cplusplus
}
#endif

#endif // OPALIB_PID_CURRENT_MODE_H_
