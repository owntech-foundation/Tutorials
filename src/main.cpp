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
 * @brief   This file it the main entry point of the
 *          OwnTech Power API. Please check the README.md
 *          file at the root of this project for basic
 *          information on how to use the Power API,
 *          or refer the the wiki for detailed information.
 *          Wiki: https://gitlab.laas.fr/owntech/power-api/core/-/wikis/home
 *
 * @author  Clément Foucher <clement.foucher@laas.fr>
 */

//-------------OWNTECH DRIVERS-------------------
#include "HardwareConfiguration.h"
#include "DataAcquisition.h"
#include "Scheduling.h"
// #include "CanCommunication.h"
#include "opalib_pid_current_mode.h"

//------------ZEPHYR DRIVERS----------------------
#include "zephyr.h"
#include "timing/timing.h"
#include "console/console.h"
#include "drivers/gpio.h"

#define APPLICATION_THREAD_PRIORITY 3
#define COMMUNICATION_THREAD_PRIORITY 5

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_hardware(); //setups the hardware peripherals of the system
void setup_software(); //setups the scheduling of the software and the control method

//-------------LOOP FUNCTIONS DECLARATION----------------------
void loop_communication_task(); //code to be executed in the slow communication task
void loop_application_task();   //code to be executed in the fast application task
void loop_control_task();       //code to be executed in real-time at 20kHz


enum serial_interface_menu_mode //LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE = 1
};

uint8_t received_serial_char;
uint8_t mode = IDLEMODE;

//--------------USER VARIABLES DECLARATIONS----------------------
timing_t start_time, end_time;
static double kp = 0.1;
static double ki = 200;

static double voltage_reference = 25;

static double V1_low_value; 
static double V2_low_value; 

static double meas_data; //temp storage meas value (ctrl task)
static double iRef = 3.4;
static double pcc_max;
static double pcc_min;
//---------------------------------------------------------------

static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false; //[bool] state of the PWM (ctrl task)

//---------------SETUP FUNCTIONS----------------------------------

void setup_hardware()
{
    hwConfig.setBoardVersion(TWIST_v_1_1_2);  
    hwConfig.configureAdcDefaultAllMeasurements();
    console_init();
    Init_CurrentMode_peripheral();
    //Init_CurrentMode_PID(kp, ki, control_task_period);
    timing_init();
    console_init();
}

void setup_software()
{
    timing_start();
    dataAcquisition.start();
    scheduling.startControlTask(loop_control_task, control_task_period);
    scheduling.startCommunicationTask(loop_communication_task,COMMUNICATION_THREAD_PRIORITY);
    scheduling.startApplicationTask(loop_application_task,APPLICATION_THREAD_PRIORITY);
}

//---------------LOOP FUNCTIONS----------------------------------

void loop_communication_task()
{
     while(1) {
        received_serial_char = console_getchar();
        switch (received_serial_char) {
            case 'h':
                //----------SERIAL INTERFACE MENU-----------------------
	        printk(" ________________________________________\n");
                printk("|     ------- MENU ---------             |\n");
                printk("|     press i : idle mode                |\n");
                printk("|     press s : serial mode              |\n");
                printk("|     press p : power mode               |\n");
                printk("|     press u : duty cycle UP            |\n");
                printk("|     press d : duty cycle DOWN          |\n");
                printk("|________________________________________|\n\n");
                //------------------------------------------------------
                break;
            case 'i':
                printk("idle mode\n");
                mode = IDLEMODE;
                break;
            case 'p':
                printk("power mode\n");
                mode = POWERMODE;
                break;
            case 'u':
                iRef = iRef + .1;
                printk("up %f\n", iRef);
                printk("pcc_max = %f\n", pcc_max);
                printk("pcc_min = %f\n", pcc_min);
                break;
            case 'd' : 
                iRef = iRef - .1;
                printk("down %f\n", iRef);
            default:
                break;

        }
    }

}



void loop_application_task()
{
    uint64_t total_cycles;
    volatile uint64_t total_ns;
    uint8_t k;
    while(1){
        hwConfig.setLedToggle();
        total_cycles = timing_cycles_get(&start_time, &end_time);
        total_ns = timing_cycles_to_ns(total_cycles);
        //printk("time = %lld, %lld\n", total_cycles, total_ns);
        k_msleep(100);
        }        
}

void loop_control_task()
{
    float32_t iMin;
    start_time = timing_counter_get();
    meas_data = dataAcquisition.getV1Low();
    if (meas_data != -10000)
        V1_low_value = meas_data;

    meas_data = dataAcquisition.getV2Low();
    if (meas_data != -10000)
        V2_low_value = meas_data;

    if (mode == IDLEMODE)
    {
        pwm_enable = false;
        Disable_CurrentMode();
        // Disable_CurrentMode_leg2();
    }
    else if (mode == POWERMODE)
    {

        if (!pwm_enable)
        {
            pwm_enable = true;
            Enable_CurrentMode();
            // Enable_CurrentMode_leg2();
        }
        iMin = iRef - 2.4;
        if (iMin < 0.0) iMin =  0.0;

        pcc_max = ((iRef * 0.1) + 1.053);
        pcc_min = ((iMin * 0.1) + 1.053);
        set_satwtooth(pcc_max, pcc_min);
        //Update_DutyCycle_CM(voltage_reference, V1_low_value);
        // Update_DutyCycle_CM_leg2(voltage_reference, V2_low_value);
    }

    end_time = timing_counter_get();
}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */

void main(void)
{
    setup_hardware();
    setup_software();
}