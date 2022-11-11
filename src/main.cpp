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
static float32_t kp = 0.1;
static float32_t ki = 200;

static float32_t voltage_reference = 25;

static float32_t V1_low_value; 
static float32_t V2_low_value; 

static float32_t meas_data; //temp storage meas value (ctrl task)
static float32_t vRef = 25.0;
static float32_t pcc_max;
static float32_t pcc_min;

typedef struct myRecord {
    float32_t v1_low;
    float32_t vHigh;
    float32_t iHigh;
    float32_t iRef;
    float32_t vRef; 
    uint64_t tCalc;
} record_t ;
record_t myRecords[0x3FF];

// 2p2z for 4ms step response
// const float32_t Az[3] = {1.000000000000000,  -0.6566,  -0.3434};
// const float32_t Bz[3] = {-0.001687,   0.004444,  0.006131};

// 2p2z for 2ms step response

const float32_t Az[3] = {1.000000000000000,  - 0.6566,  -0.3434};
const float32_t Bz[3] = {0.2763,   0.02747,  - 0.2488};



float32_t p2z2_control_v2(float32_t yref, float32_t y, bool enable)
{
    static float32_t e[3] = {0.0, 0.0, 0.0};
    static float32_t u[3] = {0.0, 0.0, 0.0};
    int kLoop;
    float32_t reset_data;

    if (enable)
    {
        e[0] = yref - y;

        u[0] = Bz[0] * e[0] + Bz[1] * e[1] + Bz[2] * e[2] - Az[1] * u[1] - Az[2] * u[2];

        // if (u[0] < 0.1) u[0] = 0.1;
        // if (u[0] > 20.0) u[0] = 20.0;
        for (kLoop = 2; kLoop > 0; --kLoop)
        {
            u[kLoop] = u[kLoop - 1];
            e[kLoop] = e[kLoop - 1];
        }
    }
    else
    {
        for (kLoop = 0; kLoop < 3; kLoop++)
        {
            e[kLoop] = 0.0;
            u[kLoop] = 0.0;
        }
    }
    return u[0];
}

//---------------------------------------------------------------

static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false; //[bool] state of the sWM (ctrl task)

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
                vRef = vRef + 1.0;
                printk("up %f\n", vRef);
                printk("pcc_max = %f\n", pcc_max);
                printk("pcc_min = %f\n", pcc_min);
                break;
            case 'd' : 
                vRef = vRef - 1.0;
                printk("down %f\n", vRef);
            default:
                break;

        }
    }

}



void loop_application_task()
{
    uint8_t k;
    while(1){
        hwConfig.setLedToggle();
        k_msleep(100);
        }        
}

void loop_control_task()
{
    float32_t iRef, iMin;
    static uint16_t kTab;
    uint64_t total_cycles;
    volatile uint64_t total_ns;
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
        iRef = p2z2_control_v2(vRef, V1_low_value, false);
        Disable_CurrentMode();
        kTab = 0;
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
        if (kTab == 100)
            vRef = 30.0;
        iRef = p2z2_control_v2(vRef, V1_low_value, true);
        if (iRef > 10.0)
            iRef = 10.0;
        iMin = iRef - 2.4;
        if (iMin < 0.0) iMin =  0.0;

        pcc_max = ((iRef * 0.1) + 1.053);
        pcc_min = ((iMin * 0.1) + 1.053);
        set_satwtooth(pcc_max, pcc_min);
        //Update_DutyCycle_CM(voltage_reference, V1_low_value);
        // Update_DutyCy   cle_CM_leg2(voltage_reference, V2_low_value);
    }
    end_time = timing_counter_get();
    total_cycles = timing_cycles_get(&start_time, &end_time);
    total_ns = timing_cycles_to_ns(total_cycles);
    myRecords[kTab].iRef = iRef;
    myRecords[kTab].v1_low = V1_low_value;
    myRecords[kTab].tCalc = total_ns;
    myRecords[kTab].vRef = vRef;
    if (kTab < 0x1FF)
        kTab++;
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