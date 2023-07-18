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

//------------ZEPHYR DRIVERS----------------------
#include "console/console.h"

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_hardware(); //setups the hardware peripherals of the system
void setup_software(); //setups the scheduling of the software and the control method

//-------------LOOP FUNCTIONS DECLARATION----------------------
void loop_communication_task(); //code to be executed in the slow communication task
void loop_application_task();   //code to be executed in the fast application task
void loop_control_task();       //code to be executed in real-time at 20kHz

int8_t communication_task_number; //holds the number of the communication task
int8_t application_task_number; //holds the number of the application task


enum serial_interface_menu_mode //LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE =0,
    SERIALMODE,
    POWERMODE

};

uint8_t received_serial_char;
uint8_t mode = IDLEMODE;

//--------------USER VARIABLES DECLARATIONS----------------------

static uint32_t counter = 0; //counter variable
static float32_t duty_cycle = 0.5; //[-] duty cycle (comm task)
static float32_t duty_cycle_step = 0.05; //[-] duty cycle step (comm task)
static bool pwm_enable = false; //[bool] state of the PWM (ctrl task)
static uint32_t control_task_period = 5000; //[us] period of the control task

static float32_t V1_low_value; //store value of V1_low (app task)
static float32_t V2_low_value; //store value of V2_low (app task)
static float32_t Vhigh_value; //store value of Vhigh (app task)

static float32_t i1_low_value; //store value of i1_low (app task)
static float32_t i2_low_value; //store value of i2_low (app task)
static float32_t ihigh_value; //store value of ihigh (app task)

static float32_t meas_data; //temp storage meas value (ctrl task)


//---------------------------------------------------------------


//---------------SETUP FUNCTIONS----------------------------------

void setup_hardware()
{
    hwConfig.setBoardVersion(SPIN_v_1_0);
    // hwConfig.initInterleavedBuckModeCenterAligned();
    hwConfig.InitAllLegsBuckMode();
    // hwConfig.initInterleavedBuckModeCenterAligned();
    // hwConfig.setHrtimFrequency(40000);
    hwConfig.setHrtimAdcTrigInterleaved(0.06);
    dataAcquisition.enableTwistDefaultChannels();
    hwConfig.setLeg1PhaseShiftCenterAligned(0);
    hwConfig.setLeg2PhaseShiftCenterAligned(180);
    hwConfig.setLeg3PhaseShiftCenterAligned(180);
    hwConfig.setLeg5PhaseShiftCenterAligned(180);
    console_init();
    //setup your hardware here
}

void setup_software()
{
    dataAcquisition.setParameters(V1_LOW,0.045022,-91.679);
    dataAcquisition.setParameters(V2_LOW,0.044907,-91.430);
    dataAcquisition.setParameters(V_HIGH,0.066457,-0.279);
    dataAcquisition.setParameters(I1_LOW,0.004563,-9.367);
    dataAcquisition.setParameters(I2_LOW,0.005507,-11.351);
    dataAcquisition.setParameters(I_HIGH,0.005171,-10.597);


    application_task_number = scheduling.defineAsynchronousTask(loop_application_task);
    communication_task_number = scheduling.defineAsynchronousTask(loop_communication_task);
    scheduling.defineUninterruptibleSynchronousTask(&loop_control_task,control_task_period);


    scheduling.startAsynchronousTask(application_task_number);
    scheduling.startAsynchronousTask(communication_task_number);
    scheduling.startUninterruptibleSynchronousTask();
}

//---------------LOOP FUNCTIONS----------------------------------

void loop_communication_task()
{
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
            mode = IDLEMODE;
            break;
        case 's':
            mode = SERIALMODE;
            break;
        case 'p':
            mode = POWERMODE;
            break;
        case 'u':
            duty_cycle = duty_cycle + duty_cycle_step;
            if(duty_cycle>1.0) duty_cycle = 1.0;
            break;
        case 'd':
            duty_cycle = duty_cycle - duty_cycle_step;
            if(duty_cycle<0.0) duty_cycle = 0.0;
            break;
        default:
            break;
    }
}


void loop_application_task()
{
    if(mode==IDLEMODE) {
        hwConfig.setLedOff();
    }else if(mode==SERIALMODE) {
        hwConfig.setLedOn();
    }else if(mode==POWERMODE) {
        hwConfig.setLedOn();
    }        
    printk("%f:", duty_cycle);
    printk("%f:", Vhigh_value);
    printk("%f:", V1_low_value);
    printk("%f:", V2_low_value);
    printk("%f:", ihigh_value);
    printk("%f:", i1_low_value);
    printk("%f\n", i2_low_value);
    scheduling.suspendCurrentTaskMs(100);     
}

void loop_control_task()
{
    meas_data = dataAcquisition.getLatest(V_HIGH);
    if(meas_data!=NO_VALUE) Vhigh_value = meas_data;

    meas_data = dataAcquisition.getLatest(V1_LOW);
    if(meas_data!=NO_VALUE) V1_low_value = meas_data;

    meas_data = dataAcquisition.getLatest(V2_LOW);
    if(meas_data!=NO_VALUE) V2_low_value= meas_data;

    meas_data = dataAcquisition.getLatest(I_HIGH);
    if(meas_data!=NO_VALUE) ihigh_value = meas_data;

    meas_data = dataAcquisition.getLatest(I1_LOW);
    if(meas_data!=NO_VALUE) i1_low_value = meas_data;

    meas_data = dataAcquisition.getLatest(I2_LOW);
    if(meas_data!=NO_VALUE) i2_low_value = meas_data;

    if(mode==IDLEMODE || mode==SERIALMODE) {
         pwm_enable = false;
        //  hwConfig.setInterleavedOff();
         hwConfig.setLeg1Off();
         hwConfig.setLeg2Off();
         hwConfig.setLeg3Off();
         hwConfig.setLeg4Off();
         hwConfig.setLeg5Off();

    }else if(mode==POWERMODE) {

        if(!pwm_enable) {
            pwm_enable = true;
         hwConfig.setLeg1On();
         hwConfig.setLeg2On();
         hwConfig.setLeg3On();
         hwConfig.setLeg4On();
         hwConfig.setLeg5On();
        }

        //Sends the PWM to the switches
        hwConfig.setLeg1DutyCycle(duty_cycle);
        hwConfig.setLeg2DutyCycle(duty_cycle);
        hwConfig.setLeg3DutyCycle(duty_cycle);
        hwConfig.setLeg4DutyCycle(duty_cycle);
        hwConfig.setLeg5DutyCycle(duty_cycle);
    }
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
