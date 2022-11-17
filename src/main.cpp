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
#include "CommunicationBus.h" 
#include "opalib_control_pid.h"
#include "adc.h"

//------------ZEPHYR DRIVERS----------------------
#include "zephyr.h"
#include "console/console.h"
#include "drivers/gpio.h"
#include "stm32_ll_adc.h"

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
    IDLEMODE =0,
};

uint8_t received_serial_char;
uint8_t mode = IDLEMODE;

//--------------USER VARIABLES DECLARATIONS----------------------

uint32_t dac_value;


//---------------------------------------------------------------


//---------------SETUP FUNCTIONS----------------------------------

void setup_hardware()
{
    hwConfig.setBoardVersion(TWIST_v_1_1_2);
    hwConfig.configureAdcDefaultAllMeasurements();
    commBus.initAnalogComm();
    
    // hwConfig.initDacConstValue(2);

	// const char* adc4_channels[] =
	// {
	// 	"ANALOG_COMM"
	// };

	// hwConfig.configureAdcChannels(4, adc4_channels, 1);



    console_init();
}

void setup_software()
{
    dataAcquisition.start();
    scheduling.startApplicationTask(loop_application_task,APPLICATION_THREAD_PRIORITY);
}

//---------------LOOP FUNCTIONS----------------------------------

void loop_communication_task()
{
    while(1) {

    }
}


void loop_application_task()
{
    while(1){

        // float32_t converted_value=0;


        hwConfig.setLedToggle();
        if(dac_value>3000) dac_value = 1000;
        commBus.setAnalogCommValue(dac_value);        

        // uint32_t data_count;
        // uint16_t* buffer;
        // buffer = dataAcquisition.getAnalogCommRawValues(data_count);
        // uint16_t raw_value = buffer[data_count - 1];
        
        //gets the data from the analog communication
        float32_t converted_value = commBus.getAnalogCommValue();
        // float32_t converted_value = dataAcquisition.getAnalogComm();



        printk("%f:", converted_value);
        // printk("%d:", raw_value);
        printk("%d\n", dac_value);
        dac_value = dac_value+100;

        commBus.triggerAnalogComm();
        // LL_ADC_REG_StartConversion(ADC4);

        k_msleep(100);
    }
}


void loop_control_task()
{
  //loop control task goes here
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
