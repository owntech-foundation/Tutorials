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
 *          Noemi Lanciotti <noemi.lanciotti@centralesupelec.fr>
 */

#include "stdio.h"
//-------------OWNTECH DRIVERS-------------------
#include "HardwareConfiguration.h"
#include "DataAcquisition.h"
#include "Scheduling.h"
#include "power.h"
#include "opalib_control_pid.h"

//------------ZEPHYR DRIVERS----------------------
#include "zephyr/zephyr.h"
#include "zephyr/console/console.h"
#include "zephyr/drivers/uart.h"

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_hardware(); //setups the hardware peripherals of the system
void setup_software(); //setups the scheduling of the software and the control method

//-------------LOOP FUNCTIONS DECLARATION----------------------
void loop_communication_task(); //code to be executed in the slow communication task
void loop_application_task();   //code to be executed in the fast application task
void loop_control_task();       //code to be executed in real-time at 20kHz

int8_t communication_task_number; //holds the number of the communication task
int8_t application_task_number; //holds the number of the application task

/* USART initialization parameters */
static const struct device *uart2_dev = DEVICE_DT_GET(DT_NODELABEL(usart2));
static uint32_t baud = 115200;
struct uart_config uart2_cfg;
struct uart_event evt2;

static uint32_t control_task_period = 50; //[us] period of the control task

uint16_t data = 0;
bool data_available = false;

const uint8_t uart2_buffer_size = 128;
uint8_t uart2_buffer[uart2_buffer_size];


enum serial_interface_menu_mode //LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE =0,
    SERIALMODE,
    POWERMODE,
    BUCKMODE,
    WIRELESSMODE,
};

uint8_t received_serial_char;
uint8_t mode = WIRELESSMODE;

//--------------USER VARIABLES DECLARATIONS----------------------


static float32_t duty_cycle = 0.5; //[-] duty cycle (comm task)
static float32_t duty_cycle_i_max = 0.0; //[-] duty cycle (comm task)
static float32_t duty_cycle_i_min = 0.0; //[-] duty cycle (comm task)
static float32_t duty_cycle_step = 0.05; //[-] duty cycle step (comm task)
static bool pwm_enable = false; //[bool] state of the PWM (ctrl task)

//Measurement data
static float32_t V1_low_value; //store value of V1_low (app task)
static float32_t V2_low_value; //store value of V2_low (app task)
static float32_t Vhigh_value; //store value of Vhigh (app task)

static float32_t i1_low_value; //store value of i1_low (app task)
static float32_t i2_low_value; //store value of i2_low (app task)
static float32_t ihigh_value; //store value of ihigh (app task)

static float32_t meas_data; //temp storage meas value (ctrl task)
//
static float32_t voltage_reference = 10; //voltage reference (app task)
static float32_t new_voltage_reference = 0; //new voltage reference (app task)
static float32_t voltage_reference_step = 2; //voltage reference step (app task)
static float32_t current_reference = 2; //current reference (app task)
static float32_t current_reference_step = 0.2; //current reference step (app task)

static float32_t maximum_voltage = 18.0; //store value of Vhigh (app task)
static float32_t maximum_current = 10.0; //store value of Vhigh (app task)


static int cpt_step = 0; //counter for voltage reference (app task)

static float32_t kp = 0.000215;
static float32_t ki = 2.86;
static float32_t kd = 0.0;

static   bool zone_high = false; //two booleans to determine in which zone we are of the remote controller


//---------------SETUP FUNCTIONS----------------------------------
void setup_hardware()
{
    hwConfig.setBoardVersion(TWIST_v_1_1_2);
    power.setShieldVersion(shield_TWIST_V1_3);
    power.initAllBuck(VOLTAGE_MODE);
    power.setAllTriggerValue(0.06);
    dataAcquisition.enableTwistDefaultChannels();
    
    //code to initialize the uart2
    uart_config_get(uart2_dev, &uart2_cfg);
    uart2_cfg.baudrate = baud;
    uart2_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    uart2_cfg.data_bits = UART_CFG_DATA_BITS_8;
    uart2_cfg.parity = UART_CFG_PARITY_NONE;
    uart2_cfg.stop_bits = UART_CFG_STOP_BITS_1;
    uart_configure(uart2_dev, &uart2_cfg);

    uart_rx_enable(uart2_dev, uart2_buffer, uart2_buffer_size, SYS_FOREVER_MS);


    console_init();
    //setup your hardware here
}

void setup_software()
{
    opalib_control_init_interleaved_pid(kp, ki, kd, control_task_period);
    // dataAcquisition.setParameters(V1_LOW,0.045022,-91.679);
    // dataAcquisition.setParameters(V2_LOW,0.044907,-91.430);
    // dataAcquisition.setParameters(V_HIGH,0.066457,-0.279);
    // dataAcquisition.setParameters(I1_LOW,0.004563,-9.367);
    // dataAcquisition.setParameters(I2_LOW,0.005507,-11.351);
    // dataAcquisition.setParameters(I_HIGH,0.005171,-10.597);

    application_task_number = scheduling.defineAsynchronousTask(loop_application_task);
    communication_task_number = scheduling.defineAsynchronousTask(loop_communication_task);
    scheduling.defineUninterruptibleSynchronousTask(&loop_control_task,control_task_period);

    scheduling.startAsynchronousTask(application_task_number);
    scheduling.startAsynchronousTask(communication_task_number);
    scheduling.startUninterruptibleSynchronousTask();
}

//---------------LOOP FUNCTIONS----------------------------------

void loop_application_task()
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
                printk("|     press b : closed-loop buck mode    |\n");
                printk("|     press m : mppt mode on             |\n");
                printk("|________________________________________|\n\n");
                //------------------------------------------------------
                break;
            case 'i':
                mode = IDLEMODE;
                break;
            case 'w':
                mode = WIRELESSMODE;
                break;
            case 'p':
                mode = POWERMODE;
                break;
            case 'u':
                duty_cycle = duty_cycle + duty_cycle_step;
                break;
            case 'd':
                duty_cycle = duty_cycle - duty_cycle_step;
                break;
            case 'a':
                voltage_reference = voltage_reference + voltage_reference_step;
                break;
            case 'z':
                voltage_reference = voltage_reference - voltage_reference_step;
                break;
            default:
                break;

        }
}


void loop_communication_task()
{
    static uint16_t partial_data;
    static bool msb_received = false;

    // For data (on 10 bits) 000000XX_XXXYYYYY, received data will be:
    // 1st byte: 010XXXXX
    // 2nd byte: 100YYYYY
    const uint8_t MASK_HEADER = 0xE0;
    const uint8_t HEADER_MSB = 0x40;
    const uint8_t HEADER_LSB = 0x80;

    static uint16_t dead_zone_high = 520;
    static uint16_t maximum_voltage_reference = 18;
    static uint16_t dead_zone_low = 480 ;
    static uint16_t minimum_voltage_reference = 18;   //leave this value positive. The minus sign comes from an operation in the code

 

    uint8_t received_char;
    int ret_val = uart_poll_in(uart2_dev, &received_char);
    //uart_rx_enable(uart_dev, &received_char, 1, SYS_FOREVER_MS);
 
    if(mode==WIRELESSMODE){
        if(ret_val == -1)                   // verifies that the connection is still up
        {
            //voltage_reference = 0;

        } else if(ret_val != -1){           //runs if the connection is up

            // First received value will be MSB: overwrite partial data
            if ( (received_char & MASK_HEADER) == HEADER_MSB)
            {
                partial_data = (received_char & ~MASK_HEADER) << 5;
                msb_received = true;
            }
            // Second received value will be LSM: complete data and make it available
            else if ( (received_char & MASK_HEADER) == HEADER_LSB)
            {
                if (msb_received == true)
                {
                    partial_data |= (received_char & ~MASK_HEADER);
                    data = partial_data;
                    data_available = true;
                    msb_received = false;
                }
            }

            if (data>dead_zone_high){
                
                zone_high = true;
                new_voltage_reference = (float32_t)(data-dead_zone_high)/dead_zone_high;
                new_voltage_reference = (float32_t)(data-dead_zone_high)/dead_zone_high;
                new_voltage_reference = new_voltage_reference*new_voltage_reference*maximum_voltage;

                if (new_voltage_reference > 18.0) {
                    //does not apply the new value 
                }else{
                    voltage_reference = new_voltage_reference;
                }


            } else if (data<dead_zone_low){
                
                zone_high = false;
                new_voltage_reference = (float32_t)(data-dead_zone_low)/dead_zone_low;
                new_voltage_reference = new_voltage_reference*new_voltage_reference*-maximum_voltage;

                if (new_voltage_reference < -18.0) { 
                    //does not apply the new value 
                }else{
                    voltage_reference = new_voltage_reference;
                }

            } else {
                zone_high=false;
                voltage_reference = 0;
            }

        }
        printk("%f:", duty_cycle);
        printk("%f:", duty_cycle_i_max);
        printk("%f:", duty_cycle_i_min);
        printk("%f:", Vhigh_value);
        printk("%f:", V1_low_value-V2_low_value);
        printk("%f:", i1_low_value);
        printk("%f\n", voltage_reference);

    }else{
        printk("%f:", duty_cycle);
        printk("%f:", duty_cycle_i_max);
        printk("%f:", duty_cycle_i_min);
        printk("%f:", Vhigh_value);
        printk("%f:", V1_low_value-V2_low_value);
        printk("%f:", i1_low_value);
        printk("%f\n", voltage_reference);
    }
    scheduling.suspendCurrentTaskMs(100); // k_msleep(50);
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
         power.stopAll();

    }else if(mode==POWERMODE || mode==BUCKMODE) {

        if(!pwm_enable) {
            pwm_enable = true;
            power.startAll();
        }
    if(mode==BUCKMODE){
        // duty_cycle = opalib_control_interleaved_pid_calculation(voltage_reference, V1_low_value);
    }
    //Sends the PWM to the switches
    power.setAllDutyCycle(duty_cycle);
    }
}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */

int main(void)
{
    setup_hardware();
    setup_software();
    return 0;
}