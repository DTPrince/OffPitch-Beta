/*
 * peripheral_interfaces.c
 *
 *  Created on: Mar 10, 2018
 *      Author: Derek Prince
 */


/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Project includes */
#include "defines.h"
#include "printf.h"
#include "peripheral_interfaces.h"

uint8_t enable_stepper(int stepper_port, int stepper_pin){
    MAP_GPIO_setOutputHighOnPin(stepper_port, stepper_pin);
    return 0;
}

uint8_t disable_stepper(int stepper_port, int stepper_pin){
    MAP_GPIO_setOutputLowOnPin(stepper_port, stepper_pin);
    return 0;
}

uint8_t set_stepperDirection(int stepper_port, int stepper_pin, bool direction){
    if(direction){
        MAP_GPIO_setOutputLowOnPin(stepper_port, stepper_pin);
    } else {
        MAP_GPIO_setOutputHighOnPin(stepper_port, stepper_pin);
    }
    return 0;
}

uint8_t set_stepperSpeed(int stepper_port, int stepper, int speed){
    //this will have to modify the timer counter values...
    return 0;
}

bool get_DIOPinState(int stepper_port, int pin){
    return 0;
}

int get_ADCValue(int stepper_port, int pin){
    return 0;
}


