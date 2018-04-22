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

uint8_t enable_stepper(int port, int pin){
    //Enable is inverted
    MAP_GPIO_setOutputLowOnPin(port, pin);
    return 0;
}

uint8_t disable_stepper(int port, int pin){
    //Enable is inverted
    MAP_GPIO_setOutputHighOnPin(port, pin);
    return 0;
}

uint8_t set_stepperDirection(int port, int pin, bool direction){
    if(direction){
        MAP_GPIO_setOutputLowOnPin(port, pin);
    } else {
        MAP_GPIO_setOutputHighOnPin(port, pin);
    }
    return 0;
}

uint8_t set_stepperSpeed(int port, int pin, int speed){
    //this will have to modify the timer counter values...
    //Probably have an input of the timer struct as well...
    return 0;
}

uint8_t get_DIOPinState(int port, int pin){
    return MAP_GPIO_getInputPinValue(port, pin);
}

uint16_t get_ADCValue(int stepper_port, int pin){
    return 0;
}


