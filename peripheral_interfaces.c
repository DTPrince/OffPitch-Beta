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

uint8_t enable_stepper(uint8_t stepper);

uint8_t disable_stepper(uint8_t stepper);

uint8_t set_stepperDirection(uint8_t stepper, bool direction);

uint8_t set_stepperSpeed(uint8_t stepper, uint8_t speed);

// probs have to change the return and argument types after looking through documentation
bool get_DIOPinState(uint8_t pin);

int get_ADCValue(uint8_t pin);


