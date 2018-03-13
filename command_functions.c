/*
 * command_functions.c
 *
 *  Created on: Mar 12, 2018
 *      Author: Aetas
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Project Includes */
#include "defines.h"
#include "command_functions.h"
#include "peripheral_interfaces.h"
#include "printf.h"

uint8_t halt() {
    return 0;
}

uint8_t burpLastPacket() {

    return 0;
}

uint8_t moveVSlot_HAB() {
    //check table location and move accordingly

    //enable_stepper((uint8_t)VSLOT_STEPPER_TOP);
    //set_stepperDirection((uint8_t)VSLOT_STEPPER_TOP, (bool)STEPPER_DIR_HAB);
    while(true){
        return 0;
    }
}

uint8_t moveVSlot_SPC() {

    return 0;
}

uint8_t moveVSlot_CEN() {

    return 0;
}

uint8_t clampExperiment() {

    return 0;
}

uint8_t releaseExperiment() {

    return 0;
}

uint8_t open_HABDoor(){

    return 0;
}

uint8_t open_SPCDoor(){

    return 0;
}

uint8_t close_HABDoor(){

    return 0;
}

uint8_t close_SPCDoor(){

    return 0;
}

bool get_tableCapSense(){

    return 0;
}

bool get_plateCapSense(){
    return 0;
}

int get_tableForceSense(){
    //return get_ADCValue(FORCE_SENSE_PIN);
    return 0;
}



bool get_VSlot_top_HEState(){

    return 0;
}

bool get_VSlot_bot_HEState(){

    return 0;
}

bool get_HABDoor_HEState(){

    return 0;
}

bool get_SPCDoor_HEState(){

    return 0;
}

bool get_HABHinge_HEState(){

    return 0;
}

bool get_SPCHinge_HEState(){

    return 0;
}

int get_Temperature(){

    return 0;
}
