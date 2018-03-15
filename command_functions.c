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

//Table position state
//0b100 : Table space side
//0b010 : Table center
//0b001 : Table hab side
uint8_t tablePosition;


uint8_t halt() {
    return 0;
}

uint8_t burpLastPacket() {

    return 0;
}

uint8_t moveVSlot_HAB() {
    //check table location and move accordingly
    switch(tablePosition) {
    case TABLE_POSITION_SPC:
        //might be worth creating a debugging version of this that doesn't argue about both doors being open
#if DEBUG_LEVEL > 0
        moveVSLot_CEN();
        tablePosition = TABLE_POSITION_CEN;
#else
        return 1;   //this is only reached in the no-debugging compile type.
#endif
    case TABLE_POSITION_CEN:
        //Set direction for required stepper to move HAB-side from center
        set_stepperDirection((uint8_t)VSLOT_STEPPER_TOP, (bool)STEPPER_DIR_HAB);
        //enable stepper required to move from center to HAB
        //either stepper can work but I am defining TOP as the "move-hab-side" stepper
        enable_stepper((uint8_t)VSLOT_STEPPER_TOP);
        //Hall-effect goes low when magnet is sensed
#if DEBUG_LEVEL > 0
        unsigned int step_counter = 0;
#endif
        while (get_DOIPinState((uint8_t)HALL_SENSE_VSLOT_TOP)) {
#if DEBUG_LEVEL > 0
            step_counter++;
#else
            //nothing
#endif

        }
        disable_stepper((uint8_t)VSLOT_STEPPER_TOP);
        tablePosition = TABLE_POSITION_HAB;
        return 0;
    case TABLE_POSITION_HAB:
        return 0;
    default:
        return 1;
    }
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
