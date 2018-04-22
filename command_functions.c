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
#include "UART_packet_interface.h"


uint8_t halt() {
    // I don't know what you will do yet.
    //Probably set outputs to logic low
    //Also likely needs to be called from the UART interrupt conditionally.
    //Otherwise it'll be gated behind previousAction_completed
    //and thus useless
    return 0;
}

uint8_t burpLastPacket() {
    // This takes advantage of the fact that the most recent packet will
    // always be the only thing between start and end in the ring buffer.
    // One thing to note is how the buffer is interrupt-driven so this could technically be interrupted and shift the end pointer
    uint8_t iter;
    for (iter = UART_RingBuffer.start; iter < UART_RingBuffer.end; iter++){
        MAP_UART_transmitData(EUSCI_A0_BASE, (char)UART_RingBuffer.data[UART_RingBuffer.start]);
    }
    return 0;
}

//Takes a pointer to a packet and transmits over UART.
//Largely for the sake of being complete and ease of debugging.
uint8_t echoPacket(commandPacket *packet) {
    MAP_UART_transmitData(EUSCI_A0_BASE, (char)packet->type);
    MAP_UART_transmitData(EUSCI_A0_BASE, (char)packet->command);
    MAP_UART_transmitData(EUSCI_A0_BASE, (char)packet->length);
    uint8_t iter;
    for (iter = 0; iter < packet->length; iter++){
        MAP_UART_transmitData(EUSCI_A0_BASE, (char)packet->data[iter]);
    }
    return 0;
}

uint8_t moveVSlot_HAB() {
    //check table location and move accordingly
    switch(tablePosition) {
    case TABLE_POSITION_SPC:
        //might be worth creating a debugging version of this that doesn't argue about both doors being open
#if DEBUG_LEVEL > 0
        moveVSlot_CEN();
        tablePosition = TABLE_POSITION_CEN;
#else
        return 1;   //this is only reached in the no-debugging compile type.
#endif
    case TABLE_POSITION_CEN:
        //Set direction for required stepper to move HAB-side from center
        set_stepperDirection(VSLOT_STEPPER_TOP_DIR, (bool)STEPPER_DIR_HAB);
        //enable stepper required to move from center to HAB
        //either stepper can work but I am defining TOP as the "move-hab-side" stepper
        enable_stepper(VSLOT_STEPPER_TOP_EN);
        //Hall-effect goes low when magnet is sensed

        while (get_DIOPinState(HALL_SENSE_VSLOT_TOP) == 0) {
            //nada
        }
        disable_stepper(VSLOT_STEPPER_TOP_EN);
        tablePosition = TABLE_POSITION_HAB;
        return 0;
    case TABLE_POSITION_HAB:
        return 0;
    default:
        return 1;
    }
}


// Note: it might be worth it to put the hall-effects on an interrupt-driven thing rather than polling.
// The polling is not  halting anything else though.
// All important tasks are interrupt-driven
uint8_t moveVSlot_SPC() {
    switch(tablePosition){
    case TABLE_POSITION_HAB:
#if DEBUG_LEVEL > 0
        moveVSlot_CEN();
        tablePosition = TABLE_POSITION_CEN;
#else
        return 1;
#endif
    case TABLE_POSITION_CEN:
        //Set direction of bottom stepper to move SPC-side from center
        set_stepperDirection(VSLOT_STEPPER_BOT_DIR, (bool)STEPPER_DIR_SPC);
        //Enable stepper motor now that everything is set
        enable_stepper(VSLOT_STEPPER_BOT_EN);
        //wait for the hall-effect detection
        while (get_DIOPinState(HALL_SENSE_VSLOT_BOT) == 0) {
            //nada
        }
        //job done, disable stepper.
        disable_stepper(VSLOT_STEPPER_BOT_EN);
        //update position
        tablePosition = TABLE_POSITION_HAB;
        return 0;
    case TABLE_POSITION_SPC:
        return 0;
    default:
        return 1;
    }
}

//if it only works correctly in one direction, split direction defines into top and bottom.
uint8_t moveVSlot_CEN() {
    switch(tablePosition){
    case TABLE_POSITION_HAB:
        //Set direction of bottom stepper to move SPC-side from center
        set_stepperDirection(VSLOT_STEPPER_TOP_DIR, (bool)STEPPER_DIR_SPC);
        //enable stepper movement
        enable_stepper(VSLOT_STEPPER_TOP_EN);
        //poll for the hall-effect sensor
        while (get_DIOPinState(HALL_SENSE_VSLOT_TOP) == 0) {
            //nada
        }
        //disable movement
        disable_stepper(VSLOT_STEPPER_TOP_EN);
        //update position
        tablePosition = TABLE_POSITION_CEN;
        return 0;
    case TABLE_POSITION_CEN:
        //nothing to do
        return 0;
    case TABLE_POSITION_SPC:
        //Set direction of bottom stepper to move SPC-side from center
        set_stepperDirection(VSLOT_STEPPER_BOT_DIR, (bool)STEPPER_DIR_HAB);
        //enable stepper movement
        enable_stepper(VSLOT_STEPPER_BOT_EN);
        //poll for the hall-effect sensor
        while (get_DIOPinState(HALL_SENSE_VSLOT_BOT) == 0) {
            //nada
        }
        //disable movement
        disable_stepper(VSLOT_STEPPER_BOT_EN);
        //update position
        tablePosition = TABLE_POSITION_CEN;
        return 0;
    default:
        return 1;
    }
}

uint8_t clampExperiment() {
    //if (0)
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

uint8_t get_tableCapSense(){
    uint8_t cap_One;
    uint8_t cap_Two;
    cap_One = get_DIOPinState(CAP_SENSE_TABLE_ONE);
    cap_Two = get_DIOPinState(CAP_SENSE_TABLE_TWO);
    if ((cap_One > 0) && (cap_Two > 0))
        return TABLE_CAP_SENSE_BOTH;
    else if (cap_One > 0)
        return TABLE_CAP_SENSE_ONE;
    else if (cap_Two > 0)
        return TABLE_CAP_SENSE_TWO;
    else
        return 0;
}

bool get_plateCapSense(){
    return get_DIOPinState(CAP_SENSE_MATE) != 0;
}

uint16_t get_tableForceSense(){
    //return get_ADCValue(FORCE_SENSE_PIN);
    return 0;
}

bool get_VSlot_top_HEState(){
    return get_DIOPinState(HALL_SENSE_VSLOT_TOP) != 0;
}

bool get_VSlot_bot_HEState(){
    return get_DIOPinState(HALL_SENSE_VSLOT_BOT) != 0;
}

bool get_HABDoor_HEState(){
    return get_DIOPinState(HALL_SENSE_DOOR_HAB) != 0;
}

bool get_SPCDoor_HEState(){
    return get_DIOPinState(HALL_SENSE_DOOR_SPC) != 0;
}

bool get_HABHinge_HEState(){
    return get_DIOPinState(HALL_SENSE_HINGE_HAB) != 0;
}

bool get_SPCHinge_HEState(){
    return get_DIOPinState(HALL_SENSE_HINGE_SPC) != 0;
}

int get_Temperature(){
    return 0;
}
