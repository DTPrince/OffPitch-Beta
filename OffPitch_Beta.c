/*
 * Off-Pitch Beta
 *
 *  Created on: Mar 5, 2018
 *      Author: Derek Prince
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Project Includes */
#include "defines.h"
#include "UART_packet_interface.h"
#include "peripheral_interfaces.h"
#include "command_functions.h"
#include "Board.h"
#if DEBUG_LEVEL > 0
#include "printf.h"
#endif


/* Initialize function
 * Sets the pinmodes, timer functions (PWM included), and ADC
 */
void initializeSettings();
void HALT();
//Stepper Sketch:
//2.4 : PWM
//2.3 : DIR
//1.7 : EN

int main(void) {
    initSettings();
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);


    commandPacket packet;
    commandPacket packet_previous;
    packet_previous.type = (uint8_t)UART_TYPE_COMMAND;
    packet_previous.command = (uint8_t)UART_COMMAND_NOP;
    packet_previous.typeCommand = (uint8_t)UART_COMMAND_NOP;
    packet_previous.length = 0;
    // Not setting data to 0 here to save instructions but also because any data that
    // comes with a packet of data length 0 is invalid by definition
    while(1){
        UART_transmitData(UART_BASE, 0x64);
        UART_transmitData(UART_BASE, 1);  //len = 1
        UART_transmitData(UART_BASE, 1);
    }
    bool lastAction_Completed = true;

    //empty -- initialize to beginning.
    UART_RingBuffer.end = 0;
    UART_RingBuffer.start = 0;

    tablePosition = TABLE_POSITION_CEN;

    /* Sleeping when not in use */
    while (1){
        //sleep
        PCM_gotoLPM0();

        //check if the two pointers are equal and that the MCU is done handling the last packet command.
        //if they are not equal then there is data in the ring buffer
        if ((UART_RingBuffer.end != UART_RingBuffer.start) && lastAction_Completed){
            parse_UART_RingBuffer(&packet);
            if (packet.command == 'a'){    //this needs a lot more logic
                moveVSlot_HAB();
            }
        }

        if ((UART_RingBuffer.end != UART_RingBuffer.start) && lastAction_Completed){
            lastAction_Completed = false;
            parse_UART_RingBuffer(&packet);
            switch (packet.command){
            //Control commands
            case UART_COMMAND_NOP:
                // no operation
                break;

            //HALT would be here but is taken care of in the ISR
            case UART_COMMAND_BURP:
                //send last packet back.
                send_packet(&packet_previous);
                break;

            //Table Commands
            case UART_COMMAND_CLAMP_EXPERIMENT:
                // if the packet requests a acknowledge...
                if (packet.type == UART_TYPE_ACK){
                    //add the command return to data
                    packet.length = 1;
                    packet.data[0] = clampExperiment();
                    //send the packet
                    send_packet(&packet);
                } else {
                    clampExperiment();
                }
                break;

            case UART_COMMAND_RELEASE_EXPERIMENT:
                // if the packet requests a acknowledge...
                if (packet.type == UART_TYPE_ACK){
                    //add the command return to data
                    packet.length = 1;
                    packet.data[0] = releaseExperiment();
                    //send the packet
                    send_packet(&packet);
                } else {
                    releaseExperiment();
                }
                break;

            case UART_COMMAND_MV_TABLE_HAB:
                // if the packet requests a acknowledge...
                if (packet.type == UART_TYPE_ACK){
                    //add the command return to data
                    packet.length = 1;
                    packet.data[0] = 1;//moveVSlot_HAB();
                    //send the packet
                    send_packet(&packet);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
                } else {
                    moveVSlot_HAB();
                    send_packet(&packet);
                }
                break;

            case UART_COMMAND_MV_TABLE_CEN:
                // if the packet requests a acknowledge...
                if (packet.type == UART_TYPE_ACK){
                    //add the command return to data
                    packet.length = 1;
                    packet.data[0] = 1;//moveVSlot_CEN();
                    //send the packet
                    send_packet(&packet);
                    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

                } else {
                    moveVSlot_CEN();
                }
                break;

            case UART_COMMAND_MV_TABLE_SPC:
                // if the packet requests a acknowledge...
                if (packet.type == UART_TYPE_ACK){
                    //add the command return to data
                    packet.length = 1;
                    packet.data[0] = moveVSlot_SPC();
                    //send the packet
                    send_packet(&packet);
                } else {
                    moveVSlot_SPC();
                }
                break;

            //Door Commands
            case UART_COMMAND_OPEN_HAB:
                // if the packet requests a acknowledge...
                if (packet.type == UART_TYPE_ACK){
                    //add the command return to data
                    packet.length = 1;
                    packet.data[0] = open_HABDoor();
                    //send the packet
                    send_packet(&packet);
                } else {
                    open_HABDoor();
                }
                break;

            case UART_COMMAND_OPEN_SPC:
                // if the packet requests a acknowledge...
                if (packet.type == UART_TYPE_ACK){
                    //add the command return to data
                    packet.length = 1;
                    packet.data[0] = open_SPCDoor();
                    //send the packet
                    send_packet(&packet);
                } else {
                    open_SPCDoor();
                }
                break;

            case UART_COMMAND_CLOSE_HAB:
                // if the packet requests a acknowledge...
                if (packet.type == UART_TYPE_ACK){
                    //add the command return to data
                    packet.length = 1;
                    packet.data[0] = close_HABDoor();
                    //send the packet
                    send_packet(&packet);
                } else {
                    close_HABDoor();
                }
                break;

            case UART_COMMAND_CLOSE_SPC:
                // if the packet requests a acknowledge...
                if (packet.type == UART_TYPE_ACK){
                    //add the command return to data
                    packet.length = 1;
                    packet.data[0] = close_SPCDoor();
                    //send the packet
                    send_packet(&packet);
                } else {
                    close_SPCDoor();
                }
                break;

            //Data commands
            case UART_COMMAND_DATA_TABLE_CAP_SENSE:
                packet.length = 1;
                packet.data[0] = get_tableCapSense();
                send_packet(&packet);
                break;

            case UART_COMMAND_DATA_PLATE_CAP_SENSE:
                packet.length = 1;
                packet.data[0] = (uint8_t)get_plateCapSense();
                send_packet(&packet);
                break;

            case UART_COMMAND_DATA_TABLE_FORCE_SENSE:
            {
                uint16_t forceValue;
                packet.length = 1;
                forceValue = get_tableForceSense();
                //ADC is configured in 8-bit mode to allow the data to be stuffed in one char
                //We do not have the resolution in the sensor to take advantage of a 14-bit value
                // and our needs don't require more than 8-bit resolution
                packet.data[0] = forceValue;
                //note how these are packed out of order to ensure high-bits are transmitted first.
                // This is to allow them to be shifted into place upon reception.

//                packet.data[1] = (uint8_t)(forceValue & 0xFF);
//                forceValue = forceValue >> 8;
//                packet.data[0] = (uint8_t)(forceValue & 0xFF);
                send_packet(&packet);
                break;
            }
            case UART_COMMAND_DATA_TABLE_POSITION:
                packet.length = 1;
                packet.data[0] = tablePosition;

            case UART_COMMAND_DATA_HAB_HINGE_HE_SENSE:
                packet.length = 1;
                packet.data[0] = (uint8_t)get_plateCapSense();
                send_packet(&packet);
                break;

            case UART_COMMAND_DATA_SPC_HINGE_HE_SENSE:
                packet.length = 1;
                packet.data[0] = (uint8_t)get_plateCapSense();
                send_packet(&packet);
                break;

            case UART_COMMAND_DATA_PRESSURE:
                packet.length = 0;
                packet.data[0] = 0; //rand();
                send_packet(&packet);
                break;


            //Since we don't actually have a pressure sensor, we send the packet back
            // and let the R.Pi generate the data
            case UART_COMMAND_DEPRESSUREIZE:
                packet.length = 0;
                packet.data[0] = 0; //rand();
                send_packet(&packet);
                break;

            case UART_COMMAND_PRESSUREIZE:
                packet.length = 0;
                packet.data[0] = 0; //rand();
                send_packet(&packet);
                break;

            // Default
            default:
                //add debugging flag here (verbose)
                break;
            }
            lastAction_Completed = true;
            packet_previous = packet;
        }
    }
}

/* Port1 ISR - This ISR will progressively step up the duty cycle of the PWM
 * on a button press
 */
//void PORT1_IRQHandler(void){
//    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
//    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);
//
//    //
//    if (status & GPIO_PIN1) {
//        //Toggle
////        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN7);
////        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
//    }
//    if (status & GPIO_PIN4) {
////        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN3);
////        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
////        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN2);
//    }
//    if (status & GPIO_PIN5) {
//        if (GPIO_getInputPinValue(CAP_SENSE_TABLE_ONE) > 0)
//            capSense.tableCapSense1 = true;
//        else
//            capSense.tableCapSense1 = false;
//    }
//    if (status & GPIO_PIN6) {
//        if (GPIO_getInputPinValue(CAP_SENSE_TABLE_TWO) > 0)
//            capSense.tableCapSense2 = true;
//        else
//            capSense.tableCapSense2 = false;
//    }
//    if (status & GPIO_PIN7) {
//        if (GPIO_getInputPinValue(CAP_SENSE_MATE) > 0)
//            capSense.plateCapSense = true;
//        else
//            capSense.plateCapSense = false;
//    }
//
//}

//void PORT2_IRQHandler(void){
//    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);
//    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, status);
//
//    if (status & GPIO_PIN1) {
//        //do things
//    }
//    if (status & GPIO_PIN2) {
//        //do things
//    }
//    if (status & GPIO_PIN3) {
//        //do things
//    }
//    if (status & GPIO_PIN4) {
//        //do things
//    }
//    if (status & GPIO_PIN5) {
//        //do things
//    }
//    if (status & GPIO_PIN6) {
//        //do things
//    }
//    if (status & GPIO_PIN7) {
//        //do things
//    }
//}

//void PORT3_IRQHandler(void){
//    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
//    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status);
//    if (status & GPIO_PIN1) {
//        //do things
//    }
//    if (status & GPIO_PIN2) {
//        GPIO_interruptEdgeSelect(HALL_SENSE_VSLOT_TOP, GPIO_LOW_TO_HIGH_TRANSITION);
//        if (GPIO_getInputPinValue(HALL_SENSE_VSLOT_TOP) == GPIO_INPUT_PIN_LOW){
//            GPIO_interruptEdgeSelect(HALL_SENSE_VSLOT_TOP, GPIO_LOW_TO_HIGH_TRANSITION);
//            heSense.vslot.top = true;
//        } else {
//            GPIO_interruptEdgeSelect(HALL_SENSE_VSLOT_TOP, GPIO_HIGH_TO_LOW_TRANSITION);
//            heSense.vslot.top = false;
//        }
//    }
//    if (status & GPIO_PIN3) {
//       if (GPIO_getInputPinValue(HALL_SENSE_VSLOT_BOT) > 0)
//           heSense.vslot.bot = false;
//       else
//           heSense.vslot.bot = true;
//    }
//    if (status & GPIO_PIN4) {
//        //do things
//    }
//    if (status & GPIO_PIN5) {
//        if (GPIO_getInputPinValue(HALL_SENSE_DOOR_HAB) > 0)
//            heSense.door.hab = false;
//        else
//            heSense.door.hab = true;
//    }
//    if (status & GPIO_PIN6) {
//        if (GPIO_getInputPinValue(HALL_SENSE_DOOR_SPC) > 0)
//            heSense.door.spc = false;
//        else
//            heSense.door.spc = true;
//    }
//    if (status & GPIO_PIN7) {
//        if (GPIO_getInputPinValue(HALL_SENSE_HINGE_HAB) > 0)
//            heSense.hinge.hab = false;
//        else
//            heSense.hinge.hab = true;
//    }
//}

//void PORT4_IRQHandler(void){
//    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
//    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, status);
//
//    if (status & GPIO_PIN0) {
//        if (GPIO_getInputPinValue(HALL_SENSE_HINGE_SPC) > 0)
//            heSense.hinge.spc = false;
//        else
//            heSense.hinge.spc = true;
//    }
//    if (status & GPIO_PIN1) {
//      //do things
//    }
//    if (status & GPIO_PIN2) {
//       //do things
//    }
//    if (status & GPIO_PIN3) {
//       //do things
//    }
//    if (status & GPIO_PIN4) {
//        //do things
//    }
//    if (status & GPIO_PIN5) {
//       //do things
//    }
//    if (status & GPIO_PIN6) {
//       //do things
//    }
//    if (status & GPIO_PIN7) {
//       //do things
//    }
//}

//void PORT5_IRQHandler(void){
//    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
//    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);
//
//    if (status & GPIO_PIN1) {
//        //do things
//    }
//    if (status & GPIO_PIN2) {
//       //do things
//    }
//    if (status & GPIO_PIN3) {
//        //do things
//    }
//    if (status & GPIO_PIN4) {
//         //do things
//    }
//    if (status & GPIO_PIN5) {
//        //do things
//    }
//    if (status & GPIO_PIN6) {
//        //do things
//    }
//    if (status & GPIO_PIN7) {
//        //do things
//    }
//}

//void PORT6_IRQHandler(void){
//    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P6);
//    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P6, status);
//
//    if (status & GPIO_PIN1) {
//        //do things
//    }
//    if (status & GPIO_PIN2) {
//        //do things
//    }
//    if (status & GPIO_PIN3) {
//        //do things
//    }
//    if (status & GPIO_PIN4) {
//        //do things
//    }
//    if (status & GPIO_PIN5) {
//        //do things
//    }
//    if (status & GPIO_PIN6) {
//        //do things
//    }
//    if (status & GPIO_PIN7) {
//        //do things
//    }
//}

/* EUSCI A0 UART ISR - Fetches char from UART and stores in UART_RingBuffer for later */
void EUSCIA0_IRQHandler(void){
    //fetch status register for interrupt data
    uint32_t status = UART_getEnabledInterruptStatus(UART_BASE);

    //clear interrupt flag
    UART_clearInterruptFlag(UART_BASE, status);

    //
    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        //UART_transmitData(UART_BASE, UART_receiveData(EUSCI_A0_BASE));
        //GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        uint8_t buffer_char = UART_receiveData(UART_BASE);
#if DEBUG_LEVEL > 0
        //UART_transmitData(UART_BASE, buffer_char);
#endif
        //if ((buffer_char != '\r') && (buffer_char != '\t') && (buffer_char != ' ') && (buffer_char != '\n')){
        if ((buffer_char != ('\r' && '\t' && ' ' && '\n'))){
            switch(buffer_char){
            case 0x93:
            case 'h':
                moveVSlot_HAB();
                UART_transmitData(UART_BASE, 0x63);
                UART_transmitData(UART_BASE, 1);  //len = 1
                UART_transmitData(UART_BASE, tablePosition);
                break;
            case 0x94:
            case 'c':
                moveVSlot_CEN();
                UART_transmitData(UART_BASE, 0x63);
                UART_transmitData(UART_BASE, 1);  //len = 1
                UART_transmitData(UART_BASE, tablePosition);
                break;
            case 0x95:
            case 's':
                moveVSlot_SPC();
                UART_transmitData(UART_BASE, 0x63);
                UART_transmitData(UART_BASE, 1);  //len = 1
                UART_transmitData(UART_BASE, tablePosition);
                break;
            case 'a':
                 open_HABDoor();
                //open_SPCDoor();
                break;
            case 'q':
                clampExperiment();
                UART_transmitData(UART_BASE, 0x61);
                UART_transmitData(UART_BASE, 1);  //len = 1
                UART_transmitData(UART_BASE, 1);
                break;
            default:
                break;
            }
        }

        //Clean out spaces and formatting for the buffer
//        if (buffer_char != ('\r' || '\t' || ' ')){
//            //HALT command check. In interrupt to address before a holding action completes
//            if (buffer_char == UART_COMMAND_HALT)// || buffer_char == 'h')
//                HALT();
//
//            if (buffer_char == '0x93')
//                moveVSlot_HAB();
////            if (buffer_char == '2')
////                moveVSlot_CEN();
////            if (buffer_char == '3')
////                moveVSlot_SPC();
//            //Loop around if at end of buffer
//            if (UART_RingBuffer.end == BUFFER_SIZE){
//                UART_RingBuffer.end = 0;
//            }
//#if DEBUG_LEVEL > 8
//            //Check if the ring buffer is full
//            if (UART_RingBuffer.end == UART_RingBuffer.start){
//// some temp value to avoid compile
//                //Send debug flag over UART
//                UART_transmitData(UART_BASE, (char)UART_DEBUG_FLAG);
//
//                //Send string message
//                // REQUIRED: terminate with \n newline. This is the 'end of message' key to look for
//                printf(UART_BASE, "Ring buffer has overrun tail at: %d\n", (unsigned char)UART_RingBuffer.end);
//            }
//#endif
//            //Append data to ring buffer
//            UART_RingBuffer.data[UART_RingBuffer.end] = buffer_char;
//            //increment for the next ring buffer pass
//            UART_RingBuffer.end++;
//        }
    }
}

void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);

    if(status & ADC_INT0)
    {
        adcResult = ADC14_getResult(ADC_MEM0);
    }

}

void HALT(){
    // Top Stepper Enable (high = disabled)
    GPIO_setOutputHighOnPin(VSLOT_STEPPER_TOP_EN);
    // Bottom Stepper Enable (high = disabled)
    GPIO_setOutputHighOnPin(VSLOT_STEPPER_BOT_EN);
    // Experiment Stepper Enable (high = disabled)
    GPIO_setOutputHighOnPin(EXP_STEPPER_EN);

    disable_PWM(PWM_VSLOT_TOP);
    disable_PWM(PWM_VSLOT_BOT);
    disable_PWM(PWM_EXP);
}


