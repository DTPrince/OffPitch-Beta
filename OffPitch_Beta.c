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
#if DEBUG_LEVEL > 0
#include "printf.h"
#endif

/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig pwmConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        120, // 40 = ~1.5Khz. 120 = 529Hz
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        60
};

/* UART Config */
const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                      // BRDIV = 78
        2,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

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

    initializeSettings();

    commandPacket packet;
    commandPacket packet_previous;
    packet_previous.type = (uint8_t)UART_TYPE_COMMAND;
    packet_previous.command = (uint8_t)UART_COMMAND_NOP;
    packet_previous.length = 0;
    // Not setting data to 0 here to save instructions but also because any data that
    // comes with a packet of data length 0 is invalid by definition

    bool lastAction_Completed = true;

    //empty -- initialize to beginning.
    UART_RingBuffer.end = 0;
    UART_RingBuffer.start = 0;

    tablePosition = TABLE_POSITION_CEN;

    /* Sleeping when not in use */
    while (1){
        //sleep
        MAP_PCM_gotoLPM0();

        //check if the two pointers are equal and that the MCU is done handling the last packet command.
        // if they are not equal then there is data in the ring buffer
        if ((UART_RingBuffer.end != UART_RingBuffer.start) && lastAction_Completed){
            lastAction_Completed = false;
            parse_UART_RingBuffer(&packet);
            switch (packet.type){
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
                    packet.data[0] = moveVSlot_HAB();
                    //send the packet
                    send_packet(&packet);
                } else {
                    moveVSlot_HAB();
                }
                break;

            case UART_COMMAND_MV_TABLE_CEN:
                // if the packet requests a acknowledge...
                if (packet.type == UART_TYPE_ACK){
                    //add the command return to data
                    packet.length = 1;
                    packet.data[0] = moveVSlot_CEN();
                    //send the packet
                    send_packet(&packet);
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
                uint16_t forceValue;
                packet.length = 2;
                forceValue = get_tableForceSense();
                //note how these are packed out of order to ensure high-bits are transmitted first.
                // This is to allow them to be shifted into place upon reception.
                packet.data[1] = (uint8_t)(forceValue & 0xFF);
                forceValue = forceValue >> 8;
                packet.data[0] = (uint8_t)(forceValue & 0xFF);
                break;

            case UART_COMMAND_DATA_HAB_DOOR_HE_SENSE:
                packet.length = 1;
                packet.data[0] = (uint8_t)get_plateCapSense();
                send_packet(&packet);
                break;

            case UART_COMMAND_DATA_SPC_DOOR_HE_SENSE:
                packet.length = 1;
                packet.data[0] = (uint8_t)get_plateCapSense();
                send_packet(&packet);
                break;

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

            case UART_COMMAND_DATA_TEMPERATURE:
                packet.length = 1;
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

void initializeSettings(){
    /* Halting the watchdog */
    MAP_WDT_A_holdTimer();


    /* Configuring P2.3 as output */
    //DIRECTION
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);
    /* Configuring P1.7 as output */
    //ENABLE
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7);
    //Green DIR=HIGH LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    //Blue DIR=LOW LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);
    //Red EN=HIGH (disabled) LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Setting MCLK to REFO at 128Khz for LF mode
     * Setting SMCLK to 64Khz */
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);//
    MAP_CS_initClockSignal(CS_MCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_2);
    MAP_PCM_setPowerState(PCM_AM_LF_VCORE0);


    /* Configuring GPIO2.4 as peripheral output for PWM  and P6.7 for button
     * interrupt */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4);


    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Setting DCO to 12MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    /* Configuring Timer_A to have a period of approximately 500ms and
     * an initial duty cycle of 10% of that (3200 ticks)  */
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    /* Enable UART module */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);
    MAP_UART_enableModule(EUSCI_A0_BASE);

    /* Enabling interrupts and starting the watchdog timer */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);

    //enable as needed. Pretty much just for capacitive sensors to avoid polling
    MAP_Interrupt_enableInterrupt(INT_PORT1);
//    MAP_Interrupt_enableInterrupt(INT_PORT2);
//    MAP_Interrupt_enableInterrupt(INT_PORT3);
//    MAP_Interrupt_enableInterrupt(INT_PORT4);
//    MAP_Interrupt_enableInterrupt(INT_PORT5);
//    MAP_Interrupt_enableInterrupt(INT_PORT6);
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();

    //EN should be HIGH for disable.
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
    //Turn on Red LED for disable notifier
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    //DIR should be whatever because we don;t know which way is which
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);
    //Turn on Green LED for DIR=HIGH
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
    //Blue LED=LOW for DIR=HIGH.
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
}

/* Port1 ISR - This ISR will progressively step up the duty cycle of the PWM
 * on a button press
 */
void PORT1_IRQHandler(void){
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    //
    if (status & GPIO_PIN1) {
        //Toggle
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN7);
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }
    if (status & GPIO_PIN4) {
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN3);
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN2);
    }

}

void PORT2_IRQHandler(void){
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, status);

    if (status & GPIO_PIN4) {
        break;
    }
}

void PORT3_IRQHandler(void){
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status);

    if (status & GPIO_PIN4) {
        break;
    }
}

void PORT4_IRQHandler(void){
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, status);

    if (status & GPIO_PIN4) {
        break;
    }
}

void PORT5_IRQHandler(void){
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);

    if (status & GPIO_PIN4) {
        break;
    }
}

void PORT6_IRQHandler(void){
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P6);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P6, status);

    if (status & GPIO_PIN4) {
        break;
    }
}

/* EUSCI A0 UART ISR - Fetches char from UART and stores in UART_RingBuffer for later */
void EUSCIA0_IRQHandler(void){
    //fetch status register for interrupt data
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    //clear interrupt flag
    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    //
    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        char buffer_char = MAP_UART_receiveData(EUSCI_A0_BASE);
#if DEBUG_LEVEL > 0
        MAP_UART_transmitData(EUSCI_A0_BASE, buffer_char);
#endif

        //Clean out spaces and formatting for the buffer
        if (buffer_char != ('\n' || '\r' || '\t' || ' ')){
            if (buffer_char == UART_COMMAND_HALT)
                HALT();
            //Loop around if at end of buffer
            if (UART_RingBuffer.end == BUFFER_SIZE){
                UART_RingBuffer.end = 0;
            }
            //Check if the ring buffer is full
            if (UART_RingBuffer.end == UART_RingBuffer.start){
#if DEBUG_LEVEL > 0
                //Send debug flag over UART
                MAP_UART_transmitData(EUSCI_A0_BASE, (char)UART_DEBUG_FLAG);

                //Send string message
                // REQUIRED: terminate with \n newline. This is the 'end of message' key to look for
//                printf(EUSCI_A0_BASE, "Ring buffer has overrun tail at: %d\n", (unsigned char)UART_RingBuffer.end);
#endif
            }
            //Append data to ring buffer
            UART_RingBuffer.data[UART_RingBuffer.end] = buffer_char;
            //increment for the next ring buffer pass
            UART_RingBuffer.end++;
        }
    }
}

void HALT(){

}


