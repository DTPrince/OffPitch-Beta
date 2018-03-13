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
        32000,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        3200
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

int main(void) {
    /* Halting the watchdog */
    MAP_WDT_A_holdTimer();

    /* Configuring P2.3 as output */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);

    /* Setting MCLK to REFO at 128Khz for LF mode
     * Setting SMCLK to 64Khz */
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
    MAP_CS_initClockSignal(CS_MCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_2);
    MAP_PCM_setPowerState(PCM_AM_LF_VCORE0);

    /* Configuring GPIO2.4 as peripheral output for PWM  and P6.7 for button
     * interrupt */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
            GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Setting DCO to 12MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    /* Configuring Timer_A to have a period of approximately 500ms and
     * an initial duty cycle of 10% of that (3200 ticks)  */
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_BASE);
    //UART_enableModule(EUSCI_A0_BASE);

    /* Enabling interrupts and starting the watchdog timer */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_Interrupt_enableInterrupt(INT_PORT1);
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();

    struct commandPacket packet;

    bool lastAction_notCompleted = false;

    UART_RingBuffer.end = 0;
    UART_RingBuffer.start = 0;

    /* Sleeping when not in use */
    while (1)
    {
        MAP_PCM_gotoLPM0();
        //MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);

        //check if the two pointers are equal
        // if they are equal then there is data in the ring buffer
        if ((UART_RingBuffer.end != UART_RingBuffer.start) && !lastAction_notCompleted){
            parse_UART_RingBuffer(&packet);
            if (packet.command == 0x13){    //this needs a lot more logic
                moveVSlot_HAB();
            }
        }
    }
}

/* Port1 ISR - This ISR will progressively step up the duty cycle of the PWM
 * on a button press
 */
void PORT1_IRQHandler(void)
{
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if (status & GPIO_PIN1)
    {
        if(pwmConfig.dutyCycle == 28800)
            pwmConfig.dutyCycle = 3200;
        else
            pwmConfig.dutyCycle += 3200;

        MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig);
    }
}
/* EUSCI A0 UART ISR - Fetches char from UART and stores in UART_RingBuffer for later */
void EUSCIA0_IRQHandler(void)
{
    //fetch status register for interrupt data
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    //clear interrupt flag
    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    //
    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        char buffer_char = MAP_UART_receiveData(EUSCI_A0_BASE);

        //Clean out spaces and formatting for the buffer
        if (buffer_char != ('\n' || '\r' || '\t' || ' ')){
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
                printf(EUSCI_A0_BASE, "Ring buffer has overrun tail at: %d\n", (unsigned char)UART_RingBuffer.end);
#endif
            }
            //Append data to ring buffer
            UART_RingBuffer.data[UART_RingBuffer.end] = buffer_char;
            //increment for the next ring buffer pass
            UART_RingBuffer.end++;
        }
    }
}
