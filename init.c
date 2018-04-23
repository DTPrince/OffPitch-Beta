/*
 * init.c
 *
 *  Created on: Apr 21, 2018
 *      Author: aetas
 */

#include "defines.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Timer_A PWM Configuration Parameter */
// Top stepper
Timer_A_PWMConfig pwmConfigA01 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        150, // 40 = ~1.5Khz. 120 = 529Hz
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        75
};

// Bottom stepper
Timer_A_PWMConfig pwmConfigA03 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        120, // 40 = ~1.5Khz. 120 = 529Hz
        TIMER_A_CAPTURECOMPARE_REGISTER_3,
        TIMER_A_OUTPUTMODE_RESET_SET,
        60
};

//Experiment stepper
Timer_A_PWMConfig pwmConfigA21 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        100, // 40 = ~1.5Khz. 120 = 529Hz
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        50
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

void initSettings(){
    /* Halting the watchdog */
    MAP_WDT_A_holdTimer();

    /* Setting MCLK to REFO at 128Khz for LF mode
     * Setting SMCLK to 64Khz */
    CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);//
    CS_initClockSignal(CS_MCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_2);
    PCM_setPowerState(PCM_AM_LF_VCORE0);


    /* Set the GPIO */
    //Inputs
    // Top Hall-Effect Sensor
    GPIO_setAsInputPin(HALL_SENSE_VSLOT_TOP);
    GPIO_enableInterrupt(HALL_SENSE_VSLOT_TOP);
    GPIO_interruptEdgeSelect(HALL_SENSE_VSLOT_TOP, GPIO_LOW_TO_HIGH_TRANSITION);
    // Bottom Hall-Effect Sensor
    GPIO_setAsInputPin(HALL_SENSE_VSLOT_BOT);
    GPIO_enableInterrupt(HALL_SENSE_VSLOT_BOT);
    GPIO_interruptEdgeSelect(HALL_SENSE_VSLOT_BOT, GPIO_LOW_TO_HIGH_TRANSITION);
    // HAB-Side Door Hall-Effect sensor
    GPIO_setAsInputPin(HALL_SENSE_HINGE_HAB);
    GPIO_enableInterrupt(HALL_SENSE_HINGE_HAB);
    GPIO_interruptEdgeSelect(HALL_SENSE_HINGE_HAB, GPIO_LOW_TO_HIGH_TRANSITION);
    // SPC-Side Door Hall-Effect sensor
    GPIO_setAsInputPin(HALL_SENSE_HINGE_SPC);
    GPIO_enableInterrupt(HALL_SENSE_HINGE_SPC);
    GPIO_interruptEdgeSelect(HALL_SENSE_HINGE_SPC, GPIO_LOW_TO_HIGH_TRANSITION);
    // Left Capacitive Sensor (as viewed from HAB)
    GPIO_setAsInputPin(CAP_SENSE_TABLE_ONE);
    GPIO_enableInterrupt(CAP_SENSE_TABLE_ONE);
    GPIO_interruptEdgeSelect(CAP_SENSE_TABLE_ONE, GPIO_LOW_TO_HIGH_TRANSITION);
    // Right Capacitive Sensor (as viewed from HAB)
    GPIO_setAsInputPin(CAP_SENSE_TABLE_TWO);
    GPIO_enableInterrupt(CAP_SENSE_TABLE_TWO);
    GPIO_interruptEdgeSelect(CAP_SENSE_TABLE_TWO, GPIO_LOW_TO_HIGH_TRANSITION);
    // Clamp Capacitive Sensor (as viewed from HAB)
    GPIO_setAsInputPin(CAP_SENSE_MATE);
    GPIO_enableInterrupt(CAP_SENSE_MATE);
    GPIO_interruptEdgeSelect(CAP_SENSE_MATE, GPIO_LOW_TO_HIGH_TRANSITION);

    //Outputs
    // Top Stepper Enable (low= enabled)
    GPIO_setAsOutputPin(VSLOT_STEPPER_TOP_EN);
    GPIO_setOutputHighOnPin(VSLOT_STEPPER_TOP_EN);
    // Bottom Stepper Enable (low= enabled)
    GPIO_setAsOutputPin(VSLOT_STEPPER_BOT_EN);
    GPIO_setOutputHighOnPin(VSLOT_STEPPER_BOT_EN);
    // Experiment Stepper Enable (low= enabled)
    GPIO_setAsOutputPin(EXP_STEPPER_EN);
    GPIO_setOutputHighOnPin(EXP_STEPPER_EN);

    // Top stepper direction
    GPIO_setAsOutputPin(VSLOT_STEPPER_TOP_DIR);
    // Bottom stepper direction
    GPIO_setAsOutputPin(VSLOT_STEPPER_BOT_DIR);
    // Experiment stepper direction
    GPIO_setAsOutputPin(EXP_STEPPER_DIR);

    // HAB-side door switch
    GPIO_setAsOutputPin(SWITCH_DOOR_HAB);
    // SPC-side door switch
    GPIO_setAsOutputPin(SWITCH_DOOR_SPC);

    /* PWM Setup */
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_VSLOT_TOP,
                GPIO_PRIMARY_MODULE_FUNCTION);

//    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_VSLOT_TOP,
//                GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_VSLOT_BOT,
                GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_EXP,
                GPIO_PRIMARY_MODULE_FUNCTION);

    /* Setting MCLK to REFO at 128Khz for LF mode
     * Setting SMCLK to 64Khz */
//    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);//
//    MAP_CS_initClockSignal(CS_MCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
//    MAP_CS_initClockSignal(CS_SMCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_2);
//    MAP_PCM_setPowerState(PCM_AM_LF_VCORE0);

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigA01);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigA03);
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigA21);


    /* Enable UART module */
    //UART_initModule(EUSCI_A0_BASE, UART_config);
    UART_enableModule(EUSCI_A0_BASE);

    /* Enabling interrupts and starting the watchdog timer */
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);

    //enable as needed. Pretty much just for capacitive sensors to avoid polling
    Interrupt_enableInterrupt(INT_PORT1);
//    MAP_Interrupt_enableInterrupt(INT_PORT2);
//    MAP_Interrupt_enableInterrupt(INT_PORT3);
//    MAP_Interrupt_enableInterrupt(INT_PORT4);
//    MAP_Interrupt_enableInterrupt(INT_PORT5);
//    MAP_Interrupt_enableInterrupt(INT_PORT6);
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();

    /* Configuring P2.3 as output */
    //DIRECTION
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);
    /* Configuring P1.7 as output */
    //ENABLE
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7);
    //Green DIR=HIGH LED
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    //Blue DIR=LOW LED
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);
    //Red EN=HIGH (disabled) LED
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

//    //EN should be HIGH for disable.
//    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN7);
//    //Turn on Red LED for disable notifier
//    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
//    //DIR should be whatever because we don;t know which way is which
//    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);
//    //Turn on Green LED for DIR=HIGH
//    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
//    //Blue LED=LOW for DIR=HIGH.
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
}

