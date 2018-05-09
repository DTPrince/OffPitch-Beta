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
#include <string.h> // needed for memset

/* Project includes */
#include "defines.h"
#include "printf.h"
#include "peripheral_interfaces.h"


Timer_A_PWMConfig pwmConfigA01 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_64,
        PWM_COUNTER_VSLOT_TOP,    // 400 = 470.28Hz
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0       // 200
};

// Bottom stepper
Timer_A_PWMConfig pwmConfigA03 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_64,
        PWM_COUNTER_VSLOT_BOT,    // 400 = 470.28Hz
        TIMER_A_CAPTURECOMPARE_REGISTER_3,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0       // 200
};

//Experiment stepper
Timer_A_PWMConfig pwmConfigA21 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_64,
        PWM_COUNTER_EXP,    // 40 = ~1.5Khz. 120 = 529Hz
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0       // 275
};
/* UART Config */
//9600 BAUD
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
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION //EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION // Oversampling
};

// global
volatile uint32_t Tick;

void initSettings(){
    /* Halting the watchdog */
    WDT_A_holdTimer();
    heSense.vslot.top = true;
    heSense.vslot.bot = true;


    /* Set the GPIO */

    //Inputs
    // Top Hall-Effect Sensor
    GPIO_setAsInputPin(HALL_SENSE_VSLOT_TOP);
//    GPIO_enableInterrupt(HALL_SENSE_VSLOT_TOP);
//    GPIO_interruptEdgeSelect(HALL_SENSE_VSLOT_TOP, GPIO_LOW_TO_HIGH_TRANSITION);
    // Bottom Hall-Effect Sensor
    GPIO_setAsInputPin(HALL_SENSE_VSLOT_BOT);
//    w3 GPIO_interruptEdgeSelect(HALL_SENSE_VSLOT_BOT, GPIO_LOW_TO_HIGH_TRANSITION);
    // HAB-Side Door Hall-Effect sensor
    GPIO_setAsInputPin(HALL_SENSE_HINGE_HAB);
//    GPIO_enableInterrupt(HALL_SENSE_HINGE_HAB);
//    GPIO_interruptEdgeSelect(HALL_SENSE_HINGE_HAB, GPIO_LOW_TO_HIGH_TRANSITION);
    // SPC-Side Door Hall-Effect sensor
    GPIO_setAsInputPin(HALL_SENSE_HINGE_SPC);
//    GPIO_enableInterrupt(HALL_SENSE_HINGE_SPC);
//    GPIO_interruptEdgeSelect(HALL_SENSE_HINGE_SPC, GPIO_LOW_TO_HIGH_TRANSITION);
    // Left Capacitive Sensor (as viewed from HAB)
    GPIO_setAsInputPin(CAP_SENSE_TABLE_ONE);
//    GPIO_enableInterrupt(CAP_SENSE_TABLE_ONE);
//    GPIO_interruptEdgeSelect(CAP_SENSE_TABLE_ONE, GPIO_LOW_TO_HIGH_TRANSITION);
    // Right Capacitive Sensor (as viewed from HAB)
    GPIO_setAsInputPin(CAP_SENSE_TABLE_TWO);
//    GPIO_enableInterrupt(CAP_SENSE_TABLE_TWO);
//    GPIO_interruptEdgeSelect(CAP_SENSE_TABLE_TWO, GPIO_LOW_TO_HIGH_TRANSITION);
    // Clamp Capacitive Sensor (as viewed from HAB)
    GPIO_setAsInputPin(CAP_SENSE_MATE);
//    GPIO_enableInterrupt(CAP_SENSE_MATE);
//    GPIO_interruptEdgeSelect(CAP_SENSE_MATE, GPIO_LOW_TO_HIGH_TRANSITION);

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
    GPIO_setOutputHighOnPin(VSLOT_STEPPER_TOP_DIR);
    // Bottom stepper direction
    GPIO_setAsOutputPin(VSLOT_STEPPER_BOT_DIR);
    GPIO_setOutputLowOnPin(VSLOT_STEPPER_BOT_DIR);
    // Experiment stepper direction
    GPIO_setAsOutputPin(EXP_STEPPER_DIR);
    GPIO_setOutputLowOnPin(EXP_STEPPER_DIR);

    // HAB-side door switch
    GPIO_setAsOutputPin(SWITCH_DOOR_HAB);
    GPIO_setOutputLowOnPin(SWITCH_DOOR_HAB);
    // SPC-side door switch
    GPIO_setAsOutputPin(SWITCH_DOOR_SPC);
    GPIO_setOutputLowOnPin(SWITCH_DOOR_SPC);

    /* ADC Setup */
    /* Zero-filling buffer */

    //![Simple ADC14 Configure]
    /* Initializing ADC (MCLK/1/1) */
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,
            0);

    /* Configuring ADC Memory (ADC_MEM0 A0/A1) in repeat mode
     * with use of external references */
    ADC14_configureSingleSampleMode(ADC_MEM0, true);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_EXTPOS_VREFNEG_EXTNEG,
            ADC_INPUT_A0, false);

    /* Setting up GPIO pins as analog inputs (and references) */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
            GPIO_PIN7 | GPIO_PIN6 | GPIO_PIN5 | GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Enabling sample timer in auto iteration mode and interrupts*/
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
    //ADC14_enableInterrupt(ADC_INT0);

    /* Triggering the start of the sample */
    ADC14_enableConversion();
    ADC14_toggleConversionTrigger();
    //![Simple ADC14 Configure]

    /* PWM Setup */
    //P2.6 as TA0.3 PWM output (shares freq. with PWM_VSLOT_BOT)
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_VSLOT_TOP,
                GPIO_PRIMARY_MODULE_FUNCTION);

    //P2.4 as TA0.1 PWM output (shares freq. with PWM_VSLOT_TOP)
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_VSLOT_BOT,
                GPIO_PRIMARY_MODULE_FUNCTION);

    //P5.6 as TA2.1 PWM Output. Stand-alone frequency
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_EXP,
                GPIO_PRIMARY_MODULE_FUNCTION);

    /* Clock setups */
    /* Setting MCLK to REFO at 128Khz for LF mode
     * Setting SMCLK to 64Khz */
    CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);//
    CS_initClockSignal(CS_MCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
//    CS_initClockSignal(CS_SMCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    PCM_setPowerState(PCM_AM_LF_VCORE0);

    /* Selecting P1.2 and P1.3 in UART mode */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
           GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);


    // Generate PWM timers.
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigA01);  //vslot bot
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigA03);  //vslot top
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigA21);  //table clamp


    /* Enable UART module */
    UART_initModule(UART_BASE, &uartConfig);
    UART_enableModule(UART_BASE);


    /* Enabling interrupts and starting the watchdog timer */
    UART_enableInterrupt(UART_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0); // herrr


//    enable as needed. Pretty much just for capacitive sensors to avoid polling
//    Interrupt_enableInterrupt(INT_PORT1);
//    Interrupt_enableInterrupt(INT_PORT2);
//    Interrupt_enableInterrupt(INT_PORT3);
//    Interrupt_enableInterrupt(INT_PORT4);
//    MAP_Interrupt_enableInterrupt(INT_PORT5);
//    MAP_Interrupt_enableInterrupt(INT_PORT6);
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();


    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);

    SysTick_enableModule();
    SysTick_setPeriod(1500000); //Depends on your clock and tick requirements
    SysTick_enableInterrupt();

    /* Enabling MASTER interrupts */
    Interrupt_enableMaster();

//    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
//    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

}

void SysTick_Handler(void){
    Tick++;
}

// assumes 1 ms tick.
void delay(uint32_t delay_ms){
    uint32_t start = Tick;

    while (Tick - start < delay_ms);
}

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

uint8_t disable_PWM(int port, int pin){
    if((port == PWM_VSLOT_TOP_PORT) && (pin == PWM_VSLOT_TOP_PIN)){
        //change duty-cycle to 0, effectively disabling the PWM signal
        pwmConfigA03.dutyCycle = 0;
        //reconfigure PWM
        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigA03);  //vslot bot

    }
    if((port ==  PWM_VSLOT_BOT_PORT) && (pin == PWM_VSLOT_BOT_PIN)) {
        //change duty-cycle to 0, effectively disabling the PWM signal
        pwmConfigA01.dutyCycle = 0;
        //reconfigure PWM
        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigA01);  //vslot top

    }
    if((port == PWM_EXP_PORT) && (pin == PWM_EXP_PIN)) {
        //change duty-cycle to 0, effectively disabling the PWM signal
        pwmConfigA21.dutyCycle = 0;
        //reconfigure PWM
        Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigA21);  //table clamp

    }
    return 0;
}

uint8_t enable_PWM(int port, int pin){
    if((port == PWM_VSLOT_TOP_PORT) && (pin == PWM_VSLOT_TOP_PIN)){
        //change duty-cycle to 200 for a 50% duty-cycle
        pwmConfigA03.dutyCycle = PWM_COUNTER_VSLOT_TOP/2;

        //reconfigure PWM
        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigA03);  //vslot bot

    }
    if((port ==  PWM_VSLOT_BOT_PORT) && (pin == PWM_VSLOT_BOT_PIN)) {
        //change duty-cycle to 200 for a 50% duty-cycle
        pwmConfigA01.dutyCycle = PWM_COUNTER_VSLOT_BOT/2;
        //reconfigure PWM
        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigA01);  //vslot top

    }
    if((port == PWM_EXP_PORT) && (pin == PWM_EXP_PIN)) {
        //change duty-cycle to 275 for a 50% duty-cycle
        pwmConfigA21.dutyCycle = PWM_COUNTER_EXP/2;//275;
        //reconfigure PWM
        Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfigA21);  //table clamp
    }
     return 0;
}

uint8_t set_stepperDirection(int port, int pin, bool direction){
    if(direction){
        GPIO_setOutputHighOnPin(port, pin);
    } else {
        GPIO_setOutputLowOnPin(port, pin);
    }
    return 0;
}

uint8_t set_stepperSpeed(int port, int pin, int speed){
    //modify timer struct duty cycle here
    return 0;
}

uint8_t get_DIOPinState(int port, int pin){
    return GPIO_getInputPinValue(port, pin);
}

uint16_t get_ADCValue(int stepper_port, int pin){
//    adcResult = ADC14_getResult(ADC_MEM0);
    return adcResult;
}


