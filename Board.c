//*****************************************************************************
// Board.c
//
// configure the device pins for different signals
//
// Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// This file was automatically generated on 4/11/2018 at 12:08:18 PM
// by TI PinMux version 4.0.1511 
//
//*****************************************************************************

#include <stdbool.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432.h>

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/rom.h>
#include <ti/devices/msp432p4xx/driverlib/rom_map.h>
#include <ti/devices/msp432p4xx/driverlib/adc14.h>
#include <ti/devices/msp432p4xx/driverlib/dma.h>
#include <ti/devices/msp432p4xx/driverlib/gpio.h>
#include <ti/devices/msp432p4xx/driverlib/i2c.h>
#include <ti/devices/msp432p4xx/driverlib/interrupt.h>
#include <ti/devices/msp432p4xx/driverlib/pmap.h>
#include <ti/devices/msp432p4xx/driverlib/ref_a.h>
#include <ti/devices/msp432p4xx/driverlib/spi.h>
#include <ti/devices/msp432p4xx/driverlib/timer_a.h>
#include <ti/devices/msp432p4xx/driverlib/timer32.h>
#include <ti/devices/msp432p4xx/driverlib/uart.h>
#include <ti/devices/msp432p4xx/driverlib/wdt_a.h>

#include "Board.h"

/*
 *  =============================== ADC ===============================
 */
#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCMSP432.h>

/* ADC objects */
ADCMSP432_Object adcMSP432Objects[Board_ADCCOUNT];

/* ADC configuration structure */
const ADCMSP432_HWAttrsV1 adcMSP432HWAttrs[Board_ADCCOUNT] = {
    {
        .adcPin = ADCMSP432_P5_5_A0,
        .refVoltage = REF_A_VREF2_5V,
        .resolution = ADC_8BIT,
    },
};

const ADC_Config ADC_config[Board_ADCCOUNT] = {
    {
        .fxnTablePtr = &ADCMSP432_fxnTable,
        .object = &adcMSP432Objects[ForceSense0],
        .hwAttrs = &adcMSP432HWAttrs[ForceSense0]
    },
};

const uint_least8_t ADC_count = Board_ADCCOUNT;

/*
 *  =============================== DMA ===============================
 */
#include <ti/drivers/dma/UDMAMSP432.h>

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(dmaControlTable, 256)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=256
#elif defined(__GNUC__)
__attribute__ ((aligned (256)))
#endif
static DMA_ControlTable dmaControlTable[8];

/*
 *  ======== dmaErrorHwi ========
 *  This is the handler for the uDMA error interrupt.
 */
static void dmaErrorHwi(uintptr_t arg)
{
    int status = MAP_DMA_getErrorStatus();
    MAP_DMA_clearErrorStatus();

    /* Suppress unused variable warning */
    (void)status;

    while (1);
}

UDMAMSP432_Object udmaMSP432Object;

const UDMAMSP432_HWAttrs udmaMSP432HWAttrs = {
    .controlBaseAddr = (void *)dmaControlTable,
    .dmaErrorFxn = (UDMAMSP432_ErrorFxn)dmaErrorHwi,
    .intNum = INT_DMA_ERR,
    .intPriority = (~0)
};

const UDMAMSP432_Config UDMAMSP432_config = {
    .object = &udmaMSP432Object,
    .hwAttrs = &udmaMSP432HWAttrs
};

/*
 *  ============================= Display =============================
 */
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#define MAXPRINTLEN 1024

DisplayUart_Object displayUartObject;

static char displayBuf[MAXPRINTLEN];

const DisplayUart_HWAttrs displayUartHWAttrs = {
    .uartIdx = Board_UART0,
    .baudRate = 115200,
    .mutexTimeout = (unsigned int)(-1),
    .strBuf = displayBuf,
    .strBufLen = MAXPRINTLEN
};

const Display_Config Display_config[] = {
    {
#  if defined(BOARD_DISPLAY_UART_USE_ANSI)
        .fxnTablePtr = &DisplayUartAnsi_fxnTable,
#  else /* Default to minimal UART with no cursor placement */
        .fxnTablePtr = &DisplayUartMin_fxnTable,
#  endif
        .object = &displayUartObject,
        .hwAttrs = &displayUartHWAttrs
    },
};

const uint_least8_t Display_count = sizeof(Display_config) / sizeof(Display_Config);

/*
 *  ======== MSP_EXP432P401R_initGeneral ========
 */
void MSP_EXP432P401R_initGeneral(void)
{
    Power_init();
}

/*
 *  =============================== GPIO ===============================
 */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOMSP432.h>

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in MSP_EXP432P401R.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array.  Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {
    GPIOMSP432_P1_1 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING,
    GPIOMSP432_P1_4 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING,
    GPIOMSP432_P1_0 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
    GPIOMSP432_P2_0 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
    GPIOMSP432_P2_1 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
    GPIOMSP432_P2_2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
    GPIOMSP432_P1_5 | GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_BOTH_EDGES,
    GPIOMSP432_P1_6 | GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_BOTH_EDGES,
    GPIOMSP432_P1_7 | GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_BOTH_EDGES,
    GPIOMSP432_P2_5 | GPIO_CFG_OUT_OD_PD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_HIGH,
    GPIOMSP432_P2_6 | GPIO_CFG_OUT_OD_PD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_HIGH,
    GPIOMSP432_P2_7 | GPIO_CFG_OUT_OD_PD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_HIGH,
    GPIOMSP432_P3_0 | GPIO_CFG_OUT_OD_PD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_HIGH,
    GPIOMSP432_P3_2 | GPIO_CFG_IN_PD | GPIO_CFG_IN_INT_BOTH_EDGES,
    GPIOMSP432_P3_3 | GPIO_CFG_IN_PD | GPIO_CFG_IN_INT_BOTH_EDGES,
    GPIOMSP432_P3_5 | GPIO_CFG_IN_PD | GPIO_CFG_IN_INT_BOTH_EDGES,
    GPIOMSP432_P3_6 | GPIO_CFG_IN_PD | GPIO_CFG_IN_INT_BOTH_EDGES,
    GPIOMSP432_P3_7 | GPIO_CFG_IN_PD | GPIO_CFG_IN_INT_BOTH_EDGES,
    GPIOMSP432_P4_0 | GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_NONE,
    GPIOMSP432_P4_1 | GPIO_CFG_OUT_OD_PU | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
    GPIOMSP432_P4_2 | GPIO_CFG_OUT_OD_PU | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
    GPIOMSP432_P4_4 | GPIO_CFG_OUT_OD_PD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_HIGH,
    GPIOMSP432_P4_5 | GPIO_CFG_OUT_OD_PD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_HIGH,
    GPIOMSP432_P4_6 | GPIO_CFG_OUT_OD_PU | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_LOW,
    GPIOMSP432_P4_7 | GPIO_CFG_OUT_OD_PD | GPIO_CFG_OUT_STR_HIGH | GPIO_CFG_OUT_HIGH,
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in MSP_EXP432P401R.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,
    NULL
};

const GPIOMSP432_Config GPIOMSP432_config = {
    .pinConfigs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = sizeof(gpioPinConfigs)/sizeof(GPIO_PinConfig),
    .numberOfCallbacks = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority = (~0)
};
/*
 *  =============================== Power ===============================
 */
const PowerMSP432_ConfigV1 PowerMSP432_config = {
    .policyInitFxn = &PowerMSP432_initPolicy,
    .policyFxn = &PowerMSP432_sleepPolicy,
    .initialPerfLevel = 2,
    .enablePolicy = true,
    .enablePerf = true,
    .enableParking = true
};
/*
 *  =============================== PWM ===============================
 */
#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerMSP432.h>

PWMTimerMSP432_Object pwmTimerMSP432Objects[Board_PWMCOUNT];

const PWMTimerMSP432_HWAttrsV2 pwmTimerMSP432HWAttrs[Board_PWMCOUNT] = {
    {        
        .clockSource = TIMER_A_CLOCKSOURCE_ACLK,
        .pwmPin = PWMTimerMSP432_P2_3_TA0CCR1A,
    },
    {        
        .clockSource = TIMER_A_CLOCKSOURCE_ACLK,
        .pwmPin = PWMTimerMSP432_P2_4_TA1CCR1A,
    },
    {        
        .clockSource = TIMER_A_CLOCKSOURCE_ACLK,
        .pwmPin = PWMTimerMSP432_P5_6_TA2CCR1A,
    },
};

const PWM_Config PWM_config[Board_PWMCOUNT] = {
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[PWMVSlotTop_Out1],
        .hwAttrs = &pwmTimerMSP432HWAttrs[PWMVSlotTop_Out1]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[PWMVSlotBot_Out1],
        .hwAttrs = &pwmTimerMSP432HWAttrs[PWMVSlotBot_Out1]
    },
    {
        .fxnTablePtr = &PWMTimerMSP432_fxnTable,
        .object = &pwmTimerMSP432Objects[PWMTableClamp_Out1],
        .hwAttrs = &pwmTimerMSP432HWAttrs[PWMTableClamp_Out1]
    },
};

const uint_least8_t PWM_count = Board_PWMCOUNT;
/*
 *  =============================== Timer ===============================
 */
#include <ti/drivers/Timer.h>
#include <ti/drivers/timer/TimerMSP432.h>

TimerMSP432_Object timerMSP432Objects[Board_TIMERCOUNT];

const TimerMSP432_HWAttrs timerMSP432HWAttrs[Board_TIMERCOUNT] = {
    {
        .timerBaseAddress = TIMER32_0_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_ACLK,
        .intNum = INT_T32_INT1,
        .intPriority = ~0
    },
    {        
        .timerBaseAddress = TIMER_A0_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_ACLK,
        .intNum = INT_TA0_0,
        .intPriority = ~0
    },
    {        
        .timerBaseAddress = TIMER_A1_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_ACLK,
        .intNum = INT_TA1_0,
        .intPriority = ~0
    },
    {        
        .timerBaseAddress = TIMER_A2_BASE,
        .clockSource = TIMER_A_CLOCKSOURCE_ACLK,
        .intNum = INT_TA2_0,
        .intPriority = ~0
    },
};

const Timer_Config Timer_config[Board_TIMERCOUNT] = {
    {
        .fxnTablePtr = &TimerMSP432_Timer32_fxnTable,
        .object = &timerMSP432Objects[ACLKTimer],
        .hwAttrs = &timerMSP432HWAttrs[ACLKTimer]
    },
    {
        .fxnTablePtr = &TimerMSP432_Timer_A_fxnTable,
        .object = &timerMSP432Objects[PWMVSlotTop_TIMER],
        .hwAttrs = &timerMSP432HWAttrs[PWMVSlotTop_TIMER]
    },
    {
        .fxnTablePtr = &TimerMSP432_Timer_A_fxnTable,
        .object = &timerMSP432Objects[PWMVSlotBot_TIMER],
        .hwAttrs = &timerMSP432HWAttrs[PWMVSlotBot_TIMER]
    },
    {
        .fxnTablePtr = &TimerMSP432_Timer_A_fxnTable,
        .object = &timerMSP432Objects[PWMTableClamp_TIMER],
        .hwAttrs = &timerMSP432HWAttrs[PWMTableClamp_TIMER]
    },
};

const uint_least8_t Timer_count = Board_TIMERCOUNT;

/*
 *  =============================== UART ===============================
 */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTMSP432.h>

UARTMSP432_Object uartMSP432Objects[Board_UARTCOUNT];
unsigned char uartMSP432RingBuffer[Board_UARTCOUNT][32];

/*
 * The baudrate dividers were determined by using the MSP432 baudrate
 * calculator
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const UARTMSP432_BaudrateConfig uartMSP432Baudrates[] = {
    /* {baudrate, input clock, prescalar, UCBRFx, UCBRSx, oversampling} */
    {
        .outputBaudrate = 115200,
        .inputClockFreq = 12000000,
        .prescalar = 6,
        .hwRegUCBRFx = 8,
        .hwRegUCBRSx = 32,
        .oversampling = 1
    },
    {115200, 6000000,   3,  4,   2, 1},
    {115200, 3000000,   1, 10,   0, 1},
    {9600,   12000000, 78,  2,   0, 1},
    {9600,   6000000,  39,  1,   0, 1},
    {9600,   3000000,  19,  8,  85, 1},
    {9600,   32768,     3,  0, 146, 0}
};

const UARTMSP432_HWAttrsV1 uartMSP432HWAttrs[Board_UARTCOUNT] = {
    {
        .baseAddr = EUSCI_A0_BASE,
        .intNum = INT_EUSCIA0,
        .intPriority = (~0),
        .clockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        .bitOrder = EUSCI_A_UART_LSB_FIRST,
        .numBaudrateEntries = sizeof(uartMSP432Baudrates) /
            sizeof(UARTMSP432_BaudrateConfig),
        .baudrateLUT = uartMSP432Baudrates,
        .ringBufPtr  = uartMSP432RingBuffer[Board_UART0],
        .ringBufSize = sizeof(uartMSP432RingBuffer[Board_UART0]),
        .rxPin = UARTMSP432_P1_2_UCA0RXD,
        .txPin = UARTMSP432_P1_3_UCA0TXD,
    },
};

const UART_Config UART_config[Board_UARTCOUNT] = {
    {
        .fxnTablePtr = &UARTMSP432_fxnTable,
        .object = &uartMSP432Objects[Board_UART0],
        .hwAttrs = &uartMSP432HWAttrs[Board_UART0]
    }
};

const uint_least8_t UART_count = Board_UARTCOUNT;
