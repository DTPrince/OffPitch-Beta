//*****************************************************************************
// Board.h
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

// This file was automatically generated on 4/2/2018 at 11:13:14 PM
// by TI PinMux version 4.0.1511 
//
//*****************************************************************************
#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* LEDs on MSP_EXP432P401R are active high. */
#define Board_GPIO_LED_OFF (0)
#define Board_GPIO_LED_ON  (1)

#define Board_initADC               ADC_init
#define Board_initADCBuf            ADCBuf_init
#define Board_initGeneral           MSP_EXP432P401R_initGeneral
#define Board_initGPIO              GPIO_init
#define Board_initI2C               I2C_init
#define Board_initPWM               PWM_init
#define Board_initSDSPI             SDSPI_init
#define Board_initSPI               SPI_init
#define Board_initUART              UART_init
#define Board_initWatchdog          Watchdog_init

/* Board specific I2C addresses */
#define Board_TMP_ADDR              (0x40)
#define Board_RF430CL330_ADDR       (0x28)
#define Board_TPL0401_ADDR          (0x40)

/*!
 *  @def    MSP_EXP432P401R_ADCName
 *  @brief  Enum of ADC channels on the MSP_EXP432P401R dev board
 */
typedef enum Board_ADCName {
    ForceSense0 = 0,
    Board_ADCCOUNT
} Board_ADCName;

/*!
 *  @def    MSP_EXP432P401R_GPIOName
 *  @brief  Enum of GPIO names on the MSP_EXP432P401R dev board
 */
typedef enum Board_GPIOName {
    Board_GPIO_BUTTON0 = 0,
    Board_GPIO_BUTTON1 = 1,
    Board_GPIO_LED0 = 2,
    Board_GPIO_LED1 = 3,
    Board_GPIO_LED2 = 4,
    Board_GPIO_LED3 = 5,
    TableCapSense0 = 6,
    TableCapSense1 = 7,
    PlateCapSense = 8,
    Relay0 = 9,
    Relay1 = 10,
    DoorSwitch0 = 11,
    DoorSwitch1 = 12,
    VSlotHESense0 = 13,
    VSlotHESense1 = 14,
    DoorHESense0 = 15,
    DoorHESense1 = 16,
    HingeHESense0 = 17,
    HingeHESense1 = 18,
    Board_GPIOCOUNT
} Board_GPIOName;

/*!
 *  @def    MSP_EXP432P401R_PWMName
 *  @brief  Enum of PWM names on the MSP_EXP432P401R dev board
 */
typedef enum Board_PWMName {
    PWMVSlotTop_Out1 = 0,
    PWMVSlotBot_Out1 = 1,
    PWMTableClamp_Out1 = 2,
    Board_PWMCOUNT
} Board_PWMName;
    
/*!
 *  @def    MSP_EXP432P401R_TimerName
 *  @brief  Enum of Timer names on the MSP_EXP432P401R dev board
 */
typedef enum Board_TimerName {
    ACLKTimer = 0,
    PWMVSlotTop_TIMER = 1,
    PWMVSlotBot_TIMER = 2,
    PWMTableClamp_TIMER = 3,
   Board_TIMERCOUNT
} Board_TimerName;

/*!
 *  @def    MSP_EXP432P401R_UARTName
 *  @brief  Enum of UART names on the MSP_EXP432P401R dev board
 */
typedef enum Board_UARTName {
    Board_UART0 = 0,
    Board_UARTCOUNT
} Board_UARTName;
    

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
extern void Board_initGeneral(void);


#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
