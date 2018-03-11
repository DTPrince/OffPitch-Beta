/*
 * defines.h
 *
 *  Created on: Mar 10, 2018
 *      Author: Derek Prince
 */

#ifndef DEFINES_H_
#define DEFINES_H_

/* Defines UART RingBuffer size */
#define BUFFER_SIZE 256 //512

/* Debug Levels for building Serial/printf */
/* 0 - No debugging info
 * 1 - Critical Events Only
 * 2 - Something in between that I haven't decided on.
 * 3 - All values
 */
#define DEBUG_NONE                              0
#define DEBUG_CRITICAL                          1
#define DEBUG_INTERMEDIATE                      2
#define DEBUG_VERBOSE                           3
//Debug level
#define DEBUG_LEVEL                             DEBUG_CRITICAL

/* UART Packet masks */
#define UART_TYPE                               0b11000000
#define UART_COMMAND                            0b00111111

/* UART Debugging Flag */
#define UART_DEBUG_FLAG                         0xFF

/* UART packet types */
#define UART_TYPE_CONTROL                       0x00    // 0b00000000
#define UART_TYPE_DATA                          0x40    // 0b01000000
#define UART_TYPE_COMMAND                       0x80    // 0b10000000
#define UART_TYPE_ACK                           0xC0    // 0b11000000

/* UART Packet Commands */
//Control Commands
#define UART_COMMAND_NOP                        0x00
#define UART_COMMAND_HALT                       0x3F
#define UART_COMMAND_BURP                       0x3E

//Table Commands
#define UART_COMMAND_CLAMP_EXPERIMENT           0x11
#define UART_COMMAND_RELEASE_EXPERIMENT         0x12
#define UART_COMMAND_MV_TABLE_HAB               0x13
#define UART_COMMAND_MV_TABLE_CEN               0x14
#define UART_COMMAND_MV_TABLE_SPC               0x15

//Door Commands
#define UART_COMMAND_OPEN_HAB                   0x16
#define UART_COMMAND_OPEN_SPC                   0x17
#define UART_COMMAND_CLOSE_HAB                  0x18
#define UART_COMMAND_CLOSE_SPC                  0x19

//Data commands
#define UART_COMMAND_DATA_TABLE_CAP_SENSE       0x20
#define UART_COMMAND_DATA_PLATE_CAP_SENSE       0x21
#define UART_COMMAND_DATA_TABLE_FORCE_SENSE     0x22
#define UART_COMMAND_DATA_HAB_DOOR_HE_SENSE     0x23
#define UART_COMMAND_DATA_SPC_DOOR_HE_SENSE     0x24
#define UART_COMMAND_DATA_HAB_HINGE_HE_SENSE    0x25
#define UART_COMMAND_DATA_SPC_HINGE_HE_SENSE    0x26
#define UART_COMMAND_DATA_TEMPERATURE           0x27

/* Packet byte sizes */
#define UART_COMMAND_PACKET_TYPECMD_SIZE        1
#define UART_COMMAND_PACKET_LENGTH_SIZE         1

/* Pin defines */
//Serial
#define COMMAND_SERIAL_PORT                     0

// Digital I/O
//Inputs
#define CAP_SENSE_TABLE                         0
#define CAP_SENSE_PLATE                         0
#define HALL_SENSE_VSLOT_ONE                    0
#define HALL_SENSE_VSLOT_TWO                    0
#define HALL_SENSE_DOOR_HAB                     0
#define HALL_SENSE_DOOR_SPC                     0
#define HALL_SENSE_HINGE_HAB                    0
#define HALL_SENSE_HINGE_SPC                    0

//Outputs
#define VSLOT_ONE_ENABLE                        0
#define VSLOT_TWO_ENABLE                        0
#define VSLOT_ONE_DIRECTION                     0
#define VSLOT_TWO_DIRECTION                     0
#define RELAY_DOOR_HAB                          0
#define RELAY_DOOR_SPC                          0

#define BOARD_LED_ONE                           0
#define BOARD_LED_TWO                           0
#define BOARD_LED_THREE                         0
#define BOARD_LED_FOUR                          0

// ADC I/O
#define FORCE_SENSE_PLATE                       0

// PWM
#define PWM_VSLOT_ONE                           0
#define PWM_VSLOT_TWO                           0
#define PWM_VSLOT_TABLE                         0




#endif /* DEFINES_H_ */
