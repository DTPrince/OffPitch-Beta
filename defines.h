/*
 * defines.h
 *
 *  Created on: Mar 10, 2018
 *      Author: Derek Prince
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>

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

#define TABLE_POSITION_SPC                      0b100
#define TABLE_POSITION_CEN                      0b010
#define TABLE_POSITION_HAB                      0b001
//Table position state
//0b100 : Table space side | TABLE_POSITION_SPC
//0b010 : Table center     | TABLE_POSITION_CEN
//0b001 : Table hab side   | TABLE_POSITION_HAB
uint8_t tablePosition;

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
// Hall-Effect sensors //
//Top VSlot HE sensor
#define HALL_SENSE_VSLOT_TOP_PORT               GPIO_PORT_P1
#define HALL_SENSE_VSLOT_TOP_PIN                GPIO_PIN1
#define HALL_SENSE_VSLOT_TOP                    HALL_SENSE_VSLOT_TOP_PORT, HALL_SENSE_VSLOT_TOP_PIN
//Bottom VSlot HE sensor
#define HALL_SENSE_VSLOT_BOT_PORT               GPIO_PORT_P1
#define HALL_SENSE_VSLOT_BOT_PIN                GPIO_PIN1
#define HALL_SENSE_VSLOT_BOT                    HALL_SENSE_VSLOT_BOT_PORT, HALL_SENSE_VSLOT_BOT_PIN

//Door closed HE Sensor - HAB
#define HALL_SENSE_DOOR_HAB_PORT                GPIO_PORT_P1
#define HALL_SENSE_DOOR_HAB_PIN                 GPIO_PIN1
#define HALL_SENSE_DOOR_HAB                     HALL_SENSE_DOOR_HAB_PORT, HALL_SENSE_DOOR_HAB_PIN
//Door closed HE Sensor - SPC
#define HALL_SENSE_DOOR_SPC_PORT                GPIO_PORT_P1
#define HALL_SENSE_DOOR_SPC_PIN                 GPIO_PIN1
#define HALL_SENSE_DOOR_SPC                     HALL_SENSE_DOOR_SPC_PORT, HALL_SENSE_DOOR_SPC_PIN

//Door open HE sensor - HAB
#define HALL_SENSE_HINGE_HAB_PORT               GPIO_PORT_P1
#define HALL_SENSE_HINGE_HAB_PIN                GPIO_PIN1
#define HALL_SENSE_HINGE_HAB                    HALL_SENSE_HINGE_HAB_PORT, HALL_SENSE_HINGE_HAB_PIN
//Door open HE sensor - SPC
#define HALL_SENSE_HINGE_SPC_PORT               GPIO_PORT_P1
#define HALL_SENSE_HINGE_SPC_PIN                GPIO_PIN1
#define HALL_SENSE_HINGE_SPC                    HALL_SENSE_HINGE_SPC_PORT, HALL_SENSE_HINGE_SPC_PIN

/* Capacitive Sensors */
//Left capacitive sensor on the table as viewed from the HAB
#define CAP_SENSE_TABLE_ONE_PORT                GPIO_PORT_P1
#define CAP_SENSE_TABLE_ONE_PIN                 GPIO_PIN1
#define CAP_SENSE_TABLE_ONE                     CAP_SENSE_TABLE_ONE_PORT, CAP_SENSE_TABLE_ONE_PIN
//Right capacitive sensor on the table as viewed from the HAB
#define CAP_SENSE_TABLE_TWO_PORT                GPIO_PORT_P1
#define CAP_SENSE_TABLE_TWO_PIN                 GPIO_PIN1
#define CAP_SENSE_TABLE_TWO                     CAP_SENSE_TABLE_TWO_PORT, CAP_SENSE_TABLE_TWO_PIN

//Capacitive Sensor on the mating connection
#define CAP_SENSE_MATE_PORT                     GPIO_PORT_P1
#define CAP_SENSE_MATE_PIN                      GPIO_PIN1
#define CAP_SENSE_MATE                          CAP_SENSE_MATE_PORT, CAP_SENSE_MATE_PIN

//Outputs
//Steppers //
//Top stepper - Enable
#define VSLOT_STEPPER_TOP_EN_PORT               GPIO_PORT_P1
#define VSLOT_STEPPER_TOP_EN_PIN                GPIO_PIN1
#define VSLOT_STEPPER_TOP_EN                    VSLOT_STEPPER_TOP_EN_PORT, VSLOT_STEPPER_TOP_EN_PIN
//Top stepper - Direction
#define VSLOT_STEPPER_TOP_DIR_PORT              GPIO_PORT_P1
#define VSLOT_STEPPER_TOP_DIR_PIN               GPIO_PIN1
#define VSLOT_STEPPER_TOP_DIR                   VSLOT_STEPPER_TOP_DIR_PORT, VSLOT_STEPPER_TOP_DIR_PIN

//Bottom stepper - Enable
#define VSLOT_STEPPER_BOT_EN_PORT               GPIO_PORT_P1
#define VSLOT_STEPPER_BOT_EN_PIN                GPIO_PIN1
#define VSLOT_STEPPER_BOT_EN                    VSLOT_STEPPER_BOT_EN_PORT, VSLOT_STEPPER_BOT_EN_PIN
//Bottom stepper - Direction
#define VSLOT_STEPPER_BOT_DIR_PORT              GPIO_PORT_P1
#define VSLOT_STEPPER_BOT_DIR_PIN               GPIO_PIN1
#define VSLOT_STEPPER_BOT_DIR                   VSLOT_STEPPER_BOT_DIR_PORT, VSLOT_STEPPER_BOT_DIR_PIN

//Experiment tray stepper - Enable
#define EXP_STEPPER_EN_PORT                     GPIO_PORT_P1
#define EXP_STEPPER_EN_PIN                      GPIO_PIN1
#define EXP_STEPPER_EN                          EXP_STEPPER_EN_PORT, EXP_STEPPER_EN_PIN
//Experiment tray stepper - Direction
#define EXP_STEPPER_DIR_PORT                    GPIO_PORT_P1
#define EXP_STEPPER_DIR_PIN                     GPIO_PIN1
#define EXP_STEPPER_DIR                         EXP_STEPPER_DIR_PORT, EXP_STEPPER_DIR_PIN

// Relay Controls //
//Hab-side relay
#define RELAY_DOOR_HAB_PORT                     GPIO_PORT_P1
#define RELAY_DOOR_HAB_PIN                      GPIO_PIN1
#define RELAY_DOOR_HAB                          RELAY_DOOR_HAB_PORT, RELAY_DOOR_HAB_PIN
//Space-side relay
#define RELAY_DOOR_SPC_PORT                     GPIO_PORT_P1
#define RELAY_DOOR_SPC_PIN                      GPIO_PIN1
#define RELAY_DOOR_SPC                          RELAY_DOOR_SPC_PORT, RELAY_DOOR_SPC_PIN

// Board LEDs //
#define BOARD_LED_ONE_PORT                      GPIO_PORT_P1
#define BOARD_LED_ONE_PIN                       GPIO_PIN1
#define BOARD_LED_ONE                           BOARD_LED_ONE_PORT, BOARD_LED_ONE_PIN

#define BOARD_LED_TWO_PORT                      GPIO_PORT_P1
#define BOARD_LED_TWO_PIN                       GPIO_PIN1
#define BOARD_LED_TWO                           BOARD_LED_TWO_PORT, BOARD_LED_TWO_PIN

#define BOARD_LED_THREE_PORT                    GPIO_PORT_P1
#define BOARD_LED_THREE_PIN                     GPIO_PIN1
#define BOARD_LED_THREE                         BOARD_LED_THREE_PORT, BOARD_LED_THREE_PIN

#define BOARD_LED_FOUR_PORT                     GPIO_PORT_P1
#define BOARD_LED_FOUR_PIN                      GPIO_PIN1
#define BOARD_LED_FOUR                          BOARD_LED_FOUR_PORT, BOARD_LED_FOUR_PIN


// ADC I/O
#define FORCE_SENSE_PLATE_PORT                  GPIO_PORT_P1
#define FORCE_SENSE_PLATE_PIN                   GPIO_PIN1
#define FORCE_SENSE_PLATE                       FORCE_SENSE_PLATE_PORT, FORCE_SENSE_PLATE_PIN

// PWM
#define PWM_VSLOT_TOP_PORT                      GPIO_PORT_P1
#define PWM_VSLOT_TOP_PIN                       GPIO_PIN1
#define PWM_VSLOT_TOP                           PWM_VSLOT_TOP_PORT, PWM_VSLOT_TOP_PIN

#define PWM_VSLOT_BOT_PORT                      GPIO_PORT_P1
#define PWM_VSLOT_BOT_PIN                       GPIO_PIN1
#define PWM_VSLOT_BOT                           PWM_VSLOT_BOT_PORT, PWM_VSLOT_BOT_PIN

#define PWM_EXP_PORT                            GPIO_PORT_P1
#define PWM_EXP_PIN                             GPIO_PIN1
#define PWM_EXP                                 PWM_EXP_PORT, PWM_EXP_PIN

/* Control Defines */
//VSlot stepper direction controls
#define STEPPER_DIR_HAB                         1
#define STEPPER_DIR_SPC                         0
//Experiment tray stepper direction controls
#define STEPPER_DIR_CLOSE                       1
#define STEPPER_DIR_OPEN                        0

#define TABLE_CAP_SENSE_ONE                     0b01
#define TABLE_CAP_SENSE_TWO                     0b10
#define TABLE_CAP_SENSE_BOTH                    0b11


#endif /* DEFINES_H_ */
