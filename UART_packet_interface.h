/*
 * UART_packet_interface.h
 *
 *  Created on: Mar 10, 2018
 *      Author: Aetas
 */

#ifndef UART_PACKET_INTERFACE_H_
#define UART_PACKET_INTERFACE_H_

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

#include "defines.h"

struct RingBuffer {
#if BUFFER_SIZE == 256
    uint8_t end;
    uint8_t start;
#else   // BUFFER_SIZE == 512
    //512 is the max buffer size allocatable so it is the default buffer size option.
    //2^16 covers way more than is required for a 512-char buffer size (2^9 is req'd)
    uint16_t end;
    uint16_t start;
#endif
    unsigned char data[BUFFER_SIZE];
} UART_RingBuffer;

struct commandPacket {
    uint8_t type;
    uint8_t command;
    uint8_t length;
    uint8_t data[sizeof(int)];
};

void parse_UART_RingBuffer(struct commandPacket *packet);

#endif /* UART_PACKET_INTERFACE_H_ */
