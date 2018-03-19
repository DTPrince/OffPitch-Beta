/*
 * UART_packet_interface.c
 *
 *  Created on: Mar 10, 2018
 *      Author: Derek Prince
 *
 */
#include "UART_packet_interface.h"
#include "defines.h"

/*
 * parse_UART_RingBuffer
 *  input:  struct commandPacket *packet
 *  return: none
 *
 *  Description:
 *
 */
void parse_UART_RingBuffer(commandPacket *packet){
    // Grab type
    packet->type = UART_RingBuffer.data[UART_RingBuffer.start] & UART_TYPE;
    // Grab command
    packet->command = UART_RingBuffer.data[UART_RingBuffer.start] & UART_COMMAND;

    // Check if the next element is the end of the ring.
    // If so, set start "pointer" to initial position
    if (UART_RingBuffer.start + 1 == BUFFER_SIZE){
#if DEBUG_LEVEL > 2 //only verbose
        //Send debug flag over UART
        MAP_UART_transmitData(EUSCI_A0_BASE, (char)UART_DEBUG_FLAG);

        //Send string message
        // REQUIRED: terminate with \n newline. This is the 'end of message' key to look for
        printf(EUSCI_A0_BASE, "Ring buffer start pointer reset before fetching packet length\n");
#endif
        UART_RingBuffer.start = 0;
        // The length is grabbed in the conditional to allow incrementing the start pointer normally in the else statement.
        // This has the effect of making the code that runs under it have no knowledge of anything unusual running before it.
        packet->length = UART_RingBuffer.data[UART_RingBuffer.start];
    } else {
        //Increment start pointer by Type/Command byte size
        UART_RingBuffer.start += UART_COMMAND_PACKET_TYPECMD_SIZE;
        packet->length = UART_RingBuffer.data[UART_RingBuffer.start];
    }
    // Increment pointer by data length size (in bytes)
    UART_RingBuffer.start += UART_COMMAND_PACKET_LENGTH_SIZE;

    // Now fetch the data byte-by-byte according to the length of the packet.
    uint8_t i;
    // Packet length is measured in bytes to allow for this iterating
    for (i = 0; i < packet->length; i++) {
        // Check every round if the start pointer is at the end of the buffer
        if(UART_RingBuffer.start == BUFFER_SIZE){
//My kingdom for code folding...
#if DEBUG_LEVEL > 2 //only verbose
            //Send debug flag over UART
            MAP_UART_transmitData(EUSCI_A0_BASE, (char)UART_DEBUG_FLAG);

            //Send string message
            // REQUIRED: terminate with \n newline. This is the 'end of message' key to look for
            printf(EUSCI_A0_BASE, "Ring buffer start pointer reset before fetching packet data. i:%d\n", (char)i);
#endif
            // reset pointer location if it is
            UART_RingBuffer.start = 0;
            packet->data[i] = UART_RingBuffer.data[UART_RingBuffer.start + 1 + i];

        } else {
            //otherwise proceed normally and increment pointer/fetch data
            UART_RingBuffer.start++;
            packet->data[i] = UART_RingBuffer.data[UART_RingBuffer.start + 1 + i];
        }
    }

    // One final overrun check for the last increment
    // This increment is to prep the start pointer for the next parse_UART_RingBuffer call as it assumes the pointer is correctly aligned
    if(UART_RingBuffer.start == BUFFER_SIZE){
        UART_RingBuffer.start = 0;
    } else {
        UART_RingBuffer.start++;
    }
}
