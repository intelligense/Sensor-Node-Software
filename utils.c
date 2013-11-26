/*
 * utils.c
 *
 *  Created on: 2013-10-21
 *      Author: lenovo
 */

#include "utils.h"


//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UART0Send(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}

void
UART4Send(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART4_BASE, *pui8Buffer++);
    }
}

// Receive data from the UART
void
UART4Receive(uint8_t *pui8Buffer, uint32_t ui32Count)
{
	int i = 0;
    //
    // Loop while there are more data expected.
    //
    while(ui32Count)
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
    	if (ROM_UARTCharsAvail(UART4_BASE)) {
    		pui8Buffer[i++] = ROM_UARTCharGetNonBlocking(UART4_BASE);
        	ui32Count--;
    	}
    }
}
