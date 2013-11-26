/*
 * utils.h
 *
 *  Created on: 2013-10-21
 *      Author: Hao Yan
 */

#ifndef UTILS_H_
#define UTILS_H_

#endif /* UTILS_H_ */


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

void UART4Send(const uint8_t *pui8Buffer, uint32_t ui32Count);
void UART4Receive(uint8_t *pui8Buffer, uint32_t ui32Count);
