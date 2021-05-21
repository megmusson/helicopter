/*
 * Altitude.h
 *
 *  Created on: 23/04/2021
 *      Author: cmcdr
 */

#ifndef ALTITUDE_H_
#define ALTITUDE_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "../circBufT.h"

// name of buffer
circBuf_t g_inBuffer;

// *******************************************************
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
void
ADCIntHandler(void);

// *******************************************************
// Initialise the variables associated with the ADC
// defined by the constants in Altitude.c.
void
initADC (void);

// *******************************************************
// Calculates the altitude of the helicopter as a percentage relative to its maximum and minimum height
int32_t
calcAltPercent(void);

// *******************************************************
//Calculates and returns the rounded average altitude of the helicopter
int32_t
calcAltAverage(void);

// *******************************************************
//Sets the maximum and minimum voltage value for altitude 
void
setMinMaxAlt(void);

// *******************************************************
//Sums all the values in buffer to determine average altitude over BUF_SIZE samples
void
readAltitude(void);

#endif /* ALTITUDE_H_ */
