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

circBuf_t g_inBuffer;

void
ADCIntHandler(void);

void
initADC (void);

int32_t
calcAltPercent(void);

int32_t
calcAltAverage(void);

void
setMinMaxAlt(void);

void
readAltitude(void);

#endif /* ALTITUDE_H_ */
