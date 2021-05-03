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

circBuf_t g_inBuffer;


void
ADCIntHandler(void);

void
initADC (void);




#endif /* ALTITUDE_H_ */
