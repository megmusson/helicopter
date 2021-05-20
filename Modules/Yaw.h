/*
 * Yaw.h
 *
 *  Created on: 23/04/2021
 *      Author: cmcdr
 */

#ifndef YAW_H_
#define YAW_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

uint8_t lookupIndex;


void
GPIOIntHandler(void);

void
initYawGPIO (void);

int32_t
calcDegrees(void);




#endif /* YAW_H_ */
