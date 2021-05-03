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

#define PHASE_A GPIO_INT_PIN_0
#define PHASE_B GPIO_INT_PIN_1

int yaw;


void
GPIOIntHandler(void);

void
initYawGPIO (void);





#endif /* YAW_H_ */
