/*
 * PWMcontrol.h
 *
 *  Created on: 7/05/2021
 *      Author: Luke Forrester
 */

#ifndef PWMCONTROL_H_
#define PWMCONTROL_H_

/*********************************************************
 * initialisePWMS
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 * M1PWM5 is used for the tail rotor
 *********************************************************/
void
resetPWMs(void);

void
initialisePWMs (void);

void
setPWMmain (uint32_t ui32Freq_1, uint32_t ui32Duty);

void
setPWMtail (uint32_t ui32Freq_1, uint32_t ui32Duty);

uint32_t
calcAltPWM(uint32_t sum );

uint32_t
calcYawPWM(void);

void
changeTargetYaw(int16_t degreesChange);

void
changeTargetAltitude(int16_t percentChange);

uint32_t
getTargetYawDeg(void);

uint32_t
getTargetAltPercent(void);

void
enablePWMs(void);

#endif /* PWMCONTROL_H_ */
