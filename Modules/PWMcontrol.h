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

void
calcAltPWM(int32_t altitude, uint32_t testFrequency);

void
calcYawPWM(uint32_t testFrequency);

void
changeTargetYaw(int16_t degreesChange);

void
changeTargetAltitude(int16_t percentChange);

void
enablePWMs(void);

#endif /* PWMCONTROL_H_ */
