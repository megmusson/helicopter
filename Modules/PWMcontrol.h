/*
 * PWMcontrol.h
 *
 *  Created on: 7/05/2021
 *      Author: Luke Forrester
 */

#ifndef PWMCONTROL_H_
#define PWMCONTROL_H_


#define YAW_P_GAIN 5
#define YAW_I_GAIN 5
#define ALT_P_GAIN 5
#define ALT_I_GAIN 5



/*********************************************************
 * initialisePWMS
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 * M1PWM5 is used for the tail rotor
 *********************************************************/
void
initialisePWMs (void)


#endif /* PWMCONTROL_H_ */
