


#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "../inc/hw_memmap.h"
#include "../inc/hw_types.h"
#include "../driverlib/pin_map.h" //Needed for pin configure
#include "../driverlib/debug.h"
#include "../driverlib/gpio.h"
#include "../driverlib/pwm.h"
#include "../utils/ustdlib.h"
#include "../driverlib/sysctl.h"
#include "Altitude.h"
#include "Yaw.h"

#include "PWMcontrol.h"
// PWM  Initialisation configuration
#define PWM_START_DUTY     0
#define PWM_FIXED_HZ  250
#define PWM_DUTY_MAX    95
#define PWM_DUTY_MIN    5
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_4
#define PWM_DIVIDER        1
//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

//  ---Tail Rotor PWM:M1PWM5 (gen 2) PC5,
#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1


#define YAW_P_GAIN 5
#define YAW_I_GAIN 5
#define ALT_P_GAIN 5
#define ALT_I_GAIN 5

#define YAW_TARGET_CHANGE_DEG 15
#define YAW_EDGES 448
#define ROTATION_DEG 360
#define TIME_CONSTANT 0.005
int32_t yawIntControl;
int32_t altIntControl;

uint32_t altitudeTarget =50; // As percentage of maximum height to minimum height
int32_t yawTarget; //Degrees

//Variables from other files


/*********************************************************
 * initialisePWMS
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 * M1PWM5 is used for the tail rotor
 *********************************************************/
void
resetPWMs(void){
        SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);
        SysCtlPeripheralReset (PWM_TAIL_PERIPH_PWM);
        SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
        SysCtlPeripheralReset (PWM_TAIL_PERIPH_GPIO);
}

void
initialisePWMs (void)
{

    //SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);
   // SysCtlPeripheralReset (PWM_TAIL_PERIPH_PWM);


    //SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
   // SysCtlPeripheralReset (PWM_TAIL_PERIPH_GPIO); // Used for PWM output TAIL

    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    setPWMmain (PWM_FIXED_HZ, PWM_START_DUTY);
    setPWMtail (PWM_FIXED_HZ, PWM_START_DUTY );

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);
    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);


    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}



/********************************************************
 * Functions to set the freq, duty cycle of M0PWM7 and M1PWM5. Frequency will be constant
 ********************************************************/
void
setPWMmain (uint32_t ui32Freq_1, uint32_t ui32Duty)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / ui32Freq_1;

    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
        ui32Period * ui32Duty / 100);
}
void
setPWMtail (uint32_t ui32Freq_1, uint32_t ui32Duty)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / ui32Freq_1;

    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
        ui32Period * ui32Duty / 100);
}


/********************************************************
 * Functions to calculate the pwm output using a PI controller
 ********************************************************/
uint32_t
calcAltPWM(uint32_t altitude){
    //Uses PI control and returns a PWM duty cycle value between 5 and 90 to power the Main motor to control
    //the altitude of the helicopter.
    uint32_t altError;
    uint32_t altPropControl;

    altError = altitude-altitudeTarget;
    altPropControl = altError*YAW_P_GAIN;
    altIntControl += altError*TIME_CONSTANT*YAW_I_GAIN;

    return (altPropControl+altIntControl);
}

uint32_t
calcYawPWM(void){
    //Using PI control, returns a PWM duty cycle value between 5 and 90 to power the Main motor to control
    //the altitude of the helicopter.
    uint32_t yawError;
    uint32_t yawPropControl = 0;

    yawError = giveMeTheYaw()-(yawTarget*YAW_EDGES/ROTATION_DEG);
    yawPropControl = yawError*YAW_P_GAIN;
    yawIntControl += yawError*TIME_CONSTANT*YAW_I_GAIN;

    return (yawPropControl+yawIntControl);
}


void
changeTargetYaw(int16_t degreesChange)
{ //Changes the global variable yawTarget by positive or negative value degreesChange.

    yawTarget += degreesChange;
    if (yawTarget >= 181) {
            yawTarget -= 360;
        } else if (yawTarget <= -179) {
            yawTarget += 360;
        }
}
void
changeTargetAltitude(int16_t percentChange)
{ //Changes the global variable altitudeTarget by positive or negative percentChange, keeping target between 0 and 90%.
    if ((altitudeTarget >= 90)&&(percentChange <= 0)){
        altitudeTarget += percentChange;}
    else if ((altitudeTarget <=0)&&(percentChange >=0)) {
        altitudeTarget += percentChange;
    }
}

uint32_t
getTargetYawDeg(void)
{
    return yawTarget;
}

uint32_t
getTargetAltPercent(void)
{
    return altitudeTarget;
}

void
enablePWMs(void)
{
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}

