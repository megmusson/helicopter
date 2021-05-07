


#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "utils/ustdlib.h"

#include "Modules/Altitude.h"


int32_t yawIntControl;
int32_t altIntControl

uint32_t altitudeTarget;
uint32_t yawTarget;


/*********************************************************
 * initialisePWMS
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 * M1PWM5 is used for the tail rotor
 *********************************************************/
void
initialisePWMs (void)
{
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
    setPWMmain (PWM_START_RATE_HZ, PWM_START_DUTY);
    setPWMtail (PWM_FIXED_RATE_HZ, PWM_FIXED_DUTY );

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
calcAltPWM(uint32_t sum, ){
    //Uses PI control and eturns a PWM duty cycle value between 5 and 90 to power the Main motor to control
    //the altitude of the helicopter.
    uint32_t altError;
    uint32_t altPropControl = 0;
    uint32_t altitude = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE);

    altitude =
    altError = yaw-altTarget;
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

    yawError = yaw-yawTarget;
    yawPropControl = yawError*YAW_P_GAIN;
    yawIntControl += yawError*TIME_CONSTANT*YAW_I_GAIN;

    return (yawPropControl+yawIntControl);
}


