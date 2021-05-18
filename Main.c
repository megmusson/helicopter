/*
 * ENCE361 Helicopter Project 2021
 * Luke Forrester, Cameron McDrury, Meg Musson
 *
 * Code for Texas Instruments TIVA TM4C123G Launchpad with added Booster board.
 * Controls a small RC helicopter on a stand, capable of altitude and yaw changes.
 *
 *
 * Heli Rig plugs:
 * Altitude = PE4
 * Yaw = PB0, PB1 (A, B)
 * PWM Main - PC5
 * PWM Tail - PF1
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "circBufT.h"
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "buttons4.h"



// Module includes
#include "Modules/Altitude.h"
#include "Modules/Yaw.h"
#include "Modules/PWMcontrol.h"
#include "Modules/Display.h"

/**********************************************************
 * Constants
 **********************************************************/
// Systick configuration
#define SYSTICK_RATE_HZ    200
#define SAMPLE_RATE_HZ 60
#define OLED_REFRESH_DIVIDER    60


// Yaw and Altitude change values
#define YAW_STEP_POS 15
#define YAW_STEP_NEG -15
#define ALT_STEP_POS 10
#define ALT_STEP_NEG -10
#define PWM_FREQ 250


#define STEADY_DESCEND_PWM 30
#define SEARCH_TAIL_DC 40
#define SEARCH_MAIN_DC 0

/*******************************************
 *      Local prototypes
 *******************************************/
void SysTickIntHandler(void);
void initClocks(void);
void initSysTick(void);



//static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;    // Counter for the interrupts
extern uint32_t altitudeTarget;
extern int yawTarget;
extern int32_t totalAltDC;
extern int32_t totalYawDC;
extern int yaw;



bool flying = 0;

bool yawReferenceSet = 0;

bool landing = 0;
bool landed = 0;


//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************
void SysTickIntHandler(void)
{
    //
    // Initiate a conversion and update the buttons
    //
    updateButtons();
    ADCProcessorTrigger(ADC0_BASE, 3);
    readAltitude();

    if (flying) {
        calcYawPWM(SAMPLE_RATE_HZ);
        calcAltPWM(calcAltPercent(),SAMPLE_RATE_HZ);

        setPWMtail (PWM_FREQ,totalYawDC);
        setPWMmain (PWM_FREQ,totalAltDC);
    }
    g_ulSampCnt++;

}

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void initClock(void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
    SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}


void
locateYawStart(void){
    //Maintains a slow steady tail rotor PWM until the signal is recieved, then sets that yaw position to zero.
    while((GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4))){
        setPWMtail (PWM_FREQ, SEARCH_TAIL_DC);
        setPWMmain (PWM_FREQ, SEARCH_MAIN_DC);
    }
/*
    setPWMtail (PWM_FREQ, 0);
    setPWMmain (PWM_FREQ, 0);
    */
    yaw = 0;
    yawTarget = 0;

    flying = 1;
    yawReferenceSet = 1;
}
void
initialiseUSB_UART (void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    //


}


void
initSwitch(void) {
    // Initialise the switch for setting landing modes
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);
}

void
checkAndChangeTargets(void){
            if (checkButton(UP) == PUSHED)
                {
                //Increase Target Altitude by 10%
                changeTargetAltitude(ALT_STEP_POS);
                }
            if (checkButton(DOWN) == PUSHED)
              {
                //Decrease Target Altitude by 10%
                changeTargetAltitude(ALT_STEP_NEG);
              }

            if (checkButton(LEFT) == PUSHED)
            {
                // Decrease yaw by 15 degrees
                changeTargetYaw(YAW_STEP_NEG);;
            }
            if (checkButton(RIGHT) == PUSHED)
            {
                // Increase yaw by 15 degrees
                changeTargetYaw(YAW_STEP_POS);
            }
}
int main(void)
{
    int yawErrorTolerance = 3;
    int altErrorTolerance = 3;
    SysCtlPeripheralReset(UP_BUT_PERIPH);        // UP button GPIO
    SysCtlPeripheralReset(DOWN_BUT_PERIPH);      // DOWN button GPIO
    SysCtlPeripheralReset(LEFT_BUT_PERIPH);      // LEFT button GPIO
    SysCtlPeripheralReset(RIGHT_BUT_PERIPH);     // RIGHT button GPIO
    resetPWMs();


    initButtons();
    initDisplay();

    initClock();
    initADC();
    initYawGPIO();
    initSwitch();

    initialisePWMs ();
    initialiseUSB_UART ();

    enablePWMs();

    // Enable interrupts to the processor.
    IntMasterEnable();
    SysCtlDelay(SysCtlClockGet() / 6);


    setMinMaxAlt();
    landed = 1;


    while (1)
    {
        // Main loop
        updateButtons();

        //Check if switch is on or off

        if ((!flying)&& (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7))) {
            if (!yawReferenceSet){
                locateYawStart();
            }

        if ((landed)&& (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7))) {
            // Start FLYING
            altitudeTarget = 0;
            locateYawStart();

            flying = 1;
            landed = 0;
        }


        if (flying) {
        checkAndChangeTargets();
        }


        if ((flying) && (!(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7)))) {
            // Start LANDING
            flying = 0;
            landing = 1;
        }

        if (landing) {
            yawTarget = 0;


            while((yaw > abs(yawErrorTolerance))) {
                // Wait for the heli to point forwards before landing.
            }

            //altitudeTarget = 0;
            setPWMmain(PWM_FREQ, STEADY_DESCEND_PWM);

            while ((calcAltAverage() > abs(altErrorTolerance))) {

            }

            flying = 0;
            setPWMtail (PWM_FREQ, 0);
            setPWMmain (PWM_FREQ, 0);
            landed = 1;
            //The end.
        }
/*
            while((yaw > abs(yawErrorTolerance))&&(calcAltAverage() > abs(altErrorTolerance))) {
                // Wait for the heli to point forwards before landing.
            }
            altitudeTarget = 0;
            setPWMtail (PWM_FREQ,0);
            setPWMmain (PWM_FREQ,0);

            */






        displayStatus(flying);
        SysCtlDelay (SysCtlClockGet() / DISPLAY_HZ);  // Update display

    }
}

