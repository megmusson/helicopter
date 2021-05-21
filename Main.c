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
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "driverlib/uart.h"

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
#define DISPLAY_HZ 150
#define SLOW_TICKRATE_HZ 4

// Yaw and Altitude change values
#define YAW_STEP_POS 15
#define YAW_STEP_NEG -15
#define ALT_STEP_POS 10
#define ALT_STEP_NEG -10
#define PWM_FREQ 250
/*******************************************
 *      Local prototypes
 *******************************************/
void SysTickIntHandler(void);
void initClocks(void);
void initSysTick(void);

//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

// Values for when the helicopter is locating the yaw reference point.
#define SEARCH_TAIL_DC 40
#define SEARCH_MAIN_DC 0

static uint32_t g_ulSampCnt;    // Counter for the interrupts
extern uint32_t altitudeTarget;
extern int yawTarget;
extern int32_t totalAltDC;
extern int32_t totalYawDC;
extern int yaw;


bool flying = 0; // Start the helicopter not flying
bool yawReferenceSet = 0; // Whether the yaw reference has been set or not
// Error tolerances for determining whether the helicopter has reached the target balue
#define YAW_ERR_TOL  2; //Degrees
#define  ALT_ERR_TOL 2; //% height

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
displayLowerSwitch(void){
    // Display a message telling the user to turn the landing/flying switch back.
        char string[16];

        usnprintf(string, sizeof(string), "Turn off switch");
        OLEDStringDraw("        ", 0, 0);
        OLEDStringDraw(string, 0, 1);
        OLEDStringDraw(string, 0, 2);
        OLEDStringDraw("        ", 0, 3);
        UARTSend(string);

}

bool
yawReferenceNotRead(void) {
    //Reads and returns the status of the yaw reference sensor.
    //Active low means it returns 1 while the helicopter is NOT detecting the
    //reference signal.
    return (GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4));
}

void
locateYawStart(void){
    //Maintains a slow steady tail rotor PWM until the signal is received that we are at 0, then sets that yaw position to zero.

    while(yawReferenceNotRead()){
        setPWMtail (PWM_FREQ, SEARCH_TAIL_DC);
        setPWMmain (PWM_FREQ, SEARCH_MAIN_DC);
    }
    setMinMaxAlt();
    yaw = 0;
    yawTarget = 0;
    flying = 1;
    yawReferenceSet = 1;

}


void
initSwitch(void) {
    // Initialise the switch for setting landing modes
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);
    //  Initialise the soft reset button
        GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6); //Initialise soft reset button
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

bool
switchIsUp(void){
    //Reads and returns the status of the right switch
    return (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7));
}

bool
softResetPushed(void){
    // Reads and returns the status of the soft reset button.
    return (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6));
}


int main(void)
{
    /*
     *  - Initialises peripherals and enables PWM signals
     *  - Sets the minimum and maximum altitude voltage of the helicopter
     *  - Refuses to enter the main while-loop until the flying switch is DOWN
     *
     *  In the main while-loop:
     *  - Polls LEFT,RIGHT,UP,DOWN buttons to change yaw and altitude target, and moves towards them via sysTick interrupts
     *  - Polls flying switch to see if the user has requested the helicopter to land, lands and set flying status to 0 if so.
     *  - Polls soft reset switch and resets (softly) if button is pressed.
     */
    bool usingEmulator = 1; // Set to 1 to disable soft reset, the emulator does not like the soft reset GPIO pin.
    int yawErrorTolerance = YAW_ERR_TOL;
    int altErrorTolerance = ALT_ERR_TOL;


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

    setMinMaxAlt(); // Sets the current altitude to the zero position

    while (switchIsUp()) {
        // Refuses to start the program until the switch is in the down position
        displayLowerSwitch();
    }

    while (1)
    {
        updateButtons();

        // If not flying and the switch is pulled up, start flying.
        if ((!flying)&& (switchIsUp())) {
            if (!yawReferenceSet){
                locateYawStart();
            }
            flying = 1;
        }

        // Check for user yaw and altitude input
        if (flying){
        checkAndChangeTargets();
        }

        // If switch brought to down position while flying, land and set yaw to zero
        if ((flying) && (!switchIsUp())) {
            yawTarget = 0;
            altitudeTarget = 0;

            while((abs(yaw) > yawErrorTolerance) && (abs(calcAltAverage()) > altErrorTolerance))
            { // Empty while-loop. Keep searching for the target until within the tolerances
            }
            //Turn off rotors
            setPWMtail (PWM_FREQ,0);
            setPWMmain (PWM_FREQ,0);
            flying = 0;
        }


        //Check whether a soft reset has been called.
        if (!usingEmulator && softResetPushed()){ //softResetPushed()     Change this line when using emulator
            SysCtlReset();
        }

        //Update display and add tick for UART transmission
        displayStatus(flying);
        SysCtlDelay (SysCtlClockGet() / DISPLAY_HZ);  // Update display
        ticks += 1;
    }
}

