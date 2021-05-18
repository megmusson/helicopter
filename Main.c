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

//static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;    // Counter for the interrupts
extern uint32_t altitudeTarget;
extern int yawTarget;
extern int32_t totalAltDC;
extern int32_t totalYawDC;
extern int yaw;

bool flying = 0;
bool yawReferenceSet = 0;


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



//*****************************************************************************
//
// Function to display the mean ADC value (10-bit value, note), yaw position and sample count.
//
//*****************************************************************************


static uint32_t ticks = 0;





void
DisplayLowerSwitch(void){
        char string[16];

        usnprintf(string, sizeof(string), "Turn off switch");
        OLEDStringDraw("        ", 0, 0);
        OLEDStringDraw(string, 0, 1);
        OLEDStringDraw(string, 0, 2);
        OLEDStringDraw("        ", 0, 3);
        UARTSend(string);

}


void
locateYawStart(void){
    //Maintains a slow steady tail rotor PWM until the signal is recieved, then sets that yaw position to zero.
    int searchTailDC = 40;
    int searchMainDC = 0;

    while(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)){
        setPWMtail (PWM_FREQ,searchTailDC);
        setPWMmain (PWM_FREQ,searchMainDC);
    }
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
    //Initialise the soft reset button
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
    return (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7));
}

int main(void)
{
    bool usingEmulator = 0;
    int yawErrorTolerance = 2;
    int altErrorTolerance = 2;
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
    //initSwitch();

    initialisePWMs ();
    initialiseUSB_UART ();

    enablePWMs();

    // Enable interrupts to the processor.
    IntMasterEnable();
    SysCtlDelay(SysCtlClockGet() / 6);


    setMinMaxAlt();

    while (switchIsUp()) {
        DisplayLowerSwitch();

    }


    while (1)
    {
        // Main loop
        updateButtons();

        //Check if switch is on or off
        if ((!flying)&& (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7))) {
            if (!yawReferenceSet){
                locateYawStart();
            }
            flying = 1;
        }


        if (flying){
        checkAndChangeTargets();
        }
        if ((flying) && (!switchIsUp())) { // If switch requests landing
            yawTarget = 0;
            altitudeTarget = 0;
            while((yaw > abs(yawErrorTolerance))&&(calcAltAverage() > abs(altErrorTolerance))) { //While not at zero points, keep goin
            }
            setPWMtail (PWM_FREQ,0);
            setPWMmain (PWM_FREQ,0);
            flying = 0;
        }

        if ((!usingEmulator)&& (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6))){
            SysCtlReset();
        }

        displayStatus(flying);
        SysCtlDelay (SysCtlClockGet() / DISPLAY_HZ);  // Update display
        ticks += 1;
    }
}

