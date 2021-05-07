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

// Module includes
#include "Modules/Altitude.h"
#include "Modules/Yaw.h"

/**********************************************************
 * Constants
 **********************************************************/

// Systick configuration
#define SYSTICK_RATE_HZ    100

// PWM configuration
#define PWM_START_RATE_HZ  250
#define PWM_RATE_STEP_HZ   50
#define PWM_RATE_MIN_HZ    50
#define PWM_RATE_MAX_HZ    400
#define PWM_START_DUTY     95
#define PWM_DUTY_STEP   5
#define PWM_DUTY_MAX    95
#define PWM_DUTY_MIN    5
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_4
#define PWM_DIVIDER        4

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

#define SAMPLE_RATE_HZ 60
#define OLED_REFRESH_DIVIDER    60

/*******************************************
 *      Local prototypes
 *******************************************/

void SysTickIntHandler(void);
void initClocks(void);
void initSysTick(void);
void initialisePWM(void);
void setPWM(uint32_t u32Freq, uint32_t u32Duty);

uint32_t ui32Freq = PWM_START_RATE_HZ;
uint32_t ui32Duty = PWM_START_DUTY;

//static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;    // Counter for the interrupts
/***********************************************************
 * ISR for the SysTick interrupt (used for button debouncing).
 ***********************************************************/
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

void initDisplay(void)
{
    // intialise the Orbit OLED display
    OLEDInitialise();
}

//*****************************************************************************
//
// Function to display the mean ADC value (10-bit value, note) and sample count.
//
//*****************************************************************************
void displayAltPercent(int32_t sum, uint32_t count, uint16_t voltageLanded,
                       uint16_t voltageMaxHeight)
{
    char string[17];  // 16 characters across the display

    OLEDStringDraw("Heli Control", 0, 0);
    // This works
    uint32_t percent = 100
            - 100 * ((2 * sum + BUF_SIZE) / 2 / BUF_SIZE)( - voltageMaxHeight)
                    / (voltageLanded - voltageMaxHeight);

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf(string, sizeof(string), "Height %% = %4d", percent);
    // Update line on display.
    OLEDStringDraw(string, 0, 1);

    usnprintf(string, sizeof(string), "Sample # %5d", count);
    OLEDStringDraw(string, 0, 3);

    usnprintf(string, sizeof(string), "Yaw = %4d", calcDegrees()); // calcDegrees()
    OLEDStringDraw(string, 0, 2);
}

void displayMeanADC(int32_t sum, uint32_t count)
{
    char string[17];  // 16 characters across the display

    OLEDStringDraw("Heli Control", 0, 0);
    // This works
    uint32_t mean = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf(string, sizeof(string), "Mean ADC = %4d", mean);
    // Update line on display.
    OLEDStringDraw(string, 0, 1);
    OLEDStringDraw("              ", 0, 2);

    usnprintf(string, sizeof(string), "Sample # %5d", count);
    OLEDStringDraw(string, 0, 3);
}

void displayOff(void)
{
    // Blank the display
    OLEDStringDraw("                ", 0, 0);
    OLEDStringDraw("                ", 0, 1);
    OLEDStringDraw("                ", 0, 2);
    OLEDStringDraw("                ", 0, 3);
}

int main(void)
{
    uint16_t i;
    int32_t sum;

    SysCtlPeripheralReset(UP_BUT_PERIPH);        // UP button GPIO
    SysCtlPeripheralReset(DOWN_BUT_PERIPH);      // DOWN button GPIO
    SysCtlPeripheralReset(LEFT_BUT_PERIPH);     // LEFT button GPIO
    initButtons();
    initClock();
    initADC();
    initDisplay();
    initYawGPIO();

    initCircBuf(&g_inBuffer, BUF_SIZE);

    // Enable interrupts to the processor.
    IntMasterEnable();
    SysCtlDelay(SysCtlClockGet() / 6);

    // Read the landed ADC.
    uint16_t voltageLanded = readCircBuf(&g_inBuffer);
    uint16_t voltageMaxHeight = voltageLanded - 1000;

    // Flag to keep track of display modes
    uint8_t flag = 0; // 0 for percent, 1 for adc or 2 for off

    while (1)
    {
        // Main loop
        updateButtons();
        // Background task: calculate the (approximate) mean of the values in the
        // circular buffer and display it, together with the sample number.
        sum = 0;
        for (i = 0; i < BUF_SIZE; i++)
        {
            sum = sum + readCircBuf(&g_inBuffer);
        }

        if (checkButton(UP) == PUSHED)
        {
            // Change the display mode flag
            flag = flag + 1;
            if (flag == 3)
            {
                // Loop back around, only 3 modes.
                flag = 0;
            }
        }
        if (checkButton(LEFT) == PUSHED)
        {
            // Reset landed voltage
            voltageLanded = readCircBuf(&g_inBuffer);
        }

        // Alternate between displays
        if (flag == 0)
        {
            displayAltPercent(sum, g_ulSampCnt, voltageLanded,
                              voltageMaxHeight);

        }
        else if (flag == 1)
        {
            displayMeanADC(sum, g_ulSampCnt);

        }
        else
        {
            displayOff();
        }

        SysCtlDelay (SysCtlClockGet() / 150);  // Update display
    }
}

