// Main helicopter control file

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

#define BUF_SIZE 10
#define SAMPLE_RATE_HZ 10

#define OLED_REFRESH_DIVIDER    60



/*******************************************
 *      Local prototypes
 *******************************************/
void SysTickIntHandler (void);
void initClocks (void);
void initSysTick (void);
void initialisePWM (void);
void setPWM (uint32_t u32Freq, uint32_t u32Duty);


//YAW MEASUREMENT DEFINITIONS AND GLOBAL VARIABLES
#define PHASE_A GPIO_INT_PIN_0
#define PHASE_B GPIO_INT_PIN_1
uint32_t ui32Freq = PWM_START_RATE_HZ;
uint32_t ui32Duty = PWM_START_DUTY;
uint32_t yaw = 0;
int8_t yawChangeTable[16] = { 0, -1, 0, 1, 1, -1, 0, 0,
                              -1, 0, 0,1, 0, 1, -1, 0};






static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt;    // Counter for the interrupts
/***********************************************************
 * ISR for the SysTick interrupt (used for button debouncing).
 ***********************************************************/
//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Initiate a conversion and update the buttons
    //
    updateButtons();
    ADCProcessorTrigger(ADC0_BASE, 3);
    g_ulSampCnt++;

}

//*****************************************************************************
//
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//
//*****************************************************************************
void
ADCIntHandler(void)
{
    uint32_t ulValue;

    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer, ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);

}


//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
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
initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE | // CHANGE HERE FOR LAB++++++++++++++++++++++++++++++++++++++++
                             ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}

void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}


uint8_t y_in_prev = 0; //global variables to save previous bit states.

void
GPIOIntHandler(void)
{
    //get values from both sensors as well as their previous values
    uint8_t Value = 0;
    uint8_t y_in_read;

    y_in_read = GPIOPinRead(GPIO_PORTB_BASE, PHASE_A | PHASE_B);


    Value = y_in_prev<<2 | y_in_read;


    //use table to determine whether add or subtract one to yaw
    yaw = yaw + yawChangeTable[Value];


    y_in_prev = y_in_read;
    GPIOIntClear(GPIO_PORTB_BASE,PHASE_A | PHASE_B);


}

void
initYawGPIO (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, PHASE_A | PHASE_B);

    //GPIOPadConfigSet (GPIO_PORTB_BASE, PHASE_A | PHASE_B,
               //       GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);


    //GPIOIntRegisterPin(GPIO_PORTB_BASE, PHASE_A, GPIOIntHandler); // Sets the interrupt action upon reading
    // GPIOIntRegisterPin(GPIO_PORTB_BASE, PHASE_B, GPIOIntHandler);

    GPIOIntRegister(GPIO_PORTB_BASE, GPIOIntHandler);

    GPIOIntTypeSet(GPIO_PORTB_BASE,PHASE_A | PHASE_B, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTB_BASE, PHASE_A | PHASE_B);//enable the interrupt on pin 0 and pin 1 on port B

}

void
displayUpdate (char *str1, char *str2, uint32_t num, uint8_t charLine)
{
    char text_buffer[17];           //Display fits 16 characters wide.

    // "Undraw" the previous contents of the line to be updated.
    OLEDStringDraw ("                ", 0, charLine);
    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf(text_buffer, sizeof(text_buffer), "%s %s %3d", str1, str2, num);
    // Update line on display.
    OLEDStringDraw (text_buffer, 0, charLine);
}

//*****************************************************************************
//
// Function to display the mean ADC value (10-bit value, note) and sample count.
//
//*****************************************************************************
void
displayAltPercent(int32_t sum, uint32_t count, uint16_t voltageLanded, uint16_t voltageMaxHeight)
{
    char string[17];  // 16 characters across the display



    OLEDStringDraw ("Heli Control", 0, 0);
    // This works
    uint32_t percent =  100 - 100*(((2 * sum + BUF_SIZE) / 2 / BUF_SIZE) - voltageMaxHeight)/(voltageLanded-voltageMaxHeight);

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf (string, sizeof(string), "Height %% = %4d", percent);
    // Update line on display.
    OLEDStringDraw (string, 0, 1);

    usnprintf (string, sizeof(string), "Sample # %5d", count);
    OLEDStringDraw (string, 0, 3);


    usnprintf (string, sizeof(string), "Yaw = %4d", yaw);
    OLEDStringDraw(string, 0, 2);
}

void
displayMeanADC(int32_t sum, uint32_t count)
{
    char string[17];  // 16 characters across the display

    OLEDStringDraw ("Heli Control", 0, 0);
    // This works
    uint32_t mean =  (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf (string, sizeof(string), "Mean ADC = %4d", mean);
    // Update line on display.
    OLEDStringDraw (string, 0, 1);
    OLEDStringDraw("              ", 0, 2);

    usnprintf (string, sizeof(string), "Sample # %5d", count);
    OLEDStringDraw (string, 0, 3);
}

void
displayOff(void)
{
    // Blank the display
    OLEDStringDraw("                ", 0, 0);
    OLEDStringDraw("                ", 0, 1);
    OLEDStringDraw("                ", 0, 2);
    OLEDStringDraw("                ", 0, 3);
}

int
main(void)
{


    uint16_t i;
    int32_t sum;
    SysCtlPeripheralReset (UP_BUT_PERIPH);        // UP button GPIO
    SysCtlPeripheralReset (DOWN_BUT_PERIPH);      // DOWN button GPIO
    SysCtlPeripheralReset(LEFT_BUT_PERIPH);     // LEFT button GPIO
    initButtons ();
    initClock ();
    initADC ();
    initDisplay ();
    initYawGPIO();

    initCircBuf (&g_inBuffer, BUF_SIZE);




    // Enable interrupts to the processor.
    IntMasterEnable();
    SysCtlDelay (SysCtlClockGet() / 6);





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
        for (i = 0; i < BUF_SIZE; i++) {
            sum = sum + readCircBuf (&g_inBuffer);
        }

        if (checkButton(UP) == PUSHED) {
            // Change the display mode flag
            flag = flag + 1;
            if (flag == 3) {
                // Loop back around, only 3 modes.
                flag = 0;
            }
        }

        if (checkButton(LEFT) == PUSHED) {
            // Reset landed voltage
            voltageLanded = readCircBuf(&g_inBuffer);
        }

        // Alternate between displays
        if (flag == 0) {
            displayAltPercent(sum, g_ulSampCnt, voltageLanded, voltageMaxHeight);

        }   else if (flag == 1) {
            displayMeanADC(sum, g_ulSampCnt);

        }   else {
            displayOff();
        }


       SysCtlDelay (SysCtlClockGet() / 150);  // Update display
    }
}

