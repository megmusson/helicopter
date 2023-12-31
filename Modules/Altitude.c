/* Altitude Module
 *  Contains everything for reading altitude values.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "circBufT.h"
#include "stdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "Modules/Altitude.h"

// Defines and initialise variables
#define BUF_SIZE 8
#define MAX_VOLT_BITS 1300
#define ADC_SEQUENCE 3
static int voltageLanded = 0;
static int voltageMaxHeight = 0;
static int sum = 0;

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

    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer, ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE);
}

void
initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    //
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.

    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE, ADC_TRIGGER_PROCESSOR, 0);
    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE, 0, ADC_CTL_CH9 | ADC_CTL_IE | // CHANGE HERE FOR LAB 9 or 0+++++++++++++++++++++++++++++++++++++++
                             ADC_CTL_END);
    //
    // Since sample sequence 3 is now configured, it must be enabled.

    ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE);
    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, ADC_SEQUENCE, ADCIntHandler);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, ADC_SEQUENCE);


    initCircBuf(&g_inBuffer, BUF_SIZE);

}


int32_t
calcAltAverage(void) {
    //Calculates and returns the rounded average altitude of the helicopter.
    return ((2 * sum + BUF_SIZE) / 2 / BUF_SIZE);

}

int32_t
calcAltPercent(void){
    // Calculates the altitude of the helicopter as a percentage relative to its maximum and minimum height
    return (100 - 100*(calcAltAverage() - voltageMaxHeight )/(voltageLanded - voltageMaxHeight));
}

void
setMinMaxAlt(void) {
    //Sets the maximum and minimum voltage value for altitude  
    voltageLanded = readCircBuf(&g_inBuffer);
    voltageMaxHeight = voltageLanded - MAX_VOLT_BITS;
}

void
readAltitude(void) {
    //Sums all the values in buffer to determine average altitude over BUF_SIZE samples
    sum = 0;
    int i;
    for (i = 0; i < BUF_SIZE; i++)
    {
        sum = sum + readCircBuf(&g_inBuffer);
    }
}
