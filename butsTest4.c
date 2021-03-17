//*****************************************************************************
//
// butsTest4.c - Test for the buttons4 module (and Orbit display)
//
// Note: uses the string conversion routines in ustdlib.c
//
// Author:  P.J. Bones	UCECE
// Last modified:	14.2.2018
//

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "stdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "buttons4.h"


//*****************************************************************************
// Initialisation functions: clock, display
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
}


// *******************************************************
void
initDisplay (void)
{
  // intialise the Orbit OLED display
	OLEDInitialise ();
}


//*****************************************************************************
//
// Function to display the mean interval in usec
//
//*****************************************************************************
void
displayButtonState (char *butStr, char *stateStr, uint8_t numPushes, uint8_t charLine)
{
    char string[17]; // Display fits 16 characters wide.
	
    OLEDStringDraw ("                ", 0, charLine);
    usnprintf (string, sizeof(string), "%s - %s %2d", butStr, stateStr, numPushes);
    OLEDStringDraw (string, 0, charLine);
}


int
main(void)
{
	int8_t upPushes = 0, downPushes = 0, leftPushes = 0, rightPushes = 0;
	
	initClock ();
	initDisplay ();
	initButtons ();
    
    displayButtonState ("UP   ", "RELS", upPushes, 0);
    displayButtonState ("DOWN ", "RELS", downPushes, 1);
    displayButtonState ("LEFT ", "RELS", upPushes, 2);
    displayButtonState ("RIGHT", "RELS", downPushes, 3);

	while(1)
	{
		uint8_t butState;

        updateButtons ();		// Poll the buttons
        // check state of each button and display if a change is detected
        butState = checkButton (UP);
        switch (butState)
        {
        case PUSHED:
        	displayButtonState ("UP   ", "PUSH", ++upPushes, 0);
        	break;
        case RELEASED:
        	displayButtonState ("UP   ", "RELS", upPushes, 0);
        	break;
        // Do nothing if state is NO_CHANGE
        }
        butState = checkButton (DOWN);
        switch (butState)
        {
        case PUSHED:
        	displayButtonState ("DOWN ", "PUSH", ++downPushes, 1);
        	break;
        case RELEASED:
        	displayButtonState ("DOWN ", "RELS", downPushes, 1);
        	break;
        // Do nothing if state is NO_CHANGE
        }
        butState = checkButton (LEFT);
        switch (butState)
        {
        case PUSHED:
            displayButtonState ("LEFT ", "PUSH", ++leftPushes, 2);
            break;
        case RELEASED:
            displayButtonState ("LEFT ", "RELS", leftPushes, 2);
            break;
        // Do nothing if state is NO_CHANGE
        }
        butState = checkButton (RIGHT);
        switch (butState)
        {
        case PUSHED:
            displayButtonState ("RIGHT", "PUSH", ++rightPushes, 3);
            break;
        case RELEASED:
            displayButtonState ("RIGHT", "RELS", rightPushes, 3);
            break;
        // Do nothing if state is NO_CHANGE
        }

		SysCtlDelay (SysCtlClockGet () / 150);	// Approx 50 Hz polling
	}
}

