
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>



#include "OrbitOLED/OrbitOLEDInterface.h"
#include "driverlib/uart.h"

#include "Modules/Display.h"
#include "Modules/Altitude.h"
#include "Modules/Yaw.h"




static uint32_t ticks = 0;
//bool flying = 0;

extern uint32_t altitudeTarget;
extern int yawTarget;
extern int32_t totalAltDC;
extern int32_t totalYawDC;
extern int yaw;


void initDisplay(void)
{
    // intialise the Orbit OLED display
    OLEDInitialise();
}

void
initialiseUSB_UART (void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    //

    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);
    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure (GPIO_PA0_U0RX);
    GPIOPinConfigure (GPIO_PA1_U0TX);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
            UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}

void
UARTSend (char *pucBuffer)
{
    // Loop while there are more characters to send.
    while(*pucBuffer)
    {
        // Write the next character to the UART Tx FIFO.
        UARTCharPut(UART_USB_BASE, *pucBuffer);
        pucBuffer++;
    }
}

void displayStatus(bool flying)
{
    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    char string[16];  // 16 characters across the display


    usnprintf(string, sizeof(string), "Main[%2d] Tail [%2d] \n\r",totalAltDC, totalYawDC);

    OLEDStringDraw(string, 0, 0);

    if (ticks >= (DISPLAY_HZ/SLOW_TICKRATE_HZ)){
        UARTSend(string);
    }


    usnprintf(string, sizeof(string), "Yaw =%3d [%3d]\n\r", calcDegrees(), yawTarget);
            OLEDStringDraw(string, 0, 1);

    if (ticks >= (DISPLAY_HZ/SLOW_TICKRATE_HZ)) {
            UARTSend(string);

     }



    if (flying) {
        usnprintf(string, sizeof(string), "Mode: Flying\n\r");

        OLEDStringDraw(string, 0, 2);
    } else {
        usnprintf(string, sizeof(string), "Mode: Landed\n\r");

        OLEDStringDraw(string, 0, 3);

    }

    if (ticks >= (DISPLAY_HZ/SLOW_TICKRATE_HZ)) {
            UARTSend(string);
            ticks = 0;
     }


    ticks += 1;
}
