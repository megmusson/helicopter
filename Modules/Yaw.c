/*
 * Module for Yaw-related code.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "Modules/Yaw.h"

//YAW MEASUREMENT DEFINITIONS AND GLOBAL VARIABLES
// PORT B
#define PHASE_A GPIO_INT_PIN_0
#define PHASE_B GPIO_INT_PIN_1


int8_t yawChangeTable[16] = { 0, -1, 1, 0, 1, 0, 0, -1,
                              -1, 0, 0,1, 0, 1, -1, 0};
uint8_t yPrev = 0; //global variables to save previous bit states.





void
GPIOIntHandler(void)
{
    //get values from both sensors as well as their previous values
    IntMasterDisable();
    yInRead = GPIOPinRead(GPIO_PORTB_BASE, PHASE_A | PHASE_B);
    Value = yPrev<<2 | yInRead;
    //use table to determine whether add or subtract one to yaw
    yaw = yaw + yawChangeTable[Value];
    yPrev = yInRead;
    GPIOIntClear(GPIO_PORTB_BASE,PHASE_A | PHASE_B);
    IntMasterEnable();

}

/*
 * uint8_t Value;
    uint8_t yInRead;yInRead = GPIOPinRead(GPIO_PORTB_BASE, PHASE_A | PHASE_B);
Value = yPrev<<2 | yInRead;
//use table to determine whether add or subtract one to yaw
yaw = yaw + yawChangeTable[Value];
yPrev = yInRead;
GPIOIntClear(GPIO_PORTB_BASE,PHASE_A | PHASE_B);
*/

void
initYawGPIO (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, PHASE_A | PHASE_B);


    GPIOIntRegister(GPIO_PORTB_BASE, GPIOIntHandler);

    GPIOIntTypeSet(GPIO_PORTB_BASE,PHASE_A | PHASE_B, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTB_BASE, PHASE_A | PHASE_B);//enable the interrupt on pin 0 and pin 1 on port B

}

int32_t calcDegrees (void)
{
    // Calculates the current yaw position in degrees between 180 and -180 degrees.
    // Also keeps yaw within -220 and +220

    // Create a critical section, these calculations take ages and should be protected


    IntMasterDisable();
    int32_t degrees;
    if (yaw >= 225) {
        yaw = -224 ;
    } else if (yaw <= -224) {
        yaw = 224;
    }
    // Sets degrees to 180 if yaw is at the 180 position, preventing the % sign from setting degrees to zero.
    if (yaw == 224) {
        degrees = 180;
    } else {
        degrees = ((yaw*360)/448)%180;
    }
    IntMasterEnable();

    return degrees;
}

