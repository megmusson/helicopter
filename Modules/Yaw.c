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
#define PHASE_A         GPIO_INT_PIN_0
#define PHASE_B         GPIO_INT_PIN_1
#define YAW_EDGES                 448
#define ROTATION_DEG            360

int yaw;

uint8_t yInRead = 0;

// Using a lookup table for quadrature decoding
int8_t yawChangeTable[16] = { 0, -1, 1, 0,
                                            1, 0, 0, -1,
                                           -1, 0, 0,1,
                                            0, 1, -1, 0};
uint8_t yPrev = 0;

void
GPIOIntHandler(void) {

    yInRead = GPIOPinRead(GPIO_PORTB_BASE, PHASE_A | PHASE_B);
    lookupIndex = yPrev<<2 | yInRead;
    //use table to determine whether add or subtract one to yaw
    yaw = yaw + yawChangeTable[lookupIndex];
    yPrev = yInRead & 0b0011;
    GPIOIntClear(GPIO_PORTB_BASE,PHASE_A | PHASE_B);


    // Keep Yaw below one full revolution so we don't end up spinning more than we have to.
    if (yaw >= (YAW_EDGES/2 + 1)) {
        yaw -= YAW_EDGES;
    } else if (yaw <= (-1 * YAW_EDGES/2)) {
        yaw += YAW_EDGES;
    }
}

void
initYawGPIO (void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, PHASE_A | PHASE_B);

    // Set up interrupts for quadrature decoding
    GPIOIntRegister(GPIO_PORTB_BASE, GPIOIntHandler);
    GPIOIntTypeSet(GPIO_PORTB_BASE,PHASE_A | PHASE_B, GPIO_BOTH_EDGES);
    GPIOIntEnable(GPIO_PORTB_BASE, PHASE_A | PHASE_B);  //enable the interrupt on pin 0 and pin 1 on port B

    // Now for the independent yaw zero
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);
}

int32_t
calcDegrees (void) {
    // Calculates the current yaw position in degrees between 180 and -180 degrees.
    // Also keeps yaw within -220 and +220
    int32_t degrees;
    // Sets degrees to 180 if yaw is at the 180 position, preventing the % sign from setting degrees to zero.
    degrees = ((yaw*ROTATION_DEG)/YAW_EDGES);
    return degrees;
}
