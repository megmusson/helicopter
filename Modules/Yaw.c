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

//YAW MEASUREMENT DEFINITIONS AND GLOBAL VARIABLES
#define PHASE_A GPIO_INT_PIN_0
#define PHASE_B GPIO_INT_PIN_1

static int yaw;
int8_t yawChangeTable[16] = { 0, -1, 1, 0, 1, 0, 0, -1,
                              -1, 0, 0,1, 0, 1, -1, 0};
uint8_t yPrev = 0; //global variables to save previous bit states.

void
GPIOIntHandler(void)
{
    //get values from both sensors as well as their previous values
    uint8_t Value = 0;
    uint8_t yInRead;

    yInRead = GPIOPinRead(GPIO_PORTB_BASE, PHASE_A | PHASE_B);


    Value = yPrev<<2 | yInRead;


    //use table to determine whether add or subtract one to yaw
    yaw = yaw + yawChangeTable[Value];


    yPrev = yInRead;
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


