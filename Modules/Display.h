/*
 * Display.h
 *
 *  Created on: 17/05/2021
 *      Author: cmcdr
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "utils/ustdlib.h"

#define DISPLAY_HZ 150
#define SLOW_TICKRATE_HZ 4

//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

static uint32_t ticks;


void initDisplay(void);

void
initialiseUSB_UART (void);

void
UARTSend (char *pucBuffer);

void displayStatus(bool flying);



#endif /* DISPLAY_H_ */
