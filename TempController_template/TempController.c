//*****************************************************************************
//
// hello.c - Simple hello world example.
//
// Copyright (c) 2011-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.0.1.11577 of the EK-LM4F232 Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "grlib/grlib.h"
#include "drivers/cfal96x64x16.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"

#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"

#include "stdio.h"
#include <math.h>
#include <string.h>
#include "driverlib/fpu.h"

#include "DADSlab/my_code.h"
#include "DADSlab/images.h"



//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Hello World (hello)</h1>
//!
//! A very simple ``hello world'' example.  It simply displays ``Hello World!''
//! on the display and is a starting point for more complicated applications.
//! This example uses calls to the TivaWare Graphics Library graphics
//! primitives functions to update the display.  For a similar example using
//! widgets, please see ``hello_widget''.
//
//*****************************************************************************

//*****************************************************************************
//
// The clock rate for the SysTick interrupt and a counter of system clock
// ticks.  The SysTick interrupt is used for basic timing in the application.
//
//*****************************************************************************
#define CLOCK_RATE              100
#define MS_PER_SYSTICK          (1000 / CLOCK_RATE)
static volatile uint32_t g_ui32TickCount;

//*****************************************************************************
//
// The number of SysTick ticks per second.
//
//*****************************************************************************
#define TICKS_PER_SECOND        20
#define FSECONDS_PER_TICK       (1.0F / (float)TICKS_PER_SECOND)

//*****************************************************************************
//
// Global variables
// can be changed by INTs
//
//*****************************************************************************

//uint32_t ulPeriod;
//int32_t ADC_Length = 4;
uint32_t ulADC0_Value[4]; 	// A buffer to hold one set of ADC data
							// that is acquired per sample time.
volatile uint8_t counter_for_ADC = 0;
volatile uint8_t counter_for_controller = 0;
volatile uint8_t counter_for_packet = 0;

volatile bool ADC_done = false;	// flag
volatile bool timer_tick = false;	// flag

volatile bool controller_time = false;
volatile bool packet_time = false;

tContext sContext;
tRectangle sRect;

// "name":"DADS","REF":"32.5","FLT":"12.4","LSH":"12.8","H":"99.9","F":"32.5"\r\n
//uint8_t packet_buffer[100];
//uint8_t ascii_REF_T[10];
//uint8_t ascii_FLT_T[10];
//uint8_t ascii_FLT_V[10];
//uint8_t ascii_LSH_T[10];
//uint8_t ascii_LSH_V[10];
//uint8_t ascii_PWMH[10];
//uint8_t ascii_PWMF[10];

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// Handles the SysTick timeout interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    // Increment the tick count.
    g_ui32TickCount++;

    counter_for_ADC++;

    if (counter_for_ADC > 9)		// 10 x 10ms = 100ms
    {
    	counter_for_ADC = 0;
    	timer_tick = true;

    // Toggle the LED on the board so the user can see that the acquisition
    // is running.
    	//MAP_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2,
                     //~MAP_GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_2));
    	//counter_for_controller++;
        //if (counter_for_controller > 2)		// 2 x 100ms = 200ms
        //{
        	//counter_for_controller = 0;
        	//controller_time = true;
        //}

    	counter_for_packet++;
        if (counter_for_packet > 4)		// 5 x 100ms = 500ms
        {
        	counter_for_packet = 0;
        	packet_time = true;
        }




    }

}

//*****************************************************************************
//
// Print "Hello World!" to the display.
//
//*****************************************************************************
int
main(void)
{
    //tContext sContext;
    //tRectangle sRect;
    uint32_t ui32SysClock;
    //uint32_t ui32LastTickCount = 0;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    MAP_FPULazyStackingEnable();
    MAP_FPUEnable();

    //
    // Set the clocking to run at 50 MHz.
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
    ui32SysClock = MAP_SysCtlClockGet();


    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    MAP_SysCtlDelay(100);


    //
    // Initialize the UART.
    //
    ConfigureUART();
    UARTprintf("Hello, world!\n");


    //
    // Enabled LED GPIO
    //
    //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);
    //
    // Toggle the LED on the board so the user can see that the acquisition
    // is running.
    //
    //MAP_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2,
                    // ~MAP_GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_2));

    //
    // Configure SysTick to periodically interrupt.
    //
    g_ui32TickCount = 0;
    MAP_SysTickPeriodSet(ui32SysClock / CLOCK_RATE);
    MAP_SysTickIntEnable();
    MAP_SysTickEnable();

    //
    // Initialize Analog to Digital Converter
    //
    InitADC();

    //
    // Initialize PWM
    //
    InitPWM();

    //InitPWMold();
    InitTimerPWM();

    //
    // Initialize UART1
    //
    InitUART1();
    //UART1printf("Hello, world!\n");

    InitI2C1();

    //
    // Initialize the display driver.
    //
    CFAL96x64x16Init();

    GrContextInit(&sContext, &g_sCFAL96x64x16);
    //InitialScreen();

    //Draw text on screen
    //ReDrawTxtBox("FTL: 5.11V 99.9C", 0,0,95,12, ClrYellow);
    //ReDrawTxtBox("H=99% F=33%", 0,12,95,12, ClrGray);
    //ReDrawTxtBox("REF: 123.1C", 0,25,95,12, ClrRed);
    //ReDrawTxtBox("C1234567890", 0,38,95,12, ClrGreen);
    //ReDrawTxtBox("FLT:5.11V 33.3C", 0,51,95,12, ClrGray);


    MAP_IntMasterEnable();	// Enables the processor interrupt.

    // DADS logo
    const uint8_t *pui8SplashLogo = g_pui8Image_UTS;
    GrImageDraw(&sContext, pui8SplashLogo, 0, 0);
    // wait for a moment
    while(g_ui32TickCount < 200){};

    // show the firmware version
    sRect.i16XMin = 0;
    sRect.i16YMin = 18;
    sRect.i16XMax = 95;
    sRect.i16YMax = 42;
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    GrContextForegroundSet(&sContext, ClrBlue);
    GrRectFill(&sContext, &sRect);

    GrContextForegroundSet(&sContext, ClrWhite);	// font colour
    GrStringDraw(&sContext, "ver. 09/05/2018", -1, 0, 23, 0);
    GrStringDraw(&sContext, "your name here ", -1, 0, 33, 0);
    // Flush any cached drawing operations.
    //GrFlush(&sContext);

    while(g_ui32TickCount < 400){};

    // the loop
    while(1)
    {
        // Wait for the next timer tick.
    	while(!timer_tick){};	// wait for timer tick
    	timer_tick = false;
    	// all the tasks here

        // Kick off the next ADC acquisition.  When these are done they will
        // cause an ADC interrupt.
    	ADC_done = false;
        MAP_ADCProcessorTrigger(ADC0_BASE, 0);

        // uncomment the following line when you finish InitADC
    	while (!ADC_done){};	// wait for ADC to finish

    	// blink the LED
    	MAP_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2,
                     ~MAP_GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_2));

    	// read temperature from the IR temperature sensor
    	IR_Read();

    	// update the PID controller
    	PID_controller();

    	// update OLED display
    	adc2ASCII();
    	//V2Temp();
    	UpdateScreen();

    	// output packet to PC
    	if(packet_time)
    	{
        	SendPacket();
        	packet_time = false;
    	}


    }
}
