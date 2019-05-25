//*****************************************************************************
//
// my_code.c - Put your code here ...
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

#include "my_code.h"


//#define CLOCK_RATE              100
//#define MS_PER_SYSTICK          (1000 / CLOCK_RATE)
//static volatile uint32_t g_ui32TickCount;

extern uint32_t ulADC0_Value[4];
extern bool ADC_done;   // flag

extern tContext sContext;
extern tRectangle sRect;

#define PWM_HEATER_FREQUENCY 1000
#define PWM_FAN_FREQUENCY 10


float ch0V, ch1V, ch2V, ch3V;
float ch0T, ch1T, ch2T, ch3T;

char packet_buffer[100];
char ascii_REF_T[10];
char ascii_REF_V[10];
char ascii_LSH_T[10];
char ascii_LSH_V[10];
char ascii_FLT_T[10];
char ascii_FLT_V[10];
char ascii_IR_T[10];
char ascii_PWMH[10];
char ascii_PWMF[10];

// parameters for PID
float Kp = 100.0f;
float Ki = 0.2f;
float Kd = 0.02f;

float previous_error = 0;
float integral = 0;
float Dt = 0.0001;  // 100ms

//*****************************************************************************
// Initialization functions
//*****************************************************************************

void InitPWM()
{
    volatile uint32_t PWM_HEATER_Period;
    volatile uint32_t PWM_FAN_Period;
    volatile uint32_t ui32Load;
    volatile uint32_t ui32PWMClock;

    //
    // Set the PWM clock to the system clock / 8 40MHz/8 =  5 MHz
    //
    MAP_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    //
    // The PWM peripheral must be enabled for use.
    //
    //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    //
    // For this example PWM1 is used with PortG Pins 6 and 7.  The actual port
    // and pins used may be different on your part, consult the data sheet for
    // more information.  GPIO port G needs to be enabled so these pins can be
    // used.
    // PWM0 PG3

    //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

    //
    // Configure the GPIO pin muxing to select PWM functions for these pins.
    // This step selects which alternate function is available for these pins.
    // This is necessary if your part supports GPIO pin function muxing.
    // Consult the data sheet to see which functions are allocated per pin.
    //
    MAP_GPIOPinConfigure(GPIO_PG6_M0PWM6);
    //MAP_GPIOPinConfigure(GPIO_PG7_M0PWM7);
    //MAP_GPIOPinConfigure(GPIO_PG2_M1PWM0);
    MAP_GPIOPinConfigure(GPIO_PG3_M1PWM1);

    //
    // Configure the GPIO pad for PWM function on pins PG6 and PG7.  Consult
    // the data sheet to see which functions are allocated per pin.
    //
    MAP_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_6);
    //MAP_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_7);
    //MAP_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_2);
    MAP_GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_3);

    //Configure pin and pad types/characteristics
    //MAP_GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    //MAP_GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_3, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    //MAP_GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_6, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    //MAP_GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);


    //
    // Configure the PWM0 to count up/down without synchronization.
    // Note: Enabling the dead-band generator automatically couples the 2
    // outputs from the PWM block so we don't use the PWM synchronization.
    //
    //MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |
    //                PWM_GEN_MODE_NO_SYNC);
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC| PWM_GEN_MODE_DBG_STOP);
    MAP_PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC| PWM_GEN_MODE_DBG_STOP);
    //
    // Set the PWM period to 250Hz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * SysClk.  Where N is the
    // function parameter, f is the desired frequency, and SysClk is the
    // system clock frequency.
    // In this case you get: (1 / 250Hz) * 16MHz = 64000 cycles.  Note that
    // the maximum period you can set is 2^16 - 1.
    //

    //PWMPeriod = MAP_SysCtlClockGet()/2.0 * Fs;
    //PWMPeriod /= 8;

    //ui32PWMClock = MAP_SysCtlClockGet();  // 40 MHz

    //PWM_HEATER_Period = MAP_SysCtlClockGet()/2.0 * Fs;
    PWM_HEATER_Period = MAP_SysCtlClockGet() / PWM_HEATER_FREQUENCY;
    PWM_HEATER_Period /= 64;
    //PWM_HEATER_Period = 5000; // 40 MHz / 1kHz /8

    PWM_FAN_Period = MAP_SysCtlClockGet() / PWM_FAN_FREQUENCY;
    PWM_FAN_Period /= 64;
    //PWM_FAN_Period = 65535;   //

    //uint32_t PWM_HEATER_Period;
    //uint32_t PWM_FAN_Period
    //#define PWM_HEATER_FREQUENCY 1000
    //#define PWM_FAN_FREQUENCY 5


    //MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWMPeriod);
   // MAP_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, PWMPeriod);

    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWM_HEATER_Period);
    MAP_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, PWM_FAN_Period);



    //
    // Set PWM0 PD0 to a duty cycle of 25%.  You set the duty cycle as a
    // function of the period.  Since the period was set above, you can use the
    // PWMGenPeriodGet() function.  For this example the PWM will be high for
    // 25% of the time or 16000 clock cycles (64000 / 4).
    //
    //MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
    //                 MAP_PWMGenPeriodGet(PWM0_BASE, PWM_OUT_6) / 4);
    //MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,
    //                 MAP_PWMGenPeriodGet(PWM0_BASE, PWM_OUT_7) / 4);
    //MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0,
    //                 MAP_PWMGenPeriodGet(PWM1_BASE, PWM_OUT_0) / 4);
    //MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,
    //                 MAP_PWMGenPeriodGet(PWM1_BASE, PWM_OUT_1) / 4);
    //Initialises the default duty cycle to be 50%
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,  PWM_HEATER_Period/2);
    //MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,  PWMPeriod/2);
    //MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0,  PWMPeriod/2);
    MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1,  PWM_FAN_Period/2);



    //
    // Enable the PWM0 Bit 6 (PD6) and Bit 7 (PD7) output signals.
    //
    //MAP_PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    //MAP_PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
    //MAP_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    MAP_PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);

    //
    // Enables the counter for a PWM generator block.
    //
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    MAP_PWMGenEnable(PWM1_BASE, PWM_GEN_0);

}

void InitTimerPWM(void)
// from here http://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/t/249597.aspx
// another example here http://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/t/236151.aspx
{
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);
      //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);    // already done
      MAP_GPIOPinConfigure(GPIO_PD0_WT2CCP0);
      MAP_GPIOPinConfigure(GPIO_PD1_WT2CCP1);
      MAP_GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
      MAP_GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_1);
      MAP_TimerConfigure(WTIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
      //MAP_TimerConfigure(WTIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM);
      MAP_TimerLoadSet(WTIMER2_BASE, TIMER_A, 50000);   // 50MHz / 50000 -> 1kHz
      MAP_TimerMatchSet(WTIMER2_BASE, TIMER_A, 25000);
      MAP_TimerLoadSet(WTIMER2_BASE, TIMER_B, 10000000);    // 50MHz / 10000000 -> 5Hz
      MAP_TimerMatchSet(WTIMER2_BASE, TIMER_B, 5000000);
      MAP_TimerEnable(WTIMER2_BASE, TIMER_A | TIMER_B);
      //MAP_TimerEnable(WTIMER2_BASE, TIMER_B);
}


void InitUART1()
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    MAP_GPIOPinConfigure(GPIO_PC4_U1RX);
    MAP_GPIOPinConfigure(GPIO_PC5_U1TX);
    MAP_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    MAP_UARTConfigSetExpClk(UART1_BASE, MAP_SysCtlClockGet(), 115200,
    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    //MAP_IntEnable(INT_UART1);
    //UARTIntEnable(UART1_BASE, UART_INT_TX);
    //MAP_UARTTxIntModeSet(UART1_BASE, UART_TXINT_MODE_EOT)
    MAP_UARTCharPut(UART1_BASE, 'E');
    MAP_UARTCharPut(UART1_BASE, 'n');
    MAP_UARTCharPut(UART1_BASE, 't');
    MAP_UARTCharPut(UART1_BASE, 'e');
    MAP_UARTCharPut(UART1_BASE, 'r');
    MAP_UARTCharPut(UART1_BASE, ' ');
    MAP_UARTCharPut(UART1_BASE, 'T');
    MAP_UARTCharPut(UART1_BASE, 'e');
    MAP_UARTCharPut(UART1_BASE, 'x');
    MAP_UARTCharPut(UART1_BASE, 't');
    MAP_UARTCharPut(UART1_BASE, ':');
    MAP_UARTCharPut(UART1_BASE, ' ');
}

void InitADC()
{
    // Enable the ADC peripherals and the associated GPIO port
    //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Configure the pins to be used as analog inputs.
    // to be finished by you
    // MAP_GPIOPinTypeADC...

    // Select the external reference for greatest accuracy.
    // to be finished by you
    // MAP_ADCReferenceSet(...

    // Apply workaround for erratum 6.1, in order to use the
    // external reference.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //MAP_SysCtlDelay(100);
    HWREG(GPIO_PORTB_BASE + GPIO_O_AMSEL) |= GPIO_PIN_6;

    //Set the ADC samples speed
    // doesn't work in Tivaware http://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/t/261496.aspx
    // don't do it
    //MAP_SysCtlADCSpeedSet(SYSCTL_ADCSPEED_125KSPS);

    MAP_ADCSequenceDisable(ADC0_BASE, 0);


    // Initialize the ADC0 peripheral using sequencer 0 and processor trigger.
    // to be finished by you
    // MAP_ADCSequenceConfigure(...

    //Sets the input positions, and order they are read, the final port (3) is set
    // to trigger an interrupt and stop the sequencer
    // to be finished by you
    //MAP_ADCSequenceStepConfigure(...
    //MAP_ADCSequenceStepConfigure(...
    //MAP_ADCSequenceStepConfigure(...
    //MAP_ADCSequenceStepConfigure(...

    //Enable the sequencer that goes through the things above
    // to be finished by you
    //MAP_ADCSequenceEnable(...

    //Clear the ADC interupt
    // to be finished by you
    //MAP_ADCIntClear(...

    //Enable the ADC interupt
    // to be finished by you
    //MAP_ADCIntEnable(...

    //Enable the ADC interupt globally
    // to be finished by you
    //MAP_IntEnable(...

}


void InitI2C1(void)
{
    // based on example from here:
    // http://e2e.ti.com/support/microcontrollers/stellaris_arm/f/471/t/169023.aspx?pi74263=2
    // initialize I2C1 peripheral (PA6=SCL, PA7=SDA)
    //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);


    MAP_GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    MAP_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    MAP_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    MAP_GPIOPinConfigure(GPIO_PA7_I2C1SDA);

    // was MAP_I2CMasterInitExpClk(I2C1_MASTER_BASE, ROM_SysCtlClockGet(), false);
    MAP_I2CMasterInitExpClk(I2C1_BASE, ROM_SysCtlClockGet(), false);
    MAP_SysCtlDelay(10000);  // delay mandatory here - otherwise portion of SlaveAddrSet() lost!

}

//*****************************************************************************
// End of Initialization functions
//*****************************************************************************

//*****************************************************************************
// Interrupt Service Routines
//*****************************************************************************

void UART1IntHandler()
{
    uint32_t ui32Status;
    ui32Status = MAP_UARTIntStatus(UART1_BASE, true); //get interrupt status
    MAP_UARTIntClear(UART1_BASE, ui32Status); //clear the asserted interrupts
}


void ADC0SS0Handler(void)
{

    // Clear the interrupts for all ADC sequencers that are used.
    MAP_ADCIntClear(ADC0_BASE, 0);

    // Retrieve the data acquired per sample time into a buffer
    MAP_ADCSequenceDataGet(ADC0_BASE, 0, ulADC0_Value);

    ADC_done = true;

}
//*****************************************************************************
// End of Interrupt Service Routines
//*****************************************************************************

//*****************************************************************************
// PID Controller Functions
//*****************************************************************************
// http://www.codeproject.com/Articles/36459/PID-process-control-a-Cruise-Control-example

void PID_controller(void)
{
    /*
     * Pseudo code (source Wikipedia)
     *
    previous_error = 0
    integral = 0
    start:
        error = setpoint – PV [actual_position]
        integral = integral + error*dt
        derivative = (error - previous_error)/dt
        output = Kp*error + Ki*integral + Kd*derivative
        previous_error = error
        wait(dt)
        goto start
    */

    float error;


    float tempPWM;
    uint32_t PWMheater;
    uint32_t PWMfan;

    float fanOutput = 34.1;


    float heaterOutput;
    // to be finished by you
    // ...
    float oldError;
    float KaF;
    float KbF;
    float KaH;
    float KbH;
    float oldFanPWM;
    float oldHeaterPWM;
    float actualTemp;


    error = ch0V - ch2V; // error = reference channel - filter output

    if(error >= 0.3)
    {
        heaterOutput = 0.01;
        fanOutput = oldFanPWM + (((KaF*(error))+ ((KbF)*oldError)));
    }
    else
    {
        heaterOutput = oldHeaterPWM + (KaH*(-error) + (KbH*(-oldError)));
        fanOutput = 0.01;
    }

    oldError = error;



    // convert output (PWM) into the value to write into registers
    if(heaterOutput > 0.99) heaterOutput = 0.99f; // If heaterOuput is over 0.99, make it equal to 0.99
    if(heaterOutput < 0.01) heaterOutput = 0.01f; // If heaterOutput is under 0.01, make it equal to 0.01

    tempPWM = heaterOutput * 50000;
    PWMheater = (uint32_t)tempPWM;
    MAP_TimerMatchSet(WTIMER2_BASE, TIMER_A, 50000 - PWMheater);

    tempPWM = fanOutput * 10000000;
    PWMfan = (uint32_t)tempPWM;
    MAP_TimerMatchSet(WTIMER2_BASE, TIMER_B, 10000000 - PWMfan);

    // update PWM values for the screen
    snprintf(ascii_PWMH, 10, "%.1f", output);
    snprintf(ascii_PWMF, 10, "%.1f", fanout);

}

//*****************************************************************************
// End of PID Controller Functions
//*****************************************************************************

//*****************************************************************************
// Utilities
//*****************************************************************************
void adc2ASCII(void)
//uint8_t ascii_REF_T[10];
//uint8_t ascii_REF_V[10];
//uint8_t ascii_FLT_T[10];
//uint8_t ascii_FLT_V[10];
//uint8_t ascii_LSH_T[10];
//uint8_t ascii_LSH_V[10];
//uint8_t ascii_PWMH[10];
//uint8_t ascii_PWMF[10];

// on entry the ulADC0_Value[4] buffer holds the current reading
// from ADC CH0-CH3
// the readings are converted into voltages and temperatures
// and later converted into ASCII
// and stored globally for oled and packet updates
{
    ui32Millivolts = (g_pui32ADCData[ui8Idx] * 4100) / 819;
    uit32_t ui32Millivolts;
    float ch0V, ch1V, ch2V, ch3V;
    float ch0T, ch1T, ch2T, ch3T;

//  ui32Millivolts = (ulADC0_Value[0] * 4100) / 819;
//  ch0V = ui32Millivolts / 1000;
    ch0V = ulADC0_Value[0] * 0.7326/146;
    ch1V = ulADC0_Value[1] * 0.7326/146;
    ch2V = ulADC0_Value[2] * 0.7326/146;
    ch3V = ulADC0_Value[3] * 0.7326/146;

    snprintf(ascii_REF_V, 10, "%.2f", ch0V);
    snprintf(ascii_LSH_V, 10, "%.2f", ch1V);
    snprintf(ascii_FLT_V, 10, "%.2f", ch2V);


    ch0T = (19.6*ch0V)+26.55; //Reference
    ch1T = (ch1V + 1.3451)/0.0507;; // Level Shifter
    ch2T = (19.6*ch2V)+26.55; // LPF
    ch3T = (ch3V/0.06 + 20); // ??

    snprintf(ascii_REF_T, 10, "%.1f", ch0T);
    snprintf(ascii_LSH_T, 10, "%.1f", ch1T);
    snprintf(ascii_FLT_T, 10, "%.1f", ch2T);

}


//*****************************************************************************
// End of Utilities
//*****************************************************************************

//*****************************************************************************
// Graphical Functions
//*****************************************************************************

void InitialScreen(void)
{

    // Initialize the graphics context.
    //GrContextInit(&sContext, &g_sCFAL96x64x16);

    // Fill the top 12 rows of the screen with blue
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = 95;
    sRect.i16YMax = 11;
    GrContextForegroundSet(&sContext, ClrRed);
    GrRectFill(&sContext, &sRect);

    // Put a white box around the banner.
    //GrContextForegroundSet(&sContext, ClrWhite);
    //GrRectDraw(&sContext, &sRect);

    // Display Set Point (Reference Temperature)
    //GrContextFontSet(&sContext, g_psFontCm12);
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    GrContextForegroundSet(&sContext, ClrWhite);    // font colour
    GrStringDraw(&sContext, "REF: 32.1 C", -1, 0, 0, 0);
    //GrStringDraw(&sContext, "REF: 0.123456789", -1, 0, 0, 0);

    // Fill the next xx rows with green
    sRect.i16XMin = 0;
    sRect.i16YMin = 12;
    sRect.i16XMax = 95;
    sRect.i16YMax = 48;
    GrContextForegroundSet(&sContext, ClrBlue);
    GrRectFill(&sContext, &sRect);

    GrContextForegroundSet(&sContext, ClrWhite);    // font colour
    GrStringDraw(&sContext, "LSH: 3.22V 34.5C", -1, 0, 12, 0);  // analog and digital sensors
    GrStringDraw(&sContext, "FLT: 5.11V 99.9C", -1, 0, 23, 0);      // IR sensor
    GrStringDraw(&sContext, "IR: 54.7 C", -1, 0, 35, 0);        // IR sensor

    // Fill the next xx rows with green
    sRect.i16XMin = 0;
    sRect.i16YMin = 49;
    sRect.i16XMax = 951;
    sRect.i16YMax = 63;
    GrContextForegroundSet(&sContext, ClrGreen);
    GrRectFill(&sContext, &sRect);
    GrContextForegroundSet(&sContext, ClrWhite);    // font colour
    GrStringDraw(&sContext, "H=99% F=33%", -1, 0, 50, 0);


    //GrStringDraw(&sContext, "H=99% F=33%", -1, 12, 0, 0);
    //ReDrawTxtBox("FTL: 5.11V 99.9C", 0,0,95,12, ClrYellow);
    //ReDrawTxtBox("H=99% F=33%", 0,12,95,12, ClrGray);
    //ReDrawTxtBox("REF: 123.1C", 0,25,95,12, ClrRed);
    //ReDrawTxtBox("C1234567890", 0,38,95,12, ClrGreen);
    //ReDrawTxtBox("FLT:5.11V 33.3C", 0,51,95,12, ClrGray);

    // Flush any cached drawing operations.
    GrFlush(&sContext);
}

void UpdateScreen(void)
{
<<<<<<< HEAD
	char temp[20];
	char lsh[20];
	char flt[20];
	char ir[20];
	char pwm[20];
=======
    char temp[20];
>>>>>>> c08139050f6450dd82f8cb481f7c147c18bd8b89

    // Initialize the graphics context.
    //GrContextInit(&sContext, &g_sCFAL96x64x16);

    // Fill the top 12 rows of the screen with blue
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = 95;
    sRect.i16YMax = 8;
    GrContextForegroundSet(&sContext, ClrRed);
    GrRectFill(&sContext, &sRect);

    // Display Set Point (Reference Temperature)
    //GrContextFontSet(&sContext, g_psFontCm12);
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    GrContextForegroundSet(&sContext, ClrWhite);    // font colour
    strcpy(temp, "REF: ");
    strcat(temp, ascii_REF_V);
    strcat(temp, "V ");
    strcat(temp, ascii_REF_T);
    strcat(temp, "C");
    GrStringDraw(&sContext, temp, -1, 0, 1, 0);

    // to be finished by you
    // ...
    // Fill the next xx rows with green
    sRect.i16XMin = 0;
    sRect.i16YMin = 12;
    sRect.i16XMax = 95;
    sRect.i16YMax = 48;
    GrContextForegroundSet(&sContext, ClrBlue);
    GrRectFill(&sContext, &sRect);

    GrContextForegroundSet(&sContext, ClrWhite);    // font colour

    strcpy(lsh, "LSH: ");
    strcat(lsh, ascii_LSH_V);
    strcat(lsh, "V ");
    strcat(lsh, ascii_LSH_T);
    strcat(lsh, "C");

    strcpy(flt, "FLT: ");
    strcat(flt, ascii_FLT_V);
    strcat(flt, "V ");
    strcat(flt, ascii_FLT_T);
    strcat(flt, "C");

    strcpy(ir, "IR: ");
    strcat(ir, ascii_IR_T);
    strcat(ir, "C");

    GrStringDraw(&sContext, lsh, -1, 0, 12, 0);      // Level Shifted Signal
    GrStringDraw(&sContext, flt, -1, 0, 23, 0);      // Filtered Signal
    GrStringDraw(&sContext, ir, -1, 0, 35, 0);       // IR sensor

    // Fill the next xx rows with green
    sRect.i16XMin = 0;
    sRect.i16YMin = 49;
    sRect.i16XMax = 951;
    sRect.i16YMax = 63;
    GrContextForegroundSet(&sContext, ClrGreen);
    GrRectFill(&sContext, &sRect);


    GrContextForegroundSet(&sContext, ClrWhite);    // font colour

    strcpy(pwm, "H=");
    strcat(pwm, ascii_PWMH);
    strcat(pwm, "% F=");
    strcat(pwm, ascii_PWMF);
    strcat(pwm, "%");
    GrStringDraw(&sContext, pwm, -1, 0, 50, 0);


    // Flush any cached drawing operations.
    GrFlush(&sContext);
}

//*****************************************************************************
// End of Graphical Functions
//*****************************************************************************
//*****************************************************************************
// Digital Sensor Read
//*****************************************************************************

// based on example from here:
// http://e2e.ti.com/support/microcontrollers/stellaris_arm/f/471/t/169023.aspx?pi74263=2

int i2c_string(unsigned char slave_addr, unsigned char command)
{
volatile unsigned long value0, value1, value2;
unsigned char SLAVE_ADDRESS = 0x5a;

SLAVE_ADDRESS = slave_addr;

// Write Address Set
// was ROM_I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, SLAVE_ADDRESS, false);// false=write
MAP_I2CMasterSlaveAddrSet(I2C1_BASE, SLAVE_ADDRESS, false);// false=write
// initialize send of character from Master to Slave
// was ROM_I2CMasterDataPut(I2C1_MASTER_BASE, command);
MAP_I2CMasterDataPut(I2C1_BASE, command);

//I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
// was I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
MAP_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
// delay until transmission completes
// was while(I2CMasterBusy(I2C1_MASTER_BASE));
while(I2CMasterBusy(I2C1_BASE));
{}
// Read Address Set - is it necessary??????
// was ROM_I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, SLAVE_ADDRESS, true);// false=read
MAP_I2CMasterSlaveAddrSet(I2C1_BASE, SLAVE_ADDRESS, true);// false=read

// tell the master to read the data
// was ROM_I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
MAP_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
// delay mandatory here - otherwise portion of SlaveAddrSet() lost!
//ROM_SysCtlDelay(10000);
// was while(I2CMasterBusy(I2C1_MASTER_BASE));
while(MAP_I2CMasterBusy(I2C1_BASE));
{}
// was value0 = I2CMasterDataGet(I2C1_MASTER_BASE);
value0 = MAP_I2CMasterDataGet(I2C1_BASE);

// was ROM_I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
MAP_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
// delay mandatory here - otherwise portion of SlaveAddrSet() lost!
//ROM_SysCtlDelay(10000);
// was while(I2CMasterBusy(I2C1_MASTER_BASE));
while(MAP_I2CMasterBusy(I2C1_BASE));
{}
// was value1 = I2CMasterDataGet(I2C1_MASTER_BASE);
value1 = MAP_I2CMasterDataGet(I2C1_BASE);

// was ROM_I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
MAP_I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
// delay mandatory here - otherwise portion of SlaveAddrSet() lost!
//ROM_SysCtlDelay(10000);
// was while(I2CMasterBusy(I2C1_MASTER_BASE));
while(MAP_I2CMasterBusy(I2C1_BASE));
{}
// was value2 = I2CMasterDataGet(I2C1_MASTER_BASE);
value2 = MAP_I2CMasterDataGet(I2C1_BASE);

return ((value1<<8)+value0);

}


void IR_Read(void)
{
    uint32_t ulTemperature;

    // read temperature from the sensor
    ulTemperature = i2c_string(0x5a, 0x07);
    ulTemperature *= 100;
    ulTemperature /= 50;
    ulTemperature -= 27315;
    ulTemperature /= 10;

    //usnprintf(cTempBuf, sizeof(cTempBuf), " %2u.%1u C",
            //ulTemperature / 10, ulTemperature %10);

    //snprintf(ascii_IR_T, 10, "%.2f", ulTemperature);

    snprintf(ascii_IR_T, 10, " %2u.%1u",
            ulTemperature / 10, ulTemperature %10);
}
//*****************************************************************************
// End of Digital Sensor Read
//*****************************************************************************
//*****************************************************************************
// PC Communication Functions
//*****************************************************************************
// packet format
// {"name":"DADS","REF":32.5,"LSH":12.4,"FLT":12.8,"IR":33.4,"H":99.9,"F":32.5}\r\n
// transmission time ~5.5 ms
void SendPacket(void)
{
    char temp[100];
    //char temp0[10];

    strcpy(temp, "{\"name\":\"DADS\",\"REF\":");
    strcat(temp, ascii_REF_T);

    strcat(temp, ",\"LSH\":");
    strcat(temp, ascii_LSH_T);

    strcat(temp, ",\"FLT\":");
    strcat(temp, ascii_FLT_T);

    strcat(temp, ",\"IR\":");
    strcat(temp, ascii_IR_T);
    //strcat(temp, "33.4");

    strcat(temp, ",\"H\":");
    strcat(temp, ascii_PWMH);
    //strcat(temp, "77");

    strcat(temp, ",\"F\":");
    strcat(temp, ascii_PWMF);
    //strcat(temp, "88");


    strcat(temp, "}\r\n");

    UARTprintf("%s", temp);

}
//*****************************************************************************
// End of PC Communication Functions
//*****************************************************************************

