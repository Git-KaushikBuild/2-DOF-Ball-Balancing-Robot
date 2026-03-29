#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"

// Pin: PD0 (Servo X) & PD1 (Servo Y)

#define SYS_CLOCK_HZ       80000000UL
#define UART_BAUD_RATE     115200
#define PWM_DIVIDER        SYSCTL_PWMDIV_64
#define PWM_PERIOD_TICKS   25000

// SERVO X (PD0)
#define SERVO_X_MIN   950 
#define SERVO_X_MID   2025 
#define SERVO_X_MAX   3400 

// SERVO Y (PD1)
#define SERVO_Y_MIN   700
#define SERVO_Y_MID   1700
#define SERVO_Y_MAX   3000

char g_rxBuff[64];
uint8_t g_rxIdx = 0;
bool g_dataReady = false;

void Setup_System(void);
void SetServoX(int angle);
void SetServoY(int angle);
void ParseCommand(char *cmd);
void UART0IntHandler(void);

int main(void)
{
    Setup_System();

    SetServoX(90);
    SetServoY(90);

    while(1)
    {
        if(g_dataReady)
        {
            ParseCommand(g_rxBuff);
            g_rxIdx = 0;
            g_dataReady = false;
        }
    }
}

// --- Servo X ---
void SetServoX(int angle)
{
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;

    uint32_t ticks = 0;

    if(angle <= 90)
    {

        ticks = SERVO_X_MIN + ((angle * (SERVO_X_MID - SERVO_X_MIN)) / 90);
    }
    else
    {

        ticks = SERVO_X_MID + (((angle - 90) * (SERVO_X_MAX - SERVO_X_MID)) / 90);
    }

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ticks);
}

// --- Servo Y ---
void SetServoY(int angle)
{
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;

    uint32_t ticks = 0;

    if(angle <= 90)
    {
        ticks = SERVO_Y_MIN + ((angle * (SERVO_Y_MID - SERVO_Y_MIN)) / 90);
    }
    else
    {
        ticks = SERVO_Y_MID + (((angle - 90) * (SERVO_Y_MAX - SERVO_Y_MID)) / 90);
    }

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ticks);
}

void ParseCommand(char *cmd)
{
    if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    else
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

    int valX = 90; int valY = 90;

    char *pX = strchr(cmd, 'X');
    if(pX) {
        if(*(pX+1) == ':') valX = atoi(pX + 2); else valX = atoi(pX + 1);
        SetServoX(valX);
    }

    char *pY = strchr(cmd, 'Y');
    if(pY) {
        if(*(pY+1) == ':') valY = atoi(pY + 2); else valY = atoi(pY + 1);
        SetServoY(valY);
    }
}

void Setup_System(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    GPIOPinConfigure(GPIO_PD1_M1PWM1);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPWMClockSet(PWM_DIVIDER);

    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, PWM_PERIOD_TICKS);

    PWMOutputInvert(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, SERVO_X_MID);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, SERVO_Y_MID);

    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), UART_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UARTIntRegister(UART0_BASE, UART0IntHandler);
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    IntMasterEnable();
}

void UART0IntHandler(void) {
    uint32_t status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, status);
    while(UARTCharsAvail(UART0_BASE)) {
        char c = UARTCharGetNonBlocking(UART0_BASE);
        if(c == '\n') { g_rxBuff[g_rxIdx] = 0; g_dataReady = true; }
        else if(g_rxIdx < 63) { g_rxBuff[g_rxIdx++] = c; }
    }
}
