#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ili9341.h"
#include <stdio.h>

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock x24 after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock by 2 to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module


void __ISR(_TIMER_3_VECTOR, IPL5SOFT) Timer3ISR(void) {
static int inc = 24;
static int counter = 0;
IFS0bits.T3IF = 0;
OC3RS = OC3RS + inc;

if (counter == 99){ // direction pin check
    counter = 0;
    inc = -inc;
    LATBbits.LATB13 = !LATBbits.LATB13; // toggle LED
} else{
    counter++;
}
// how many times has the interrupt occurred?

// set the duty cycle and direction pin

}

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
    ANSELA = 0;
    ANSELB = 0;
    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0; // sets pin A4 to output
    LATAbits.LATA4 = 0; // sets pin A4 to high
    TRISBbits.TRISB4 = 1; // sets pin B4 to input
    TRISBbits.TRISB13 = 0; // sets pin B13 to output
    SPI1_init();
    LCD_init();
    RPB9Rbits.RPB9R = 0b0101; // sets pin B9 to OC3
    T2CONbits.TCKPS = 0; // Timer2 prescaler N=1 (1:1)
    T3CONbits.TCKPS = 3; // prescaler = 8
    PR2 = 2399; // PR = PBCLK / N / desiredF - 1
    PR3 = 59999;
    OC3CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
    OC3RS = 0; // duty cycle
    OC3R = 0; // initialize before turning OC1 on; afterward it is read-only
    IPC3bits.T3IP = 5;            // step 4: interrupt priority 
    IPC3bits.T3IS = 0;            // step 4: interrupt subpriority 0
    IFS0bits.T3IF = 0;            // step 5: clear the flag
    IEC0bits.T3IE = 1;            // step 6: enable T2 by setting IEC0
  
    TMR3 = 0;                // initial TMR count is 0
    TMR2 = 0;
    T2CONbits.ON = 1; // turn on Timer2
    T3CONbits.ON = 1; // turn on Timer3
    OC3CONbits.ON = 1; // turn on OC3
    __builtin_enable_interrupts();
    LCD_clearScreen(ILI9341_MAGENTA);
    unsigned char red[240], green[240], blue[240];
    int i = 0;
    srand(0);
    while (i<240){
        red[i] = rand();
        green[i]=rand();
        blue[i]=rand();
        i++;
    }
    for (i=0;i<239;i++){
            LCD_drawPixel(i,20+((char)(red[i]/36)),ILI9341_RED); // draw red line
            LCD_drawPixel(i,40+((char)(green[i]/36)),ILI9341_GREEN);
            LCD_drawPixel(i,60+((char)(blue[i]/36)),ILI9341_BLUE);
        }
    while(1) {
        ;
    }
}