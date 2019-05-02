#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
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
    SPI1_init();
    LCD_init();
    LCD_clearScreen(ILI9341_MAGENTA);
    __builtin_enable_interrupts();
    int pressed = 0;
    int i = 0;
    float fps;
    char m[100];
    unsigned short rawx,rawy,rawz;
    unsigned int x,y;
    //draw I++
    LCD_drawBar_LR(110,90,ILI9341_WHITE,ILI9341_BLACK,25,0);
    LCD_drawBar_LR(110,95,ILI9341_WHITE,ILI9341_BLACK,25,0);
    LCD_drawBar_LR(110,100,ILI9341_WHITE,ILI9341_BLACK,25,0);
    LCD_drawBar_LR(110,105,ILI9341_WHITE,ILI9341_BLACK,25,0);
    LCD_drawBar_LR(110,110,ILI9341_WHITE,ILI9341_BLACK,25,0);
    sprintf(m,"I++");
    LCD_print(m,115,98,ILI9341_WHITE,ILI9341_BLACK);
    
    //draw I--
    LCD_drawBar_LR(110,150,ILI9341_WHITE,ILI9341_BLACK,25,0);
    LCD_drawBar_LR(110,155,ILI9341_WHITE,ILI9341_BLACK,25,0);
    LCD_drawBar_LR(110,160,ILI9341_WHITE,ILI9341_BLACK,25,0);
    LCD_drawBar_LR(110,165,ILI9341_WHITE,ILI9341_BLACK,25,0);
    LCD_drawBar_LR(110,170,ILI9341_WHITE,ILI9341_BLACK,25,0);
    sprintf(m,"I--");
    LCD_print(m,115,158,ILI9341_WHITE,ILI9341_BLACK);
    
    sprintf(m,"I = %d",i);
    LCD_print(m,110,125,ILI9341_WHITE,ILI9341_BLACK);
    while(1) {
        _CP0_SET_COUNT(0);
        rawx = XPT2046_read('x');
        sprintf(m,"rawx: %d     ",rawx);
        LCD_print(m,10,20,ILI9341_WHITE,ILI9341_BLACK);
        rawy = XPT2046_read('y'); 
        sprintf(m,"rawy: %d     ",rawy);
        LCD_print(m,10,30,ILI9341_WHITE,ILI9341_BLACK);
        rawz = XPT2046_read('z');
        sprintf(m,"rawz: %d     ",rawz);
        LCD_print(m,10,40,ILI9341_WHITE,ILI9341_BLACK);
        if (rawz>3300){
            x = (rawx*0.266 - 26.666);
            sprintf(m,"x: %d   ",x);
            LCD_print(m,10,50,ILI9341_WHITE,ILI9341_BLACK);
            y = (rawy*-0.355 + 355.55);
            sprintf(m,"y: %d   ",y);
            LCD_print(m,10,60,ILI9341_WHITE,ILI9341_BLACK);
            pressed = 1;
            
        }else if (pressed == 1){
            pressed = 0;
            if (x>=100 & x<=145){
                if (y>=80 & y<=115){
                    i++;
                    sprintf(m,"I = %d  ",i);
                    LCD_print(m,110,125,ILI9341_WHITE,ILI9341_BLACK);
                }else if(y>= 140 & y <=175){
                    i--;
                    sprintf(m,"I = %d  ",i);
                    LCD_print(m,110,125,ILI9341_WHITE,ILI9341_BLACK);
                }
            }
            
        }
        
        fps = 24000000.0/_CP0_GET_COUNT();
        sprintf(m,"FPS: %5.2f   ",fps);
        LCD_print(m,10,10,ILI9341_WHITE,ILI9341_BLACK);
        
        
    }
}