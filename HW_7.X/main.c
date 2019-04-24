#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ili9341.h"
#include "i2c_master_noint.h"
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

#define SLAVE_ADR 0b1101011

void I2C_read_multiple(unsigned char * data, int length);
void init_IMU(void);
char I2C_read_WHOAMI(void);


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
    i2c_master_setup();
    SPI1_init();
    init_IMU();
    LCD_init();
    LCD_clearScreen(ILI9341_MAGENTA);
    __builtin_enable_interrupts();
    char m[100];
    char data[14];
    short temp,gyroX,gyroY,gyroZ,accelX,accelY,accelZ,absX,absY;
    sprintf(m,"WHOAMI: %d",I2C_read_WHOAMI());
    LCD_print(m,20,10,ILI9341_WHITE,ILI9341_BLACK);
    LCD_drawBar_UD(118,160,ILI9341_WHITE,ILI9341_BLACK,100,0);
    LCD_drawBar_UD(118,60,ILI9341_WHITE,ILI9341_BLACK,100,0);
    LCD_drawBar_LR(120,158,ILI9341_WHITE,ILI9341_BLACK,100,0);
    LCD_drawBar_LR(20,158,ILI9341_WHITE,ILI9341_BLACK,100,0);
    while(1) {
        LATAbits.LATA4 = !LATAbits.LATA4;
        
        I2C_read_multiple(data,14);
        temp =(data[1]<<8)|data[0];
        gyroX =(data[3]<<8)|data[2];
        gyroY =(data[5]<<8)|data[4];
        gyroZ =(data[7]<<8)|data[6];
        accelX =(data[9]<<8)|data[8];
        accelY =(data[11]<<8)|data[10];
        accelZ =(data[13]<<8)|data[12];
        if(accelX>0){
            absX = (short)((accelX/15000.0)*100);
            LCD_drawBar_LR(120,158,ILI9341_WHITE,ILI9341_BLACK,100,0);
            LCD_drawBar_LR(20,158,ILI9341_BLACK,ILI9341_WHITE,100,100-absX);
        }else if(accelX<=0){
            absX = (short)((accelX/15000.0)*-100);
            LCD_drawBar_LR(20,158,ILI9341_WHITE,ILI9341_BLACK,100,0);
            LCD_drawBar_LR(120,158,ILI9341_WHITE,ILI9341_BLACK,100,absX);
        }
        if(accelY>0){
            absY = (short)((accelY/15000.0)*100);
            LCD_drawBar_UD(118,160,ILI9341_WHITE,ILI9341_BLACK,100,0);
            LCD_drawBar_UD(118,60,ILI9341_BLACK,ILI9341_WHITE,100,100-absY);
        }else if(accelY<=0){
            absY = (short)((accelY/15000.0)*-100);
            LCD_drawBar_UD(118,60,ILI9341_WHITE,ILI9341_BLACK,100,0);
            LCD_drawBar_UD(118,160,ILI9341_WHITE,ILI9341_BLACK,100,absY);
        }
        sprintf(m,"AX: %d     ",accelX);
        LCD_print(m,20,20,ILI9341_WHITE,ILI9341_BLACK);
        sprintf(m,"AY: %d     ",accelY);
        LCD_print(m,20,30,ILI9341_WHITE,ILI9341_BLACK);
        _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT()<1200000){;} // delay for 20 Hz cycle
    }
}


void init_IMU(void){
    // configure accelerometer
    i2c_master_start();
    i2c_master_send(SLAVE_ADR<<1|0); //device w/ write command
    i2c_master_send(0x10); // send address of SFR (CTRL1_XL)
    i2c_master_send(0b10000010); //sets to 1.66 kHz, 2g, 100 Hz filter
    i2c_master_stop();
    // configure gyroscope
    i2c_master_start();
    i2c_master_send(SLAVE_ADR<<1|0); //device w/ write command
    i2c_master_send(0x11); // send address of SFR (CTRL2_G)
    i2c_master_send(0b10001000); //sets to 1.66 kHz, 1000 dps sensitivity
    i2c_master_stop();
    // turn on increment
    i2c_master_start();
    i2c_master_send(SLAVE_ADR<<1|0); //device w/ write command
    i2c_master_send(0x12); // send address of SFR (CTRL3_C)
    i2c_master_send(0b00000100); //turns on auto increment
    i2c_master_stop();
    
}
void I2C_read_multiple(unsigned char * data, int length){
    int i;
    i2c_master_start();
    i2c_master_send(SLAVE_ADR<<1|0); // write first
    i2c_master_send(0x20); // do OUT_TEMP_L
    i2c_master_restart(); // restart for read
    i2c_master_send(SLAVE_ADR<<1|1); // read device
    for(i=0;i<length;i++){
        data[i] = i2c_master_recv(); 
        if (i < length-1){ // before last read, keep asking for data
            i2c_master_ack(0); // read more
        } else if (i==length-1){ // last read we say we have enough data
            i2c_master_ack(1); // send acknowledgment
        }
    }
    i2c_master_stop();
    
}

char I2C_read_WHOAMI(void){
    i2c_master_start();
    i2c_master_send(SLAVE_ADR<<1|0); // write first
    i2c_master_send(0x0F); // WHOAMI
    i2c_master_restart(); // restart for read
    i2c_master_send(SLAVE_ADR<<1|1); // read device
    char r = i2c_master_recv(); 
    i2c_master_ack(1); // send acknowledgement
    i2c_master_stop();
    return r;
}