/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include <stdio.h>
#include "app.h"
#include "i2c_master_noint.h"
#include "ili9341.h"
#define SLAVE_ADR 0b1101011
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
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
    char m[100];
    
    sprintf(m,"WHOAMI: %d",I2C_read_WHOAMI());
    LCD_print(m,20,10,ILI9341_WHITE,ILI9341_BLACK);
    LCD_drawBar_UD(118,160,ILI9341_WHITE,ILI9341_BLACK,100,0);
    LCD_drawBar_UD(118,60,ILI9341_WHITE,ILI9341_BLACK,100,0);
    LCD_drawBar_LR(120,158,ILI9341_WHITE,ILI9341_BLACK,100,0);
    LCD_drawBar_LR(20,158,ILI9341_WHITE,ILI9341_BLACK,100,0);
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
        char m[100];
        char data[14];
        short temp,gyroX,gyroY,gyroZ,accelX,accelY,accelZ,absX,absY;
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
            
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
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

/*******************************************************************************
 End of File
 */
