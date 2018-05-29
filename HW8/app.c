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

#include "app.h"

#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<stdio.h>  
#include "i2c_master_noint.h"
#include"ST7735.h"
//LSsMDS33 SLAVE ADDRESS
#define I2C_ADDR 0b1101011
//LSM6DS33 Registers
#define CTR1_XL 0b10000010                 //linear acceleration sensor 1.66kHz, 2g, 100Hz
#define CTRL2_G 0b10001000                 //gyroscope 1.66kHz, 1000 dps
#define CTRL3_C 0b00000100                 //IF_INC bit enabled for sequential read
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
void LCD_init(void);
void I2C_read_multiple(unsigned char SLAVEaddr, unsigned char addr, unsigned char * data, int length);
unsigned char getIMU(unsigned char SLAVEaddr, unsigned char addr);
void setIMU(unsigned char SLAVEaddr, unsigned char addr, unsigned char DIN);
void initIMU(void);
void drawChar(char x, char y, char message, short colorON, short colorOFF);
void drawString(char x, char y, char* message, short colorOn, short colorOFF);
void drawProgressBar(char x, char y,int length, short colorON, short colorOFF);

void I2C_read_multiple(unsigned char SLAVEaddr, unsigned char addr, unsigned char * data, int length){
    //i2C read sequence S -> OP -> W -> ADDR -> SR -> OP -> R -> DOUT -> P
    // S = start; OP = device opcode; W =  write; ADDR = register adress -> SR = restart; R = read; DOUT =data out; P = stop
    //device opcode: 0 1 0 0 A2 A1 A0 R/W; A2 A1 A0 = address, R/W read = 1  write = 0;
   
    int i = 0;
    i2c_master_start();                                     //start
    i2c_master_send(SLAVEaddr<<1);            //write 
    i2c_master_send(addr);                  //register address 
    i2c_master_restart();                                //restart
    i2c_master_send((SLAVEaddr<<1)|1);            //read
    for (i = 0; i < length; i++){
        data[i] = i2c_master_recv();
        if (i < length - 1){
            i2c_master_ack(0);              //keep reading
        }
        else{
            i2c_master_ack(1);
        }
    }

    i2c_master_stop();
    
}


unsigned char getIMU(unsigned char SLAVEaddr, unsigned char addr){
    //i2C read sequence S -> OP -> W -> ADDR -> SR -> OP -> R -> DOUT -> P
    // S = start; OP = device opcode; W =  write; ADDR = register adress -> SR = restart; R = read; DOUT =data out; P = stop
    //device opcode: 0 1 0 0 A2 A1 A0 R/W; A2 A1 A0 = address, R/W read = 1  write = 0;
    i2c_master_start();                                     //start
    i2c_master_send(SLAVEaddr<<1);            //write 
    i2c_master_send(addr);                  //register address 
    i2c_master_restart();                                //restart
    i2c_master_send((SLAVEaddr<<1)|1);            //read
    unsigned char DOUT = i2c_master_recv();                              //receive data
    i2c_master_ack(1);
    i2c_master_stop();
    return DOUT;
    
}

void setIMU(unsigned char SLAVEaddr, unsigned char addr, unsigned char DIN){
    i2c_master_start();
    i2c_master_send(SLAVEaddr<<1);           //write 
    i2c_master_send(addr);                 //register address                              
    i2c_master_send(DIN);            //write
    i2c_master_stop();
}
  void initIMU(void){
    //clear analog input pins
    ANSELBbits.ANSB2 = 0;           //SCL
    ANSELBbits.ANSB3 = 0;           //SDA
    i2c_master_setup();             //BRG = 53 for 400KHZ
    //set registers
    setIMU(I2C_ADDR, 0x10, CTR1_XL);
    setIMU(I2C_ADDR, 0x11, CTRL2_G);
    setIMU(I2C_ADDR, 0x12, CTRL3_C);
       
}

void drawChar(char x, char y, char message, short colorON, short colorOFF){
    int col = 0;
    int j = 0;
    char pixels;
    
    for(col = 0; col < 5; col++){
        pixels = ASCII[message - 0x20][col];
        for(j = 0; j < 8; j++){
            if ((pixels >> j) & 1){
                LCD_drawPixel(x + col, y + j, colorON);
            }
            else {
                LCD_drawPixel(x + col, y + j, colorOFF);
            }
        }
    }
}

void drawString(char x, char y, char* message, short colorOn, short colorOFF){
    int i = 0;
    while(message[i]){
        drawChar(x + 5 * i, y, message[i], colorOn, colorOFF);
        i++;
    }
}

void drawProgressBar(char x, char y,int length, short colorON, short colorOFF){
    int i = 0;
    int j = 0;
    for(i = 0; i < length; i++){
        LCD_drawPixel(x + i, y, colorON);
        for (j = x+ i; j < x + length; j++){
        LCD_drawPixel(j - i, y, colorOFF);
        }
    }
}

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
    
    // do your TRIS and LAT commands here
    TRISBbits.TRISB4=1;
    TRISAbits.TRISA4=0;
    LATAbits.LATA4=1;
    
    initIMU();
    LCD_init();

    
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
             char message[30];
    //sprintf(message,"Hello World");
    LCD_clearScreen(BLACK);
    unsigned char data [13];
    signed short scale = 16500;

    while(1) {
        //if (PORTBbits.RB4 == 1){        //push button is not pressed
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk
        //start the timer at 0
        _CP0_SET_COUNT(0);
        LATAINV = 0x10;                  //toggle LATA4 LED on
       
         
        I2C_read_multiple(I2C_ADDR, 0x20, data, 14);
        
        //reconstruct signed short by shifting high byte and ORing with the low byte
        //angular rate sensor pitch axis (x)
        //Gyroscope readings
        signed short OUTX_G = ((data[3] << 8)| data[2]);
        sprintf(message,"OUTX_G = %d",OUTX_G);
        drawString(28, 12, message, RED, BLACK);
        signed short OUTY_G = ((data[5] << 8)| data[4]);
        sprintf(message,"OUTY_G = %d",OUTY_G);
        drawString(28, 22, message, RED, BLACK);
        signed short OUTZ_G = ((data[7] << 8)| data[6]);
        sprintf(message,"OUTZ_G = %d",OUTZ_G);
        drawString(28, 32, message, RED, BLACK);
        //Accelerometer Readings
        signed short OUTX_XL = ((data[9] << 8)| data[8]);
        sprintf(message,"OUTX_XL = %d",OUTX_XL);
        drawString(28, 42, message, CYAN, BLACK);
        signed short OUTY_XL = ((data[11] << 8)| data[10]);
        sprintf(message,"OUTY_XL = %d",OUTY_XL);
        drawString(28, 52, message, CYAN, BLACK);
        signed short OUTZ_XL = ((data[13] << 8)| data[12]);
        sprintf(message,"OUTZ_XL = %d",OUTZ_XL);
        drawString(28, 62, message, CYAN, BLACK);
        
        
        
        // 20 Hz delay
        while(_CP0_GET_COUNT()<24000000/20){
           
        }
        
        
        
    }
    return 0;
          
        
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

 

/*******************************************************************************
 End of File
 */
