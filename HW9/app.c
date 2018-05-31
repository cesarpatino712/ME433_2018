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
#include <stdio.h>
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"
#include"ST7735.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

uint8_t APP_MAKE_BUFFER_DMA_READY dataOut[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
int len, i = 0;
int startTime = 0; // to remember the loop time

//LSsMDS33 SLAVE ADDRESS
#define I2C_ADDR 0b1101011
//LSM6DS33 Registers
#define CTR1_XL 0b10000010                 //linear acceleration sensor 1.66kHz, 2g, 100Hz
#define CTRL2_G 0b10001000                 //gyroscope 1.66kHz, 1000 dps
#define CTRL3_C 0b00000100                 //IF_INC bit enabled for sequential read
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
/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
        USB_DEVICE_CDC_INDEX index,
        USB_DEVICE_CDC_EVENT event,
        void * pData,
        uintptr_t userData
        ) {
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *) userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch (event) {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof (USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *) pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *) pData)->breakDuration;

            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context) {
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch (event) {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuratio. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*) eventData;
            if (configuredEventData->configurationValue == 1) {
                /* Update LED to show configured state */

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t) & appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void) {
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if (appData.isConfigured == false) {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    } else {
        retVal = false;
    }

    return (retVal);
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

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    //appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];

    /* PUT YOUR LCD, IMU, AND PIN INITIALIZATIONS HERE */
    TRISBbits.TRISB4=1;
    TRISAbits.TRISA4=0;
    LATAbits.LATA4=1;
    
    initIMU();
    LCD_init();

    startTime = _CP0_GET_COUNT();
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    /* Update the application state machine based
     * on the current state */

    switch (appData.state) {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE);

            if (appData.deviceHandle != USB_DEVICE_HANDLE_INVALID) {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            } else {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if (appData.isConfigured) {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if (APP_StateReset()) {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if (appData.isReadComplete == true) {
                appData.isReadComplete = false;
                appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read(USB_DEVICE_CDC_INDEX_0,
                        &appData.readTransferHandle, appData.readBuffer,
                        APP_READ_BUFFER_SIZE);
                

                        /* AT THIS POINT, appData.readBuffer[0] CONTAINS A LETTER
                        THAT WAS SENT FROM THE COMPUTER */
                        /* YOU COULD PUT AN IF STATEMENT HERE TO DETERMINE WHICH LETTER
                        WAS RECEIVED (USUALLY IT IS THE NULL CHARACTER BECAUSE NOTHING WAS
                      TYPED) */
                

                if (appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID) {
                    appData.state = APP_STATE_ERROR;
                    break;
                }
            }

            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
        case APP_STATE_CHECK_TIMER:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

             /* WAIT FOR 5HZ TO PASS OR UNTIL A LETTER IS RECEIVED */
            if (appData.isReadComplete && appData.readBuffer[0] == 'r'){ // || _CP0_GET_COUNT() - startTime > (48000000 / 2 / 100)) {
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }


            break;


        case APP_STATE_SCHEDULE_WRITE:

            if (APP_StateReset()) {
                break;
            }

            /* Setup the write */

            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;

            /* PUT THE TEXT YOU WANT TO SEND TO THE COMPUTER IN dataOut
            AND REMEMBER THE NUMBER OF CHARACTERS IN len */
            /* THIS IS WHERE YOU CAN READ YOUR IMU, PRINT TO THE LCD, ETC */
            char message[30];
            //sprintf(message,"Hello World");
            LCD_clearScreen(BLACK);
            unsigned char data [13];
           

            
          
            _CP0_SET_COUNT(0);
            LATAINV = 0x10;                  //toggle LATA4 LED on
            
            for (i = 0;i <=  100; i++) {
         
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
            while(_CP0_GET_COUNT()<24000000/100){
           
            }
            //write accelerometer data to computer
            len = sprintf(dataOut, "%d\t%d\t%d\t%d\t%d\t%d\r\n", OUTX_G, OUTY_G, OUTZ_G,OUTX_XL, OUTY_XL, OUTZ_XL);
           
            //i++; // increment the index so we see a change in the text
            /* IF A LETTER WAS RECEIVED, ECHO IT BACK SO THE USER CAN SEE IT */
            /*if (appData.isReadComplete) {
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle,
                        appData.readBuffer, 1,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
            }
            /* ELSE SEND THE MESSAGE YOU WANTED TO SEND */
           //else {
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle, dataOut, len,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                startTime = _CP0_GET_COUNT(); // reset the timer for accurate delays
            //}
               
            }
            
          appData.isWriteComplete = true;
                
                
            break;
            
    
        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if (APP_StateReset()) {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if (appData.isWriteComplete == true) {
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}
