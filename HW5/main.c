#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"

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
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 00000000 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF// allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

void delay(void);

int main() {
    // some initialization function to set the right speed setting
    char buf[100] = {};                       // buffer for sending messages to the user
    unsigned char master_write0 = 0xCD;       // first byte that master writes
    unsigned char master_write1 = 0x91;       // second byte that master writes
    unsigned char master_read0  = 0x00;       // first received byte
    unsigned char master_read1  = 0x00;       // second received byte

    // some initialization function to set the right speed setting
    Startup(); 
    __builtin_disable_interrupts();
    
    i2c_master_setup();                       // init I2C2, which we use as a master
    
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISBbits.TRISB4 = 1;           // tristate register: determines if port is input = 1 or output = 0
    TRISAbits.TRISA4 = 0;           //LED port A is an output
    LATAbits.LATA4 = 0;             //LED off
    //initialize pins
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();             //BRG = 53 for 400KHZ
    
    
    
    __builtin_enable_interrupts();
    
   

    while(1) {
        if (PORTBbits.RB4 == 1){        //push button is not pressed
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk
        //start the timer at 0
        _CP0_SET_COUNT(0);
        //LATAbits.LATA4 = 0;
        LATAINV = 0x10;                  //toggle LATA4 LED on
        // leave on for 0.5 ms, 48 MHz/2 * 0.5 ms = 12,000 counts
        while(_CP0_GET_COUNT()<12000){
            ;
        }
        }
        else {
            LATAbits.LATA4 = 0;         //if button is pressed LED is off
        }
        
        //i2C2
        i2c_master_start();                     // Begin the start sequence
        i2c_master_send(SLAVE_ADDR << 1);       // send the slave address, left shifted by 1, 
                                            // which clears bit 0, indicating a write
        i2c_master_send(master_write0);         // send a byte to the slave       
        i2c_master_send(master_write1);         // send another byte to the slave
        i2c_master_restart();                   // send a RESTART so we can begin reading 
        i2c_master_send((SLAVE_ADDR << 1) | 1); // send slave address, left shifted by 1,
                                            // and then a 1 in lsb, indicating read
        master_read0 = i2c_master_recv();       // receive a byte from the bus
        i2c_master_ack(0);                      // send ACK (0): master wants another byte!
        master_read1 = i2c_master_recv();       // receive another byte from the bus
        i2c_master_ack(1);                      // send NACK (1):  master needs no more bytes
        i2c_master_stop();                      // send STOP:  end transmission, give up bus

    }
            
}
    
    
 

       
               
    
