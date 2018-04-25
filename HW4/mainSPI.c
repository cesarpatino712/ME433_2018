#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>        //input math library

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

//SPI Pins
//A1 -> SDI (pin 3 to pin 5)
//B14 -> SCK (pin 25 to pin 4)
//B8 -> CS (pin 17 to pin 3)

#define CS LATAbits.LATA1       // chip select pin

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void DAC_init() {
  // set up the chip select pin as an output
  // the chip select pin is used by the DAC to indicate
  // when a command is beginning (clear CS to low) and when it
  // is ending (set CS high)
    
  // Set peripheral pins according to output pin selection 
  TRISAbits.TRISA0 = 0;             //set A1 (CS) as output
  TRISAbits.TRISA1 = 0;             //set A1 as output
  RPA1Rbits.RPA1R = 0b0011;         //set pin as SDO1
  CS = 1;

  // Master - SPI1, pins are: SDO1(A1), SCK1(B14).  
  // we manually control SS1 as a digital output (F12)
  // since the pic is just starting, we know that spi is off. We rely on defaults here
 
  // setup spi1
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 3;            // baud rate to 6 MHz [SPI4BRG = (48000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1

                            // send a ram set status command.
  CS = 0;                   // enable the DAC
  spi_io(0x01);             // ram write status
  spi_io(0x41);             // sequential mode (mode = 0b01), hold disabled (hold = 0)
  CS = 1;                   // finish the command
}

void set_voltages(int chAB, char V){
    unsigned short t;
    //chAB = channel select 1 = write to DAC_B, 0 = write to DAC_A
    //V = voltage value (10 bits) for the MCP4912 DAC
    // bit designations: Bit 15: A/B_select, BUF, GA, SHDN, D9, D8, D7, D6, D5, D4, D3, D2, D1 D0, x, x
    //A/B  = channel select, BUF = Vref input buffer control, GA = gain select,  SHDN = shutdown, D9-0 = data, x =  unused
    t = chAB << 15;                             //select the channel to write to
    t = t | 0b0111000000000000;                 //BUF = 1 (buffered), GA = 1 (x1),SHDN enable
    t = t | ((V & 0b111111111) <<2);            //set the DATA bits with the voltage values
    //send DAC command
    CS = 0;
    spi_io(t>>8);                               //MSB
    spi_io(t);                                  //LSB
    CS = 1;
}

#define PI 3.14159

int main() {

    __builtin_disable_interrupts();

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

    __builtin_enable_interrupts();
    
    DAC_init();
    
    
   //initialize parameters
     //initialize parameters
    unsigned int V_sin = 0;
    unsigned int V_triangle = 0;
    float time = 0;
    float slope = 2.56;         // slope = 0.25 Vmax / #iters in 0.1 sec; 256/100 

    while(1) {
       
        int iter = 0;
        
        for (iter = 0; iter <= 100; iter ++){
             //start the timer at 0
            _CP0_SET_COUNT(0);
            time = time + 0.001;              //ms timestep for 1 KHz sampling frequency
            //triangle 0 to Vmax/4: 0 - 256
            //max V = 1024
            V_triangle = int(V_triangle + slope);
            set_voltages(1,V_triangle);
            //sine wave
            //DC offset Vmax/2 = 512; f = 10Hz, Amp = 1/4 Vmax = 256
            V_sin = 512 + 256 * sin(2*PI*10*time);
            set_voltages(0,V_sin);
            //delay
            while(_CP0_GET_COUNT()<6000){               //1 KHz sampling freq: 1 ms period: 6 MHz * 1 ms = 6000 count
            ;
        }
        }
        slope = slope * -1;
    
    }
    
  }
