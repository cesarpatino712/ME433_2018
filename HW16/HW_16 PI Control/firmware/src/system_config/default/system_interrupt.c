/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "system/common/sys_common.h"
#include "app.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

 
void __ISR(_USB_1_VECTOR, ipl4AUTO) _IntHandlerUSBInstance0(void)
{
    DRV_USBFS_Tasks_ISR(sysObj.drvUSBObject);
}

void __ISR(_TIMER_4_VECTOR, IPL4SOFT) Timer4ISR(void) {
  // code for PI control goes here
    //input reference velocity(from android)
    ref_vel5 = 1;                  //step function
    ref_vel3 = 2;
    //motor: 7 pulses per revolution; 100 gear ratio = 700 pulses/rev
    //velocity [rev/s] = encoder pulse # * GR / time : pulse #  = TMR, GR = 100, time = 1/500 HZ;
    vel5 = TMR5 * GearRatio / 500;
    vel3 = TMR3 * GearRatio / 500;
    //calculate the error
    e5 = ref_vel5 - vel5;
    e3 = ref_vel3 - vel3;
    //multiply velocity error by controller parameters (start with P controller)
    U5 = Kp * e5;
    U5new = U5 + 50.0;
    if (U5new > 100.0){
        U5new = 100.0;
    } else if (U5new < 0.0){
        U5new = 0.0;
    }
    OC1RS = (unsigned int) ((U5new/100.0) * PR2);
    
    U3 = Kp * e3;
    U3new = U3 + 50.0;
    if (U3new > 100.0){
        U3new = 100.0;
    } else if (U3new < 0.0){
        U3new = 0.0;
    }
    OC4RS = (unsigned int) ((U3new/100.0) * PR2);
    
    //error sum for integral
    esum5 = esum5 + e5;
    esum3 = esum3 + e3;
    
   //reset the encoder timers to 0 
    TMR5 = 0;
    TMR3 = 0;
    
    
    //

  IFS0bits.T4IF = 0; // clear interrupt flag, last line
}
/*******************************************************************************
 End of File
*/
