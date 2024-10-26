/*

 Copyright (C) Pete Brownlow 2014-2017   software@upsys.co.uk


 This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                   and indicate if changes were made. You may do so in any reasonable manner,
                   but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                  your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                  legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

**************************************************************************************************************
  Note:   This source code has been written using a tab stop and indentation setting
          of 4 characters. To see everything lined up correctly, please set your
          IDE or text editor to the same settings.
******************************************************************************************************

*/
#ifndef HWSETTINGS_H
#define	HWSETTINGS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "devincs.h"
#include "GenericTypeDefs.h"
#include "cbusdefs.h"

// Control bit definitions

#if defined( __18F25K80) || defined(__18F26K80)
    #define INTEN               INTCONbits.GIEH
#endif

    /*
     * Module specific hardware definitions. Clock speeds and LED ports.
     */
// FLiM Pushbutton and status LEDs
/*
 * These definitions are required by the FLiM library code
 */

//#pragma message ("HARDWARE    IS " HARDWARE)

#if HARDWARE==MTYP_CANPanel
#define SetPortDirections(){WPUA=0b00101000;TRISBbits.TRISB6=TRISBbits.TRISB7=0,TRISAbits.TRISA3=1;}
#define FLiM_SW         PORTAbits.RA3
#define LED1Y           LATBbits.LATB6  // Yellow LED
#define LED2G           LATBbits.LATB7  // Green LED
#define TRIS_LED1Y      TRISBbits.TRISB6
#define TRIS_LED2G      TRISBbits.TRISB7

#define LED_ON          1               // LEDs are active high
#define LED_OFF         0
#elif HARDWARE==MTYP_CANMIO
#define SetPortDirections(){TRISBbits.TRISB6=TRISBbits.TRISB7=0,TRISAbits.TRISA2=1;}
#define FLiM_SW         PORTAbits.RA2
#define LED1Y           LATBbits.LATB6  // Yellow LED
#define LED2G           LATBbits.LATB7  // Green LED
#define TRIS_LED1Y      TRISBbits.TRISB6
#define TRIS_LED2G      TRISBbits.TRISB7
    
#define LED_ON          1               // LEDs are active high
#define LED_OFF         0
#else
#error You must define a HARDWARE setting in the MPLABX XC8 Defines configuration
#endif


// Macros for clock frequencies

extern BYTE clkMHz;

#define GetSystemClock()               (clkMHz * 1000000ul)

#define GetInstructionClock()          (GetSystemClock()/4)     // For PIC18 there are 4 clock cycles per instruction
#define GetPeripheralClock()           (GetSystemClock()/4)     // For PIC18 peripheral clock is same as instruction clock

#define GetSystemMHz()                 clkMHz
#define GetInstructionMHz()            clkMHz

#if defined( __18CXX)
#define ei()    INTCONbits.GIEH = 1;INTCONbits.GIEL = 1
#define di()    INTCONbits.GIEH = 0;INTCONbits.GIEL = 0
#endif

#define geti()  INTCONbits.GIE


// Global routine definitions

void setclkMHz( void );



#ifdef	__cplusplus
}
#endif

#endif	/* HWSETTINGS_H */


