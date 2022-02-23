/*

 Copyright (C) Ian Hogg

 Routines for CBUS event management - part of CBUS libraries for PIC 18F

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
	
 *************************************************************** * * * * * * * * * * * * * * *
 *	CBUS bootloader
 *
 *	Based on the Microchip botloader 'canio.asm' tho which full acknowledgement is made.
 *	Relevant information is contained in the Microchip Application note AN247
 *
 *   This part of the CBUS firmware is distributed under the Microchip
 *   license which requires that it is only used on Microchip hardware.
 *   See page 15 of <http:\\ww1.microchip.com/downloads/en/AppNotes/00247a.pdf>
 *   for details of the Microchip licensing.  This bootloader is distributed with
 *   the CBUS software as a "system library" as defined in section 1 of the GNU Public
 *   license and is not licensed under GNU or Creative Commons.
 *   You must conform to Microchip license terms in respect of the bootloader.
 *
 *
 * Basic Operation:
 * The following is a CAN bootloader designed for PIC18F microcontrollers
 * with built-in CAN such as the PIC18F458. The bootloader is designed to
 * be simple, small, flexible, and portable.
 * 
 * ;*
 * Commands:
 * Put commands received from source (Master --> Slave)
 * The count (DLC) can vary.
 *                   RXB0EIDH RXB0EIDL RXB0D0 RXB0D1 RXB0D2 RXB0D3 RXB0D4 RXB0D5 RXB0D6 RXB0D7
 * XXXXXXXXXXX 0 0 8 XXXXXXXX XXXXXX00 ADDRL  ADDRH  ADDRU  RESVD  CTLBT  SPCMD  CPDTL  CPDTH
 * XXXXXXXXXXX 0 0 8 XXXXXXXX XXXXXX01 DATA0  DATA1  DATA2  DATA3  DATA4  DATA5  DATA6  DATA7
 *
 *
 * ADDRL - Bits 0 to 7 of the memory pointer.
 * ADDRH - Bits 8 - 15 of the memory pointer.
 * ADDRU - Bits 16 - 23 of the memory pointer.
 * RESVD - Reserved for future use.
 * CTLBT - Control bits. See MODE_ defines
 * SPCMD - Special command. See CMD_ defines
 * CPDTL - Bits 0 - 7 of 2s complement checksum
 * CPDTH - Bits 8 - 15 of 2s complement checksum
 * DATAX - General data.
 *
 * Control bits:
 * MODE_WRT_UNLCK  1  -Set this to allow write and erase operations to memory.
 * MODE_ERASE_ONLY 2  -Set this to only erase Program Memory on a put command. Must be on 64-byte
 *	boundary.
 * MODE_AUTO_ERASE 4  -Set this to automatically erase Program Memory while writing data.
 * MODE_AUTO_INC   8  -Set this to automatically increment the pointer after writing.
 * MODE_ACK       10  -Set this to generate an acknowledge after a 'put' (PG Mode only)
 *
 * Special Commands:
 * CMD_NOP			0x00	Do nothing, no response
 * CMD_RESET		0x01	Issue a soft reset after setting last EEPROM data to 0x00, no response
 * CMD_RST_CHKSM 	0x02	Reset the checksum counter and error status flags, no response
 * CMD_CHK_RUN		0x03	Check the checksum. reply with ok or nok.
 * CMD_BOOT_TEST 	0x04	Just sends a BOOT response message frame back to verify boot mode.
 * 
 * Responses:
 * RESPONSE_NOK     0x00
 * RESPONSE_OK      0x01
 * RESPONSE_BOOT    0x02
 * 
 * The control register set contains a pointer, some control bits and special command registers.
 * Control
 *   <PG><CD><ADDRL><ADDRH><ADDRU><_RES_><CTLBT><SPCMD><CPDTL><CPDTH>
 * Data
 *   <PG><CD><DATA0><DATA1><DATA2><DATA3><DATA4><DATA5><DATA6><DATA7>
 *  PG bit:	Put = 0, Get = 1
 *  CD bit:	Control = 0, Data = 1
 * 
 * Typical sequence:
 * -> CBUS RQNPN
 * -> CBUS BOOTM
 * -> Control, 0x000000 Boot test (AUTO_INC, AUTO_ERASE, UNLOCK)
 * <- OK
 * -> Control, 0x000800 Reset checksum (AUTO_INC, AUTO_ERASE, UNLOCK)
 * -> Put Data, data bytes
 * -> Put Data, data bytes
 * -> Control, 0x300000  (AUTO_INC, AUTO_ERASE, UNLOCK)
 * -> Put Data, data bytes
 * -> Control, 0xF00000  (AUTO_INC, AUTO_ERASE, UNLOCK)
 * -> Put Data, data bytes
 * -> Control, 0x000000 check run (AUTO_INC, AUTO_ERASE, UNLOCK) checksum
 * 
 * The user program must have the following vectors. 
 *
 * User code reset vector  0x0800
 * User code HPINT vector	0x0808
 * user code LPINT vector	0x0818
 * This would normally be achieved using XC8 option --codeoffset=0x800
 *
 * Checksum is 16 bit addition of all programmable bytes.
 * User sends 2s complement of addition at end of program in command 0x03 (16 bits only)
 *	
 * History for this file:
 *  30/10/09    Mike Bolton     - Modified Bootloader.asm version of the Microchip code 
 *  27/01/22    Ian Hogg        - ported to XC8 from Bootloader.asm
 * 
 * 
 * Currently written for:
 *  XC8 compiler
 *     uses cbusdefs.h from cbusdefs 
 *     uses devincs.h from CBUSlib although this could probably be eliminated
 *     Must be compiled with options:
 *         XC8 linker -> MEMORY model -> ROM ranges 0-07FF
 *     application code packed with this bootloader must be compiled with options:
 *         XC8 global options -> Additional options --codeoffset=0x800 APP_START
 * 
 * This file used the following PIC peripherals:
 *  * ECAN
 * 
 */

#include <xc.h>
#include <stdint.h>
#include <hwsettings.h>
#include <main.h>
#include <bl_romops.h>

#define	PIC_HIGH_INT_VECT	0x0008	//HP interrupt vector redirect. Change if target is different
#define	PIC_LOW_INT_VECT	0x0018	//LP interrupt vector redirect. Change if target is different.
#define	PIC_RESET_VECT      0x0000	//start of bootloader

#define	APP_HIGH_INT_VECT	0x0808	//HP interrupt vector redirect. Change if target is different
#define	APP_LOW_INT_VECT	0x0818	//LP interrupt vector redirect. Change if target is different.
#define	APP_RESET_VECT      0x0800	//start of app


// Interrupt service routine vectors
#asm
    PSECT HiVector,class=CODE,delta=1,abs
    ORG PIC_HIGH_INT_VECT
    goto APP_HIGH_INT_VECT
            
    PSECT LoVector,class=CODE,delta=1,abs
    ORG PIC_LOW_INT_VECT
    goto APP_LOW_INT_VECT
#endasm


#define	CAN_CD_BIT      RXB0EIDLbits.RXB0EID0	//Received control  bit
#define	CANTX_CD_BIT	TXB0EIDLbits.TXB0EID0	//Transmit control/data select bit
#define	CAN_TXB0SIDH	0b10000000	//Transmitted ID for target node/ data select bit
#define	CAN_PG_BIT      RXB0EIDLbits.RXB0EID1	//Received PUT / GET

#define FLASH_BLOCK_MASK 0x3F
#define FLASH_BLOCK_SIZE 0x40
#define CLK_MHZ         16

            // Program memory < 0x300000 for PIC18F26K80
            // Config memory = 0x300000 for PIC18F26K80
            // EEPROM data = 0xF00000 for PIC18F26K80
#define PROGRAM_ADDRESSU    0x00
#define EEPROM_ADDRESSU     0xF0
#define CONFIG_ADDRESSU     0x30
#define PROGRAM_LOWER_ADDRESSH 0x08    // reserve area below this for bootloader

            // transmit header
#define	CAN_TXB0SIDH	0b10000000
#define	CAN_TXB0SIDL	0b00001000
#define	CAN_TXB0EIDH	0b00000000	
#define	CAN_TXB0EIDL	0b00000100
            // receive filter
#define	CAN_RXF0SIDH	0b00000000
#define	CAN_RXF0SIDL	0b00001000
#define	CAN_RXF0EIDH	0b00000000
#define	CAN_RXF0EIDL	0b00000111
            // receive masks
#define	CAN_RXM0SIDH	0b11111111
#define	CAN_RXM0SIDL	0b11101011
#define	CAN_RXM0EIDH	0b11111111
#define	CAN_RXM0EIDL	0b11111000

#define CAN_BRGCON1  (CLK_MHZ/4)-1  // Work out BRGCON value from clock MHz    
#define	CAN_BRGCON2	0b10011110
#define	CAN_BRGCON3	0b00000011
#define	CAN_CIOCON	0b00100000	// CAN I/O control

// Bootloader protocol defines
#define	MODE_WRT_UNLCK	 0x01       // Unlock write and erase
#define	MODE_ERASE_ONLY	 0x02       // Erase without write
#define	MODE_AUTO_ERASE	 0x04       // Enable auto erase before write
#define	MODE_AUTO_INC	 0x08       // Enable auto inc the address
#define	MODE_ACK		 0x10       // Acknowledge mode
            
            // response codes
#define RESPONSE_NOK    0x00
#define RESPONSE_OK     0x01
#define RESPONSE_BOOT   0x02

#define READ_BYTES_QTY     8

            // commands
#define	CMD_NOP			0x00	
#define	CMD_RESET		0x01	
#define	CMD_RST_CHKSM	0x02	
#define	CMD_CHK_RUN		0x03
#define CMD_BOOT_TEST 	0x04	
            
            // error codes
#define NO_ERROR        0x00
#define VERIFY_ERROR    0x01
#define	ADDRESS_ERROR   0x02       // Tried to write to an invalid address
            
typedef union {
    struct {
        unsigned char l;
        unsigned char h;
        unsigned char u;
    } ;
    uint24_t triple;
} Integer24;

//
// Checksums
//
typedef union {
    struct {
        unsigned char l;
        unsigned char h;
    } ;
    uint16_t word;
} Integer16;


void canInit(void);
void canSendMessage(void);


typedef struct ControlFrame {
    Integer24 bootAddress;		// Address info	
    unsigned char _unused0;		//(Reserved)
    unsigned char bootControlBits;	// Boot Mode Control bits
    unsigned char bootSpecialCommand;  // Special boot commands
    Integer16 bootPCChecksum	;	// Chksum byte fromPC	
} ControlFrame;
ControlFrame controlFrame;   // To keep a copy of the control frame
unsigned char * controlFramePtr;

typedef struct DataFrame {
    unsigned char data[8];
}DataFrame;

unsigned char frameLength;		
Integer16 ourChecksum;	//16 bit checksum we're calculating
unsigned char w;    // general purpose
volatile unsigned char * bufferPtr;
unsigned char addrL;
unsigned char addrH;

//unsigned char writtenEeprom;

/* TODOs
 Holding down PB on power up
 * overwrite self check
 */
    
// make sure cbus.h is not included before here
void main(void) {
//    writtenEeprom=0;
    // first check if the bootflag is set and go to the application if clear
    EEADR = 0xFF;
    EEADRH = 0xFF;
    // Turn off analogue
    ANCON0 = 0;
    ANCON1 = 0;
    // Set bit rate as some apps use this to work out clock MHz
    clkMHz = CLK_MHZ;

    /*
     * The CAN baud rate pre-scaler is preset by the bootloader, so this code is written to be clock speed independent.
     * The global variable clkMHz is set by reading in CAN prescaler value from BRGCON or CiCFG for use by any other routines
     * that need to know the clock speed. Note that on the PIC32 the Peripheral bus prescaler setting will also need to be taken
     * into accounts when configuring time related values for peripherals that use pbclk.
     */
    BRGCON1 = CAN_BRGCON1;  // Work out BRGCON value from clock MHz
  
    // FLIM_SW == 0 when pressed
    if ((ee_read() == 0) && (FLiM_SW)) {  // read last byte of EEPROM and check FLiM switch
#asm
        goto APP_RESET_VECT ;
#endasm
    }
    // by the time we get here we know the EEBOOT flag is set 
    controlFrame.bootSpecialCommand = CMD_NOP;
    controlFrame.bootControlBits = MODE_AUTO_ERASE | MODE_AUTO_INC | MODE_ACK;
    
    // Both LEDs on
    TRIS_LED1Y = 0; // Output
    TRIS_LED2G = 0; // Output
    
    LED1Y = LED_ON;
    LED2G = LED_ON;
    
    //Initialise the CAN peripheral ready for bootloading
    canInit();
    //Initialise the rom routines
    initRomops();
    
    ourChecksum.word = 0;
    errorStatus = NO_ERROR;

    
    // enter bootloading loop awaiting command
    while (1) {
        CLRWDT();   // ensure watchdog is cleared whilst waiting
        
        RXB0CONbits.RXFUL = 0;
        // Wait for CAN frame matching filter
        while(RXB0CONbits.RXFUL == 0);
        
        frameLength = RXB0DLC & 0x0F;
        
        // We have received a frame
        // check CD bit first  *  CD bit:	Control = 0, Data = 1
        if (CAN_CD_BIT) {
            /******************
             * Data frame
             ******************/

            // get the address. Load both the EE and Flash/CONFIG addresses.
            TBLPTRU = controlFrame.bootAddress.u & 0xF0;
            TBLPTRH = controlFrame.bootAddress.h;
            TBLPTRL = controlFrame.bootAddress.l;
            EEADRH = TBLPTRH;
            EEADR = TBLPTRL;
            
            // check if we need to auto increment ready for next data packet
            if (controlFrame.bootControlBits & MODE_AUTO_INC) {
                controlFrame.bootAddress.triple += frameLength;
            }
            
            // Now work out what type of memory we are accessing based upon TBLPTRU
            if (TBLPTRU == PROGRAM_ADDRESSU) {    // Will need to be changed if Flash > 64K
                // Program flash address
                if (CAN_PG_BIT) {
                    // read
                    for (w=0; w<= READ_BYTES_QTY; w++) {
                        EECON1 = 0x80;  //Flash program space
                        ((DataFrame *)(&TXB0D0))->data[w] = readFlashByte();
                        TBLPTRL++;
                        if (TBLPTRL==0) {
                            TBLPTRH++;
                            if (TBLPTRH==0) {
                                TBLPTRU++;
                            }
                        }
                    }
                    TXB0DLC = READ_BYTES_QTY;
                    canSendMessage();
                } else {
                    // write
                    if (TBLPTRH >= PROGRAM_LOWER_ADDRESSH) {
                        /* The erase is done as part of flushFlash() */
                        if ( controlFrame.bootControlBits & MODE_ERASE_ONLY) {
                            eraseFlash();
                        } else {
                            // do write
                            for (w=0; w<frameLength; w++) {
                                writeFlashByte(((DataFrame*)&RXB0D0)->data[w]);
                                ourChecksum.word += (((DataFrame*)&RXB0D0)->data[w]);
                                TBLPTRL++;
                                if (TBLPTRL==0) {
                                    TBLPTRH++;
                                    if (TBLPTRH==0) {
                                        TBLPTRU++;
                                    }
                                }
                                /* Verify is done within flushFlash */
                            }
                        } 
                    } else {
                        errorStatus = ADDRESS_ERROR;
                    }
                }
            }
            
            
            if (TBLPTRU == CONFIG_ADDRESSU) {
                // Config address
                if (CAN_PG_BIT) {
                    // read
                    // Note if we need more space this could be merged with read PGM
                    for (w=0; w<= READ_BYTES_QTY; w++) {
                        EECON1 = 0xC0;  //Flash configuration space
                        ((DataFrame *)(&TXB0D0))->data[w] = readFlashByte();
                        TBLPTRL++;
                        if (TBLPTRL==0) {
                             TBLPTRH++;
                        }
                    }
                    TXB0DLC = READ_BYTES_QTY;
                    canSendMessage();
                } else {
                    // write
                    for (w=0; w<frameLength; w++) {
                        // no need to erase
                        writeConfigByte(((DataFrame*)&RXB0D0)->data[w]);
                        ourChecksum.word += (((DataFrame*)&RXB0D0)->data[w]);
                        TBLPTRL++;
                        if (TBLPTRL==0) {
                             TBLPTRH++;
                        }
                    }
                }
            }
            if (TBLPTRU == EEPROM_ADDRESSU) {
                // EE address
                if (CAN_PG_BIT) {
                    // read
                    for (w=0; w<= READ_BYTES_QTY; w++) {
                        ((DataFrame *)(&TXB0D0))->data[w] = ee_read();
                        EEADR++;
                        if (EEADR == 0) EEADRH++;
                    }
                    TXB0DLC = READ_BYTES_QTY;
                    canSendMessage();
                } else {
                    // do write
                    for (w=0; w<frameLength; w++) {
                        ee_write(((DataFrame*)&RXB0D0)->data[w]);
                        ourChecksum.word += (((DataFrame*)&RXB0D0)->data[w]);
                        EEADR++;
                        if (EEADR == 0) EEADRH++;
                    }
//writtenEeprom=1;
                }
            }
            
            // check whether we have to send an ACK
            if ((! CAN_PG_BIT) && (controlFrame.bootControlBits & MODE_ACK)) {
                // send an ack
                TXB0D0 = errorStatus ? RESPONSE_NOK : RESPONSE_OK;
                TXB0DLC = 1;
                canSendMessage();
            }
            
            
        } else {
            /******************
             * Control  frame
             ******************/
            //Copy the specified address and info
            controlFramePtr = (unsigned char*)&controlFrame;
            bufferPtr = &RXB0D0;
            for (w=0; w<frameLength; w++) {
                *controlFramePtr = *bufferPtr;
                controlFramePtr++;
                bufferPtr++;
            }
 //if (writtenEeprom) {
//     writtenEeprom=0;
 //}           
        
            /********************************************************* 
             * This is the NOP command. No need to do anything.
             */
            //if (CMD_NOP) {
            //    
            //}

        
            /********************************************************* 
             * This is the reset command. Used to run the application.
             */
            if (controlFrame.bootSpecialCommand == CMD_RESET) {
                flushFlash();
               // Clear the boot flag and enter the application
                EEADR = 0xFF;
                EEADRH = 0xFF;
                ee_write(0);
                LED1Y = LED_OFF;
                LED2G = LED_OFF;
                RESET();
            }
        
            /*********************************************************
             * This is the Reset checksum command. This routine 
             * resets the internal checksum registers and error status. 
             */
            if (controlFrame.bootSpecialCommand == CMD_RST_CHKSM) {
                ourChecksum.word = 0;
                errorStatus = NO_ERROR;
            }
        
            /************************************************************
             *  This is the Test and Run command. The checksum is
             * verified, and the self-write verification bit is checked. 
             * If both pass then OK is sent otherwise a NOK is sent.
             */
            if (controlFrame.bootSpecialCommand == CMD_CHK_RUN) {
                flushFlash();
                if ((ourChecksum.word + controlFrame.bootPCChecksum.word != 0) || (errorStatus)) {
                    TXB0D0 = RESPONSE_NOK;
                    TXB0DLC = 1;
                    canSendMessage();
                } else {
                    TXB0D0 = RESPONSE_OK;
                    TXB0DLC = 1;
                    canSendMessage();
                }
            }
            
            
            /************************************************************
             * test module is in BOOT mode
             */
            if (controlFrame.bootSpecialCommand == CMD_BOOT_TEST) {
                TXB0D0 = RESPONSE_BOOT;
                TXB0DLC = 1;
                canSendMessage();
            }

        }
    }
}



void canSendMessage() {
     // wait for TXREQ bit to be clear
    while (TXB0CONbits.TXREQ) {
        ;
    }

	TXB0SIDH = CAN_TXB0SIDH; 
	TXB0SIDL = CAN_TXB0SIDL; 
	TXB0EIDH = CAN_TXB0EIDH;
	TXB0EIDL = CAN_TXB0EIDL;
    
	CANTX_CD_BIT = 1;
	if (CAN_CD_BIT) {
        CANTX_CD_BIT = 0;
    }
    // request that the message is sent
	TXB0CONbits.TXREQ = 1;
}

void canInit(void) {

    IPR5 = 0;    // CAN interrupts priority

    // Put module into Configuration mode.
    CANCON = 0b10000000;
    // Wait for config mode
    while (CANSTATbits.OPMODE2 == 0);

    ECANCON   = 0;          // ECAN legacy mode with no FIFOs
 
    /*  The CAN bit rates used for CBUS are calculated as follows:
     *
     * Sync segment is fixed at 1 Tq
     * We are using propogation time of 7tq, phase 1 of 4Tq and phase2 of 4Tq.
     * Total bit time is Sync + prop + phase 1 + phase 2
     * or 16 * Tq in our case
     * So, for 125kbits/s, bit time = 8us, we need Tq = 500ns
     * To get 500nS, we set the CAN bit rate prescaler, in BRGCON1, to half the FOsc clock rate.
     * For example, 16MHz oscillator using PLL, Fosc is 64MHz, Tosc is 15.625nS, so we use prescaler of 1:32 to give Tq of 500nS  (15.625 x 32)
     * Having set Tq to 500nS, all other CAN timings are relative to Tq, so do not need changing with processor clock speed
     */
    BRGCON2 = CAN_BRGCON2; // freely programmable, sample once, phase 1 = 4xTq, prop time = 7xTq
    BRGCON3 = CAN_BRGCON3; // Wake-up enabled, wake-up filter not used, phase 2 = 4xTq
  
    CIOCON    = CAN_CIOCON;    // TX drives Vdd when recessive, CAN capture to CCP1 disabled
          
    //Initialise CAN registers
    // Set filter 0
	RXF0SIDH = CAN_RXF0SIDH; 	
	RXF0SIDL = CAN_RXF0SIDL; 
	// Prevent filter 1 from causing a receive event
    RXF1SIDL =	0xF3;		
	RXF0EIDH = CAN_RXF0EIDH;	
	RXF0EIDL = CAN_RXF0EIDL;
    // Set mask
	RXM0SIDH = CAN_RXM0SIDH;	
	RXM0SIDL = CAN_RXM0SIDL;	
	RXM0EIDH = CAN_RXM0EIDH;			
	RXM0EIDL = CAN_RXM0EIDL;			
    // Note - BRGCON1 initialised first above
	BRGCON2 = CAN_BRGCON2;     		
	BRGCON3 = CAN_BRGCON3;			

    BIE0 = 0;                 // No Rx buffer interrupts
    CANCON = 0;               // Set normal operation mode
}




