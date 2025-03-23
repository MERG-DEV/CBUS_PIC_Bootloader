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
******************************************************************************************************/

 /**
 * @file
 * CBUS bootloader
 *
 *	Based on the Microchip bootloader 'canio.asm' tho which full acknowledgement is made.
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
 * @details
 * Basic Operation:
 * The following is a CAN bootloader designed for PIC18F microcontrollers
 * with built-in CAN such as the PIC18F458. The bootloader is designed to
 * be simple, small, flexible, and portable. The bootloader resides within the 
 * bottom 2K bytes (0x0 ~ 0x07FF) of Flash memory. The processor's reset vector
 * points to the start of the bootloader. Upon reset the bootloader checks the
 * top address of EEPROM and if this is zero control is then passed to the user
 * application at address 0x800.
 * The user application should be compiled with option --CODEOFFSET=0x800 so that 
 * the compiler places its reset and interrupt vectors starting at 0x800.
 * If the top EEPROM address is non zero then control continues within the 
 * bootloader which awaits commands from the CAN interface.
 * The bootloader uses 29bit extended ids to carry the command and control
 * information.
 * 
 * 
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
 *  27/01/22    Ian Hogg        - ported to XC8 from Bootloader.asm for PIC18F26K80
 *  03/04/24    Ian Hogg        - updated to support PIC18F27Q83
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
 *  * CAN
 * 
 */

#include <xc.h>
#include <stdint.h>
#include <hwsettings.h>
#include <main.h>
#include "vlcbdefs.h"

#define BL_VERSION  2
const char bl_version[] = { 'B','L','_','V','E','R','S','I','O','N','=', BL_TYPE_IanHogg, BL_VERSION};

// #define STATS        // Uncomment to collect stats of numbers of each type of message received
#define SELF_VERIFY     // uncomment to read back data to ensure it has written ok

#include <bl_romops.h>

#define	PIC_HIGH_INT_VECT	0x0008	//HP interrupt vector redirect. Change if target is different
#define	PIC_LOW_INT_VECT	0x0018	//LP interrupt vector redirect. Change if target is different.
#define	PIC_RESET_VECT      0x0000	//start of bootloader

#define	APP_HIGH_INT_VECT	0x0808	//HP interrupt vector redirect. Change if target is different
#define	APP_LOW_INT_VECT	0x0818	//LP interrupt vector redirect. Change if target is different.
#define	APP_RESET_VECT      0x0800	//start of app

/**
 * Even though this bootloader doesn't use interrupts we must ensure that the
 * interrupts are available in case the application wants to use them.
 * Here we redirect the interrupt vectors through to the start of App +0x0008 
 * and +0x0018 so that they reside at 0x0808 and 0x0818.
 */
// Interrupt service routine vectors
//#asm
    asm("PSECT HiVector,class=CODE,delta=1,abs");
    asm("ORG "  ___mkstr(PIC_HIGH_INT_VECT) );
    asm("goto "  ___mkstr(APP_HIGH_INT_VECT) );
            
    asm("PSECT LoVector,class=CODE,delta=1,abs");
    asm("ORG "  ___mkstr(PIC_LOW_INT_VECT) );
    asm("goto "  ___mkstr(APP_LOW_INT_VECT) );
//#endasm

#define CLK_MHZ         16
    
#if defined(_18F66K80_FAMILY_)
#define	CAN_CD_BIT      RXB0EIDLbits.RXB0EID0	//Received control  bit
#define	CANTX_CD_BIT	TXB0EIDLbits.TXB0EID0	//Transmit control/data select bit
#define	CAN_PG_BIT      RXB0EIDLbits.RXB0EID1	//Received PUT / GET

#define FLASH_BLOCK_MASK 0x3F
#define FLASH_BLOCK_SIZE 0x40

            // Program memory < 0x00 0000 for PIC18F26K80
            // Config memory = 0x30 0000 for PIC18F26K80
            // EEPROM data = 0xF0 0000 for PIC18F26K80
#define PROGRAM_ADDRESSU    (unsigned char)0x00
#define EEPROM_ADDRESSU     (unsigned char)0xF0
#define CONFIG_ADDRESSU     (unsigned char)0x30
#define ADDRESSU_TYPE_MASK  (unsigned char)0xF0
#define PROGRAM_LOWER_ADDRESSH (unsigned char)0x08    // reserve area below this for bootloader
#endif
#if defined(_18FXXQ83_FAMILY_)
#define	CAN_CD_BIT      (rxFifoObj[RX_EIDL]&0x08)	//Received control  bit
#define	CANTX_CD_BIT	(txFifoObj[TX_EIDL]&0x08)	//Transmit control/data select bit
#define	CAN_SIDH	0b10000000	//Transmitted ID for target node/ data select bit
#define	CAN_PG_BIT      (rxFifoObj[RX_EIDL]&0x10)	//Received PUT / GET

#define FLASH_BLOCK_MASK 0xFF
#define FLASH_BLOCK_SIZE 0x100
            // Program memory < 0x00 0000 for PIC18F26K80
            // Config memory = 0x30 0000 for PIC18F26K80
            // EEPROM data = 0xF0 0000 for PIC18F26K80
#define PROGRAM_ADDRESSU    (unsigned char)0x01
#define EEPROM_ADDRESSU     (unsigned char)0x38
#define CONFIG_ADDRESSU     (unsigned char)0x30
#define ADDRESSU_TYPE_MASK  (unsigned char)0xFF
#define PROGRAM_LOWER_ADDRESSH (unsigned char)0x08    // reserve area below this for bootloader
    
#define CAN1_BUFFERS_BASE_ADDRESS           0x3800
// Transmit FIFO
#define CAN1_FIFO2_BUFFERS_BASE_ADDRESS     CAN1_BUFFERS_BASE_ADDRESS
#define CAN1_FIFO2_PAYLOAD_SIZE     16
#define CAN1_FIFO2_SIZE             32
// Receive FIFO
#define CAN1_FIFO3_BUFFERS_BASE_ADDRESS     (CAN1_FIFO2_BUFFERS_BASE_ADDRESS+(CAN1_FIFO2_PAYLOAD_SIZE*CAN1_FIFO2_SIZE))
#define CAN1_FIFO3_PAYLOAD_SIZE     16
#define CAN1_FIFO3_SIZE             32
    
uint8_t* rxFifoObj;
// RX Object offsets
#define RX_SIDL 0
#define RX_EIDL 1
#define RX_EIDH 2
#define RX_EIDU 3
#define RX_DLC  4
#define RX_D0   8
#define RX_D1   9
#define RX_D2   10
#define RX_D3   11
#define RX_D4   12
#define RX_D5   13
#define RX_D6   14
#define RX_D7   15

uint8_t* txFifoObj;
// TX Object offsets
#define TX_SIDL 0
#define TX_EIDL 1
#define TX_EIDH 2
#define TX_EIDU 3
#define TX_DLC  4
#define TX_D0   8
#define TX_D1   9
#define TX_D2   10
#define TX_D3   11
#define TX_D4   12
#define TX_D5   13
#define TX_D6   14
#define TX_D7   15

#define TX_IDE  0x10

/*******************************
 * These copied from MCC generated code
 ********************************/
/**
 @ingroup  can_driver
 @enum     CAN_OP_MODES
 @brief    Defines the CAN operation modes that are available for the module to use.
*/
enum CAN_OP_MODES
{
    CAN_NORMAL_FD_MODE = 0x0,           /**< CAN FD Normal Operation Mode (Supported only in CAN FD mode) */
    CAN_DISABLE_MODE = 0x1,             /**< CAN Disable Operation Mode */               
    CAN_INTERNAL_LOOPBACK_MODE = 0x2,   /**< CAN Internal Loopback Operation Mode */
    CAN_LISTEN_ONLY_MODE = 0x3,         /**< CAN Listen only Operation Mode */
    CAN_CONFIGURATION_MODE = 0x4,       /**< CAN Configuration Operation Mode */
    CAN_EXTERNAL_LOOPBACK_MODE = 0x5,   /**< CAN External Loopback Operation Mode */
    CAN_NORMAL_2_0_MODE = 0x6,          /**< CAN 2.0 Operation Mode */
    CAN_RESTRICTED_OPERATION_MODE =0x7  /**< CAN Restricted Operation Mode */
}; 

/**
 @ingroup  can_driver
 @enum     CAN_OP_MODE_STATUS
 @brief    Defines the return status of CAN operation mode set API.
*/
enum CAN_OP_MODE_STATUS
{
    CAN_OP_MODE_REQUEST_SUCCESS,     /**< The requested operation mode set successfully */
    CAN_OP_MODE_REQUEST_FAIL,        /**< The requested operation mode set failure */
    CAN_OP_MODE_SYS_ERROR_OCCURED    /**< The system error occurred while setting operation mode. */
};


/**
 * @ingroup can_driver
 * @brief Sets the CAN1 Operation mode.
 * @pre CAN1_Initialize() function is already called.
 * @param [in] requestMode - CAN1 Operation mode as described in CAN_OP_MODES.
 */
void CAN1_OperationModeSet(const enum CAN_OP_MODES requestMode);

/**
 * @ingroup can_driver
 * @brief Gets the CAN1 Operation mode.
 * @pre CAN1_Initialize() function is already called.
 * @param None.
 * @return The present CAN1 Operation mode as described in CAN_OP_MODES.
 */
enum CAN_OP_MODES CAN1_OperationModeGet(void);
#endif

            // transmit header
// Set up ID
// SID=0b100 00000000
// EID= 0b00 00000000 00000100
#if defined(_18F66K80_FAMILY_)
#define	CAN_SIDH	0b10000000  //SID=0
#define	CAN_SIDL	0b00001000  // IDE bit set for extended frame and EID top bit set
#define	CAN_EIDH	0b00000000	
#define	CAN_EIDL	0b00000100
#endif
#if defined(_18FXXQ83_FAMILY_)
#define	CAN_TXO0	0b00000000
#define	CAN_TXO1	0b00100100
#define	CAN_TXO2	0b00000000	
#define	CAN_TXO3	0b00000000
#endif
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
void canSendMessage(uint8_t cdBit);


typedef struct ControlFrame {
    Integer24 bootAddress;		// Address info	
    unsigned char _unused0;		//(Reserved)
    unsigned char bootControlBits;	// Boot Mode Control bits
    unsigned char bootSpecialCommand;  // Special boot commands
    Integer16 bootPCChecksum	;	// Chksum byte fromPC	
} ControlFrame;
/**
 * The Control Frame stores a copy of the last control frame received.
 * The bootAddress element provides the address of the start of the last 
 * requested section. The actual address being written or read is kept in the 
 * TBLPTR or NVMADR registers.
 */
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
unsigned long t1;   // timers used to 2 sec counter for when PB is held down

unsigned char t2;
#define TIMER_2SEC 0xFFFE

#ifdef STATS
unsigned char flashFrames;
unsigned char configFrames;
unsigned char controlFrames;
unsigned char eepromFrames;
unsigned char dataFrames;
unsigned char reset_chk_command;
#endif

/* TODOs
 Holding down PB on power up
 * overwrite self check
 */
    
// make sure cbus.h is not included before here
void main(void) {
    // Set bit rate as some apps use this to work out clock MHz
    clkMHz = CLK_MHZ;

    /*
     * The CAN baud rate pre-scaler is preset by the bootloader, so this code is written to be clock speed independent.
     * The global variable clkMHz is set by reading in CAN prescaler value from BRGCON or CiCFG for use by any other routines
     * that need to know the clock speed. Note that on the PIC32 the Peripheral bus prescaler setting will also need to be taken
     * into accounts when configuring time related values for peripherals that use pbclk.
     */
#if defined(_18F66K80_FAMILY_)
    BRGCON1 = CAN_BRGCON1;  // Work out BRGCON value from clock MHz to 3

    // Turn off analogue
    ANCON0 = 0;
    ANCON1 = 0;
    // initialise the CAN peripheral
#endif
#if defined(_18FXXQ83_FAMILY_)
    ANSELA = 0;         // Turn off analogue
    ANSELB = 0;         // Turn off analogue
#endif
    // initialise the push button and LED ports
    SetPortDirections();  // make input
    
#if defined(_18F66K80_FAMILY_)
    EEADR = 0xFF;
    EEADRH = 0xFF;
#endif
#if defined(_18FXXQ83_FAMILY_)
    //Load NVMADR with the desired byte address
    NVMADRU = 0x38;
    NVMADRH = 0x03;
    NVMADRL = 0xFF;
#endif
    // next check if the bootflag is set and go to the application if clear
    // FLIM_SW == 0 when pressed
    if ((ee_read() == 0) && (FLiM_SW)) {  // read last byte of EEPROM and check FLiM switch
//#asm
        asm("goto "  ___mkstr(APP_RESET_VECT) );
//#endasm
    }
    if (!(FLiM_SW)) {
        // PB is being held down. If released before 2 seconds then continue to 
        // bootloader. If held down for full 2 seconds go to the application
        // to perhaps do factory reset or enter test mode
        t1 = 0;
        while (!(FLiM_SW)) {
            if (t1 > TIMER_2SEC) {
                asm("goto "  ___mkstr(APP_RESET_VECT) );
            }
            t2=0;
            while (t2 < 0x07) {
                t2++;
            }
            t1++;
        }
    }
    
    // enable the 4x PLL
    clkMHz = 64;
#if defined(_18F66K80_FAMILY_)
    OSCTUNEbits.PLLEN = 1; 
    BRGCON1 = (clkMHz/4 -1);    // adjust the CAN prescalar to 15
#endif
#if defined(_18FXXQ83_FAMILY_)
    OSCCON1bits.NOSC = 2;
#endif
    
#ifdef STATS
    flashFrames = 0;
    configFrames = 0;
    controlFrames = 0;
    eepromFrames = 0;
    dataFrames = 0;
    reset_chk_command = 0;
#endif
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
#ifdef MODE_SELF_VERIFY
    errorStatus = NO_ERROR;
#endif
    
    /******************************************
     * enter bootloading loop awaiting command
     *****************************************/
    while (1) {
        CLRWDT();   // ensure watchdog is cleared whilst waiting
#if defined(_18F66K80_FAMILY_)        
        RXB0CONbits.RXFUL = 0;
        // Wait for CAN frame matching filter
        while(RXB0CONbits.RXFUL == 0)
            ;
        
        frameLength = RXB0DLC & 0x0F;
#endif
#if defined(_18FXXQ83_FAMILY_)
        while (! C1FIFOSTA3Lbits.TFNRFNIF)
            ;
        rxFifoObj = (uint8_t*) C1FIFOUA3;   // Pointer to FIFO entry
        frameLength = rxFifoObj[RX_DLC] & 0x0F;
#endif
        /****************************************
         * We have received a frame, decode it
         ***************************************/
        // check CD bit first  *  CD bit:	Control = 0, Data = 1
        if (CAN_CD_BIT) {
            /******************
             * Data frame
             ******************/
#ifdef STATS
            dataFrames++;
#endif
            // get the address. Load both the EE and Flash/CONFIG addresses.
#if defined(_18F66K80_FAMILY_) 
            TBLPTRU = controlFrame.bootAddress.u & 0x1F;
            TBLPTRH = controlFrame.bootAddress.h;
            TBLPTRL = controlFrame.bootAddress.l;
            EEADRH = TBLPTRH;
            EEADR = TBLPTRL;
#endif
#if defined(_18FXXQ83_FAMILY_)
            NVMADRU = controlFrame.bootAddress.u;
            NVMADRH = controlFrame.bootAddress.h;
            NVMADRL = controlFrame.bootAddress.l;
#endif
            
            // check if we need to auto increment ready for next data packet
            if (controlFrame.bootControlBits & MODE_AUTO_INC) {
                controlFrame.bootAddress.triple += frameLength;
            }
            
            // Now work out what type of memory we are accessing based upon top 4 bits of address
            // This also
            if ((controlFrame.bootAddress.u & ADDRESSU_TYPE_MASK) <= PROGRAM_ADDRESSU) { 
                /************** FLASH ***************/
#ifdef STATS
                flashFrames++;
#endif
                // Program flash address
                if (CAN_PG_BIT) {   
#if defined(_18F66K80_FAMILY_) 
                    // read flash memory
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
#endif
#if defined(_18FXXQ83_FAMILY_)
                    // read fash memory
                    // wait for buffer
                    while (! C1FIFOSTA2Lbits.TFNRFNIF)
                        ;
                    txFifoObj = (uint8_t*) C1FIFOUA2;
                    for (w=0; w<= READ_BYTES_QTY; w++) {
                        txFifoObj[TX_D0+w] = readFlashByte();
                        TBLPTRL++;
                        if (TBLPTRL==0) {
                            TBLPTRH++;
                            if (TBLPTRH==0) {
                                TBLPTRU++;
                            }
                        }
                    }
                    txFifoObj[TX_DLC] = TX_IDE | READ_BYTES_QTY;
#endif
                    canSendMessage(1);  // send with data
                } else {
                    // write flash memory
#if defined(_18F66K80_FAMILY_)
                    if (TBLPTRH >= PROGRAM_LOWER_ADDRESSH)
#endif
#if defined(_18FXXQ83_FAMILY_)
                    if ((NVMADRU > 0) || (NVMADRH >=PROGRAM_LOWER_ADDRESSH))
#endif
                    {
                        /* The erase is done as part of flushFlash() */
                        if ( controlFrame.bootControlBits & MODE_ERASE_ONLY) {
                            eraseFlash();
                        } else {
                            // do write
                            /* Writing of flash is done in blocks. The underlying 
                             * writeFlashByte function handles the handling of 
                             * the block buffer.
                             */
                            for (w=0; w<frameLength; w++) {
#if defined(_18F66K80_FAMILY_)
                                writeFlashByte(((DataFrame*)&RXB0D0)->data[w]);
                                ourChecksum.word += (((DataFrame*)&RXB0D0)->data[w]);
                                TBLPTRL++;  // increment pointer
                                if (TBLPTRL==0) {
                                    TBLPTRH++;
                                    if (TBLPTRH==0) {
                                        TBLPTRU++;
                                    }
                                }
#endif
#if defined(_18FXXQ83_FAMILY_)
                                writeFlashByte(rxFifoObj[RX_D0+w]);
                                ourChecksum.word += rxFifoObj[RX_D0+w];
                                NVMADRL++;  // increment pointer
                                if (NVMADRL==0) {
                                    NVMADRH++;
                                    if (NVMADRH==0) {
                                        NVMADRU++;
                                    }
                                }
#endif
                                /* Verify is done within flushFlash */
                            }
                        } 
                    }
#ifdef MODE_SELF_VERIFY
                    else {
                        errorStatus = ADDRESS_ERROR;
                    }
#endif
                }
            } 
            
            if ((controlFrame.bootAddress.u & ADDRESSU_TYPE_MASK) == CONFIG_ADDRESSU) {
                /************** CONFIG ***************/
#ifdef STATS
                configFrames++;
#endif
                // Config address
                if (CAN_PG_BIT) {
                    // read CONFIG
                    // Note if we need more space this could be merged with read PGM
#if defined(_18F66K80_FAMILY_)
                    for (w=0; w<= READ_BYTES_QTY; w++) {
                        EECON1 = 0xC0;  //Flash configuration space
                        ((DataFrame *)(&TXB0D0))->data[w] = readFlashByte();
                        TBLPTRL++;
                        if (TBLPTRL==0) {
                             TBLPTRH++;
                        }
                    }
                    TXB0DLC = READ_BYTES_QTY;
#endif
#if defined(_18FXXQ83_FAMILY_)
                    // wait for buffer
                    while (! C1FIFOSTA2Lbits.TFNRFNIF)
                        ;
                    txFifoObj = (uint8_t*) C1FIFOUA2;
                    for (w=0; w<= READ_BYTES_QTY; w++) {
                        txFifoObj[TX_D0+w] = readFlashByte();
                        TBLPTRL++;
                        if (TBLPTRL==0) {
                            TBLPTRH++;
                            if (TBLPTRH==0) {
                                TBLPTRU++;
                            }
                        }
                    }
                    txFifoObj[TX_DLC] = TX_IDE | READ_BYTES_QTY;
#endif
                    canSendMessage(1);  // send with data
                } else {
                    // write CONFIG
                    for (w=0; w<frameLength; w++) {
                        // no need to erase config bytes
                        CLRWDT();   // ensure watchdog is cleared whilst writing
                        
                        /* Cannot perform SELF_VERIFY for CONFIG as FCU fills in
                         * missing CONFIG values, such as 0x30004, with 0xFF but these are read back 
                         * as 0x00. This means the verify always fails.
                         */
#if defined(_18F66K80_FAMILY_)
                        // write config byte
                        writeConfigByte(((DataFrame*)&RXB0D0)->data[w]);
                        ourChecksum.word += (((DataFrame*)&RXB0D0)->data[w]);
                        TBLPTRL++;
                        if (TBLPTRL==0) {
                             TBLPTRH++;
                        }
#endif
#if defined(_18FXXQ83_FAMILY_)
                        // write config byte
                        writeConfigByte(rxFifoObj[RX_D0+w]);
                        ourChecksum.word += rxFifoObj[RX_D0+w];
                        TBLPTRL++;
                        if (TBLPTRL==0) {
                             TBLPTRH++;
                        }
#endif
                    }
                }
            }
            if ((controlFrame.bootAddress.u & ADDRESSU_TYPE_MASK) == EEPROM_ADDRESSU) {
                /************** EEPROM ***************/
#ifdef STATS
                eepromFrames++;
#endif
                // EE address
                if (CAN_PG_BIT) {
                    // read
#if defined(_18F66K80_FAMILY_)
                    for (w=0; w<= READ_BYTES_QTY; w++) {
                        ((DataFrame *)(&TXB0D0))->data[w] = ee_read();
                        EEADR++;
                        if (EEADR == 0) EEADRH++;
                    }
                    TXB0DLC = READ_BYTES_QTY;
#endif
#if defined(_18FXXQ83_FAMILY_)
                    while (! C1FIFOSTA2Lbits.TFNRFNIF)
                        ;
                    txFifoObj = (uint8_t*) C1FIFOUA2;
                    for (w=0; w<= READ_BYTES_QTY; w++) {
                        txFifoObj[TX_D0+w] = ee_read();
                        NVMADRL++;
                        if (NVMADRL==0) {
                             NVMADRH++;
                        }
                    }
                    txFifoObj[TX_DLC] = TX_IDE | READ_BYTES_QTY;
#endif
                    canSendMessage(1);  // send with data
                } else {
                    // do write
#if defined(_18F66K80_FAMILY_)
                    for (w=0; w<frameLength; w++) {
                        ee_write(((DataFrame*)&RXB0D0)->data[w]);
                        ourChecksum.word += (((DataFrame*)&RXB0D0)->data[w]);
                        EEADR++;
                        if (EEADR == 0) EEADRH++;
                    }
#endif
#if defined(_18FXXQ83_FAMILY_)
                    for (w=0; w<frameLength; w++) {
                        ee_write(rxFifoObj[RX_D0+w]);
                        ourChecksum.word += rxFifoObj[RX_D0+w];
                        NVMADRL++;
                        if (NVMADRL==0) {
                             NVMADRH++;
                        }
                    }
#endif
                }
            }
            
            // check whether we have to send an ACK
            if ((! CAN_PG_BIT) && (controlFrame.bootControlBits & MODE_ACK)) {
                // send an ack
#if defined(_18F66K80_FAMILY_)
#ifdef MODE_SELF_VERIFY
                TXB0D0 = errorStatus ? RESPONSE_NOK : RESPONSE_OK;
#else
                TXB0D0 = RESPONSE_OK;
#endif
                TXB0DLC = 1;
#endif
#if defined(_18FXXQ83_FAMILY_)
                while (! C1FIFOSTA2Lbits.TFNRFNIF)
                    ;
                txFifoObj = (uint8_t*) C1FIFOUA2;
#ifdef MODE_SELF_VERIFY
                txFifoObj[TX_D0] = errorStatus ? RESPONSE_NOK : RESPONSE_OK;
#else
                txFifoObj[TX_D0] = RESPONSE_OK;
#endif
                txFifoObj[TX_DLC] = TX_IDE | 1;
#endif
                canSendMessage(0);  // send with control
            }
            
        } else {
            /******************
             * Control  frame
             ******************/
#ifdef STATS
            controlFrames++;
#endif
            // we probably need to flush the Program Flash buffer. No harm done
            // if we don't need to do it.
            flushFlash();
            //Copy the control frame info to save the programming address
            controlFramePtr = (unsigned char*)&controlFrame;
#if defined(_18F66K80_FAMILY_)
            bufferPtr = &RXB0D0;
#endif
#if defined(_18FXXQ83_FAMILY_)
            bufferPtr = &(rxFifoObj[RX_D0]);
#endif
            for (w=0; w<frameLength; w++) {
                *controlFramePtr = *bufferPtr;
                controlFramePtr++;
                bufferPtr++;
            }
        
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
               // Clear the boot flag and enter the application
#if defined(_18F66K80_FAMILY_)
                EEADR = 0xFF;
                EEADRH = 0xFF;
#endif
#if defined(_18FXXQ83_FAMILY_)
                NVMADRU = 0x38;
                NVMADRH = 0x03;
                NVMADRL = 0xFF;
#endif
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
#ifdef STATS
                reset_chk_command++;
#endif
                ourChecksum.word = 0;
#ifdef MODE_SELF_VERIFY
                errorStatus = NO_ERROR;
#endif
            }
        
            /************************************************************
             * This is the Test and Run command. The checksum is
             * verified, and the self-write verification bit is checked. 
             * If both pass then OK is sent otherwise a NOK is sent.
             * This only does VERIFY and does NOT run the application.
             */
            if (controlFrame.bootSpecialCommand == CMD_CHK_RUN) {
#if defined(_18F66K80_FAMILY_)
#ifdef MODE_SELF_VERIFY
                TXB0D0 = (((ourChecksum.word + controlFrame.bootPCChecksum.word) != 0) || (errorStatus)) ? 
                    RESPONSE_NOK : RESPONSE_OK;
#else
                TXB0D0 = ((ourChecksum.word + controlFrame.bootPCChecksum.word) != 0) ? 
                    RESPONSE_NOK : RESPONSE_OK;
#endif
                TXB0DLC = 1;
#endif
#if defined(_18FXXQ83_FAMILY_)
                while (! C1FIFOSTA2Lbits.TFNRFNIF)
                    ;
                txFifoObj = (uint8_t*) C1FIFOUA2;
#ifdef MODE_SELF_VERIFY
                txFifoObj[TX_D0] = (((ourChecksum.word + controlFrame.bootPCChecksum.word) != 0) || (errorStatus)) ? 
                    RESPONSE_NOK : RESPONSE_OK;
#else
                txFifoObj[TX_D0] = ((ourChecksum.word + controlFrame.bootPCChecksum.word) != 0) ? 
                    RESPONSE_NOK : RESPONSE_OK;
#endif
                txFifoObj[TX_DLC] = TX_IDE | 1;
#endif
                canSendMessage(0);  // send with control
            }
            
            
            /************************************************************
             * test module is in BOOT mode
             */
            if (controlFrame.bootSpecialCommand == CMD_BOOT_TEST) {
                // A Good response :X80180004N02;
#if defined(_18F66K80_FAMILY_)
                TXB0D0 = RESPONSE_BOOT;
                TXB0DLC = 1;
#endif
#if defined(_18FXXQ83_FAMILY_)
                while (! C1FIFOSTA2Lbits.TFNRFNIF)
                    ;
                txFifoObj = (uint8_t*) C1FIFOUA2;
                txFifoObj[TX_D0] = RESPONSE_BOOT;
                txFifoObj[TX_DLC] = TX_IDE | 1;
#endif
                canSendMessage(0);  // send with control
            }

        }
#if defined(_18FXXQ83_FAMILY_)
        // ready for next frame
        C1FIFOCON3Hbits.UINC = 1;   // Indicate that we have got the message from FIFO
#endif
    }
}



void canSendMessage(uint8_t cdBit) {
#if defined(_18F66K80_FAMILY_)
     // wait for TXREQ bit to be clear
    while (TXB0CONbits.TXREQ) {
        ;
    }
	TXB0SIDH = CAN_SIDH; 
	TXB0SIDL = CAN_SIDL; 
	TXB0EIDH = CAN_EIDH;
	TXB0EIDL = CAN_EIDL;
    // set the CD in EID[0] bit correctly
	if (cdBit) {
        CANTX_CD_BIT = 1;
    } else {
        CANTX_CD_BIT = 0;
    }
#endif
#if defined(_18FXXQ83_FAMILY_)
    txFifoObj[0] = CAN_TXO0;
    txFifoObj[1] = CAN_TXO1;
    txFifoObj[2] = CAN_TXO2;
    txFifoObj[3] = CAN_TXO3;
    // Set the CD bit in EID correctly
	if (cdBit) {
        txFifoObj[1] |= 0x08;
    } else {
        txFifoObj[1] &= 0xF7;
    }
#endif
	
    // request that the message is sent
#if defined(_18F66K80_FAMILY_)
	TXB0CONbits.TXREQ = 1;
#endif
#if defined(_18FXXQ83_FAMILY_)
    C1FIFOCON2H |= (_C1FIFOCON2H_TXREQ_MASK | _C1FIFOCON2H_UINC_MASK);
#endif
}

#if defined(_18F66K80_FAMILY_)
void canInit(void) {
    IPR5 = 0;    // CAN interrupts priority

    // Put module into Configuration mode.
    CANCON = 0b10000000;
    // Wait for config mode
    while (CANSTATbits.OPMODE2 == 0)
        ;

    ECANCON = 0;          // ECAN legacy mode with no FIFOs
 
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
    // enable filter 0
    RXFCON0 = 1;
    // Note - BRGCON1 initialised first above
	BRGCON2 = CAN_BRGCON2;     		
	BRGCON3 = CAN_BRGCON3;			

    BIE0 = 0;                 // No Rx buffer interrupts
    CANCON = 0;               // Set normal operation mode
}
#endif
#if defined(_18FXXQ83_FAMILY_)
void canInit(void)
{
    // initialise the CAN peripheral
    RB2PPS = 0x46;      // CANTX
    CANRXPPS = 013 ;    // octal 1=PORTB 3 = port B3
    TRISBbits.TRISB2 = 0;  // CAN TX output
    TRISBbits.TRISB3 = 1;  // CAN RX input
    IPR5 = 0;    // CAN interrupts priority
    /* Enable the CAN module */
    C1CONHbits.ON = 1;
    
    // Put module into Configuration mode.
    CAN1_OperationModeSet(CAN_CONFIGURATION_MODE);
        
    /* Initialize the C1FIFOBA with the start address of the CAN FIFO message object area. */
    C1FIFOBA = CAN1_BUFFERS_BASE_ADDRESS;

    C1CONL = 0x00;      // CLKSEL0 disabled; DeviceNet filter disabled
    C1CONH = 0x87;      // ON enabled; SIDL disabled; BUSY disabled; WFT T11 Filter; WAKFIL enabled;
    C1CONU = 0x10;      // TXQEN disabled; STEF disabled; SERRLOM disabled; RTXAT disabled;
    C1CONT = 0x50;      // TXBWS=5; ABAT=0; REQOP=0

    C1NBTCFGL = 0x00;   // SJW 1;
    C1NBTCFGH = 0x03;   // TSEG2 4;
    C1NBTCFGU = 0x02;   // TSEG1 3;
    C1NBTCFGT = 0x3F;   // BRP 15;


    /*
     * FIFO2 used for transmit
     * FIFO3 used for receive
     */

    // Normal TX FIFO
    C1FIFOCON2L = 0x80; // TXEN enabled; RTREN disabled; RXTSEN disabled; TXATIE disabled; RXOVIE disabled; TFERFFIE disabled; TFHRFHIE disabled; TFNRFNIE disabled;
    C1FIFOCON2H = 0x04; // FRESET enabled; TXREQ disabled; UINC disabled;
    C1FIFOCON2U = 0x60; // TXAT unlimited retransmission attempts; TXPRI 0 (low);
    C1FIFOCON2T = (((CAN1_FIFO2_PAYLOAD_SIZE<32) ? (CAN1_FIFO2_PAYLOAD_SIZE/4)-2 :
                        (CAN1_FIFO2_PAYLOAD_SIZE==32) ? 5 :
                                                (CAN1_FIFO2_PAYLOAD_SIZE/16)+3) << 5) | (CAN1_FIFO2_SIZE-1);// PLSIZE 8; FSIZE 32;

    // Normal RX FIFO
    C1FIFOCON3L = 0x08; // TXEN disabled; RTREN disabled; RXTSEN disabled; TXATIE disabled; RXOVIE disabled; TFERFFIE disabled; TFHRFHIE disabled; TFNRFNIE disabled;
    C1FIFOCON3H = 0x04; // FRESET enabled; TXREQ disabled; UINC disabled;
    C1FIFOCON3U = 0x00; // TXAT retransmission disabled; TXPRI 1;
    C1FIFOCON3T = (((CAN1_FIFO3_PAYLOAD_SIZE<32) ? (CAN1_FIFO3_PAYLOAD_SIZE/4)-2 :
                        (CAN1_FIFO3_PAYLOAD_SIZE==32) ? 5 :
                                                (CAN1_FIFO3_PAYLOAD_SIZE/16)+3) << 5) | (CAN1_FIFO3_SIZE-1); // PLSIZE 8; FSIZE 32;

    // Filter 0 for All Extended messages
    //C1FLTOBJ0L = 0b00000000;
    //C1FLTOBJ0H = 0b00000000;
    //C1FLTOBJ0U = 0b00000000;
    C1FLTOBJ0T = 0b01000000;    // EXIDE set: allow extended ID only
    // Set up a mask for just extended messages
    //C1MASK0L = 0b00000000;
    //C1MASK0H = 0b00000000;
    //C1MASK0U = 0b00000000;
    C1MASK0T = 0b01000000;      // MIDE set: filter on EXIDE

    C1FLTCON0L = 0x83;  // FLTEN0 enabled; F1BP FIFO 3 - the normal RX FIFO
        
    /* Place CAN1 module in Normal Operation mode */
    CAN1_OperationModeSet(CAN_NORMAL_2_0_MODE);    
}
    /************************************
 * Section copied from MCC generated code
 *************************************/
void CAN1_OperationModeSet(const enum CAN_OP_MODES requestMode)
{
    C1CONTbits.REQOP = requestMode;
    while (C1CONUbits.OPMOD != requestMode)
        ;
}

#endif

