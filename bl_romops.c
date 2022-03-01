/* 
 romops.c - EEPROM and FLASH ROM routines 


 Original CANACC8 assembler version (c) Mike Bolton
 Modifications to EEPROM routines and conversion to C18 (c) Andrew Crosland
 FLASH routines by (c) Chuck Hoelzen
 Modifications, refinements & combine EEPROM and FLASH into one module (C) Pete Brownlow 2014-2017   software@upsys.co.uk

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
	
 For library version number and revision history see CBUSLib.h

 Ported to XC8 from CBUSlib romops.c by Ian Hogg 31/1/2022

*/

#include <xc.h>
#include <bl_romops.h>

#include <main.h>

#include "GenericTypeDefs.h"

#define FLASH_BLOCK_SIZE    64
#define FLASH_BLOCK_MASK    0x3F
#define FALSE 0
#define TRUE 1
#define CONFIGU             0x30

static unsigned char buffer[FLASH_BLOCK_SIZE];
static unsigned char bufferAddrL;
static unsigned char bufferAddrH;
static unsigned char dirty;
#define BLOCK(h, l)  (((int)h<<8)|(l&(~FLASH_BLOCK_MASK)))
#define CLEARING_BITS(a, b) (a & (~b))

static unsigned char w;
static unsigned char addrL;
static unsigned char addrH;

unsigned char errorStatus;


void initRomops(void) {
    dirty = FALSE;
    for (w=0; w<FLASH_BLOCK_SIZE; w++) {
        buffer[w] = 0xFF;
    }
}

/**
 * Load a byte directly from Flash - either Program or Config - memory.
 * The address to be read must already be loaded into TBLPTR.
 */
unsigned char readFlashByte(void) {
    asm("TBLRD*");
    return TABLAT;
}

/* 
 * Write a byte into the buffer. writes to the TBLPTR address
 * It will flush the old buffer contents if this is for a different block.
 */
void writeFlashByte(unsigned char value) {
    // check if this is the block we already have buffered or a different block
    if (BLOCK(TBLPTRH, TBLPTRL) != BLOCK(bufferAddrH, bufferAddrL)) {
        addrL = TBLPTRL;    // preserve the TBLPTR
        addrH = TBLPTRH;
        
        // different block
        flushFlash();
        
        // record the start address of the block we have buffered
        bufferAddrL = addrL & ~FLASH_BLOCK_MASK;
        bufferAddrH = addrH;
        //read the entire new block into buffer
        for (w=0; w<FLASH_BLOCK_SIZE; w++) {
            TBLPTRL = bufferAddrL + w;
            EECON1 = 0x80;  // Flash program space
            buffer[w] = readFlashByte();
        }
        TBLPTRL = addrL;    // restore it
        TBLPTRH = addrH;
    }
    if (CLEARING_BITS(buffer[TBLPTRL & FLASH_BLOCK_MASK], value)) {
        dirty = TRUE;
    }
    // save to the lock buffer
    buffer[TBLPTRL & FLASH_BLOCK_MASK] = value;

}

/**
 * Erase the blocked pointed to by TBLPTR
 */
void eraseFlash(void) {
    EECON1bits.EEPGD = 1;   // 1=Program memory, 0=EEPROM
    EECON1bits.CFGS = 0;    // 0=Program memory/EEPROM, 1=ConfigBits
    EECON1bits.WREN = 1;    // enable write to memory
    EECON1bits.FREE = 1;    // enable row erase operation

    // unlock
    EECON2 = 0x55;          // write 0x55
    EECON2 = 0xaa;          // write 0xaa
    EECON1bits.WR = 1;      // start erasing
    EECON1bits.WREN = 0;    // disable write to memory
}

/**
 * Flush the current buffer to Flash to the address of bufferAddr
 * Verify if required.
 * Corrupts TBLPTR
 * @return 
 */
void flushFlash(void) {
    // no need to do anything if clean
    if (!dirty) {
        return;
    }
    TBLPTRL = bufferAddrL;
    TBLPTRH = bufferAddrH;
    eraseFlash();

    for (w=0; w<FLASH_BLOCK_SIZE; w++) {
        TABLAT = buffer[w];
        asm("TBLWT*+");
    }
    // Note from data sheet: 
    //   Before setting the WR bit, the Table
    //   Pointer address needs to be within the
    //   intended address range of the 64 bytes in
    //   the holding register.
    // So we put it back into the block here
    TBLPTR--;
    EECON1 = 0x84;   // Flash, program, enable write
    // unlock
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = TRUE;       // start writing 
    asm("nop");                 // needs a nop before testing the bit
    while (EECON1bits.WR)       // wait for write to complete
        ;
    EECON1bits.WREN = FALSE;    // disable write to memory
    
    dirty = FALSE;
    
#ifdef MODE_SELF_VERIFY
    // read back to verify
    for (w=0; w<FLASH_BLOCK_SIZE; w++) {
        TBLPTRL = bufferAddrL+w;
        TBLPTRH = bufferAddrH;
        EECON1 = 0x80;  // Flash Program space
        if (buffer[w] != readFlashByte()) {
            errorStatus = VERIFY_ERROR;
        }
    }
#endif
}

/**
 * This writes directly to the Flash Config. It uses the TBLPTR as the address
 */
void writeConfigByte(unsigned char value) {
    TABLAT = value;
    asm("TBLWT*");
    EECON1 = 0xC4;   // Flash, Config, enable write
    // unlock
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = TRUE;       // start writing
    asm("nop");                 // needs a nop before testing the bit
    while (EECON1bits.WR)       // Wait for the write to complete
        ;
    EECON1bits.WREN = FALSE;    // disable write to memory
    /* 
     * The following SELF_VERIFY code was originally written but FCU fills in
     * missing CONFIG values, such as 0x30004, with 0xFF but these are read back 
     * as 0x00.
     * 
#ifdef MODE_SELF_VERIFY
    EECON1 = 0xC0;  // Flash Configuration space
    if (readFlashByte() != value) {
        errorStatus = VERIFY_ERROR;
    }
#endif
     */
}

/*****************************
 *
 *  EE ROUTINES
 *
 ******************************/
/**
 * Write one byte to data EEPROM.
 * We verify at end and try again if necessary.
 * The address to be written is passed in TBLPTR
 * @param data the data to be written
 */
void ee_write(unsigned char data) {
    do {
        EEDATA = data;
        EECON1bits.EEPGD = 0;       /* Point to DATA memory */
        EECON1bits.CFGS = 0;        /* Access program FLASH/Data EEPROM memory */
        EECON1bits.WREN = 1;        /* Enable writes */

        EECON2 = 0x55;
        EECON2 = 0xAA;
        EECON1bits.WR = 1;

        while (EECON1bits.WR)
            ;
        while (!EEIF)
            ;
        EEIF = 0;
        EECON1bits.WREN = 0;		/* Disable writes */
    } while (ee_read() != data);    //repeat is no match
}

/**
 * Read a byte from data EEPROM.
 * The address to be read is passed in TBLPTR
 * @return the byte from EEPROM
 */
unsigned char ee_read(void) {
    while (EECON1bits.WR)       // Errata says this is required
        ;
    EECON1bits.EEPGD = 0;    	/* Point to DATA memory */
    EECON1bits.CFGS = 0;    	/* Access program FLASH/Data EEPROM memory */
    EECON1bits.RD = 1;			/* EEPROM Read */
    while (EECON1bits.RD)
        ;
    asm("NOP");                 /* data available after a NOP */
    return EEDATA;
}
