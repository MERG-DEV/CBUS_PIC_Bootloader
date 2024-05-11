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

#if defined(_18F66K80_FAMILY_)
#define FLASH_BLOCK_SIZE    64
#define FLASH_BLOCK_MASK    0x3F
#define CONFIGU             0x30
#endif
#if defined(_18FXXQ83_FAMILY_)
#define FLASH_BLOCK_SIZE    256
#define FLASH_BLOCK_MASK    0xFF
#endif
#define FALSE 0
#define TRUE 1

static unsigned char buffer[FLASH_BLOCK_SIZE];
static unsigned char bufferAddrL;
static unsigned char bufferAddrH;
#if defined(_18F66K80_FAMILY_)
#define BLOCK(h, l)  (((int)h<<8)|(l&(~FLASH_BLOCK_MASK)))
#endif
#if defined(_18FXXQ83_FAMILY_)
static unsigned char bufferAddrU;
#define BLOCK(u, h, l)  (((int24_t)u<<16)|((int)h<<8)|(l&(~FLASH_BLOCK_MASK)))
#endif


#define CLEARING_BITS(a, b) (a & (~b))
static unsigned char dirty;

#if defined(_18F66K80_FAMILY_)
static unsigned char w;
#endif
static unsigned char addrL;
static unsigned char addrH;
#if defined(_18FXXQ83_FAMILY_)
static unsigned char addrU;

static uint16_t w;
#endif

#ifdef MODE_SELF_VERIFY
unsigned char errorStatus;
#endif

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
 * Write a byte into the buffer. Writes to the TBLPTR or NVMADR address
 * It will flush the old buffer contents if this is for a different block.
 */
void writeFlashByte(unsigned char value) {
    // check if this is the block we already have buffered or a different block
    
#if defined(_18F66K80_FAMILY_)
    if (BLOCK(TBLPTRH, TBLPTRL) != BLOCK(bufferAddrH, bufferAddrL)) {
        // different block
        
        addrL = TBLPTRL;    // preserve the TBLPTR
        addrH = TBLPTRH;
        
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
#endif
#if defined(_18FXXQ83_FAMILY_)
    if (BLOCK(NVMADRU, NVMADRH, NVMADRL) != BLOCK(bufferAddrU, bufferAddrH, bufferAddrL)) {
        // different block
        
        addrL = NVMADRL;    // preserve the NVMADR
        addrH = NVMADRH;
        addrU = NVMADRU;
        
        flushFlash();       // erase and save old block
        // record the start address of the block we have buffered
        bufferAddrL = addrL & ~FLASH_BLOCK_MASK;
        bufferAddrH = addrH;
        bufferAddrU = addrU;
        
        // ready?
        while (NVMCON0bits.GO)
            ;
        //Load NVMADR with the starting address of the memory page
        NVMADRU = bufferAddrU;
        NVMADRH = bufferAddrH;
        NVMADRL = bufferAddrL;
        NVMCON1bits.NVMCMD = 0x02;      //Set the page read command
        NVMCON0bits.GO = 1;             //Start page read
        NVMCON1bits.NVMCMD = 0x00;      //Clear the NVM Command
    }
    if (CLEARING_BITS(buffer[bufferAddrL & FLASH_BLOCK_MASK], value)) {
        dirty = TRUE;
    }
    // save to the lock buffer
    buffer[bufferAddrL & FLASH_BLOCK_MASK] = value;
#endif
}

/**
 * Erase the blocked pointed to by TBLPTR
 */
void eraseFlash(void) {
#if defined(_18F66K80_FAMILY_)
    EECON1bits.EEPGD = 1;   // 1=Program memory, 0=EEPROM
    EECON1bits.CFGS = 0;    // 0=Program memory/EEPROM, 1=ConfigBits
    EECON1bits.WREN = 1;    // enable write to memory
    EECON1bits.FREE = 1;    // enable row erase operation

    // unlock
    EECON2 = 0x55;          // write 0x55
    EECON2 = 0xaa;          // write 0xaa
    EECON1bits.WR = 1;      // start erasing
    EECON1bits.WREN = 0;    // disable write to memory
#endif
#if defined(_18FXXQ83_FAMILY_)
    NVMCON1bits.NVMCMD = 0x06;      //Set the page erase command
    //Perform the unlock sequence 
    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;
    NVMCON0bits.GO = 1;             //Start byte write
    while (NVMCON0bits.GO)
        ;
    NVMCON1bits.NVMCMD = 0x00;      //Clear the NVM Command
#endif
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
#if defined(_18F66K80_FAMILY_)
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
#endif
#if defined(_18FXXQ83_FAMILY_)
    NVMADRL = bufferAddrL;
    NVMADRH = bufferAddrH;
    NVMADRU = bufferAddrU;
    eraseFlash();
    NVMCON1bits.NVMCMD = 0x05;      //Set the page write command
    //Perform the unlock sequence 
    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;
    NVMCON0bits.GO = 1;             //Start byte write
    while (NVMCON0bits.GO)
        ;
    NVMCON1bits.NVMCMD = 0x00;      //Clear the NVM Command
#endif
    
    dirty = FALSE;
    
#ifdef MODE_SELF_VERIFY
    // read back to verify
#if defined(_18F66K80_FAMILY_)
    for (w=0; w<FLASH_BLOCK_SIZE; w++) {
        TBLPTRL = bufferAddrL+w;
        TBLPTRH = bufferAddrH;
        EECON1 = 0x80;  // Flash Program space
        if (buffer[w] != readFlashByte()) {
            errorStatus = VERIFY_ERROR;
        }
    }
#endif
#if defined(_18FXXQ83_FAMILY_)
    //Load NVMADR with the starting address of the memory page
    NVMADRU = bufferAddrU;
    NVMADRH = bufferAddrH;
    NVMADRL = bufferAddrL;
    NVMCON1bits.NVMCMD = 0x02;      //Set the page read command
    NVMCON0bits.GO = 1;             //Start page read
    while (NVMCON0bits.GO)
        ;
    NVMCON1bits.NVMCMD = 0x00;      //Clear the NVM Command
#endif
#endif
}

/**
 * This writes directly to the Flash Config. It uses the TBLPTR as the address
 */
void writeConfigByte(unsigned char value) {
#if defined(_18F66K80_FAMILY_)
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
     * as 0x00 so many CONFIG bytes would fail verification but be correct.
     * 
#ifdef MODE_SELF_VERIFY
    EECON1 = 0xC0;  // Flash Configuration space
    if (readFlashByte() != value) {
        errorStatus = VERIFY_ERROR;
    }
#endif
     */
#endif
#if defined(_18FXXQ83_FAMILY_)
    // ready?
    while (NVMCON0bits.GO)
        ;
    NVMCON1bits.NVMCMD = 0;
    NVMCON0bits.GO = 1;
    
    NVMDATL = value;
    NVMDATH = 0;
    NVMCON1bits.NVMCMD = 3;
    //Perform the unlock sequence 
    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;
    NVMCON0bits.GO = 1;
    NVMCON1bits.NVMCMD = 0;
#endif
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
#if defined(_18F66K80_FAMILY_)
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
#endif
#if defined(_18FXXQ83_FAMILY_)
        //Load NVMDAT with the desired value
        NVMDATL = data;
        //Set the byte write command
        NVMCON1bits.NVMCMD = 0x03;
        //Perform the unlock sequence 
        NVMLOCK = 0x55;
        NVMLOCK = 0xAA;
        //Start byte write
        NVMCON0bits.GO = 1;
        while (NVMCON0bits.GO)
            ;
        //Clear the NVM Command
        NVMCON1bits.NVMCMD = 0x00;
#endif
        
    } while (ee_read() != data);    //repeat if no match. This makes SELF_VERIFY unnecessary here
}

/**
 * Read a byte from data EEPROM.
 * The address to be read is passed in TBLPTR
 * @return the byte from EEPROM
 */
unsigned char ee_read(void) {
#if defined(_18F66K80_FAMILY_)
    while (EECON1bits.WR)       // Errata says this is required
        ;
    EECON1bits.EEPGD = 0;    	/* Point to DATA memory */
    EECON1bits.CFGS = 0;    	/* Access program FLASH/Data EEPROM memory */
    EECON1bits.RD = 1;			/* EEPROM Read */
    while (EECON1bits.RD)
        ;
    asm("NOP");                 /* data available after a NOP */
    return EEDATA;
#endif
#if defined(_18FXXQ83_FAMILY_)
    //Set the byte read command
    NVMCON1bits.NVMCMD = 0x00;
    //Start byte read
    NVMCON0bits.GO = 1;
    while (NVMCON0bits.GO)
        ;
    return NVMDATL;
#endif
}
