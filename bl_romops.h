/* 
 * File:   romops.h
 * Author: Ian
 *
 * Created on 31 January 2022, 21:21
 */

#ifndef ROMOPS_H
#define	ROMOPS_H

#ifdef	__cplusplus
extern "C" {
#endif

extern void initRomops(void);
extern unsigned char ee_read(void);
extern void ee_write(unsigned char data);

extern void flushFlash(void);
extern void eraseFlash(void);
extern unsigned char readFlashByte(void);
extern void writeFlashByte(unsigned char value);
extern void writeConfigByte(unsigned char value);

            // error codes
#define NO_ERROR        0x00
#define VERIFY_ERROR    0x01
#define	ADDRESS_ERROR   0x02       // Tried to write to an invalid address

#ifdef MODE_SELF_VERIFY
    extern uint8_t errorStatus;
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* ROMOPS_H */

