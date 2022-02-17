/* 
 * File:   module.h
 * Author: Ian
 *
 * Created on 28 January 2022, 18:48
 */

#ifndef MODULE_H
#define	MODULE_H

#ifdef	__cplusplus
extern "C" {
#endif

#define MIN_WRITEABLE_FLASH 0x0800
#define MAX_WRITEABLE_FLASH 0xFFFF
    
// Whether the module uses high or low priority for CAN interrupts
// set to 0 for LP. Set to 0xFF for HP
#define CAN_INTERRUPT_PRIORITY 0    // all low priority We should not actually use this!


#ifdef	__cplusplus
}
#endif

#endif	/* MODULE_H */

