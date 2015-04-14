/* 
 * File:   config.h
 * Author: fcc89
 *
 * Created on April 11, 2015, 1:48 PM
 */
        // __delayXXX() functions macros defined here
#ifndef CONFIG_H
#define	CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

    #define    FCY    40000000UL    // Instruction cycle frequency, Hz - required for __delayXXX() to work
    #define    SERVO1 PORTBbits.RB0
    #define    TRIS_SERVO1  TRISBbits.TRISB0
    #define    SERVO2 PORTBbits.RB1
    #define    TRIS_SERVO2  TRISBbits.TRISB1
    void configBoard();


#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */

