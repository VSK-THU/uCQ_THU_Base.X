//##############################################################################
//    filename:        lcd_config.h
//##############################################################################
//    configuration file for LCD library (pins, timing, voltage, ...)
//##############################################################################
//
//      Author:            	V.SchK
//      Company:        	HS-Ulm
//
//      Revision:        	3.0
// 	    Date:               May 2019
//      Assembled using		XC8 2.00+
//
//##############################################################################

#ifndef _LCD_CONFIG_H
#define _LCD_CONFIG_H

#include <xc.h>
#include "demo_functions.h"     //#define _XTAL_FREQ ...

#define LCD_TIMEOUT 100     // max nr. of busy checks ...

#define LCD_DELAY_5MS() __delay_ms(5)
#define LCD_DELAY_1US() __delay_us(1)


//#warning "###############################"
//#warning "##### LCD-pinning for uC-Quick board! #####"
//#warning "###############################"
#define	LCD_E		LATCbits.LATC1
#define	LCD_E_DIR	TRISCbits.TRISC1
#define	LCD_RW		LATCbits.LATC0
#define	LCD_RW_DIR	TRISCbits.TRISC0
#define	LCD_RS		LATAbits.LATA5
#define	LCD_RS_DIR	TRISAbits.TRISA5

#define	LCD_D4_IN	PORTBbits.RB2
#define	LCD_D5_IN	PORTBbits.RB3
#define	LCD_D6_IN	PORTBbits.RB4
#define	LCD_D7_IN	PORTBbits.RB5
#define	LCD_D4_OUT	LATBbits.LATB2
#define	LCD_D5_OUT	LATBbits.LATB3
#define	LCD_D6_OUT	LATBbits.LATB4
#define	LCD_D7_OUT	LATBbits.LATB5
#define	LCD_D4_DIR	TRISBbits.TRISB2
#define	LCD_D5_DIR	TRISBbits.TRISB3
#define	LCD_D6_DIR	TRISBbits.TRISB4
#define	LCD_D7_DIR	TRISBbits.TRISB5

#define LCD_DIR_IN()  LCD_D4_DIR = LCD_D5_DIR = LCD_D6_DIR = LCD_D7_DIR = 1
#define LCD_DIR_OUT() LCD_D4_DIR = LCD_D5_DIR = LCD_D6_DIR = LCD_D7_DIR = 0
//#define LCD_DIR_IN()    TRISB |= 0x3C
//#define LCD_DIR_OUT()   TRISB &= 0xC3

#define LCD_PINS_DIGITAL()  ANSELB &= 0b11000011 

#endif //_LCD_CONFIG_H
