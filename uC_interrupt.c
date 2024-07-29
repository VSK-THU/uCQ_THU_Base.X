//##############################################################################
//    filename:        Quick_intr.c
//##############################################################################
//    interrupt functions for uCQuick demo project
//##############################################################################
//
//      Author:            	V.SchK
//      Company:            THU-Ulm
//
//      Revision:           3.0 (XC8 C99 version)
//      Date:               May 2019
//      Assembled using     XC8 2.00+
//
//   	todo	- add comments ;-)
//             	-
//
//##############################################################################

/** I N C L U D E S ***********************************************************/
#include "uCQuick/uCQ_2013.h"
#include "demo_functions.h"

//#if __STDC_VERSION__ == 199901L
//        #warning    ###############--C99--###############
//#endif

#if defined(USE_ENCODER_POLLING_IR)
typedef enum {
    ENC_d4      = -4,   // ENC_DOWN for 1-pulse/1-detent (puls)
    ENC_d3      = -3,
    ENC_DOWN    = -2,   // ENC_DOWN for 1-pulse/2-detent (toggle)
    ENC_d1      = -1,   // ENC_DOWN for quadratur encoder
    ENC_IDLE    =  0,
    ENC_u1      =  1,   // ENC_UP for quadratur encoder
    ENC_UP      =  2,   // ENC_UP for 1-pulse/2-detent (toggle)
    ENC_u3      =  3,
    ENC_u4      =  4    // ENC_UP for 1-pulse/1-detent (puls)
} ENC_STATE;
ENC_STATE encState;
unsigned char encNew, encOld;
#endif


/** D E C L A R A T I O N S ***************************************************/
//##############################################################################
// Function:        void high_isr(void)
// PreCondition:    None
// Input:
// Output:
// Side Effects:
// Overview:
//##############################################################################
void __interrupt(high_priority) high_isr(void)
{
#if defined(USE_TMR0_IR)
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF){      // if timer 0 overflow
        mTOG_LED_3();           //   then toggle LED
        INTCONbits.TMR0IF = 0;  //   and clear flag
        return;
    }
#endif
//------------------------------------------------------------------------------
#if defined(USE_ENCODER_IR)
    if(ENC_IR){
        if(ENC_DIR == ENC_DIR_UP)  flags.encUp = 1;
          else flags.encDown = 1;

        mENC_IR_RST();          // clear flag and toggle edge if necessary
        return;
    }
#endif
//------------------------------------------------------------------------------
#if defined(USE_ADC_IR)
    if(ADC_IR){
        static char tsec = 0;
        if (adc_value != ADRES){
            adc_value = ADRES;
            flags.newADC = 1;
        }
        if(++tsec >= 10){
            flags.newSec = 1;
            tsec = 0;
        }
        mADC_IR_CLR();
        return;
    }
#endif
//------------------------------------------------------------------------------
#if defined(USE_CAPTURE_IR)
    if(CAPTR_IR){
        capture_value = (unsigned short)CCPR1 - old_capture;
        old_capture = (unsigned short)CCPR1;
        flags.newCapture = 1;
        mCAPTR_IR_CLR();
        return;
    }
#endif
//------------------------------------------------------------------------------
#if defined(USE_SECONDS_IR)
    if(SEC_IR){
        //        if(flags.newSec) -> not yet processed ???
        flags.newSec = 1;
        mSEC_IR_CLR();
        return;
    }
#endif
//------------------------------------------------------------------------------
#if defined(USE_ENCODER_POLLING_IR)
    if(INP_POLL_IR){                            // timer IR (i.e. 1000Hz)
        if (flags.firstLoop){
//            encOld = 2 * ENC_A + ENC_B;
            encNew = 0; if(ENC_A) encNew |= 2; if(ENC_B) encNew |= 1;
            encState = ENC_IDLE;
            flags.firstLoop = 0;
        } else{
//            encNew = (ENC_A << 1) + ENC_B;
            encNew = 0; if(ENC_A) encNew |= 2; if(ENC_B) encNew |= 1;
            if (encNew != encOld){             // signals changed
                if((encNew ^ encOld) == 0b11){ // both signals changed -> ERROR
                    flags.encCERR = 1;
                }
                else {                          // only one signal changed ? OK
                    switch(encOld){                    // up   00-10-11-01-00...
                        case 0b00:                      // down 00-01-11-10-00...
                            if(encNew == 0b10){encState++;}
                            else{encState--;} break;
                        case 0b10:
                            if(encNew == 0b11){encState++;}
                            else{ encState--; } break;
                        case 0b11:
                            if(encNew == 0b01){encState++;}
                            else{ encState--; } break;
                        case 0b01:
                            if(encNew == 0b00){encState++;}
                            else{ encState--;} break;
                    }
                    if(encState == ENC_UP){
                        if(flags.encUp){
                            flags.encOERR = 1;  // <- breakpoint here
                        }
                        flags.encUp = 1; encState = ENC_IDLE;
                    }
                    else if(encState == ENC_DOWN){
                        if(flags.encDown){
                            flags.encOERR = 1;
                        }
                        flags.encDown = 1; encState = ENC_IDLE;
                    }
                }
                encOld = encNew;
            }
        }
        mINP_POLL_IR_CLR();
        return;
    }
#endif
//------------------------------------------------------------------------------
    while(1){;}     // (detect unexpected IR sources)
}

//##############################################################################
// Function:        void low_isr(void)
// PreCondition:    None
// Input:
// Output:
// Side Effects:
// Overview:
//##############################################################################
#ifdef USE_IR_PRIORITIES
void __interrupt(low_priority) low_isr(void)
{
    while(1){;}     // (detect unexpected IR sources)
}
#endif
