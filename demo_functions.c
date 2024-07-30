//##############################################################################
//    	filename:        	uC-Quick     demo_functions.c
//
//##############################################################################
//
//  Author:            	V.SchK
//  Company:            THU-Ulm
//
//  Revision:           3.0 (XC8 C99 version)
//  Date:               May 2019
//  Assembled using     XC8 2.00+
//
//   	todo	- add comments ;-)
//             	-
//
//##############################################################################

#pragma warning disable 520 // function never called
#pragma warning disable 373 // implicit signed to unsigned conversion

#include "uCQuick/uCQ_2018.h"
#include "LCD/GLCDnokia.h"
#include "demo_functions.h"

#include "PLIB/plib.h"

union DemoFlags flags;
unsigned short adc_value;      // for AnalogLCD()
unsigned short capture_value;  // for TimeMeasure()
unsigned short old_capture;    // ""


//##############################################################################
void TemplateCode(void)     // Template Code from uC_Quick-X.pdf
{
    unsigned char counter_1 = 0;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_31KHZ;       // defined in uCQ_2018.h
    OSCTUNEbits.PLLEN = 0;
    LED_1_TRI = OUTPUT_PIN;             //   ""
//----------------------------------------------------------------------- main()
    while(1){
        if(++counter_1 == 0){
            mTOG_LED_1();               // defined in uCQ_2018.h
        }
    }
}

//##############################################################################
void TestLEDs(void)
{
//--------------------------------------------------------------------- __init()
    LATBbits.LATB2 = 1;     // LED1 off
    LATBbits.LATB3 = 1;     // LED2 off
    LATBbits.LATB4 = 1;     // LED3 off
    LATBbits.LATB5 = 1;     // LED4 off

    TRISBbits.TRISB2 = 0;   // pin for LED1 -> output
    TRISBbits.TRISB3 = 0;   // pin for LED2 -> output
    TRISBbits.TRISB4 = 0;   // pin for LED3 -> output
    TRISBbits.TRISB5 = 0;   // pin for LED4 -> output

// optional init version 1
//    LATB  |= 0b00111100;    // binary
//    TRISB &= 0b11000011;    // hexadezimal
//    LATB  |= 0x3C;
//    TRISB &= 0xC3;

// optional init version 2
//    LED_1 = LED_OFF;    LED_1_TRI = OUTPUT_PIN;
//    LED_2 = LED_OFF;    LED_2_TRI = OUTPUT_PIN;
//    LED_3 = LED_OFF;    LED_3_TRI = OUTPUT_PIN;
//    LED_4 = LED_OFF;    LED_4_TRI = OUTPUT_PIN;

// optional init version 3
//    mALL_LED_OFF();     mALL_LED_OUTPUT();

//----------------------------------------------------------------------- main()
    while(1){
        LATBbits.LATB2 = 1; // LED1 off
        LATBbits.LATB3 = 1; // LED2 off
        LATBbits.LATB4 = 1; // LED3 off
        LATBbits.LATB5 = 1; // LED4 off
        LATBbits.LATB2 = 0; // LED1 on          <- place break-point here
        LATBbits.LATB3 = 0; // LED2 on          then single step
        LATBbits.LATB4 = 0; // LED3 on
        LATBbits.LATB5 = 0; // LED4 on

        LATB = 0b11111111;  // all off binary
        LATB = 0b00000000;  // all on
        LATB = 0xFF;        //   hex
        LATB = 0x00;
        LATB |= 0b00111100; //   mask
        LATB &= 0b11000011;

        LATB |= 0b00000100; // LED1 off
        LATB |= 0b00001000; // LED2 off
        LATB |= 0b00010000; // LED3 off
        LATB |= 0b00100000; // LED4 off

        LATB &= 0b11111011; // LED1 on
//      ...

        mSET_LED_1_OFF();
        mSET_LED_1_ON();
        mSET_LED_1_OFF();
//      ...

        mALL_LED_OFF();
        mALL_LED_ON();
    }
}

//##############################################################################
void ButtonLEDs(void)
{
//--------------------------------------------------------------------- __init()
    mALL_LED_OUTPUT();
    ENC_BTN_TRI = INPUT_PIN; ENC_BTN_ANS = DIGITAL_PIN;

//----------------------------------------------------------------------- main()
    while(1){
        if(mGET_ENC_BTN()){
            mALL_LED_ON();
        }
        else{
            mALL_LED_OFF();
        }
    }
}

//##############################################################################
void ButtonRandomLEDs(void)
{
//--------------------------------------------------------------------- __init()
    mSET_LED_1_ON();mSET_LED_2_ON();
    mSET_LED_3_OFF();mSET_LED_4_OFF();
    mALL_LED_OUTPUT();
    ENC_BTN_TRI = INPUT_PIN; ENC_BTN_ANS = DIGITAL_PIN;

//----------------------------------------------------------------------- main()
    while(1){
        if(mGET_ENC_BTN()){
            mTOG_LED_1();
            mTOG_LED_2();
            mTOG_LED_3();
            mTOG_LED_4();
        }
    }
}

//##############################################################################
void ButtonToggleLEDs(void)
{
//--------------------------------------------------------------------- __init()
    mSET_LED_1_ON();mSET_LED_2_ON();
    mSET_LED_3_OFF();mSET_LED_4_OFF();
    mALL_LED_OUTPUT();
    ENC_BTN_TRI = INPUT_PIN; ENC_BTN_ANS = DIGITAL_PIN;

//----------------------------------------------------------------------- main()
    while(1){
        if(mGET_ENC_BTN()){
            mTOG_LED_1();
            mTOG_LED_2();
            mTOG_LED_3();
            mTOG_LED_4();
            while (mGET_ENC_BTN()){;}
        }
    }
}

//##############################################################################
void SwitchIOs(void)
{
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0; // (default setting)

    mSET_LED_1_ON();mSET_LED_2_ON();mSET_LED_3_OFF();mSET_LED_4_OFF();
    mALL_LED_OUTPUT();

    BTN_L_TRI = BTN_R_TRI = INPUT_PIN;
    BTN_L_ANS = BTN_R_ANS = DIGITAL_PIN;
//----------------------------------------------------------------------- main()
    while(1){
        BTN_L_TRI = INPUT_PIN;
        __delay_us(10);          //  _XTAL_FREQ defined in demo_functions.h
        if(mGET_BTN_L()){
            mTOG_LED_1();
            mTOG_LED_2();
            while (mGET_BTN_L()){;}
        }
        LED_1_TRI = OUTPUT_PIN;

        BTN_R_TRI = INPUT_PIN; __delay_us(10);
        if(mGET_BTN_R()){
            mTOG_LED_3();
            mTOG_LED_4();
            while (mGET_BTN_R()){;}
        }
        LED_3_TRI = OUTPUT_PIN;
    }
}

//##############################################################################
#define MASK_L_BTN      0b00000100
#define MASK_R_BTN      ???
#define MASK_BTNs       ???
#define MASK_LEDs       0b00111100
#define MASK_TGL_L_LEDS 0b00001100
void IPO_btn_led(void)      // IPO buttons L/R toggle LEDs
{
    unsigned char buttons_ago, buttons_now, leds;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0;
    mSET_LED_1_ON();mSET_LED_2_ON();mSET_LED_3_OFF();mSET_LED_4_OFF();
    mALL_LED_OUTPUT();
    BTN_L_ANS = BTN_R_ANS = DIGITAL_PIN;
    WPUBbits.WPUB2 = 1; WPUBbits.WPUB5 = 1; INTCON2bits.NOT_RBPU = 0;

    buttons_now = buttons_ago = 0b00010100; // initialize as not pressed
    leds = LATB & MASK_LEDs;                // initialize leds variable
//----------------------------------------------------------------------- main()
    while(1){
// input
        BTN_L_TRI = BTN_R_TRI = INPUT_PIN;
        __delay_us(10);
        buttons_now = PORTB & 0b00010100;       // read buttons
        LED_1_TRI = LED_3_TRI = OUTPUT_PIN;
// process
        if(buttons_now != buttons_ago){         // something happened
            if( (buttons_ago & MASK_L_BTN) > (buttons_now & MASK_L_BTN) ){
                leds ^= MASK_TGL_L_LEDS;
            }
            if( (buttons_ago & 0x10) > (buttons_now & 0x10) ){  // MASK_R_BTN
                leds ^= 0x30;                                   // MASK_TGL_R...
            }
            buttons_ago = buttons_now;          // prepare for next run
        }
// output
        LATB = leds;
//        __delay_ms(10);                         // slow down
    }
}

//##############################################################################
#define MASK_TGL_L_LED  0b00000100
#define MASK_TGL_R_LED  ???
#define MASK_TGL_M_LEDS ???
void TimeSliced_IPO(void)      // IPO buttons L/R toggle outer LEDs
{
    unsigned char buttons_ago, buttons_now, leds;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0;
    mSET_LED_1_ON();mSET_LED_2_ON();mSET_LED_3_OFF();mSET_LED_4_OFF();
    mALL_LED_OUTPUT();
    BTN_L_ANS = BTN_R_ANS = DIGITAL_PIN;
    WPUBbits.WPUB2 = 1; WPUBbits.WPUB5 = 1; INTCON2bits.NOT_RBPU = 0;

    T0CONbits.T0CS = 0;                     // instruction clock (fosc/4)
    T0CONbits.T08BIT = 1;
    T0CONbits.PSA = 0;                      // prescaler is assigned
    T0CONbits.T0PS = 0b110;                 // prescaler 1/128
    T0CONbits.TMR0ON = 1;

    buttons_now = buttons_ago = 0b00010100; // initialize as not pressed
    leds = LATB & MASK_LEDs;                // initialize leds variable
//----------------------------------------------------------------------- main()
    while(1){
// timing (alternatively at the end of main loop)
        if(INTCONbits.T0IF){                // timing error detection (stop)
            mSET_LED_1_ON();mSET_LED_2_ON();mSET_LED_3_ON();mSET_LED_4_ON();
            while(1){;}
        }
        while(!INTCONbits.T0IF){;}          // wait
        INTCONbits.T0IF = 0;
// input
        BTN_L_TRI = INPUT_PIN; BTN_R_TRI = INPUT_PIN;
        __delay_us(10);
        buttons_now = PORTB & 0b00010100;   // read buttons (MASK_BTNs)
        LED_1_TRI = LED_3_TRI = OUTPUT_PIN;
// process
        if(buttons_now != buttons_ago){     // something happened
            if( (buttons_ago & MASK_L_BTN) > (buttons_now & MASK_L_BTN) ){
                leds ^= MASK_TGL_L_LED;
            }
            if( (buttons_ago & 0x10) > (buttons_now & 0x10) ){  // MASK_R_BTN
                leds ^= 0x20;                                   // MASK_TGL_R...
            }
            buttons_ago = buttons_now;      // prepare for next run
        }
        __delay_ms(125);                    // simulate more code to execute
// output
        leds ^=  0x18;                      //MASK_TGL_M_LEDS
        LATB = leds;
    }
}

//##############################################################################
void TTL_ST_toLED(void)
{
//--------------------------------------------------------------------- __init()
    TTL_IN_TRI = INPUT_PIN; TTL_IN_ANS = DIGITAL_PIN;
    ST_IN_TRI = INPUT_PIN; ST_IN_ANS = DIGITAL_PIN;  // jumper next to poti !!!
    LED_1_TRI = OUTPUT_PIN;
    LED_2_TRI = OUTPUT_PIN;
//----------------------------------------------------------------------- main()
    while(1){               // VDD = 3.3V
        LED_1 = !TTL_IN;    // ON/OFF ~1.05V
        LED_2 = !ST_IN;     // ON ~1.66V    OFF ~1.3V
    }
}

//##############################################################################
void Blink_Counter(void)
{
    unsigned short time;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_31KHZ; OSCTUNEbits.PLLEN = 0;    // 31.25kHz clock
    ENC_BTN_TRI = INPUT_PIN; ENC_BTN_ANS = DIGITAL_PIN;
    LED_2_TRI = OUTPUT_PIN; // LED button
    LED_3_TRI = OUTPUT_PIN; // LED blink
    mSET_LED_3_ON();
//----------------------------------------------------------------------- main()
    while(1){
        if(mGET_ENC_BTN())  mSET_LED_2_ON();
        else                mSET_LED_2_OFF();

//        for (time = 0; time < 3906; time++) {;} // wait 1/2 sec. ???
        for (time = 0; time < 309; time++) {;}  // wait 1/2 sec. (better ;-) ((360))
          mTOG_LED_3();                         //   then toggle LED
    }
}

//##############################################################################
void Blink_Timer_Flag(void)
{
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_4MHZ; OSCTUNEbits.PLLEN = 0;
    ENC_BTN_TRI = INPUT_PIN;  ENC_BTN_ANS = DIGITAL_PIN;
    LED_2_TRI = OUTPUT_PIN;     // LED button
    LED_3_TRI = OUTPUT_PIN;     // LED blink
    mSET_LED_3_ON();
                                // TIMER_0 setup
    T0CON = 0b10000010;         // TMR0ON T08BIT T0CS T0SE PSA T0PS2 T0PS1 T0PS0
//----------------------------------------------------------------------- main()
    while (1) {
        if(mGET_ENC_BTN())  mSET_LED_2_ON();
        else                mSET_LED_2_OFF();

        if(INTCONbits.TMR0IF){      // if timer 0 overflow
            mTOG_LED_3();           //   then toggle LED
            INTCONbits.TMR0IF = 0;  //   and clear flag
        }
    }
}

//##############################################################################
void Blink_Timer_IR(void)
{
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_4MHZ; OSCTUNEbits.PLLEN = 0;
    ENC_BTN_TRI = INPUT_PIN; ENC_BTN_ANS = DIGITAL_PIN;
    LED_2_TRI = OUTPUT_PIN;     // LED button
    LED_3_TRI = OUTPUT_PIN;     // LED blink
    mSET_LED_3_ON();
                                // TIMER_0 setup
    T0CON = 0b10000010;         // TMR0ON T08BIT T0CS T0SE PSA T0PS2 T0PS1 T0PS0

    INTCONbits.TMR0IE = 1;      // enable timer_0 IR
    INTCONbits.GIE = 1;         // global IR enable
//----------------------------------------------------------------------- main()
    while (1) {
        if(mGET_ENC_BTN())  mSET_LED_2_ON();
        else mSET_LED_2_OFF();
    }
}

//##############################################################################
void Reaktionstest(void)
{
    while(1){;}
}

//##############################################################################
void EncoderLEDs(void)
{
    unsigned char dummy = 0b00000100;    // LEDs RB2..RB5
//--------------------------------------------------------------------- __init()
    mALL_LED_OUTPUT();

    ENC_INT_TRI = INPUT_PIN;  ENC_INT_ANS = DIGITAL_PIN;
    ENC_DIR_TRI = INPUT_PIN;  ENC_DIR_ANS = DIGITAL_PIN;

    mENC_IR_RST();
    mENC_IR_EN();

    flags.all = 0;                      // initializing new variable
    LATB = ~dummy;                      // initialize LEDs

    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        if(flags.encUp){
            dummy = dummy << 1;                 // shift left (LED2->3->4->5)
            if(dummy > 0b00100000) dummy = 0b00000100;
            LATB = ~dummy;                      // LEDs are low-active
            flags.encUp = 0;
        }
        if(flags.encDown){
            dummy = dummy >> 1;                 // shift right (LED5->4->3->2)
            if(dummy < 0b00000100 ) dummy = 0b00100000;
            LATB = ~dummy;                      // LEDs are low-active
            flags.encDown = 0;
        }
    }
}

//##############################################################################
void EncoderLCD_Text(void)
{
    unsigned char row = 0;
//    char poem[12][9]= {"Dunkel  ", "wars    ", "der Mond", "schien  ",
//                       "helle.  ", "Als ein ", "Auto    ", "blitze- ",
//                       "schnelle", "langsam ", "um die  ", "Ecke fuhr"};

    char poem[10][15]= {"Dunkel war's, ", "der Mond      ",
                        "schien helle, ", "schneebedeckt ",
                        "die grüne Flur", "als ein Wagen ",
                        "blitzeschnelle", "langsam um die",
                        "Ecke fuhr.    ", "              "};
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0;
    GLCD_Init();
    GLCD_Text2Out(0,1,"uC-Quick");
    GLCD_Text2Out(1,1,"  THU   ");

    // encoder setup
    ENC_INT_TRI = INPUT_PIN;  ENC_INT_ANS = DIGITAL_PIN;
    ENC_DIR_TRI = INPUT_PIN;  ENC_DIR_ANS = DIGITAL_PIN;

    mENC_IR_RST();
    mENC_IR_EN();

    flags.all = 0;

    INTCONbits.GIE = 1;
    while(!(flags.encUp || flags.encDown)); // wait for (any) encoder action
    GLCD_Clear(); flags.all = 0;
//----------------------------------------------------------------------- main()
    while(1){
        if(flags.encUp || flags.encDown){
            if(flags.encUp){
                if(row < 4) row++;
                flags.encUp = 0;
            }
            if(flags.encDown){
                if(row > 0) row--;
                flags.encDown = 0;
            }
//            GLCD_Clear();
            GLCD_TextOut(0,0,&(poem[row][0]));
            GLCD_TextOut(1,0,&(poem[row+1][0]));
            GLCD_TextOut(2,0,&(poem[row+2][0]));
            GLCD_TextOut(3,0,&(poem[row+3][0]));
            GLCD_TextOut(4,0,&(poem[row+4][0]));
            GLCD_TextOut(5,0,&(poem[row+5][0]));
        }
    }
}

//##############################################################################
void EncoderLCD_Value(void)
{
    short value = 0;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0;
    GLCD_Init();
    GLCD_Text2Out(0,1,"uC-Quick");
    GLCD_Text2Out(1,1,"  THU ");

    // encoder setup
    ENC_INT_TRI = INPUT_PIN;  ENC_INT_ANS = DIGITAL_PIN;
    ENC_DIR_TRI = INPUT_PIN;  ENC_DIR_ANS = DIGITAL_PIN;

    mENC_IR_RST();
    mENC_IR_EN();

    flags.all = 0;

    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        if(flags.encUp){
            value++;
            GLCD_Text2Out(2,0,"        ");   // delete old value
            GLCD_Value2Out(2,2,value);
            flags.encUp = 0;
        }
        if(flags.encDown){
            value--;
            GLCD_Text2Out(2,0,"        ");   // delete old value
            GLCD_Value2Out(2,2,value);
            flags.encDown = 0;
        }
    }
}

//##############################################################################
void EncoderPolling(void)
{
    short value = 0;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_16MHZ; OSCTUNEbits.PLLEN = 0;

// use TMR1 and CCP3 for input polling
    OpenTimer1(TIMER_INT_OFF & T1_16BIT_RW & T1_SOURCE_FOSC &
               T1_PS_1_1 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF,
               TIMER_GATE_OFF);
    OpenECompare3(COM_INT_ON & ECOM_TRIG_SEVNT & ECCP_3_SEL_TMR12, 16000);

    GLCD_Init();
    GLCD_Text2Out(0,1,"uCQ_2018");
    GLCD_Text2Out(1,1,"Enc.Poll");
    GLCD_Text2Out(2,4," 0 ");

    ENC_A_TRI = ENC_B_TRI = INPUT_PIN;// encoder setup
    ENC_A_ANS = ENC_B_ANS = DIGITAL_PIN;

    flags.all = 0;
    flags.firstLoop = 1;

    mINP_POLL_IR_EN();
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        if(flags.encUp){
            value++;
            GLCD_Text2Out(2,2,"        ");   // delete old value
            GLCD_Value2Out(2,4,value);
            flags.encUp = 0;
        }
        if(flags.encDown){
            value--;
            GLCD_Text2Out(2,2,"        ");   // delete old value
            GLCD_Value2Out(2,4,value);
            flags.encDown = 0;
        }
    }
}

//##############################################################################
void PWMcounter(void)
{
    unsigned char counter = 0, compare = 0b00011111;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0;

    GLCD_Init();
    GLCD_Text2Out(0,1,"uCQ_2018");
    GLCD_Text2Out(1,0,"PWM Count");
    GLCD_Text2Out(2,0,"comp:");

    mALL_LED_OUTPUT();

    // encoder setup
    ENC_INT_TRI = INPUT_PIN;  ENC_INT_ANS = DIGITAL_PIN;
    ENC_DIR_TRI = INPUT_PIN;  ENC_DIR_ANS = DIGITAL_PIN;
    mENC_IR_RST(); mENC_IR_EN();

    flags.all = 0;

    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        if(flags.encUp){
            compare = (compare << 1)+1; // 0->1->3->7->15->31->63->127->255
            GLCD_Value2Out_00(2,6,compare,3);
            flags.encUp = 0;
        }
        if(flags.encDown){
            compare = compare >> 1;     // 255->127->63->31->15->7->3->1->0
            GLCD_Value2Out_00(2,6,compare,3);
            flags.encDown = 0;
        }
        if(++counter == compare)        // <- count (increment) and compare
            mALL_LED_OFF();
        else if(counter == 0)           // counter overflow
            mALL_LED_ON();              //   start new
    }
}

//##############################################################################
void PWMccpPWM(void)
{
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0;
    GLCD_Init();
    GLCD_Text2Out(0,1,"uCQ_2018");
    GLCD_Text2Out(1,1,"PWM PWM");
    GLCD_Text2Out(2,0,"CCPR2L:");

    ENC_INT_TRI = INPUT_PIN;  ENC_INT_ANS = DIGITAL_PIN; // encoder setup
    ENC_DIR_TRI = INPUT_PIN;  ENC_DIR_ANS = DIGITAL_PIN;
    mENC_IR_RST(); mENC_IR_EN();
//#warning "######################################"
//#warning "##### Ensure #pragma config CCP2MX = PORTB3! #####"
//#warning "######################################"
    mSET_LED_2_OFF();
    LED_2_TRI = INPUT_PIN;   // LED2 is connected to CCP2(RB3) (disable for setup)

    CCP2CONbits.CCP2M = 0b1110;         // PWM mode PxA actice low
    CCP2CONbits.P2M0 = CCP2CONbits.P2M1 = 0;        // single output
    CCPTMRS0bits.C2TSEL = 0;            // CCP2-TMR2
    CCPR2L = 31;                        // duty cycle for pwm 31/256
    PR2 = 255;
//    SetOutputEPWM2(SINGLE_OUT, PWM_MODE_3);
//    SetDCEPWM2(512);
//    OpenEPWM2(255, ECCP_2_SEL_TMR12);       // 255 -> period

    T2CONbits.T2CKPS = 0;   // no pre scaler
    T2CONbits.T2OUTPS = 0;  // no post scaler
    T2CONbits.TMR2ON = 1;   // -> ~1kHz (IRCF_1MHZ)
//    OpenTimer2(TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1);  // ~1kHz (IRCF_1MHZ)
    LED_2_TRI = OUTPUT_PIN; // enable CCP2 output pin

    flags.all = 0;

    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        if(flags.encUp){
            CCPR2L = (CCPR2L << 1)+1; // 0->1->3->7->15->31->63->127->255
            GLCD_Value2Out_00(2,7,CCPR2L,3);
            flags.encUp = 0;
        }
        if(flags.encDown){
            CCPR2L = CCPR2L >> 1;     // 255->127->63->31->15->7->3->1->0
            GLCD_Value2Out_00(2,7,CCPR2L,3);
            flags.encDown = 0;
        }
    }
}

//##############################################################################
void PWMccpCompare(void)
{
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_8MHZ; OSCTUNEbits.PLLEN = 1;    // ->32MHz
    GLCD_Init();
    GLCD_Text2Out(0,1,"uCQ_2018");
    GLCD_Text2Out(1,0,"PWM COMPM");
    GLCD_Text2Out(2,0,"CCPR2H:");

    ENC_INT_TRI = INPUT_PIN;  ENC_INT_ANS = DIGITAL_PIN; // encoder setup
    ENC_DIR_TRI = INPUT_PIN;  ENC_DIR_ANS = DIGITAL_PIN;
    mENC_IR_RST(); mENC_IR_EN();

    mSET_LED_2_OFF();
    LED_2_TRI = OUTPUT_PIN;             // LED2 is connected to CCP2 (RB3)

    OpenECompare2(COM_INT_OFF & ECOM_HI_MATCH & ECCP_2_SEL_TMR12, (31<<8)+20);

    OpenTimer1(TIMER_INT_OFF & T5_16BIT_RW & T1_SOURCE_FOSC_4 & //~60Hz (32MHz)
               T1_PS_1_2 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF,
               TIMER_GATE_OFF);

    flags.all = 0;

    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        if(PIR1bits.TMR1IF){            // should be done in ISR of course ;-)
            if (flags.encUp) {
                CCPR2H = (CCPR2H << 1) + 1; // 0->1->3->7->15->31->63->127->255
                GLCD_Value2Out_00(2,7,CCPR2H,3);
                flags.encUp = 0;
            }
            if (flags.encDown) {
                CCPR2H = CCPR2H >> 1;       // 255->127->63->31->15->7->3->1->0
                GLCD_Value2Out_00(2,7,CCPR2H,3);
                flags.encDown = 0;
            }
            CCP2CON = ECOM_HI_MATCH;        // restart compare
            PIR1bits.TMR1IF = 0;
        }
    }
}

//##############################################################################
void AnalogLEDs(void)
{
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0;    // ->1MHz

    mALL_LED_OUTPUT();
    POTI_TRI = INPUT_PIN; POTI_ANS = ANALOG_PIN;

    ADCON0bits.CHS = 0;         // poti is connected to analog channel 0
    ADCON1bits.NVCFG = 0;       // ADref- connected to AVss (GND)
    ADCON1bits.PVCFG = 0;       // ADref+ connected to AVdd ()
    ADCON2bits.ADCS = 0;        // Tad = 1/(Fosc/2) = 2us
    ADCON2bits.ACQT = 0b010;    // 4*Tad = 8us
    ADCON2bits.ADFM = 0;        // left -> 8 most significant bits in ADRESH
    ADCON0bits.ADON = 1;

//    OpenADC(ADC_FOSC_2 & ADC_LEFT_JUST & ADC_4_TAD,             // ADCON2
//            ADC_POTI & ADC_INT_OFF,                             // ADCON0
//            ADC_TRIG_CCP5 & ADC_REF_VDD_VDD & ADC_REF_VDD_VSS); // ADCON1
//----------------------------------------------------------------------- main()
    while(1){
        ADCON0bits.GO = 1;              // start conversion
        while(ADCON0bits.NOT_DONE){;}   // wait for completion
        LATB = ~(ADRESH >> 2);          // display result
    }
}

//##############################################################################
void AnalogLEDs_PWM(void)
{
    unsigned char pwmCounter = 0;
    unsigned char adresult = 0;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0;
//    GLCD_Init(); GLCD_Text2Out(0,0,"LEDs PWM");
    mALL_LED_OUTPUT();;

    POTI_TRI = INPUT_PIN; POTI_ANS = ANALOG_PIN;

    OpenADC(ADC_FOSC_8 & ADC_LEFT_JUST & ADC_12_TAD,
            ADC_POTI & ADC_INT_OFF,
            ADC_TRIG_CCP5 &  ADC_REF_VDD_VDD & ADC_REF_VDD_VSS);
//----------------------------------------------------------------------- main()
    while(1){
        if(ADCON0bits.NOT_DONE == 0){
            if(ADRESH != adresult){
                adresult = ADRESH;
// add code here to linearize intensity adjustment
            }
            ADCON0bits.GO = 1;
        }
        if(++pwmCounter == adresult){
            mALL_LED_OFF();
        }
        else if(pwmCounter == 0){
            mALL_LED_ON();
        }
    }
}
//##############################################################################
void AnalogLED2_COMP(void)
{
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_8MHZ; OSCTUNEbits.PLLEN = 1;     // ->32MHz
//    GLCD_Init(); GLCD_Text2Out(0,0,"LED2 CMP");

    mSET_LED_2_OFF();
    LED_2_TRI = OUTPUT_PIN;     // LED2 is connected to CCP2 (RB3)

    POTI_TRI = INPUT_PIN; POTI_ANS = ANALOG_PIN;

    OpenADC(ADC_FOSC_8 & ADC_LEFT_JUST & ADC_12_TAD,
            ADC_POTI & ADC_INT_OFF,
            ADC_TRIG_CCP5 &  ADC_REF_VDD_VDD & ADC_REF_VDD_VSS);
    ADCON0bits.GO = 1;
    while(ADCON0bits.NOT_DONE){;}

    OpenECompare2(COM_INT_OFF & ECOM_HI_MATCH & ECCP_2_SEL_TMR12, ADRES);

    OpenTimer1(TIMER_INT_OFF & T5_16BIT_RW & T1_SOURCE_FOSC_4 & //~60Hz (32MHz)
               T1_PS_1_2 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF,
               TIMER_GATE_OFF);

//----------------------------------------------------------------------- main()
    while(1){
        if(PIR1bits.TMR1IF){            // should be done in ISR of course ;-)
            if(!ADCON0bits.GO){
                if(ADRES != CCPR2){     // new (different) value
                    CCPR2 = ADRES;
            // add code here to linearize intensity adjustment
                }
                ADCON0bits.GO = 1;
            }
            CCP2CON = ECOM_HI_MATCH;    // restart compare
            PIR1bits.TMR1IF = 0;
        }
    }
}

//##############################################################################
void AnalogLED2_PWM(void)
{
/* #ifdef __18CXX
    unsigned short oldAdres = 0;

//    GLCD_Init(); GLCD_Text2Out(0,0,"LED2 PWM");
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ;

    mSET_LED_2_OFF();
    LED_2_TRI = OUTPUT_PIN; // LED2 is connected to CCP2 (RB3)

    POTI_TRI = INPUT_PIN; POTI_ANS = ANALOG_PIN;

    OpenADC(ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_12_TAD,
            ADC_POTI & ADC_INT_OFF,
            ADC_TRIG_CCP5 &  ADC_REF_VDD_VDD & ADC_REF_VDD_VSS);

    OpenEPWM2(255, ECCP_2_SEL_TMR12);       // 255 -> period
    SetOutputEPWM2(SINGLE_OUT, PWM_MODE_3);
    SetDCEPWM2(512);

    OpenTimer2(TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1);    // ~1kHz (IRCF_1MHZ)

//----------------------------------------------------------------------- main()
    while(1){
        if(ADCON0bits.NOT_DONE == 0){
            if(ADRES != oldAdres){
            // add code here to linearize intensity adjustment
                SetDCEPWM2(ADRES);
                oldAdres = ADRES;
            }
            ADCON0bits.GO = 1;
        }
    }
#else */
    while(1){;}     // XC8 compiler not jet implemented :-(
//#endif
}

//##############################################################################
void AnalogLCD(void)
{
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0; // -> 1MHz (default)
    POTI_TRI = INPUT_PIN; POTI_ANS = ANALOG_PIN;;

    OpenADC(ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_12_TAD,
            ADC_POTI & ADC_INT_ON,
            ADC_TRIG_CCP5 & ADC_REF_VDD_VDD & ADC_REF_VDD_VSS);

    OpenTimer5(TIMER_INT_OFF & T5_16BIT_RW & T5_SOURCE_FOSC_4 &
               T5_PS_1_1 & T5_OSC1EN_OFF & T5_SYNC_EXT_OFF,
               TIMER_GATE_OFF);

    CCPTMRS1bits.C5TSEL  = 2;   // timer <-> ccp module (CCP5 / TMR5)
    CCPR5 = 25000;              // Fosc/4 / prescaler / Fadc = 1MHz/4 /2 /10Hz
    CCP5CONbits.CCP5M = 0b1011; // Compare Mode with Special Event Trigger

    flags.all = 0;

    GLCD_Init();
    GLCD_Text2Out(0,0,"SEC     ");
    GLCD_Text2Out(1,0,"ADC     ");

    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        static unsigned short sec = 0;
        if(flags.newADC){
            GLCD_Text2Out(1,4,"     ");   // delete old value
            GLCD_Value2Out_00(1,4,ADRES,4);
            flags.newADC = 0;
        }
        if(flags.newSec){
            GLCD_Value2Out(0,4,++sec);
            flags.newSec = 0;
        }
    }
}

//##############################################################################
void Analog_Out(void)
{
    unsigned char dac_value = 0;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0; // -> 1MHz (default)
    BTN_L_TRI = INPUT_PIN; BTN_L_ANS = DIGITAL_PIN;
    BTN_R_TRI = INPUT_PIN; BTN_R_ANS = DIGITAL_PIN;
    ENC_BTN_TRI = INPUT_PIN; ENC_BTN_ANS = DIGITAL_PIN;

    VREFCON0bits.FVRS = 3;      // 4.096 V
    VREFCON0bits.FVREN = 1;
    VREFCON1bits.DACPSS = 2;    // FVR
    VREFCON1bits.DACNSS = 0;    // VSS
    VREFCON1bits.DACOE = 1;
    VREFCON1bits.DACEN = 1;

//----------------------------------------------------------------------- main()
    while(1){
        dac_value = 0;
        if(mGET_BTN_L()) dac_value +=  4;
        if(mGET_BTN_R()) dac_value +=  8;
        if(mGET_ENC_BTN()) dac_value += 16;

        VREFCON2 = dac_value;
    }
}
//##############################################################################
void Signal_Out(void)
{
    unsigned char signal_idx = 0;
//#define NR_SAMPLES  32
//    unsigned char signal_data[NR_SAMPLES] = {6, 7, 8, 7, 6, 6, 6, 4,22,31,
//                                            0, 3, 6, 6, 8, 9,11,12,11, 8,
//                                            6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6};
//#define NR_SAMPLES  36
//    unsigned char signal_data[NR_SAMPLES] ={18, 20, 23, 25, 27, 29, 30, 31, 31,
//                                            31, 30, 29, 27, 25, 23, 20, 18, 15,
//                                            13, 11, 8, 6, 4, 2, 1, 0, 0,
//                                            0, 1, 2, 4, 6, 8, 11, 13, 16};
#define NR_SAMPLES  72
    unsigned char signal_data[NR_SAMPLES] ={18, 19, 20, 22, 23, 24, 25, 26, 27, 28, 29, 30, 30, 31, 31, 31, 31, 31,
                                            31, 31, 30, 30, 29, 28, 27, 26, 25, 24, 23, 22, 20, 19, 18, 17, 15, 14,
                                            13, 12, 11, 10,  8,  7,  6,  5,  4,  3,  2,  2,  1,  1,  0,  0,  0,  0,
                                             0,  1,  1,  2,  2,  3,  4,  5,  6,  7,  8, 10, 11, 12, 13, 15, 16, 17};
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0; // -> 1MHz (default)

    VREFCON0bits.FVRS = 3;      // 4.096 V
    VREFCON0bits.FVREN = 1;
    VREFCON1bits.DACPSS = 2;    // FVR
    VREFCON1bits.DACNSS = 0;    // VSS
    VREFCON1bits.DACOE = 1;
    VREFCON1bits.DACEN = 1;

//----------------------------------------------------------------------- main()
    while(1){
        VREFCON2 = signal_data[signal_idx];
//        VREFCON2 = signal_idx;
        if(++signal_idx >= NR_SAMPLES) signal_idx = 0;
    }
}

//##############################################################################
void Sound_PWM(void)       //PWM   ###TODO -> volume !
{
    unsigned short frequency = 300;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_4MHZ; OSCTUNEbits.PLLEN = 0;     // -> 4MHz
    GLCD_Init();
    GLCD_Text2Out(0,1,"uC-Quick");
    GLCD_Text2Out(1,3,"Sound");
    GLCD_Value2Out_00(2,1,frequency,5);
    GLCD_Text2Out(2,7,"Hz");

    // encoder setup
    ENC_INT_TRI = INPUT_PIN;  ENC_INT_ANS = DIGITAL_PIN;
    ENC_DIR_TRI = INPUT_PIN;  ENC_DIR_ANS = DIGITAL_PIN;
    mENC_IR_RST(); mENC_IR_EN();

    CCP1CONbits.CCP1M = 0b1100;                             // PWM mode
    CCP1CONbits.P1M = 0;                                    // single
    CCPTMRS0bits.C1TSEL = 0;                                // CCP1-TMR2
    PR2 = 62500 / frequency;                                // 1MHz:16
    CCPR1L = PR2 >> 1;
    SPEAKER_TRI = OUTPUT_PIN;

    OpenTimer2(TIMER_INT_OFF & T2_PS_1_16 & T2_POST_1_1);

    flags.all = 0;

    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        if(flags.encUp){
            if(frequency < 800) {frequency = frequency + 10;}
              else {frequency = frequency + 100;}
            if(frequency > 15600){frequency = 15600;}
            PR2 = (62500 / frequency) -1;             // 1MHz : 16
            CCPR1L = PR2 >> 1;
            GLCD_Value2Out_00(2,1,frequency,5);
            flags.encUp = 0;
        }
        if(flags.encDown){
            if(frequency < 800) {frequency = frequency - 10;}
              else {frequency = frequency - 100;}
            if(frequency < 250) {frequency = 250;}
            PR2 = (62500 / frequency)-1;              // 1MHz : 16
            CCPR1L = PR2 >> 1;
            GLCD_Value2Out_00(2,1,frequency,5);
            flags.encDown = 0;
        }
    }
}

//##############################################################################
/*
#define F_OSC 4000000
void Sound_Compare(void){
    unsigned short frequency = 400;

  -->> see AnalogLED2_COMP()

//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_1MHZ; OSCTUNEbits.PLLEN = 0;     // -> 1MHz
 * GLCD_Init();
    GLCD_Text2Out(0,0,"uC-Quick");
    GLCD_Text2Out(1,0," Sound ");

    ENC_INT_TRI = INPUT_PIN;    // encoder setup
    ENC_DIR_TRI = INPUT_PIN;
#ifdef ENCODER_PANASONIC
    mINIT_ENC_EDGE();
#endif
    mENC_IR_RST();
    mENC_IR_EN();

    CCP1CONbits.CCP1M = 0b1011;                             // Compare sp. event
    CCPTMRS0bits.C1TSEL = 0;                                // CCP1-TMR2
    OpenTimer1(TIMER_INT_OFF & T5_16BIT_RW & T1_SOURCE_FOSC &
               T1_PS_1_1 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF,
               TIMER_GATE_OFF);
    CCPR1 = (F_OSC/2) / frequency;
    SPEAKER_TRI = OUTPUT_PIN;

    flags.all = 0;

    PIR1bits.CCP1IF = 0;
    PIE1bits.CCP1IE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        if(flags.encUp){
            if(frequency < 800) {frequency = frequency + 10;}
              else {frequency = frequency + 100;}
//            if(frequency > 15600){frequency = 15600;}
            if(frequency > 65000){frequency = 65000;}

            CCPR1 = (F_OSC/2) / frequency;
            GLCD_Text2Out(1,0,"      Hz ");      // delete old value
            GLCD_Value2Out(1,1,frequency);
            flags.encUp = 0;
        }
        if(flags.encDown){
            if(frequency < 800) {frequency = frequency - 10;}
              else {frequency = frequency - 100;}
            if(frequency < 250) {frequency = 250;}
            CCPR1 = (F_OSC/2) / frequency;
            GLCD_Text2Out(1,0,"      Hz ");      // delete old value
            GLCD_Value2Out(1,1,frequency);
            flags.encDown = 0;
        }
    }
}
*/

//##############################################################################
void RS232_TX()
{
    unsigned char character = '0';
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_4MHZ; OSCTUNEbits.PLLEN = 0;     // -> 4MHz
    TX_TRI = RX_TRI = INPUT_PIN;

    SPBRG1 = 16;                    // 57600 at 4MHz
    SPBRGH1 = 0;
    TXSTA1bits.BRGH = 1;
    BAUDCON1bits.BRG16 = 1;
    RCSTA1bits.SPEN = 1;
    TXSTAbits.TXEN = 1;

    ENC_BTN_TRI = INPUT_PIN; ENC_BTN_ANS = DIGITAL_PIN;
//----------------------------------------------------------------------- main()
    while(1){
        if(mGET_ENC_BTN()){
            if(++character > 'z')
                character = '0';
            if(TX_FREE)             // if() <-> while() ???
                TXREG1 = character;
            while(mGET_ENC_BTN()){;}  // ??? without this line ???
        }
    }
}

//##############################################################################
void RS232_RX(void)
{
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_4MHZ; OSCTUNEbits.PLLEN = 0;     // -> 4MHz
    mALL_LED_OFF(); mALL_LED_OUTPUT();
    TX_TRI = RX_TRI = INPUT_PIN; RX_ANS = DIGITAL_PIN;

    SPBRG1 = 16;                    // 57600 for 4MHz
    SPBRGH1 = 0;
    TXSTA1bits.BRGH = 1;
    BAUDCON1bits.BRG16 = 1;
    RCSTA1bits.SPEN = 1;
    RCSTA1bits.CREN1 = 1;
//----------------------------------------------------------------------- main()
    while(1){
        unsigned char command;
        if(PIR1bits.RC1IF){
            command = RCREG1;
            switch(command){
                case '0': mSET_LED_1_OFF(); mSET_LED_2_OFF();
                          mSET_LED_3_OFF(); mSET_LED_4_OFF(); break;
                case '1': mSET_LED_1_ON(); break;
                case '2': mSET_LED_2_ON(); break;
                case '3': mSET_LED_3_ON(); break;
                case '4': mSET_LED_4_ON(); break;
                default: break;
            }
        }
    }
}

//##############################################################################
void MiniRS232(void)
{
    unsigned char command;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_4MHZ; OSCTUNEbits.PLLEN = 0;     // -> 4MHz

    mALL_LED_OUTPUT();
    ENC_BTN_TRI = INPUT_PIN; ENC_BTN_ANS = DIGITAL_PIN;
    BTN_L_ANS = BTN_R_ANS = DIGITAL_PIN;

    TX_TRI = RX_TRI = INPUT_PIN; RX_ANS = DIGITAL_PIN;

//#define _XTAL_FREQ  4000000 // in a global header
//#define BAUDRATE    19200   // ""
//#define SPBRG_VAL (((_XTAL_FREQ/BAUDRATE)+32)/64)-1     // BRG16=0, BGH=0 !
//#define SPBRG_VAL (((_XTAL_FREQ/BAUDRATE)+8)/16)-1      // BRG16=0, BGH=1 !
#define SPBRG_VAL (((_XTAL_FREQ/BAUDRATE)+2)/4)-1       // BRG16=1, BGH=1 !

    baud1USART(BAUD_IDLE_RX_PIN_STATE_HIGH &
                 BAUD_IDLE_TX_PIN_STATE_HIGH &
                 BAUD_16_BIT_RATE &
                 BAUD_WAKEUP_OFF &
                 BAUD_AUTO_OFF);

    Open1USART( USART_TX_INT_OFF & USART_RX_INT_OFF &
                USART_ASYNCH_MODE & USART_EIGHT_BIT &
                USART_CONT_RX &
                USART_BRGH_HIGH & USART_ADDEN_OFF,
                SPBRG_VAL );
//----------------------------------------------------------------------- main()
    while(1){
        if(DataRdy1USART()){
            command = RCREG1;
            switch(command){
                case '0': mALL_LED_OFF(); break;
                case '1': mSET_LED_1_OFF(); break;
                case '2': mSET_LED_2_OFF(); break;
                case '3': mSET_LED_3_OFF(); break;
                case '4': mSET_LED_4_OFF(); break;
                case '5': mSET_LED_1_ON(); break;
                case '6': mSET_LED_2_ON(); break;
                case '7': mSET_LED_3_ON(); break;
                case '8': mSET_LED_4_ON(); break;
                case '9': mALL_LED_ON(); break;
                case '?':
                    BTN_L_TRI = BTN_R_TRI = INPUT_PIN;
                    __delay_us(10);
                    command = mGET_BTN_L();
                    command += 2 * mGET_ENC_BTN();
                    command += 4 * mGET_BTN_R();
                    if(command < 10)command += '0';
                    else command += 'A' - 10;
                    TXREG1 = command;
                    mALL_LED_OUTPUT();
                    break;
                default: break;
            }//switch(command){
        }//if(DataRdy1USART()){
    }//while(1){
}

//##############################################################################
void TimeMeasure(void)         // CCP1 => RC2
{
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_4MHZ; OSCTUNEbits.PLLEN = 0;     // -> 4MHz

    TRISCbits.TRISC2 = INPUT_PIN; ANSELCbits.ANSC2 = DIGITAL_PIN;

    GLCD_Init();
    GLCD_Text2Out(0,1,"uC-Quick");
    GLCD_Text2Out(1,1,"ZeitMess");
    GLCD_Text2Out(2,7,"us");

    OpenTimer1(TIMER_INT_OFF & T5_16BIT_RW & T1_SOURCE_FOSC_4 &
               T1_PS_1_1 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF,
               TIMER_GATE_OFF);
    OpenECapture1(CAPTURE_INT_ON & ECAP_EVERY_RISE_EDGE & ECCP_1_SEL_TMR12);

    flags.all = 0;

    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        if(flags.newCapture){
            GLCD_Text2Out(2,6," ");     // (if value changed from - to +)
            GLCD_Value2Out_00(2,1,capture_value,5);
            flags.newCapture = 0;
        }
    }
}

//##############################################################################
typedef enum{
    S_START,
    S_OFF,
    S_ON
}State_t;

void StartStopMenu(void)
{
    State_t state = S_START;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_4MHZ; OSCTUNEbits.PLLEN = 0;     // -> 4MHz

    ENC_BTN_TRI = INPUT_PIN; ENC_BTN_ANS = DIGITAL_PIN;
//    SPEAKER_TRI = OUTPUT_PIN;------controlled later in the appropriate states

    LATAbits.LATA0 = INPUT_PIN; ANSELAbits.ANSA0 = ANALOG_PIN;

    OpenADC(ADC_FOSC_8 & ADC_LEFT_JUST & ADC_12_TAD,
            ADC_POTI & ADC_INT_OFF,
            ADC_TRIG_CCP5 & ADC_REF_VDD_VDD & ADC_REF_VDD_VSS);

// set up PWM out and analog in
    ADCON0bits.GO = 1; while (ADCON0bits.NOT_DONE) {;}
    if (ADRESH >= 4) PR2 = ADRESH;
    else PR2 = 4;

    PR2 = 127;

    CCPR1L = PR2 >> 1;
    CCP1CONbits.CCP1M = 0b1100;                             // PWM mode
    CCP1CONbits.P1M = 0;                                    // single
    CCPTMRS0bits.C1TSEL = 0;                                // CCP1-TMR2
    OpenTimer2(TIMER_INT_OFF & T2_PS_1_16 & T2_POST_1_1);

    GLCD_Init();
    GLCD_Text2Out(1,2, "PWR_ON");
    GLCD_Text2Out(2,1,"-->  <--");

    while(!mGET_ENC_BTN()){;} while(mGET_ENC_BTN()){;}
    
    GLCD_Text2Out(0,1, "ADC: ---");
    GLCD_Text2Out(1,2," start");
    state = S_OFF;

//----------------------------------------------------------------------- main()
    while(1){
#ifndef DEMO_START_STOP_IPO   // in demo_functions.h
        switch(state){
            case S_OFF:
                GLCD_Text2Out(0,6,"---");
                GLCD_Text2Out(1,2," start");
                SPEAKER_TRI = INPUT_PIN;
                while(mGET_ENC_BTN()){;}        // ensure released
                do{;}while(!mGET_ENC_BTN());    // nothing until pressed again
                state = S_ON;
                break;
            case S_ON:
                GLCD_Text2Out(1,3,"stop ");
                SPEAKER_TRI = OUTPUT_PIN;
                while(mGET_ENC_BTN()){;}        // ensure released
                do{
                    ADCON0bits.GO = 1; while(ADCON0bits.NOT_DONE){;}
                    GLCD_Value2Out_00(0,6,ADRESH,3);
                    if(ADRESH >= 4){
                        PR2 = ADRESH;
                        CCPR1L = PR2 >> 1;
                    }
                }while(!mGET_ENC_BTN());        // until btn pressed again
                state = S_OFF;
                break;
            default: break;
        }
#else
// DEMO_START_STOP_IPO
        if(mGET_ENC_BTN()){ //------------------------ state transitions
            switch(state){
              case S_OFF:
                state = S_ON;                       // next state ON
                SPEAKER_TRI = OUTPUT_PIN;           // switch on sound
                GLCD_Text2Out(1,3,"stop ");
                break;
              case S_ON:
                state = S_OFF;                      // next state OFF
                SPEAKER_TRI = INPUT_PIN;            // switch off sound
                GLCD_Text2Out(0,6,"---");
                GLCD_Text2Out(1,2," start");
                break;
              default: break;
            }
            while(mGET_ENC_BTN());
        }
        switch(state){ //---------------------------- state actions
          case S_OFF:
            break;
          case S_ON:
            ADCON0bits.GO = 1; while(ADCON0bits.NOT_DONE){;}
            if(ADRESH >= 4){
                GLCD_Value2Out_00(0,6,ADRESH,3);
                PR2 = ADRESH;
                CCPR1L = PR2 >> 1;
            }
            break;
          default: break;
        }
#endif
    }
}


//######################################################################## Clock
void setHours(unsigned char* hr);
void setMinutes(unsigned char* min);
void setSeconds(unsigned char* sec);
void Clock(unsigned char* hr, unsigned char* min,unsigned char* sec);

typedef enum{
    CM_HR = 0,
    CM_MIN,
    CM_SEC,
    CM_CLOCK
}CLOCKMENU;

void ClockMenu(void)
{
    CLOCKMENU menu;
    unsigned char hours, minutes, seconds;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_2MHZ; OSCTUNEbits.PLLEN = 0;     // -> 2MHz

    // encoder setup
    ENC_INT_TRI = INPUT_PIN;  ENC_INT_ANS = DIGITAL_PIN;
    ENC_DIR_TRI = INPUT_PIN;  ENC_DIR_ANS = DIGITAL_PIN;
    ENC_BTN_TRI = INPUT_PIN;  ENC_BTN_ANS = DIGITAL_PIN;

    OpenTimer1(TIMER_INT_OFF & T5_16BIT_RW & T1_SOURCE_FOSC_4 &
               T1_PS_1_8 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF,
               TIMER_GATE_OFF);

    CCPTMRS0bits.C2TSEL  = 0;       // timer <-> ccp module (CCP2 / TMR1)
    CCPR2 = 62500;                  // Fosc/4 / prescaler  = 500kHz /8
    CCP2CONbits.CCP2M = 0b1011;     // Compare Mode with Special Event Trigger

    flags.all = 0;
    hours = minutes = seconds = 0;
    menu = CM_HR;                   // Begin with adjusting hours

    GLCD_Init();
    GLCD_Text2Out(0,0,"hrs. +- ");
    GLCD_Text2Out(1,0,"00:00:00");

    mENC_IR_RST();
    mENC_IR_EN();
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        switch (menu){
            case CM_HR:
                setHours(&hours);
                menu++;
                break;
            case CM_MIN:
                setMinutes(&minutes);
                menu++;
                break;
            case CM_SEC:
                setSeconds(&seconds);
                menu++;
                break;
            case CM_CLOCK:
                mENC_IR_DIS();
                mSEC_IR_EN();
                Clock(&hours,&minutes,&seconds);
                mSEC_IR_DIS();
                mENC_IR_EN();
                menu = CM_HR;
                break;
            default:
                menu = CM_CLOCK;
                break;
        }
    }
}//------------------------------------------------------------------------Clock
void setHours(unsigned char* hr)
{
    GLCD_Text2Out(0,0,"hrs. +- ");
    while(!mGET_ENC_BTN()){
        if(flags.encUp){
            if(++*hr >= 24) *hr = 0;
            GLCD_Value2Out_00(1,0,*hr,2);
            flags.encUp = 0;
        }
        if(flags.encDown){
            if(--*hr >= 24) *hr = 23;
            GLCD_Value2Out_00(1,0,*hr,2);
            flags.encDown = 0;
        }
    }
    while(mGET_ENC_BTN()){;}
}//------------------------------------------------------------------------Clock
void setMinutes(unsigned char* min)
{
    GLCD_Text2Out(0,0,"min. +- ");
    while(!mGET_ENC_BTN()){
        if(flags.encUp){
            if(++*min >= 59) *min = 0;
            GLCD_Value2Out_00(1,3,*min,2);
            flags.encUp = 0;
        }
        if(flags.encDown){
            if(--*min >= 59) *min = 59;
            GLCD_Value2Out_00(1,3,*min,2);
            flags.encDown = 0;
        }
    }
    while(mGET_ENC_BTN()){;}
}//------------------------------------------------------------------------Clock
void setSeconds(unsigned char* sec)
{
    GLCD_Text2Out(0,0,"sec. +- ");
    while(!mGET_ENC_BTN()){
        if(flags.encUp){
            if(++*sec >= 59) *sec = 0;
            GLCD_Value2Out_00(1,6,*sec,2);
            flags.encUp = 0;
        }
        if(flags.encDown){
            if(--*sec >= 59) *sec = 59;
            GLCD_Value2Out_00(1,6,*sec,2);
            flags.encDown = 0;
        }
    }
    while(mGET_ENC_BTN()){;}
}//------------------------------------------------------------------------Clock
void Clock(unsigned char* hr, unsigned char* min, unsigned char* sec)
{
    GLCD_Text2Out(0, 0, "Time:   ");
    while (!mGET_ENC_BTN()) {
        if (flags.newSec) {
            if (++*sec >= 60) {
                *sec = 0;
                if (++*min >= 60) {
                    *min = 0;
                    if (++*hr >= 24) {
                        *hr = 0;
                    }
                    GLCD_Value2Out_00(1, 0, *hr, 2);
                }
                GLCD_Value2Out_00(1, 3, *min, 2);
            }
            GLCD_Value2Out_00(1, 6, *sec, 2);
            flags.newSec = 0;
        }
    }
    while (mGET_ENC_BTN()) {;}
}//--------------------------------------------------------------------end Clock

//##############################################################################

extern const char hsulmLogo47x5[];
void BoardTest(void)
{
    unsigned short frequency = 250;

//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_4MHZ; OSCTUNEbits.PLLEN = 0;     // -> 4MHz

    GLCD_Init();
    GLCDputrbm_XY(5,47, &hsulmLogo47x5[0], 0, 18);
    GLCD_TextOut(5,0, "THU"); GLCD_TextOut(5,11, "THU");
    GLCD_TextOut(0,0, "THU"); GLCD_TextOut(0,11, "THU");

    mALL_LED_OUTPUT();

    // encoder setup
    ENC_INT_TRI = ENC_DIR_TRI = ENC_BTN_TRI = INPUT_PIN;
    ENC_INT_ANS = ENC_DIR_ANS = ENC_BTN_ANS = DIGITAL_PIN;

    mENC_IR_RST();
    mENC_IR_EN();

    BTN_L_ANS = BTN_R_ANS = DIGITAL_PIN;

//PWM --------------------------------------------------------------------------
    CCP1CONbits.CCP1M = 0b1100;                             // PWM mode
    CCP1CONbits.P1M = 0;                                    // single
    CCPTMRS0bits.C1TSEL = 0;                                // CCP1-TMR2
    OpenTimer2(TIMER_INT_OFF & T2_PS_1_16 & T2_POST_1_1);
//    OpenTimer2(TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1);
    PR2 = 62500 / frequency;                                // 1MHz:16
    CCPR1L = PR2 >> 1;
    SPEAKER_TRI = OUTPUT_PIN;

//ADC --------------------------------------------------------------------------
    POTI_TRI = INPUT_PIN; POTI_ANS = ANALOG_PIN;

    OpenADC(ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_12_TAD,
            ADC_POTI & ADC_INT_ON,
            ADC_TRIG_CCP5 & ADC_REF_VDD_VDD & ADC_REF_VDD_VSS);

    OpenTimer5(TIMER_INT_OFF & T5_16BIT_RW & T5_SOURCE_FOSC_4 &
               T5_PS_1_2 & T5_OSC1EN_OFF & T5_SYNC_EXT_OFF,
               TIMER_GATE_OFF);

    CCPTMRS1bits.C5TSEL  = 2;   // timer <-> ccp module (CCP5 / TMR5)
    CCPR5 = 50000;              // Fosc/4 / prescaler / Fadc = 1MHz /2 /10Hz
    CCP5CONbits.CCP5M = 0b1011; // Compare Mode with Special Event Trigger

//USART ------------------------------------------------------------------------
    TX_TRI = RX_TRI = INPUT_PIN; RX_ANS = DIGITAL_PIN;

//#define _XTAL_FREQ  4000000 // in a global header
//#define BAUDRATE    19200   // ""
//#define SPBRG_VAL (((_XTAL_FREQ/BAUDRATE)+32)/64)-1     // BRG16=0, BGH=0 !
//#define SPBRG_VAL (((_XTAL_FREQ/BAUDRATE)+8)/16)-1      // BRG16=0, BGH=1 !
#define SPBRG_VAL (((_XTAL_FREQ/BAUDRATE)+2)/4)-1       // BRG16=1, BGH=1 !

    baud1USART(BAUD_IDLE_RX_PIN_STATE_HIGH &
                 BAUD_IDLE_TX_PIN_STATE_HIGH &
                 BAUD_16_BIT_RATE &
                 BAUD_WAKEUP_OFF &
                 BAUD_AUTO_OFF);

    Open1USART( USART_TX_INT_OFF & USART_RX_INT_OFF &
                USART_ASYNCH_MODE & USART_EIGHT_BIT &
                USART_CONT_RX &
                USART_BRGH_HIGH & USART_ADDEN_OFF,
                SPBRG_VAL );

    while(!mGET_ENC_BTN()){
        if(DataRdy1USART()){TXREG1 = RCREG1 + 1;}
    }
    GLCD_Clear();
    GLCD_Text2Out(0,0,"ADC ??? ");
    GLCD_Text2Out(1,0,"PWM ??? ");

    flags.all = 0;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

//----------------------------------------------------------------------- main()
    while(1) {
    //----------------------------------------------------- check buttons & LEDs
        while (mGET_ENC_BTN()) {
            BTN_L_TRI = INPUT_PIN; __delay_us(10);
            if (mGET_BTN_L()) {
                while (mGET_BTN_L()) {;}
                mTOG_LED_1();
                mTOG_LED_2();
            }
            LED_1_TRI = OUTPUT_PIN;

            BTN_R_TRI = INPUT_PIN; __delay_us(10);
            if (mGET_BTN_R()) {
                while (mGET_BTN_R()) {;}
                mTOG_LED_3();
                mTOG_LED_4();
            }
            LED_3_TRI = OUTPUT_PIN;
        }
    //------------------------------------------------------ check encoder & PWM
        if(flags.encUp){
            if(frequency < 800) {frequency = frequency + 10;}
              else {frequency = frequency + 100;}
            if(frequency > 15600){frequency = 15600;}
            PR2 = (62500 / frequency) -1;   // 1MHz : 16
            CCPR1L = PR2 >> 1;
            GLCD_Text2Out(1,4,"    ");   // delete old value
            GLCD_Value2Out(1,4,frequency);
            flags.encUp = 0;
        }
        if(flags.encDown){
            if(frequency < 800) {frequency = frequency - 10;}
              else {frequency = frequency - 100;}
            if(frequency < 250) {frequency  = 250;}
            PR2 = (62500 / frequency)-1;    // 1MHz : 16
            CCPR1L = PR2 >> 1;
            GLCD_Text2Out(1,4,"    ");   // delete old value
            GLCD_Value2Out(1,4,frequency);
            flags.encDown = 0;
        }
    //---------------------------------------------------------------- check ADC
        if(flags.newADC){
            GLCD_Text2Out(0,4,"    ");   // delete old value
            GLCD_Value2Out(0,4,adc_value);
            flags.newADC = 0;
        }
    //-------------------------------------------------------------- check USART
        if(DataRdy1USART()){
            TXREG1 = RCREG1 + 1;            // 'a' -> 'b' ...
        }
    }
}

//##############################################################################
#define TOGGLE_LED() switch(led_nr){                        \
                        case 1: mTOG_LED_1();break;         \
                        case 2: mTOG_LED_2();break;         \
                        case 3: mTOG_LED_3();break;         \
                        case 4: mTOG_LED_4();break;         \
                     }

void MacroTest(void){
    unsigned char led_nr = 1;
//--------------------------------------------------------------------- __init()
    OSCCONbits.IRCF = IRCF_4MHZ; OSCTUNEbits.PLLEN = 0;     // -> 4MHz

    GLCD_Init();
    GLCD_Text2Out(0,1,"MCON-LAB");
    GLCD_Text2Out(1,1,"MacrTest");

    mALL_LED_OUTPUT();

    // encoder setup
    ENC_INT_TRI = INPUT_PIN;  ENC_INT_ANS = DIGITAL_PIN;
    ENC_DIR_TRI = INPUT_PIN;  ENC_DIR_ANS = DIGITAL_PIN;
    ENC_BTN_TRI = INPUT_PIN;  ENC_BTN_ANS = DIGITAL_PIN;

    mENC_IR_RST();
    mENC_IR_EN();

    BTN_L_ANS = BTN_R_ANS = DIGITAL_PIN;

    T0CON = 0b10000010;         // TMR0ON T08BIT T0CS T0SE PSA T0PS2 T0PS1 T0PS0

    flags.all = 0;
    INTCONbits.GIE = 1;
//----------------------------------------------------------------------- main()
    while(1){
        if(flags.encUp){
            if(led_nr < 4){ led_nr++; }
            flags.encUp = 0;
        }
        if(flags.encDown){
            if(led_nr > 0){ led_nr--; }
            flags.encDown = 0;
        }
        if(INTCONbits.TMR0IF){      // if timer 0 overflow
            TOGGLE_LED();               //   then toggle LED
            INTCONbits.TMR0IF = 0;      //   and clear flag
        }
    }
}
