/*
 * File:   demo_functions.h
 * Author: VSK
 *
 * Created on 18. July 2012, 10:03
 */

#ifndef DEMO_FUNCTIONS_H
#define DEMO_FUNCTIONS_H

//####################################################################
//#            please select one of the following demos              #
//# (if you select more than one -> the first one will be executed)  #
//####################################################################

//#define DEMO_BOARD_TEST         // function test (all) components

//#define DEMO_TEMPLATE_CODE      // Template Code from uC_Quick-X.pdf
//#define DEMO_TEST_LEDS          // LEDs in Debug-Mode (single step) ON/OFF
//#define DEMO_BTN_LEDS           // encoder button switch on LED_1/2/3/4
//#define DEMO_BTN_RANDOM_LEDS    // encoder button switches LEDs randomly
//#define DEMO_BTN_TOGGLE_LEDS    // encoder button toggles LEDs
//#define DEMO_SWITCH_IOS         // buttons L/R toggle the LEDs
//#define DEMO_IPO_BTN_LED        // IPO buttons L/R toggle LEDs
//#define DEMO_TIME_SLICED_IPO    // time sliced Input-Process-Output
//#define DEMO_TTL_ST_TO_LED      // determining of logic levels
//#define DEMO_BLINK_COUNTER      // timing based on counter loop
//#define DEMO_BLINK_TMR_FLAG     // timing using timer0 module (polling flag)
//#define DEMO_BLINK_TMR_IR       // timing using timer0 module (interrupt)
//#define DEMO_ENCODER_LEDS       // encoder "rotates" LEDs
//#define DEMO_ENCODER_LCD_TEXT   // encoder controls text on LCD Display
//#define DEMO_ENCODER_LCD_VALUE  // encoder controls values on LCD Display
//#define DEMO_ENCODER_POLLING    // encoder signals polled during timer IR
//#define DEMO_PWM_COUNTER        // a counter is incremented in main loop
//#define DEMO_PWM_CCP_PWM        // using PWM mode of CCP module
//#define DEMO_PWM_CCP_COMPARE    // using compare mode of CCP module
//#define DEMO_ANALOG_LEDS        // ADC (poti) display at LEDs
//#define DEMO_ANALOG_LEDS_PWM    // ADC (poti) -> LED intensity
//#define DEMO_ANALOG_LED2_COMP   // ADC -> brightness of LED2 via CCP compare
//#define DEMO_ANALOG_LED2_PWM    // ADC -> brightness of LED2 via CCP pwm (###not yet implemented for XC8###)
//#define DEMO_ANALOG_LCD         // ADC (poti) -> LCD
//#define DEMO_ANALOG_OUT         // Analog signal out (buttons)
//#define DEMO_ANALOG_SIGNAL_OUT  // Analog signal out (array)
//#define DEMO_SOUND_PWM          // sound out at speaker using PWM module
//#define DEMO_RS232_TX           // Send serial data
//#define DEMO_RS232_RX           // Receive serial data
//#define DEMO_MINI_RS232         // Serial Communication (both Directions)
//#define DEMO_TIME_MEASURE       // measure time between two pos. edges [us]
//#define DEMO_START_STOP_MENU    // simple start-stop state machine
//#define DEMO_START_STOP_IPO     // start-stop state machine IPO model
//#define DEMO_CLOCK_MENU         // LCD Clock with menu for setting
#define DEMO_MAKRO_TEST         //

////TODO    Reaktionstest();        // Blink_Counter-> game
////TODO    Blink_Interrupt();      // timing with timer and CCP modules (IR)
////TODO    Sound_Compare();        //
////TODO    FreqMess();             // measure frequency (1s)
////TODO    FunctionPointers();

#if defined(DEMO_BOARD_TEST)
    #define DEMO_FUNCTION()     BoardTest()
    #define USE_ENCODER_IR
    #define USE_ADC_IR
    #define _XTAL_FREQ          4000000
    #define BAUDRATE            19200

#elif defined(DEMO_TEMPLATE_CODE)
    #define DEMO_FUNCTION()     TemplateCode()
#elif defined(DEMO_TEST_LEDS)
    #define DEMO_FUNCTION()     TestLEDs()
#elif defined(DEMO_BTN_LEDS)
    #define DEMO_FUNCTION()     ButtonLEDs()
#elif defined(DEMO_BTN_RANDOM_LEDS)
    #define DEMO_FUNCTION()     ButtonRandomLEDs()
#elif defined(DEMO_BTN_TOGGLE_LEDS)
    #define DEMO_FUNCTION()     ButtonToggleLEDs()
#elif defined(DEMO_SWITCH_IOS)
    #define DEMO_FUNCTION()     SwitchIOs()
    #define _XTAL_FREQ          1000000
#elif defined(DEMO_IPO_BTN_LED)
    #define DEMO_FUNCTION()     IPO_btn_led()
    #define _XTAL_FREQ          1000000
#elif defined(DEMO_TIME_SLICED_IPO)
    #define DEMO_FUNCTION()     TimeSliced_IPO()
    #define _XTAL_FREQ          1000000
#elif defined(DEMO_TTL_ST_TO_LED)
    #define DEMO_FUNCTION()     TTL_ST_toLED()
#elif defined(DEMO_BLINK_COUNTER)
    #define DEMO_FUNCTION()     Blink_Counter()
    #define _XTAL_FREQ          31250
#elif defined(DEMO_BLINK_TMR_FLAG)
    #define DEMO_FUNCTION()     Blink_Timer_Flag()
    #define _XTAL_FREQ          4000000
#elif defined(DEMO_BLINK_TMR_IR)
    #define DEMO_FUNCTION()     Blink_Timer_IR()
    #define USE_TMR0_IR
    #define _XTAL_FREQ          4000000
#elif defined(DEMO_ENCODER_LEDS)
    #define DEMO_FUNCTION()     EncoderLEDs()
    #define USE_ENCODER_IR
#elif defined(DEMO_ENCODER_LCD_TEXT)
    #define DEMO_FUNCTION()     EncoderLCD_Text()
    #define USE_ENCODER_IR
    #define _XTAL_FREQ          1000000
#elif defined(DEMO_ENCODER_LCD_VALUE)
    #define DEMO_FUNCTION()     EncoderLCD_Value()
    #define USE_ENCODER_IR
    #define _XTAL_FREQ          1000000
#elif defined(DEMO_ENCODER_POLLING)
    #define DEMO_FUNCTION()     EncoderPolling()
    #define USE_ENCODER_POLLING_IR
    #define _XTAL_FREQ          16000000
#elif defined(DEMO_PWM_COUNTER)
    #define DEMO_FUNCTION()     PWMcounter()
    #define USE_ENCODER_IR
    #define _XTAL_FREQ          1000000
#elif defined(DEMO_PWM_CCP_PWM)
    #define DEMO_FUNCTION()     PWMccpPWM()
    #define USE_ENCODER_IR
    #define _XTAL_FREQ          1000000
#elif defined(DEMO_PWM_CCP_COMPARE)
    #define DEMO_FUNCTION()     PWMccpCompare()
    #define USE_ENCODER_IR
    #define _XTAL_FREQ          32000000
#elif defined(DEMO_ANALOG_LEDS)
    #define DEMO_FUNCTION()     AnalogLEDs()
    #define _XTAL_FREQ          1000000
#elif defined(DEMO_ANALOG_LEDS_PWM)
    #define DEMO_FUNCTION()     AnalogLEDs_PWM()
    #define _XTAL_FREQ          1000000
#elif defined(DEMO_ANALOG_LED2_PWM)
    #ERROR: for XC8 not yet implemented
    #define DEMO_FUNCTION()     AnalogLED2_PWM()
    #define _XTAL_FREQ          1000000
#elif defined(DEMO_ANALOG_LED2_COMP)
    #define DEMO_FUNCTION()     AnalogLED2_COMP()
    #define _XTAL_FREQ          32000000
#elif defined(DEMO_ANALOG_LCD)
    #define DEMO_FUNCTION()     AnalogLCD()
    #define USE_ADC_IR
    #define USE_SECONDS_IR
    #define _XTAL_FREQ          1000000
#elif defined(DEMO_ANALOG_OUT)
    #define DEMO_FUNCTION()     Analog_Out()
    #define _XTAL_FREQ          1000000
    #warning "###########################################################"
    #warning "DACOUT is connected to PORT_A_2 - remove jumper JP4 please!
    #warning "###########################################################"
#elif defined(DEMO_ANALOG_SIGNAL_OUT)
    #define DEMO_FUNCTION()     Signal_Out()
    #warning "###########################################################"
    #warning "DACOUT is connected to PORT_A_2 - remove jumper JP4 please!
    #warning "###########################################################"
#elif defined(DEMO_SOUND_PWM)
    #define DEMO_FUNCTION()     Sound_PWM()
    #define USE_ENCODER_IR
    #define _XTAL_FREQ          4000000
#elif defined(DEMO_RS232_TX)
    #define DEMO_FUNCTION()     RS232_TX()
    #define _XTAL_FREQ          4000000
#elif defined(DEMO_RS232_RX)
    #define DEMO_FUNCTION()     RS232_RX()
    #define _XTAL_FREQ          4000000
#elif defined(DEMO_MINI_RS232)
    #define DEMO_FUNCTION()     MiniRS232()
    #define _XTAL_FREQ          4000000
    #define BAUDRATE            19200
#elif defined(DEMO_TIME_MEASURE)
    #define DEMO_FUNCTION()     TimeMeasure()
    #define USE_CAPTURE_IR
    #define _XTAL_FREQ          4000000
    #warning "############################################################"
    #warning "### Connect a signal to PORTC_2 please (JP4_2 -> JP11_1) ###"
    #warning "############################################################"
#elif defined(DEMO_START_STOP_MENU)||defined(DEMO_START_STOP_IPO)
    #define DEMO_FUNCTION()     StartStopMenu()
    #define _XTAL_FREQ          4000000
#elif defined(DEMO_CLOCK_MENU)
    #define DEMO_FUNCTION()     ClockMenu()
    #define USE_ENCODER_IR
    #define USE_SECONDS_IR
    #define _XTAL_FREQ          2000000
#elif defined(DEMO_MAKRO_TEST)
    #define DEMO_FUNCTION()     MacroTest()
    #define USE_ENCODER_IR
//#elif defined(DEMO_)
//    #define DEMO_FUNCTION()
#endif

#ifndef _XTAL_FREQ
    #define _XTAL_FREQ 1000000  // default value for 18FxxK22
#endif
#ifndef BAUDRATE
    #define BAUDRATE    19200
#endif

union DemoFlags{
    char all;

    struct {
        unsigned LN : 4;
        unsigned HN : 4;
    };

    struct {
        unsigned encUp      : 1;
        unsigned encDown    : 1;
        unsigned encCERR    : 1;    // code error
        unsigned encOERR    : 1;    // overflow error
        unsigned newADC     : 1;
        unsigned newCapture : 1;
        unsigned newSec     : 1;
        unsigned firstLoop  : 1;
    };
};

// IR used from timers/ccp modules
//  T2          -> Blink LED ~1Hz   (TMR0_IR)
//  T5/CCP5     -> ADC auto (ADC IR)
//  T2/CCP1     -> PWM sound (no IR))
//  T1/ECCP1    -> Timer measurement via capture (CCP1_IR)
//  T1/CCP2     -> Timer Clock (CCP2_IR)
//  T1/CCP3     -> Input polling (CCP3_IR)
//
// for encoder IR
//  INT1

// ---------------------------------------------------------time measurement CCP
#define CAPTR_IR        PIE1bits.CCP1IE && PIR1bits.CCP1IF
#define mCAPTR_IR_EN()  PIE1bits.CCP1IE = 1
#define mCAPTR_IR_DIS() PIE1bits.CCP1IE = 0
#define mCAPTR_IR_CLR() PIR1bits.CCP1IF = 0

// --------------------------------------------------------------------clock CCP
#define SEC_IR          PIE2bits.CCP2IE && PIR2bits.CCP2IF
#define mSEC_IR_EN()    PIE2bits.CCP2IE = 1
#define mSEC_IR_DIS()   PIE2bits.CCP2IE = 0
#define mSEC_IR_CLR()   PIR2bits.CCP2IF = 0

// ------------------------------------------------------------ input polling IR
#define INP_POLL_IR         PIE4bits.CCP3IE && PIR4bits.CCP3IF
#define mINP_POLL_IR_CLR()  PIR4bits.CCP3IF = 0
#define mINP_POLL_IR_EN()   mINP_POLL_IR_CLR(); PIE4bits.CCP3IE = 1
#define mINP_POLL_IR_DIS()  PIE4bits.CCP3IE = 0


extern union DemoFlags flags;
extern unsigned short adc_value;
extern unsigned short capture_value, old_capture;

extern void TemplateCode(void);     // Template Code from uC_Quick-X.pdf
extern void TestLEDs(void);         // LEDs in Debug-Mode (single step) ON/OFF
extern void ButtonLEDs(void);       // encoder button switches on LED_1/2/3/4
extern void ButtonRandomLEDs(void); // encoder button switches LEDs randomly
extern void ButtonToggleLEDs(void); // encoder button toggles LEDs
extern void SwitchIOs(void);        // buttons L/R toggle LEDs
extern void IPO_btn_led(void);      // IPO buttons L/R toggle LEDs
extern void TimeSliced_IPO(void);   // time sliced IPO
extern void TTL_ST_toLED(void);     // determining of logic levels
extern void Blink_Counter(void);    // timing based on  counter loop
//extern void Reaktionstest(void);  // Blink_Counter-> game
extern void Blink_Timer_Flag(void); // timing based on Timer 0 (polling flag)
extern void Blink_Timer_IR(void);   // timing based on Timer 0 (interrupt()
//extern void Blink_Interrupt(void);// timing based on Timer and CCP-Interrupt
extern void EncoderLEDs(void);      // encoder rotates LEDs
extern void EncoderLCD_Text(void);  // encoder controls text on LCD Display
extern void EncoderLCD_Value(void); // encoder controls values on LCD Display
extern void EncoderPolling(void);   // encoder signals polled during timer IR
extern void PWMcounter(void);
extern void PWMccpPWM(void);
extern void PWMccpCompare(void);
extern void AnalogLEDs(void);       // ADC of Poti display at LEDs
extern void AnalogLEDs_PWM(void);   // ADC (poti) -> LED intensity
extern void AnalogLED2_PWM(void);   // ADC -> brightness of LED via CCP pwm
extern void AnalogLED2_COMP(void);  // ADC -> brightness of LED via CCP compare
extern void AnalogLCD(void);        // ADC of Poti at LCD Display
extern void Analog_Out(void);       // analog signal output (buttons)
extern void Signal_Out(void);       // analog signal from array
extern void AnalogLEDs_PWM(void);   // PWM signal (counter) -> brightness LEDs
extern void Sound_PWM(void);        // Sound at speaker PWM generated
//extern void Sound_Compare(void);
extern void RS232_TX(void);         // send serial data
extern void RS232_RX(void);         // receive serial data
extern void MiniRS232(void);        // bidirectional serial communication
extern void TimeMeasure(void);      // time measurement between two neg. edges

extern void StartStopMenu(void);    // simple Start-Stop state machine
extern void ClockMenu(void);        // LCD Clock with setting menu

extern void BoardTest(void);        // check (all) board functions

extern void MacroTest(void);

#endif	/* DEMO_FUNCTIONS_H */
