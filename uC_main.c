//##############################################################################
//      filename:        uC-Quick_demo_main.c
//
//      main file for demo projects
//
//##############################################################################
//
//  Author:             V.SchK
//  Company:            THU-Ulm
//
//  Revision:           3.0 (XC8 C99 version)
//  Date:               May 2019
//  Assembled using     XC8 2.00+
//
//  todo    - add comments ;-)
//          -
//
//##############################################################################

#include "uCQuick/uCQ_2013.h"
#include "LCD/LCD_lib_busy.h"
#include "demo_functions.h"

//--- P R I V A T E   P R O T O T Y P E S --------------------------------------
void __init(void);

//##############################################################################
// Function:        void main(void)
//                      called from the startup code
// PreCondition:    None
// Input:
// Output:
// Side Effects:
// Overview:        Es kann jeweils nur eine Funktion in main ausgefuehrt werden
//                  da die Demofunktionen komplett eigenstaendige Programme mit
//                  eigener Initialisierung und while(1){...} Schleife sind
//##############################################################################
void main(void)
{
//    __init();

    DEMO_FUNCTION();        // demo function is selected in demo_functions.h

//    while(1);
}

//##############################################################################
// Function:        void __init(void)
//                      usually called from main()
// PreCondition:    None
// Input:
// Output:
// Side Effects:
// Overview:
//##############################################################################
void __init(void)
{
    // everything done in demo functions init section now...
}
