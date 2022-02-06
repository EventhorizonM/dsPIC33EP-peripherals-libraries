
/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__dsPIC33E__)
    	#include <p33Exxxx.h>
    #elif defined(__dsPIC33F__)
    	#include <p33Fxxxx.h>
    #endif
#endif

#include "System.h"        /* System funct/params, like osc/peripheral config */

#include "LibPorts.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
unsigned int Value1,Value2;

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int main(void)
{
    SetUp_OSC();

    InitAnalogPinsPortB(All_PINS_DIG & RB6AN & RB7AN);

    InitPinA0(ANA_IN,NO_PULL,CN_DIS);
    InitPinA1(DIG_OUT_OD,PULL_UP,CN_DIS);
    InitPinA2(DIG_IN,PULL_DN,CN_EN);

    InitPinK0(ANA_IN,NO_PULL,CN_DIS);
    InitPinK1(DIG_OUT_OD,PULL_UP,CN_DIS);
    InitPinK12(DIG_IN,PULL_DN,CN_EN);

    PPS_Unlock();
    PPS_Mapping(CONNECT_RP64_TO, SDI1_INPUT);
    PPS_Mapping(CONNECT_RP64_TO, SDO1_OUTPUT);
    PPS_Mapping(CONNECT_RP67_TO, U1TX_OUTPUT);
    PPS_Mapping(CONNECT_RP100_TO, IC7_INPUT);
    PPS_Mapping(CONNECT_INTERNAL_C3OUT_TO, FLT1_INPUT);
    PPS_Mapping(CONNECT_INTERNAL_VSS_TO, U1CTS_INPUT);
    PPS_Lock();

    while(1)
    {
        SetPORTA(0xFFFF);
        SetPinB1(1);
        Nop();
        Value1 = ReadPORTA();
        Value2 = ReadPinB1();
    }

}

