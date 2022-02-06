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


#include <stdint.h>			/* Includes uint16_t definition */
#include <libpic30.h>			/* for __delay32 */

#include "System.h"			 /* System funct/params, like osc/peripheral config */
#include "LibTimer.h"

uint32_t temp;
/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/* i.e. uint16_t <variable_name>; */

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
	Setup_OSC();

	SetupTimer32Bit23(NORMAL_TIMER,PRESCALER_1,567891234UL);
	_T2IF = 0;
	SetIntPriorityTimer32Bit23(5);
	EnableIntTimer32Bit23();
	WriteCompareValue32Bit23(123456789UL);
	temp = ReadCompareValue32Bit23();
	WriteTimer32Bit23(987654321UL);
	temp = ReadTimer32Bit23();
//	CloseTimer32Bit23();

	
    while(1)
    {
	    Nop();
    }  
}
