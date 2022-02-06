
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


#include <stdint.h>        /* Includes uint16_t definition                    */

#include "System.h"        /* System funct/params, like osc/peripheral config */
#include "LibQEI.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

qeiCounter POS1;

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
	Setup_OSC();

	Setup32BitQEI1(	GATED_COUNTER_MODE,
			POS_COUNT_NO_INIT,
			QEI_INPUT_PRESCALE_1,
			COUNT_POSITIVE,
			DIS_INPUT_CAPTURE,
			KEEP_DEFAULT);

	Setup32BitQEI1IO(DIV_FLTR_PRESCALE_DIS,
			INDEX_MATCH_QEB_1_QEA_1,
			QEA_QEB_NOT_SWAPPED,
			QEA_POL_NON_INVERTED,
			QEB_POL_NON_INVERTED,
			INDX_POL_NON_INVERTED,
			HOM_POL_NON_INVERTED,
			DISABLE_CTNCMP_OUTPUT);
	
	Setup32BitQEI1Interrupt(4, EN_INT_POS_OVERFLOW,
				DIS_INT_INDEX,
				DIS_INT_HOME,
				DIS_INT_VELO_OVERFLOW,
				DIS_INT_POS_LESS_EQU,
				DIS_INT_POS_GREAT_EQU,
				DIS_INT_POS_INIT);

	POS1.l = Read32bitQEI1PositionCounter();
	Write32bitQEI2GreaterEqual(&POS1);

    while(1)
    {
	    Nop();
    }
    
}
void __attribute__((interrupt, no_auto_psv)) _QEI1Interrupt(void){

	_QEI1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _QEI2Interrupt(void){

	_QEI2IF = 0;
}