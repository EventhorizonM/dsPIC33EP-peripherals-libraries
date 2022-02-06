
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
#include "LibDMA.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

__eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));
__eds__ unsigned int BufferB[10] __attribute__((eds,space(dma)));

//unsigned int BufferB[4];

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
	Setup_OSC();

	SetupDMA0Channel(DATA_SIZE_16_BIT,10,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ); // Consider ADC1 Module is Intialized and Active.

	// SetUpDMA0_RAM_Buffers(CONTINUOUS_PINGPONG_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),__builtin_dmaoffset(BufferB));
	SetupDMA0_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDR_POST_INC,__builtin_dmaoffset(BufferA),0);
	SetupDMA0Interrupt(WHEN_ALL_DATA_MOVED,DMA0_INT_PRI_3);

	DMA0_INT_FLAG = 0;
	EnableIntDMA0();
	EnableChannelDMA0();

	TMR3 = 0x0000;
	T3CONbits.TCKPS = 3; // 256 Prescaler.
	PR3 = 20000;
	_T3IF = 0;
	_T3IE = 1;

	//Start Timer 3
	T3CONbits.TON = 1;

	while(1)
	{
		Nop();
		Nop();
		Nop();
	}
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void){

	_T3IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void){

	DMA0_INT_FLAG = 0;
}


void __attribute__((interrupt, no_auto_psv)) _DMACError(void){
        INTCON1bits.DMACERR = 0;        //Clear the trap flag
        while (1);
}
