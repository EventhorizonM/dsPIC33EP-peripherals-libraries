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


#include <stdint.h>		/* Includes uint16_t definition */
#include <libpic30.h>		/* for __delay32*/

#include "System.h"		/* System funct/params, like osc/peripheral config */
#include "LibUART.h"
#include "LibPorts.h"


/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
uint16_t i,UARTErr;
char Text[200];
char OutputText[]="Guess Who am I?\r";
/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
	Setup_OSC();

	UARTErr = SetupUART1(115200,NO_PAR_8_BIT,ONE_STOP_BIT);
	SetupUART1Advance(TX_RX_ONLY, RTS_FLOW_CONTROL_MODE, DIS_IrDA, DIS_WAKEUP_WHEN_SLEEP, CONTINUE_UART_IN_IDEL);
	SetupUART1Interrupts(INT_WHEN_4BYT_IN_RX_BUFFER, 3, INT_WHEN_TX_BUFFER_IS_EMPTY, 3, 1);
	SetupUART1Pins(RX_IDLE_STATE_IS_1, TX_IDLE_STATE_IS_1, DIS_LOOPBACK);

	InitPinF0(DIG_IN,NO_PULL,CN_DIS);    // U1RX_INPUT
	InitPinF1(DIG_OUT,NO_PULL,CN_DIS);   // U1TX_OUTPUT
	
	PPS_Unlock();
	PPS_Mapping(CONNECT_RP96_TO, U1RX_INPUT);
	PPS_Mapping(CONNECT_RP97_TO, U1TX_OUTPUT);
	PPS_Lock();

	U1RX_INT_FLAG = 0;
	U1TX_INT_FLAG = 0;

	DisableIntU1RX();
	DisableIntU1TX();

	EnableUART1();
	
	while(!UART1_DATA_AVAILABLE){}
	ReadTextUART1(Text,".",200,UNLIMITED_TIME);
	WriteTextUART1("I am Very Happy To See You :)\r");
	WriteTextUART1(OutputText);
	WriteTextUART1(Text);

	while(!UART1_TX_SHIFT_REG_IS_EMPTY){}
	WriteUART1(13);	// CR (Carriage Return) in ASCII. (New Line)
	WriteUART1(79);	// "O" in ASCII.
	WriteUART1(75);	// "K" in ASCII.
	WriteUART1(13); // CR (Carriage Return) in ASCII. (New Line)
	
    while(1){
	ReadTextUART1(Text,".",UNLIMITED_CHARS,UNLIMITED_TIME);
	WriteTextUART1("\r");
	WriteTextUART1(Text);
	for(i=0;i<sizeof(Text);i++){
		Text[i]=0;
	}
    }
}
