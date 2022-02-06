
/*
 * UART (Universal Asynchronous Receiver Transmitter) Libary For dsPIC33E/PIC24E Devices.
 * Copyright (c) 2015 AL-Moutaz Billah Tabbakha <eventhorizon.g@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: LibUART.h, V1.1 2015/04/4 AL-Moutaz Billah Tabbakha Exp $
 *
 ************************************************
 * - Macro Functions Represented in This Library: ("x" Denotes to UART Module Number 1 to 4.)
 *
 * uint16_t SetupUARTx( uint32_t BaudRate, ParityAndDataWidth, StopBits)
 * void	SetupUARTxAdvance(IO_Mode, RTS_Mode, IrDA_EN, WakeUpEN, IdleStop)
 * void SetupUARTxPins(RX_Polarity, TX_Polarity, LoopBackEN)
 * void SetupUARTxInterrupts(RxIntMode, TxIntMode, RxIntPriority, TxIntPriority, ErrIntPriority)
 * void WriteTextUARTx(char *String)
 * void ReadTextUARTx(char *OutputBuff, char *Delimiter, uint16_t CharectersLimit, uint32_t TimeOut)
 * void EnableUARTx()
 * void DisableUARTx()
 * uint8_t ReadUARTx()
 * void WriteUARTx(uint8_t DATA)
 * void EnableIntUxRX()
 * void DisableIntUxRX()
 * void EnableIntUxTX()
 * void DisableIntUxTX()
 * void EnableIntUxErr()
 * void DisableIntUxErr()
 * void EnableAddresDetectModeUARTx()
 * void DisableAddresDetectModeUARTx()
 * void SendSyncBreakUARTx()
 *
 *************************************************
 * - UARTx Status Bits.
 *
 * UARTx_TX_BUFF_IS_FULL
 * UARTx_TX_SHIFT_REG_IS_EMPTY
 * UARTx_DATA_AVAILABLE
 * UARTx_PARITY_ERROR_DETECTED
 * UARTx_FRAMING_ERROR_DETECTED
 * UARTx_RX_BUFF_OVER_FLOW_DETECTED
 *
 */

/***********************************************************************************************************************
 * Important Notes:
 * 1-	In multi-node direct-connect UARTx networks,UARTx receive inputs react to the complementary
 *	logic level defined by the URXINV bit(UxMODE<4>), which defines the Idle state, the
 *	default of which is logic high (i.e., URXINV = 0).Because remote devices do not initialize at the
 *	same time, it is likely that one of the devices,because the RX line is floating, will trigger a
 *	Start bit detection and will cause the first byte received, after the device has been initialized, to
 *	be invalid. To avoid this situation, the user should use a pull-up or pull-down resistor on the
 *	RX pin depending on the value of the URXINV bit.
 *	a) If URXINV = 0, use a pull-up resistor on the RX pin.
 *	b) If URXINV = 1, use a pull-down resistor on the RX pin.
 *
 * 2-	The first character received on a wake-up fromS leep mode, caused by activity on the UxRX pin
 *	of the UARTx module, will be invalid. In Sleep mode, peripheral clocks are disabled. By the
 *	time the oscillator system has restarted and stabilized from Sleep mode, the baud rate bit
 *	sampling clock, relative to the incoming UxRX bit timing, is no longer synchronized resulting in
 *	the first character being invalid. This is to be expected.
 **********************************************************************************************************************/

#ifndef _LIBUART_
#define _LIBUART_

#include <string.h>

/**********************************************************************************************************************/
/***************************************** UART1 Macro Functions & Defines ********************************************/
/**********************************************************************************************************************/
#if defined _U1RXIF

// <editor-fold defaultstate="collapsed" desc="UART1 Library Macro and Defines">

/* UART1 is Enabled; All UART1 pins are controlled by UART1 as defined by UEN<1:0>*/
#define EnableUART1()  ({U1MODEbits.UARTEN = 1; U1STAbits.UTXEN = 1;})
/* UART1 is Disabled; Any pending transmission is aborted, and the TX/RX buffers are reset, And all UART1 pins are controlled by port latches */
#define DisableUART1() ({int Clear; U1STAbits.UTXEN = 0; U1MODEbits.UARTEN = 0; Clear=U1RXREG; Clear=U1RXREG; Clear=U1RXREG; Clear=U1RXREG; Clear=U1RXREG;})
/* The function receives a byte via UART module.*/
#define ReadUART1()		U1RXREG
/* The function transmits a byte via the UART module.*/
#define WriteUART1(DATA)	U1TXREG = (DATA)
/***/
#define EnableIntU1RX()		_U1RXIE = 1
#define DisableIntU1RX()	_U1RXIE = 0
#define U1RX_INT_FLAG		_U1RXIF

#define EnableIntU1TX()		_U1TXIE = 1
#define DisableIntU1TX()	_U1TXIE = 0
#define U1TX_INT_FLAG		_U1TXIF

#define EnableIntU1Err()	_U1EIE = 1
#define DisableIntU1Err()	_U1EIE = 0
#define U1ERR_INT_FLAG		_U1EIF

/** Address Detect mode is enabled; if 9-bit mode is not selected, this does not take effect. */
#define EnableAddresDetectModeUART1()  U1STAbits.ADDEN = 1
/** Address Detect mode is disabled. */
#define DisableAddresDetectModeUART1() U1STAbits.ADDEN = 0

/**************************************************************************************
 * \Description
 * Sends Sync Break on next transmission ( a Start bit, followed by twelve '0' bits,
 * followed by Stop bit )   cleared by hardware upon completion.
 *************************************************************************************/
#define SendSyncBreakUART1()	U1STAbits.UTXBRK = 1

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// UART1 Status Bits.

/**************************************************************************************
 * <b>UART1_TX_BUFF_IS_FULL</b> Status:\n
 * 1 => Transmit buffer is full\n
 * 0 => Transmit buffer is not full, at least one more character can be written
 *************************************************************************************/
#define UART1_TX_BUFF_IS_FULL		U1STAbits.UTXBF

/**************************************************************************************
 * <b>UART1_TX_SHIFT_REG_IS_EMPTY</b> Status:\n
 * 1 => Transmit Shift Register and transmit buffer is empty (the last transmission has completed).\n
 * 0 => Transmit Shift Register is not empty, a transmission is in progress or queued.
 *************************************************************************************/
#define UART1_TX_SHIFT_REG_IS_EMPTY	U1STAbits.TRMT

/**************************************************************************************
 * <b>UART1_DATA_AVAILABLE</b> Status:\n
 * 1 => Receive buffer has data, at least one more character can be read.\n
 * 0 => Receive buffer is empty.
 *************************************************************************************/
#define UART1_DATA_AVAILABLE		U1STAbits.URXDA

/**************************************************************************************
 * <b>UART1_PARITY_ERROR_DETECTED</b> Status:\n
 * 1 => Parity error has been detected for the current character.\n
 * 0 => Parity error has not been detected.
 *************************************************************************************/
#define UART1_PARITY_ERROR_DETECTED	U1STAbits.PERR

/**************************************************************************************
 * <b>UART1_FRAMING_ERROR_DETECTED</b> Status:\n
 * 1 => Framing error has been detected for the current character.\n
 * 0 => Framing error has not been detected.
 *************************************************************************************/
#define UART1_FRAMING_ERROR_DETECTED	U1STAbits.FERR

/**************************************************************************************
 * <b>UART1_RX_BUFF_OVER_FLOW_DETECTED</b> Status:\n
 * 1 => Receive buffer has overflowed.\n
 * 0 => Receive buffer has not overflowed; clearing a previously set OERR bit (1 -> 0 transition)
 * resets the receiver buffer and the UxRSR to the empty state.
 *************************************************************************************/
#define UART1_RX_BUFF_OVER_FLOW_DETECTED	U1STAbits.OERR

// </editor-fold>

/***********************************************************************************************************************
 * \Function        unsigned int SetupUART1( uint32_t BaudRate, ParityAndDataWidth, StopBits)
 *
 * \Description     Configures UART1 Basic Operation Parameters.
 *
 * \Preconditions    You Must Define The FCY Before Using This Function.
 *
 * \Inputs
 * <u>BaudRate:</u>\n
 *	- AUTO_BAUDRATE.<i>/ If You Want to Set The Baud Rate Automaticaly By Receving a "U" Char From The Other Side.</i> \n
 *	- a Value Of Desired BaudRate 2400,9600,115200,... etc. <i>/ If You Want to Set The BaudRate Manually.</i>
 *
 * <u>ParityAndDataWidth:</u>	\n
 *	- NO_PAR_8_BIT		\n
 *	- EVEN_PAR_8_BIT	\n
 *	- ODD_PAR_8_BIT		\n
 *	- NO_PAR_9_BIT
 *
 * <u>StopBits:</u> \n
 *	- ONE_STOP_BIT	\n
 *	- TWO_STOP_BIT
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The UART Module Has Been Configered Without Any Errors.\n
 *		<b> 1: </b> Means That The Given Baud Rate Is Out Side The Allowable Limits For The Current Processor Freq.\n
 *		<b> 2: </b> Means That The Error Between Desired And Nearest Achievable Baud Rate Is More Than 1%.
 *
 * \Notes       Try To Set The Baud Rate Value Near a Divisors Of (FCY/16) OR (FCY/4) to Keep The Baud Rate Error As Small As Possible.
 *
 * \Example     UART1Error = Setup_UART1(115200,NO_PAR_8_BIT,ONE_STOP_BIT);
 *
 **********************************************************************************************************************/
#define SetupUART1( BaudRate, ParityAndDataWidth, StopBits)({										\
		unsigned long Temp_IGBT,BaudRateErr,_BaudRate = BaudRate;									\
		unsigned int *RemPointer,Reminder,_Return=2;										\
		RemPointer = &Reminder;													\
		U1MODEbits.PDSEL = ParityAndDataWidth;											\
		U1MODEbits.STSEL = StopBits;												\
																	\
		if(_BaudRate == AUTO_BAUDRATE){												\
			U1MODEbits.ABAUD = 1;												\
			_Return=0;													\
		}else{															\
			if(_BaudRate>MAX_BR_FOR_X4_MODE) {	/* Baud Rate is Out Of Range For The Current Frequency.*/		\
				_Return=1;												\
			}else{														\
				if(_BaudRate<MIN_BR_FOR_X16_MODE) {	/* Baud Rate is Out Of Range For The Current Frequency.*/	\
					_Return=1;											\
				}else{													\
					if(_BaudRate<=MAX_BR_FOR_X16_MODE){								\
						Temp_IGBT = (unsigned long long)FCY*1000/(_BaudRate * 16);	/* X16 BRG Mode.*/		\
						Temp_IGBT = __builtin_divmodud(Temp_IGBT,1000,RemPointer);					\
						if(Reminder>500) {Temp_IGBT+=1; Reminder=1000-Reminder;}					\
						BaudRateErr = ((unsigned long)Reminder)/Temp_IGBT;		/* Error Per Thousand.*/	\
						if(BaudRateErr<=10) {		/* Check if Error is More Than 1%.*/			\
							U1MODEbits.BRGH = 0;	/* X16 BRG Mode.*/					\
							U1BRG = Temp_IGBT-1;									\
							_Return=0;		/* x16 Mode and Every Thing is OK.*/			\
						}											\
					}												\
					/* If X16 Mode is Not Accepted Or The Error In X16 Mode Is More Than 1% So Try X4 Mode.*/	\
					if((_BaudRate>=MIN_BR_FOR_X4_MODE) && (_Return==2)){						\
						Temp_IGBT = (unsigned long long)FCY*1000/(_BaudRate * 4);	/* X4 BRG Mode.*/		\
						Temp_IGBT = __builtin_divmodud(Temp_IGBT,1000,RemPointer);					\
						if(Reminder>500) {Temp_IGBT+=1; Reminder=1000-Reminder;}					\
						BaudRateErr = ((unsigned long)Reminder)/Temp_IGBT;		/* Error Per Thousand.*/	\
						if(BaudRateErr<=10) {		/* Check if Error is More Than 1%.*/			\
							U1MODEbits.BRGH = 1;	/* X4 BRG Mode.*/					\
							U1BRG = Temp_IGBT-1;									\
							_Return=0;	/* x4 Mode and Every Thing is OK.*/				\
						} else {_Return=2;}	/* Error Between Desired and Best Achieved Baud Rate Value is*/ \
					}				/* More Than 1% Even For x4 Mode.*/				\
				}													\
			}														\
		}															\
	_Return;															\
	})

/***********************************************************************************************************************
 * \Function        void SetupUART1Advance(IO_Mode, RTS_Mode, IrDA_EN, WakeUpEN, IdleStop)
 *
 * \Description     Configures UART1 Advance Operation Parameters.
 *
 * \Preconditions    You Must To Setup The UART1 Basic Parameters First (By Using SetupUART1() Function For Example).
 *
 * \Inputs
 * <u>IO_Mode:</u>\n
 *	- TX_RX_ONLY		\n
 *	- TX_RX_RTS_CTS		\n
 *	- TX_RX_RTS		\n
 *	- TX_RX_BCLK
 *
 * <u>RTS_Mode:</u>\n
 *	- RTS_FLOW_CONTROL_MODE	\n
 *	- RTS_SIMPLEX_MODE
 *
 * <u>IrDA_EN:</u>\n
 *	- DIS_IrDA		\n
 *	- EN_IrDA
 *
 * <u>WakeUpEN:</u>\n
 *	- DIS_WAKEUP_WHEN_SLEEP	\n
 *	- EN_WAKEUP_WHEN_SLEEP
 *
 * <u>IdleStop:</u>\n
 *	- CONTINUE_UART_IN_IDEL	\n
 *	- STOP_UART_IN_IDEL
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupUART1Advance(TX_RX_ONLY, RTS_FLOW_CONTROL_MODE, DIS_IrDA, DIS_WAKEUP_WHEN_SLEEP, CONTINUE_UART_IN_IDEL);
 *
 **********************************************************************************************************************/
#define SetupUART1Advance(IO_Mode, RTS_Mode, IrDA_EN, WakeUpEN, IdleStop)({	\
		U1MODEbits.UEN = IO_Mode;					\
		U1MODEbits.RTSMD = RTS_Mode;					\
		U1MODEbits.IREN = IrDA_EN;					\
		U1MODEbits.WAKE = WakeUpEN;					\
		U1MODEbits.USIDL = IdleStop;					\
})

/***********************************************************************************************************************
 * \Function        void SetupUART1Pins(RX_Polarity, TX_Polarity, LoopBackEN)
 *
 * \Description     Configures UART1 Pins Polarity And Mode.
 *
 * \Preconditions   None.
 *
 * \Inputs
 * <u>RX_Polarity:</u>\n
 *	- RX_IDLE_STATE_IS_1	\n
 *	- RX_IDLE_STATE_IS_0
 *
 * <u>TX_Polarity:</u>\n
 *	- TX_IDLE_STATE_IS_1	\n
 *	- TX_IDLE_STATE_IS_0
 *
 * <u>LoopBackEN:</u>\n
 *	- DIS_LOOPBACK		\n
 *	- EN_LOOPBACK
 *
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupUART1Pins(RX_IDLE_STATE_IS_1, TX_IDLE_STATE_IS_1, DIS_LOOPBACK);
 *
 **********************************************************************************************************************/
#define SetupUART1Pins(RX_Polarity, TX_Polarity, LoopBackEN) ({	\
		U1MODEbits.URXINV = RX_Polarity;		\
		U1STAbits.UTXINV  = TX_Polarity;		\
		U1MODEbits.LPBACK = LoopBackEN;			\
})

/***********************************************************************************************************************
 * \Function        void SetupUART1Interrupts(RxIntMode, TxIntMode, RxIntPriority, TxIntPriority, ErrIntPriority)
 *
 * \Description     Configures UART1 Interrupts.
 *
 * \Preconditions    None.
 *
 * \Inputs
 * <u>RxIntMode:</u>\n
 *	- INT_WHEN_1BYT_IN_BUFFER		\n
 *	- INT_WHEN_3BYT_IN_BUFFER	\n
 *	- INT_WHEN_4BYT_IN_BUFFER
 *
 * <u>TxIntMode:</u>\n
 *	- INT_WHEN_ANY_TX_BUFFER_EMPTY	\n
 *	- INT_WHEN_TRANSMIT_IS_OVER	\n
 *	- INT_WHEN_TX_BUFFER_IS_EMPTY
 *
 * <u>RxIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * <u>TxIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * <u>ErrIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupUART1Interrupts(INT_WHEN_4BYT_IN_RX_BUFFER, INT_WHEN_TX_BUFFER_IS_EMPTY, 3, 3, 1);
 *
 **********************************************************************************************************************/
#define SetupUART1Interrupts(RxIntMode, TxIntMode, RxIntPriority, TxIntPriority, ErrIntPriority)({	\
		U1STAbits.URXISEL = RxIntMode;								\
		U1STAbits.UTXISEL0 = (TxIntMode&0b01);							\
		U1STAbits.UTXISEL1 = TxIntMode>>1;							\
		_U1RXIP = RxIntPriority;								\
		_U1TXIP = TxIntPriority;								\
		_U1EIP = ErrIntPriority;								\
})

/***********************************************************************************************************************
 * \Function        void WriteTextUART1(char *String)
 *
 * \Description     Sends text via UART.
 *
 * \Preconditions    None.
 *
 * \Inputs
 * <u>*String:</u> text to be sent \n
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     char Text[]="Guess Who am I?";			\n
 *
 *		WriteTextUART1("I am Very Happy To See You");	\n
 *		WriteTextUART1(Text);
 *
 **********************************************************************************************************************/
#define WriteTextUART1(String)({			\
	unsigned int len;				\
	char *StringPointer;				\
	StringPointer = String;				\
	len=strlen(String);				\
	for(i=0;i<len;i++){				\
		while(UART1_TX_BUFF_IS_FULL){}		\
		U1TXREG = *StringPointer;		\
		StringPointer = StringPointer+1;	\
	}						\
})

/***********************************************************************************************************************
 * \Function        void ReadTextUART1(char *OutputBuff, char *Delimiter, uint16_t CharectersLimit, uint32_t TimeOut)
 *
 * \Description     Reads characters received via UART until the Delimiter sequence is detected or a Time Out Occured.
 *		    The read sequence is stored in the parameter OutputBuff; delimiter sequence is stored in the parameter Delimiter.
 *		    The TimeOut is in milli seconds.
 *
 * \Preconditions    You Must Define The FCY Before Using This Function..
 *
 * \Inputs
 * <u>*OutputBuff:</u>		The Outbut Buffer Pointer.
 *
 * <u>*Delimiter:</u>		Sequence of characters that identifies the end of a received string.
 *
 * <u>CharectersLimit:</u>	Defines number of received characters in which Delimiter sequence is expected.
 *				If CharectersLimit is set to 60000 (UNLIMITED_CHARS) or More, this routine will continuously try
 *				to detect the Delimiter sequence.
 *
 * <u>TimeOut:</u>	Defines the Maximum idle receive time in (ms), if the UART Module stops receiving any bytes for more than TimeOut Value,
 *			The routine will end the receiving Process even if No Delimiter sequence is detected.
 *			If TimeOut is set to 300000 (UNLIMITED_TIME) or More, this routine will continuously try to detect the Delimiter sequence.
 *
 * \Return      None.
 *
 * \Notes       The received text will Contain the Delimiter sequence.
 *
 * \Example     char Text[200];					\n
 *
 *		while(!UART1_DATA_AVAILABLE){}			\n
 *		ReadTextUART1(Text,"OK",200,UNLIMITED_TIME);
 *
 **********************************************************************************************************************/
#define ReadTextUART1(_OutputBuff, _Delimiter, CharectersLimit, _TimeOut)({						\
	unsigned char DelimFlag,Var;											\
	unsigned int CharCounts,CountAddition,DelimiterLen;								\
	unsigned long Time;												\
	char *OutputBuff=_OutputBuff;											\
	char *Delimiter=_Delimiter;											\
	unsigned long TimeOut=_TimeOut;											\
															\
	DelimiterLen = strlen(Delimiter);										\
	CharCounts = DelimFlag = 0;											\
	Time = 0;													\
	if(TimeOut>=UNLIMITED_TIME)	Var=1;										\
	else				Var=0;										\
	TimeOut = TimeOut*(FCY/1000UL/38UL)+2;	/*This Time Constatn is Just For PIC24EP/dsPIC33EP Devices ONLY!*/	\
															\
	if(CharectersLimit>=UNLIMITED_CHARS)	CountAddition=0;							\
	else					CountAddition=1;							\
															\
	while(CharCounts<CharectersLimit && DelimFlag<DelimiterLen){							\
		if(UART1_DATA_AVAILABLE==0){										\
			Time=Time+Var;											\
			if(Time>(unsigned long)TimeOut){								\
				CharCounts = CharectersLimit;								\
			}												\
		}else{													\
			Time=0;												\
			CharCounts+=CountAddition;									\
			*OutputBuff = U1RXREG;										\
			if(*OutputBuff==*Delimiter){									\
				DelimFlag++;										\
				Delimiter = Delimiter+1;								\
			}else{												\
				Delimiter = Delimiter-DelimFlag;							\
				DelimFlag=0;										\
			}												\
			OutputBuff = OutputBuff+1;									\
		}													\
	}														\
})

#endif

/**********************************************************************************************************************/
/***************************************** UART2 Macro Functions & Defines ********************************************/
/**********************************************************************************************************************/
#if defined _U2RXIF

// <editor-fold defaultstate="collapsed" desc="UART2 Library Macro and Defines">

/** UART2 is Enabled; All UART2 pins are controlled by UART2 as defined by UEN<1:0>*/
#define EnableUART2()  ({U2MODEbits.UARTEN = 1; U2STAbits.UTXEN = 1;})
/** UART2 is Disabled; Any pending transmission is aborted, and the TX/RX buffers are reset, And all UART2 pins are controlled by port latches */
#define DisableUART2() ({int Clear; U2STAbits.UTXEN = 0; U2MODEbits.UARTEN = 0; Clear=U2RXREG; Clear=U2RXREG; Clear=U2RXREG; Clear=U2RXREG; Clear=U2RXREG;})
/** The function receives a byte via UART module.*/
#define ReadUART2()		U2RXREG
/** The function transmits a byte via the UART module.*/
#define WriteUART2(DATA)	U2TXREG = (DATA)
/***/
#define EnableIntU2RX()		_U2RXIE = 1
#define DisableIntU2RX()	_U2RXIE = 0
#define U2RX_INT_FLAG		_U2RXIF

#define EnableIntU2TX()		_U2TXIE = 1
#define DisableIntU2TX()	_U2TXIE = 0
#define U2TX_INT_FLAG		_U2TXIF

#define EnableIntU2Err()	_U2EIE = 1
#define DisableIntU2Err()	_U2EIE = 0
#define U2ERR_INT_FLAG		_U2EIF

/** Address Detect mode is enabled; if 9-bit mode is not selected, this does not take effect. */
#define EnableAddresDetectModeUART2()  U2STAbits.ADDEN = 1
/** Address Detect mode is disabled. */
#define DisableAddresDetectModeUART2() U2STAbits.ADDEN = 0

/**************************************************************************************
 * \Description
 * Sends Sync Break on next transmission ( a Start bit, followed by twelve '0' bits,
 * followed by Stop bit )   cleared by hardware upon completion.
 *************************************************************************************/
#define SendSyncBreakUART2()	U2STAbits.UTXBRK = 1

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// UART2 Status Bits.

/**************************************************************************************
 * <b>UART2_TX_BUFF_IS_FULL</b> Status:\n
 * 1 => Transmit buffer is full\n
 * 0 => Transmit buffer is not full, at least one more character can be written
 *************************************************************************************/
#define UART2_TX_BUFF_IS_FULL		U2STAbits.UTXBF

/**************************************************************************************
 * <b>UART2_TX_SHIFT_REG_IS_EMPTY</b> Status:\n
 * 1 => Transmit Shift Register and transmit buffer is empty (the last transmission has completed).\n
 * 0 => Transmit Shift Register is not empty, a transmission is in progress or queued.
 *************************************************************************************/
#define UART2_TX_SHIFT_REG_IS_EMPTY	U2STAbits.TRMT

/**************************************************************************************
 * <b>UART2_DATA_AVAILABLE</b> Status:\n
 * 1 => Receive buffer has data, at least one more character can be read.\n
 * 0 => Receive buffer is empty.
 *************************************************************************************/
#define UART2_DATA_AVAILABLE		U2STAbits.URXDA

/**************************************************************************************
 * <b>UART2_PARITY_ERROR_DETECTED</b> Status:\n
 * 1 => Parity error has been detected for the current character.\n
 * 0 => Parity error has not been detected.
 *************************************************************************************/
#define UART2_PARITY_ERROR_DETECTED	U2STAbits.PERR

/**************************************************************************************
 * <b>UART2_FRAMING_ERROR_DETECTED</b> Status:\n
 * 1 => Framing error has been detected for the current character.\n
 * 0 => Framing error has not been detected.
 *************************************************************************************/
#define UART2_FRAMING_ERROR_DETECTED	U2STAbits.FERR

/**************************************************************************************
 * <b>UART2_RX_BUFF_OVER_FLOW_DETECTED</b> Status:\n
 * 1 => Receive buffer has overflowed.\n
 * 0 => Receive buffer has not overflowed; clearing a previously set OERR bit (1 -> 0 transition)
 * resets the receiver buffer and the UxRSR to the empty state.
 *************************************************************************************/
#define UART2_RX_BUFF_OVER_FLOW_DETECTED	U2STAbits.OERR

// </editor-fold>

/***********************************************************************************************************************
 * \Function        unsigned int SetupUART2( uint32_t BaudRate, ParityAndDataWidth, StopBits)
 *
 * \Description     Configures UART2 Basic Operation Parameters.
 *
 * \Preconditions    You Must Define The FCY Before Using This Function.
 *
 * \Inputs
 * <u>BaudRate:</u>\n
 *	- AUTO_BAUDRATE.<i>/ If You Want to Set The Baud Rate Automaticaly By Receving a "U" Char From The Other Side.</i> \n
 *	- a Value Of Desired BaudRate 2400,9600,115200,... etc. <i>/ If You Want to Set The BaudRate Manually.</i>
 *
 * <u>ParityAndDataWidth:</u>	\n
 *	- NO_PAR_8_BIT		\n
 *	- EVEN_PAR_8_BIT	\n
 *	- ODD_PAR_8_BIT		\n
 *	- NO_PAR_9_BIT
 *
 * <u>StopBits:</u> \n
 *	- ONE_STOP_BIT	\n
 *	- TWO_STOP_BIT
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The UART Module Has Been Configered Without Any Errors.\n
 *		<b> 1: </b> Means That The Given Baud Rate Is Out Side The Allowable Limits For The Current Processor Freq.\n
 *		<b> 2: </b> Means That The Error Between Desired And Nearest Achievable Baud Rate Is More Than 1%.
 *
 * \Notes       Try To Set The Baud Rate Value Near a Divisors Of (FCY/16) OR (FCY/4) to Keep The Baud Rate Error As Small As Possible.
 *
 * \Example     UART2Error = Setup_UART2(115200,NO_PAR_8_BIT,ONE_STOP_BIT);
 *
 **********************************************************************************************************************/
#define SetupUART2( BaudRate, ParityAndDataWidth, StopBits)({										\
		unsigned long Temp_IGBT,BaudRateErr,_BaudRate = BaudRate;									\
		unsigned int *RemPointer,Reminder,_Return=2;										\
		RemPointer = &Reminder;													\
		U2MODEbits.PDSEL = ParityAndDataWidth;											\
		U2MODEbits.STSEL = StopBits;												\
																	\
		if(_BaudRate == AUTO_BAUDRATE){												\
			U2MODEbits.ABAUD = 1;												\
			_Return=0;													\
		}else{															\
			if(_BaudRate>MAX_BR_FOR_X4_MODE) {	/* Baud Rate is Out Of Range For The Current Frequency.*/		\
				_Return=1;												\
			}else{														\
				if(_BaudRate<MIN_BR_FOR_X16_MODE) {	/* Baud Rate is Out Of Range For The Current Frequency.*/	\
					_Return=1;											\
				}else{													\
					if(_BaudRate<=MAX_BR_FOR_X16_MODE){								\
						Temp_IGBT = (unsigned long long)FCY*1000/(_BaudRate * 16);	/* X16 BRG Mode.*/		\
						Temp_IGBT = __builtin_divmodud(Temp_IGBT,1000,RemPointer);					\
						if(Reminder>500) {Temp_IGBT+=1; Reminder=1000-Reminder;}					\
						BaudRateErr = ((unsigned long)Reminder)/Temp_IGBT;		/* Error Per Thousand.*/	\
						if(BaudRateErr<=10) {		/* Check if Error is More Than 1%.*/			\
							U2MODEbits.BRGH = 0;	/* X16 BRG Mode.*/					\
							U2BRG = Temp_IGBT-1;									\
							_Return=0;		/* x16 Mode and Every Thing is OK.*/			\
						}											\
					}												\
					/* If X16 Mode is Not Accepted Or The Error In X16 Mode Is More Than 1% So Try X4 Mode.*/	\
					if((_BaudRate>=MIN_BR_FOR_X4_MODE) && (_Return==2)){						\
						Temp_IGBT = (unsigned long long)FCY*1000/(_BaudRate * 4);	/* X4 BRG Mode.*/		\
						Temp_IGBT = __builtin_divmodud(Temp_IGBT,1000,RemPointer);					\
						if(Reminder>500) {Temp_IGBT+=1; Reminder=1000-Reminder;}					\
						BaudRateErr = ((unsigned long)Reminder)/Temp_IGBT;		/* Error Per Thousand.*/	\
						if(BaudRateErr<=10) {		/* Check if Error is More Than 1%.*/			\
							U2MODEbits.BRGH = 1;	/* X4 BRG Mode.*/					\
							U2BRG = Temp_IGBT-1;									\
							_Return=0;	/* x4 Mode and Every Thing is OK.*/				\
						} else {_Return=2;}	/* Error Between Desired and Best Achieved Baud Rate Value is*/ \
					}				/* More Than 1% Even For x4 Mode.*/				\
				}													\
			}														\
		}															\
	_Return;															\
	})

/***********************************************************************************************************************
 * \Function        void SetupUART2Advance(IO_Mode, RTS_Mode, IrDA_EN, WakeUpEN, IdleStop)
 *
 * \Description     Configures UART2 Advance Operation Parameters.
 *
 * \Preconditions    You Must To Setup The UART2 Basic Parameters First (By Using SetupUART2() Function For Example).
 *
 * \Inputs
 * <u>IO_Mode:</u>\n
 *	- TX_RX_ONLY		\n
 *	- TX_RX_RTS_CTS		\n
 *	- TX_RX_RTS		\n
 *	- TX_RX_BCLK
 *
 * <u>RTS_Mode:</u>\n
 *	- RTS_FLOW_CONTROL_MODE	\n
 *	- RTS_SIMPLEX_MODE
 *
 * <u>IrDA_EN:</u>\n
 *	- DIS_IrDA		\n
 *	- EN_IrDA
 *
 * <u>WakeUpEN:</u>\n
 *	- DIS_WAKEUP_WHEN_SLEEP	\n
 *	- EN_WAKEUP_WHEN_SLEEP
 *
 * <u>IdleStop:</u>\n
 *	- CONTINUE_UART_IN_IDEL	\n
 *	- STOP_UART_IN_IDEL
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupUART2Advance(TX_RX_ONLY, RTS_FLOW_CONTROL_MODE, DIS_IrDA, DIS_WAKEUP_WHEN_SLEEP, CONTINUE_UART_IN_IDEL);
 *
 **********************************************************************************************************************/
#define SetupUART2Advance(IO_Mode, RTS_Mode, IrDA_EN, WakeUpEN, IdleStop)({	\
		U2MODEbits.UEN = IO_Mode;					\
		U2MODEbits.RTSMD = RTS_Mode;					\
		U2MODEbits.IREN = IrDA_EN;					\
		U2MODEbits.WAKE = WakeUpEN;					\
		U2MODEbits.USIDL = IdleStop;					\
})

/***********************************************************************************************************************
 * \Function        void SetupUART2Pins(RX_Polarity, TX_Polarity, LoopBackEN)
 *
 * \Description     Configures UART2 Pins Polarity And Mode.
 *
 * \Preconditions   None.
 *
 * \Inputs
 * <u>RX_Polarity:</u>\n
 *	- RX_IDLE_STATE_IS_1	\n
 *	- RX_IDLE_STATE_IS_0
 *
 * <u>TX_Polarity:</u>\n
 *	- TX_IDLE_STATE_IS_1	\n
 *	- TX_IDLE_STATE_IS_0
 *
 * <u>LoopBackEN:</u>\n
 *	- DIS_LOOPBACK		\n
 *	- EN_LOOPBACK
 *
 * <u>TxIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * <u>ErrIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupUART2Pins(RX_IDLE_STATE_IS_1, TX_IDLE_STATE_IS_1, DIS_LOOPBACK);
 *
 **********************************************************************************************************************/
#define SetupUART2Pins(RX_Polarity, TX_Polarity, LoopBackEN) ({	\
		U2MODEbits.URXINV = RX_Polarity;		\
		U2STAbits.UTXINV  = TX_Polarity;		\
		U2MODEbits.LPBACK = LoopBackEN;			\
})

/***********************************************************************************************************************
 * \Function        void SetupUART2Interrupts(RxIntMode, TxIntMode, RxIntPriority, TxIntPriority, ErrIntPriority)
 *
 * \Description     Configures UART2 Interrupts.
 *
 * \Preconditions    None.
 *
 * \Inputs
 * <u>RxIntMode:</u>\n
 *	- INT_WHEN_1BYT_IN_BUFFER		\n
 *	- INT_WHEN_3BYT_IN_BUFFER	\n
 *	- INT_WHEN_4BYT_IN_BUFFER
 *
 * <u>TxIntMode:</u>\n
 *	- INT_WHEN_ANY_TX_BUFFER_EMPTY	\n
 *	- INT_WHEN_TRANSMIT_IS_OVER	\n
 *	- INT_WHEN_TX_BUFFER_IS_EMPTY
 *
 * <u>RxIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * <u>TxIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * <u>ErrIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupUART2Interrupts(INT_WHEN_4BYT_IN_RX_BUFFER, INT_WHEN_TX_BUFFER_IS_EMPTY, 3, 3, 1);
 *
 **********************************************************************************************************************/
#define SetupUART2Interrupts(RxIntMode, TxIntMode, RxIntPriority, TxIntPriority, ErrIntPriority)({	\
		U2STAbits.URXISEL = RxIntMode;								\
		U2STAbits.UTXISEL0 = (TxIntMode&0b01);							\
		U2STAbits.UTXISEL1 = TxIntMode>>1;							\
		_U2RXIP = RxIntPriority;								\
		_U2TXIP = TxIntPriority;								\
		_U2EIP = ErrIntPriority;								\
})

/***********************************************************************************************************************
 * \Function        void WriteTextUART2(char *String)
 *
 * \Description     Sends text via UART.
 *
 * \Preconditions    None.
 *
 * \Inputs
 * <u>*String:</u> text to be sent \n
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     char Text[]="Guess Who am I?";			\n
 *
 *		WriteTextUART2("I am Very Happy To See You");	\n
 *		WriteTextUART2(Text);
 *
 **********************************************************************************************************************/
#define WriteTextUART2(String)({			\
	unsigned int len;				\
	char *StringPointer;				\
	StringPointer = String;				\
	len=strlen(String);				\
	for(i=0;i<len;i++){				\
		while(UART2_TX_BUFF_IS_FULL){}		\
		U2TXREG = *StringPointer;		\
		StringPointer = StringPointer+1;	\
	}						\
})

/***********************************************************************************************************************
 * \Function        void ReadTextUART2(char *OutputBuff, char *Delimiter, uint16_t CharectersLimit, uint32_t TimeOut)
 *
 * \Description     Reads characters received via UART until the Delimiter sequence is detected or a Time Out Occured.
 *		    The read sequence is stored in the parameter OutputBuff; delimiter sequence is stored in the parameter Delimiter.
 *		    The TimeOut is in milli seconds.
 *
 * \Preconditions    You Must Define The FCY Before Using This Function..
 *
 * \Inputs
 * <u>*OutputBuff:</u>		The Outbut Buffer Pointer.
 *
 * <u>*Delimiter:</u>		Sequence of characters that identifies the end of a received string.
 *
 * <u>CharectersLimit:</u>	Defines number of received characters in which Delimiter sequence is expected.
 *				If CharectersLimit is set to 60000 (UNLIMITED_CHARS) or More, this routine will continuously try
 *				to detect the Delimiter sequence.
 *
 * <u>TimeOut:</u>	Defines the Maximum idle receive time in (ms), if the UART Module stops receiving any bytes for more than TimeOut Value,
 *			The routine will end the receiving Process even if No Delimiter sequence is detected.
 *			If TimeOut is set to 300000 (UNLIMITED_TIME) or More, this routine will continuously try to detect the Delimiter sequence.
 *
 * \Return      None.
 *
 * \Notes       The received text will Contain the Delimiter sequence.
 *
 * \Example     char Text[200];					\n
 *
 *		while(!UART2_DATA_AVAILABLE){}			\n
 *		ReadTextUART2(Text,"OK",200,UNLIMITED_TIME);
 *
 **********************************************************************************************************************/
#define ReadTextUART2(_OutputBuff, _Delimiter, CharectersLimit, _TimeOut)({						\
	unsigned char DelimFlag,Var;											\
	unsigned int CharCounts,CountAddition,DelimiterLen;								\
	unsigned long Time;												\
	char *OutputBuff=_OutputBuff;											\
	char *Delimiter=_Delimiter;											\
	unsigned long TimeOut=_TimeOut;											\
															\
	DelimiterLen = strlen(Delimiter);										\
	CharCounts = DelimFlag = 0;											\
	Time = 0;													\
	if(TimeOut>=UNLIMITED_TIME)	Var=1;										\
	else				Var=0;										\
	TimeOut = TimeOut*(FCY/1000UL/38UL)+2;	/*This Time Constatn is Just For PIC24EP/dsPIC33EP Devices ONLY!*/	\
															\
	if(CharectersLimit>=UNLIMITED_CHARS)	CountAddition=0;							\
	else					CountAddition=1;							\
															\
	while(CharCounts<CharectersLimit && DelimFlag<DelimiterLen){							\
		if(UART2_DATA_AVAILABLE==0){										\
			Time=Time+Var;											\
			if(Time>(unsigned long)TimeOut){								\
				CharCounts = CharectersLimit;								\
			}												\
		}else{													\
			Time=0;												\
			CharCounts+=CountAddition;									\
			*OutputBuff = U2RXREG;										\
			if(*OutputBuff==*Delimiter){									\
				DelimFlag++;										\
				Delimiter = Delimiter+1;								\
			}else{												\
				Delimiter = Delimiter-DelimFlag;							\
				DelimFlag=0;										\
			}												\
			OutputBuff = OutputBuff+1;									\
		}													\
	}														\
})															

#endif

/**********************************************************************************************************************/
/***************************************** UART3 Macro Functions & Defines ********************************************/
/**********************************************************************************************************************/
#if defined _U3RXIF

// <editor-fold defaultstate="collapsed" desc="UART3 Library Macro and Defines">

/** UART3 is Enabled; All UART3 pins are controlled by UART3 as defined by UEN<1:0>*/
#define EnableUART3()  ({U3MODEbits.UARTEN = 1; U3STAbits.UTXEN = 1;})
/** UART3 is Disabled; Any pending transmission is aborted, and the TX/RX buffers are reset, And all UART3 pins are controlled by port latches */
#define DisableUART3() ({int Clear; U3STAbits.UTXEN = 0; U3MODEbits.UARTEN = 0; Clear=U3RXREG; Clear=U3RXREG; Clear=U3RXREG; Clear=U3RXREG; Clear=U3RXREG;})
/** The function receives a byte via UART module.*/
#define ReadUART3()		U3RXREG
/** The function transmits a byte via the UART module.*/
#define WriteUART3(DATA)	U3TXREG = (DATA)
/***/
#define EnableIntU3RX()		_U3RXIE = 1
#define DisableIntU3RX()	_U3RXIE = 0
#define U3RX_INT_FLAG		_U3RXIF

#define EnableIntU3TX()		_U3TXIE = 1
#define DisableIntU3TX()	_U3TXIE = 0
#define U3TX_INT_FLAG		_U3TXIF

#define EnableIntU3Err()	_U3EIE = 1
#define DisableIntU3Err()	_U3EIE = 0
#define U3ERR_INT_FLAG		_U3EIF

/** Address Detect mode is enabled; if 9-bit mode is not selected, this does not take effect. */
#define EnableAddresDetectModeUART3()  U3STAbits.ADDEN = 1
/** Address Detect mode is disabled. */
#define DisableAddresDetectModeUART3() U3STAbits.ADDEN = 0

/**************************************************************************************
 * \Description
 * Sends Sync Break on next transmission ( a Start bit, followed by twelve '0' bits,
 * followed by Stop bit )   cleared by hardware upon completion.
 *************************************************************************************/
#define SendSyncBreakUART3()	U3STAbits.UTXBRK = 1

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// UART3 Status Bits.

/**************************************************************************************
 * <b>UART3_TX_BUFF_IS_FULL</b> Status:\n
 * 1 => Transmit buffer is full\n
 * 0 => Transmit buffer is not full, at least one more character can be written
 *************************************************************************************/
#define UART3_TX_BUFF_IS_FULL		U3STAbits.UTXBF

/**************************************************************************************
 * <b>UART3_TX_SHIFT_REG_IS_EMPTY</b> Status:\n
 * 1 => Transmit Shift Register and transmit buffer is empty (the last transmission has completed).\n
 * 0 => Transmit Shift Register is not empty, a transmission is in progress or queued.
 *************************************************************************************/
#define UART3_TX_SHIFT_REG_IS_EMPTY	U3STAbits.TRMT

/**************************************************************************************
 * <b>UART3_DATA_AVAILABLE</b> Status:\n
 * 1 => Receive buffer has data, at least one more character can be read.\n
 * 0 => Receive buffer is empty.
 *************************************************************************************/
#define UART3_DATA_AVAILABLE		U3STAbits.URXDA

/**************************************************************************************
 * <b>UART3_PARITY_ERROR_DETECTED</b> Status:\n
 * 1 => Parity error has been detected for the current character.\n
 * 0 => Parity error has not been detected.
 *************************************************************************************/
#define UART3_PARITY_ERROR_DETECTED	U3STAbits.PERR

/**************************************************************************************
 * <b>UART3_FRAMING_ERROR_DETECTED</b> Status:\n
 * 1 => Framing error has been detected for the current character.\n
 * 0 => Framing error has not been detected.
 *************************************************************************************/
#define UART3_FRAMING_ERROR_DETECTED	U3STAbits.FERR

/**************************************************************************************
 * <b>UART3_RX_BUFF_OVER_FLOW_DETECTED</b> Status:\n
 * 1 => Receive buffer has overflowed.\n
 * 0 => Receive buffer has not overflowed; clearing a previously set OERR bit (1 -> 0 transition)
 * resets the receiver buffer and the UxRSR to the empty state.
 *************************************************************************************/
#define UART3_RX_BUFF_OVER_FLOW_DETECTED	U3STAbits.OERR

// </editor-fold>

/***********************************************************************************************************************
 * \Function        unsigned int SetupUART3( uint32_t BaudRate, ParityAndDataWidth, StopBits)
 *
 * \Description     Configures UART3 Basic Operation Parameters.
 *
 * \Preconditions    You Must Define The FCY Before Using This Function.
 *
 * \Inputs
 * <u>BaudRate:</u>\n
 *	- AUTO_BAUDRATE.<i>/ If You Want to Set The Baud Rate Automaticaly By Receving a "U" Char From The Other Side.</i> \n
 *	- a Value Of Desired BaudRate 2400,9600,115200,... etc. <i>/ If You Want to Set The BaudRate Manually.</i>
 *
 * <u>ParityAndDataWidth:</u>	\n
 *	- NO_PAR_8_BIT		\n
 *	- EVEN_PAR_8_BIT	\n
 *	- ODD_PAR_8_BIT		\n
 *	- NO_PAR_9_BIT
 *
 * <u>StopBits:</u> \n
 *	- ONE_STOP_BIT	\n
 *	- TWO_STOP_BIT
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The UART Module Has Been Configered Without Any Errors.\n
 *		<b> 1: </b> Means That The Given Baud Rate Is Out Side The Allowable Limits For The Current Processor Freq.\n
 *		<b> 2: </b> Means That The Error Between Desired And Nearest Achievable Baud Rate Is More Than 1%.
 *
 * \Notes       Try To Set The Baud Rate Value Near a Divisors Of (FCY/16) OR (FCY/4) to Keep The Baud Rate Error As Small As Possible.
 *
 * \Example     UART3Error = Setup_UART3(115200,NO_PAR_8_BIT,ONE_STOP_BIT);
 *
 **********************************************************************************************************************/
#define SetupUART3( BaudRate, ParityAndDataWidth, StopBits)({										\
		unsigned long Temp_IGBT,BaudRateErr,_BaudRate = BaudRate;									\
		unsigned int *RemPointer,Reminder,_Return=2;										\
		RemPointer = &Reminder;													\
		U3MODEbits.PDSEL = ParityAndDataWidth;											\
		U3MODEbits.STSEL = StopBits;												\
																	\
		if(_BaudRate == AUTO_BAUDRATE){												\
			U3MODEbits.ABAUD = 1;												\
			_Return=0;													\
		}else{															\
			if(_BaudRate>MAX_BR_FOR_X4_MODE) {	/* Baud Rate is Out Of Range For The Current Frequency.*/		\
				_Return=1;												\
			}else{														\
				if(_BaudRate<MIN_BR_FOR_X16_MODE) {	/* Baud Rate is Out Of Range For The Current Frequency.*/	\
					_Return=1;											\
				}else{													\
					if(_BaudRate<=MAX_BR_FOR_X16_MODE){								\
						Temp_IGBT = (unsigned long long)FCY*1000/(_BaudRate * 16);	/* X16 BRG Mode.*/		\
						Temp_IGBT = __builtin_divmodud(Temp_IGBT,1000,RemPointer);					\
						if(Reminder>500) {Temp_IGBT+=1; Reminder=1000-Reminder;}					\
						BaudRateErr = ((unsigned long)Reminder)/Temp_IGBT;		/* Error Per Thousand.*/	\
						if(BaudRateErr<=10) {		/* Check if Error is More Than 1%.*/			\
							U3MODEbits.BRGH = 0;	/* X16 BRG Mode.*/					\
							U3BRG = Temp_IGBT-1;									\
							_Return=0;		/* x16 Mode and Every Thing is OK.*/			\
						}											\
					}												\
					/* If X16 Mode is Not Accepted Or The Error In X16 Mode Is More Than 1% So Try X4 Mode.*/	\
					if((_BaudRate>=MIN_BR_FOR_X4_MODE) && (_Return==2)){						\
						Temp_IGBT = (unsigned long long)FCY*1000/(_BaudRate * 4);	/* X4 BRG Mode.*/		\
						Temp_IGBT = __builtin_divmodud(Temp_IGBT,1000,RemPointer);					\
						if(Reminder>500) {Temp_IGBT+=1; Reminder=1000-Reminder;}					\
						BaudRateErr = ((unsigned long)Reminder)/Temp_IGBT;		/* Error Per Thousand.*/	\
						if(BaudRateErr<=10) {		/* Check if Error is More Than 1%.*/			\
							U3MODEbits.BRGH = 1;	/* X4 BRG Mode.*/					\
							U3BRG = Temp_IGBT-1;									\
							_Return=0;	/* x4 Mode and Every Thing is OK.*/				\
						} else {_Return=2;}	/* Error Between Desired and Best Achieved Baud Rate Value is*/ \
					}				/* More Than 1% Even For x4 Mode.*/				\
				}													\
			}														\
		}															\
	_Return;															\
	})

/***********************************************************************************************************************
 * \Function        void SetupUART3Advance(IO_Mode, RTS_Mode, IrDA_EN, WakeUpEN, IdleStop)
 *
 * \Description     Configures UART3 Advance Operation Parameters.
 *
 * \Preconditions    You Must To Setup The UART3 Basic Parameters First (By Using SetupUART3() Function For Example).
 *
 * \Inputs
 * <u>IO_Mode:</u>\n
 *	- TX_RX_ONLY		\n
 *	- TX_RX_RTS_CTS		\n
 *	- TX_RX_RTS		\n
 *	- TX_RX_BCLK
 *
 * <u>RTS_Mode:</u>\n
 *	- RTS_FLOW_CONTROL_MODE	\n
 *	- RTS_SIMPLEX_MODE
 *
 * <u>IrDA_EN:</u>\n
 *	- DIS_IrDA		\n
 *	- EN_IrDA
 *
 * <u>WakeUpEN:</u>\n
 *	- DIS_WAKEUP_WHEN_SLEEP	\n
 *	- EN_WAKEUP_WHEN_SLEEP
 *
 * <u>IdleStop:</u>\n
 *	- CONTINUE_UART_IN_IDEL	\n
 *	- STOP_UART_IN_IDEL
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupUART3Advance(TX_RX_ONLY, RTS_FLOW_CONTROL_MODE, DIS_IrDA, DIS_WAKEUP_WHEN_SLEEP, CONTINUE_UART_IN_IDEL);
 *
 **********************************************************************************************************************/
#define SetupUART3Advance(IO_Mode, RTS_Mode, IrDA_EN, WakeUpEN, IdleStop)({	\
		U3MODEbits.UEN = IO_Mode;					\
		U3MODEbits.RTSMD = RTS_Mode;					\
		U3MODEbits.IREN = IrDA_EN;					\
		U3MODEbits.WAKE = WakeUpEN;					\
		U3MODEbits.USIDL = IdleStop;					\
})

/***********************************************************************************************************************
 * \Function        void SetupUART3Pins(RX_Polarity, TX_Polarity, LoopBackEN)
 *
 * \Description     Configures UART3 Pins Polarity And Mode.
 *
 * \Preconditions   None.
 *
 * \Inputs
 * <u>RX_Polarity:</u>\n
 *	- RX_IDLE_STATE_IS_1	\n
 *	- RX_IDLE_STATE_IS_0
 *
 * <u>TX_Polarity:</u>\n
 *	- TX_IDLE_STATE_IS_1	\n
 *	- TX_IDLE_STATE_IS_0
 *
 * <u>LoopBackEN:</u>\n
 *	- DIS_LOOPBACK		\n
 *	- EN_LOOPBACK
 *
 * <u>TxIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * <u>ErrIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupUART3Pins(RX_IDLE_STATE_IS_1, TX_IDLE_STATE_IS_1, DIS_LOOPBACK);
 *
 **********************************************************************************************************************/
#define SetupUART3Pins(RX_Polarity, TX_Polarity, LoopBackEN) ({	\
		U3MODEbits.URXINV = RX_Polarity;		\
		U3STAbits.UTXINV  = TX_Polarity;		\
		U3MODEbits.LPBACK = LoopBackEN;			\
})

/***********************************************************************************************************************
 * \Function        void SetupUART3Interrupts(RxIntMode, TxIntMode, RxIntPriority, TxIntPriority, ErrIntPriority)
 *
 * \Description     Configures UART3 Interrupts.
 *
 * \Preconditions    None.
 *
 * \Inputs
 * <u>RxIntMode:</u>\n
 *	- INT_WHEN_1BYT_IN_BUFFER		\n
 *	- INT_WHEN_3BYT_IN_BUFFER	\n
 *	- INT_WHEN_4BYT_IN_BUFFER
 *
 * <u>TxIntMode:</u>\n
 *	- INT_WHEN_ANY_TX_BUFFER_EMPTY	\n
 *	- INT_WHEN_TRANSMIT_IS_OVER	\n
 *	- INT_WHEN_TX_BUFFER_IS_EMPTY
 *
 * <u>RxIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * <u>TxIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * <u>ErrIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupUART3Interrupts(INT_WHEN_4BYT_IN_RX_BUFFER, INT_WHEN_TX_BUFFER_IS_EMPTY, 3, 3, 1);
 *
 **********************************************************************************************************************/
#define SetupUART3Interrupts(RxIntMode, TxIntMode, RxIntPriority, TxIntPriority, ErrIntPriority)({	\
		U3STAbits.URXISEL = RxIntMode;								\
		U3STAbits.UTXISEL0 = (TxIntMode&0b01);							\
		U3STAbits.UTXISEL1 = TxIntMode>>1;							\
		_U3RXIP = RxIntPriority;								\
		_U3TXIP = TxIntPriority;								\
		_U3EIP = ErrIntPriority;								\
})

/***********************************************************************************************************************
 * \Function        void WriteTextUART3(char *String)
 *
 * \Description     Sends text via UART.
 *
 * \Preconditions    None.
 *
 * \Inputs
 * <u>*String:</u> text to be sent \n
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     char Text[]="Guess Who am I?";			\n
 *
 *		WriteTextUART3("I am Very Happy To See You");	\n
 *		WriteTextUART3(Text);
 *
 **********************************************************************************************************************/
#define WriteTextUART3(String)({			\
	unsigned int len;				\
	char *StringPointer;				\
	StringPointer = String;				\
	len=strlen(String);				\
	for(i=0;i<len;i++){				\
		while(UART3_TX_BUFF_IS_FULL){}		\
		U3TXREG = *StringPointer;		\
		StringPointer = StringPointer+1;	\
	}						\
})

/***********************************************************************************************************************
 * \Function        void ReadTextUART3(char *OutputBuff, char *Delimiter, uint16_t CharectersLimit, uint32_t TimeOut)
 *
 * \Description     Reads characters received via UART until the Delimiter sequence is detected or a Time Out Occured.
 *		    The read sequence is stored in the parameter OutputBuff; delimiter sequence is stored in the parameter Delimiter.
 *		    The TimeOut is in milli seconds.
 *
 * \Preconditions    You Must Define The FCY Before Using This Function..
 *
 * \Inputs
 * <u>*OutputBuff:</u>		The Outbut Buffer Pointer.
 *
 * <u>*Delimiter:</u>		Sequence of characters that identifies the end of a received string.
 *
 * <u>CharectersLimit:</u>	Defines number of received characters in which Delimiter sequence is expected.
 *				If CharectersLimit is set to 60000 (UNLIMITED_CHARS) or More, this routine will continuously try
 *				to detect the Delimiter sequence.
 *
 * <u>TimeOut:</u>	Defines the Maximum idle receive time in (ms), if the UART Module stops receiving any bytes for more than TimeOut Value,
 *			The routine will end the receiving Process even if No Delimiter sequence is detected.
 *			If TimeOut is set to 300000 (UNLIMITED_TIME) or More, this routine will continuously try to detect the Delimiter sequence.
 *
 * \Return      None.
 *
 * \Notes       The received text will Contain the Delimiter sequence.
 *
 * \Example     char Text[200];					\n
 *						
 *		while(!UART3_DATA_AVAILABLE){}			\n
 *		ReadTextUART3(Text,"OK",200,UNLIMITED_TIME);
 *
 **********************************************************************************************************************/
#define ReadTextUART3(_OutputBuff, _Delimiter, CharectersLimit, _TimeOut)({						\
	unsigned char DelimFlag,Var;											\
	unsigned int CharCounts,CountAddition,DelimiterLen;								\
	unsigned long Time;												\
	char *OutputBuff=_OutputBuff;											\
	char *Delimiter=_Delimiter;											\
	unsigned long TimeOut=_TimeOut;											\
															\
	DelimiterLen = strlen(Delimiter);										\
	CharCounts = DelimFlag = 0;											\
	Time = 0;													\
	if(TimeOut>=UNLIMITED_TIME)	Var=1;										\
	else				Var=0;										\
	TimeOut = TimeOut*(FCY/1000UL/38UL)+2;	/*This Time Constatn is Just For PIC24EP/dsPIC33EP Devices ONLY!*/	\
															\
	if(CharectersLimit>=UNLIMITED_CHARS)	CountAddition=0;							\
	else					CountAddition=1;							\
															\
	while(CharCounts<CharectersLimit && DelimFlag<DelimiterLen){							\
		if(UART3_DATA_AVAILABLE==0){										\
			Time=Time+Var;											\
			if(Time>(unsigned long)TimeOut){								\
				CharCounts = CharectersLimit;								\
			}												\
		}else{													\
			Time=0;												\
			CharCounts+=CountAddition;									\
			*OutputBuff = U3RXREG;										\
			if(*OutputBuff==*Delimiter){									\
				DelimFlag++;										\
				Delimiter = Delimiter+1;								\
			}else{												\
				Delimiter = Delimiter-DelimFlag;							\
				DelimFlag=0;										\
			}												\
			OutputBuff = OutputBuff+1;									\
		}													\
	}														\
})															

#endif

/**********************************************************************************************************************/
/***************************************** UART4 Macro Functions & Defines ********************************************/
/**********************************************************************************************************************/
#if defined _U4RXIF

// <editor-fold defaultstate="collapsed" desc="UART4 Library Macro and Defines">

/** UART4 is Enabled; All UART4 pins are controlled by UART4 as defined by UEN<1:0>*/
#define EnableUART4()  ({U4MODEbits.UARTEN = 1; U4STAbits.UTXEN = 1;})
/** UART4 is Disabled; Any pending transmission is aborted, and the TX/RX buffers are reset, And all UART4 pins are controlled by port latches */
#define DisableUART4() ({int Clear; U4STAbits.UTXEN = 0; U4MODEbits.UARTEN = 0; Clear=U4RXREG; Clear=U4RXREG; Clear=U4RXREG; Clear=U4RXREG; Clear=U4RXREG;})
/** The function receives a byte via UART module.*/
#define ReadUART4()		U4RXREG
/** The function transmits a byte via the UART module.*/
#define WriteUART4(DATA)	U4TXREG = (DATA)
/***/
#define EnableIntU4RX()		_U4RXIE = 1
#define DisableIntU4RX()	_U4RXIE = 0
#define U4RX_INT_FLAG		_U4RXIF

#define EnableIntU4TX()		_U4TXIE = 1
#define DisableIntU4TX()	_U4TXIE = 0
#define U4TX_INT_FLAG		_U4TXIF

#define EnableIntU4Err()	_U4EIE = 1
#define DisableIntU4Err()	_U4EIE = 0
#define U4ERR_INT_FLAG		_U4EIF

/** Address Detect mode is enabled; if 9-bit mode is not selected, this does not take effect. */
#define EnableAddresDetectModeUART4()  U4STAbits.ADDEN = 1
/** Address Detect mode is disabled. */
#define DisableAddresDetectModeUART4() U4STAbits.ADDEN = 0

/**************************************************************************************
 * \Description
 * Sends Sync Break on next transmission ( a Start bit, followed by twelve '0' bits,
 * followed by Stop bit )   cleared by hardware upon completion.
 *************************************************************************************/
#define SendSyncBreakUART4()	U4STAbits.UTXBRK = 1

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// UART4 Status Bits.

/**************************************************************************************
 * <b>UART4_TX_BUFF_IS_FULL</b> Status:\n
 * 1 => Transmit buffer is full\n
 * 0 => Transmit buffer is not full, at least one more character can be written
 *************************************************************************************/
#define UART4_TX_BUFF_IS_FULL		U4STAbits.UTXBF

/**************************************************************************************
 * <b>UART4_TX_SHIFT_REG_IS_EMPTY</b> Status:\n
 * 1 => Transmit Shift Register and transmit buffer is empty (the last transmission has completed).\n
 * 0 => Transmit Shift Register is not empty, a transmission is in progress or queued.
 *************************************************************************************/
#define UART4_TX_SHIFT_REG_IS_EMPTY	U4STAbits.TRMT

/**************************************************************************************
 * <b>UART4_DATA_AVAILABLE</b> Status:\n
 * 1 => Receive buffer has data, at least one more character can be read.\n
 * 0 => Receive buffer is empty.
 *************************************************************************************/
#define UART4_DATA_AVAILABLE		U4STAbits.URXDA

/**************************************************************************************
 * <b>UART4_PARITY_ERROR_DETECTED</b> Status:\n
 * 1 => Parity error has been detected for the current character.\n
 * 0 => Parity error has not been detected.
 *************************************************************************************/
#define UART4_PARITY_ERROR_DETECTED	U4STAbits.PERR

/**************************************************************************************
 * <b>UART4_FRAMING_ERROR_DETECTED</b> Status:\n
 * 1 => Framing error has been detected for the current character.\n
 * 0 => Framing error has not been detected.
 *************************************************************************************/
#define UART4_FRAMING_ERROR_DETECTED	U4STAbits.FERR

/**************************************************************************************
 * <b>UART4_RX_BUFF_OVER_FLOW_DETECTED</b> Status:\n
 * 1 => Receive buffer has overflowed.\n
 * 0 => Receive buffer has not overflowed; clearing a previously set OERR bit (1 -> 0 transition)
 * resets the receiver buffer and the UxRSR to the empty state.
 *************************************************************************************/
#define UART4_RX_BUFF_OVER_FLOW_DETECTED	U4STAbits.OERR

// </editor-fold>

/***********************************************************************************************************************
 * \Function        unsigned int SetupUART4( uint32_t BaudRate, ParityAndDataWidth, StopBits)
 *
 * \Description     Configures UART4 Basic Operation Parameters.
 *
 * \Preconditions    You Must Define The FCY Before Using This Function.
 *
 * \Inputs
 * <u>BaudRate:</u>\n
 *	- AUTO_BAUDRATE.<i>/ If You Want to Set The Baud Rate Automaticaly By Receving a "U" Char From The Other Side.</i> \n
 *	- a Value Of Desired BaudRate 2400,9600,115200,... etc. <i>/ If You Want to Set The BaudRate Manually.</i>
 *
 * <u>ParityAndDataWidth:</u>	\n
 *	- NO_PAR_8_BIT		\n
 *	- EVEN_PAR_8_BIT	\n
 *	- ODD_PAR_8_BIT		\n
 *	- NO_PAR_9_BIT
 *
 * <u>StopBits:</u> \n
 *	- ONE_STOP_BIT	\n
 *	- TWO_STOP_BIT
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The UART Module Has Been Configered Without Any Errors.\n
 *		<b> 1: </b> Means That The Given Baud Rate Is Out Side The Allowable Limits For The Current Processor Freq.\n
 *		<b> 2: </b> Means That The Error Between Desired And Nearest Achievable Baud Rate Is More Than 1%.
 *
 * \Notes       Try To Set The Baud Rate Value Near a Divisors Of (FCY/16) OR (FCY/4) to Keep The Baud Rate Error As Small As Possible.
 *
 * \Example     UART4Error = Setup_UART4(115200,NO_PAR_8_BIT,ONE_STOP_BIT);
 *
 **********************************************************************************************************************/
#define SetupUART4( BaudRate, ParityAndDataWidth, StopBits)({										\
		unsigned long Temp_IGBT,BaudRateErr,_BaudRate = BaudRate;									\
		unsigned int *RemPointer,Reminder,_Return=2;										\
		RemPointer = &Reminder;													\
		U4MODEbits.PDSEL = ParityAndDataWidth;											\
		U4MODEbits.STSEL = StopBits;												\
																	\
		if(_BaudRate == AUTO_BAUDRATE){												\
			U4MODEbits.ABAUD = 1;												\
			_Return=0;													\
		}else{															\
			if(_BaudRate>MAX_BR_FOR_X4_MODE) {	/* Baud Rate is Out Of Range For The Current Frequency.*/		\
				_Return=1;												\
			}else{														\
				if(_BaudRate<MIN_BR_FOR_X16_MODE) {	/* Baud Rate is Out Of Range For The Current Frequency.*/	\
					_Return=1;											\
				}else{													\
					if(_BaudRate<=MAX_BR_FOR_X16_MODE){								\
						Temp_IGBT = (unsigned long long)FCY*1000/(_BaudRate * 16);	/* X16 BRG Mode.*/		\
						Temp_IGBT = __builtin_divmodud(Temp_IGBT,1000,RemPointer);					\
						if(Reminder>500) {Temp_IGBT+=1; Reminder=1000-Reminder;}					\
						BaudRateErr = ((unsigned long)Reminder)/Temp_IGBT;		/* Error Per Thousand.*/	\
						if(BaudRateErr<=10) {		/* Check if Error is More Than 1%.*/			\
							U4MODEbits.BRGH = 0;	/* X16 BRG Mode.*/					\
							U4BRG = Temp_IGBT-1;									\
							_Return=0;		/* x16 Mode and Every Thing is OK.*/			\
						}											\
					}												\
					/* If X16 Mode is Not Accepted Or The Error In X16 Mode Is More Than 1% So Try X4 Mode.*/	\
					if((_BaudRate>=MIN_BR_FOR_X4_MODE) && (_Return==2)){						\
						Temp_IGBT = (unsigned long long)FCY*1000/(_BaudRate * 4);	/* X4 BRG Mode.*/		\
						Temp_IGBT = __builtin_divmodud(Temp_IGBT,1000,RemPointer);					\
						if(Reminder>500) {Temp_IGBT+=1; Reminder=1000-Reminder;}					\
						BaudRateErr = ((unsigned long)Reminder)/Temp_IGBT;		/* Error Per Thousand.*/	\
						if(BaudRateErr<=10) {		/* Check if Error is More Than 1%.*/			\
							U4MODEbits.BRGH = 1;	/* X4 BRG Mode.*/					\
							U4BRG = Temp_IGBT-1;									\
							_Return=0;	/* x4 Mode and Every Thing is OK.*/				\
						} else {_Return=2;}	/* Error Between Desired and Best Achieved Baud Rate Value is*/ \
					}				/* More Than 1% Even For x4 Mode.*/				\
				}													\
			}														\
		}															\
	_Return;															\
	})

/***********************************************************************************************************************
 * \Function        void SetupUART4Advance(IO_Mode, RTS_Mode, IrDA_EN, WakeUpEN, IdleStop)
 *
 * \Description     Configures UART4 Advance Operation Parameters.
 *
 * \Preconditions    You Must To Setup The UART4 Basic Parameters First (By Using SetupUART4() Function For Example).
 *
 * \Inputs
 * <u>IO_Mode:</u>\n
 *	- TX_RX_ONLY		\n
 *	- TX_RX_RTS_CTS		\n
 *	- TX_RX_RTS		\n
 *	- TX_RX_BCLK
 *
 * <u>RTS_Mode:</u>\n
 *	- RTS_FLOW_CONTROL_MODE	\n
 *	- RTS_SIMPLEX_MODE
 *
 * <u>IrDA_EN:</u>\n
 *	- DIS_IrDA		\n
 *	- EN_IrDA
 *
 * <u>WakeUpEN:</u>\n
 *	- DIS_WAKEUP_WHEN_SLEEP	\n
 *	- EN_WAKEUP_WHEN_SLEEP
 *
 * <u>IdleStop:</u>\n
 *	- CONTINUE_UART_IN_IDEL	\n
 *	- STOP_UART_IN_IDEL
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupUART4Advance(TX_RX_ONLY, RTS_FLOW_CONTROL_MODE, DIS_IrDA, DIS_WAKEUP_WHEN_SLEEP, CONTINUE_UART_IN_IDEL);
 *
 **********************************************************************************************************************/
#define SetupUART4Advance(IO_Mode, RTS_Mode, IrDA_EN, WakeUpEN, IdleStop)({	\
		U4MODEbits.UEN = IO_Mode;					\
		U4MODEbits.RTSMD = RTS_Mode;					\
		U4MODEbits.IREN = IrDA_EN;					\
		U4MODEbits.WAKE = WakeUpEN;					\
		U4MODEbits.USIDL = IdleStop;					\
})

/***********************************************************************************************************************
 * \Function        void SetupUART4Pins(RX_Polarity, TX_Polarity, LoopBackEN)
 *
 * \Description     Configures UART4 Pins Polarity And Mode.
 *
 * \Preconditions   None.
 *
 * \Inputs
 * <u>RX_Polarity:</u>\n
 *	- RX_IDLE_STATE_IS_1	\n
 *	- RX_IDLE_STATE_IS_0
 *
 * <u>TX_Polarity:</u>\n
 *	- TX_IDLE_STATE_IS_1	\n
 *	- TX_IDLE_STATE_IS_0
 *
 * <u>LoopBackEN:</u>\n
 *	- DIS_LOOPBACK		\n
 *	- EN_LOOPBACK
 *
 * <u>TxIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * <u>ErrIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupUART4Pins(RX_IDLE_STATE_IS_1, TX_IDLE_STATE_IS_1, DIS_LOOPBACK);
 *
 **********************************************************************************************************************/
#define SetupUART4Pins(RX_Polarity, TX_Polarity, LoopBackEN) ({	\
		U4MODEbits.URXINV = RX_Polarity;		\
		U4STAbits.UTXINV  = TX_Polarity;		\
		U4MODEbits.LPBACK = LoopBackEN;			\
})

/***********************************************************************************************************************
 * \Function        void SetupUART4Interrupts(RxIntMode, TxIntMode, RxIntPriority, TxIntPriority, ErrIntPriority)
 *
 * \Description     Configures UART4 Interrupts.
 *
 * \Preconditions    None.
 *
 * \Inputs
 * <u>RxIntMode:</u>\n
 *	- INT_WHEN_1BYT_IN_BUFFER	\n
 *	- INT_WHEN_3BYT_IN_BUFFER	\n
 *	- INT_WHEN_4BYT_IN_BUFFER
 *
 * <u>TxIntMode:</u>\n
 *	- INT_WHEN_ANY_TX_BUFFER_EMPTY	\n
 *	- INT_WHEN_TRANSMIT_IS_OVER	\n
 *	- INT_WHEN_TX_BUFFER_IS_EMPTY
 *
 * <u>RxIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * <u>TxIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * <u>ErrIntPriority:</u>\n
 *	- a Value Between 0 And 7 Inclusively.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupUART4Interrupts(INT_WHEN_4BYT_IN_RX_BUFFER, INT_WHEN_TX_BUFFER_IS_EMPTY, 3, 3, 1);
 *
 **********************************************************************************************************************/
#define SetupUART4Interrupts(RxIntMode, TxIntMode, RxIntPriority, TxIntPriority, ErrIntPriority)({	\
		U4STAbits.URXISEL = RxIntMode;								\
		U4STAbits.UTXISEL0 = (TxIntMode&0b01);							\
		U4STAbits.UTXISEL1 = TxIntMode>>1;							\
		_U4RXIP = RxIntPriority;								\
		_U4TXIP = TxIntPriority;								\
		_U4EIP = ErrIntPriority;								\
})

/***********************************************************************************************************************
 * \Function        void WriteTextUART4(char *String)
 *
 * \Description     Sends text via UART.
 *
 * \Preconditions    None.
 *
 * \Inputs
 * <u>*String:</u> text to be sent \n
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     char Text[]="Guess Who am I?";			\n
 *
 *		WriteTextUART4("I am Very Happy To See You");	\n
 *		WriteTextUART4(Text);
 *
 **********************************************************************************************************************/
#define WriteTextUART4(String)({			\
	unsigned int len;				\
	char *StringPointer;				\
	StringPointer = String;				\
	len=strlen(String);				\
	for(i=0;i<len;i++){				\
		while(UART4_TX_BUFF_IS_FULL){}		\
		U4TXREG = *StringPointer;		\
		StringPointer = StringPointer+1;	\
	}						\
})

/***********************************************************************************************************************
 * \Function        void ReadTextUART4(char *OutputBuff, char *Delimiter, uint16_t CharectersLimit, uint32_t TimeOut)
 *
 * \Description     Reads characters received via UART until the Delimiter sequence is detected or a Time Out Occured.
 *		    The read sequence is stored in the parameter OutputBuff; delimiter sequence is stored in the parameter Delimiter.
 *		    The TimeOut is in milli seconds.
 *
 * \Preconditions    You Must Define The FCY Before Using This Function..
 *
 * \Inputs
 * <u>*OutputBuff:</u>		The Outbut Buffer Pointer.
 *
 * <u>*Delimiter:</u>		Sequence of characters that identifies the end of a received string.
 *
 * <u>CharectersLimit:</u>	Defines number of received characters in which Delimiter sequence is expected.
 *				If CharectersLimit is set to 60000 (UNLIMITED_CHARS) or More, this routine will continuously try
 *				to detect the Delimiter sequence.
 *
 * <u>TimeOut:</u>	Defines the Maximum idle receive time in (ms), if the UART Module stops receiving any bytes for more than TimeOut Value,
 *			The routine will end the receiving Process even if No Delimiter sequence is detected.
 *			If TimeOut is set to 300000 (UNLIMITED_TIME) or More, this routine will continuously try to detect the Delimiter sequence.
 *
 * \Return      None.
 *
 * \Notes       The received text will Contain the Delimiter sequence.
 *
 * \Example     char Text[200];					\n
 *						
 *		while(!UART4_DATA_AVAILABLE){}			\n
 *		ReadTextUART4(Text,"OK",200,UNLIMITED_TIME);
 *
 **********************************************************************************************************************/
#define ReadTextUART4(_OutputBuff, _Delimiter, CharectersLimit, _TimeOut)({						\
	unsigned char DelimFlag,Var;											\
	unsigned int CharCounts,CountAddition,DelimiterLen;								\
	unsigned long Time;												\
	char *OutputBuff=_OutputBuff;											\
	char *Delimiter=_Delimiter;											\
	unsigned long TimeOut=_TimeOut;											\
															\
	DelimiterLen = strlen(Delimiter);										\
	CharCounts = DelimFlag = 0;											\
	Time = 0;													\
	if(TimeOut>=UNLIMITED_TIME)	Var=1;										\
	else				Var=0;										\
	TimeOut = TimeOut*(FCY/1000UL/38UL)+2;	/*This Time Constatn is Just For PIC24EP/dsPIC33EP Devices ONLY!*/	\
															\
	if(CharectersLimit>=UNLIMITED_CHARS)	CountAddition=0;							\
	else					CountAddition=1;							\
															\
	while(CharCounts<CharectersLimit && DelimFlag<DelimiterLen){							\
		if(UART4_DATA_AVAILABLE==0){										\
			Time=Time+Var;											\
			if(Time>(unsigned long)TimeOut){								\
				CharCounts = CharectersLimit;								\
			}												\
		}else{													\
			Time=0;												\
			CharCounts+=CountAddition;									\
			*OutputBuff = U4RXREG;										\
			if(*OutputBuff==*Delimiter){									\
				DelimFlag++;										\
				Delimiter = Delimiter+1;								\
			}else{												\
				Delimiter = Delimiter-DelimFlag;							\
				DelimFlag=0;										\
			}												\
			OutputBuff = OutputBuff+1;									\
		}													\
	}														\
})															

#endif

/**********************************************************************************************************************/
/********************************************** UART Library Arguments ************************************************/
/**********************************************************************************************************************/

// <editor-fold defaultstate="collapsed" desc="UART Library Arguments">

#ifndef _COMMON_DIRECTIVES_
#define _COMMON_DIRECTIVES_

#define DONT_CHANGE	0xFFFF
#define DONT_CARE	0
#define KEEP_DEFAULT	0
#define SET_DEFAULT	0

#endif

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* SetupUARTx */

/** BaudRate */
#define AUTO_BAUDRATE	1
/* Or Set The Baud Rate Manually Such as 2400,9600,19200,115200... etc. */

/** ParityAndDataWidth */
#define NO_PAR_8_BIT	0
#define EVEN_PAR_8_BIT	1
#define ODD_PAR_8_BIT	2
#define NO_PAR_9_BIT	3

/** StopBits */
#define ONE_STOP_BIT	0
#define TWO_STOP_BIT	1
/***/

#define MAX_BR_FOR_X16_MODE	(FCY/16)
#define MIN_BR_FOR_X16_MODE	(FCY/(16*65536))
#define MAX_BR_FOR_X4_MODE	(FCY/4)
#define MIN_BR_FOR_X4_MODE	(FCY/(4*65536))

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* SetupUARTxAdvance */

/** IO_Mode */
#define TX_RX_ONLY		0	// UxTX and UxRX pins are enabled and used; UxCTS, UxRTS and BCLKx pins are controlled by port latches
#define TX_RX_RTS		1	// UxTX, UxRX and UxRTS pins are enabled and used; UxCTS pin is controlled by port latches
#define TX_RX_RTS_CTS		2	// UxTX, UxRX, UxCTS and UxRTS pins are enabled and used
#define TX_RX_BCLK		3	// UxTX, UxRX and BCLKx pins are enabled and used; UxCTS pin is controlled by port latches

/** RTS_Mode */
#define RTS_FLOW_CONTROL_MODE	0
#define RTS_SIMPLEX_MODE	1

/** IrDA_EN */
#define DIS_IrDA		0	// IrDA encoder and decoder are Disabled
#define EN_IrDA			1	// IrDA encoder and decoder are Enabled

/** WakeUpEN */
#define DIS_WAKEUP_WHEN_SLEEP	0	// Disable Wake-up on Start bit Detect During Sleep Mode bit
#define EN_WAKEUP_WHEN_SLEEP	1	// Enable Wake-up on Start bit Detect During Sleep Mode bit

/** IdleStop*/
#define CONTINUE_UART_IN_IDEL	0	// Continues operation in Idle mode
#define STOP_UART_IN_IDEL	1	// Discontinues operation when the device enters Idle mode

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* SetupUARTxInterrupts */

/** RxIntMode */
#define INT_WHEN_1BYT_IN_RX_BUFFER	0	// Interrupt flag bit is set when a character is received
#define INT_WHEN_3BYT_IN_RX_BUFFER	1	// Interrupt flag bit is set when the receive buffer is 3/4 full (i.e., 3 data characters)
#define INT_WHEN_4BYT_IN_RX_BUFFER	2	// Interrupt flag bit is set when the receive buffer is full (i.e., 4 data characters)

/** TxIntMode */
#define INT_WHEN_ANY_TX_BUFFER_EMPTY	0	/* Interrupt is generated when any character is transferred to the Transmit Shift Register and the
						transmit buffer is empty (which implies at least one location is empty in the transmit buffer)*/
#define	INT_WHEN_TRANSMIT_IS_OVER	1	/* Interrupt is generated when the last transmission is over, transmit buffer is empty (i.e., the last
						character has been shifted out of the Transmit Shift Register) and all the transmit operations are completed*/
#define INT_WHEN_TX_BUFFER_IS_EMPTY	2	/* Interrupt is generated when a character is transferred to the Transmit Shift Register (TSR)
						and the transmit buffer becomes empty*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* SetupUARTxPins */

/** RX_Polarity */
#define RX_IDLE_STATE_IS_1	0
#define RX_IDLE_STATE_IS_0	1

/** TX_Polarity */
#define TX_IDLE_STATE_IS_1	0
#define TX_IDLE_STATE_IS_0	1

/** LoopBackEN*/
#define DIS_LOOPBACK		0
#define EN_LOOPBACK		1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* ReadTextUARTx */

/** TimeOut (Maximum)*/
#define UNLIMITED_TIME		3000000

/** CharectersLimit */
#define UNLIMITED_CHARS		60000


// </editor-fold>

#endif