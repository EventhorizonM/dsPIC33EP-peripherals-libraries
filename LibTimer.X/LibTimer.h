
/*
 * 16/32 bit Timers Libary For dsPIC33E/PIC24E Devices.
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
 * File: $Id: LibTimer.h, V1.00 2015/04/4 AL-Moutaz Billah Tabbakha Exp $
 *
 ************************************************************************************************
 * - Macro Functions Represented in This Library: ("x" Denotes to Timer Module Number 1 to 9.)
 *
 * void SetupTimerx(uint16_t TimerMode, uint16_t Prescaler, uint16_t CompareValue)
 * void CloseTimerx()
 * void EnableIntTimerx()
 * void DisableIntTimerx()
 * void SetIntPriorityTimerx(priority)
 * void ReadTimerx()
 * void WriteTimerx(value)
 * void ReadCompareValueTimerx()
 * void WriteCompareValueTimerx(value)
 *
 ************************************************************************************************
 * - Macro Functions Represented in This Library: ("x" Denotes to Timers Pair Number: 23 or 45 or 67 or 89.)
 *
 * void SetupTimer32Bitx(uint16_t TimerMode,uint16_t Prescaler,uint16_t CompareValue)
 * void CloseTimer32Bitx()
 * void EnableIntTimer32Bitx()
 * void DisableIntTimer32Bitx()
 * void SetIntPriorityTimer32Bitx(priority)
 * void ReadTimer32Bitx()
 * void WriteTimer32Bitx(Value)
 * void ReadCompareValue32Bitx()
 * void WritePeriod32Bitx(Value)
 */

#ifndef _LIBTIMER_
#define	_LIBTIMER_

/***********************************************************************************************************************
 * Important Timers Notes:
 *  
 * 1-	A timer operating from a synchronized external clock source does not operate in Sleep mode,
 *		because the synchronization circuit is shut off during Sleep mode.
 * 2-	For Type C timers, it is necessary for the external clock input period to be high for at least 0.5 TCY
 *		(and an additional input buffer delay of 20 ns), and low for at least 0.5 TCY (and an additional input
 *		buffer delay of 20 ns) for proper synchronization.
 * 3-	For a Type A and Type B timer, the external clock input period must be at least 0.5 TCY
 *		(and an additional input buffer delay of 20 ns) divided by the prescaler value.
 * 4-	However, the high and low time of the external clock input must not violate the minimum
 *		pulse-width requirement of 10 ns nominal (or 50 MHz nominal frequency).
 * 5-	Timers, when configured for the External Counter mode (TCS = 1), operate as follows:
 *		Type A and Type B timers start counting from the second rising edge, while
 *		Type C timers start counting from the first rising edge.
 * 6-	The TxIF bit is set one instruction cycle after a period match.
 * 7-	A special case occurs when the period register, PRx, is loaded with 0x0000 and the
 *		timer is enabled. No timer interrupts are generated for this configuration.
 * 8-	Type B and Type C timers do not support the Asynchronous External Clock mode
 *		therefore, 32-bit Asynchronous Counter mode is not supported.
 *
 **********************************************************************************************************************/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*SetupTimerx*/

/** TimerMode */
#define NORMAL_TIMER	1
#define GATED_TIMER	2
#define SYNC_COUNTER	3
#define ASYNC_COUNTER	4

/** Prescaler */
#define PRESCALER_1	0
#define PRESCALER_8	1
#define PRESCALER_64	2
#define PRESCALER_256	3

/**********************************************************************************************************************/
/********************************************* Timer1 Macro Functions *************************************************/
/**********************************************************************************************************************/
#if defined _T1IE

/***********************************************************************************************************************
 * \Function		void SetupTimer1(uint16_t TimerMode, uint16_t Prescaler, uint16_t CompareValue)
 *
 * \Description		Configures Timer1.
 *
 * \Preconditions	 None.
 *
 * \Inputs
 * <u>TimerMode:</u>\n
 *	- NORMAL_TIMER.	<i>/ The input clock is derived from the internal instruction cycle clock (FCY).</i> \n
 *	- GATED_TIMER.	<i>/ The timer increments on every rising edge of the input clock as long as the external gate signal at TxCK pin is high.</i>\n
 *	- SYNC_COUNTER.	<i>/ The input clock to the timer is derived from the external clock input at TxCK pin divided by a programmable prescaler.
 *			     In This Mode The external clock input is synchronized with the internal device clock.</i>\n
 *	- ASYNC_COUNTER.<i>/ As (SYNC_COUNTER) Mode But The external clock input is NOT synchronized with the internal device clock.</i>
 *
 * <u>Prescaler:</u>	\n
 *	- PRESCALER_1.	\n
 *	- PRESCALER_8.	\n
 *	- PRESCALER_64.	\n
 *	- PRESCALER_256.
 *
 * <u>CompareValue:</u> <i>/ When The Timer Register Match This Compare Value The Timer Will Reset And Will Generate An Interrupt (if Enabled).</i>\n
 *	- A value between 0 - 65535 inclusive.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupTimer1(NORMAL_TIMER,PRESCALER_64,4000);
 *
 **********************************************************************************************************************/
#define SetupTimer1(TimerMode,Prescaler,CompareValue)({					\
	switch (TimerMode) {								\
		case NORMAL_TIMER  : T1CONbits.TCS = 0; T1CONbits.TGATE = 0; break;	\
		case GATED_TIMER   : T1CONbits.TCS = 0; T1CONbits.TGATE = 1; break;	\
		case SYNC_COUNTER  : T1CONbits.TCS = 1; T1CONbits.TSYNC = 1; break;	\
		case ASYNC_COUNTER : T1CONbits.TCS = 1; T1CONbits.TSYNC = 0;		\
	}										\
											\
	T1CONbits.TCKPS	= Prescaler;							\
	PR1 = CompareValue;								\
	TMR1 = 0x00;									\
	T1CONbits.TON = 1;})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/** Turn Off Timer1 And Reset Its Value. */
#define CloseTimer1()	({_T1IE = 0; T1CON = TMR1 = 0;})
/***/
#define EnableIntTimer1()		_T1IE = 1

#define DisableIntTimer1()		_T1IE = 0

#define SetIntPriorityTimer1(priority)	_T1IP = (priority)

#define ReadTimer1()			TMR1

#define WriteTimer1(value)		TMR1 = (value)

#define ReadCompareValueTimer1()	PR1

#define WriteCompareValueTimer1(value)	PR1 = (value)

#endif

/**********************************************************************************************************************/
/********************************************* Timer2 Macro Functions *************************************************/
/**********************************************************************************************************************/
#if defined _T2IE

/***********************************************************************************************************************
 * \Function		void SetupTimer2(uint16_t TimerMode, uint16_t Prescaler, uint16_t CompareValue)
 *
 * \Description		Configures Timer2.
 *
 * \Preconditions	 None.
 *
 * \Inputs
 * <u>TimerMode:</u>\n
 *	- NORMAL_TIMER.	<i>/ The input clock is derived from the internal instruction cycle clock (FCY).</i> \n
 *	- GATED_TIMER.	<i>/ The timer increments on every rising edge of the input clock as long as the external gate signal at TxCK pin is high.</i>\n
 *	- SYNC_COUNTER.	<i>/ The input clock to the timer is derived from the external clock input at TxCK pin divided by a programmable prescaler.
 *			     In This Mode The external clock input is synchronized with the internal device clock.</i>\n
 *
 * <u>Prescaler:</u>	\n
 *	- PRESCALER_1.	\n
 *	- PRESCALER_8.	\n
 *	- PRESCALER_64.	\n
 *	- PRESCALER_256.
 *
 * <u>CompareValue:</u> <i>/ When The Timer Register Match This Compare Value The Timer Will Reset And Will Generate An Interrupt (if Enabled).</i>\n
 *	- A value between 0 - 65535 inclusive.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupTimer2(NORMAL_TIMER,PRESCALER_64,4000);
 *
 **********************************************************************************************************************/
#define SetupTimer2(TimerMode,Prescaler,CompareValue)({					\
	switch (TimerMode) {								\
		case NORMAL_TIMER  : T2CONbits.TCS = 0; T2CONbits.TGATE = 0; break;	\
		case GATED_TIMER   : T2CONbits.TCS = 0; T2CONbits.TGATE = 1; break;	\
		case SYNC_COUNTER  : T2CONbits.TCS = 1;			     break;	\
	}										\
											\
	T2CONbits.TCKPS	= Prescaler;							\
	PR2 = CompareValue;								\
	TMR2 = 0x00;									\
	T2CONbits.TON = 1;})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/** Turn Off Timer2 And Reset Its Value. */
#define CloseTimer2()	({_T2IE = 0; T2CON = TMR2 = 0;})
/***/
#define EnableIntTimer2()		_T2IE = 1

#define DisableIntTimer2()		_T2IE = 0

#define SetIntPriorityTimer2(priority)	_T2IP = (priority)

#define ReadTimer2()			TMR2

#define WriteTimer2(value)		TMR2 = (value)

#define ReadCompareValueTimer2()	PR2

#define WriteCompareValueTimer2(value)	PR2 = (value)

#endif

/**********************************************************************************************************************/
/********************************************* Timer3 Macro Functions *************************************************/
/**********************************************************************************************************************/
#if defined _T3IE

/***********************************************************************************************************************
 * \Function		void SetupTimer3(uint16_t TimerMode, uint16_t Prescaler, uint16_t CompareValue)
 *
 * \Description		Configures Timer3.
 *
 * \Preconditions	 None.
 *
 * \Inputs
 * <u>TimerMode:</u>\n
 *	- NORMAL_TIMER.	<i>/ The input clock is derived from the internal instruction cycle clock (FCY).</i> \n
 *	- GATED_TIMER.	<i>/ The timer increments on every rising edge of the input clock as long as the external gate signal at TxCK pin is high.</i>\n
 *	- SYNC_COUNTER.	<i>/ The input clock to the timer is derived from the external clock input at TxCK pin divided by a programmable prescaler.
 *			     In This Mode The external clock input is synchronized with the internal device clock.</i>\n
 *
 * <u>Prescaler:</u>	\n
 *	- PRESCALER_1.	\n
 *	- PRESCALER_8.	\n
 *	- PRESCALER_64.	\n
 *	- PRESCALER_256.
 *
 * <u>CompareValue:</u> <i>/ When The Timer Register Match This Compare Value The Timer Will Reset And Will Generate An Interrupt (if Enabled).</i>\n
 *	- A value between 0 - 65535 inclusive.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupTimer3(NORMAL_TIMER,PRESCALER_64,4000);
 *
 **********************************************************************************************************************/
#define SetupTimer3(TimerMode,Prescaler,CompareValue)({					\
	switch (TimerMode) {								\
		case NORMAL_TIMER  : T3CONbits.TCS = 0; T3CONbits.TGATE = 0; break;	\
		case GATED_TIMER   : T3CONbits.TCS = 0; T3CONbits.TGATE = 1; break;	\
		case SYNC_COUNTER  : T3CONbits.TCS = 1;			     break;	\
	}										\
											\
	T3CONbits.TCKPS	= Prescaler;							\
	PR3 = CompareValue;								\
	TMR3 = 0x00;									\
	T3CONbits.TON = 1;})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/** Turn Off Timer3 And Reset Its Value. */
#define CloseTimer3()	({_T3IE = 0; T3CON = TMR3 = 0;})
/***/
#define EnableIntTimer3()		_T3IE = 1

#define DisableIntTimer3()		_T3IE = 0

#define SetIntPriorityTimer3(priority)	_T3IP = (priority)

#define ReadTimer3()			TMR3

#define WriteTimer3(value)		TMR3 = (value)

#define ReadCompareValueTimer3()	PR3

#define WriteCompareValueTimer3(value)	PR3 = (value)

#endif

/**********************************************************************************************************************/
/********************************************* Timer4 Macro Functions *************************************************/
/**********************************************************************************************************************/
#if defined _T4IE

/***********************************************************************************************************************
 * \Function		void SetupTimer4(uint16_t TimerMode, uint16_t Prescaler, uint16_t CompareValue)
 *
 * \Description		Configures Timer4.
 *
 * \Preconditions	 None.
 *
 * \Inputs
 * <u>TimerMode:</u>\n
 *	- NORMAL_TIMER.	<i>/ The input clock is derived from the internal instruction cycle clock (FCY).</i> \n
 *	- GATED_TIMER.	<i>/ The timer increments on every rising edge of the input clock as long as the external gate signal at TxCK pin is high.</i>\n
 *	- SYNC_COUNTER.	<i>/ The input clock to the timer is derived from the external clock input at TxCK pin divided by a programmable prescaler.
 *			     In This Mode The external clock input is synchronized with the internal device clock.</i>\n
 *
 * <u>Prescaler:</u>	\n
 *	- PRESCALER_1.	\n
 *	- PRESCALER_8.	\n
 *	- PRESCALER_64.	\n
 *	- PRESCALER_256.
 *
 * <u>CompareValue:</u> <i>/ When The Timer Register Match This Compare Value The Timer Will Reset And Will Generate An Interrupt (if Enabled).</i>\n
 *	- A value between 0 - 65535 inclusive.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupTimer4(NORMAL_TIMER,PRESCALER_64,4000);
 *
 **********************************************************************************************************************/
#define SetupTimer4(TimerMode,Prescaler,CompareValue)({					\
	switch (TimerMode) {								\
		case NORMAL_TIMER  : T4CONbits.TCS = 0; T4CONbits.TGATE = 0; break;	\
		case GATED_TIMER   : T4CONbits.TCS = 0; T4CONbits.TGATE = 1; break;	\
		case SYNC_COUNTER  : T4CONbits.TCS = 1;			     break;	\
	}										\
											\
	T4CONbits.TCKPS	= Prescaler;							\
	PR4 = CompareValue;								\
	TMR4 = 0x00;									\
	T4CONbits.TON = 1;})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/** Turn Off Timer4 And Reset Its Value. */
#define CloseTimer4()	({_T4IE = 0; T4CON = TMR4 = 0;})
/***/
#define EnableIntTimer4()		_T4IE = 1

#define DisableIntTimer4()		_T4IE = 0

#define SetIntPriorityTimer4(priority)	_T4IP = (priority)

#define ReadTimer4()			TMR4

#define WriteTimer4(value)		TMR4 = (value)

#define ReadCompareValueTimer4()	PR4

#define WriteCompareValueTimer4(value)	PR4 = (value)

#endif

/**********************************************************************************************************************/
/********************************************* Timer5 Macro Functions *************************************************/
/**********************************************************************************************************************/
#if defined _T5IE

/***********************************************************************************************************************
 * \Function		void SetupTimer5(uint16_t TimerMode, uint16_t Prescaler, uint16_t CompareValue)
 *
 * \Description		Configures Timer5.
 *
 * \Preconditions	 None.
 *
 * \Inputs
 * <u>TimerMode:</u>\n
 *	- NORMAL_TIMER.	<i>/ The input clock is derived from the internal instruction cycle clock (FCY).</i> \n
 *	- GATED_TIMER.	<i>/ The timer increments on every rising edge of the input clock as long as the external gate signal at TxCK pin is high.</i>\n
 *	- SYNC_COUNTER.	<i>/ The input clock to the timer is derived from the external clock input at TxCK pin divided by a programmable prescaler.
 *			     In This Mode The external clock input is synchronized with the internal device clock.</i>\n
 *
 * <u>Prescaler:</u>	\n
 *	- PRESCALER_1.	\n
 *	- PRESCALER_8.	\n
 *	- PRESCALER_64.	\n
 *	- PRESCALER_256.
 *
 * <u>CompareValue:</u> <i>/ When The Timer Register Match This Compare Value The Timer Will Reset And Will Generate An Interrupt (if Enabled).</i>\n
 *	- A value between 0 - 65535 inclusive.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupTimer5(NORMAL_TIMER,PRESCALER_64,4000);
 *
 **********************************************************************************************************************/
#define SetupTimer5(TimerMode,Prescaler,CompareValue)({					\
	switch (TimerMode) {								\
		case NORMAL_TIMER  : T5CONbits.TCS = 0; T5CONbits.TGATE = 0; break;	\
		case GATED_TIMER   : T5CONbits.TCS = 0; T5CONbits.TGATE = 1; break;	\
		case SYNC_COUNTER  : T5CONbits.TCS = 1;			     break;	\
	}										\
											\
	T5CONbits.TCKPS	= Prescaler;							\
	PR5 = CompareValue;								\
	TMR5 = 0x00;									\
	T5CONbits.TON = 1;})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/** Turn Off Timer5 And Reset Its Value. */
#define CloseTimer5()	({_T5IE = 0; T5CON = TMR5 = 0;})
/***/
#define EnableIntTimer5()		_T5IE = 1

#define DisableIntTimer5()		_T5IE = 0

#define SetIntPriorityTimer5(priority)	_T5IP = (priority)

#define ReadTimer5()			TMR5

#define WriteTimer5(value)		TMR5 = (value)

#define ReadCompareValueTimer5()	PR5

#define WriteCompareValueTimer5(value)	PR5 = (value)

#endif

/**********************************************************************************************************************/
/********************************************* Timer6 Macro Functions *************************************************/
/**********************************************************************************************************************/
#if defined _T6IE

/***********************************************************************************************************************
 * \Function		void SetupTimer6(uint16_t TimerMode, uint16_t Prescaler, uint16_t CompareValue)
 *
 * \Description		Configures Timer6.
 *
 * \Preconditions	 None.
 *
 * \Inputs
 * <u>TimerMode:</u>\n
 *	- NORMAL_TIMER.	<i>/ The input clock is derived from the internal instruction cycle clock (FCY).</i> \n
 *	- GATED_TIMER.	<i>/ The timer increments on every rising edge of the input clock as long as the external gate signal at TxCK pin is high.</i>\n
 *	- SYNC_COUNTER.	<i>/ The input clock to the timer is derived from the external clock input at TxCK pin divided by a programmable prescaler.
 *			     In This Mode The external clock input is synchronized with the internal device clock.</i>\n
 *
 * <u>Prescaler:</u>	\n
 *	- PRESCALER_1.	\n
 *	- PRESCALER_8.	\n
 *	- PRESCALER_64.	\n
 *	- PRESCALER_256.
 *
 * <u>CompareValue:</u> <i>/ When The Timer Register Match This Compare Value The Timer Will Reset And Will Generate An Interrupt (if Enabled).</i>\n
 *	- A value between 0 - 65535 inclusive.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupTimer6(NORMAL_TIMER,PRESCALER_64,4000);
 *
 **********************************************************************************************************************/
#define SetupTimer6(TimerMode,Prescaler,CompareValue)({					\
	switch (TimerMode) {								\
		case NORMAL_TIMER  : T6CONbits.TCS = 0; T6CONbits.TGATE = 0; break;	\
		case GATED_TIMER   : T6CONbits.TCS = 0; T6CONbits.TGATE = 1; break;	\
		case SYNC_COUNTER  : T6CONbits.TCS = 1;			     break;	\
	}										\
											\
	T6CONbits.TCKPS	= Prescaler;							\
	PR6 = CompareValue;								\
	TMR6 = 0x00;									\
	T6CONbits.TON = 1;})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/** Turn Off Timer6 And Reset Its Value. */
#define CloseTimer6()	({_T6IE = 0; T6CON = TMR6 = 0;})
/***/
#define EnableIntTimer6()		_T6IE = 1

#define DisableIntTimer6()		_T6IE = 0

#define SetIntPriorityTimer6(priority)	_T6IP = (priority)

#define ReadTimer6()			TMR6

#define WriteTimer6(value)		TMR6 = (value)

#define ReadCompareValueTimer6()	PR6

#define WriteCompareValueTimer6(value)	PR6 = (value)

#endif

/**********************************************************************************************************************/
/********************************************* Timer7 Macro Functions *************************************************/
/**********************************************************************************************************************/
#if defined _T7IE

/***********************************************************************************************************************
 * \Function		void SetupTimer7(uint16_t TimerMode, uint16_t Prescaler, uint16_t CompareValue)
 *
 * \Description		Configures Timer7.
 *
 * \Preconditions	 None.
 *
 * \Inputs
 * <u>TimerMode:</u>\n
 *	- NORMAL_TIMER.	<i>/ The input clock is derived from the internal instruction cycle clock (FCY).</i> \n
 *	- GATED_TIMER.	<i>/ The timer increments on every rising edge of the input clock as long as the external gate signal at TxCK pin is high.</i>\n
 *	- SYNC_COUNTER.	<i>/ The input clock to the timer is derived from the external clock input at TxCK pin divided by a programmable prescaler.
 *			     In This Mode The external clock input is synchronized with the internal device clock.</i>\n
 *
 * <u>Prescaler:</u>	\n
 *	- PRESCALER_1.	\n
 *	- PRESCALER_8.	\n
 *	- PRESCALER_64.	\n
 *	- PRESCALER_256.
 *
 * <u>CompareValue:</u> <i>/ When The Timer Register Match This Compare Value The Timer Will Reset And Will Generate An Interrupt (if Enabled).</i>\n
 *	- A value between 0 - 65535 inclusive.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupTimer7(NORMAL_TIMER,PRESCALER_64,4000);
 *
 **********************************************************************************************************************/
#define SetupTimer7(TimerMode,Prescaler,CompareValue)({					\
	switch (TimerMode) {								\
		case NORMAL_TIMER  : T7CONbits.TCS = 0; T7CONbits.TGATE = 0; break;	\
		case GATED_TIMER   : T7CONbits.TCS = 0; T7CONbits.TGATE = 1; break;	\
		case SYNC_COUNTER  : T7CONbits.TCS = 1;			     break;	\
	}										\
											\
	T7CONbits.TCKPS	= Prescaler;							\
	PR7 = CompareValue;								\
	TMR7 = 0x00;									\
	T7CONbits.TON = 1;})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/** Turn Off Timer7 And Reset Its Value. */
#define CloseTimer7()	({_T7IE = 0; T7CON = TMR7 = 0;})
/***/
#define EnableIntTimer7()		_T7IE = 1

#define DisableIntTimer7()		_T7IE = 0

#define SetIntPriorityTimer7(priority)	_T7IP = (priority)

#define ReadTimer7()			TMR7

#define WriteTimer7(value)		TMR7 = (value)

#define ReadCompareValueTimer7()	PR7

#define WriteCompareValueTimer7(value)	PR7 = (value)

#endif

/**********************************************************************************************************************/
/********************************************* Timer8 Macro Functions *************************************************/
/**********************************************************************************************************************/
#if defined _T8IE

/***********************************************************************************************************************
 * \Function		void SetupTimer8(uint16_t TimerMode, uint16_t Prescaler, uint16_t CompareValue)
 *
 * \Description		Configures Timer8.
 *
 * \Preconditions	 None.
 *
 * \Inputs
 * <u>TimerMode:</u>\n
 *	- NORMAL_TIMER.	<i>/ The input clock is derived from the internal instruction cycle clock (FCY).</i> \n
 *	- GATED_TIMER.	<i>/ The timer increments on every rising edge of the input clock as long as the external gate signal at TxCK pin is high.</i>\n
 *	- SYNC_COUNTER.	<i>/ The input clock to the timer is derived from the external clock input at TxCK pin divided by a programmable prescaler.
 *			     In This Mode The external clock input is synchronized with the internal device clock.</i>\n
 *
 * <u>Prescaler:</u>	\n
 *	- PRESCALER_1.	\n
 *	- PRESCALER_8.	\n
 *	- PRESCALER_64.	\n
 *	- PRESCALER_256.
 *
 * <u>CompareValue:</u> <i>/ When The Timer Register Match This Compare Value The Timer Will Reset And Will Generate An Interrupt (if Enabled).</i>\n
 *	- A value between 0 - 65535 inclusive.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupTimer8(NORMAL_TIMER,PRESCALER_64,4000);
 *
 **********************************************************************************************************************/
#define SetupTimer8(TimerMode,Prescaler,CompareValue)({					\
	switch (TimerMode) {								\
		case NORMAL_TIMER  : T8CONbits.TCS = 0; T8CONbits.TGATE = 0; break;	\
		case GATED_TIMER   : T8CONbits.TCS = 0; T8CONbits.TGATE = 1; break;	\
		case SYNC_COUNTER  : T8CONbits.TCS = 1;			     break;	\
	}										\
											\
	T8CONbits.TCKPS	= Prescaler;							\
	PR8 = CompareValue;								\
	TMR8 = 0x00;									\
	T8CONbits.TON = 1;})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/** Turn Off Timer8 And Reset Its Value. */
#define CloseTimer8()	({_T8IE = 0; T8CON = TMR8 = 0;})
/***/
#define EnableIntTimer8()		_T8IE = 1

#define DisableIntTimer8()		_T8IE = 0

#define SetIntPriorityTimer8(priority)	_T8IP = (priority)

#define ReadTimer8()			TMR8

#define WriteTimer8(value)		TMR8 = (value)

#define ReadCompareValueTimer8()	PR8

#define WriteCompareValueTimer8(value)	PR8 = (value)

#endif

/**********************************************************************************************************************/
/********************************************* Timer9 Macro Functions *************************************************/
/**********************************************************************************************************************/
#if defined _T9IE

/***********************************************************************************************************************
 * \Function		void SetupTimer9(uint16_t TimerMode, uint16_t Prescaler, uint16_t CompareValue)
 *
 * \Description		Configures Timer9.
 *
 * \Preconditions	 None.
 *
 * \Inputs
 * <u>TimerMode:</u>\n
 *	- NORMAL_TIMER.	<i>/ The input clock is derived from the internal instruction cycle clock (FCY).</i> \n
 *	- GATED_TIMER.	<i>/ The timer increments on every rising edge of the input clock as long as the external gate signal at TxCK pin is high.</i>\n
 *	- SYNC_COUNTER.	<i>/ The input clock to the timer is derived from the external clock input at TxCK pin divided by a programmable prescaler.
 *			     In This Mode The external clock input is synchronized with the internal device clock.</i>\n
 *
 * <u>Prescaler:</u>	\n
 *	- PRESCALER_1.	\n
 *	- PRESCALER_8.	\n
 *	- PRESCALER_64.	\n
 *	- PRESCALER_256.
 *
 * <u>CompareValue:</u> <i>/ When The Timer Register Match This Compare Value The Timer Will Reset And Will Generate An Interrupt (if Enabled).</i>\n
 *	- A value between 0 - 65535 inclusive.
 *
 * \Return      None.
 *
 * \Notes       None.
 *
 * \Example     SetupTimer9(NORMAL_TIMER,PRESCALER_64,4000);
 *
 **********************************************************************************************************************/
#define SetupTimer9(TimerMode,Prescaler,CompareValue)({					\
	switch (TimerMode) {								\
		case NORMAL_TIMER  : T9CONbits.TCS = 0; T9CONbits.TGATE = 0; break;	\
		case GATED_TIMER   : T9CONbits.TCS = 0; T9CONbits.TGATE = 1; break;	\
		case SYNC_COUNTER  : T9CONbits.TCS = 1;			     break;	\
	}										\
											\
	T9CONbits.TCKPS	= Prescaler;							\
	PR9 = CompareValue;								\
	TMR9 = 0x00;									\
	T9CONbits.TON = 1;})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/** Turn Off Timer9 And Reset Its Value. */
#define CloseTimer9()	({_T9IE = 0; T9CON = TMR9 = 0;})
/***/
#define EnableIntTimer9()		_T9IE = 1

#define DisableIntTimer9()		_T9IE = 0

#define SetIntPriorityTimer9(priority)	_T9IP = (priority)

#define ReadTimer9()			TMR9

#define WriteTimer9(value)		TMR9 = (value)

#define ReadCompareValueTimer9()	PR9

#define WriteCompareValueTimer9(value)	PR9 = (value)

#endif

/**********************************************************************************************************************/
/*************************************** Timer32Bit23 T2+T3 Macro Functions *******************************************/
/**********************************************************************************************************************/
#if defined _T3IE

/***********************************************************************************************************************
 * \Function		void SetupTimer32Bit23(uint16_t TimerMode,uint16_t Prescaler,uint16_t CompareValue)
 *
 * \Description		Configures Timer23 as 32-bit timer Constructed by pairing Timer 2 & 3.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>TimerMode:</u>\n
 *	- NORMAL_TIMER.	<i>/ The input clock is derived from the internal instruction cycle clock (FCY).</i> \n
 *	- GATED_TIMER.	<i>/ The timer increments on every rising edge of the input clock as long as the external gate signal at TxCK pin is high.</i>\n
 *	- SYNC_COUNTER.	<i>/ The input clock to the timer is derived from the external clock input at TxCK pin divided by a programmable prescaler.
 *			     In This Mode The external clock input is synchronized with the internal device clock.</i>\n
 *
 * <u>Prescaler:</u>	\n
 *	- PRESCALER_1.	\n
 *	- PRESCALER_8.	\n
 *	- PRESCALER_64.	\n
 *	- PRESCALER_256.
 *
 * <u>CompareValue:</u> <i>/ When The Timer Register Match This Compare Value The Timer Will Reset And Will Generate An Interrupt (if Enabled).</i>\n
 *	- A value between 0 - 4294967295 inclusive.
 *
 * \Return      None.
 *
 * \Notes       The prescaler counter is cleared when any of the following occurs:
 *		1- A write to the Timer register (TMRx) or Timer Control (TxCON) register.
 *		2- Clearing the Timer Enable (TON) bit in the Timer Control (TxCON<15>) register.
 *		3- Any device Reset.
 *
 * \Example     SetupTimer32Bit23(NORMAL_TIMER,PRESCALER_64,123456789);
 *
 **********************************************************************************************************************/
#define SetupTimer32Bit23(TimerMode,Prescaler,CompareValue)({				\
	T2CONbits.T32 = 1;								\
	switch (TimerMode) {								\
		case NORMAL_TIMER  : T2CONbits.TCS = 0; T2CONbits.TGATE = 0; break;	\
		case GATED_TIMER   : T2CONbits.TCS = 0; T2CONbits.TGATE = 1; break;	\
		case SYNC_COUNTER  : T2CONbits.TCS = 1;					\
	}										\
											\
	T2CONbits.TCKPS	= Prescaler;							\
	TMR3 = 0;									\
	TMR2 = 0;									\
	PR3 = CompareValue>>16;								\
	PR2 = (unsigned int)CompareValue;						\
	T2CONbits.TON = 1;								\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/** Turn Off Timer23 And Reset Its Value. */
#define CloseTimer32Bit23()			({_T3IE = 0; TMR3HLD = 0 ; T2CON = TMR2 = 0;})
/***/
#define EnableIntTimer32Bit23()			(_T3IE = 1)

#define DisableIntTimer32Bit23()		(_T3IE = 0)

#define SetIntPriorityTimer32Bit23(priority)	(_T3IP = priority)

#define ReadTimer32Bit23()			(TMR2+((unsigned long)TMR3HLD<<16))

#define WriteTimer32Bit23(Value)		({unsigned long _Value = Value; TMR3HLD = (_Value)>>16; TMR2 = (unsigned int)(_Value);})

#define ReadCompareValue32Bit23()		(PR2+((unsigned long)PR3<<16))

/***********************************************************************************************************************
 * \Note A special case occurs when the period register, PRx, is loaded with 0x0000 and the timer is enabled. No timer
 *	 interrupts are generated for this configuration.
 **********************************************************************************************************************/
#define WriteCompareValue32Bit23(Value)		({unsigned long _Value = Value; PR3 = (_Value)>>16; PR2 = (unsigned int)(_Value);})

#endif

/**********************************************************************************************************************/
/*************************************** Timer32Bit45 T4+T5 Macro Functions *******************************************/
/**********************************************************************************************************************/
#if defined _T5IE

/***********************************************************************************************************************
 * \Function		void SetupTimer32Bit45(uint16_t TimerMode,uint16_t Prescaler,uint16_t CompareValue)
 *
 * \Description		Configures Timer45 as 32-bit timer Constructed by pairing Timer 4 & 5.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>TimerMode:</u>\n
 *	- NORMAL_TIMER.	<i>/ The input clock is derived from the internal instruction cycle clock (FCY).</i> \n
 *	- GATED_TIMER.	<i>/ The timer increments on every rising edge of the input clock as long as the external gate signal at TxCK pin is high.</i>\n
 *	- SYNC_COUNTER.	<i>/ The input clock to the timer is derived from the external clock input at TxCK pin divided by a programmable prescaler.
 *			     In This Mode The external clock input is synchronized with the internal device clock.</i>\n
 *
 * <u>Prescaler:</u>	\n
 *	- PRESCALER_1.	\n
 *	- PRESCALER_8.	\n
 *	- PRESCALER_64.	\n
 *	- PRESCALER_256.
 *
 * <u>CompareValue:</u> <i>/ When The Timer Register Match This Compare Value The Timer Will Reset And Will Generate An Interrupt (if Enabled).</i>\n
 *	- A value between 0 - 4294967295 inclusive.
 *
 * \Return      None.
 *
 * \Notes       The prescaler counter is cleared when any of the following occurs:
 *		1- A write to the Timer register (TMRx) or Timer Control (TxCON) register.
 *		2- Clearing the Timer Enable (TON) bit in the Timer Control (TxCON<15>) register.
 *		3- Any device Reset.
 *
 * \Example     SetupTimer32Bit45(NORMAL_TIMER,PRESCALER_64,123456789);
 *
 **********************************************************************************************************************/
#define SetupTimer32Bit45(TimerMode,Prescaler,CompareValue)({				\
	T4CONbits.T32 = 1;								\
	switch (TimerMode) {								\
		case NORMAL_TIMER  : T4CONbits.TCS = 0; T4CONbits.TGATE = 0; break;	\
		case GATED_TIMER   : T4CONbits.TCS = 0; T4CONbits.TGATE = 1; break;	\
		case SYNC_COUNTER  : T4CONbits.TCS = 1;					\
	}										\
											\
	T4CONbits.TCKPS	= Prescaler;							\
	TMR5 = 0;									\
	TMR4 = 0;									\
	PR5 = CompareValue>>16;								\
	PR4 = (unsigned int)CompareValue;						\
	T4CONbits.TON = 1;								\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/** Turn Off Timer45 And Reset Its Value. */
#define CloseTimer32Bit45()			({_T5IE = 0; TMR5HLD = 0 ; T4CON = TMR4 = 0;})
/***/
#define EnableIntTimer32Bit45()			(_T5IE = 1)

#define DisableIntTimer32Bit45()		(_T5IE = 0)

#define SetIntPriorityTimer32Bit45(priority)	(_T5IP = priority)

#define ReadTimer32Bit45()			(TMR4+((unsigned long)TMR5HLD<<16))

#define WriteTimer32Bit45(Value)		({unsigned long _Value = Value; TMR5HLD = (_Value)>>16; TMR4 = (unsigned int)(_Value);})

#define ReadCompareValue32Bit45()		(PR4+((unsigned long)PR5<<16))

/***********************************************************************************************************************
 * \Note A special case occurs when the period register, PRx, is loaded with 0x0000 and the timer is enabled. No timer
 *	 interrupts are generated for this configuration.
 **********************************************************************************************************************/
#define WriteCompareValue32Bit45(Value)		({unsigned long _Value = Value; PR5 = (_Value)>>16; PR4 = (unsigned int)(_Value);})

#endif

/**********************************************************************************************************************/
/*************************************** Timer32Bit67 T6+T7 Macro Functions *******************************************/
/**********************************************************************************************************************/
#if defined _T7IE

/***********************************************************************************************************************
 * \Function		void SetupTimer32Bit67(uint16_t TimerMode,uint16_t Prescaler,uint16_t CompareValue)
 *
 * \Description		Configures Timer67 as 32-bit timer Constructed by pairing Timer 6 & 7.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>TimerMode:</u>\n
 *	- NORMAL_TIMER.	<i>/ The input clock is derived from the internal instruction cycle clock (FCY).</i> \n
 *	- GATED_TIMER.	<i>/ The timer increments on every rising edge of the input clock as long as the external gate signal at TxCK pin is high.</i>\n
 *	- SYNC_COUNTER.	<i>/ The input clock to the timer is derived from the external clock input at TxCK pin divided by a programmable prescaler.
 *			     In This Mode The external clock input is synchronized with the internal device clock.</i>\n
 *
 * <u>Prescaler:</u>	\n
 *	- PRESCALER_1.	\n
 *	- PRESCALER_8.	\n
 *	- PRESCALER_64.	\n
 *	- PRESCALER_256.
 *
 * <u>CompareValue:</u> <i>/ When The Timer Register Match This Compare Value The Timer Will Reset And Will Generate An Interrupt (if Enabled).</i>\n
 *	- A value between 0 - 4294967295 inclusive.
 *
 * \Return      None.
 *
 * \Notes       The prescaler counter is cleared when any of the following occurs:
 *		1- A write to the Timer register (TMRx) or Timer Control (TxCON) register.
 *		2- Clearing the Timer Enable (TON) bit in the Timer Control (TxCON<15>) register.
 *		3- Any device Reset.
 *
 * \Example     SetupTimer32Bit67(NORMAL_TIMER,PRESCALER_64,123456789);
 *
 **********************************************************************************************************************/
#define SetupTimer32Bit67(TimerMode,Prescaler,CompareValue)({				\
	T6CONbits.T32 = 1;								\
	switch (TimerMode) {								\
		case NORMAL_TIMER  : T6CONbits.TCS = 0; T6CONbits.TGATE = 0; break;	\
		case GATED_TIMER   : T6CONbits.TCS = 0; T6CONbits.TGATE = 1; break;	\
		case SYNC_COUNTER  : T6CONbits.TCS = 1;					\
	}										\
											\
	T6CONbits.TCKPS	= Prescaler;							\
	TMR7 = 0;									\
	TMR6 = 0;									\
	PR7 = CompareValue>>16;								\
	PR6 = (unsigned int)CompareValue;						\
	T6CONbits.TON = 1;								\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/** Turn Off Timer67 And Reset Its Value. */
#define CloseTimer32Bit67()			({_T7IE = 0; TMR7HLD = 0 ; T6CON = TMR6 = 0;})
/***/
#define EnableIntTimer32Bit67()			(_T7IE = 1)

#define DisableIntTimer32Bit67()		(_T7IE = 0)

#define SetIntPriorityTimer32Bit67(priority)	(_T7IP = priority)

#define ReadTimer32Bit67()			(TMR6+((unsigned long)TMR7HLD<<16))

#define WriteTimer32Bit67(Value)		({unsigned long _Value = Value; TMR7HLD = (_Value)>>16; TMR6 = (unsigned int)(_Value);})

#define ReadCompareValue32Bit67()		(PR6+((unsigned long)PR7<<16))

/***********************************************************************************************************************
 * \Note A special case occurs when the period register, PRx, is loaded with 0x0000 and the timer is enabled. No timer
 *	 interrupts are generated for this configuration.
 **********************************************************************************************************************/
#define WriteCompareValue32Bit67(Value)		({unsigned long _Value = Value; PR7 = (_Value)>>16; PR6 = (unsigned int)(_Value);})

#endif

/**********************************************************************************************************************/
/*************************************** Timer32Bit89 T8+T9 Macro Functions *******************************************/
/**********************************************************************************************************************/
#if defined _T9IE

/***********************************************************************************************************************
 * \Function		void SetupTimer32Bit89(uint16_t TimerMode,uint16_t Prescaler,uint16_t CompareValue)
 *
 * \Description		Configures Timer89 as 32-bit timer Constructed by pairing Timer 8 & 9.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>TimerMode:</u>\n
 *	- NORMAL_TIMER.	<i>/ The input clock is derived from the internal instruction cycle clock (FCY).</i> \n
 *	- GATED_TIMER.	<i>/ The timer increments on every rising edge of the input clock as long as the external gate signal at TxCK pin is high.</i>\n
 *	- SYNC_COUNTER.	<i>/ The input clock to the timer is derived from the external clock input at TxCK pin divided by a programmable prescaler.
 *			     In This Mode The external clock input is synchronized with the internal device clock.</i>\n
 *
 * <u>Prescaler:</u>	\n
 *	- PRESCALER_1.	\n
 *	- PRESCALER_8.	\n
 *	- PRESCALER_64.	\n
 *	- PRESCALER_256.
 *
 * <u>CompareValue:</u> <i>/ When The Timer Register Match This Compare Value The Timer Will Reset And Will Generate An Interrupt (if Enabled).</i>\n
 *	- A value between 0 - 4294967295 inclusive.
 *
 * \Return      None.
 *
 * \Notes       The prescaler counter is cleared when any of the following occurs:
 *		1- A write to the Timer register (TMRx) or Timer Control (TxCON) register.
 *		2- Clearing the Timer Enable (TON) bit in the Timer Control (TxCON<15>) register.
 *		3- Any device Reset.
 *
 * \Example     SetupTimer32Bit89(NORMAL_TIMER,PRESCALER_64,123456789);
 *
 **********************************************************************************************************************/
#define SetupTimer32Bit89(TimerMode,Prescaler,CompareValue)({				\
	T8CONbits.T32 = 1;								\
	switch (TimerMode) {								\
		case NORMAL_TIMER  : T8CONbits.TCS = 0; T8CONbits.TGATE = 0; break;	\
		case GATED_TIMER   : T8CONbits.TCS = 0; T8CONbits.TGATE = 1; break;	\
		case SYNC_COUNTER  : T8CONbits.TCS = 1;					\
	}										\
											\
	T8CONbits.TCKPS	= Prescaler;							\
	TMR9 = 0;									\
	TMR8 = 0;									\
	PR9 = CompareValue>>16;								\
	PR8 = (unsigned int)CompareValue;						\
	T8CONbits.TON = 1;								\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/** Turn Off Timer89 And Reset Its Value. */
#define CloseTimer32Bit89()			({_T9IE = 0; TMR9HLD = 0 ; T8CON = TMR8 = 0;})
/***/
#define EnableIntTimer32Bit89()			(_T9IE = 1)

#define DisableIntTimer32Bit89()		(_T9IE = 0)

#define SetIntPriorityTimer32Bit89(priority)	(_T9IP = priority)

#define ReadTimer32Bit89()			(TMR8+((unsigned long)TMR9HLD<<16))

#define WriteTimer32Bit89(Value)		({unsigned long _Value = Value; TMR9HLD = (_Value)>>16; TMR8 = (unsigned int)(_Value);})

#define ReadCompareValue32Bit89()		(PR8+((unsigned long)PR9<<16))

/***********************************************************************************************************************
 * \Note A special case occurs when the period register, PRx, is loaded with 0x0000 and the timer is enabled. No timer
 *	 interrupts are generated for this configuration.
 **********************************************************************************************************************/
#define WriteCompareValue32Bit89(Value)		({unsigned long _Value = Value; PR9 = (_Value)>>16; PR8 = (unsigned int)(_Value);})

#endif


#endif	/* _LIBTIMER_ */

