
/*
 * QEI (Quadrature Encoder Interface) Libary For dsPIC33EP/PIC24EP Devices.
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
 * File: $Id: LibQEI.h, V1.1 2015/01/15 AL-Moutaz Billah Tabbakha Exp $
 *
 **************************************************************************************************
 * - Macro Functions Represented in This Library: ("x" Denotes to QEI Module Number 1 or 2)
 *
 * void Setup32BitQEIx(CounterMode,CounterInitMode,InputPrescaler,CountPolarity,InputCapEN,IdelEN)
 * void Setup32BitQEIxIO( InputFilterDiv, IndexMatchValue, SwapEN, QEAPol, QEBPol,IndexPol, HomePol, OutputMode)
 * void Setup32BitQEIxInterrupt(IntPriority,CountOverFlowIntEN,IndxIntEN,HomeIntEN,VelocityIntEN,LesserOrEqualIntEN,GreaterOrEqualIntEN,InitializDoneIntEN)
 * void EnableIntQEIx()
 * void DisableIntQEIx()
 * void SetPriorityIntQEIx(priority)
 * unsigned long Read32bitQEIxPositionCounter(void)
 * unsigned long Read32bitQEIxCapture(void)
 * unsigned long Read32bitQEIxIndexCounter(void)
 * unsigned long Read32bitQEI2IntervalTimer(void)
 * unsigned int Read32bitQEIxVelocityCounter(void)
 * void Write32bitQEIxGreaterEqual(qeiCounter *ptr)
 * void Write32bitQEIxIndexCounter(qeiCounter *ptr)
 * void Write32bitQEIxInitialization(qeiCounter *ptr)
 * void Write32bitQEIxIntervalTimer(qeiCounter *ptr)
 * void Write32bitQEIxLesserEqual(qeiCounter *ptr)
 * void Write32bitQEIxPositionCounter(qeiCounter *ptr)
 * void Close32bitQEIx(void)
 *
 **************************************************************************************************
 * - QEIx Status Bits.
 *
 * QEIx_INDEX_OCCURRED
 * QEIx_HOME_OCCURRED
 * QEIx_VELO_OVERFLOW_OCCURRED
 * QEIx_POS_INIT_COMPLETE
 * QEIx_POS_OVERFLOW_OCCURRED
 * QEIx_POS_LESS_EQU_OCCURRED
 * QEIx_POS_GREAT_EQU_OCCURRED
 */

#ifndef _LIBQEI_
#define _LIBQEI_

typedef union{
    struct{
        unsigned int  LoWord;       	// Counter high word
        unsigned int  HiWord;       	// Counter low word
    }f;                     		// field access
    unsigned int      w[2];       	// 16 bits access
    unsigned long     l;          	// 32 bits access
}uQEI_Counter;
extern uQEI_Counter	QEI_Enc1_Pos, QEI_Enc2_Pos;

/**********************************************************************************************************************/
/************************************************ QEI1 Macro Functions ************************************************/
/**********************************************************************************************************************/
#if defined _QEI1IF

/***********************************************************************************************************************
 * \Function		void Setup32BitQEI1(CounterMode,CounterInitMode,InputPrescaler,CountPolarity,InputCapEN,IdelEN)
 *
 * \Description		Configures QEI1 Module Operation Parameters.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>CounterMode:</u>			\n
 *	- QUADRATURE_COUNTER_MODE	\n
 *	- UPDOWN_COUNTER_MODE		\n
 *	- NORMAL_COUNTER_MODE		\n
 *	- GATED_COUNTER_MODE		\n
 *	- NORMAL_TIMER_MODE		\n
 *	- GATED_TIMER_MODE
 *
 * <u>CounterInitMode:</u>	<i>/Refer to (15.3.8 Position Counter Initialization Modes) page(28) in Section 15.Quadrature Encoder Interface.</i>\n
 *	- POS_COUNT_NO_INIT		<i>/ Index does not affect position counter.</i> \n
 *	- POS_COUNT_INIT_EVERY_INDEX	<i>/ Every index input resets position counter.</i> \n
 *	- POS_COUNT_INIT_NEXT_INDEX	<i>/ Next index input moves content of QEIxIC into Position counter.</i> \n
 *	- POS_COUNT_INIT_FIRST_INDEX	<i>/ First index event after home moves content of QEIxIC into Position counter.</i> \n
 *	- POS_COUNT_INIT_SECOND_INDEX	<i>/ Second index event after home moves content of QEIxIC into Position counter.</i> \n
 *	- POS_COUNT_INIT_EQUALS_QEIGEC	<i>/ Resets position counter when position counter equals QEIxGEC.</i> \n
 *	- POS_COUNT_INIT_MODULO_COUNT	<i>/ Modulo count mode for position counter.</i> \n
 *
 * <u>InputPrescaler:</u>	<i>Timer/Counter input clock prescalar Applied to (interval timer, main timer (position counter),
 *				   velocity counter and index counter internal clock divider select).</i>\n
 *	- QEI_INPUT_PRESCALE_1		\n
 *	- QEI_INPUT_PRESCALE_2		\n
 *	- QEI_INPUT_PRESCALE_4		\n
 *	- QEI_INPUT_PRESCALE_8		\n
 *	- QEI_INPUT_PRESCALE_16		\n
 *	- QEI_INPUT_PRESCALE_32		\n
 *	- QEI_INPUT_PRESCALE_64		\n
 *	- QEI_INPUT_PRESCALE_256
 *
 * <u>CountPolarity:</u>
 *	- COUNT_POSITIVE	<i>/ Counter direction is positive.</i> \n
 *	- COUNT_NEGATIVE	<i>/ Counter direction is negative.</i>
 *
 * <u>InputCapEN:</u>
 *	- DIS_INPUT_CAPTURE	<i>/ capture function is Disabled.</i> \n
 *	- EN_INPUT_CAPTURE	<i>/ Positive edge detected at home input triggers position capture function.</i>
 *
 * <u>IdelEN:</u>
 *	- CONTINUE_IN_IDEL	<i>/ Continue QEI in Idle mode.</i> \n
 *	- STOP_IN_IDEL		<i>/ Stop QEI in Idle mode.</i>
 *
 * \Return      None.
 *
 * \Notes       The selected clock rate should be at least twice the expected maximum quadrature count rate.
 *
 * \Example     Setup32BitQEI1(	QUADRATURE_COUNTER_MODE,
 *				POS_COUNT_NO_INIT,
 *				QEI_INPUT_PRESCALE_1,
 *				COUNT_POSITIVE,
 *				DIS_INPUT_CAPTURE,
 *				KEEP_DEFAULT);
 *
 **********************************************************************************************************************/
#define Setup32BitQEI1(CounterMode,CounterInitMode,InputPrescaler,CountPolarity,InputCapEN,IdelEN)({	\
													\
	QEI1CONbits.CCM		= (CounterMode)&0b011;							\
	QEI1CONbits.GATEN	= CounterMode>>2;							\
	QEI1CONbits.PIMOD	= CounterInitMode;							\
	QEI1CONbits.INTDIV	= InputPrescaler;							\
	QEI1CONbits.CNTPOL	= CountPolarity;							\
	QEI1IOCbits.QCAPEN	= InputCapEN;								\
	QEI1CONbits.QEISIDL	= IdelEN;								\
	QEI1CONbits.QEIEN	= 1;									\
})

/***********************************************************************************************************************
 * \Function		void Setup32BitQEI1IO( InputFilterDiv, IndexMatchValue, SwapEN, QEAPol, QEBPol,IndexPol, HomePol, OutputMode)
 *
 * \Description		Configures QEI1 Inputs and Outputs.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * 
 * <u>InputFilterDiv:</u>	<i>/ Digital filter clock prescalar applied at QEAx/QEBx/INDXx/HOMEx Digital Inputs.</i> \n
 *	- DIV_FLTR_PRESCALE_DIS <i>/ Max QEI Input Frequency is FCY.</i>\n
 *	- DIV_FLTR_PRESCALE_1	<i>/ Max QEI Input Frequency is FCY/3.</i>\n
 *	- DIV_FLTR_PRESCALE_2	<i>/ Max QEI Input Frequency is FCY/6.</i>\n
 *	- DIV_FLTR_PRESCALE_4	<i>/ Max QEI Input Frequency is FCY/12.</i>\n
 *	- DIV_FLTR_PRESCALE_8	<i>/ Max QEI Input Frequency is FCY/24.</i>\n
 *	- DIV_FLTR_PRESCALE_16	<i>/ Max QEI Input Frequency is FCY/48.</i>\n
 *	- DIV_FLTR_PRESCALE_32	<i>/ Max QEI Input Frequency is FCY/96.</i>\n
 *	- DIV_FLTR_PRESCALE_64	<i>/ Max QEI Input Frequency is FCY/192.</i>\n
 *	- DIV_FLTR_PRESCALE_256	<i>/ Max QEI Input Frequency is FCY/768.</i>
 *
 * <u>IndexMatchValue:</u>\n
 *	- INDEX_MATCH_NO_EFFECT		<i>/ Index does not affect position counter.</i> \n
 *	- INDEX_MATCH_QEB_0_QEA_1	<i>/ Index match when QEB = 0 and QEA = 1.</i> \n
 *	- INDEX_MATCH_QEB_1_QEA_0	<i>/ Index match when QEB = 1 and QEA = 0.</i> \n
 *	- INDEX_MATCH_QEB_1_QEA_1	<i>/ Index match when QEB = 1 and QEA = 1.</i>
 *
 * <u>SwapEN:</u>\n
 *	- QEA_QEB_NOT_SWAPPED		<i>/ QEA QEB inputs are not swapped.</i> \n
 *	- QEA_QEB_SWAPPED		<i>/ QEA QEB inputs are swapped.</i>
 *
 * <u>QEAPol:</u>\n
 *	- QEA_POL_NON_INVERTED		<i>/ QEA input polarity is non-inverted.</i> \n
 *	- QEA_POL_INVERTED		<i>/ QEA input polarity is inverted.</i>
 *
 * <u>QEBPol:</u>\n
 *	- QEB_POL_NON_INVERTED		<i>/ QEB input polarity is non-inverted.</i> \n
 *	- QEB_POL_INVERTED		<i>/ QEB input polarity is inverted.</i>
 *
 * <u>IndexPol:</u>\n
 *	- INDX_POL_NON_INVERTED		<i>/ Index input polarity is non-inverted.</i> \n
 *	- INDX_POL_INVERTED		<i>/ Index input polarity is inverted.</i>
 *
 * <u>HomePol:</u>\n
 *	- HOM_POL_NON_INVERTED		<i>/ Home input polarity is non-inverted.</i> \n
 *	- HOM_POL_INVERTED		<i>/ Home input polarity is inverted.</i>
 *
 * <u>OutputMode:</u>\n
 *	- DISABLE_CTNCMP_OUTPUT		<i>/ Output is disabled.</i> \n
 *	- OUTPUT_ON_GREATER_QEIGEC	<i>/ CTNCMP pin goes high whn POSCNT Greater or Equla QEIGEC.</i> \n
 *	- OUTPUT_ON_LESSER_QEILEC	<i>/ CTNCMP pin goes high whn POSCNT Lesser or Equla QEILEC.</i> \n
 *	- OUTPUT_ON_GREATER_LESSER	<i>/ CTNCMP pin goes high whn POSCNT Greater or Equla QEIGEC or POSCNT Lesser or Equla QEILEC.</i>
 *
 * \Return      None.
 *
 * \Notes       The selected clock rate should be at least twice the expected maximum quadrature count rate.
 *
 * \Example     Setup32BitQEI1IO(DIV_FLTR_PRESCALE_64,
 *				INDEX_MATCH_QEB_1_QEA_1,
 *				QEA_QEB_NOT_SWAPPED,
 *				QEA_POL_NON_INVERTED,
 *				QEB_POL_NON_INVERTED,
 *				INDX_POL_NON_INVERTED,
 *				HOM_POL_NON_INVERTED,
 *				DISABLE_CTNCMP_OUTPUT);
 *
 **********************************************************************************************************************/
#define Setup32BitQEI1IO( InputFilterDiv, IndexMatchValue, SwapEN, QEAPol, QEBPol,IndexPol, HomePol, OutputMode)({	\
															\
	QEI1IOCbits.QFDIV	= (InputFilterDiv)&0b0111;								\
	QEI1IOCbits.FLTREN	= (~(InputFilterDiv>>3))&0b00000001;					\
	QEI1CONbits.IMV		= IndexMatchValue;									\
	QEI1IOCbits.SWPAB	= SwapEN;										\
	QEI1IOCbits.QEAPOL	= QEAPol;										\
	QEI1IOCbits.QEBPOL	= QEBPol;										\
	QEI1IOCbits.IDXPOL	= IndexPol;										\
	QEI1IOCbits.HOMPOL	= HomePol;										\
	QEI1IOCbits.OUTFNC	= OutputMode;										\
})

/***********************************************************************************************************************
 * \Function		void Setup32BitQEI1Interrupt( IntPriority, CountOverFlowIntEN, IndxIntEN, HomeIntEN, VelocityIntEN,
 *						      LesserOrEqualIntEN, GreaterOrEqualIntEN, InitializDoneIntEN)
 * \Description		Configures QEI1 Interrupts.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>IntPriority:</u>\n
 *	- A value Between 0 to 7 inclusively.
 *
 * <u>CountOverFlowIntEN:</u>\n
 *	- DIS_INT_POS_OVERFLOW		<i>/ Position counter overflow interrupt disabled.</i>\n
 *	- EN_INT_POS_OVERFLOW		<i>/ Position counter overflow interrupt enabled.</i>\n
 *
 * <u>IndxIntEN:</u>\n
 *	- DIS_INT_INDEX			<i>/ Index input event interrupt disabled.</i>\n
 *	- EN_INT_INDEX			<i>/ Index input event interrupt enabled.</i>\n
 *
 * <u>HomeIntEN:</u>\n
 *	- DIS_INT_HOME			<i>/ Home input event interrupt disabled.</i>\n
 *	- EN_INT_HOME			<i>/ Home input event interrupt enabled.</i>\n
 *
 * <u>VelocityIntEN:</u>\n
 *	- DIS_INT_VELO_OVERFLOW		<i>/ Velocity counter overflow interrupt disabled.</i>\n
 *	- EN_INT_VELO_OVERFLOW		<i>/ Velocity counter overflow interrupt enabled.</i>\n
 *
 * <u>LesserOrEqualIntEN:</u>\n
 *	- DIS_INT_POS_LESS_EQU		<i>/ Position counter less than or equal QEILEC interrupt disabled.</i>\n
 *	- EN_INT_POS_LESS_EQU		<i>/ Position counter less than or equal QEILEC interrupt enabled.</i>\n
 *
 * <u>GreaterOrEqualIntEN:</u>\n
 *	- DIS_INT_POS_GREAT_EQU		<i>/ Position counter greater than or equal QEIGEC interrupt disabled.</i>\n
 *	- EN_INT_POS_GREAT_EQU		<i>/ Position counter greater than or equal QEIGEC interrupt enabled.</i>\n
 *
 * <u>InitializDoneIntEN:</u>\n
 *	- DIS_INT_POS_INIT		<i>/ Position counter init complete interrupt disabled.</i>\n
 *	- EN_INT_POS_INIT		<i>/ Position counter init complete interrupt enabled.</i>\n
 *
 * \Return      None.
 *
 * \Notes       The selected clock rate should be at least twice the expected maximum quadrature count rate.
 *
 * \Example     Setup32BitQEI1Interrupt(4,
 *					EN_INT_POS_OVERFLOW,
 *					DIS_INT_INDEX,
 *					DIS_INT_HOME,
 *					DIS_INT_VELO_OVERFLOW,
 *					DIS_INT_POS_LESS_EQU,
 *					DIS_INT_POS_GREAT_EQU,
 *					DIS_INT_POS_INIT);
 *
 **********************************************************************************************************************/
#define Setup32BitQEI1Interrupt(IntPriority, CountOverFlowIntEN, IndxIntEN, HomeIntEN, VelocityIntEN, 	\
				LesserOrEqualIntEN, GreaterOrEqualIntEN, InitializDoneIntEN)({		\
													\
	_QEI1IP			= IntPriority;								\
	QEI1STATbits.POSOVIEN	= CountOverFlowIntEN;							\
	QEI1STATbits.IDXIEN	= IndxIntEN;								\
	QEI1STATbits.HOMIEN	= HomeIntEN;								\
	QEI1STATbits.VELOVIEN	= VelocityIntEN;							\
	QEI1STATbits.PCLEQIEN	= LesserOrEqualIntEN;							\
	QEI1STATbits.PCHEQIEN	= GreaterOrEqualIntEN;							\
	QEI1STATbits.PCIIEN	= InitializDoneIntEN;							\
})

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#define EnableIntQEI1()			_QEI1IE = 1
#define DisableIntQEI1()		_QEI1IE = 0
#define SetPriorityIntQEI1(priority)	_QEI1IP = (priority)

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/** QEI1 Status Bits */

/**Status Flag for Index Event Status bit*/
#define QEI1_INDEX_OCCURRED			(QEI1STATbits.IDXIRQ)
/**Status Flag for Home Event Status bit*/
#define QEI1_HOME_OCCURRED			(QEI1STATbits.HOMIRQ)
/**Velocity Counter Overflow Status bit*/
#define QEI1_VELO_OVERFLOW_OCCURRED		(QEI1STATbits.VELOVIRQ)
/**Position Counter (Homing) Initialization Process Complete Status bit*/
#define QEI1_POS_INIT_COMPLETE			(QEI1STATbits.PCIIRQ)
/**Position Counter Overflow Status bit*/
#define QEI1_POS_OVERFLOW_OCCURRED		(QEI1STATbits.POSOVIRQ)
/**Position Counter Less Than or Equal Compare Status bit*/
#define QEI1_POS_LESS_EQU_OCCURRED		(QEI1STATbits.PCLEQIRQ)
/**Position Counter Greater Than or Equal Compare Status bit*/
#define QEI1_POS_GREAT_EQU_OCCURRED		(QEI1STATbits.PCHEQIRQ)	

#endif

/**********************************************************************************************************************/
/************************************************ QEI2 Macro Functions ************************************************/
/**********************************************************************************************************************/
#if defined _QEI2IF

/***********************************************************************************************************************
 * \Function		void Setup32BitQEI2(CounterMode,CounterInitMode,InputPrescaler,CountPolarity,InputCapEN,IdelEN)
 *
 * \Description		Configures QEI2 Module Operation Parameters.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>CounterMode:</u>			\n
 *	- QUADRATURE_COUNTER_MODE	\n
 *	- UPDOWN_COUNTER_MODE		\n
 *	- NORMAL_COUNTER_MODE		\n
 *	- GATED_COUNTER_MODE		\n
 *	- NORMAL_TIMER_MODE		\n
 *	- GATED_TIMER_MODE
 *
 * <u>CounterInitMode:</u>	<i>/Refer to (15.3.8 Position Counter Initialization Modes) page(28) in Section 15.Quadrature Encoder Interface.</i>\n
 *	- POS_COUNT_NO_INIT		<i>/ Index does not affect position counter.</i> \n
 *	- POS_COUNT_INIT_EVERY_INDEX	<i>/ Every index input resets position counter.</i> \n
 *	- POS_COUNT_INIT_NEXT_INDEX	<i>/ Next index input moves content of QEIxIC into Position counter.</i> \n
 *	- POS_COUNT_INIT_FIRST_INDEX	<i>/ First index event after home moves content of QEIxIC into Position counter.</i> \n
 *	- POS_COUNT_INIT_SECOND_INDEX	<i>/ Second index event after home moves content of QEIxIC into Position counter.</i> \n
 *	- POS_COUNT_INIT_EQUALS_QEIGEC	<i>/ Resets position counter when position counter equals QEIxGEC.</i> \n
 *	- POS_COUNT_INIT_MODULO_COUNT	<i>/ Modulo count mode for position counter.</i> \n
 *
 * <u>InputPrescaler:</u>	<i>Timer/Counter input clock prescalar Applied to (interval timer, main timer (position counter),
 *				   velocity counter and index counter internal clock divider select).</i>\n
 *	- QEI_INPUT_PRESCALE_1		\n
 *	- QEI_INPUT_PRESCALE_2		\n
 *	- QEI_INPUT_PRESCALE_4		\n
 *	- QEI_INPUT_PRESCALE_8		\n
 *	- QEI_INPUT_PRESCALE_16		\n
 *	- QEI_INPUT_PRESCALE_32		\n
 *	- QEI_INPUT_PRESCALE_64		\n
 *	- QEI_INPUT_PRESCALE_256
 *
 * <u>CountPolarity:</u>
 *	- COUNT_POSITIVE	<i>/ Counter direction is positive.</i> \n
 *	- COUNT_NEGATIVE	<i>/ Counter direction is negative.</i>
 *
 * <u>InputCapEN:</u>
 *	- DIS_INPUT_CAPTURE	<i>/ capture function is Disabled.</i> \n
 *	- EN_INPUT_CAPTURE	<i>/ Positive edge detected at home input triggers position capture function.</i>
 *
 * <u>IdelEN:</u>
 *	- CONTINUE_IN_IDEL	<i>/ Continue QEI in Idle mode.</i> \n
 *	- STOP_IN_IDEL		<i>/ Stop QEI in Idle mode.</i>
 *
 * \Return      None.
 *
 * \Notes       The selected clock rate should be at least twice the expected maximum quadrature count rate.
 *
 * \Example     Setup32BitQEI2(	QUADRATURE_COUNTER_MODE,
 *				POS_COUNT_NO_INIT,
 *				QEI_INPUT_PRESCALE_1,
 *				COUNT_POSITIVE,
 *				DIS_INPUT_CAPTURE,
 *				KEEP_DEFAULT);
 *
 **********************************************************************************************************************/
#define Setup32BitQEI2(CounterMode,CounterInitMode,InputPrescaler,CountPolarity,InputCapEN,IdelEN)({	\
													\
	QEI2CONbits.CCM		= (CounterMode)&0b011;							\
	QEI2CONbits.GATEN	= CounterMode>>2;							\
	QEI2CONbits.PIMOD	= CounterInitMode;							\
	QEI2CONbits.INTDIV	= InputPrescaler;							\
	QEI2CONbits.CNTPOL	= CountPolarity;							\
	QEI2IOCbits.QCAPEN	= InputCapEN;								\
	QEI2CONbits.QEISIDL	= IdelEN;								\
	QEI2CONbits.QEIEN	= 1;									\
})

/***********************************************************************************************************************
 * \Function		void Setup32BitQEI2IO( InputFilterDiv, IndexMatchValue, SwapEN, QEAPol, QEBPol,IndexPol, HomePol, OutputMode)
 *
 * \Description		Configures QEI2 Inputs and Outputs.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>InputFilterDiv:</u>	<i>/ Digital filter clock prescalar applied at QEAx/QEBx/INDXx/HOMEx Digital Inputs.</i> \n
 *	- DIV_FLTR_PRESCALE_DIS <i>/ Max QEI Input Frequency is FCY.</i>\n
 *	- DIV_FLTR_PRESCALE_1	<i>/ Max QEI Input Frequency is FCY/3.</i>\n
 *	- DIV_FLTR_PRESCALE_2	<i>/ Max QEI Input Frequency is FCY/6.</i>\n
 *	- DIV_FLTR_PRESCALE_4	<i>/ Max QEI Input Frequency is FCY/12.</i>\n
 *	- DIV_FLTR_PRESCALE_8	<i>/ Max QEI Input Frequency is FCY/24.</i>\n
 *	- DIV_FLTR_PRESCALE_16	<i>/ Max QEI Input Frequency is FCY/48.</i>\n
 *	- DIV_FLTR_PRESCALE_32	<i>/ Max QEI Input Frequency is FCY/96.</i>\n
 *	- DIV_FLTR_PRESCALE_64	<i>/ Max QEI Input Frequency is FCY/192.</i>\n
 *	- DIV_FLTR_PRESCALE_256	<i>/ Max QEI Input Frequency is FCY/768.</i>
 *
 * <u>IndexMatchValue:</u>\n
 *	- INDEX_MATCH_NO_EFFECT		<i>/ Index does not affect position counter.</i> \n
 *	- INDEX_MATCH_QEB_0_QEA_1	<i>/ Index match when QEB = 0 and QEA = 1.</i> \n
 *	- INDEX_MATCH_QEB_1_QEA_0	<i>/ Index match when QEB = 1 and QEA = 0.</i> \n
 *	- INDEX_MATCH_QEB_1_QEA_1	<i>/ Index match when QEB = 1 and QEA = 1.</i>
 *
 * <u>SwapEN:</u>\n
 *	- QEA_QEB_NOT_SWAPPED		<i>/ QEA QEB inputs are not swapped.</i> \n
 *	- QEA_QEB_SWAPPED		<i>/ QEA QEB inputs are swapped.</i>
 *
 * <u>QEAPol:</u>\n
 *	- QEA_POL_NON_INVERTED		<i>/ QEA input polarity is non-inverted.</i> \n
 *	- QEA_POL_INVERTED		<i>/ QEA input polarity is inverted.</i>
 *
 * <u>QEBPol:</u>\n
 *	- QEB_POL_NON_INVERTED		<i>/ QEB input polarity is non-inverted.</i> \n
 *	- QEB_POL_INVERTED		<i>/ QEB input polarity is inverted.</i>
 *
 * <u>IndexPol:</u>\n
 *	- INDX_POL_NON_INVERTED		<i>/ Index input polarity is non-inverted.</i> \n
 *	- INDX_POL_INVERTED		<i>/ Index input polarity is inverted.</i>
 *
 * <u>HomePol:</u>\n
 *	- HOM_POL_NON_INVERTED		<i>/ Home input polarity is non-inverted.</i> \n
 *	- HOM_POL_INVERTED		<i>/ Home input polarity is inverted.</i>
 *
 * <u>OutputMode:</u>\n
 *	- DISABLE_CTNCMP_OUTPUT		<i>/ Output is disabled.</i> \n
 *	- OUTPUT_ON_GREATER_QEIGEC	<i>/ CTNCMP pin goes high whn POSCNT Greater or Equla QEIGEC.</i> \n
 *	- OUTPUT_ON_LESSER_QEILEC	<i>/ CTNCMP pin goes high whn POSCNT Lesser or Equla QEILEC.</i> \n
 *	- OUTPUT_ON_GREATER_LESSER	<i>/ CTNCMP pin goes high whn POSCNT Greater or Equla QEIGEC or POSCNT Lesser or Equla QEILEC.</i>
 *
 * \Return      None.
 *
 * \Notes       The selected clock rate should be at least twice the expected maximum quadrature count rate.
 *
 * \Example     Setup32BitQEI2IO(DIV_FLTR_PRESCALE_64,
 *				INDEX_MATCH_QEB_1_QEA_1,
 *				QEA_QEB_NOT_SWAPPED,
 *				QEA_POL_NON_INVERTED,
 *				QEB_POL_NON_INVERTED,
 *				INDX_POL_NON_INVERTED,
 *				HOM_POL_NON_INVERTED,
 *				DISABLE_CTNCMP_OUTPUT);
 *
 **********************************************************************************************************************/
#define Setup32BitQEI2IO( InputFilterDiv, IndexMatchValue, SwapEN, QEAPol, QEBPol,IndexPol, HomePol, OutputMode)({	\
															\
	QEI2IOCbits.QFDIV	= (InputFilterDiv)&0b0111;								\
	QEI2IOCbits.FLTREN	= (~(InputFilterDiv>>3))&0b00000001;							\
	QEI2CONbits.IMV		= IndexMatchValue;									\
	QEI2IOCbits.SWPAB	= SwapEN;										\
	QEI2IOCbits.QEAPOL	= QEAPol;										\
	QEI2IOCbits.QEBPOL	= QEBPol;										\
	QEI2IOCbits.IDXPOL	= IndexPol;										\
	QEI2IOCbits.HOMPOL	= HomePol;										\
	QEI2IOCbits.OUTFNC	= OutputMode;										\
})

/***********************************************************************************************************************
 * \Function		void Setup32BitQEI2Interrupt( IntPriority, CountOverFlowIntEN, IndxIntEN, HomeIntEN, VelocityIntEN,
 *						      LesserOrEqualIntEN, GreaterOrEqualIntEN, InitializDoneIntEN)
 * \Description		Configures QEI2 Interrupts.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>IntPriority:</u>\n
 *	- A value Between 0 to 7 inclusively.
 *
 * <u>CountOverFlowIntEN:</u>\n
 *	- DIS_INT_POS_OVERFLOW		<i>/ Position counter overflow interrupt disabled.</i>\n
 *	- EN_INT_POS_OVERFLOW		<i>/ Position counter overflow interrupt enabled.</i>\n
 *
 * <u>IndxIntEN:</u>\n
 *	- DIS_INT_INDEX			<i>/ Index input event interrupt disabled.</i>\n
 *	- EN_INT_INDEX			<i>/ Index input event interrupt enabled.</i>\n
 *
 * <u>HomeIntEN:</u>\n
 *	- DIS_INT_HOME			<i>/ Home input event interrupt disabled.</i>\n
 *	- EN_INT_HOME			<i>/ Home input event interrupt enabled.</i>\n
 *
 * <u>VelocityIntEN:</u>\n
 *	- DIS_INT_VELO_OVERFLOW		<i>/ Velocity counter overflow interrupt disabled.</i>\n
 *	- EN_INT_VELO_OVERFLOW		<i>/ Velocity counter overflow interrupt enabled.</i>\n
 *
 * <u>LesserOrEqualIntEN:</u>\n
 *	- DIS_INT_POS_LESS_EQU		<i>/ Position counter less than or equal QEILEC interrupt disabled.</i>\n
 *	- EN_INT_POS_LESS_EQU		<i>/ Position counter less than or equal QEILEC interrupt enabled.</i>\n
 *
 * <u>GreaterOrEqualIntEN:</u>\n
 *	- DIS_INT_POS_GREAT_EQU		<i>/ Position counter greater than or equal QEIGEC interrupt disabled.</i>\n
 *	- EN_INT_POS_GREAT_EQU		<i>/ Position counter greater than or equal QEIGEC interrupt enabled.</i>\n
 *
 * <u>InitializDoneIntEN:</u>\n
 *	- DIS_INT_POS_INIT		<i>/ Position counter init complete interrupt disabled.</i>\n
 *	- EN_INT_POS_INIT		<i>/ Position counter init complete interrupt enabled.</i>\n
 *
 * \Return      None.
 *
 * \Notes       The selected clock rate should be at least twice the expected maximum quadrature count rate.
 *
 * \Example     Setup32BitQEI2Interrupt(4,
 *					EN_INT_POS_OVERFLOW,
 *					DIS_INT_INDEX,
 *					DIS_INT_HOME,
 *					DIS_INT_VELO_OVERFLOW,
 *					DIS_INT_POS_LESS_EQU,
 *					DIS_INT_POS_GREAT_EQU,
 *					DIS_INT_POS_INIT);
 *
 **********************************************************************************************************************/
#define Setup32BitQEI2Interrupt(IntPriority, CountOverFlowIntEN, IndxIntEN, HomeIntEN, VelocityIntEN, 	\
				LesserOrEqualIntEN, GreaterOrEqualIntEN, InitializDoneIntEN)({		\
													\
	_QEI2IP			= IntPriority;								\
	QEI2STATbits.POSOVIEN	= CountOverFlowIntEN;							\
	QEI2STATbits.IDXIEN	= IndxIntEN;								\
	QEI2STATbits.HOMIEN	= HomeIntEN;								\
	QEI2STATbits.VELOVIEN	= VelocityIntEN;							\
	QEI2STATbits.PCLEQIEN	= LesserOrEqualIntEN;							\
	QEI2STATbits.PCHEQIEN	= GreaterOrEqualIntEN;							\
	QEI2STATbits.PCIIEN	= InitializDoneIntEN;							\
})

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#define EnableIntQEI2()			_QEI2IE = 1
#define DisableIntQEI2()		_QEI2IE = 0
#define SetPriorityIntQEI2(priority)	_QEI2IP = (priority)

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/** QEI2 Status Bits */

/**Status Flag for Index Event Status bit*/
#define QEI2_INDEX_OCCURRED			(QEI2STATbits.IDXIRQ)
/**Status Flag for Home Event Status bit*/
#define QEI2_HOME_OCCURRED			(QEI2STATbits.HOMIRQ)
/**Velocity Counter Overflow Status bit*/
#define QEI2_VELO_OVERFLOW_OCCURRED		(QEI2STATbits.VELOVIRQ)
/**Position Counter (Homing) Initialization Process Complete Status bit*/
#define QEI2_POS_INIT_COMPLETE			(QEI2STATbits.PCIIRQ)
/**Position Counter Overflow Status bit*/
#define QEI2_POS_OVERFLOW_OCCURRED		(QEI2STATbits.POSOVIRQ)
/**Position Counter Less Than or Equal Compare Status bit*/
#define QEI2_POS_LESS_EQU_OCCURRED		(QEI2STATbits.PCLEQIRQ)
/**Position Counter Greater Than or Equal Compare Status bit*/
#define QEI2_POS_GREAT_EQU_OCCURRED		(QEI2STATbits.PCHEQIRQ)
/***/
#endif

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// <editor-fold defaultstate="collapsed" desc="QEI Library Arguments">


#ifndef _COMMON_DIRECTIVES_
#define _COMMON_DIRECTIVES_

#define DONT_CHANGE	0xFFFF
#define DONT_CARE	0
#define KEEP_DEFAULT	0
#define SET_DEFAULT	0

#endif

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Setup32BitQEIx */

/** CounterMode */
#define QUADRATURE_COUNTER_MODE		0b000
#define UPDOWN_COUNTER_MODE		0b001
#define NORMAL_COUNTER_MODE		0b010
#define GATED_COUNTER_MODE		0b110
#define NORMAL_TIMER_MODE		0b011
#define GATED_TIMER_MODE		0b111

/** CounterInitMode */
#define POS_COUNT_NO_INIT			0	/* Index does not affect position counter */
#define	POS_COUNT_INIT_EVERY_INDEX		1	/* Every index input resets position counter */
#define	POS_COUNT_INIT_NEXT_INDEX		2	/* Next index input moves content of QEIxIC into Position counter */
#define	POS_COUNT_INIT_FIRST_INDEX		3	/* First index event after home moves content of QEIxIC into Position counter */
#define	POS_COUNT_INIT_SECOND_INDEX		4	/* Second index event after home moves content of QEIxIC into Position counter */
#define	POS_COUNT_INIT_EQUALS_QEIGEC		5	/* Resets position counter when position counter equals QEIxGEC */
#define	POS_COUNT_INIT_MODULO_COUNT		6	/* Modulo count mode for position counter */

/** InputPrescaler */
#define	QEI_INPUT_PRESCALE_1			0	/* Timer input clock prescalar selction */
#define	QEI_INPUT_PRESCALE_2			1
#define	QEI_INPUT_PRESCALE_4			2
#define	QEI_INPUT_PRESCALE_8			3
#define	QEI_INPUT_PRESCALE_16			4
#define	QEI_INPUT_PRESCALE_32			5
#define	QEI_INPUT_PRESCALE_64			6
#define	QEI_INPUT_PRESCALE_256			7

/** CountPolarity */
#define	COUNT_POSITIVE				0	/* Counter direction is positive */
#define	COUNT_NEGATIVE				1	/* Counter direction is negative */

/** InputCapEN */
#define DIS_INPUT_CAPTURE			0
#define EN_INPUT_CAPTURE			1

/** IdelEN */
#define	CONTINUE_QEI_IN_IDEL			0	/* Continue QEI in Idle mode */
#define	STOP_QEI_IN_IDEL			1	/* Stop QEI in Idle mode */

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Setup32BitQEIxIO */

/** InputFilterDiv */
#define	DIV_FLTR_PRESCALE_1		0	/* Digital filter clock prescalar select */
#define	DIV_FLTR_PRESCALE_2		1
#define	DIV_FLTR_PRESCALE_4		2
#define	DIV_FLTR_PRESCALE_8		3
#define	DIV_FLTR_PRESCALE_16		4
#define	DIV_FLTR_PRESCALE_32		5
#define	DIV_FLTR_PRESCALE_64		6
#define	DIV_FLTR_PRESCALE_256		7
#define	DIV_FLTR_PRESCALE_DIS		8

/** IndexMatchValue */
#define	INDEX_MATCH_NO_EFFECT		0	/* Index does not affect position counter */
#define	INDEX_MATCH_QEB_0_QEA_1		1	/* Index match when QEB = 0 and QEA = 1 */
#define	INDEX_MATCH_QEB_1_QEA_0		2	/* Index match when QEB = 1 and QEA = 0 */
#define	INDEX_MATCH_QEB_1_QEA_1		3	/* Index match when QEB = 1 and QEA = 1 */

/** SwapEN */
#define	QEA_QEB_NOT_SWAPPED		0	/* QEA QEB inputs are not swapped */
#define	QEA_QEB_SWAPPED			1	/* QEA QEB inputs are swapped */

/** QEAPol */
#define	QEA_POL_NON_INVERTED		0	/* QEA input polarity is non-inverted */
#define	QEA_POL_INVERTED		1	/* QEA input polarity is inverted */

/** QEBPol */
#define	QEB_POL_NON_INVERTED		0	/* QEB input polarity is non-inverted */
#define	QEB_POL_INVERTED		1	/* QEB input polarity is inverted */

/** IndexPol */
#define	INDX_POL_NON_INVERTED		0	/* Index input polarity is non-inverted */
#define	INDX_POL_INVERTED		1	/* Index input polarity is inverted */

/** HomePol */
#define	HOM_POL_NON_INVERTED		0	/* Home input polarity is non-inverted */
#define	HOM_POL_INVERTED		1	/* Home input polarity is inverted */

/** OutputMode */
#define	DISABLE_CTNCMP_OUTPUT		0	/* Output is disabled */
#define	OUTPUT_ON_GREATER_QEIGEC	1	/* CTNCMP pin goes high whn POSCNT >= QEIGEC */
#define	OUTPUT_ON_LESSER_QEILEC		2	/* CTNCMP pin goes high whn POSCNT <= QEILEC */
#define	OUTPUT_ON_GREATER_LESSER	3	/* CTNCMP pin goes high whn POSCNT >= QEIGEC or POSCNT <= QEILEC */

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Setup32BitQEIxInterrupt */

/** CountOverFlowIntEN */
#define	DIS_INT_POS_OVERFLOW	0	/* Position counter overflow interrupt disabled */
#define	EN_INT_POS_OVERFLOW	1	/* Position counter overflow interrupt enabled */

/** IndxIntEN */
#define	DIS_INT_INDEX		0	/* Index input event interrupt disabled */
#define	EN_INT_INDEX		1	/* Index input event interrupt enabled */

/** HomeIntEN */
#define	DIS_INT_HOME		0	/* Home input event interrupt disabled */
#define	EN_INT_HOME		1	/* Home input event interrupt enabled */

/** VelocityIntEN */
#define	DIS_INT_VELO_OVERFLOW	0	/* Velocity counter overflow interrupt disabled */
#define	EN_INT_VELO_OVERFLOW	1	/* Velocity counter overflow interrupt enabled */

/** LesserOrEqualIntEN */
#define	DIS_INT_POS_LESS_EQU	0	/* Position counter less than or equal QEILEC interrupt disabled */
#define	EN_INT_POS_LESS_EQU	1	/* Position counter less than or equal QEILEC interrupt enabled */

/** GreaterOrEqualIntEN */
#define	DIS_INT_POS_GREAT_EQU	0	/* Position counter greater than or equal QEIGEC interrupt disabled  */
#define	EN_INT_POS_GREAT_EQU	1	/* Position counter greater than or equal QEIGEC interrupt enabled  */

/** InitializDoneIntEN */
#define	DIS_INT_POS_INIT	0	/* Position counter init complete interrupt disabled */
#define	EN_INT_POS_INIT		1	/* Position counter init complete interrupt enabled */
/***/
// </editor-fold>

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// <editor-fold defaultstate="collapsed" desc="QEI Library Function Prototypes">

#ifdef _QEI1IF

unsigned long Read32bitQEI1PositionCounter(void);	/* reads value of position counter */
unsigned long Read32bitQEI1Capture(void);	
unsigned long Read32bitQEI1IndexCounter(void);		/* reads value of index counter */
unsigned long Read32bitQEI1IntervalTimer(void);		/* reads value of interval timer */
/************************************************************************************
 * Note:
 * The velocity counter specifies the distance traveled between the time interval of
 * each sample. Reading the VELxCNT register results in counter reset. The user
 * application should read the velocity counter at a rate of 1-4 kHz.
 ***********************************************************************************/
unsigned int Read32bitQEI1VelocityCounter(void);	/* reads value of velocity counter */
void Write32bitQEI1GreaterEqual(uQEI_Counter *ptr);	/* writes to greater than or equal */
void Write32bitQEI1IndexCounter(uQEI_Counter *ptr);	/* writes to index counter */
void Write32bitQEI1Initialization(uQEI_Counter *ptr);	/* writes to initialization register */
void Write32bitQEI1IntervalTimer(uQEI_Counter *ptr);	/* writes to interval timer */
void Write32bitQEI1LesserEqual(uQEI_Counter *ptr);	/* writes to lesser than or equal register */
void Write32bitQEI1PositionCounter(uQEI_Counter *ptr);/* writes to position counter */
void Close32bitQEI1(void);							/* disable interrupt and stop QEI module */
#endif

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#ifdef _QEI2IF

unsigned long Read32bitQEI2PositionCounter(void);	/* reads value of position counter */
unsigned long Read32bitQEI2Capture(void);			/* reads value of capture register */
unsigned long Read32bitQEI2IndexCounter(void);		/* reads value of index counter */
unsigned long Read32bitQEI2IntervalTimer(void);		/* reads value of interval timer */
/************************************************************************************
 * Note:
 * The velocity counter specifies the distance traveled between the time interval of
 * each sample. Reading the VELxCNT register results in counter reset. The user
 * application should read the velocity counter at a rate of 1-4 kHz.
 ***********************************************************************************/
unsigned int Read32bitQEI2VelocityCounter(void);	/* reads value of velocity counter */
void Write32bitQEI2GreaterEqual(uQEI_Counter *ptr);	/* writes to greater than or equal */
void Write32bitQEI2IndexCounter(uQEI_Counter *ptr);	/* writes to index counter */
void Write32bitQEI2Initialization(uQEI_Counter *ptr);	/* writes to initialization register */
void Write32bitQEI2IntervalTimer(uQEI_Counter *ptr);	/* writes to interval timer */
void Write32bitQEI2LesserEqual(uQEI_Counter *ptr);	/* writes to lesser than or equal register */
void Write32bitQEI2PositionCounter(uQEI_Counter *ptr);/* writes to position counter */
void Close32bitQEI2(void);							/* disable interrupt and stop QEI module */

#endif

// </editor-fold>


#endif