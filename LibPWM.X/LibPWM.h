
/*
 * PWM Module Libary For dsPIC33E/PIC24E Devices.
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
 * File: $Id: LibPWM.h, V1.00 2015/04/15 AL-Moutaz Billah Tabbakha Exp $
 *
 ***********************************************************************************************************************
 * - Macro Functions Represented in This Library: ("x" Denotes to PWM Generator Number 1 to 7.)
 *
 * unsigned int SetupPWM_PrimaryMasterTB(ClockPrescaler,PriPeriod,SEVPostscaler,SEVCompVal,ENimmediatPeriodUpdate)
 *	EnableIntPWMSEV()
 *	DiableIntPWMSEV()
 *	SetIntPriorityPWMSEV(Priority)
 *
 * unsigned int SetupPWM_SecondaryMasterTB(ClockPrescaler,SecPeriod,SSEVPostscaler,SSEVCompVal,ENimmediatPeriodUpdate)
 *	EnableIntPWMSSEV()
 *	DiableIntPWMSSEV()
 *	SetIntPriorityPWMSSEV(Priority)
 *
 * void SetupPWMChopGenerator(ChopClockDivider, ChopClockGeneratorEN)
 *
 * unsigned int SetupPWMGenx(PWM_Mode,Aligment,TimeBaseSelect,Period_H,Period_L,DCMode,ENimmediatDCupdate)
 *
 * void SetupPWMGenxPins(H_PinOwnership,L_PinOwnership,H_PinPolarity,L_PinPolarity,EN_PinSwap)
 *
 * void SetupPWMGenxOverride(H_PinOverEN,L_PinOverEN,OverDataForH_Pin,OverDataForL_Pin,SyncOverride)
 *
 * unsigned int SetupPWMGenxPhaseShifting(PhShift_H, PhShift_L)
 *
 * unsigned int SetupPWMGenxDeadTime(DeadTimeMode,DeadTime,AlternatDeadTime)
 *
 * unsigned int SetupPWMGenxADCTrigger(TriggerCompareVal,TrigStartWaitCycles,TrigOutputDiv)
 *	EnableIntPWMxTrig()
 *	DisableIntPWMxTrig()
 *
 * unsigned int SetupPWMGenxFault(IndependentFaultEN,FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *	EnableIntPWMxFault()
 *	DisableIntPWMxFault()
 *
 * unsigned int SetupPWMGenxCurrentLimit(CurrentLimSource,CurrentLimData_H,CurrentLimData_L,CurrentLimInputActiveState)
 *	EnablePWMxCurrentLim()
 *	DisablePWMxCurrentLim()
 *	EnableIntPWMxCLim()
 *	DisableIntPWMxCLim()
 *
 * void SetupPWMGenxLEB(PWMH_RisingEdgeBlankingEN,PWMH_FallingEdgeBlankingEN,PWML_RisingEdgeBlankingEN,PWML_FallingEdgeBlankingEN,LEB_Delay)
 *	EnablePWMxFaultBlanking()
 *	EnablePWMxCurrentLimBlanking()
 *	DisablePWMxFaultBlanking()
 *	DisablePWMxCurrentLimBlanking()
 *
 * void SetupPWMGenxStateBlanking(PWMH_BlankingState,PWML_BlankingState,SelectedSignalForStateBlank,SelectedSignalBlankingState)
 *	EnablePWMxFaultBlanking()
 *	EnablePWMxCurrentLimBlanking()
 *	DisablePWMxFaultBlanking()
 *	DisablePWMxCurrentLimBlanking()
 *
 * void SetupPWMGenxChop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)
 *
 * void CaptuerPWMxPriTB_Val()
 *
 * void SetDutyCyclePWM1(DC)
 * void SetDutyCyclePWM1H(DC)
 * void SetDutyCyclePWM1L(DC)
 * void SetMasterDutyCycle(DC)
 *
 * void EnableIntPWMSEV()
 * void DisbleIntPWMSEV()
 * void SetIntPriorityPWMSEV(Priority)
 *
 * void EnableIntPWMSSEV()
 * void DisbleIntPWMSSEV()
 * void SetIntPriorityPWMSSEV(Priority)
 *
 * void EnableIntPWMx()
 * void DisableIntPWMx()
 * void SetIntPriorityPWMx(Priority)
 *
 *
 ***********************************************************************************************************************
 * - PWMx Status Bits.
 *
 * PWM1_TRIG_INTERRUPT_IS_PENDING
 * PWM1_FAULT_INTERRUPT_IS_PENDING
 * PWM1_CUR_LIM_INTERRUPT_IS_PENDING
 * PWM1_INT_FLAG
 */

/****************************************************************************************************
 * Important Notes:
 * 1-	Certain devices incorporate a write protection feature for the IOCONx and FCLCONx registers,
 *	which prevents any inadvertent writes to these registers. This feature can be controlled by the
 *	PWMLOCK Configuration bit (FOSCSEL<6>). The default state of the write protection feature is
 *	enabled (PWMLOCK = 1), So You Have To Disable This feature By Force The PWMLOCK ConfigBit to Zero
 *	Befor You Can Use This Library.
 *
 * 2-	Fault Exit Sequence:
 *
 *	If Cycle-by-Cycle Fault mode is selected, the fault is automatically reset on every PWM cycle. No
 *	additional coding is needed to exit the Fault condition.
 *	For the Latched Fault mode, however, the following sequence must be followed to exit the Fault condition:
 *	1. Poll the PWM fault source to determine, if the fault signal has been deasserted.
 *	2. If the PWM fault interrupt is not enabled, skip the following sub-steps and proceed to step (3),
 *	   If the PWM fault interrupt is enabled, perform the following sub-steps, and then proceed to step (4).
 *		a) Complete the PWM fault Interrupt Service Routine.
 *		b) Disable the PWM fault interrupt by clearing the FLTIEN bit (PWMCONx <12>).
 *		c) Enable the PWM fault interrupt by setting FLTMOD<1:0> (FCLCONx <1:0>) = 0b00.
 *	3. Disable PWM faults by setting FLTMOD<1:0> (FCLCONx <1:0>) = 0b11.
 *	4. Enable the latched PWM Fault mode by setting FLTMOD<1:0> (FCLCONx <1:0>) = 0b00.
*****************************************************************************************************/

#include <stdint.h>					// Includes uint16_t definition.

/**********************************************************************************************************************/
/********************************** PWM Module Library Macro Functions & Defines **************************************/
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWM_PrimaryMasterTB(ClockPrescaler,PriPeriod,SEVPostscaler,SEVCompVal,ENimmediatPeriodUpdate)
 *
 * \Description		Configures PWM Primary Master Time Base Operation Parameters.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>ClockPrescaler:</u>\n
 *	- PWM_PRESCALER_DIV_1	\n
 *	- PWM_PRESCALER_DIV_2	\n
 *	- PWM_PRESCALER_DIV_4	\n
 *	- PWM_PRESCALER_DIV_8	\n
 *	- PWM_PRESCALER_DIV_16	\n
 *	- PWM_PRESCALER_DIV_32	\n
 *	- PWM_PRESCALER_DIV_64	\n
 * 	- DONT_CHANGE
 *
 * <u>PriPeriod:</u>	\n
 *	- a Value Between 1 and 65535 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).
 *
 * <u>SEVPostscaler:</u> \n
 *	- SEV_POSTSCALER_DIV_1	\n
 *	- SEV_POSTSCALER_DIV_2	\n
 *	- SEV_POSTSCALER_DIV_3	\n
 *	- SEV_POSTSCALER_DIV_4	\n
 *	- SEV_POSTSCALER_DIV_5	\n
 *	- SEV_POSTSCALER_DIV_6	\n
 *	- SEV_POSTSCALER_DIV_7	\n
 *	- SEV_POSTSCALER_DIV_8	\n
 *	- SEV_POSTSCALER_DIV_9	\n
 *	- SEV_POSTSCALER_DIV_10	\n
 *	- SEV_POSTSCALER_DIV_11	\n
 *	- SEV_POSTSCALER_DIV_12	\n
 *	- SEV_POSTSCALER_DIV_13	\n
 *	- SEV_POSTSCALER_DIV_14	\n
 *	- SEV_POSTSCALER_DIV_15	\n
 * 	- DONT_CHANGE
 *
 * <u>SEVCompVal:</u> \n
 *	- a Value Leeser Or Equal to PriPeriod.
 *
 * <u>ENimmediatPeriodUpdate:</u> \n
 *	- DISABLE_IMMEDIAT_UPDATE	\n
 *	- ENABLE_IMMEDIAT_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The Primary Master Time Has Been Configered Without Any Errors.\n
 *		<b> 1: </b> Err1: An Attempt to Change The PMTMR Clock Prescaler while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change The Primary Special Event Postscaler while The PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change The Primary Immediat Period Update Bit While The PWM Module is Enabled.\n
 *		<b> 4: </b> Err4: The Sepecial Event Postscaler Is Out Of Range (1 to 16).
 *
 * \RelatedMacros	EnableIntPWMSEV()		\n
 *			DiableIntPWMSEV()		\n
 *			SetIntPriorityPWMSEV(Priority)
 *
 * \Notes       ClockPrescaler & SEVPostscaler & ENimmediatPeriodUpdate Cant be Changed When The PWM Module is Enabled.
 *
 * \Example     Err=SetupPWM_PrimaryMasterTB(PWM_PRESCALER_DIV_1,30000,SEV_POSTSCALER_DIV_1,21000,DISABLE_IMMEDIAT_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWM_PrimaryMasterTB(_ClockPrescaler, PriPeriod, _SEVPostscaler, SEVCompVal, _ENimmediatPeriodUpdate)({			\
																	\
	uint16_t _Return=0;														\
	uint16_t ClockPrescaler=_ClockPrescaler;											\
	uint16_t SEVPostscaler=_SEVPostscaler;												\
	uint16_t ENimmediatPeriodUpdate=_ENimmediatPeriodUpdate;									\
	if(ClockPrescaler == DONT_CHANGE) {ClockPrescaler = PTCON2bits.PCLKDIV;}							\
	if(SEVPostscaler == DONT_CHANGE) {SEVPostscaler = PTCONbits.SEVTPS+1;}								\
	if(ENimmediatPeriodUpdate == DONT_CHANGE) {ENimmediatPeriodUpdate = PTCONbits.EIPU;}						\
																	\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/			\
																	\
		PTCON2bits.PCLKDIV = ClockPrescaler;											\
		SEVPostscaler = SEVPostscaler - 1;											\
		if (SEVPostscaler<16) {PTCONbits.SEVTPS = SEVPostscaler;}								\
		else _Return=4;		/*Set Error "Sepecial Event Postscaler must be only from 1 to 16"*/				\
																	\
		PTCONbits.EIPU = ENimmediatPeriodUpdate;										\
	} else {															\
		if (ClockPrescaler != PTCON2bits.PCLKDIV)										\
			_Return=1;	/*Set Error "PMTMR Clock Prescaler Cant be Changed When The PWM Module is Enabled"*/		\
		if (SEVPostscaler != PTCONbits.SEVTPS)											\
			_Return=2;	/*Set Error "Primary Special Event Postscaler Cant be Changed When The PWM Module is Enabled"*/	\
		if (ENimmediatPeriodUpdate != PTCONbits.EIPU)										\
			_Return=3;	/*Set Error "Primary Immediat Period Update Bit Cant be Changed When The PWM Module is Enabled"*/\
	}																\
	PTPER = PriPeriod ;		/*Note: 1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation.*/ 				\
	SEVTCMP = SEVCompVal;														\
	_Return;															\
})

#ifdef STPER
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWM_SecondaryMasterTB(ClockPrescaler,SecPeriod,SSEVPostscaler,SSEVCompVal,ENimmediatPeriodUpdate)
 *
 * \Description		Configures PWM Secondary Master Time Base Operation Parameters.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>ClockPrescaler:</u>\n
 *	- PWM_PRESCALER_DIV_1	\n
 *	- PWM_PRESCALER_DIV_2	\n
 *	- PWM_PRESCALER_DIV_4	\n
 *	- PWM_PRESCALER_DIV_8	\n
 *	- PWM_PRESCALER_DIV_16	\n
 *	- PWM_PRESCALER_DIV_32	\n
 *	- PWM_PRESCALER_DIV_64	\n
 * 	- DONT_CHANGE
 *
 * <u>SecPeriod:</u>	\n
 *	- a Value Between 1 and 65535 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).
 *
 * <u>SSEVPostscaler:</u> \n
 *	- SEV_POSTSCALER_DIV_1	\n
 *	- SEV_POSTSCALER_DIV_2	\n
 *	- SEV_POSTSCALER_DIV_3	\n
 *	- SEV_POSTSCALER_DIV_4	\n
 *	- SEV_POSTSCALER_DIV_5	\n
 *	- SEV_POSTSCALER_DIV_6	\n
 *	- SEV_POSTSCALER_DIV_7	\n
 *	- SEV_POSTSCALER_DIV_8	\n
 *	- SEV_POSTSCALER_DIV_9	\n
 *	- SEV_POSTSCALER_DIV_10	\n
 *	- SEV_POSTSCALER_DIV_11	\n
 *	- SEV_POSTSCALER_DIV_12	\n
 *	- SEV_POSTSCALER_DIV_13	\n
 *	- SEV_POSTSCALER_DIV_14	\n
 *	- SEV_POSTSCALER_DIV_15	\n
 * 	- DONT_CHANGE
 *
 * <u>SSEVCompVal:</u> \n
 *	- a Value Leeser Or Equal to SecPeriod.
 *
 * <u>ENimmediatPeriodUpdate:</u> \n
 *	- DISABLE_IMMEDIAT_UPDATE	\n
 *	- ENABLE_IMMEDIAT_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The Secondary Master Time Has Been Configered Without Any Errors.\n
 *		<b> 1: </b> Err1: An Attempt to Change The SMTMR Clock Prescaler while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change The Secondary Special Event Postscaler while The PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change The Secondary Immediat Period Update Bit While The PWM Module is Enabled.\n
 *		<b> 4: </b> Err4: The Sepecial Event Postscaler Is Out Of Range (1 to 16).
 *
 * \RelatedMacros	EnableIntPWMSSEV()		\n
 *			DiableIntPWMSSEV()		\n
 *			SetIntPriorityPWMSSEV(Priority)
 *
 * \Notes       ClockPrescaler & SEVPostscaler & ENimmediatPeriodUpdate Cant be Changed When The PWM Module is Enabled.
 *
 * \Example     Err=SetupPWM_SecondaryMasterTB(PWM_PRESCALER_DIV_1,30000,SEV_POSTSCALER_DIV_1,21000,DISABLE_IMMEDIAT_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWM_SecondaryMasterTB(_ClockPrescaler, SecPeriod, _SSEVPostscaler, SSEVCompVal, _ENimmediatPeriodUpdate)({			\
																	\
	uint16_t _Return=0;														\
	uint16_t ClockPrescaler=_ClockPrescaler;											\
	uint16_t SSEVPostscaler=_SSEVPostscaler;											\
	uint16_t ENimmediatPeriodUpdate=_ENimmediatPeriodUpdate;									\
	if(ClockPrescaler == DONT_CHANGE) {ClockPrescaler = STCON2bits.PCLKDIV;}							\
	if(SSEVPostscaler == DONT_CHANGE) {SSEVPostscaler = STCONbits.SEVTPS+1;}							\
	if(ENimmediatPeriodUpdate == DONT_CHANGE) {ENimmediatPeriodUpdate = STCONbits.EIPU;}						\
																	\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/			\
																	\
		ClockPrescaler = STCON2bits.PCLKDIV;											\
		SSEVPostscaler =- 1;													\
		if (SSEVPostscaler<16) {STCONbits.SEVTPS = SSEVPostscaler;}								\
		else _Return=4;		/*Set Error "Secondary Sepecial Event Postscaler must be only from 1 to 16"*/			\
																	\
		STCONbits.EIPU = ENimmediatPeriodUpdate;										\
	} else {															\
		if (ClockPrescaler != STCON2bits.PCLKDIV)										\
			_Return=1;	/*Set Error "SMTMR Clock Prescaler Cant be Changed When The PWM Module is Enabled"*/		\
		if (SSEVPostscaler != STCONbits.SEVTPS)											\
			_Return=2;	/*Set Error "Secondary Special Event Postscaler Cant be Changed When The PWM Module is Enabled"*/\
		if (ENimmediatPeriodUpdate != STCONbits.EIPU)										\
			_Return=3;	/*Set Error "Secondary Immediat Period Update Bit Cant be Changed When The PWM Module is Enabled"*/\
	}																\
	STPER = SecPeriod ;		/*Note: 1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation.*/				\
	SSEVTCMP = SSEVCompVal;														\
	_Return;															\
})
#endif

/***********************************************************************************************************************
 * \Function		void SetupPWMChopGenerator(ChopClockDivider, ChopClockGeneratorEN)
 *
 * \Description		Configures PWM Chop Generator Parameters.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>ChopClockDivider:</u>\n
 *	- a Value Between 1 and 1023 Inclusivly, So Chop Frequency = ( FP / Primary Master TB Prescaler ) / (ChopClockDivider + 1).
 *	- DONT_CHANGE
 *
 * <u>ChopClockGeneratorEN:</u>	\n
 *	- DISABLE_CHOP_CLOCK_GENERATOR	\n
 *	- ENABLE_CHOP_CLOCK_GENERATOR
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes       The chop clock generator operates Only with the Primary PWM Clock Prescaler bits (PCLKDIV<2:0>) in the PTCON2 register.
 *
 * \Example     SetupPWMChopGenerator(10, ENABLE_CHOP_CLOCK_GENERATOR); <i>/ So The chop clock is 6 MHz for devices running at 60 MIPS
 *									     with the PWM clock prescaler configured for fastest clock.</i>
 *
 **********************************************************************************************************************/
#define SetupPWMChopGenerator(ChopClockDivider, ChopClockGeneratorEN)({					\
													\
	/* Note: The chop clock generator operates Only with the Primary PWM Clock Prescaler bits */	\
	/* (PCLKDIV<2:0>) in the PTCON2 register.*/							\
	if(ChopClockDivider	!= DONT_CHANGE) {CHOPbits.CHOPCLK = ChopClockDivider&0b1111111111;}	\
	if(ChopClockGeneratorEN	!= DONT_CHANGE) {CHOPbits.CHPCLKEN = ChopClockGeneratorEN&0b1;}		\
})

/*////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
/*////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

#ifdef _PWM1IF

#ifdef STPER
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen1(PWM_Mode,Aligment,TimeBaseSelect,Period_H,Period_L,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator1.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 *	- TRUE_INDEPENDENT_MODE	<i>/True Independent Output Mode is Not Avaliable On All Devices.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- SECONDARY_TIME_BASE	<i>/Secondary Time Base is Not Avaliable On All Devices.</i> \n
 *	- INDEPENDENT_TIME_BASE	\n
 * 	- DONT_CHANGE
 *
 * <u>Period_H:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>Period_L:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator1 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).\n
 *		<b> 3: </b> Err3: An Attempt to use secondary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM1(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetDutyCyclePWM1H(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetDutyCyclePWM1L(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen1(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1(_PWM_Mode, _Aligment, _TimeBaseSelect, Period_H, Period_L, _DCMode, _ENimmediatDCupdate)({					\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON1bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON1bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON1bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON1bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON1bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON1bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON1bits.PMOD = PWM_Mode;	/* 3 = PWM I/O pin pair is in True Independent PWM Output mode*/				\
						/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON1bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON1bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON1bits.CAM)	PWMCON1bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case SECONDARY_TIME_BASE:	/* 1 = PWM generator uses the secondary master time base FixedSecPeriod*/		\
				if(!PWMCON1bits.CAM)	PWMCON1bits.MTBS = 1;									\
				else			_Return=3; /* Set Error "We cant use secondary time base in center aligne mode"*/	\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx/SPHASEx registers provide time base period for this PWM generator*/	\
				PWMCON1bits.ITB  = 1;												\
				PHASE1 = Period_H;	/* Independent Period For PWM1H*/							\
				SPHASE1 = Period_L;	/* Independent Period For PWM1L*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON1bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
				/*0 = (Independent Duty Cycle) PDCx and SDCx registers provide duty cycle information for this PWM generator*/	\
		PWMCON1bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen1(PWM_Mode,Aligment,TimeBaseSelect,Period,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator1.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- INDEPENDENT_TIME_BASE	<i>/PHASEx register provides time base period for this PWM generator.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Period:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator1 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM1(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen1(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1(_PWM_Mode, _Aligment, _TimeBaseSelect, Period, _DCMode, _ENimmediatDCupdate)({							\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON1bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON1bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON1bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON1bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON1bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON1bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON1bits.PMOD = PWM_Mode;	/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON1bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON1bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON1bits.CAM)	PWMCON1bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx register provide time base period for this PWM generator*/		\
				PWMCON1bits.ITB  = 1;												\
				PHASE1 = Period;	/* Independent Period For PWM1*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON1bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
					/*0 = (Independent Duty Cycle) PDCx register provide duty cycle information for this PWM generator*/	\
		PWMCON1bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#endif

/***********************************************************************************************************************
 * \Function		void SetupPWMGen1Pins(H_PinOwnership,L_PinOwnership,H_PinPolarity,L_PinPolarity,EN_PinSwap)
 *
 * \Description		Configures PWM Generator1 Output Pins.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_H_PIN	\n
 *	- PWM_MODULE_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_L_PIN	\n
 *	- PWM_MODULE_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>H_PinPolarity:</u>	\n
 *	- H_PIN_ACTIVE_HIGH	\n
 *	- H_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>L_PinPolarity:</u>	\n
 *	- L_PIN_ACTIVE_HIGH	\n
 *	- L_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>EN_PinSwap:</u>		\n
 *	- H_L_PINS_NOT_SWAPED	\n
 *	- H_L_PINS_SWAPED	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen1Pins(PWM_MODULE_CONTROL_H_PIN,PWM_MODULE_CONTROL_L_PIN,H_PIN_ACTIVE_HIGH,L_PIN_ACTIVE_HIGH,H_L_PINS_NOT_SWAPED);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1Pins(H_PinOwnership, L_PinOwnership, H_PinPolarity, L_PinPolarity, EN_PinSwap)({	\
	if(H_PinOwnership != DONT_CHANGE) {IOCON1bits.PENH = H_PinOwnership&0b1;}			\
	if(L_PinOwnership != DONT_CHANGE) {IOCON1bits.PENL = L_PinOwnership&0b1;}			\
	if(H_PinPolarity  != DONT_CHANGE) {IOCON1bits.POLH = H_PinPolarity&0b1;}			\
	if(L_PinPolarity  != DONT_CHANGE) {IOCON1bits.POLL = L_PinPolarity&0b1;}			\
	if(EN_PinSwap     != DONT_CHANGE) {IOCON1bits.SWAP = EN_PinSwap&0b1;}				\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen1Override(H_PinOverEN,L_PinOverEN,OverDataForH_Pin,OverDataForL_Pin,SyncOverride)
 *
 * \Description		Configures And Set PWM Generator1 Pins Override.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_H_PIN	\n
 *	- OVR_DAT_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_L_PIN	\n
 *	- OVR_DAT_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>OverDataForH_Pin:</u>	\n
 *	- OVERRIDE_H_PIN_LOW	\n
 *	- OVERRIDE_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>OverDataForL_Pin:</u>	\n
 *	- OVERRIDE_L_PIN_LOW	\n
 *	- OVERRIDE_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>SyncOverride:</u>		\n
 *	- DISABLE_OVERRIDE_SYNC	\n
 *	- ENABLE_OVERRIDE_SYNC	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen1Override(PWM_GEN_CONTROL_H_PIN, PWM_GEN_CONTROL_L_PIN, DONT_CHANGE, DONT_CHANGE, DONT_CHANGE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1Override(H_PinOverEN, L_PinOverEN, OverDataForH_Pin, OverDataForL_Pin, SyncOverride)({	\
	if(H_PinOverEN		!= DONT_CHANGE) {IOCON1bits.OVRENH  = H_PinOverEN&0b1;}				\
	if(L_PinOverEN		!= DONT_CHANGE) {IOCON1bits.OVRENL  = L_PinOverEN&0b1;}				\
	if(OverDataForL_Pin	!= DONT_CHANGE) {IOCON1bits.OVRDAT0 = OverDataForL_Pin&0b1;}			\
	if(OverDataForH_Pin	!= DONT_CHANGE) {IOCON1bits.OVRDAT1 = OverDataForH_Pin&0b1;}			\
	if(SyncOverride		!= DONT_CHANGE) {IOCON1bits.OSYNC   = SyncOverride&0b1;}			\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen1PhaseShifting(PhShift_H, PhShift_L)
 *
 * \Description		Set PWM Generator1 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift_H:</u> <i>/For Complementary And Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * <u>PhShift_L:</u> <i>/ONLY For Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator1 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen1PhaseShifting(6000, 12000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1PhaseShifting(PhShift_H, PhShift_L)({							\
														\
	unsigned int _Return=0;											\
	if(PWMCON1bits.CAM == EDGE_ALIGNED){									\
		if(PhShift_H != DONT_CHANGE) {PHASE1 = PhShift_H;}						\
		if(PhShift_L != DONT_CHANGE) {SPHASE1 = PhShift_L;}						\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen1PhaseShifting(PhShift)
 *
 * \Description		Set PWM Generator1 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift:</u> \n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator1 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen1PhaseShifting(6000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1PhaseShifting(PhShift)({									\
														\
	unsigned int _Return=0;											\
	if(PWMCON1bits.CAM == EDGE_ALIGNED){									\
		if(PhShift != DONT_CHANGE) {PHASE1 = PhShift;}							\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen1DeadTime(DeadTimeMode,DeadTime,AlternatDeadTime)
 *
 * \Description		Configures And Set PWM Generator1 Dead Time.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>DeadTimeMode:</u>	\n
 *	- POSITIVE_DT_MODE			\n
 *	- NEGATIVE_DT_MODE			<i>/Not allowed in Center_Align mode.</i>\n
 *	- DISABLE_DT_MODE			\n
 *	- NEGATIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxH is shortened and PWMxL is lengthened And
 *							If DTCMPx Pin = 1, PWMxL is shortened and PWMxH is lengthened.</i>\n
 *	- POSITIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxL is shortened and PWMxH is lengthened And
 *							If DTCMPx Pin = 1, PWMxH is shortened and PWMxL is lengthened.</i>\n
 *	- DONT_CHANGE
 *
 * <u>DeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Is Not Used For Dead Time Value In CENTER_ALIGNED Output Mode So Use AlternatDeadTime Param For Both PWMxH & PWMxL Outputs.\n
 *	- This Parameter Holds The Value To Be Added to Or Subtracted From The Duty Cycle in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE.
 *
 * <u>AlternatDeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in CENTER_ALIGNED PWM Output Mode.\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE DeadTimeMode.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator1 Dead Time Parameters Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Dead time Compensation in None Complementary PWM output mode with Positive Dead Time.\n
 *		<b> 2: </b> Err2: An Attempt to Use Negative Dead Time in None Complementary Edge Aligned PWM output mode.\n
 *		<b> 3: </b> Warn1: An Attempt to Use Primary Dead Time (DeadTime) in Center Align Mode With Positive dead time, (You May Keep it Zero).
 *
 * \RelatedMacros	None.
 *
 * \Notes	1- Open "Dead Time Configrations.png" Chart in "Important Files" Folder for Comprehensive Understanding of Dead Time Module Operation.\n
 *		2- Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode.			\n
 *		3- Primary Dead Time (DeadTime) is not used in Center Align Mode So Keep it Zero.						\n
 *		4- Negative Dead time is available only for Complementary Edge Aligned PWM output mode.
 *
 * \Example     Err=SetupPWMGen1DeadTime(POSITIVE_DT_MODE,1000,1200);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1DeadTime(_DeadTimeMode, DeadTime, AlternatDeadTime)({										\
																			\
	unsigned int _Return=0;																\
	unsigned int DeadTimeMode=_DeadTimeMode;													\
	if(DeadTimeMode == DONT_CHANGE) {														\
		DeadTimeMode=PWMCON1bits.DTC;														\
		if( (DeadTimeMode==0b11) && PWMCON1bits.DTCP)												\
			DeadTimeMode=POSITIVE_COMPENSATION_DT_MODE;											\
	}																		\
	switch (DeadTimeMode){																\
		case POSITIVE_COMPENSATION_DT_MODE :													\
			if(IOCON1bits.PMOD) { /* check if Complementary PWM output mode is Used But if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON1bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON1bits.DTCP= 1;		/* Dead-Time Compensation Polarity is Positive */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers */\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case NEGATIVE_COMPENSATION_DT_MODE :													\
			if(IOCON1bits.PMOD) { /* check if Complementary PWM output mode is Used And if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON1bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON1bits.DTCP= 0;		/* Dead-Time Compensation Polarity is Negative */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers*/	\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case POSITIVE_DT_MODE :															\
			PWMCON1bits.DTC = 0;		/* Positive dead time actively applied for all output modes */					\
			if((PWMCON1bits.CAM)&&(DeadTime)) {												\
				/*Set Warning "Primary Dead Time (DTRx) is not used in Center Align Mode With Positive dead time So Keep it Zero!"*/	\
				_Return=3;														\
			}																\
			/* DeadTime Holds The Value of Dead Time for PWMxH output signal */								\
			/* AlternatDeadTime Holds The Dead Time Value for PWMxL output signal OR for both the PWMxH and PWMxL in Center Align mode */	\
			break;																\
		case NEGATIVE_DT_MODE :															\
			if ((IOCON1bits.PMOD == COMPLEMENTARY_MODE) && (PWMCON1bits.CAM == EDGE_ALIGNED)) {						\
				PWMCON1bits.DTC = 1;	/* Negative dead time actively applied for Complementary Output mode */				\
				/* DeadTime Holds The Value of Dead Time for PWMxL output signal */							\
				/* AlternatDeadTime Holds The Dead Time Value for PWMxH output signal */						\
			}else{																\
				/*Set Error "Negative Dead time is available only for Complementary Edge Aligned PWM output mode"*/			\
				_Return=2;														\
			}																\
			break;																\
		case DISABLE_DT_MODE :															\
			PWMCON1bits.DTC = 0;														\
	}																		\
	if(DeadTime		!= DONT_CHANGE) {DTR1	= DeadTime&0b11111111111111;}									\
	if(AlternatDeadTime	!= DONT_CHANGE) {ALTDTR1 = AlternatDeadTime&0b11111111111111;}								\
	_Return;																	\
})								

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen1ADCTrigger(TriggerCompareVal,TrigStartWaitCycles,TrigOutputDiv)
 *
 * \Description		Configures PWM Generator1 For ADC Triggering.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>TriggerCompareVal:</u>\n
 *	- a Value Between 1 and 65534 Inclusivly.\n
 * 	- DONT_CHANGE
 *
 * <u>TrigStartWaitCycles:</u>\n
 *	- WAIT_0_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	- WAIT_1_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	-            .				\n
 *	-            .				\n
 *	- WAIT_63_PWM_CYC_BEFORE_FIRST_TRIG	\n
 * 	- DONT_CHANGE
 *
 * <u>TrigOutputDiv:</u>\n
 *	- TRIGGER_ON_EVERY_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_2ND_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_3RD_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_4TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_5TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_6TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_7TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_8TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_9TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_10TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_11TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_12TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_13TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_14TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_15TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_16TH_TRIGGER_EVENT	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator1 ADC Triggering Function Configered Correctly.\n
 *		<b> 1: </b> Err1: TRIGx Value Is Bigger Than PTPER, (When Using Primary Time Base).\n
 *		<b> 2: </b> Err2: TRIGx Value Is Bigger Than STPER, (When Using Secondary Time Base).\n
 *		<b> 3: </b> Err3: TRIGx Value Is Bigger Than PHASEx, (When Using Independent Time Base Mode).
 *
 * \RelatedMacros	EnableIntPWM1Trig()		\n
 *			DisableIntPWM1Trig()		\n
 *			PWM1_TRIG_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- TriggerCompareVal Must Be Smaller Than PHASEx In Independent Time Base Mode.	\n
 *		2- TriggerCompareVal Must Be Smaller Than STPER When Using Secondary Time Base.	\n
 *		3- TriggerCompareVal Must Be Smaller Than PTPER When Using Primary Time Base.
 *
 * \Example     Err=SetupPWMGen1ADCTrigger(2400,WAIT_16_PWM_CYC_BEFORE_FIRST_TRIG,TRIGGER_ON_EVERY_TRIGGER_EVENT);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1ADCTrigger(TriggerCompareVal, TrigStartWaitCycles, TrigOutputDiv)({								\
																		\
	unsigned int _Return=0;															\
	if(TriggerCompareVal != DONT_CHANGE) {													\
		switch (PWMCON1bits.ITB){													\
			case 1:															\
				if(TriggerCompareVal<PHASE1) {TRIG1 = TriggerCompareVal;}							\
				else {_Return=3;/*Set Error "TRIGx Value Must Be Smaller than PHASEx In Independent Time Base Mode"*/}		\
				break;														\
			case 0:															\
				if(PWMCON1bits.MTBS){												\
					if(TriggerCompareVal<STPER) {TRIG1 = TriggerCompareVal;}						\
					else {_Return=2;/*Set Error "TRIGx Value Must Be Smaller Than STPER When Using Secondary Time Base"*/}	\
				}else{														\
					if(TriggerCompareVal<PTPER) {TRIG1 = TriggerCompareVal;}						\
					else {_Return=1;/*Set Error "TRIGx Value Must Be Smaller Than PTPER When Using Primary Time Base"*/}	\
				}														\
		}																\
	}																	\
	if(TrigStartWaitCycles	!= DONT_CHANGE) {TRGCON1bits.TRGSTRT = TrigStartWaitCycles&0b111111;}						\
	if(TrigOutputDiv	!= DONT_CHANGE) {TRGCON1bits.TRGDIV  = TrigOutputDiv&0b1111;}							\
	_Return;																\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen1Fault(IndependentFaultEN,FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator1 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>IndependentFaultEN:</u>\n
 *	- NORMAL_FAULT_MODE		<i>/ Fault inputs map FLTDAT<1:0> to PWMxH and PWMxL and 
 *									 Current-limit map CLDAT<1:0> to PWMxH and PWMxL.</i>\n
 *	- INDEPENDENT_FAULT_MODE<i>/ Current-limit inputs map FLTDAT<1> to PWMxH output and 
 *								   Fault input maps FLTDAT<0> to PWMxL output.
 *								   The CLDAT<1:0> bits are not used for override functions.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>FaultSource:</u> <i>/Refer to Device Datasheet For Avaliable Fault Sources.</i>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_FAULT8_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator1 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM1Fault()		\n
 *			DisableIntPWM1Fault()		\n
 *			PWM1_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen1Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(IndependentFaultEN	!= DONT_CHANGE) {FCLCON1bits.IFLTMOD = IndependentFaultEN&0b1;}				\
	if(FaultSource		!= DONT_CHANGE) {FCLCON1bits.FLTSRC = FaultSource&0b11111;}				\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM1 = FaultMode&0b1;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON1bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON1bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON1bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON1bits.FLTDAT1 = FaultData_H;									\
		IOCON1bits.FLTDAT0 = FaultData_L;									\
		FCLCON1bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON1bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON1bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON1bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#else 
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen1Fault(FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator1 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 *
 * <u>FaultSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator1 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM1Fault()		\n
 *			DisableIntPWM1Fault()		\n
 *			PWM1_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen1Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(FaultSource		!= DONT_CHANGE) {FCLCON1bits.FLTSRC = FaultSource&0b1;}					\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM1 = FaultMode&0b11111;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON1bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON1bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON1bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON1bits.FLTDAT1 = FaultData_H;									\
		IOCON1bits.FLTDAT0 = FaultData_L;									\
		FCLCON1bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON1bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON1bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON1bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen1CurrentLimit(CurrentLimSource,CurrentLimData_H,CurrentLimData_L,CurrentLimInputActiveState)
 *
 * \Description		Configures PWM Generator1 Current Limit Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>CurrentLimSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimInputActiveState:</u>\n
 *	- CURRENT_LIM_INPUT_ACTIVE_HIGH	\n
 *	- CURRENT_LIM_INPUT_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator1 Current Limit Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Current Limit Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Use Current Limit Data High/Low in Independent Fault Mode.
 *
 * \RelatedMacros	EnablePWM1CurrentLim()		\n
 *			DisablePWM1CurrentLim()		\n
 *			EnableIntPWM1CLim()		\n
 *			DisableIntPWM1CLim()		\n
 *			PWM1_CUR_LIM_INTERRUPT_IS_PENDING <i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- CurrentLimInputActiveState should not be changed after the PWM is enabled (PTEN = 1).\n
 *		2- Current Limit Data High/Low is Not used in Independent Fault Mode.\n
 *		3- Refer to Device Datasheet For Avaliable Current Limit Sources.
 *
 * \Example    Err=SetupPWMGen1CurrentLimit(SOURCE_IS_COMPARATOR2,SET_H_PIN_LOW,SET_L_PIN_LOW,CURRENT_LIM_INPUT_ACTIVE_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1CurrentLimit(CurrentLimSource, CurrentLimData_H, CurrentLimData_L, CurrentLimInputActiveState)({			\
																	\
	unsigned int _Return=0;														\
	if(CurrentLimData_H != DONT_CHANGE) {												\
		IOCON1bits.CLDAT1 = CurrentLimData_H&0b1;										\
		if(FCLCON1bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																	\
	if(CurrentLimData_L != DONT_CHANGE) {												\
		IOCON1bits.CLDAT0 = CurrentLimData_L&0b1;										\
		if(FCLCON1bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																 	\
	if(CurrentLimSource != DONT_CHANGE) {FCLCON1bits.CLSRC = CurrentLimSource&0b11111;}						\
	if(CurrentLimInputActiveState != DONT_CHANGE) {											\
		if(PTCONbits.PTEN == 0){												\
			FCLCON1bits.CLPOL = CurrentLimInputActiveState&0b1;								\
		} else {														\
			if (CurrentLimInputActiveState != FCLCON1bits.CLPOL) {								\
				_Return=1; /*Set Error "PWM Current Limit Polarity Cant be Changed When The PWM Module is Enabled"*/	\
			}														\
		}															\
	}																\
	_Return;															\
})

/***********************************************************************************************************************
 * \Function	void SetupPWMGen1LEB(PWMH_RisingEdgeBlankingEN,PWMH_FallingEdgeBlankingEN,PWML_RisingEdgeBlankingEN,PWML_FallingEdgeBlankingEN,LEB_Delay)
 *
 * \Description		Configures PWM Generator1 Leading Edge Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_RISING_EDGE_BLANK \n
 *	- ENABLE_PWMH_RISING_EDGE_BLANK	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- DONT_CHANGE
 *
 * <u>PWML_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_RISING_EDGE_BLANK \n
 *	- ENABLE_PWML_RISING_EDGE_BLANK	\n
 * 	- DONT_CHANGE
 *
 * <u>PWML_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWML_FALLING_EDGE_BLANK \n
 * 	- DONT_CHANGE
 *
 * <u>LEB_Delay:</u>\n
 *	- a Value Between 1 and 4096 Inclusivly. (LEB Delay = (0 to 4096) * PWM Clock Freq) \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM1FaultBlanking()	\n
 *			EnablePWM1CurrentLimBlanking()	\n
 *			DisablePWM1FaultBlanking()	\n
 *			DisablePWM1CurrentLimBlanking()
 *
 * \Notes		LEB Delay = (0 to 4096) * PWM Clock Freq.
 *
 * \Example     
 * SetupPWMGen1LEB(ENABLE_PWMH_RISING_EDGE_BLANK,ENABLE_PWMH_FALLING_EDGE_BLANK,ENABLE_PWML_RISING_EDGE_BLANK,ENABLE_PWML_FALLING_EDGE_BLANK,990);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1LEB(PWMH_RisingEdgeBlankingEN, PWMH_FallingEdgeBlankingEN,  PWML_RisingEdgeBlankingEN, PWML_FallingEdgeBlankingEN, LEB_Delay)({	\
																			\
	if(PWMH_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON1bits.PHR = PWMH_RisingEdgeBlankingEN&0b1;}						\
	if(PWMH_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON1bits.PHF = PWMH_FallingEdgeBlankingEN&0b1;}						\
	if(PWML_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON1bits.PLR = PWML_RisingEdgeBlankingEN&0b1;}						\
	if(PWML_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON1bits.PLF = PWML_FallingEdgeBlankingEN&0b1;}						\
	if(LEB_Delay			!= DONT_CHANGE) {LEBDLY1 = LEB_Delay&0b111111111111;}								\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen1StateBlanking(PWMH_BlankingState,PWML_BlankingState,SelectedSignalForStateBlank,SelectedSignalBlankingState)
 *
 * \Description		Configures PWM Generator1 State Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWMH_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_LOW	\n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>PWML_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWML_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>SelectedSignalForStateBlank:</u>\n
 *	- NO_SIGNAL_SOURCE_FOR_STATE_BLNKING \n
 *	- PWM1H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM2H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM3H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM4H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM5H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM6H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM7H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 * 	- DONT_CHANGE
 *
 * <u>SelectedSignalBlankingState:</u>\n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM1FaultBlanking()	\n
 *			EnablePWM1CurrentLimBlanking()	\n
 *			DisablePWM1FaultBlanking()	\n
 *			DisablePWM1CurrentLimBlanking()
 *
 * \Notes		None.
 *
 * \Example	SetupPWMGen1StateBlanking(ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH,ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH,
 *					  PWM1H_AS_STATE_BLANK_SIGNAL_SOURCE,ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1StateBlanking(PWMH_BlankingState, PWML_BlankingState, SelectedSignalForStateBlank, SelectedSignalBlankingState)({			\
																			\
	if(PWMH_BlankingState		!= DONT_CHANGE) {LEBCON1bits.BPHH = (PWMH_BlankingState&0b11)>>1; LEBCON1bits.BPHL = PWMH_BlankingState&0b1;}	\
	if(PWML_BlankingState		!= DONT_CHANGE) {LEBCON1bits.BPLH = (PWML_BlankingState&0b11)>>1; LEBCON1bits.BPLL = PWML_BlankingState&0b1;}	\
	if(SelectedSignalBlankingState	!= DONT_CHANGE) {LEBCON1bits.BCH  = (SelectedSignalBlankingState&0b11)>>1;					\
							 LEBCON1bits.BCL  =  SelectedSignalBlankingState&0b1;}						\
	if(SelectedSignalForStateBlank	!= DONT_CHANGE) {AUXCON1bits.BLANKSEL = SelectedSignalForStateBlank&0b111;}					\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen1Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)
 *
 * \Description		Configures PWM Generator1 Chop Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>ChopClkSource:</u> <i>/Refer to Device Datasheet For Avaliable Chop Clock Sources.</i>\n
 *	- CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE \n
 *	- PWM1H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM2H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM3H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM4H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM5H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM6H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM7H_AS_CHOP_CLOCK_SOURCE	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_ChopEN:</u>\n
 *	- DISABLE_PWMH_CHOP \n
 *	- ENABLE_PWMH_CHOP \n
 *	- DONT_CHANGE
 *
 * <u>PWML_ChopEN:</u>\n
 *	- DISABLE_PWML_CHOP \n
 *	- ENABLE_PWML_CHOP \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		Refer to Device Datasheet For Avaliable Chop Clock Sources.
 *
 * \Example	SetupPWMGen1Chop(CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE, ENABLE_PWMH_CHOP, ENABLE_PWML_CHOP);
 *
 **********************************************************************************************************************/
#define SetupPWMGen1Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)({				\
												\
	if(ChopClkSource	!= DONT_CHANGE) {AUXCON1bits.CHOPSEL = ChopClkSource&0b111;}	\
	if(PWMH_ChopEN		!= DONT_CHANGE) {AUXCON1bits.CHOPHEN = PWMH_ChopEN&0b1;}	\
	if(PWML_ChopEN		!= DONT_CHANGE) {AUXCON1bits.CHOPLEN = PWML_ChopEN&0b1;}	\
})
/***/

// <editor-fold defaultstate="collapsed" desc="PWM1 Macros and Defines">

#ifdef PWMCAP1
/****************************************************************************************************** 
 * \brief	The value Returned From this Routine represents the captured PWM time base value when
 *		a leading edge is detected on the current-limit input.
 * 
 * \Notes	1: The capture feature is only available on the primary output (PWMxH).
 *		2: The feature is only active after LEB processing on the current-limit input signal is complete.
 * 
 *****************************************************************************************************/
#define CaptuerPWM1PriTB_Val()			PWMCAP1
#endif
/***/
#define EnablePWM1Fault()			FCLCON1bits.FLTMOD = FaultModePWM1
#define DisablePWM1Fault()			FCLCON1bits.FLTMOD = 0b11

#define EnablePWM1CurrentLim()			FCLCON1bits.CLMOD = 1
#define DisablePWM1CurrentLim()			FCLCON1bits.CLMOD = 0

#define EnablePWM1FaultBlanking()		LEBCON1bits.FLTLEBEN = 1
#define DisablePWM1FaultBlanking()		LEBCON1bits.FLTLEBEN = 0

#define EnablePWM1CurrentLimBlanking()		LEBCON1bits.CLLEBEN = 1
#define DisablePWM1CurrentLimBlanking()		LEBCON1bits.CLLEBEN = 0

#define SetDutyCyclePWM1(DC)			(PDC1=DC)	/*For Independent PWM DCMode With Complementary PWM Output Mode.*/
#ifdef STPER /* Devices That Have Secondary Time Base Will Have True Independent Duty Cycle Mode Option.*/
#define SetDutyCyclePWM1H(DC)			(PDC1=DC)	/*For Independent PWM DCMode*/
#define SetDutyCyclePWM1L(DC)			(SDC1=DC)	/*For Independent PWM DCMode*/
#endif
#define SetMasterDutyCycle(DC)			(MDC=DC)	/*When Using Master DCMode.*/

/** PWM Geneator1 Interrupt Handling */
#define EnableIntPWM1()				_PWM1IE = 1
#define DisableIntPWM1()			_PWM1IE = 0
#define SetIntPriorityPWM1(Priority)		_PWM1IP = (Priority)
#define PWM1_INT_FLAG				_PWM1IF

#define EnableIntPWM1Trig()			PWMCON1bits.TRGIEN = 1
#define DisableIntPWM1Trig()			PWMCON1bits.TRGIEN = 0
#define PWM1_TRIG_INTERRUPT_IS_PENDING		PWMCON1bits.TRGSTAT

#define	EnableIntPWM1Fault()			PWMCON1bits.FLTIEN = 1
#define DisableIntPWM1Fault()			PWMCON1bits.FLTIEN = 0
#define PWM1_FAULT_INTERRUPT_IS_PENDING		PWMCON1bits.FLTSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

#define	EnableIntPWM1CLim()			PWMCON1bits.CLIEN = 1
#define	DisableIntPWM1CLim()			PWMCON1bits.CLIEN = 0
#define PWM1_CUR_LIM_INTERRUPT_IS_PENDING	PWMCON1bits.CLSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

// </editor-fold>

#endif

#ifdef _PWM2IF

#ifdef STPER
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen2(PWM_Mode,Aligment,TimeBaseSelect,Period_H,Period_L,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator2.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 *	- TRUE_INDEPENDENT_MODE	<i>/True Independent Output Mode is Not Avaliable On All Devices.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- SECONDARY_TIME_BASE	<i>/Secondary Time Base is Not Avaliable On All Devices.</i> \n
 *	- INDEPENDENT_TIME_BASE	\n
 * 	- DONT_CHANGE
 *
 * <u>Period_H:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>Period_L:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator2 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).\n
 *		<b> 3: </b> Err3: An Attempt to use secondary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM2(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetDutyCyclePWM2H(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetDutyCyclePWM2L(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen2(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2(_PWM_Mode, _Aligment, _TimeBaseSelect, Period_H, Period_L, _DCMode, _ENimmediatDCupdate)({					\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON2bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON2bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON2bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON2bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON2bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON2bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON2bits.PMOD = PWM_Mode;	/* 3 = PWM I/O pin pair is in True Independent PWM Output mode*/				\
						/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON2bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON2bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON2bits.CAM)	PWMCON2bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case SECONDARY_TIME_BASE:	/* 1 = PWM generator uses the secondary master time base FixedSecPeriod*/		\
				if(!PWMCON2bits.CAM)	PWMCON2bits.MTBS = 1;									\
				else			_Return=3; /* Set Error "We cant use secondary time base in center aligne mode"*/	\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx/SPHASEx registers provide time base period for this PWM generator*/	\
				PWMCON2bits.ITB  = 1;												\
				PHASE2 = Period_H;	/* Independent Period For PWM2H*/							\
				SPHASE2 = Period_L;	/* Independent Period For PWM2L*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON2bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
				/*0 = (Independent Duty Cycle) PDCx and SDCx registers provide duty cycle information for this PWM generator*/	\
		PWMCON2bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen2(PWM_Mode,Aligment,TimeBaseSelect,Period,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator2.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- INDEPENDENT_TIME_BASE	<i>/PHASEx register provides time base period for this PWM generator.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Period:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator2 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM2(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen2(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2(_PWM_Mode, _Aligment, _TimeBaseSelect, Period, _DCMode, _ENimmediatDCupdate)({							\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON2bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON2bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON2bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON2bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON2bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON2bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON2bits.PMOD = PWM_Mode;	/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON2bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON2bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON2bits.CAM)	PWMCON2bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx register provide time base period for this PWM generator*/		\
				PWMCON2bits.ITB  = 1;												\
				PHASE2 = Period;	/* Independent Period For PWM2*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON2bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
					/*0 = (Independent Duty Cycle) PDCx register provide duty cycle information for this PWM generator*/	\
		PWMCON2bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#endif

/***********************************************************************************************************************
 * \Function		void SetupPWMGen2Pins(H_PinOwnership,L_PinOwnership,H_PinPolarity,L_PinPolarity,EN_PinSwap)
 *
 * \Description		Configures PWM Generator2 Output Pins.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_H_PIN	\n
 *	- PWM_MODULE_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_L_PIN	\n
 *	- PWM_MODULE_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>H_PinPolarity:</u>	\n
 *	- H_PIN_ACTIVE_HIGH	\n
 *	- H_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>L_PinPolarity:</u>	\n
 *	- L_PIN_ACTIVE_HIGH	\n
 *	- L_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>EN_PinSwap:</u>		\n
 *	- H_L_PINS_NOT_SWAPED	\n
 *	- H_L_PINS_SWAPED	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen2Pins(PWM_MODULE_CONTROL_H_PIN,PWM_MODULE_CONTROL_L_PIN,H_PIN_ACTIVE_HIGH,L_PIN_ACTIVE_HIGH,H_L_PINS_NOT_SWAPED);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2Pins(H_PinOwnership, L_PinOwnership, H_PinPolarity, L_PinPolarity, EN_PinSwap)({	\
	if(H_PinOwnership != DONT_CHANGE) {IOCON2bits.PENH = H_PinOwnership&0b1;}			\
	if(L_PinOwnership != DONT_CHANGE) {IOCON2bits.PENL = L_PinOwnership&0b1;}			\
	if(H_PinPolarity  != DONT_CHANGE) {IOCON2bits.POLH = H_PinPolarity&0b1;}			\
	if(L_PinPolarity  != DONT_CHANGE) {IOCON2bits.POLL = L_PinPolarity&0b1;}			\
	if(EN_PinSwap     != DONT_CHANGE) {IOCON2bits.SWAP = EN_PinSwap&0b1;}				\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen2Override(H_PinOverEN,L_PinOverEN,OverDataForH_Pin,OverDataForL_Pin,SyncOverride)
 *
 * \Description		Configures And Set PWM Generator2 Pins Override.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_H_PIN	\n
 *	- OVR_DAT_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_L_PIN	\n
 *	- OVR_DAT_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>OverDataForH_Pin:</u>	\n
 *	- OVERRIDE_H_PIN_LOW	\n
 *	- OVERRIDE_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>OverDataForL_Pin:</u>	\n
 *	- OVERRIDE_L_PIN_LOW	\n
 *	- OVERRIDE_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>SyncOverride:</u>		\n
 *	- DISABLE_OVERRIDE_SYNC	\n
 *	- ENABLE_OVERRIDE_SYNC	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen2Override(PWM_GEN_CONTROL_H_PIN, PWM_GEN_CONTROL_L_PIN, DONT_CHANGE, DONT_CHANGE, DONT_CHANGE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2Override(H_PinOverEN, L_PinOverEN, OverDataForH_Pin, OverDataForL_Pin, SyncOverride)({	\
	if(H_PinOverEN		!= DONT_CHANGE) {IOCON2bits.OVRENH  = H_PinOverEN&0b1;}				\
	if(L_PinOverEN		!= DONT_CHANGE) {IOCON2bits.OVRENL  = L_PinOverEN&0b1;}				\
	if(OverDataForL_Pin	!= DONT_CHANGE) {IOCON2bits.OVRDAT0 = OverDataForL_Pin&0b1;}			\
	if(OverDataForH_Pin	!= DONT_CHANGE) {IOCON2bits.OVRDAT1 = OverDataForH_Pin&0b1;}			\
	if(SyncOverride		!= DONT_CHANGE) {IOCON2bits.OSYNC   = SyncOverride&0b1;}			\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen2PhaseShifting(PhShift_H, PhShift_L)
 *
 * \Description		Set PWM Generator2 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift_H:</u> <i>/For Complementary And Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * <u>PhShift_L:</u> <i>/ONLY For Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator2 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen2PhaseShifting(6000, 12000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2PhaseShifting(PhShift_H, PhShift_L)({							\
														\
	unsigned int _Return=0;											\
	if(PWMCON2bits.CAM == EDGE_ALIGNED){									\
		if(PhShift_H != DONT_CHANGE) {PHASE2 = PhShift_H;}						\
		if(PhShift_L != DONT_CHANGE) {SPHASE2 = PhShift_L;}						\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen2PhaseShifting(PhShift)
 *
 * \Description		Set PWM Generator2 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift:</u> \n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator2 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen2PhaseShifting(6000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2PhaseShifting(PhShift)({									\
														\
	unsigned int _Return=0;											\
	if(PWMCON2bits.CAM == EDGE_ALIGNED){									\
		if(PhShift != DONT_CHANGE) {PHASE2 = PhShift;}							\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen2DeadTime(DeadTimeMode,DeadTime,AlternatDeadTime)
 *
 * \Description		Configures And Set PWM Generator2 Dead Time.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>DeadTimeMode:</u>	\n
 *	- POSITIVE_DT_MODE			\n
 *	- NEGATIVE_DT_MODE			<i>/Not allowed in Center_Align mode.</i>\n
 *	- DISABLE_DT_MODE			\n
 *	- NEGATIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxH is shortened and PWMxL is lengthened And
 *							If DTCMPx Pin = 1, PWMxL is shortened and PWMxH is lengthened.</i>\n
 *	- POSITIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxL is shortened and PWMxH is lengthened And
 *							If DTCMPx Pin = 1, PWMxH is shortened and PWMxL is lengthened.</i>\n
 *	- DONT_CHANGE
 *
 * <u>DeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Is Not Used For Dead Time Value In CENTER_ALIGNED Output Mode So Use AlternatDeadTime Param For Both PWMxH & PWMxL Outputs.\n
 *	- This Parameter Holds The Value To Be Added to Or Subtracted From The Duty Cycle in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE.
 *
 * <u>AlternatDeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in CENTER_ALIGNED PWM Output Mode.\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE DeadTimeMode.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator2 Dead Time Parameters Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Dead time Compensation in None Complementary PWM output mode with Positive Dead Time.\n
 *		<b> 2: </b> Err2: An Attempt to Use Negative Dead Time in None Complementary Edge Aligned PWM output mode.\n
 *		<b> 3: </b> Warn1: An Attempt to Use Primary Dead Time (DeadTime) in Center Align Mode With Positive dead time, (You May Keep it Zero).
 *
 * \RelatedMacros	None.
 *
 * \Notes	1- Open "Dead Time Configrations.png" Chart in "Important Files" Folder for Comprehensive Understanding of Dead Time Module Operation.\n
 *		2- Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode.			\n
 *		3- Primary Dead Time (DeadTime) is not used in Center Align Mode So Keep it Zero.						\n
 *		4- Negative Dead time is available only for Complementary Edge Aligned PWM output mode.
 *
 * \Example     Err=SetupPWMGen2DeadTime(POSITIVE_DT_MODE,1000,1200);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2DeadTime(_DeadTimeMode, DeadTime, AlternatDeadTime)({										\
																			\
	unsigned int _Return=0;																\
	unsigned int DeadTimeMode=_DeadTimeMode;													\
	if(DeadTimeMode == DONT_CHANGE) {														\
		DeadTimeMode=PWMCON2bits.DTC;														\
		if( (DeadTimeMode==0b11) && PWMCON2bits.DTCP)												\
			DeadTimeMode=POSITIVE_COMPENSATION_DT_MODE;											\
	}																		\
	switch (DeadTimeMode){																\
		case POSITIVE_COMPENSATION_DT_MODE :													\
			if(IOCON2bits.PMOD) { /* check if Complementary PWM output mode is Used But if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON2bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON2bits.DTCP= 1;		/* Dead-Time Compensation Polarity is Positive */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers */\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case NEGATIVE_COMPENSATION_DT_MODE :													\
			if(IOCON2bits.PMOD) { /* check if Complementary PWM output mode is Used And if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON2bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON2bits.DTCP= 0;		/* Dead-Time Compensation Polarity is Negative */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers*/	\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case POSITIVE_DT_MODE :															\
			PWMCON2bits.DTC = 0;		/* Positive dead time actively applied for all output modes */					\
			if((PWMCON2bits.CAM)&&(DeadTime)) {												\
				/*Set Warning "Primary Dead Time (DTRx) is not used in Center Align Mode With Positive dead time So Keep it Zero!"*/	\
				_Return=3;														\
			}																\
			/* DeadTime Holds The Value of Dead Time for PWMxH output signal */								\
			/* AlternatDeadTime Holds The Dead Time Value for PWMxL output signal OR for both the PWMxH and PWMxL in Center Align mode */	\
			break;																\
		case NEGATIVE_DT_MODE :															\
			if ((IOCON2bits.PMOD == COMPLEMENTARY_MODE) && (PWMCON2bits.CAM == EDGE_ALIGNED)) {						\
				PWMCON2bits.DTC = 1;	/* Negative dead time actively applied for Complementary Output mode */				\
				/* DeadTime Holds The Value of Dead Time for PWMxL output signal */							\
				/* AlternatDeadTime Holds The Dead Time Value for PWMxH output signal */						\
			}else{																\
				/*Set Error "Negative Dead time is available only for Complementary Edge Aligned PWM output mode"*/			\
				_Return=2;														\
			}																\
			break;																\
		case DISABLE_DT_MODE :															\
			PWMCON2bits.DTC = 0;														\
	}																		\
	if(DeadTime		!= DONT_CHANGE) {DTR2	= DeadTime&0b11111111111111;}									\
	if(AlternatDeadTime	!= DONT_CHANGE) {ALTDTR2 = AlternatDeadTime&0b11111111111111;}								\
	_Return;																	\
})

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen2ADCTrigger(TriggerCompareVal,TrigStartWaitCycles,TrigOutputDiv)
 *
 * \Description		Configures PWM Generator2 For ADC Triggering.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>TriggerCompareVal:</u>\n
 *	- a Value Between 1 and 65534 Inclusivly.\n
 * 	- DONT_CHANGE
 *
 * <u>TrigStartWaitCycles:</u>\n
 *	- WAIT_0_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	- WAIT_1_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	-            .				\n
 *	-            .				\n
 *	- WAIT_63_PWM_CYC_BEFORE_FIRST_TRIG	\n
 * 	- DONT_CHANGE
 *
 * <u>TrigOutputDiv:</u>\n
 *	- TRIGGER_ON_EVERY_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_2ND_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_3RD_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_4TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_5TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_6TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_7TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_8TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_9TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_10TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_11TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_12TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_13TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_14TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_15TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_16TH_TRIGGER_EVENT	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator2 ADC Triggering Function Configered Correctly.\n
 *		<b> 1: </b> Err1: TRIGx Value Is Bigger Than PTPER, (When Using Primary Time Base).\n
 *		<b> 2: </b> Err2: TRIGx Value Is Bigger Than STPER, (When Using Secondary Time Base).\n
 *		<b> 3: </b> Err3: TRIGx Value Is Bigger Than PHASEx, (When Using Independent Time Base Mode).
 *
 * \RelatedMacros	EnableIntPWM2Trig()		\n
 *			DisableIntPWM2Trig()		\n
 *			PWM2_TRIG_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- TriggerCompareVal Must Be Smaller Than PHASEx In Independent Time Base Mode.	\n
 *		2- TriggerCompareVal Must Be Smaller Than STPER When Using Secondary Time Base.	\n
 *		3- TriggerCompareVal Must Be Smaller Than PTPER When Using Primary Time Base.
 *
 * \Example     Err=SetupPWMGen2ADCTrigger(2400,WAIT_16_PWM_CYC_BEFORE_FIRST_TRIG,TRIGGER_ON_EVERY_TRIGGER_EVENT);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2ADCTrigger(TriggerCompareVal, TrigStartWaitCycles, TrigOutputDiv)({								\
																		\
	unsigned int _Return=0;															\
	if(TriggerCompareVal != DONT_CHANGE) {													\
		switch (PWMCON2bits.ITB){													\
			case 1:															\
				if(TriggerCompareVal<PHASE2) {TRIG2 = TriggerCompareVal;}							\
				else {_Return=3;/*Set Error "TRIGx Value Must Be Smaller than PHASEx In Independent Time Base Mode"*/}		\
				break;														\
			case 0:															\
				if(PWMCON2bits.MTBS){												\
					if(TriggerCompareVal<STPER) {TRIG2 = TriggerCompareVal;}						\
					else {_Return=2;/*Set Error "TRIGx Value Must Be Smaller Than STPER When Using Secondary Time Base"*/}	\
				}else{														\
					if(TriggerCompareVal<PTPER) {TRIG2 = TriggerCompareVal;}						\
					else {_Return=1;/*Set Error "TRIGx Value Must Be Smaller Than PTPER When Using Primary Time Base"*/}	\
				}														\
		}																\
	}																	\
	if(TrigStartWaitCycles	!= DONT_CHANGE) {TRGCON2bits.TRGSTRT = TrigStartWaitCycles&0b111111;}						\
	if(TrigOutputDiv	!= DONT_CHANGE) {TRGCON2bits.TRGDIV  = TrigOutputDiv&0b1111;}							\
	_Return;																\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen2Fault(IndependentFaultEN,FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator2 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>IndependentFaultEN:</u>\n
 *	- NORMAL_FAULT_MODE <i>/ Fault inputs map FLTDAT<1:0> to PWMxH and PWMxL and 
 *									 Current-limit map CLDAT<1:0> to PWMxH and PWMxL.</i>\n
 *	- INDEPENDENT_FAULT_MODE<i>/ Current-limit inputs map FLTDAT<1> to PWMxH output and 
 *								   Fault input maps FLTDAT<0> to PWMxL output.
 *								   The CLDAT<1:0> bits are not used for override functions.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>FaultSource:</u> <i>/Refer to Device Datasheet For Avaliable Fault Sources.</i>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_FAULT8_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator2 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM2Fault()		\n
 *			DisableIntPWM2Fault()		\n
 *			PWM2_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen2Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(IndependentFaultEN	!= DONT_CHANGE) {FCLCON2bits.IFLTMOD = IndependentFaultEN&0b1;}				\
	if(FaultSource		!= DONT_CHANGE) {FCLCON2bits.FLTSRC = FaultSource&0b11111;}				\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM2 = FaultMode&0b1;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON2bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON2bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON2bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON2bits.FLTDAT1 = FaultData_H;									\
		IOCON2bits.FLTDAT0 = FaultData_L;									\
		FCLCON2bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON2bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON2bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON2bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen2Fault(FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator2 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 *
 * <u>FaultSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator2 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM2Fault()		\n
 *			DisableIntPWM2Fault()		\n
 *			PWM2_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen2Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(FaultSource		!= DONT_CHANGE) {FCLCON2bits.FLTSRC = FaultSource&0b1;}					\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM2 = FaultMode&0b11111;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON2bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON2bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON2bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON2bits.FLTDAT1 = FaultData_H;									\
		IOCON2bits.FLTDAT0 = FaultData_L;									\
		FCLCON2bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON2bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON2bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON2bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen2CurrentLimit(CurrentLimSource,CurrentLimData_H,CurrentLimData_L,CurrentLimInputActiveState)
 *
 * \Description		Configures PWM Generator2 Current Limit Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>CurrentLimSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimInputActiveState:</u>\n
 *	- CURRENT_LIM_INPUT_ACTIVE_HIGH	\n
 *	- CURRENT_LIM_INPUT_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator2 Current Limit Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Current Limit Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Use Current Limit Data High/Low in Independent Fault Mode.
 *
 * \RelatedMacros	EnablePWM2CurrentLim()		\n
 *			DisablePWM2CurrentLim()		\n
 *			EnableIntPWM2CLim()		\n
 *			DisableIntPWM2CLim()		\n
 *			PWM2_CUR_LIM_INTERRUPT_IS_PENDING <i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- CurrentLimInputActiveState should not be changed after the PWM is enabled (PTEN = 1).\n
 *		2- Current Limit Data High/Low is Not used in Independent Fault Mode.\n
 *		3- Refer to Device Datasheet For Avaliable Current Limit Sources.
 *
 * \Example    Err=SetupPWMGen2CurrentLimit(SOURCE_IS_COMPARATOR2,SET_H_PIN_LOW,SET_L_PIN_LOW,CURRENT_LIM_INPUT_ACTIVE_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2CurrentLimit(CurrentLimSource, CurrentLimData_H, CurrentLimData_L, CurrentLimInputActiveState)({			\
																	\
	unsigned int _Return=0;														\
	if(CurrentLimData_H != DONT_CHANGE) {												\
		IOCON2bits.CLDAT1 = CurrentLimData_H&0b1;										\
		if(FCLCON2bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																	\
	if(CurrentLimData_L != DONT_CHANGE) {												\
		IOCON2bits.CLDAT0 = CurrentLimData_L&0b1;										\
		if(FCLCON2bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																 	\
	if(CurrentLimSource != DONT_CHANGE) {FCLCON2bits.CLSRC = CurrentLimSource&0b11111;}						\
	if(CurrentLimInputActiveState != DONT_CHANGE) {											\
		if(PTCONbits.PTEN == 0){												\
			FCLCON2bits.CLPOL = CurrentLimInputActiveState&0b1;								\
		} else {														\
			if (CurrentLimInputActiveState != FCLCON2bits.CLPOL) {								\
				_Return=1; /*Set Error "PWM Current Limit Polarity Cant be Changed When The PWM Module is Enabled"*/	\
			}														\
		}															\
	}																\
	_Return;															\
})

/***********************************************************************************************************************
 * \Function	void SetupPWMGen2LEB(PWMH_RisingEdgeBlankingEN,PWMH_FallingEdgeBlankingEN,PWML_RisingEdgeBlankingEN,PWML_FallingEdgeBlankingEN,LEB_Delay)
 *
 * \Description		Configures PWM Generator2 Leading Edge Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_RISING_EDGE_BLANK \n
 *	- ENABLE_PWMH_RISING_EDGE_BLANK	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- DONT_CHANGE
 *
 * <u>PWML_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_RISING_EDGE_BLANK \n
 *	- ENABLE_PWML_RISING_EDGE_BLANK	\n
 * 	- DONT_CHANGE
 *
 * <u>PWML_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWML_FALLING_EDGE_BLANK \n
 * 	- DONT_CHANGE
 *
 * <u>LEB_Delay:</u>\n
 *	- a Value Between 1 and 4096 Inclusivly. (LEB Delay = (0 to 4096) * PWM Clock Freq) \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM2FaultBlanking()	\n
 *			EnablePWM2CurrentLimBlanking()	\n
 *			DisablePWM2FaultBlanking()	\n
 *			DisablePWM2CurrentLimBlanking()
 *
 * \Notes		LEB Delay = (0 to 4096) * PWM Clock Freq.
 *
 * \Example
 * SetupPWMGen2LEB(ENABLE_PWMH_RISING_EDGE_BLANK,ENABLE_PWMH_FALLING_EDGE_BLANK,ENABLE_PWML_RISING_EDGE_BLANK,ENABLE_PWML_FALLING_EDGE_BLANK,990);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2LEB(PWMH_RisingEdgeBlankingEN, PWMH_FallingEdgeBlankingEN,  PWML_RisingEdgeBlankingEN, PWML_FallingEdgeBlankingEN, LEB_Delay)({	\
																			\
	if(PWMH_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON2bits.PHR = PWMH_RisingEdgeBlankingEN&0b1;}						\
	if(PWMH_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON2bits.PHF = PWMH_FallingEdgeBlankingEN&0b1;}						\
	if(PWML_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON2bits.PLR = PWML_RisingEdgeBlankingEN&0b1;}						\
	if(PWML_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON2bits.PLF = PWML_FallingEdgeBlankingEN&0b1;}						\
	if(LEB_Delay			!= DONT_CHANGE) {LEBDLY2 = LEB_Delay&0b111111111111;}								\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen2StateBlanking(PWMH_BlankingState,PWML_BlankingState,SelectedSignalForStateBlank,SelectedSignalBlankingState)
 *
 * \Description		Configures PWM Generator2 State Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWMH_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_LOW	\n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>PWML_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWML_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>SelectedSignalForStateBlank:</u>\n
 *	- NO_SIGNAL_SOURCE_FOR_STATE_BLNKING \n
 *	- PWM2H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM2H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM3H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM4H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM5H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM6H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM7H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 * 	- DONT_CHANGE
 *
 * <u>SelectedSignalBlankingState:</u>\n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM2FaultBlanking()	\n
 *			EnablePWM2CurrentLimBlanking()	\n
 *			DisablePWM2FaultBlanking()	\n
 *			DisablePWM2CurrentLimBlanking()
 *
 * \Notes		None.
 *
 * \Example	SetupPWMGen2StateBlanking(ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH,ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH,
 *					  PWM2H_AS_STATE_BLANK_SIGNAL_SOURCE,ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2StateBlanking(PWMH_BlankingState, PWML_BlankingState, SelectedSignalForStateBlank, SelectedSignalBlankingState)({			\
																			\
	if(PWMH_BlankingState		!= DONT_CHANGE) {LEBCON2bits.BPHH = (PWMH_BlankingState&0b11)>>1; LEBCON2bits.BPHL = PWMH_BlankingState&0b1;}	\
	if(PWML_BlankingState		!= DONT_CHANGE) {LEBCON2bits.BPLH = (PWML_BlankingState&0b11)>>1; LEBCON2bits.BPLL = PWML_BlankingState&0b1;}	\
	if(SelectedSignalBlankingState	!= DONT_CHANGE) {LEBCON2bits.BCH  = (SelectedSignalBlankingState&0b11)>>1;					\
							 LEBCON2bits.BCL  =  SelectedSignalBlankingState&0b1;}						\
	if(SelectedSignalForStateBlank	!= DONT_CHANGE) {AUXCON2bits.BLANKSEL = SelectedSignalForStateBlank&0b111;}					\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen2Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)
 *
 * \Description		Configures PWM Generator2 Chop Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>ChopClkSource:</u> <i>/Refer to Device Datasheet For Avaliable Chop Clock Sources.</i>\n
 *	- CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE \n
 *	- PWM2H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM2H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM3H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM4H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM5H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM6H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM7H_AS_CHOP_CLOCK_SOURCE	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_ChopEN:</u>\n
 *	- DISABLE_PWMH_CHOP \n
 *	- ENABLE_PWMH_CHOP \n
 *	- DONT_CHANGE
 *
 * <u>PWML_ChopEN:</u>\n
 *	- DISABLE_PWML_CHOP \n
 *	- ENABLE_PWML_CHOP \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		Refer to Device Datasheet For Avaliable Chop Clock Sources.
 *
 * \Example	SetupPWMGen2Chop(CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE, ENABLE_PWMH_CHOP, ENABLE_PWML_CHOP);
 *
 **********************************************************************************************************************/
#define SetupPWMGen2Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)({				\
												\
	if(ChopClkSource	!= DONT_CHANGE) {AUXCON2bits.CHOPSEL = ChopClkSource&0b111;}	\
	if(PWMH_ChopEN		!= DONT_CHANGE) {AUXCON2bits.CHOPHEN = PWMH_ChopEN&0b1;}	\
	if(PWML_ChopEN		!= DONT_CHANGE) {AUXCON2bits.CHOPLEN = PWML_ChopEN&0b1;}	\
})
/***/

// <editor-fold defaultstate="collapsed" desc="PWM2 Macros and Defines">

#ifdef PWMCAP2
/******************************************************************************************************
 * \brief	The value Returned From this Routine represents the captured PWM time base value when
 *		a leading edge is detected on the current-limit input.
 *
 * \Notes	1: The capture feature is only available on the primary output (PWMxH).
 *		2: The feature is only active after LEB processing on the current-limit input signal is complete.
 *
 *****************************************************************************************************/
#define CaptuerPWM2PriTB_Val()			PWMCAP2
#endif
/***/
#define EnablePWM2Fault()			FCLCON2bits.FLTMOD = FaultModePWM2
#define DisablePWM2Fault()			FCLCON2bits.FLTMOD = 0b11

#define EnablePWM2CurrentLim()			FCLCON2bits.CLMOD = 1
#define DisablePWM2CurrentLim()			FCLCON2bits.CLMOD = 0

#define EnablePWM2FaultBlanking()		LEBCON2bits.FLTLEBEN = 1
#define DisablePWM2FaultBlanking()		LEBCON2bits.FLTLEBEN = 0

#define EnablePWM2CurrentLimBlanking()		LEBCON2bits.CLLEBEN = 1
#define DisablePWM2CurrentLimBlanking()		LEBCON2bits.CLLEBEN = 0

#define SetDutyCyclePWM2(DC)			(PDC2=DC)	/*For Independent PWM DCMode With Complementary PWM Output Mode.*/
#ifdef STPER /* Devices That Have Secondary Time Base Will Have True Independent Duty Cycle Mode Option.*/
#define SetDutyCyclePWM2H(DC)			(PDC2=DC)	/*For Independent PWM DCMode*/
#define SetDutyCyclePWM2L(DC)			(SDC2=DC)	/*For Independent PWM DCMode*/
#endif
#define SetMasterDutyCycle(DC)			(MDC=DC)	/*When Using Master DCMode.*/

/** PWM Geneator1 Interrupt Handling */
#define EnableIntPWM2()				_PWM2IE = 1
#define DisableIntPWM2()			_PWM2IE = 0
#define SetIntPriorityPWM2(Priority)		_PWM2IP = (Priority)
#define PWM2_INT_FLAG				_PWM2IF

#define EnableIntPWM2Trig()			PWMCON2bits.TRGIEN = 1
#define DisableIntPWM2Trig()			PWMCON2bits.TRGIEN = 0
#define PWM2_TRIG_INTERRUPT_IS_PENDING		PWMCON2bits.TRGSTAT

#define	EnableIntPWM2Fault()			PWMCON2bits.FLTIEN = 1
#define DisableIntPWM2Fault()			PWMCON2bits.FLTIEN = 0
#define PWM2_FAULT_INTERRUPT_IS_PENDING		PWMCON2bits.FLTSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

#define	EnableIntPWM2CLim()			PWMCON2bits.CLIEN = 1
#define	DisableIntPWM2CLim()			PWMCON2bits.CLIEN = 0
#define PWM2_CUR_LIM_INTERRUPT_IS_PENDING	PWMCON2bits.CLSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

// </editor-fold>

#endif

#ifdef _PWM3IF

#ifdef STPER
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen3(PWM_Mode,Aligment,TimeBaseSelect,Period_H,Period_L,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator3.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 *	- TRUE_INDEPENDENT_MODE	<i>/True Independent Output Mode is Not Avaliable On All Devices.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- SECONDARY_TIME_BASE	<i>/Secondary Time Base is Not Avaliable On All Devices.</i> \n
 *	- INDEPENDENT_TIME_BASE	\n
 * 	- DONT_CHANGE
 *
 * <u>Period_H:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>Period_L:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator3 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).\n
 *		<b> 3: </b> Err3: An Attempt to use secondary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM3(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetDutyCyclePWM3H(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetDutyCyclePWM3L(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen3(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3(_PWM_Mode, _Aligment, _TimeBaseSelect, Period_H, Period_L, _DCMode, _ENimmediatDCupdate)({					\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON3bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON3bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON3bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON3bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON3bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON3bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON3bits.PMOD = PWM_Mode;	/* 3 = PWM I/O pin pair is in True Independent PWM Output mode*/				\
						/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON3bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON3bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON3bits.CAM)	PWMCON3bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case SECONDARY_TIME_BASE:	/* 1 = PWM generator uses the secondary master time base FixedSecPeriod*/		\
				if(!PWMCON3bits.CAM)	PWMCON3bits.MTBS = 1;									\
				else			_Return=3; /* Set Error "We cant use secondary time base in center aligne mode"*/	\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx/SPHASEx registers provide time base period for this PWM generator*/	\
				PWMCON3bits.ITB  = 1;												\
				PHASE3 = Period_H;	/* Independent Period For PWM3H*/							\
				SPHASE3 = Period_L;	/* Independent Period For PWM3L*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON3bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
				/*0 = (Independent Duty Cycle) PDCx and SDCx registers provide duty cycle information for this PWM generator*/	\
		PWMCON3bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen3(PWM_Mode,Aligment,TimeBaseSelect,Period,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator3.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- INDEPENDENT_TIME_BASE	<i>/PHASEx register provides time base period for this PWM generator.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Period:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator3 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM3(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen3(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3(_PWM_Mode, _Aligment, _TimeBaseSelect, Period, _DCMode, _ENimmediatDCupdate)({							\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON3bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON3bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON3bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON3bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON3bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON3bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON3bits.PMOD = PWM_Mode;	/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON3bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON3bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON3bits.CAM)	PWMCON3bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx register provide time base period for this PWM generator*/		\
				PWMCON3bits.ITB  = 1;												\
				PHASE3 = Period;	/* Independent Period For PWM3*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON3bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
					/*0 = (Independent Duty Cycle) PDCx register provide duty cycle information for this PWM generator*/	\
		PWMCON3bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#endif

/***********************************************************************************************************************
 * \Function		void SetupPWMGen3Pins(H_PinOwnership,L_PinOwnership,H_PinPolarity,L_PinPolarity,EN_PinSwap)
 *
 * \Description		Configures PWM Generator3 Output Pins.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_H_PIN	\n
 *	- PWM_MODULE_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_L_PIN	\n
 *	- PWM_MODULE_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>H_PinPolarity:</u>	\n
 *	- H_PIN_ACTIVE_HIGH	\n
 *	- H_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>L_PinPolarity:</u>	\n
 *	- L_PIN_ACTIVE_HIGH	\n
 *	- L_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>EN_PinSwap:</u>		\n
 *	- H_L_PINS_NOT_SWAPED	\n
 *	- H_L_PINS_SWAPED	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen3Pins(PWM_MODULE_CONTROL_H_PIN,PWM_MODULE_CONTROL_L_PIN,H_PIN_ACTIVE_HIGH,L_PIN_ACTIVE_HIGH,H_L_PINS_NOT_SWAPED);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3Pins(H_PinOwnership, L_PinOwnership, H_PinPolarity, L_PinPolarity, EN_PinSwap)({	\
	if(H_PinOwnership != DONT_CHANGE) {IOCON3bits.PENH = H_PinOwnership&0b1;}			\
	if(L_PinOwnership != DONT_CHANGE) {IOCON3bits.PENL = L_PinOwnership&0b1;}			\
	if(H_PinPolarity  != DONT_CHANGE) {IOCON3bits.POLH = H_PinPolarity&0b1;}			\
	if(L_PinPolarity  != DONT_CHANGE) {IOCON3bits.POLL = L_PinPolarity&0b1;}			\
	if(EN_PinSwap     != DONT_CHANGE) {IOCON3bits.SWAP = EN_PinSwap&0b1;}				\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen3Override(H_PinOverEN,L_PinOverEN,OverDataForH_Pin,OverDataForL_Pin,SyncOverride)
 *
 * \Description		Configures And Set PWM Generator3 Pins Override.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_H_PIN	\n
 *	- OVR_DAT_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_L_PIN	\n
 *	- OVR_DAT_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>OverDataForH_Pin:</u>	\n
 *	- OVERRIDE_H_PIN_LOW	\n
 *	- OVERRIDE_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>OverDataForL_Pin:</u>	\n
 *	- OVERRIDE_L_PIN_LOW	\n
 *	- OVERRIDE_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>SyncOverride:</u>		\n
 *	- DISABLE_OVERRIDE_SYNC	\n
 *	- ENABLE_OVERRIDE_SYNC	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen3Override(PWM_GEN_CONTROL_H_PIN, PWM_GEN_CONTROL_L_PIN, DONT_CHANGE, DONT_CHANGE, DONT_CHANGE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3Override(H_PinOverEN, L_PinOverEN, OverDataForH_Pin, OverDataForL_Pin, SyncOverride)({	\
	if(H_PinOverEN		!= DONT_CHANGE) {IOCON3bits.OVRENH  = H_PinOverEN&0b1;}				\
	if(L_PinOverEN		!= DONT_CHANGE) {IOCON3bits.OVRENL  = L_PinOverEN&0b1;}				\
	if(OverDataForL_Pin	!= DONT_CHANGE) {IOCON3bits.OVRDAT0 = OverDataForL_Pin&0b1;}			\
	if(OverDataForH_Pin	!= DONT_CHANGE) {IOCON3bits.OVRDAT1 = OverDataForH_Pin&0b1;}			\
	if(SyncOverride		!= DONT_CHANGE) {IOCON3bits.OSYNC   = SyncOverride&0b1;}			\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen3PhaseShifting(PhShift_H, PhShift_L)
 *
 * \Description		Set PWM Generator3 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift_H:</u> <i>/For Complementary And Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * <u>PhShift_L:</u> <i>/ONLY For Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator3 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen3PhaseShifting(6000, 12000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3PhaseShifting(PhShift_H, PhShift_L)({							\
														\
	unsigned int _Return=0;											\
	if(PWMCON3bits.CAM == EDGE_ALIGNED){									\
		if(PhShift_H != DONT_CHANGE) {PHASE3 = PhShift_H;}						\
		if(PhShift_L != DONT_CHANGE) {SPHASE3 = PhShift_L;}						\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen3PhaseShifting(PhShift)
 *
 * \Description		Set PWM Generator3 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift:</u> \n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator3 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen3PhaseShifting(6000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3PhaseShifting(PhShift)({									\
														\
	unsigned int _Return=0;											\
	if(PWMCON3bits.CAM == EDGE_ALIGNED){									\
		if(PhShift != DONT_CHANGE) {PHASE3 = PhShift;}							\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen3DeadTime(DeadTimeMode,DeadTime,AlternatDeadTime)
 *
 * \Description		Configures And Set PWM Generator3 Dead Time.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>DeadTimeMode:</u>	\n
 *	- POSITIVE_DT_MODE			\n
 *	- NEGATIVE_DT_MODE			<i>/Not allowed in Center_Align mode.</i>\n
 *	- DISABLE_DT_MODE			\n
 *	- NEGATIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxH is shortened and PWMxL is lengthened And
 *							If DTCMPx Pin = 1, PWMxL is shortened and PWMxH is lengthened.</i>\n
 *	- POSITIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxL is shortened and PWMxH is lengthened And
 *							If DTCMPx Pin = 1, PWMxH is shortened and PWMxL is lengthened.</i>\n
 *	- DONT_CHANGE
 *
 * <u>DeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Is Not Used For Dead Time Value In CENTER_ALIGNED Output Mode So Use AlternatDeadTime Param For Both PWMxH & PWMxL Outputs.\n
 *	- This Parameter Holds The Value To Be Added to Or Subtracted From The Duty Cycle in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE.
 *
 * <u>AlternatDeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in CENTER_ALIGNED PWM Output Mode.\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE DeadTimeMode.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator3 Dead Time Parameters Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Dead time Compensation in None Complementary PWM output mode with Positive Dead Time.\n
 *		<b> 2: </b> Err2: An Attempt to Use Negative Dead Time in None Complementary Edge Aligned PWM output mode.\n
 *		<b> 3: </b> Warn1: An Attempt to Use Primary Dead Time (DeadTime) in Center Align Mode With Positive dead time, (You May Keep it Zero).
 *
 * \RelatedMacros	None.
 *
 * \Notes	1- Open "Dead Time Configrations.png" Chart in "Important Files" Folder for Comprehensive Understanding of Dead Time Module Operation.\n
 *		2- Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode.			\n
 *		3- Primary Dead Time (DeadTime) is not used in Center Align Mode So Keep it Zero.						\n
 *		4- Negative Dead time is available only for Complementary Edge Aligned PWM output mode.
 *
 * \Example     Err=SetupPWMGen3DeadTime(POSITIVE_DT_MODE,1000,1200);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3DeadTime(_DeadTimeMode, DeadTime, AlternatDeadTime)({										\
																			\
	unsigned int _Return=0;																\
	unsigned int DeadTimeMode=_DeadTimeMode;													\
	if(DeadTimeMode == DONT_CHANGE) {														\
		DeadTimeMode=PWMCON3bits.DTC;														\
		if( (DeadTimeMode==0b11) && PWMCON3bits.DTCP)												\
			DeadTimeMode=POSITIVE_COMPENSATION_DT_MODE;											\
	}																		\
	switch (DeadTimeMode){																\
		case POSITIVE_COMPENSATION_DT_MODE :													\
			if(IOCON3bits.PMOD) { /* check if Complementary PWM output mode is Used But if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON3bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON3bits.DTCP= 1;		/* Dead-Time Compensation Polarity is Positive */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers */\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case NEGATIVE_COMPENSATION_DT_MODE :													\
			if(IOCON3bits.PMOD) { /* check if Complementary PWM output mode is Used And if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON3bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON3bits.DTCP= 0;		/* Dead-Time Compensation Polarity is Negative */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers*/	\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case POSITIVE_DT_MODE :															\
			PWMCON3bits.DTC = 0;		/* Positive dead time actively applied for all output modes */					\
			if((PWMCON3bits.CAM)&&(DeadTime)) {												\
				/*Set Warning "Primary Dead Time (DTRx) is not used in Center Align Mode With Positive dead time So Keep it Zero!"*/	\
				_Return=3;														\
			}																\
			/* DeadTime Holds The Value of Dead Time for PWMxH output signal */								\
			/* AlternatDeadTime Holds The Dead Time Value for PWMxL output signal OR for both the PWMxH and PWMxL in Center Align mode */	\
			break;																\
		case NEGATIVE_DT_MODE :															\
			if ((IOCON3bits.PMOD == COMPLEMENTARY_MODE) && (PWMCON3bits.CAM == EDGE_ALIGNED)) {						\
				PWMCON3bits.DTC = 1;	/* Negative dead time actively applied for Complementary Output mode */				\
				/* DeadTime Holds The Value of Dead Time for PWMxL output signal */							\
				/* AlternatDeadTime Holds The Dead Time Value for PWMxH output signal */						\
			}else{																\
				/*Set Error "Negative Dead time is available only for Complementary Edge Aligned PWM output mode"*/			\
				_Return=2;														\
			}																\
			break;																\
		case DISABLE_DT_MODE :															\
			PWMCON3bits.DTC = 0;														\
	}																		\
	if(DeadTime		!= DONT_CHANGE) {DTR3	= DeadTime&0b11111111111111;}									\
	if(AlternatDeadTime	!= DONT_CHANGE) {ALTDTR3 = AlternatDeadTime&0b11111111111111;}								\
	_Return;																	\
})

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen3ADCTrigger(TriggerCompareVal,TrigStartWaitCycles,TrigOutputDiv)
 *
 * \Description		Configures PWM Generator3 For ADC Triggering.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>TriggerCompareVal:</u>\n
 *	- a Value Between 1 and 65534 Inclusivly.\n
 * 	- DONT_CHANGE
 *
 * <u>TrigStartWaitCycles:</u>\n
 *	- WAIT_0_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	- WAIT_1_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	-            .				\n
 *	-            .				\n
 *	- WAIT_63_PWM_CYC_BEFORE_FIRST_TRIG	\n
 * 	- DONT_CHANGE
 *
 * <u>TrigOutputDiv:</u>\n
 *	- TRIGGER_ON_EVERY_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_2ND_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_3RD_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_4TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_5TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_6TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_7TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_8TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_9TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_10TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_11TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_12TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_13TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_14TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_15TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_16TH_TRIGGER_EVENT	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator3 ADC Triggering Function Configered Correctly.\n
 *		<b> 1: </b> Err1: TRIGx Value Is Bigger Than PTPER, (When Using Primary Time Base).\n
 *		<b> 2: </b> Err2: TRIGx Value Is Bigger Than STPER, (When Using Secondary Time Base).\n
 *		<b> 3: </b> Err3: TRIGx Value Is Bigger Than PHASEx, (When Using Independent Time Base Mode).
 *
 * \RelatedMacros	EnableIntPWM3Trig()		\n
 *			DisableIntPWM3Trig()		\n
 *			PWM3_TRIG_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- TriggerCompareVal Must Be Smaller Than PHASEx In Independent Time Base Mode.	\n
 *		2- TriggerCompareVal Must Be Smaller Than STPER When Using Secondary Time Base.	\n
 *		3- TriggerCompareVal Must Be Smaller Than PTPER When Using Primary Time Base.
 *
 * \Example     Err=SetupPWMGen3ADCTrigger(2400,WAIT_16_PWM_CYC_BEFORE_FIRST_TRIG,TRIGGER_ON_EVERY_TRIGGER_EVENT);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3ADCTrigger(TriggerCompareVal, TrigStartWaitCycles, TrigOutputDiv)({								\
																		\
	unsigned int _Return=0;															\
	if(TriggerCompareVal != DONT_CHANGE) {													\
		switch (PWMCON3bits.ITB){													\
			case 1:															\
				if(TriggerCompareVal<PHASE3) {TRIG3 = TriggerCompareVal;}							\
				else {_Return=3;/*Set Error "TRIGx Value Must Be Smaller than PHASEx In Independent Time Base Mode"*/}		\
				break;														\
			case 0:															\
				if(PWMCON3bits.MTBS){												\
					if(TriggerCompareVal<STPER) {TRIG3 = TriggerCompareVal;}						\
					else {_Return=2;/*Set Error "TRIGx Value Must Be Smaller Than STPER When Using Secondary Time Base"*/}	\
				}else{														\
					if(TriggerCompareVal<PTPER) {TRIG3 = TriggerCompareVal;}						\
					else {_Return=1;/*Set Error "TRIGx Value Must Be Smaller Than PTPER When Using Primary Time Base"*/}	\
				}														\
		}																\
	}																	\
	if(TrigStartWaitCycles	!= DONT_CHANGE) {TRGCON3bits.TRGSTRT = TrigStartWaitCycles&0b111111;}						\
	if(TrigOutputDiv	!= DONT_CHANGE) {TRGCON3bits.TRGDIV  = TrigOutputDiv&0b1111;}							\
	_Return;																\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen3Fault(IndependentFaultEN,FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator3 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>IndependentFaultEN:</u>\n
 *	- NORMAL_FAULT_MODE <i>/ Fault inputs map FLTDAT<1:0> to PWMxH and PWMxL and 
 *									 Current-limit map CLDAT<1:0> to PWMxH and PWMxL.</i>\n
 *	- INDEPENDENT_FAULT_MODE<i>/ Current-limit inputs map FLTDAT<1> to PWMxH output and 
 *								   Fault input maps FLTDAT<0> to PWMxL output.
 *								   The CLDAT<1:0> bits are not used for override functions.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>FaultSource:</u> <i>/Refer to Device Datasheet For Avaliable Fault Sources.</i>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_FAULT8_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator3 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM3Fault()		\n
 *			DisableIntPWM3Fault()		\n
 *			PWM3_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen3Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(IndependentFaultEN	!= DONT_CHANGE) {FCLCON3bits.IFLTMOD = IndependentFaultEN&0b1;}				\
	if(FaultSource		!= DONT_CHANGE) {FCLCON3bits.FLTSRC = FaultSource&0b11111;}				\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM3 = FaultMode&0b1;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON3bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON3bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON3bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON3bits.FLTDAT1 = FaultData_H;									\
		IOCON3bits.FLTDAT0 = FaultData_L;									\
		FCLCON3bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON3bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON3bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON3bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen3Fault(FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator3 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 *
 * <u>FaultSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator3 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM3Fault()		\n
 *			DisableIntPWM3Fault()		\n
 *			PWM3_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen3Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(FaultSource		!= DONT_CHANGE) {FCLCON3bits.FLTSRC = FaultSource&0b1;}					\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM3 = FaultMode&0b11111;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON3bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON3bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON3bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON3bits.FLTDAT1 = FaultData_H;									\
		IOCON3bits.FLTDAT0 = FaultData_L;									\
		FCLCON3bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON3bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON3bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON3bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen3CurrentLimit(CurrentLimSource,CurrentLimData_H,CurrentLimData_L,CurrentLimInputActiveState)
 *
 * \Description		Configures PWM Generator3 Current Limit Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>CurrentLimSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimInputActiveState:</u>\n
 *	- CURRENT_LIM_INPUT_ACTIVE_HIGH	\n
 *	- CURRENT_LIM_INPUT_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator3 Current Limit Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Current Limit Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Use Current Limit Data High/Low in Independent Fault Mode.
 *
 * \RelatedMacros	EnablePWM3CurrentLim()		\n
 *			DisablePWM3CurrentLim()		\n
 *			EnableIntPWM3CLim()		\n
 *			DisableIntPWM3CLim()		\n
 *			PWM3_CUR_LIM_INTERRUPT_IS_PENDING <i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- CurrentLimInputActiveState should not be changed after the PWM is enabled (PTEN = 1).\n
 *		2- Current Limit Data High/Low is Not used in Independent Fault Mode.\n
 *		3- Refer to Device Datasheet For Avaliable Current Limit Sources.
 *
 * \Example    Err=SetupPWMGen3CurrentLimit(SOURCE_IS_COMPARATOR2,SET_H_PIN_LOW,SET_L_PIN_LOW,CURRENT_LIM_INPUT_ACTIVE_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3CurrentLimit(CurrentLimSource, CurrentLimData_H, CurrentLimData_L, CurrentLimInputActiveState)({			\
																	\
	unsigned int _Return=0;														\
	if(CurrentLimData_H != DONT_CHANGE) {												\
		IOCON3bits.CLDAT1 = CurrentLimData_H&0b1;										\
		if(FCLCON3bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																	\
	if(CurrentLimData_L != DONT_CHANGE) {												\
		IOCON3bits.CLDAT0 = CurrentLimData_L&0b1;										\
		if(FCLCON3bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																 	\
	if(CurrentLimSource != DONT_CHANGE) {FCLCON3bits.CLSRC = CurrentLimSource&0b11111;}						\
	if(CurrentLimInputActiveState != DONT_CHANGE) {											\
		if(PTCONbits.PTEN == 0){												\
			FCLCON3bits.CLPOL = CurrentLimInputActiveState&0b1;								\
		} else {														\
			if (CurrentLimInputActiveState != FCLCON3bits.CLPOL) {								\
				_Return=1; /*Set Error "PWM Current Limit Polarity Cant be Changed When The PWM Module is Enabled"*/	\
			}														\
		}															\
	}																\
	_Return;															\
})

/***********************************************************************************************************************
 * \Function	void SetupPWMGen3LEB(PWMH_RisingEdgeBlankingEN,PWMH_FallingEdgeBlankingEN,PWML_RisingEdgeBlankingEN,PWML_FallingEdgeBlankingEN,LEB_Delay)
 *
 * \Description		Configures PWM Generator3 Leading Edge Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_RISING_EDGE_BLANK \n
 *	- ENABLE_PWMH_RISING_EDGE_BLANK	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- DONT_CHANGE
 *
 * <u>PWML_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_RISING_EDGE_BLANK \n
 *	- ENABLE_PWML_RISING_EDGE_BLANK	\n
 * 	- DONT_CHANGE
 *
 * <u>PWML_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWML_FALLING_EDGE_BLANK \n
 * 	- DONT_CHANGE
 *
 * <u>LEB_Delay:</u>\n
 *	- a Value Between 1 and 4096 Inclusivly. (LEB Delay = (0 to 4096) * PWM Clock Freq) \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM3FaultBlanking()	\n
 *			EnablePWM3CurrentLimBlanking()	\n
 *			DisablePWM3FaultBlanking()	\n
 *			DisablePWM3CurrentLimBlanking()
 *
 * \Notes		LEB Delay = (0 to 4096) * PWM Clock Freq.
 *
 * \Example
 * SetupPWMGen3LEB(ENABLE_PWMH_RISING_EDGE_BLANK,ENABLE_PWMH_FALLING_EDGE_BLANK,ENABLE_PWML_RISING_EDGE_BLANK,ENABLE_PWML_FALLING_EDGE_BLANK,990);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3LEB(PWMH_RisingEdgeBlankingEN, PWMH_FallingEdgeBlankingEN,  PWML_RisingEdgeBlankingEN, PWML_FallingEdgeBlankingEN, LEB_Delay)({	\
																			\
	if(PWMH_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON3bits.PHR = PWMH_RisingEdgeBlankingEN&0b1;}						\
	if(PWMH_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON3bits.PHF = PWMH_FallingEdgeBlankingEN&0b1;}						\
	if(PWML_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON3bits.PLR = PWML_RisingEdgeBlankingEN&0b1;}						\
	if(PWML_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON3bits.PLF = PWML_FallingEdgeBlankingEN&0b1;}						\
	if(LEB_Delay			!= DONT_CHANGE) {LEBDLY3 = LEB_Delay&0b111111111111;}								\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen3StateBlanking(PWMH_BlankingState,PWML_BlankingState,SelectedSignalForStateBlank,SelectedSignalBlankingState)
 *
 * \Description		Configures PWM Generator3 State Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWMH_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_LOW	\n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>PWML_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWML_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>SelectedSignalForStateBlank:</u>\n
 *	- NO_SIGNAL_SOURCE_FOR_STATE_BLNKING \n
 *	- PWM3H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM2H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM3H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM4H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM5H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM6H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM7H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 * 	- DONT_CHANGE
 *
 * <u>SelectedSignalBlankingState:</u>\n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM3FaultBlanking()	\n
 *			EnablePWM3CurrentLimBlanking()	\n
 *			DisablePWM3FaultBlanking()	\n
 *			DisablePWM3CurrentLimBlanking()
 *
 * \Notes		None.
 *
 * \Example	SetupPWMGen3StateBlanking(ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH,ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH,
 *					  PWM3H_AS_STATE_BLANK_SIGNAL_SOURCE,ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3StateBlanking(PWMH_BlankingState, PWML_BlankingState, SelectedSignalForStateBlank, SelectedSignalBlankingState)({			\
																			\
	if(PWMH_BlankingState		!= DONT_CHANGE) {LEBCON3bits.BPHH = (PWMH_BlankingState&0b11)>>1; LEBCON3bits.BPHL = PWMH_BlankingState&0b1;}	\
	if(PWML_BlankingState		!= DONT_CHANGE) {LEBCON3bits.BPLH = (PWML_BlankingState&0b11)>>1; LEBCON3bits.BPLL = PWML_BlankingState&0b1;}	\
	if(SelectedSignalBlankingState	!= DONT_CHANGE) {LEBCON3bits.BCH  = (SelectedSignalBlankingState&0b11)>>1;					\
							 LEBCON3bits.BCL  =  SelectedSignalBlankingState&0b1;}						\
	if(SelectedSignalForStateBlank	!= DONT_CHANGE) {AUXCON3bits.BLANKSEL = SelectedSignalForStateBlank&0b111;}					\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen3Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)
 *
 * \Description		Configures PWM Generator3 Chop Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>ChopClkSource:</u> <i>/Refer to Device Datasheet For Avaliable Chop Clock Sources.</i>\n
 *	- CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE \n
 *	- PWM3H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM2H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM3H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM4H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM5H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM6H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM7H_AS_CHOP_CLOCK_SOURCE	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_ChopEN:</u>\n
 *	- DISABLE_PWMH_CHOP \n
 *	- ENABLE_PWMH_CHOP \n
 *	- DONT_CHANGE
 *
 * <u>PWML_ChopEN:</u>\n
 *	- DISABLE_PWML_CHOP \n
 *	- ENABLE_PWML_CHOP \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		Refer to Device Datasheet For Avaliable Chop Clock Sources.
 *
 * \Example	SetupPWMGen3Chop(CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE, ENABLE_PWMH_CHOP, ENABLE_PWML_CHOP);
 *
 **********************************************************************************************************************/
#define SetupPWMGen3Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)({				\
												\
	if(ChopClkSource	!= DONT_CHANGE) {AUXCON3bits.CHOPSEL = ChopClkSource&0b111;}	\
	if(PWMH_ChopEN		!= DONT_CHANGE) {AUXCON3bits.CHOPHEN = PWMH_ChopEN&0b1;}	\
	if(PWML_ChopEN		!= DONT_CHANGE) {AUXCON3bits.CHOPLEN = PWML_ChopEN&0b1;}	\
})
/***/

// <editor-fold defaultstate="collapsed" desc="PWM3 Macros and Defines">

#ifdef PWMCAP3
/******************************************************************************************************
 * \brief	The value Returned From this Routine represents the captured PWM time base value when
 *		a leading edge is detected on the current-limit input.
 *
 * \Notes	1: The capture feature is only available on the primary output (PWMxH).
 *		2: The feature is only active after LEB processing on the current-limit input signal is complete.
 *
 *****************************************************************************************************/
#define CaptuerPWM3PriTB_Val()			PWMCAP3
#endif
/***/
#define EnablePWM3Fault()			FCLCON3bits.FLTMOD = FaultModePWM3
#define DisablePWM3Fault()			FCLCON3bits.FLTMOD = 0b11

#define EnablePWM3CurrentLim()			FCLCON3bits.CLMOD = 1
#define DisablePWM3CurrentLim()			FCLCON3bits.CLMOD = 0

#define EnablePWM3FaultBlanking()		LEBCON3bits.FLTLEBEN = 1
#define DisablePWM3FaultBlanking()		LEBCON3bits.FLTLEBEN = 0

#define EnablePWM3CurrentLimBlanking()		LEBCON3bits.CLLEBEN = 1
#define DisablePWM3CurrentLimBlanking()		LEBCON3bits.CLLEBEN = 0

#define SetDutyCyclePWM3(DC)			(PDC3=DC)	/*For Independent PWM DCMode With Complementary PWM Output Mode.*/
#ifdef STPER /* Devices That Have Secondary Time Base Will Have True Independent Duty Cycle Mode Option.*/
#define SetDutyCyclePWM3H(DC)			(PDC3=DC)	/*For Independent PWM DCMode*/
#define SetDutyCyclePWM3L(DC)			(SDC3=DC)	/*For Independent PWM DCMode*/
#endif
#define SetMasterDutyCycle(DC)			(MDC=DC)	/*When Using Master DCMode.*/

/** PWM Geneator1 Interrupt Handling */
#define EnableIntPWM3()				_PWM3IE = 1
#define DisableIntPWM3()			_PWM3IE = 0
#define SetIntPriorityPWM3(Priority)		_PWM3IP = (Priority)
#define PWM3_INT_FLAG				_PWM3IF

#define EnableIntPWM3Trig()			PWMCON3bits.TRGIEN = 1
#define DisableIntPWM3Trig()			PWMCON3bits.TRGIEN = 0
#define PWM3_TRIG_INTERRUPT_IS_PENDING		PWMCON3bits.TRGSTAT

#define	EnableIntPWM3Fault()			PWMCON3bits.FLTIEN = 1
#define DisableIntPWM3Fault()			PWMCON3bits.FLTIEN = 0
#define PWM3_FAULT_INTERRUPT_IS_PENDING		PWMCON3bits.FLTSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

#define	EnableIntPWM3CLim()			PWMCON3bits.CLIEN = 1
#define	DisableIntPWM3CLim()			PWMCON3bits.CLIEN = 0
#define PWM3_CUR_LIM_INTERRUPT_IS_PENDING	PWMCON3bits.CLSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

// </editor-fold>

#endif

#ifdef _PWM4IF

#ifdef STPER
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen4(PWM_Mode,Aligment,TimeBaseSelect,Period_H,Period_L,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator4.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 *	- TRUE_INDEPENDENT_MODE	<i>/True Independent Output Mode is Not Avaliable On All Devices.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- SECONDARY_TIME_BASE	<i>/Secondary Time Base is Not Avaliable On All Devices.</i> \n
 *	- INDEPENDENT_TIME_BASE	\n
 * 	- DONT_CHANGE
 *
 * <u>Period_H:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>Period_L:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator4 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).\n
 *		<b> 3: </b> Err3: An Attempt to use secondary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM4(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetDutyCyclePWM4H(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetDutyCyclePWM4L(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen4(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4(_PWM_Mode, _Aligment, _TimeBaseSelect, Period_H, Period_L, _DCMode, _ENimmediatDCupdate)({					\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON4bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON4bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON4bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON4bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON4bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON4bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON4bits.PMOD = PWM_Mode;	/* 3 = PWM I/O pin pair is in True Independent PWM Output mode*/				\
						/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON4bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON4bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON4bits.CAM)	PWMCON4bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case SECONDARY_TIME_BASE:	/* 1 = PWM generator uses the secondary master time base FixedSecPeriod*/		\
				if(!PWMCON4bits.CAM)	PWMCON4bits.MTBS = 1;									\
				else			_Return=3; /* Set Error "We cant use secondary time base in center aligne mode"*/	\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx/SPHASEx registers provide time base period for this PWM generator*/	\
				PWMCON4bits.ITB  = 1;												\
				PHASE4 = Period_H;	/* Independent Period For PWM4H*/							\
				SPHASE4 = Period_L;	/* Independent Period For PWM4L*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON4bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
				/*0 = (Independent Duty Cycle) PDCx and SDCx registers provide duty cycle information for this PWM generator*/	\
		PWMCON4bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen4(PWM_Mode,Aligment,TimeBaseSelect,Period,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator4.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- INDEPENDENT_TIME_BASE	<i>/PHASEx register provides time base period for this PWM generator.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Period:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator4 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM4(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen4(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4(_PWM_Mode, _Aligment, _TimeBaseSelect, Period, _DCMode, _ENimmediatDCupdate)({							\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON4bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON4bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON4bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON4bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON4bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON4bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON4bits.PMOD = PWM_Mode;	/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON4bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON4bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON4bits.CAM)	PWMCON4bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx register provide time base period for this PWM generator*/		\
				PWMCON4bits.ITB  = 1;												\
				PHASE4 = Period;	/* Independent Period For PWM4*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON4bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
					/*0 = (Independent Duty Cycle) PDCx register provide duty cycle information for this PWM generator*/	\
		PWMCON4bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#endif

/***********************************************************************************************************************
 * \Function		void SetupPWMGen4Pins(H_PinOwnership,L_PinOwnership,H_PinPolarity,L_PinPolarity,EN_PinSwap)
 *
 * \Description		Configures PWM Generator4 Output Pins.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_H_PIN	\n
 *	- PWM_MODULE_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_L_PIN	\n
 *	- PWM_MODULE_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>H_PinPolarity:</u>	\n
 *	- H_PIN_ACTIVE_HIGH	\n
 *	- H_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>L_PinPolarity:</u>	\n
 *	- L_PIN_ACTIVE_HIGH	\n
 *	- L_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>EN_PinSwap:</u>		\n
 *	- H_L_PINS_NOT_SWAPED	\n
 *	- H_L_PINS_SWAPED	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen4Pins(PWM_MODULE_CONTROL_H_PIN,PWM_MODULE_CONTROL_L_PIN,H_PIN_ACTIVE_HIGH,L_PIN_ACTIVE_HIGH,H_L_PINS_NOT_SWAPED);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4Pins(H_PinOwnership, L_PinOwnership, H_PinPolarity, L_PinPolarity, EN_PinSwap)({	\
	if(H_PinOwnership != DONT_CHANGE) {IOCON4bits.PENH = H_PinOwnership&0b1;}			\
	if(L_PinOwnership != DONT_CHANGE) {IOCON4bits.PENL = L_PinOwnership&0b1;}			\
	if(H_PinPolarity  != DONT_CHANGE) {IOCON4bits.POLH = H_PinPolarity&0b1;}			\
	if(L_PinPolarity  != DONT_CHANGE) {IOCON4bits.POLL = L_PinPolarity&0b1;}			\
	if(EN_PinSwap     != DONT_CHANGE) {IOCON4bits.SWAP = EN_PinSwap&0b1;}				\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen4Override(H_PinOverEN,L_PinOverEN,OverDataForH_Pin,OverDataForL_Pin,SyncOverride)
 *
 * \Description		Configures And Set PWM Generator4 Pins Override.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_H_PIN	\n
 *	- OVR_DAT_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_L_PIN	\n
 *	- OVR_DAT_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>OverDataForH_Pin:</u>	\n
 *	- OVERRIDE_H_PIN_LOW	\n
 *	- OVERRIDE_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>OverDataForL_Pin:</u>	\n
 *	- OVERRIDE_L_PIN_LOW	\n
 *	- OVERRIDE_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>SyncOverride:</u>		\n
 *	- DISABLE_OVERRIDE_SYNC	\n
 *	- ENABLE_OVERRIDE_SYNC	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen4Override(PWM_GEN_CONTROL_H_PIN, PWM_GEN_CONTROL_L_PIN, DONT_CHANGE, DONT_CHANGE, DONT_CHANGE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4Override(H_PinOverEN, L_PinOverEN, OverDataForH_Pin, OverDataForL_Pin, SyncOverride)({	\
	if(H_PinOverEN		!= DONT_CHANGE) {IOCON4bits.OVRENH  = H_PinOverEN&0b1;}				\
	if(L_PinOverEN		!= DONT_CHANGE) {IOCON4bits.OVRENL  = L_PinOverEN&0b1;}				\
	if(OverDataForL_Pin	!= DONT_CHANGE) {IOCON4bits.OVRDAT0 = OverDataForL_Pin&0b1;}			\
	if(OverDataForH_Pin	!= DONT_CHANGE) {IOCON4bits.OVRDAT1 = OverDataForH_Pin&0b1;}			\
	if(SyncOverride		!= DONT_CHANGE) {IOCON4bits.OSYNC   = SyncOverride&0b1;}			\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen4PhaseShifting(PhShift_H, PhShift_L)
 *
 * \Description		Set PWM Generator4 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift_H:</u> <i>/For Complementary And Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * <u>PhShift_L:</u> <i>/ONLY For Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator4 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen4PhaseShifting(6000, 12000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4PhaseShifting(PhShift_H, PhShift_L)({							\
														\
	unsigned int _Return=0;											\
	if(PWMCON4bits.CAM == EDGE_ALIGNED){									\
		if(PhShift_H != DONT_CHANGE) {PHASE4 = PhShift_H;}						\
		if(PhShift_L != DONT_CHANGE) {SPHASE4 = PhShift_L;}						\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen4PhaseShifting(PhShift)
 *
 * \Description		Set PWM Generator4 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift:</u> \n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator4 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen4PhaseShifting(6000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4PhaseShifting(PhShift)({									\
														\
	unsigned int _Return=0;											\
	if(PWMCON4bits.CAM == EDGE_ALIGNED){									\
		if(PhShift != DONT_CHANGE) {PHASE4 = PhShift;}							\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen4DeadTime(DeadTimeMode,DeadTime,AlternatDeadTime)
 *
 * \Description		Configures And Set PWM Generator4 Dead Time.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>DeadTimeMode:</u>	\n
 *	- POSITIVE_DT_MODE			\n
 *	- NEGATIVE_DT_MODE			<i>/Not allowed in Center_Align mode.</i>\n
 *	- DISABLE_DT_MODE			\n
 *	- NEGATIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxH is shortened and PWMxL is lengthened And
 *							If DTCMPx Pin = 1, PWMxL is shortened and PWMxH is lengthened.</i>\n
 *	- POSITIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxL is shortened and PWMxH is lengthened And
 *							If DTCMPx Pin = 1, PWMxH is shortened and PWMxL is lengthened.</i>\n
 *	- DONT_CHANGE
 *
 * <u>DeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Is Not Used For Dead Time Value In CENTER_ALIGNED Output Mode So Use AlternatDeadTime Param For Both PWMxH & PWMxL Outputs.\n
 *	- This Parameter Holds The Value To Be Added to Or Subtracted From The Duty Cycle in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE.
 *
 * <u>AlternatDeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in CENTER_ALIGNED PWM Output Mode.\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE DeadTimeMode.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator4 Dead Time Parameters Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Dead time Compensation in None Complementary PWM output mode with Positive Dead Time.\n
 *		<b> 2: </b> Err2: An Attempt to Use Negative Dead Time in None Complementary Edge Aligned PWM output mode.\n
 *		<b> 3: </b> Warn1: An Attempt to Use Primary Dead Time (DeadTime) in Center Align Mode With Positive dead time, (You May Keep it Zero).
 *
 * \RelatedMacros	None.
 *
 * \Notes	1- Open "Dead Time Configrations.png" Chart in "Important Files" Folder for Comprehensive Understanding of Dead Time Module Operation.\n
 *		2- Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode.			\n
 *		3- Primary Dead Time (DeadTime) is not used in Center Align Mode So Keep it Zero.						\n
 *		4- Negative Dead time is available only for Complementary Edge Aligned PWM output mode.
 *
 * \Example     Err=SetupPWMGen4DeadTime(POSITIVE_DT_MODE,1000,1200);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4DeadTime(_DeadTimeMode, DeadTime, AlternatDeadTime)({										\
																			\
	unsigned int _Return=0;																\
	unsigned int DeadTimeMode=_DeadTimeMode;													\
	if(DeadTimeMode == DONT_CHANGE) {														\
		DeadTimeMode=PWMCON4bits.DTC;														\
		if( (DeadTimeMode==0b11) && PWMCON4bits.DTCP)												\
			DeadTimeMode=POSITIVE_COMPENSATION_DT_MODE;											\
	}																		\
	switch (DeadTimeMode){																\
		case POSITIVE_COMPENSATION_DT_MODE :													\
			if(IOCON4bits.PMOD) { /* check if Complementary PWM output mode is Used But if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON4bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON4bits.DTCP= 1;		/* Dead-Time Compensation Polarity is Positive */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers */\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case NEGATIVE_COMPENSATION_DT_MODE :													\
			if(IOCON4bits.PMOD) { /* check if Complementary PWM output mode is Used And if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON4bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON4bits.DTCP= 0;		/* Dead-Time Compensation Polarity is Negative */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers*/	\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case POSITIVE_DT_MODE :															\
			PWMCON4bits.DTC = 0;		/* Positive dead time actively applied for all output modes */					\
			if((PWMCON4bits.CAM)&&(DeadTime)) {												\
				/*Set Warning "Primary Dead Time (DTRx) is not used in Center Align Mode With Positive dead time So Keep it Zero!"*/	\
				_Return=3;														\
			}																\
			/* DeadTime Holds The Value of Dead Time for PWMxH output signal */								\
			/* AlternatDeadTime Holds The Dead Time Value for PWMxL output signal OR for both the PWMxH and PWMxL in Center Align mode */	\
			break;																\
		case NEGATIVE_DT_MODE :															\
			if ((IOCON4bits.PMOD == COMPLEMENTARY_MODE) && (PWMCON4bits.CAM == EDGE_ALIGNED)) {						\
				PWMCON4bits.DTC = 1;	/* Negative dead time actively applied for Complementary Output mode */				\
				/* DeadTime Holds The Value of Dead Time for PWMxL output signal */							\
				/* AlternatDeadTime Holds The Dead Time Value for PWMxH output signal */						\
			}else{																\
				/*Set Error "Negative Dead time is available only for Complementary Edge Aligned PWM output mode"*/			\
				_Return=2;														\
			}																\
			break;																\
		case DISABLE_DT_MODE :															\
			PWMCON4bits.DTC = 0;														\
	}																		\
	if(DeadTime		!= DONT_CHANGE) {DTR4	= DeadTime&0b11111111111111;}									\
	if(AlternatDeadTime	!= DONT_CHANGE) {ALTDTR4 = AlternatDeadTime&0b11111111111111;}								\
	_Return;																	\
})

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen4ADCTrigger(TriggerCompareVal,TrigStartWaitCycles,TrigOutputDiv)
 *
 * \Description		Configures PWM Generator4 For ADC Triggering.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>TriggerCompareVal:</u>\n
 *	- a Value Between 1 and 65534 Inclusivly.\n
 * 	- DONT_CHANGE
 *
 * <u>TrigStartWaitCycles:</u>\n
 *	- WAIT_0_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	- WAIT_1_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	-            .				\n
 *	-            .				\n
 *	- WAIT_63_PWM_CYC_BEFORE_FIRST_TRIG	\n
 * 	- DONT_CHANGE
 *
 * <u>TrigOutputDiv:</u>\n
 *	- TRIGGER_ON_EVERY_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_2ND_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_3RD_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_4TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_5TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_6TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_7TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_8TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_9TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_10TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_11TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_12TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_13TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_14TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_15TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_16TH_TRIGGER_EVENT	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator4 ADC Triggering Function Configered Correctly.\n
 *		<b> 1: </b> Err1: TRIGx Value Is Bigger Than PTPER, (When Using Primary Time Base).\n
 *		<b> 2: </b> Err2: TRIGx Value Is Bigger Than STPER, (When Using Secondary Time Base).\n
 *		<b> 3: </b> Err3: TRIGx Value Is Bigger Than PHASEx, (When Using Independent Time Base Mode).
 *
 * \RelatedMacros	EnableIntPWM4Trig()		\n
 *			DisableIntPWM4Trig()		\n
 *			PWM4_TRIG_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- TriggerCompareVal Must Be Smaller Than PHASEx In Independent Time Base Mode.	\n
 *		2- TriggerCompareVal Must Be Smaller Than STPER When Using Secondary Time Base.	\n
 *		3- TriggerCompareVal Must Be Smaller Than PTPER When Using Primary Time Base.
 *
 * \Example     Err=SetupPWMGen4ADCTrigger(2400,WAIT_16_PWM_CYC_BEFORE_FIRST_TRIG,TRIGGER_ON_EVERY_TRIGGER_EVENT);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4ADCTrigger(TriggerCompareVal, TrigStartWaitCycles, TrigOutputDiv)({								\
																		\
	unsigned int _Return=0;															\
	if(TriggerCompareVal != DONT_CHANGE) {													\
		switch (PWMCON4bits.ITB){													\
			case 1:															\
				if(TriggerCompareVal<PHASE4) {TRIG4 = TriggerCompareVal;}							\
				else {_Return=3;/*Set Error "TRIGx Value Must Be Smaller than PHASEx In Independent Time Base Mode"*/}		\
				break;														\
			case 0:															\
				if(PWMCON4bits.MTBS){												\
					if(TriggerCompareVal<STPER) {TRIG4 = TriggerCompareVal;}						\
					else {_Return=2;/*Set Error "TRIGx Value Must Be Smaller Than STPER When Using Secondary Time Base"*/}	\
				}else{														\
					if(TriggerCompareVal<PTPER) {TRIG4 = TriggerCompareVal;}						\
					else {_Return=1;/*Set Error "TRIGx Value Must Be Smaller Than PTPER When Using Primary Time Base"*/}	\
				}														\
		}																\
	}																	\
	if(TrigStartWaitCycles	!= DONT_CHANGE) {TRGCON4bits.TRGSTRT = TrigStartWaitCycles&0b111111;}						\
	if(TrigOutputDiv	!= DONT_CHANGE) {TRGCON4bits.TRGDIV  = TrigOutputDiv&0b1111;}							\
	_Return;																\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen4Fault(IndependentFaultEN,FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator4 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>IndependentFaultEN:</u>\n
 *	- NORMAL_FAULT_MODE <i>/ Fault inputs map FLTDAT<1:0> to PWMxH and PWMxL and 
 *									 Current-limit map CLDAT<1:0> to PWMxH and PWMxL.</i>\n
 *	- INDEPENDENT_FAULT_MODE<i>/ Current-limit inputs map FLTDAT<1> to PWMxH output and 
 *								   Fault input maps FLTDAT<0> to PWMxL output.
 *								   The CLDAT<1:0> bits are not used for override functions.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>FaultSource:</u> <i>/Refer to Device Datasheet For Avaliable Fault Sources.</i>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_FAULT8_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator4 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM4Fault()		\n
 *			DisableIntPWM4Fault()		\n
 *			PWM4_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen4Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(IndependentFaultEN	!= DONT_CHANGE) {FCLCON4bits.IFLTMOD = IndependentFaultEN&0b1;}				\
	if(FaultSource		!= DONT_CHANGE) {FCLCON4bits.FLTSRC = FaultSource&0b11111;}				\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM4 = FaultMode&0b1;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON4bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON4bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON4bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON4bits.FLTDAT1 = FaultData_H;									\
		IOCON4bits.FLTDAT0 = FaultData_L;									\
		FCLCON4bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON4bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON4bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON4bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen4Fault(FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator4 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 *
 * <u>FaultSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator4 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM4Fault()		\n
 *			DisableIntPWM4Fault()		\n
 *			PWM4_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen4Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(FaultSource		!= DONT_CHANGE) {FCLCON4bits.FLTSRC = FaultSource&0b1;}					\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM4 = FaultMode&0b11111;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON4bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON4bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON4bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON4bits.FLTDAT1 = FaultData_H;									\
		IOCON4bits.FLTDAT0 = FaultData_L;									\
		FCLCON4bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON4bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON4bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON4bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen4CurrentLimit(CurrentLimSource,CurrentLimData_H,CurrentLimData_L,CurrentLimInputActiveState)
 *
 * \Description		Configures PWM Generator4 Current Limit Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>CurrentLimSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimInputActiveState:</u>\n
 *	- CURRENT_LIM_INPUT_ACTIVE_HIGH	\n
 *	- CURRENT_LIM_INPUT_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator4 Current Limit Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Current Limit Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Use Current Limit Data High/Low in Independent Fault Mode.
 *
 * \RelatedMacros	EnablePWM4CurrentLim()		\n
 *			DisablePWM4CurrentLim()		\n
 *			EnableIntPWM4CLim()		\n
 *			DisableIntPWM4CLim()		\n
 *			PWM4_CUR_LIM_INTERRUPT_IS_PENDING <i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- CurrentLimInputActiveState should not be changed after the PWM is enabled (PTEN = 1).\n
 *		2- Current Limit Data High/Low is Not used in Independent Fault Mode.\n
 *		3- Refer to Device Datasheet For Avaliable Current Limit Sources.
 *
 * \Example    Err=SetupPWMGen4CurrentLimit(SOURCE_IS_COMPARATOR2,SET_H_PIN_LOW,SET_L_PIN_LOW,CURRENT_LIM_INPUT_ACTIVE_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4CurrentLimit(CurrentLimSource, CurrentLimData_H, CurrentLimData_L, CurrentLimInputActiveState)({			\
																	\
	unsigned int _Return=0;														\
	if(CurrentLimData_H != DONT_CHANGE) {												\
		IOCON4bits.CLDAT1 = CurrentLimData_H&0b1;										\
		if(FCLCON4bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																	\
	if(CurrentLimData_L != DONT_CHANGE) {												\
		IOCON4bits.CLDAT0 = CurrentLimData_L&0b1;										\
		if(FCLCON4bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																 	\
	if(CurrentLimSource != DONT_CHANGE) {FCLCON4bits.CLSRC = CurrentLimSource&0b11111;}						\
	if(CurrentLimInputActiveState != DONT_CHANGE) {											\
		if(PTCONbits.PTEN == 0){												\
			FCLCON4bits.CLPOL = CurrentLimInputActiveState&0b1;								\
		} else {														\
			if (CurrentLimInputActiveState != FCLCON4bits.CLPOL) {								\
				_Return=1; /*Set Error "PWM Current Limit Polarity Cant be Changed When The PWM Module is Enabled"*/	\
			}														\
		}															\
	}																\
	_Return;															\
})

/***********************************************************************************************************************
 * \Function	void SetupPWMGen4LEB(PWMH_RisingEdgeBlankingEN,PWMH_FallingEdgeBlankingEN,PWML_RisingEdgeBlankingEN,PWML_FallingEdgeBlankingEN,LEB_Delay)
 *
 * \Description		Configures PWM Generator4 Leading Edge Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_RISING_EDGE_BLANK \n
 *	- ENABLE_PWMH_RISING_EDGE_BLANK	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- DONT_CHANGE
 *
 * <u>PWML_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_RISING_EDGE_BLANK \n
 *	- ENABLE_PWML_RISING_EDGE_BLANK	\n
 * 	- DONT_CHANGE
 *
 * <u>PWML_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWML_FALLING_EDGE_BLANK \n
 * 	- DONT_CHANGE
 *
 * <u>LEB_Delay:</u>\n
 *	- a Value Between 1 and 4096 Inclusivly. (LEB Delay = (0 to 4096) * PWM Clock Freq) \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM4FaultBlanking()	\n
 *			EnablePWM4CurrentLimBlanking()	\n
 *			DisablePWM4FaultBlanking()	\n
 *			DisablePWM4CurrentLimBlanking()
 *
 * \Notes		LEB Delay = (0 to 4096) * PWM Clock Freq.
 *
 * \Example
 * SetupPWMGen4LEB(ENABLE_PWMH_RISING_EDGE_BLANK,ENABLE_PWMH_FALLING_EDGE_BLANK,ENABLE_PWML_RISING_EDGE_BLANK,ENABLE_PWML_FALLING_EDGE_BLANK,990);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4LEB(PWMH_RisingEdgeBlankingEN, PWMH_FallingEdgeBlankingEN,  PWML_RisingEdgeBlankingEN, PWML_FallingEdgeBlankingEN, LEB_Delay)({	\
																			\
	if(PWMH_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON4bits.PHR = PWMH_RisingEdgeBlankingEN&0b1;}						\
	if(PWMH_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON4bits.PHF = PWMH_FallingEdgeBlankingEN&0b1;}						\
	if(PWML_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON4bits.PLR = PWML_RisingEdgeBlankingEN&0b1;}						\
	if(PWML_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON4bits.PLF = PWML_FallingEdgeBlankingEN&0b1;}						\
	if(LEB_Delay			!= DONT_CHANGE) {LEBDLY4 = LEB_Delay&0b111111111111;}								\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen4StateBlanking(PWMH_BlankingState,PWML_BlankingState,SelectedSignalForStateBlank,SelectedSignalBlankingState)
 *
 * \Description		Configures PWM Generator4 State Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWMH_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_LOW	\n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>PWML_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWML_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>SelectedSignalForStateBlank:</u>\n
 *	- NO_SIGNAL_SOURCE_FOR_STATE_BLNKING \n
 *	- PWM4H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM2H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM3H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM4H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM5H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM6H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM7H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 * 	- DONT_CHANGE
 *
 * <u>SelectedSignalBlankingState:</u>\n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM4FaultBlanking()	\n
 *			EnablePWM4CurrentLimBlanking()	\n
 *			DisablePWM4FaultBlanking()	\n
 *			DisablePWM4CurrentLimBlanking()
 *
 * \Notes		None.
 *
 * \Example	SetupPWMGen4StateBlanking(ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH,ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH,
 *					  PWM4H_AS_STATE_BLANK_SIGNAL_SOURCE,ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4StateBlanking(PWMH_BlankingState, PWML_BlankingState, SelectedSignalForStateBlank, SelectedSignalBlankingState)({			\
																			\
	if(PWMH_BlankingState		!= DONT_CHANGE) {LEBCON4bits.BPHH = (PWMH_BlankingState&0b11)>>1; LEBCON4bits.BPHL = PWMH_BlankingState&0b1;}	\
	if(PWML_BlankingState		!= DONT_CHANGE) {LEBCON4bits.BPLH = (PWML_BlankingState&0b11)>>1; LEBCON4bits.BPLL = PWML_BlankingState&0b1;}	\
	if(SelectedSignalBlankingState	!= DONT_CHANGE) {LEBCON4bits.BCH  = (SelectedSignalBlankingState&0b11)>>1;					\
							 LEBCON4bits.BCL  =  SelectedSignalBlankingState&0b1;}						\
	if(SelectedSignalForStateBlank	!= DONT_CHANGE) {AUXCON4bits.BLANKSEL = SelectedSignalForStateBlank&0b111;}					\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen4Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)
 *
 * \Description		Configures PWM Generator4 Chop Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>ChopClkSource:</u> <i>/Refer to Device Datasheet For Avaliable Chop Clock Sources.</i>\n
 *	- CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE \n
 *	- PWM4H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM2H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM3H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM4H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM5H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM6H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM7H_AS_CHOP_CLOCK_SOURCE	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_ChopEN:</u>\n
 *	- DISABLE_PWMH_CHOP \n
 *	- ENABLE_PWMH_CHOP \n
 *	- DONT_CHANGE
 *
 * <u>PWML_ChopEN:</u>\n
 *	- DISABLE_PWML_CHOP \n
 *	- ENABLE_PWML_CHOP \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		Refer to Device Datasheet For Avaliable Chop Clock Sources.
 *
 * \Example	SetupPWMGen4Chop(CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE, ENABLE_PWMH_CHOP, ENABLE_PWML_CHOP);
 *
 **********************************************************************************************************************/
#define SetupPWMGen4Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)({				\
												\
	if(ChopClkSource	!= DONT_CHANGE) {AUXCON4bits.CHOPSEL = ChopClkSource&0b111;}	\
	if(PWMH_ChopEN		!= DONT_CHANGE) {AUXCON4bits.CHOPHEN = PWMH_ChopEN&0b1;}	\
	if(PWML_ChopEN		!= DONT_CHANGE) {AUXCON4bits.CHOPLEN = PWML_ChopEN&0b1;}	\
})
/***/

// <editor-fold defaultstate="collapsed" desc="PWM4 Macros and Defines">

#ifdef PWMCAP4
/******************************************************************************************************
 * \brief	The value Returned From this Routine represents the captured PWM time base value when
 *		a leading edge is detected on the current-limit input.
 *
 * \Notes	1: The capture feature is only available on the primary output (PWMxH).
 *		2: The feature is only active after LEB processing on the current-limit input signal is complete.
 *
 *****************************************************************************************************/
#define CaptuerPWM4PriTB_Val()			PWMCAP4
#endif
/***/
#define EnablePWM4Fault()			FCLCON4bits.FLTMOD = FaultModePWM4
#define DisablePWM4Fault()			FCLCON4bits.FLTMOD = 0b11

#define EnablePWM4CurrentLim()			FCLCON4bits.CLMOD = 1
#define DisablePWM4CurrentLim()			FCLCON4bits.CLMOD = 0

#define EnablePWM4FaultBlanking()		LEBCON4bits.FLTLEBEN = 1
#define DisablePWM4FaultBlanking()		LEBCON4bits.FLTLEBEN = 0

#define EnablePWM4CurrentLimBlanking()		LEBCON4bits.CLLEBEN = 1
#define DisablePWM4CurrentLimBlanking()		LEBCON4bits.CLLEBEN = 0

#define SetDutyCyclePWM4(DC)			(PDC4=DC)	/*For Independent PWM DCMode With Complementary PWM Output Mode.*/
#ifdef STPER /* Devices That Have Secondary Time Base Will Have True Independent Duty Cycle Mode Option.*/
#define SetDutyCyclePWM4H(DC)			(PDC4=DC)	/*For Independent PWM DCMode*/
#define SetDutyCyclePWM4L(DC)			(SDC4=DC)	/*For Independent PWM DCMode*/
#endif
#define SetMasterDutyCycle(DC)			(MDC=DC)	/*When Using Master DCMode.*/

/** PWM Geneator1 Interrupt Handling */
#define EnableIntPWM4()				_PWM4IE = 1
#define DisableIntPWM4()			_PWM4IE = 0
#define SetIntPriorityPWM4(Priority)		_PWM4IP = (Priority)
#define PWM4_INT_FLAG				_PWM4IF

#define EnableIntPWM4Trig()			PWMCON4bits.TRGIEN = 1
#define DisableIntPWM4Trig()			PWMCON4bits.TRGIEN = 0
#define PWM4_TRIG_INTERRUPT_IS_PENDING		PWMCON4bits.TRGSTAT

#define	EnableIntPWM4Fault()			PWMCON4bits.FLTIEN = 1
#define DisableIntPWM4Fault()			PWMCON4bits.FLTIEN = 0
#define PWM4_FAULT_INTERRUPT_IS_PENDING		PWMCON4bits.FLTSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

#define	EnableIntPWM4CLim()			PWMCON4bits.CLIEN = 1
#define	DisableIntPWM4CLim()			PWMCON4bits.CLIEN = 0
#define PWM4_CUR_LIM_INTERRUPT_IS_PENDING	PWMCON4bits.CLSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

// </editor-fold>

#endif

#ifdef _PWM5IF

#ifdef STPER
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen5(PWM_Mode,Aligment,TimeBaseSelect,Period_H,Period_L,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator5.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 *	- TRUE_INDEPENDENT_MODE	<i>/True Independent Output Mode is Not Avaliable On All Devices.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- SECONDARY_TIME_BASE	<i>/Secondary Time Base is Not Avaliable On All Devices.</i> \n
 *	- INDEPENDENT_TIME_BASE	\n
 * 	- DONT_CHANGE
 *
 * <u>Period_H:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>Period_L:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator5 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).\n
 *		<b> 3: </b> Err3: An Attempt to use secondary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM5(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetDutyCyclePWM5H(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetDutyCyclePWM5L(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen5(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5(_PWM_Mode, _Aligment, _TimeBaseSelect, Period_H, Period_L, _DCMode, _ENimmediatDCupdate)({					\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON5bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON5bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON5bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON5bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON5bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON5bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON5bits.PMOD = PWM_Mode;	/* 3 = PWM I/O pin pair is in True Independent PWM Output mode*/				\
						/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON5bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON5bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON5bits.CAM)	PWMCON5bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case SECONDARY_TIME_BASE:	/* 1 = PWM generator uses the secondary master time base FixedSecPeriod*/		\
				if(!PWMCON5bits.CAM)	PWMCON5bits.MTBS = 1;									\
				else			_Return=3; /* Set Error "We cant use secondary time base in center aligne mode"*/	\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx/SPHASEx registers provide time base period for this PWM generator*/	\
				PWMCON5bits.ITB  = 1;												\
				PHASE5 = Period_H;	/* Independent Period For PWM5H*/							\
				SPHASE5 = Period_L;	/* Independent Period For PWM5L*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON5bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
				/*0 = (Independent Duty Cycle) PDCx and SDCx registers provide duty cycle information for this PWM generator*/	\
		PWMCON5bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen5(PWM_Mode,Aligment,TimeBaseSelect,Period,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator5.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- INDEPENDENT_TIME_BASE	<i>/PHASEx register provides time base period for this PWM generator.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Period:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator5 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM5(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen5(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5(_PWM_Mode, _Aligment, _TimeBaseSelect, Period, _DCMode, _ENimmediatDCupdate)({							\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON5bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON5bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON5bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON5bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON5bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON5bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON5bits.PMOD = PWM_Mode;	/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON5bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON5bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON5bits.CAM)	PWMCON5bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx register provide time base period for this PWM generator*/		\
				PWMCON5bits.ITB  = 1;												\
				PHASE5 = Period;	/* Independent Period For PWM5*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON5bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
					/*0 = (Independent Duty Cycle) PDCx register provide duty cycle information for this PWM generator*/	\
		PWMCON5bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#endif

/***********************************************************************************************************************
 * \Function		void SetupPWMGen5Pins(H_PinOwnership,L_PinOwnership,H_PinPolarity,L_PinPolarity,EN_PinSwap)
 *
 * \Description		Configures PWM Generator5 Output Pins.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_H_PIN	\n
 *	- PWM_MODULE_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_L_PIN	\n
 *	- PWM_MODULE_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>H_PinPolarity:</u>	\n
 *	- H_PIN_ACTIVE_HIGH	\n
 *	- H_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>L_PinPolarity:</u>	\n
 *	- L_PIN_ACTIVE_HIGH	\n
 *	- L_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>EN_PinSwap:</u>		\n
 *	- H_L_PINS_NOT_SWAPED	\n
 *	- H_L_PINS_SWAPED	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen5Pins(PWM_MODULE_CONTROL_H_PIN,PWM_MODULE_CONTROL_L_PIN,H_PIN_ACTIVE_HIGH,L_PIN_ACTIVE_HIGH,H_L_PINS_NOT_SWAPED);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5Pins(H_PinOwnership, L_PinOwnership, H_PinPolarity, L_PinPolarity, EN_PinSwap)({	\
	if(H_PinOwnership != DONT_CHANGE) {IOCON5bits.PENH = H_PinOwnership&0b1;}			\
	if(L_PinOwnership != DONT_CHANGE) {IOCON5bits.PENL = L_PinOwnership&0b1;}			\
	if(H_PinPolarity  != DONT_CHANGE) {IOCON5bits.POLH = H_PinPolarity&0b1;}			\
	if(L_PinPolarity  != DONT_CHANGE) {IOCON5bits.POLL = L_PinPolarity&0b1;}			\
	if(EN_PinSwap     != DONT_CHANGE) {IOCON5bits.SWAP = EN_PinSwap&0b1;}				\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen5Override(H_PinOverEN,L_PinOverEN,OverDataForH_Pin,OverDataForL_Pin,SyncOverride)
 *
 * \Description		Configures And Set PWM Generator5 Pins Override.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_H_PIN	\n
 *	- OVR_DAT_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_L_PIN	\n
 *	- OVR_DAT_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>OverDataForH_Pin:</u>	\n
 *	- OVERRIDE_H_PIN_LOW	\n
 *	- OVERRIDE_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>OverDataForL_Pin:</u>	\n
 *	- OVERRIDE_L_PIN_LOW	\n
 *	- OVERRIDE_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>SyncOverride:</u>		\n
 *	- DISABLE_OVERRIDE_SYNC	\n
 *	- ENABLE_OVERRIDE_SYNC	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen5Override(PWM_GEN_CONTROL_H_PIN, PWM_GEN_CONTROL_L_PIN, DONT_CHANGE, DONT_CHANGE, DONT_CHANGE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5Override(H_PinOverEN, L_PinOverEN, OverDataForH_Pin, OverDataForL_Pin, SyncOverride)({	\
	if(H_PinOverEN		!= DONT_CHANGE) {IOCON5bits.OVRENH  = H_PinOverEN&0b1;}				\
	if(L_PinOverEN		!= DONT_CHANGE) {IOCON5bits.OVRENL  = L_PinOverEN&0b1;}				\
	if(OverDataForL_Pin	!= DONT_CHANGE) {IOCON5bits.OVRDAT0 = OverDataForL_Pin&0b1;}			\
	if(OverDataForH_Pin	!= DONT_CHANGE) {IOCON5bits.OVRDAT1 = OverDataForH_Pin&0b1;}			\
	if(SyncOverride		!= DONT_CHANGE) {IOCON5bits.OSYNC   = SyncOverride&0b1;}			\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen5PhaseShifting(PhShift_H, PhShift_L)
 *
 * \Description		Set PWM Generator5 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift_H:</u> <i>/For Complementary And Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * <u>PhShift_L:</u> <i>/ONLY For Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator5 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen5PhaseShifting(6000, 12000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5PhaseShifting(PhShift_H, PhShift_L)({							\
														\
	unsigned int _Return=0;											\
	if(PWMCON5bits.CAM == EDGE_ALIGNED){									\
		if(PhShift_H != DONT_CHANGE) {PHASE5 = PhShift_H;}						\
		if(PhShift_L != DONT_CHANGE) {SPHASE5 = PhShift_L;}						\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen5PhaseShifting(PhShift)
 *
 * \Description		Set PWM Generator5 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift:</u> \n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator5 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen5PhaseShifting(6000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5PhaseShifting(PhShift)({									\
														\
	unsigned int _Return=0;											\
	if(PWMCON5bits.CAM == EDGE_ALIGNED){									\
		if(PhShift != DONT_CHANGE) {PHASE5 = PhShift;}							\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen5DeadTime(DeadTimeMode,DeadTime,AlternatDeadTime)
 *
 * \Description		Configures And Set PWM Generator5 Dead Time.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>DeadTimeMode:</u>	\n
 *	- POSITIVE_DT_MODE			\n
 *	- NEGATIVE_DT_MODE			<i>/Not allowed in Center_Align mode.</i>\n
 *	- DISABLE_DT_MODE			\n
 *	- NEGATIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxH is shortened and PWMxL is lengthened And
 *							If DTCMPx Pin = 1, PWMxL is shortened and PWMxH is lengthened.</i>\n
 *	- POSITIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxL is shortened and PWMxH is lengthened And
 *							If DTCMPx Pin = 1, PWMxH is shortened and PWMxL is lengthened.</i>\n
 *	- DONT_CHANGE
 *
 * <u>DeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Is Not Used For Dead Time Value In CENTER_ALIGNED Output Mode So Use AlternatDeadTime Param For Both PWMxH & PWMxL Outputs.\n
 *	- This Parameter Holds The Value To Be Added to Or Subtracted From The Duty Cycle in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE.
 *
 * <u>AlternatDeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in CENTER_ALIGNED PWM Output Mode.\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE DeadTimeMode.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator5 Dead Time Parameters Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Dead time Compensation in None Complementary PWM output mode with Positive Dead Time.\n
 *		<b> 2: </b> Err2: An Attempt to Use Negative Dead Time in None Complementary Edge Aligned PWM output mode.\n
 *		<b> 3: </b> Warn1: An Attempt to Use Primary Dead Time (DeadTime) in Center Align Mode With Positive dead time, (You May Keep it Zero).
 *
 * \RelatedMacros	None.
 *
 * \Notes	1- Open "Dead Time Configrations.png" Chart in "Important Files" Folder for Comprehensive Understanding of Dead Time Module Operation.\n
 *		2- Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode.			\n
 *		3- Primary Dead Time (DeadTime) is not used in Center Align Mode So Keep it Zero.						\n
 *		4- Negative Dead time is available only for Complementary Edge Aligned PWM output mode.
 *
 * \Example     Err=SetupPWMGen5DeadTime(POSITIVE_DT_MODE,1000,1200);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5DeadTime(_DeadTimeMode, DeadTime, AlternatDeadTime)({										\
																			\
	unsigned int _Return=0;																\
	unsigned int DeadTimeMode=_DeadTimeMode;													\
	if(DeadTimeMode == DONT_CHANGE) {														\
		DeadTimeMode=PWMCON5bits.DTC;														\
		if( (DeadTimeMode==0b11) && PWMCON5bits.DTCP)												\
			DeadTimeMode=POSITIVE_COMPENSATION_DT_MODE;											\
	}																		\
	switch (DeadTimeMode){																\
		case POSITIVE_COMPENSATION_DT_MODE :													\
			if(IOCON5bits.PMOD) { /* check if Complementary PWM output mode is Used But if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON5bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON5bits.DTCP= 1;		/* Dead-Time Compensation Polarity is Positive */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers */\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case NEGATIVE_COMPENSATION_DT_MODE :													\
			if(IOCON5bits.PMOD) { /* check if Complementary PWM output mode is Used And if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON5bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON5bits.DTCP= 0;		/* Dead-Time Compensation Polarity is Negative */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers*/	\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case POSITIVE_DT_MODE :															\
			PWMCON5bits.DTC = 0;		/* Positive dead time actively applied for all output modes */					\
			if((PWMCON5bits.CAM)&&(DeadTime)) {												\
				/*Set Warning "Primary Dead Time (DTRx) is not used in Center Align Mode With Positive dead time So Keep it Zero!"*/	\
				_Return=3;														\
			}																\
			/* DeadTime Holds The Value of Dead Time for PWMxH output signal */								\
			/* AlternatDeadTime Holds The Dead Time Value for PWMxL output signal OR for both the PWMxH and PWMxL in Center Align mode */	\
			break;																\
		case NEGATIVE_DT_MODE :															\
			if ((IOCON5bits.PMOD == COMPLEMENTARY_MODE) && (PWMCON5bits.CAM == EDGE_ALIGNED)) {						\
				PWMCON5bits.DTC = 1;	/* Negative dead time actively applied for Complementary Output mode */				\
				/* DeadTime Holds The Value of Dead Time for PWMxL output signal */							\
				/* AlternatDeadTime Holds The Dead Time Value for PWMxH output signal */						\
			}else{																\
				/*Set Error "Negative Dead time is available only for Complementary Edge Aligned PWM output mode"*/			\
				_Return=2;														\
			}																\
			break;																\
		case DISABLE_DT_MODE :															\
			PWMCON5bits.DTC = 0;														\
	}																		\
	if(DeadTime		!= DONT_CHANGE) {DTR5	= DeadTime&0b11111111111111;}									\
	if(AlternatDeadTime	!= DONT_CHANGE) {ALTDTR5 = AlternatDeadTime&0b11111111111111;}								\
	_Return;																	\
})

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen5ADCTrigger(TriggerCompareVal,TrigStartWaitCycles,TrigOutputDiv)
 *
 * \Description		Configures PWM Generator5 For ADC Triggering.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>TriggerCompareVal:</u>\n
 *	- a Value Between 1 and 65534 Inclusivly.\n
 * 	- DONT_CHANGE
 *
 * <u>TrigStartWaitCycles:</u>\n
 *	- WAIT_0_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	- WAIT_1_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	-            .				\n
 *	-            .				\n
 *	- WAIT_63_PWM_CYC_BEFORE_FIRST_TRIG	\n
 * 	- DONT_CHANGE
 *
 * <u>TrigOutputDiv:</u>\n
 *	- TRIGGER_ON_EVERY_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_2ND_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_3RD_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_4TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_5TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_6TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_7TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_8TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_9TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_10TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_11TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_12TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_13TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_14TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_15TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_16TH_TRIGGER_EVENT	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator5 ADC Triggering Function Configered Correctly.\n
 *		<b> 1: </b> Err1: TRIGx Value Is Bigger Than PTPER, (When Using Primary Time Base).\n
 *		<b> 2: </b> Err2: TRIGx Value Is Bigger Than STPER, (When Using Secondary Time Base).\n
 *		<b> 3: </b> Err3: TRIGx Value Is Bigger Than PHASEx, (When Using Independent Time Base Mode).
 *
 * \RelatedMacros	EnableIntPWM5Trig()		\n
 *			DisableIntPWM5Trig()		\n
 *			PWM5_TRIG_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- TriggerCompareVal Must Be Smaller Than PHASEx In Independent Time Base Mode.	\n
 *		2- TriggerCompareVal Must Be Smaller Than STPER When Using Secondary Time Base.	\n
 *		3- TriggerCompareVal Must Be Smaller Than PTPER When Using Primary Time Base.
 *
 * \Example     Err=SetupPWMGen5ADCTrigger(2400,WAIT_16_PWM_CYC_BEFORE_FIRST_TRIG,TRIGGER_ON_EVERY_TRIGGER_EVENT);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5ADCTrigger(TriggerCompareVal, TrigStartWaitCycles, TrigOutputDiv)({								\
																		\
	unsigned int _Return=0;															\
	if(TriggerCompareVal != DONT_CHANGE) {													\
		switch (PWMCON5bits.ITB){													\
			case 1:															\
				if(TriggerCompareVal<PHASE5) {TRIG5 = TriggerCompareVal;}							\
				else {_Return=3;/*Set Error "TRIGx Value Must Be Smaller than PHASEx In Independent Time Base Mode"*/}		\
				break;														\
			case 0:															\
				if(PWMCON5bits.MTBS){												\
					if(TriggerCompareVal<STPER) {TRIG5 = TriggerCompareVal;}						\
					else {_Return=2;/*Set Error "TRIGx Value Must Be Smaller Than STPER When Using Secondary Time Base"*/}	\
				}else{														\
					if(TriggerCompareVal<PTPER) {TRIG5 = TriggerCompareVal;}						\
					else {_Return=1;/*Set Error "TRIGx Value Must Be Smaller Than PTPER When Using Primary Time Base"*/}	\
				}														\
		}																\
	}																	\
	if(TrigStartWaitCycles	!= DONT_CHANGE) {TRGCON5bits.TRGSTRT = TrigStartWaitCycles&0b111111;}						\
	if(TrigOutputDiv	!= DONT_CHANGE) {TRGCON5bits.TRGDIV  = TrigOutputDiv&0b1111;}							\
	_Return;																\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen5Fault(IndependentFaultEN,FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator5 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>IndependentFaultEN:</u>\n
 *	- NORMAL_FAULT_MODE <i>/ Fault inputs map FLTDAT<1:0> to PWMxH and PWMxL and 
 *									 Current-limit map CLDAT<1:0> to PWMxH and PWMxL.</i>\n
 *	- INDEPENDENT_FAULT_MODE<i>/ Current-limit inputs map FLTDAT<1> to PWMxH output and 
 *								   Fault input maps FLTDAT<0> to PWMxL output.
 *								   The CLDAT<1:0> bits are not used for override functions.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>FaultSource:</u> <i>/Refer to Device Datasheet For Avaliable Fault Sources.</i>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_FAULT8_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator5 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM5Fault()		\n
 *			DisableIntPWM5Fault()		\n
 *			PWM5_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen5Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(IndependentFaultEN	!= DONT_CHANGE) {FCLCON5bits.IFLTMOD = IndependentFaultEN&0b1;}				\
	if(FaultSource		!= DONT_CHANGE) {FCLCON5bits.FLTSRC = FaultSource&0b11111;}				\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM5 = FaultMode&0b1;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON5bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON5bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON5bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON5bits.FLTDAT1 = FaultData_H;									\
		IOCON5bits.FLTDAT0 = FaultData_L;									\
		FCLCON5bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON5bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON5bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON5bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen5Fault(FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator5 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 *
 * <u>FaultSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator5 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM5Fault()		\n
 *			DisableIntPWM5Fault()		\n
 *			PWM5_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen5Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(FaultSource		!= DONT_CHANGE) {FCLCON5bits.FLTSRC = FaultSource&0b1;}					\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM5 = FaultMode&0b11111;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON5bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON5bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON5bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON5bits.FLTDAT1 = FaultData_H;									\
		IOCON5bits.FLTDAT0 = FaultData_L;									\
		FCLCON5bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON5bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON5bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON5bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen5CurrentLimit(CurrentLimSource,CurrentLimData_H,CurrentLimData_L,CurrentLimInputActiveState)
 *
 * \Description		Configures PWM Generator5 Current Limit Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>CurrentLimSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimInputActiveState:</u>\n
 *	- CURRENT_LIM_INPUT_ACTIVE_HIGH	\n
 *	- CURRENT_LIM_INPUT_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator5 Current Limit Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Current Limit Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Use Current Limit Data High/Low in Independent Fault Mode.
 *
 * \RelatedMacros	EnablePWM5CurrentLim()		\n
 *			DisablePWM5CurrentLim()		\n
 *			EnableIntPWM5CLim()		\n
 *			DisableIntPWM5CLim()		\n
 *			PWM5_CUR_LIM_INTERRUPT_IS_PENDING <i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- CurrentLimInputActiveState should not be changed after the PWM is enabled (PTEN = 1).\n
 *		2- Current Limit Data High/Low is Not used in Independent Fault Mode.\n
 *		3- Refer to Device Datasheet For Avaliable Current Limit Sources.
 *
 * \Example    Err=SetupPWMGen5CurrentLimit(SOURCE_IS_COMPARATOR2,SET_H_PIN_LOW,SET_L_PIN_LOW,CURRENT_LIM_INPUT_ACTIVE_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5CurrentLimit(CurrentLimSource, CurrentLimData_H, CurrentLimData_L, CurrentLimInputActiveState)({			\
																	\
	unsigned int _Return=0;														\
	if(CurrentLimData_H != DONT_CHANGE) {												\
		IOCON5bits.CLDAT1 = CurrentLimData_H&0b1;										\
		if(FCLCON5bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																	\
	if(CurrentLimData_L != DONT_CHANGE) {												\
		IOCON5bits.CLDAT0 = CurrentLimData_L&0b1;										\
		if(FCLCON5bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																 	\
	if(CurrentLimSource != DONT_CHANGE) {FCLCON5bits.CLSRC = CurrentLimSource&0b11111;}						\
	if(CurrentLimInputActiveState != DONT_CHANGE) {											\
		if(PTCONbits.PTEN == 0){												\
			FCLCON5bits.CLPOL = CurrentLimInputActiveState&0b1;								\
		} else {														\
			if (CurrentLimInputActiveState != FCLCON5bits.CLPOL) {								\
				_Return=1; /*Set Error "PWM Current Limit Polarity Cant be Changed When The PWM Module is Enabled"*/	\
			}														\
		}															\
	}																\
	_Return;															\
})

/***********************************************************************************************************************
 * \Function	void SetupPWMGen5LEB(PWMH_RisingEdgeBlankingEN,PWMH_FallingEdgeBlankingEN,PWML_RisingEdgeBlankingEN,PWML_FallingEdgeBlankingEN,LEB_Delay)
 *
 * \Description		Configures PWM Generator5 Leading Edge Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_RISING_EDGE_BLANK \n
 *	- ENABLE_PWMH_RISING_EDGE_BLANK	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- DONT_CHANGE
 *
 * <u>PWML_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_RISING_EDGE_BLANK \n
 *	- ENABLE_PWML_RISING_EDGE_BLANK	\n
 * 	- DONT_CHANGE
 *
 * <u>PWML_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWML_FALLING_EDGE_BLANK \n
 * 	- DONT_CHANGE
 *
 * <u>LEB_Delay:</u>\n
 *	- a Value Between 1 and 4096 Inclusivly. (LEB Delay = (0 to 4096) * PWM Clock Freq) \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM5FaultBlanking()	\n
 *			EnablePWM5CurrentLimBlanking()	\n
 *			DisablePWM5FaultBlanking()	\n
 *			DisablePWM5CurrentLimBlanking()
 *
 * \Notes		LEB Delay = (0 to 4096) * PWM Clock Freq.
 *
 * \Example
 * SetupPWMGen5LEB(ENABLE_PWMH_RISING_EDGE_BLANK,ENABLE_PWMH_FALLING_EDGE_BLANK,ENABLE_PWML_RISING_EDGE_BLANK,ENABLE_PWML_FALLING_EDGE_BLANK,990);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5LEB(PWMH_RisingEdgeBlankingEN, PWMH_FallingEdgeBlankingEN,  PWML_RisingEdgeBlankingEN, PWML_FallingEdgeBlankingEN, LEB_Delay)({	\
																			\
	if(PWMH_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON5bits.PHR = PWMH_RisingEdgeBlankingEN&0b1;}						\
	if(PWMH_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON5bits.PHF = PWMH_FallingEdgeBlankingEN&0b1;}						\
	if(PWML_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON5bits.PLR = PWML_RisingEdgeBlankingEN&0b1;}						\
	if(PWML_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON5bits.PLF = PWML_FallingEdgeBlankingEN&0b1;}						\
	if(LEB_Delay			!= DONT_CHANGE) {LEBDLY5 = LEB_Delay&0b111111111111;}								\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen5StateBlanking(PWMH_BlankingState,PWML_BlankingState,SelectedSignalForStateBlank,SelectedSignalBlankingState)
 *
 * \Description		Configures PWM Generator5 State Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWMH_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_LOW	\n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>PWML_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWML_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>SelectedSignalForStateBlank:</u>\n
 *	- NO_SIGNAL_SOURCE_FOR_STATE_BLNKING \n
 *	- PWM5H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM2H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM3H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM4H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM5H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM6H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM7H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 * 	- DONT_CHANGE
 *
 * <u>SelectedSignalBlankingState:</u>\n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM5FaultBlanking()	\n
 *			EnablePWM5CurrentLimBlanking()	\n
 *			DisablePWM5FaultBlanking()	\n
 *			DisablePWM5CurrentLimBlanking()
 *
 * \Notes		None.
 *
 * \Example	SetupPWMGen5StateBlanking(ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH,ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH,
 *					  PWM5H_AS_STATE_BLANK_SIGNAL_SOURCE,ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5StateBlanking(PWMH_BlankingState, PWML_BlankingState, SelectedSignalForStateBlank, SelectedSignalBlankingState)({			\
																			\
	if(PWMH_BlankingState		!= DONT_CHANGE) {LEBCON5bits.BPHH = (PWMH_BlankingState&0b11)>>1; LEBCON5bits.BPHL = PWMH_BlankingState&0b1;}	\
	if(PWML_BlankingState		!= DONT_CHANGE) {LEBCON5bits.BPLH = (PWML_BlankingState&0b11)>>1; LEBCON5bits.BPLL = PWML_BlankingState&0b1;}	\
	if(SelectedSignalBlankingState	!= DONT_CHANGE) {LEBCON5bits.BCH  = (SelectedSignalBlankingState&0b11)>>1;					\
							 LEBCON5bits.BCL  =  SelectedSignalBlankingState&0b1;}						\
	if(SelectedSignalForStateBlank	!= DONT_CHANGE) {AUXCON5bits.BLANKSEL = SelectedSignalForStateBlank&0b111;}					\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen5Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)
 *
 * \Description		Configures PWM Generator5 Chop Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>ChopClkSource:</u> <i>/Refer to Device Datasheet For Avaliable Chop Clock Sources.</i>\n
 *	- CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE \n
 *	- PWM5H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM2H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM3H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM4H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM5H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM6H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM7H_AS_CHOP_CLOCK_SOURCE	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_ChopEN:</u>\n
 *	- DISABLE_PWMH_CHOP \n
 *	- ENABLE_PWMH_CHOP \n
 *	- DONT_CHANGE
 *
 * <u>PWML_ChopEN:</u>\n
 *	- DISABLE_PWML_CHOP \n
 *	- ENABLE_PWML_CHOP \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		Refer to Device Datasheet For Avaliable Chop Clock Sources.
 *
 * \Example	SetupPWMGen5Chop(CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE, ENABLE_PWMH_CHOP, ENABLE_PWML_CHOP);
 *
 **********************************************************************************************************************/
#define SetupPWMGen5Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)({				\
												\
	if(ChopClkSource	!= DONT_CHANGE) {AUXCON5bits.CHOPSEL = ChopClkSource&0b111;}	\
	if(PWMH_ChopEN		!= DONT_CHANGE) {AUXCON5bits.CHOPHEN = PWMH_ChopEN&0b1;}	\
	if(PWML_ChopEN		!= DONT_CHANGE) {AUXCON5bits.CHOPLEN = PWML_ChopEN&0b1;}	\
})
/***/

// <editor-fold defaultstate="collapsed" desc="PWM5 Macros and Defines">

#ifdef PWMCAP5
/******************************************************************************************************
 * \brief	The value Returned From this Routine represents the captured PWM time base value when
 *		a leading edge is detected on the current-limit input.
 *
 * \Notes	1: The capture feature is only available on the primary output (PWMxH).
 *		2: The feature is only active after LEB processing on the current-limit input signal is complete.
 *
 *****************************************************************************************************/
#define CaptuerPWM5PriTB_Val()			PWMCAP5
#endif
/***/
#define EnablePWM5Fault()			FCLCON5bits.FLTMOD = FaultModePWM5
#define DisablePWM5Fault()			FCLCON5bits.FLTMOD = 0b11

#define EnablePWM5CurrentLim()			FCLCON5bits.CLMOD = 1
#define DisablePWM5CurrentLim()			FCLCON5bits.CLMOD = 0

#define EnablePWM5FaultBlanking()		LEBCON5bits.FLTLEBEN = 1
#define DisablePWM5FaultBlanking()		LEBCON5bits.FLTLEBEN = 0

#define EnablePWM5CurrentLimBlanking()		LEBCON5bits.CLLEBEN = 1
#define DisablePWM5CurrentLimBlanking()		LEBCON5bits.CLLEBEN = 0

#define SetDutyCyclePWM5(DC)			(PDC5=DC)	/*For Independent PWM DCMode With Complementary PWM Output Mode.*/
#ifdef STPER /* Devices That Have Secondary Time Base Will Have True Independent Duty Cycle Mode Option.*/
#define SetDutyCyclePWM5H(DC)			(PDC5=DC)	/*For Independent PWM DCMode*/
#define SetDutyCyclePWM5L(DC)			(SDC5=DC)	/*For Independent PWM DCMode*/
#endif
#define SetMasterDutyCycle(DC)			(MDC=DC)	/*When Using Master DCMode.*/

/** PWM Geneator1 Interrupt Handling */
#define EnableIntPWM5()				_PWM5IE = 1
#define DisableIntPWM5()			_PWM5IE = 0
#define SetIntPriorityPWM5(Priority)		_PWM5IP = (Priority)
#define PWM5_INT_FLAG				_PWM5IF

#define EnableIntPWM5Trig()			PWMCON5bits.TRGIEN = 1
#define DisableIntPWM5Trig()			PWMCON5bits.TRGIEN = 0
#define PWM5_TRIG_INTERRUPT_IS_PENDING		PWMCON5bits.TRGSTAT

#define	EnableIntPWM5Fault()			PWMCON5bits.FLTIEN = 1
#define DisableIntPWM5Fault()			PWMCON5bits.FLTIEN = 0
#define PWM5_FAULT_INTERRUPT_IS_PENDING		PWMCON5bits.FLTSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

#define	EnableIntPWM5CLim()			PWMCON5bits.CLIEN = 1
#define	DisableIntPWM5CLim()			PWMCON5bits.CLIEN = 0
#define PWM5_CUR_LIM_INTERRUPT_IS_PENDING	PWMCON5bits.CLSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

// </editor-fold>

#endif

#ifdef _PWM6IF

#ifdef STPER
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen6(PWM_Mode,Aligment,TimeBaseSelect,Period_H,Period_L,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator6.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 *	- TRUE_INDEPENDENT_MODE	<i>/True Independent Output Mode is Not Avaliable On All Devices.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- SECONDARY_TIME_BASE	<i>/Secondary Time Base is Not Avaliable On All Devices.</i> \n
 *	- INDEPENDENT_TIME_BASE	\n
 * 	- DONT_CHANGE
 *
 * <u>Period_H:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>Period_L:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator6 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).\n
 *		<b> 3: </b> Err3: An Attempt to use secondary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM6(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetDutyCyclePWM6H(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetDutyCyclePWM6L(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen6(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6(_PWM_Mode, _Aligment, _TimeBaseSelect, Period_H, Period_L, _DCMode, _ENimmediatDCupdate)({					\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON6bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON6bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON6bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON6bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON6bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON6bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON6bits.PMOD = PWM_Mode;	/* 3 = PWM I/O pin pair is in True Independent PWM Output mode*/				\
						/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON6bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON6bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON6bits.CAM)	PWMCON6bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case SECONDARY_TIME_BASE:	/* 1 = PWM generator uses the secondary master time base FixedSecPeriod*/		\
				if(!PWMCON6bits.CAM)	PWMCON6bits.MTBS = 1;									\
				else			_Return=3; /* Set Error "We cant use secondary time base in center aligne mode"*/	\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx/SPHASEx registers provide time base period for this PWM generator*/	\
				PWMCON6bits.ITB  = 1;												\
				PHASE6 = Period_H;	/* Independent Period For PWM6H*/							\
				SPHASE6 = Period_L;	/* Independent Period For PWM6L*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON6bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
				/*0 = (Independent Duty Cycle) PDCx and SDCx registers provide duty cycle information for this PWM generator*/	\
		PWMCON6bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen6(PWM_Mode,Aligment,TimeBaseSelect,Period,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator6.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- INDEPENDENT_TIME_BASE	<i>/PHASEx register provides time base period for this PWM generator.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Period:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator6 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM6(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen6(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6(_PWM_Mode, _Aligment, _TimeBaseSelect, Period, _DCMode, _ENimmediatDCupdate)({							\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON6bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON6bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON6bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON6bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON6bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON6bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON6bits.PMOD = PWM_Mode;	/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON6bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON6bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON6bits.CAM)	PWMCON6bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx register provide time base period for this PWM generator*/		\
				PWMCON6bits.ITB  = 1;												\
				PHASE6 = Period;	/* Independent Period For PWM6*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON6bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
					/*0 = (Independent Duty Cycle) PDCx register provide duty cycle information for this PWM generator*/	\
		PWMCON6bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#endif

/***********************************************************************************************************************
 * \Function		void SetupPWMGen6Pins(H_PinOwnership,L_PinOwnership,H_PinPolarity,L_PinPolarity,EN_PinSwap)
 *
 * \Description		Configures PWM Generator6 Output Pins.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_H_PIN	\n
 *	- PWM_MODULE_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_L_PIN	\n
 *	- PWM_MODULE_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>H_PinPolarity:</u>	\n
 *	- H_PIN_ACTIVE_HIGH	\n
 *	- H_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>L_PinPolarity:</u>	\n
 *	- L_PIN_ACTIVE_HIGH	\n
 *	- L_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>EN_PinSwap:</u>		\n
 *	- H_L_PINS_NOT_SWAPED	\n
 *	- H_L_PINS_SWAPED	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen6Pins(PWM_MODULE_CONTROL_H_PIN,PWM_MODULE_CONTROL_L_PIN,H_PIN_ACTIVE_HIGH,L_PIN_ACTIVE_HIGH,H_L_PINS_NOT_SWAPED);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6Pins(H_PinOwnership, L_PinOwnership, H_PinPolarity, L_PinPolarity, EN_PinSwap)({	\
	if(H_PinOwnership != DONT_CHANGE) {IOCON6bits.PENH = H_PinOwnership&0b1;}			\
	if(L_PinOwnership != DONT_CHANGE) {IOCON6bits.PENL = L_PinOwnership&0b1;}			\
	if(H_PinPolarity  != DONT_CHANGE) {IOCON6bits.POLH = H_PinPolarity&0b1;}			\
	if(L_PinPolarity  != DONT_CHANGE) {IOCON6bits.POLL = L_PinPolarity&0b1;}			\
	if(EN_PinSwap     != DONT_CHANGE) {IOCON6bits.SWAP = EN_PinSwap&0b1;}				\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen6Override(H_PinOverEN,L_PinOverEN,OverDataForH_Pin,OverDataForL_Pin,SyncOverride)
 *
 * \Description		Configures And Set PWM Generator6 Pins Override.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_H_PIN	\n
 *	- OVR_DAT_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_L_PIN	\n
 *	- OVR_DAT_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>OverDataForH_Pin:</u>	\n
 *	- OVERRIDE_H_PIN_LOW	\n
 *	- OVERRIDE_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>OverDataForL_Pin:</u>	\n
 *	- OVERRIDE_L_PIN_LOW	\n
 *	- OVERRIDE_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>SyncOverride:</u>		\n
 *	- DISABLE_OVERRIDE_SYNC	\n
 *	- ENABLE_OVERRIDE_SYNC	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen6Override(PWM_GEN_CONTROL_H_PIN, PWM_GEN_CONTROL_L_PIN, DONT_CHANGE, DONT_CHANGE, DONT_CHANGE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6Override(H_PinOverEN, L_PinOverEN, OverDataForH_Pin, OverDataForL_Pin, SyncOverride)({	\
	if(H_PinOverEN		!= DONT_CHANGE) {IOCON6bits.OVRENH  = H_PinOverEN&0b1;}				\
	if(L_PinOverEN		!= DONT_CHANGE) {IOCON6bits.OVRENL  = L_PinOverEN&0b1;}				\
	if(OverDataForL_Pin	!= DONT_CHANGE) {IOCON6bits.OVRDAT0 = OverDataForL_Pin&0b1;}			\
	if(OverDataForH_Pin	!= DONT_CHANGE) {IOCON6bits.OVRDAT1 = OverDataForH_Pin&0b1;}			\
	if(SyncOverride		!= DONT_CHANGE) {IOCON6bits.OSYNC   = SyncOverride&0b1;}			\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen6PhaseShifting(PhShift_H, PhShift_L)
 *
 * \Description		Set PWM Generator6 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift_H:</u> <i>/For Complementary And Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * <u>PhShift_L:</u> <i>/ONLY For Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator6 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen6PhaseShifting(6000, 12000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6PhaseShifting(PhShift_H, PhShift_L)({							\
														\
	unsigned int _Return=0;											\
	if(PWMCON6bits.CAM == EDGE_ALIGNED){									\
		if(PhShift_H != DONT_CHANGE) {PHASE6 = PhShift_H;}						\
		if(PhShift_L != DONT_CHANGE) {SPHASE6 = PhShift_L;}						\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen6PhaseShifting(PhShift)
 *
 * \Description		Set PWM Generator6 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift:</u> \n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator6 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen6PhaseShifting(6000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6PhaseShifting(PhShift)({									\
														\
	unsigned int _Return=0;											\
	if(PWMCON6bits.CAM == EDGE_ALIGNED){									\
		if(PhShift != DONT_CHANGE) {PHASE6 = PhShift;}							\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen6DeadTime(DeadTimeMode,DeadTime,AlternatDeadTime)
 *
 * \Description		Configures And Set PWM Generator6 Dead Time.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>DeadTimeMode:</u>	\n
 *	- POSITIVE_DT_MODE			\n
 *	- NEGATIVE_DT_MODE			<i>/Not allowed in Center_Align mode.</i>\n
 *	- DISABLE_DT_MODE			\n
 *	- NEGATIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxH is shortened and PWMxL is lengthened And
 *							If DTCMPx Pin = 1, PWMxL is shortened and PWMxH is lengthened.</i>\n
 *	- POSITIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxL is shortened and PWMxH is lengthened And
 *							If DTCMPx Pin = 1, PWMxH is shortened and PWMxL is lengthened.</i>\n
 *	- DONT_CHANGE
 *
 * <u>DeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Is Not Used For Dead Time Value In CENTER_ALIGNED Output Mode So Use AlternatDeadTime Param For Both PWMxH & PWMxL Outputs.\n
 *	- This Parameter Holds The Value To Be Added to Or Subtracted From The Duty Cycle in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE.
 *
 * <u>AlternatDeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in CENTER_ALIGNED PWM Output Mode.\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE DeadTimeMode.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator6 Dead Time Parameters Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Dead time Compensation in None Complementary PWM output mode with Positive Dead Time.\n
 *		<b> 2: </b> Err2: An Attempt to Use Negative Dead Time in None Complementary Edge Aligned PWM output mode.\n
 *		<b> 3: </b> Warn1: An Attempt to Use Primary Dead Time (DeadTime) in Center Align Mode With Positive dead time, (You May Keep it Zero).
 *
 * \RelatedMacros	None.
 *
 * \Notes	1- Open "Dead Time Configrations.png" Chart in "Important Files" Folder for Comprehensive Understanding of Dead Time Module Operation.\n
 *		2- Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode.			\n
 *		3- Primary Dead Time (DeadTime) is not used in Center Align Mode So Keep it Zero.						\n
 *		4- Negative Dead time is available only for Complementary Edge Aligned PWM output mode.
 *
 * \Example     Err=SetupPWMGen6DeadTime(POSITIVE_DT_MODE,1000,1200);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6DeadTime(_DeadTimeMode, DeadTime, AlternatDeadTime)({										\
																			\
	unsigned int _Return=0;																\
	unsigned int DeadTimeMode=_DeadTimeMode;													\
	if(DeadTimeMode == DONT_CHANGE) {														\
		DeadTimeMode=PWMCON6bits.DTC;														\
		if( (DeadTimeMode==0b11) && PWMCON6bits.DTCP)												\
			DeadTimeMode=POSITIVE_COMPENSATION_DT_MODE;											\
	}																		\
	switch (DeadTimeMode){																\
		case POSITIVE_COMPENSATION_DT_MODE :													\
			if(IOCON6bits.PMOD) { /* check if Complementary PWM output mode is Used But if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON6bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON6bits.DTCP= 1;		/* Dead-Time Compensation Polarity is Positive */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers */\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case NEGATIVE_COMPENSATION_DT_MODE :													\
			if(IOCON6bits.PMOD) { /* check if Complementary PWM output mode is Used And if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON6bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON6bits.DTCP= 0;		/* Dead-Time Compensation Polarity is Negative */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers*/	\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case POSITIVE_DT_MODE :															\
			PWMCON6bits.DTC = 0;		/* Positive dead time actively applied for all output modes */					\
			if((PWMCON6bits.CAM)&&(DeadTime)) {												\
				/*Set Warning "Primary Dead Time (DTRx) is not used in Center Align Mode With Positive dead time So Keep it Zero!"*/	\
				_Return=3;														\
			}																\
			/* DeadTime Holds The Value of Dead Time for PWMxH output signal */								\
			/* AlternatDeadTime Holds The Dead Time Value for PWMxL output signal OR for both the PWMxH and PWMxL in Center Align mode */	\
			break;																\
		case NEGATIVE_DT_MODE :															\
			if ((IOCON6bits.PMOD == COMPLEMENTARY_MODE) && (PWMCON6bits.CAM == EDGE_ALIGNED)) {						\
				PWMCON6bits.DTC = 1;	/* Negative dead time actively applied for Complementary Output mode */				\
				/* DeadTime Holds The Value of Dead Time for PWMxL output signal */							\
				/* AlternatDeadTime Holds The Dead Time Value for PWMxH output signal */						\
			}else{																\
				/*Set Error "Negative Dead time is available only for Complementary Edge Aligned PWM output mode"*/			\
				_Return=2;														\
			}																\
			break;																\
		case DISABLE_DT_MODE :															\
			PWMCON6bits.DTC = 0;														\
	}																		\
	if(DeadTime		!= DONT_CHANGE) {DTR6	= DeadTime&0b11111111111111;}									\
	if(AlternatDeadTime	!= DONT_CHANGE) {ALTDTR6 = AlternatDeadTime&0b11111111111111;}								\
	_Return;																	\
})

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen6ADCTrigger(TriggerCompareVal,TrigStartWaitCycles,TrigOutputDiv)
 *
 * \Description		Configures PWM Generator6 For ADC Triggering.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>TriggerCompareVal:</u>\n
 *	- a Value Between 1 and 65534 Inclusivly.\n
 * 	- DONT_CHANGE
 *
 * <u>TrigStartWaitCycles:</u>\n
 *	- WAIT_0_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	- WAIT_1_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	-            .				\n
 *	-            .				\n
 *	- WAIT_63_PWM_CYC_BEFORE_FIRST_TRIG	\n
 * 	- DONT_CHANGE
 *
 * <u>TrigOutputDiv:</u>\n
 *	- TRIGGER_ON_EVERY_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_2ND_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_3RD_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_4TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_5TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_6TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_7TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_8TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_9TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_10TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_11TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_12TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_13TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_14TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_15TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_16TH_TRIGGER_EVENT	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator6 ADC Triggering Function Configered Correctly.\n
 *		<b> 1: </b> Err1: TRIGx Value Is Bigger Than PTPER, (When Using Primary Time Base).\n
 *		<b> 2: </b> Err2: TRIGx Value Is Bigger Than STPER, (When Using Secondary Time Base).\n
 *		<b> 3: </b> Err3: TRIGx Value Is Bigger Than PHASEx, (When Using Independent Time Base Mode).
 *
 * \RelatedMacros	EnableIntPWM6Trig()		\n
 *			DisableIntPWM6Trig()		\n
 *			PWM6_TRIG_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- TriggerCompareVal Must Be Smaller Than PHASEx In Independent Time Base Mode.	\n
 *		2- TriggerCompareVal Must Be Smaller Than STPER When Using Secondary Time Base.	\n
 *		3- TriggerCompareVal Must Be Smaller Than PTPER When Using Primary Time Base.
 *
 * \Example     Err=SetupPWMGen6ADCTrigger(2400,WAIT_16_PWM_CYC_BEFORE_FIRST_TRIG,TRIGGER_ON_EVERY_TRIGGER_EVENT);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6ADCTrigger(TriggerCompareVal, TrigStartWaitCycles, TrigOutputDiv)({								\
																		\
	unsigned int _Return=0;															\
	if(TriggerCompareVal != DONT_CHANGE) {													\
		switch (PWMCON6bits.ITB){													\
			case 1:															\
				if(TriggerCompareVal<PHASE6) {TRIG6 = TriggerCompareVal;}							\
				else {_Return=3;/*Set Error "TRIGx Value Must Be Smaller than PHASEx In Independent Time Base Mode"*/}		\
				break;														\
			case 0:															\
				if(PWMCON6bits.MTBS){												\
					if(TriggerCompareVal<STPER) {TRIG6 = TriggerCompareVal;}						\
					else {_Return=2;/*Set Error "TRIGx Value Must Be Smaller Than STPER When Using Secondary Time Base"*/}	\
				}else{														\
					if(TriggerCompareVal<PTPER) {TRIG6 = TriggerCompareVal;}						\
					else {_Return=1;/*Set Error "TRIGx Value Must Be Smaller Than PTPER When Using Primary Time Base"*/}	\
				}														\
		}																\
	}																	\
	if(TrigStartWaitCycles	!= DONT_CHANGE) {TRGCON6bits.TRGSTRT = TrigStartWaitCycles&0b111111;}						\
	if(TrigOutputDiv	!= DONT_CHANGE) {TRGCON6bits.TRGDIV  = TrigOutputDiv&0b1111;}							\
	_Return;																\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen6Fault(IndependentFaultEN,FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator6 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>IndependentFaultEN:</u>\n
 *	- NORMAL_FAULT_MODE <i>/ Fault inputs map FLTDAT<1:0> to PWMxH and PWMxL and 
 *									 Current-limit map CLDAT<1:0> to PWMxH and PWMxL.</i>\n
 *	- INDEPENDENT_FAULT_MODE<i>/ Current-limit inputs map FLTDAT<1> to PWMxH output and 
 *								   Fault input maps FLTDAT<0> to PWMxL output.
 *								   The CLDAT<1:0> bits are not used for override functions.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>FaultSource:</u> <i>/Refer to Device Datasheet For Avaliable Fault Sources.</i>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_FAULT8_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator6 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM6Fault()		\n
 *			DisableIntPWM6Fault()		\n
 *			PWM6_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen6Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(IndependentFaultEN	!= DONT_CHANGE) {FCLCON6bits.IFLTMOD = IndependentFaultEN&0b1;}				\
	if(FaultSource		!= DONT_CHANGE) {FCLCON6bits.FLTSRC = FaultSource&0b11111;}				\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM6 = FaultMode&0b1;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON6bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON6bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON6bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON6bits.FLTDAT1 = FaultData_H;									\
		IOCON6bits.FLTDAT0 = FaultData_L;									\
		FCLCON6bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON6bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON6bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON6bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen6Fault(FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator6 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 *
 * <u>FaultSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator6 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM6Fault()		\n
 *			DisableIntPWM6Fault()		\n
 *			PWM6_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen6Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(FaultSource		!= DONT_CHANGE) {FCLCON6bits.FLTSRC = FaultSource&0b1;}					\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM6 = FaultMode&0b11111;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON6bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON6bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON6bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON6bits.FLTDAT1 = FaultData_H;									\
		IOCON6bits.FLTDAT0 = FaultData_L;									\
		FCLCON6bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON6bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON6bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON6bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen6CurrentLimit(CurrentLimSource,CurrentLimData_H,CurrentLimData_L,CurrentLimInputActiveState)
 *
 * \Description		Configures PWM Generator6 Current Limit Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>CurrentLimSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimInputActiveState:</u>\n
 *	- CURRENT_LIM_INPUT_ACTIVE_HIGH	\n
 *	- CURRENT_LIM_INPUT_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator6 Current Limit Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Current Limit Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Use Current Limit Data High/Low in Independent Fault Mode.
 *
 * \RelatedMacros	EnablePWM6CurrentLim()		\n
 *			DisablePWM6CurrentLim()		\n
 *			EnableIntPWM6CLim()		\n
 *			DisableIntPWM6CLim()		\n
 *			PWM6_CUR_LIM_INTERRUPT_IS_PENDING <i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- CurrentLimInputActiveState should not be changed after the PWM is enabled (PTEN = 1).\n
 *		2- Current Limit Data High/Low is Not used in Independent Fault Mode.\n
 *		3- Refer to Device Datasheet For Avaliable Current Limit Sources.
 *
 * \Example    Err=SetupPWMGen6CurrentLimit(SOURCE_IS_COMPARATOR2,SET_H_PIN_LOW,SET_L_PIN_LOW,CURRENT_LIM_INPUT_ACTIVE_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6CurrentLimit(CurrentLimSource, CurrentLimData_H, CurrentLimData_L, CurrentLimInputActiveState)({			\
																	\
	unsigned int _Return=0;														\
	if(CurrentLimData_H != DONT_CHANGE) {												\
		IOCON6bits.CLDAT1 = CurrentLimData_H&0b1;										\
		if(FCLCON6bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																	\
	if(CurrentLimData_L != DONT_CHANGE) {												\
		IOCON6bits.CLDAT0 = CurrentLimData_L&0b1;										\
		if(FCLCON6bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																 	\
	if(CurrentLimSource != DONT_CHANGE) {FCLCON6bits.CLSRC = CurrentLimSource&0b11111;}						\
	if(CurrentLimInputActiveState != DONT_CHANGE) {											\
		if(PTCONbits.PTEN == 0){												\
			FCLCON6bits.CLPOL = CurrentLimInputActiveState&0b1;								\
		} else {														\
			if (CurrentLimInputActiveState != FCLCON6bits.CLPOL) {								\
				_Return=1; /*Set Error "PWM Current Limit Polarity Cant be Changed When The PWM Module is Enabled"*/	\
			}														\
		}															\
	}																\
	_Return;															\
})

/***********************************************************************************************************************
 * \Function	void SetupPWMGen6LEB(PWMH_RisingEdgeBlankingEN,PWMH_FallingEdgeBlankingEN,PWML_RisingEdgeBlankingEN,PWML_FallingEdgeBlankingEN,LEB_Delay)
 *
 * \Description		Configures PWM Generator6 Leading Edge Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_RISING_EDGE_BLANK \n
 *	- ENABLE_PWMH_RISING_EDGE_BLANK	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- DONT_CHANGE
 *
 * <u>PWML_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_RISING_EDGE_BLANK \n
 *	- ENABLE_PWML_RISING_EDGE_BLANK	\n
 * 	- DONT_CHANGE
 *
 * <u>PWML_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWML_FALLING_EDGE_BLANK \n
 * 	- DONT_CHANGE
 *
 * <u>LEB_Delay:</u>\n
 *	- a Value Between 1 and 4096 Inclusivly. (LEB Delay = (0 to 4096) * PWM Clock Freq) \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM6FaultBlanking()	\n
 *			EnablePWM6CurrentLimBlanking()	\n
 *			DisablePWM6FaultBlanking()	\n
 *			DisablePWM6CurrentLimBlanking()
 *
 * \Notes		LEB Delay = (0 to 4096) * PWM Clock Freq.
 *
 * \Example
 * SetupPWMGen6LEB(ENABLE_PWMH_RISING_EDGE_BLANK,ENABLE_PWMH_FALLING_EDGE_BLANK,ENABLE_PWML_RISING_EDGE_BLANK,ENABLE_PWML_FALLING_EDGE_BLANK,990);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6LEB(PWMH_RisingEdgeBlankingEN, PWMH_FallingEdgeBlankingEN,  PWML_RisingEdgeBlankingEN, PWML_FallingEdgeBlankingEN, LEB_Delay)({	\
																			\
	if(PWMH_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON6bits.PHR = PWMH_RisingEdgeBlankingEN&0b1;}						\
	if(PWMH_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON6bits.PHF = PWMH_FallingEdgeBlankingEN&0b1;}						\
	if(PWML_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON6bits.PLR = PWML_RisingEdgeBlankingEN&0b1;}						\
	if(PWML_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON6bits.PLF = PWML_FallingEdgeBlankingEN&0b1;}						\
	if(LEB_Delay			!= DONT_CHANGE) {LEBDLY6 = LEB_Delay&0b111111111111;}								\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen6StateBlanking(PWMH_BlankingState,PWML_BlankingState,SelectedSignalForStateBlank,SelectedSignalBlankingState)
 *
 * \Description		Configures PWM Generator6 State Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWMH_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_LOW	\n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>PWML_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWML_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>SelectedSignalForStateBlank:</u>\n
 *	- NO_SIGNAL_SOURCE_FOR_STATE_BLNKING \n
 *	- PWM6H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM2H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM3H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM4H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM5H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM6H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM7H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 * 	- DONT_CHANGE
 *
 * <u>SelectedSignalBlankingState:</u>\n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM6FaultBlanking()	\n
 *			EnablePWM6CurrentLimBlanking()	\n
 *			DisablePWM6FaultBlanking()	\n
 *			DisablePWM6CurrentLimBlanking()
 *
 * \Notes		None.
 *
 * \Example	SetupPWMGen6StateBlanking(ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH,ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH,
 *					  PWM6H_AS_STATE_BLANK_SIGNAL_SOURCE,ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6StateBlanking(PWMH_BlankingState, PWML_BlankingState, SelectedSignalForStateBlank, SelectedSignalBlankingState)({			\
																			\
	if(PWMH_BlankingState		!= DONT_CHANGE) {LEBCON6bits.BPHH = (PWMH_BlankingState&0b11)>>1; LEBCON6bits.BPHL = PWMH_BlankingState&0b1;}	\
	if(PWML_BlankingState		!= DONT_CHANGE) {LEBCON6bits.BPLH = (PWML_BlankingState&0b11)>>1; LEBCON6bits.BPLL = PWML_BlankingState&0b1;}	\
	if(SelectedSignalBlankingState	!= DONT_CHANGE) {LEBCON6bits.BCH  = (SelectedSignalBlankingState&0b11)>>1;					\
							 LEBCON6bits.BCL  =  SelectedSignalBlankingState&0b1;}						\
	if(SelectedSignalForStateBlank	!= DONT_CHANGE) {AUXCON6bits.BLANKSEL = SelectedSignalForStateBlank&0b111;}					\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen6Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)
 *
 * \Description		Configures PWM Generator6 Chop Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>ChopClkSource:</u> <i>/Refer to Device Datasheet For Avaliable Chop Clock Sources.</i>\n
 *	- CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE \n
 *	- PWM6H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM2H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM3H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM4H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM5H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM6H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM7H_AS_CHOP_CLOCK_SOURCE	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_ChopEN:</u>\n
 *	- DISABLE_PWMH_CHOP \n
 *	- ENABLE_PWMH_CHOP \n
 *	- DONT_CHANGE
 *
 * <u>PWML_ChopEN:</u>\n
 *	- DISABLE_PWML_CHOP \n
 *	- ENABLE_PWML_CHOP \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		Refer to Device Datasheet For Avaliable Chop Clock Sources.
 *
 * \Example	SetupPWMGen6Chop(CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE, ENABLE_PWMH_CHOP, ENABLE_PWML_CHOP);
 *
 **********************************************************************************************************************/
#define SetupPWMGen6Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)({				\
												\
	if(ChopClkSource	!= DONT_CHANGE) {AUXCON6bits.CHOPSEL = ChopClkSource&0b111;}	\
	if(PWMH_ChopEN		!= DONT_CHANGE) {AUXCON6bits.CHOPHEN = PWMH_ChopEN&0b1;}	\
	if(PWML_ChopEN		!= DONT_CHANGE) {AUXCON6bits.CHOPLEN = PWML_ChopEN&0b1;}	\
})
/***/

// <editor-fold defaultstate="collapsed" desc="PWM6 Macros and Defines">

#ifdef PWMCAP6
/******************************************************************************************************
 * \brief	The value Returned From this Routine represents the captured PWM time base value when
 *		a leading edge is detected on the current-limit input.
 *
 * \Notes	1: The capture feature is only available on the primary output (PWMxH).
 *		2: The feature is only active after LEB processing on the current-limit input signal is complete.
 *
 *****************************************************************************************************/
#define CaptuerPWM6PriTB_Val()			PWMCAP6
#endif
/***/
#define EnablePWM6Fault()			FCLCON6bits.FLTMOD = FaultModePWM6
#define DisablePWM6Fault()			FCLCON6bits.FLTMOD = 0b11

#define EnablePWM6CurrentLim()			FCLCON6bits.CLMOD = 1
#define DisablePWM6CurrentLim()			FCLCON6bits.CLMOD = 0

#define EnablePWM6FaultBlanking()		LEBCON6bits.FLTLEBEN = 1
#define DisablePWM6FaultBlanking()		LEBCON6bits.FLTLEBEN = 0

#define EnablePWM6CurrentLimBlanking()		LEBCON6bits.CLLEBEN = 1
#define DisablePWM6CurrentLimBlanking()		LEBCON6bits.CLLEBEN = 0

#define SetDutyCyclePWM6(DC)			(PDC6=DC)	/*For Independent PWM DCMode With Complementary PWM Output Mode.*/
#ifdef STPER /* Devices That Have Secondary Time Base Will Have True Independent Duty Cycle Mode Option.*/
#define SetDutyCyclePWM6H(DC)			(PDC6=DC)	/*For Independent PWM DCMode*/
#define SetDutyCyclePWM6L(DC)			(SDC6=DC)	/*For Independent PWM DCMode*/
#endif
#define SetMasterDutyCycle(DC)			(MDC=DC)	/*When Using Master DCMode.*/

/** PWM Geneator1 Interrupt Handling */
#define EnableIntPWM6()				_PWM6IE = 1
#define DisableIntPWM6()			_PWM6IE = 0
#define SetIntPriorityPWM6(Priority)		_PWM6IP = (Priority)
#define PWM6_INT_FLAG				_PWM6IF

#define EnableIntPWM6Trig()			PWMCON6bits.TRGIEN = 1
#define DisableIntPWM6Trig()			PWMCON6bits.TRGIEN = 0
#define PWM6_TRIG_INTERRUPT_IS_PENDING		PWMCON6bits.TRGSTAT

#define	EnableIntPWM6Fault()			PWMCON6bits.FLTIEN = 1
#define DisableIntPWM6Fault()			PWMCON6bits.FLTIEN = 0
#define PWM6_FAULT_INTERRUPT_IS_PENDING		PWMCON6bits.FLTSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

#define	EnableIntPWM6CLim()			PWMCON6bits.CLIEN = 1
#define	DisableIntPWM6CLim()			PWMCON6bits.CLIEN = 0
#define PWM6_CUR_LIM_INTERRUPT_IS_PENDING	PWMCON6bits.CLSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

// </editor-fold>

#endif

#ifdef _PWM7IF

#ifdef STPER
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen7(PWM_Mode,Aligment,TimeBaseSelect,Period_H,Period_L,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator7.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 *	- TRUE_INDEPENDENT_MODE	<i>/True Independent Output Mode is Not Avaliable On All Devices.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- SECONDARY_TIME_BASE	<i>/Secondary Time Base is Not Avaliable On All Devices.</i> \n
 *	- INDEPENDENT_TIME_BASE	\n
 * 	- DONT_CHANGE
 *
 * <u>Period_H:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>Period_L:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator7 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).\n
 *		<b> 3: </b> Err3: An Attempt to use secondary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM7(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetDutyCyclePWM7H(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetDutyCyclePWM7L(DC)	<i>/For Independent PWM DCMode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen7(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7(_PWM_Mode, _Aligment, _TimeBaseSelect, Period_H, Period_L, _DCMode, _ENimmediatDCupdate)({					\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON7bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON7bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON7bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON7bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON7bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON7bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON7bits.PMOD = PWM_Mode;	/* 3 = PWM I/O pin pair is in True Independent PWM Output mode*/				\
						/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON7bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON7bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON7bits.CAM)	PWMCON7bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case SECONDARY_TIME_BASE:	/* 1 = PWM generator uses the secondary master time base FixedSecPeriod*/		\
				if(!PWMCON7bits.CAM)	PWMCON7bits.MTBS = 1;									\
				else			_Return=3; /* Set Error "We cant use secondary time base in center aligne mode"*/	\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx/SPHASEx registers provide time base period for this PWM generator*/	\
				PWMCON7bits.ITB  = 1;												\
				PHASE7 = Period_H;	/* Independent Period For PWM7H*/							\
				SPHASE7 = Period_L;	/* Independent Period For PWM7L*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON7bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
				/*0 = (Independent Duty Cycle) PDCx and SDCx registers provide duty cycle information for this PWM generator*/	\
		PWMCON7bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen7(PWM_Mode,Aligment,TimeBaseSelect,Period,DCMode,ENimmediatDCupdate)
 *
 * \Description		Configures PWM Generator7.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWM_Mode:</u>		\n
 *	- COMPLEMENTARY_MODE	\n
 *	- REDUNDANT_MODE	\n
 *	- PUSHPULL_MODE		\n
 * 	- DONT_CHANGE
 *
 * <u>Aligment:</u>		\n
 *	- EDGE_ALIGNED		\n
 *	- CENTER_ALIGNED	<i>/Must Be Used With INDEPENDENT_DUTY_CYCLE.</i> \n
 * 	- DONT_CHANGE
 *
 * <u>TimeBaseSelect:</u>	\n
 *	- PRIMARY_TIME_BASE	\n
 *	- INDEPENDENT_TIME_BASE	<i>/PHASEx register provides time base period for this PWM generator.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>Period:</u> <i>/ Use For INDEPENDENT_TIME_BASE Mode ONLY, Else Keep it Zero.</i> \n
 *	- a Value Between 1 and 65535 Inclusivly.
 *
 * <u>DCMode:</u> \n
 *	- INDEPENDENT_DUTY_CYCLE	\n
 *	- MASTER_DUTY_CYCLE		\n
 * 	- DONT_CHANGE
 *
 * <u>ENimmediatDCupdate:</u> \n
 *	- DISABLE_IMMEDIAT_DC_UPDATE	\n
 *	- ENABLE_IMMEDIAT_DC_UPDATE	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator7 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Any Parameter while The PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to use pimary time base in center aligne mode,(INDEPENDENT_TIME_BASE Must Be Used).
 *
 * \RelatedMacros	SetDutyCyclePWM7(DC)	<i>/For Independent PWM DCMode With Complementary PWM Output Mode.</i>\n
 *			SetMasterDutyCycle(DC)	<i>/When Using Master DCMode.</i>
 *
 * \Notes       All Argements should not be changed after the PWM is enabled (PTEN = 1).
 *
 * \Example     Err=SetupPWMGen7(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7(_PWM_Mode, _Aligment, _TimeBaseSelect, Period, _DCMode, _ENimmediatDCupdate)({							\
																		\
	uint16_t _Return=0;															\
	uint16_t PWM_Mode=_PWM_Mode;														\
	uint16_t Aligment=_Aligment;														\
	uint16_t TimeBaseSelect=_TimeBaseSelect;												\
	uint16_t DCMode=_DCMode;														\
	uint16_t ENimmediatDCupdate=_ENimmediatDCupdate;											\
	if(PWM_Mode == DONT_CHANGE)		{PWM_Mode=IOCON7bits.PMOD;}									\
	if(Aligment == DONT_CHANGE)		{Aligment=PWMCON7bits.CAM;}									\
	if(TimeBaseSelect == DONT_CHANGE)	{if(PWMCON7bits.ITB) TimeBaseSelect=INDEPENDENT_TIME_BASE; else TimeBaseSelect=PWMCON7bits.MTBS;}\
	if(DCMode == DONT_CHANGE)		{DCMode=PWMCON7bits.MDCS;}									\
	if(ENimmediatDCupdate == DONT_CHANGE)	{ENimmediatDCupdate=PWMCON7bits.IUE;}								\
																		\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/				\
		IOCON7bits.PMOD = PWM_Mode;	/* 2 = PWM I/O pin pair is in Push-Pull Output mode*/						\
						/* 1 = PWM I/O pin pair is in Redundant Output mode*/						\
						/* 0 = PWM I/O pin pair is in Complementary Output mode*/					\
		 /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON7bits.CAM = Aligment ;	/* 1 = Center-Aligned mode is enabled*/								\
						/* 0 = Edge-Aligned mode is enabled*/								\
		PWMCON7bits.ITB = Aligment ;	/* Must be set for Center-Aligned mode*/							\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		switch (TimeBaseSelect) {													\
			case PRIMARY_TIME_BASE:		/* 0 = PWM generator uses the primary master time base (FixedPriPeriod)*/		\
				if(!PWMCON7bits.CAM)	PWMCON7bits.MTBS = 0;									\
				else			_Return=2; /* Set Error "We cant use pimary time base in center aligne mode"*/		\
				break;														\
			case INDEPENDENT_TIME_BASE:	/* 2 = PHASEx register provide time base period for this PWM generator*/		\
				PWMCON7bits.ITB  = 1;												\
				PHASE7 = Period;	/* Independent Period For PWM7*/							\
			}															\
		/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/					\
		PWMCON7bits.MDCS = DCMode ; /*1 = (Master Duty Cycle) MDC register provides duty cycle information for this PWM generator*/	\
					/*0 = (Independent Duty Cycle) PDCx register provide duty cycle information for this PWM generator*/	\
		PWMCON7bits.IUE = ENimmediatDCupdate;												\
	} else {																\
		_Return=1;	/*Set Error "All Argements should not be changed after the PWM is Enabled (PTEN = 1)."*/			\
	}																	\
	_Return;																\
})
#endif

/***********************************************************************************************************************
 * \Function		void SetupPWMGen7Pins(H_PinOwnership,L_PinOwnership,H_PinPolarity,L_PinPolarity,EN_PinSwap)
 *
 * \Description		Configures PWM Generator7 Output Pins.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_H_PIN	\n
 *	- PWM_MODULE_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOwnership:</u>		\n
 *	- GPIO_MODULE_CONTROL_L_PIN	\n
 *	- PWM_MODULE_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>H_PinPolarity:</u>	\n
 *	- H_PIN_ACTIVE_HIGH	\n
 *	- H_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>L_PinPolarity:</u>	\n
 *	- L_PIN_ACTIVE_HIGH	\n
 *	- L_PIN_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * <u>EN_PinSwap:</u>		\n
 *	- H_L_PINS_NOT_SWAPED	\n
 *	- H_L_PINS_SWAPED	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen7Pins(PWM_MODULE_CONTROL_H_PIN,PWM_MODULE_CONTROL_L_PIN,H_PIN_ACTIVE_HIGH,L_PIN_ACTIVE_HIGH,H_L_PINS_NOT_SWAPED);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7Pins(H_PinOwnership, L_PinOwnership, H_PinPolarity, L_PinPolarity, EN_PinSwap)({	\
	if(H_PinOwnership != DONT_CHANGE) {IOCON7bits.PENH = H_PinOwnership&0b1;}			\
	if(L_PinOwnership != DONT_CHANGE) {IOCON7bits.PENL = L_PinOwnership&0b1;}			\
	if(H_PinPolarity  != DONT_CHANGE) {IOCON7bits.POLH = H_PinPolarity&0b1;}			\
	if(L_PinPolarity  != DONT_CHANGE) {IOCON7bits.POLL = L_PinPolarity&0b1;}			\
	if(EN_PinSwap     != DONT_CHANGE) {IOCON7bits.SWAP = EN_PinSwap&0b1;}				\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen7Override(H_PinOverEN,L_PinOverEN,OverDataForH_Pin,OverDataForL_Pin,SyncOverride)
 *
 * \Description		Configures And Set PWM Generator7 Pins Override.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>H_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_H_PIN	\n
 *	- OVR_DAT_CONTROL_H_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>L_PinOverEN:</u>		\n
 *	- PWM_GEN_CONTROL_L_PIN	\n
 *	- OVR_DAT_CONTROL_L_PIN	\n
 *	- DONT_CHANGE
 *
 * <u>OverDataForH_Pin:</u>	\n
 *	- OVERRIDE_H_PIN_LOW	\n
 *	- OVERRIDE_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>OverDataForL_Pin:</u>	\n
 *	- OVERRIDE_L_PIN_LOW	\n
 *	- OVERRIDE_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>SyncOverride:</u>		\n
 *	- DISABLE_OVERRIDE_SYNC	\n
 *	- ENABLE_OVERRIDE_SYNC	\n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		None.
 *
 * \Example     SetupPWMGen7Override(PWM_GEN_CONTROL_H_PIN, PWM_GEN_CONTROL_L_PIN, DONT_CHANGE, DONT_CHANGE, DONT_CHANGE);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7Override(H_PinOverEN, L_PinOverEN, OverDataForH_Pin, OverDataForL_Pin, SyncOverride)({	\
	if(H_PinOverEN		!= DONT_CHANGE) {IOCON7bits.OVRENH  = H_PinOverEN&0b1;}				\
	if(L_PinOverEN		!= DONT_CHANGE) {IOCON7bits.OVRENL  = L_PinOverEN&0b1;}				\
	if(OverDataForL_Pin	!= DONT_CHANGE) {IOCON7bits.OVRDAT0 = OverDataForL_Pin&0b1;}			\
	if(OverDataForH_Pin	!= DONT_CHANGE) {IOCON7bits.OVRDAT1 = OverDataForH_Pin&0b1;}			\
	if(SyncOverride		!= DONT_CHANGE) {IOCON7bits.OSYNC   = SyncOverride&0b1;}			\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen7PhaseShifting(PhShift_H, PhShift_L)
 *
 * \Description		Set PWM Generator7 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift_H:</u> <i>/For Complementary And Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * <u>PhShift_L:</u> <i>/ONLY For Independent PWM Output Mode.</i>\n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator7 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen7PhaseShifting(6000, 12000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7PhaseShifting(PhShift_H, PhShift_L)({							\
														\
	unsigned int _Return=0;											\
	if(PWMCON7bits.CAM == EDGE_ALIGNED){									\
		if(PhShift_H != DONT_CHANGE) {PHASE7 = PhShift_H;}						\
		if(PhShift_L != DONT_CHANGE) {SPHASE7 = PhShift_L;}						\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen7PhaseShifting(PhShift)
 *
 * \Description		Set PWM Generator7 Phase Shifting.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PhShift:</u> \n
 *	- a Value Between 1 and 65534 Inclusivly.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator7 Has Been Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Phase Shifting In None Edge Aligned PWM output mode (it's Available only in Edge Aligned mode).
 *
 * \RelatedMacros	None.
 *
 * \Notes		Phase Shifting is available only in Edge Aligned PWM output mode.
 *
 * \Example     SetupPWMGen7PhaseShifting(6000);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7PhaseShifting(PhShift)({									\
														\
	unsigned int _Return=0;											\
	if(PWMCON7bits.CAM == EDGE_ALIGNED){									\
		if(PhShift != DONT_CHANGE) {PHASE7 = PhShift;}							\
	} else {												\
		_Return=1;/* Set Error "Phase Shifting is available only in Edge Aligned PWM output mode"*/	\
	}													\
	_Return;												\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen7DeadTime(DeadTimeMode,DeadTime,AlternatDeadTime)
 *
 * \Description		Configures And Set PWM Generator7 Dead Time.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>DeadTimeMode:</u>	\n
 *	- POSITIVE_DT_MODE			\n
 *	- NEGATIVE_DT_MODE			<i>/Not allowed in Center_Align mode.</i>\n
 *	- DISABLE_DT_MODE			\n
 *	- NEGATIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxH is shortened and PWMxL is lengthened And
 *							If DTCMPx Pin = 1, PWMxL is shortened and PWMxH is lengthened.</i>\n
 *	- POSITIVE_COMPENSATION_DT_MODE		<i>/	If DTCMPx Pin = 0, PWMxL is shortened and PWMxH is lengthened And
 *							If DTCMPx Pin = 1, PWMxH is shortened and PWMxL is lengthened.</i>\n
 *	- DONT_CHANGE
 *
 * <u>DeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Is Not Used For Dead Time Value In CENTER_ALIGNED Output Mode So Use AlternatDeadTime Param For Both PWMxH & PWMxL Outputs.\n
 *	- This Parameter Holds The Value To Be Added to Or Subtracted From The Duty Cycle in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE.
 *
 * <u>AlternatDeadTime:</u>\n
 *	- a Value Between 1 and 16384 Inclusivly, (1 LSb = 1 Tosc. For example, 7.14 ns for 70 MIPS operation).	\n
 * 	- DONT_CHANGE												\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in CENTER_ALIGNED PWM Output Mode.\n
 *	- This Parameter Holds The Value Of Dead Time For Both PWMxH & PWMxL in POSITIVE_COMPENSATION_DT_MODE/NEGATIVE_COMPENSATION_DT_MODE DeadTimeMode.
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator7 Dead Time Parameters Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Use Dead time Compensation in None Complementary PWM output mode with Positive Dead Time.\n
 *		<b> 2: </b> Err2: An Attempt to Use Negative Dead Time in None Complementary Edge Aligned PWM output mode.\n
 *		<b> 3: </b> Warn1: An Attempt to Use Primary Dead Time (DeadTime) in Center Align Mode With Positive dead time, (You May Keep it Zero).
 *
 * \RelatedMacros	None.
 *
 * \Notes	1- Open "Dead Time Configrations.png" Chart in "Important Files" Folder for Comprehensive Understanding of Dead Time Module Operation.\n
 *		2- Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode.			\n
 *		3- Primary Dead Time (DeadTime) is not used in Center Align Mode So Keep it Zero.						\n
 *		4- Negative Dead time is available only for Complementary Edge Aligned PWM output mode.
 *
 * \Example     Err=SetupPWMGen7DeadTime(POSITIVE_DT_MODE,1000,1200);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7DeadTime(_DeadTimeMode, DeadTime, AlternatDeadTime)({										\
																			\
	unsigned int _Return=0;																\
	unsigned int DeadTimeMode=_DeadTimeMode;													\
	if(DeadTimeMode == DONT_CHANGE) {														\
		DeadTimeMode=PWMCON7bits.DTC;														\
		if( (DeadTimeMode==0b11) && PWMCON7bits.DTCP)												\
			DeadTimeMode=POSITIVE_COMPENSATION_DT_MODE;											\
	}																		\
	switch (DeadTimeMode){																\
		case POSITIVE_COMPENSATION_DT_MODE :													\
			if(IOCON7bits.PMOD) { /* check if Complementary PWM output mode is Used But if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON7bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON7bits.DTCP= 1;		/* Dead-Time Compensation Polarity is Positive */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers */\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case NEGATIVE_COMPENSATION_DT_MODE :													\
			if(IOCON7bits.PMOD) { /* check if Complementary PWM output mode is Used And if Not Set Error */					\
				_Return=1;														\
				/*Set Error "Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode"*/	\
			}else{																\
				PWMCON7bits.DTC = 0b11;		/* Dead-Time Compensation mode */							\
				PWMCON7bits.DTCP= 0;		/* Dead-Time Compensation Polarity is Negative */					\
				/* DeadTime Holds The Value to be added to, or subtracted from, the duty cycle specified by the PDCx or MDC registers*/	\
				/* AlternatDeadTime Holds The Dead Time Value for both the PWMxH and PWMxL output signals */				\
			}																\
			break;																\
		case POSITIVE_DT_MODE :															\
			PWMCON7bits.DTC = 0;		/* Positive dead time actively applied for all output modes */					\
			if((PWMCON7bits.CAM)&&(DeadTime)) {												\
				/*Set Warning "Primary Dead Time (DTRx) is not used in Center Align Mode With Positive dead time So Keep it Zero!"*/	\
				_Return=3;														\
			}																\
			/* DeadTime Holds The Value of Dead Time for PWMxH output signal */								\
			/* AlternatDeadTime Holds The Dead Time Value for PWMxL output signal OR for both the PWMxH and PWMxL in Center Align mode */	\
			break;																\
		case NEGATIVE_DT_MODE :															\
			if ((IOCON7bits.PMOD == COMPLEMENTARY_MODE) && (PWMCON7bits.CAM == EDGE_ALIGNED)) {						\
				PWMCON7bits.DTC = 1;	/* Negative dead time actively applied for Complementary Output mode */				\
				/* DeadTime Holds The Value of Dead Time for PWMxL output signal */							\
				/* AlternatDeadTime Holds The Dead Time Value for PWMxH output signal */						\
			}else{																\
				/*Set Error "Negative Dead time is available only for Complementary Edge Aligned PWM output mode"*/			\
				_Return=2;														\
			}																\
			break;																\
		case DISABLE_DT_MODE :															\
			PWMCON7bits.DTC = 0;														\
	}																		\
	if(DeadTime		!= DONT_CHANGE) {DTR7	= DeadTime&0b11111111111111;}									\
	if(AlternatDeadTime	!= DONT_CHANGE) {ALTDTR7 = AlternatDeadTime&0b11111111111111;}								\
	_Return;																	\
})

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen7ADCTrigger(TriggerCompareVal,TrigStartWaitCycles,TrigOutputDiv)
 *
 * \Description		Configures PWM Generator7 For ADC Triggering.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>TriggerCompareVal:</u>\n
 *	- a Value Between 1 and 65534 Inclusivly.\n
 * 	- DONT_CHANGE
 *
 * <u>TrigStartWaitCycles:</u>\n
 *	- WAIT_0_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	- WAIT_1_PWM_CYC_BEFORE_FIRST_TRIG	\n
 *	-            .				\n
 *	-            .				\n
 *	- WAIT_63_PWM_CYC_BEFORE_FIRST_TRIG	\n
 * 	- DONT_CHANGE
 *
 * <u>TrigOutputDiv:</u>\n
 *	- TRIGGER_ON_EVERY_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_2ND_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_3RD_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_4TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_5TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_6TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_7TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_8TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_9TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_10TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_11TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_12TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_13TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_14TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_15TH_TRIGGER_EVENT	\n
 *	- TRIGGER_ON_EVERY_16TH_TRIGGER_EVENT	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator7 ADC Triggering Function Configered Correctly.\n
 *		<b> 1: </b> Err1: TRIGx Value Is Bigger Than PTPER, (When Using Primary Time Base).\n
 *		<b> 2: </b> Err2: TRIGx Value Is Bigger Than STPER, (When Using Secondary Time Base).\n
 *		<b> 3: </b> Err3: TRIGx Value Is Bigger Than PHASEx, (When Using Independent Time Base Mode).
 *
 * \RelatedMacros	EnableIntPWM7Trig()		\n
 *			DisableIntPWM7Trig()		\n
 *			PWM7_TRIG_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- TriggerCompareVal Must Be Smaller Than PHASEx In Independent Time Base Mode.	\n
 *		2- TriggerCompareVal Must Be Smaller Than STPER When Using Secondary Time Base.	\n
 *		3- TriggerCompareVal Must Be Smaller Than PTPER When Using Primary Time Base.
 *
 * \Example     Err=SetupPWMGen7ADCTrigger(2400,WAIT_16_PWM_CYC_BEFORE_FIRST_TRIG,TRIGGER_ON_EVERY_TRIGGER_EVENT);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7ADCTrigger(TriggerCompareVal, TrigStartWaitCycles, TrigOutputDiv)({								\
																		\
	unsigned int _Return=0;															\
	if(TriggerCompareVal != DONT_CHANGE) {													\
		switch (PWMCON7bits.ITB){													\
			case 1:															\
				if(TriggerCompareVal<PHASE7) {TRIG7 = TriggerCompareVal;}							\
				else {_Return=3;/*Set Error "TRIGx Value Must Be Smaller than PHASEx In Independent Time Base Mode"*/}		\
				break;														\
			case 0:															\
				if(PWMCON7bits.MTBS){												\
					if(TriggerCompareVal<STPER) {TRIG7 = TriggerCompareVal;}						\
					else {_Return=2;/*Set Error "TRIGx Value Must Be Smaller Than STPER When Using Secondary Time Base"*/}	\
				}else{														\
					if(TriggerCompareVal<PTPER) {TRIG7 = TriggerCompareVal;}						\
					else {_Return=1;/*Set Error "TRIGx Value Must Be Smaller Than PTPER When Using Primary Time Base"*/}	\
				}														\
		}																\
	}																	\
	if(TrigStartWaitCycles	!= DONT_CHANGE) {TRGCON7bits.TRGSTRT = TrigStartWaitCycles&0b111111;}						\
	if(TrigOutputDiv	!= DONT_CHANGE) {TRGCON7bits.TRGDIV  = TrigOutputDiv&0b1111;}							\
	_Return;																\
})

#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen7Fault(IndependentFaultEN,FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator7 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>IndependentFaultEN:</u>\n
 *	- NORMAL_FAULT_MODE <i>/ Fault inputs map FLTDAT<1:0> to PWMxH and PWMxL and 
 *									 Current-limit map CLDAT<1:0> to PWMxH and PWMxL.</i>\n
 *	- INDEPENDENT_FAULT_MODE<i>/ Current-limit inputs map FLTDAT<1> to PWMxH output and 
 *								   Fault input maps FLTDAT<0> to PWMxL output.
 *								   The CLDAT<1:0> bits are not used for override functions.</i>\n
 * 	- DONT_CHANGE
 *
 * <u>FaultSource:</u> <i>/Refer to Device Datasheet For Avaliable Fault Sources.</i>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_FAULT8_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator7 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM7Fault()		\n
 *			DisableIntPWM7Fault()		\n
 *			PWM7_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen7Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(IndependentFaultEN	!= DONT_CHANGE) {FCLCON7bits.IFLTMOD = IndependentFaultEN&0b1;}				\
	if(FaultSource		!= DONT_CHANGE) {FCLCON7bits.FLTSRC = FaultSource&0b11111;}				\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM7 = FaultMode&0b1;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON7bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON7bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON7bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON7bits.FLTDAT1 = FaultData_H;									\
		IOCON7bits.FLTDAT0 = FaultData_L;									\
		FCLCON7bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON7bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON7bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON7bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#else
/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen7Fault(FaultSource,FaultMode,FaultData_H,FaultData_L,FaultInputActiveState)
 *
 * \Description		Configures PWM Generator7 Fault Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 *
 * <u>FaultSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>FaultMode:</u>\n
 *	- LATCHED_FAUL_TMODE	\n
 *	- CYCLE_FAULT_MODE	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>FaultInputActiveState:</u>\n
 *	- FAULT_INPUT_ACTIVE_LOW	\n
 *	- FAULT_INPUT_ACTIVE_HIGH	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator7 Fault Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Fault Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Change Fault Data High While PWM Module is Enabled.\n
 *		<b> 3: </b> Err3: An Attempt to Change Fault Data Low While PWM Module is Enabled.
 *
 * \RelatedMacros	EnableIntPWM7Fault()		\n
 *			DisableIntPWM7Fault()		\n
 *			PWM7_FAULT_INTERRUPT_IS_PENDING	<i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- FaultData_H & FaultData_L & FaultInputActiveState should not be changed after the PWM is enabled (PTEN = 1).	\n
 *		2- Refer to Device Datasheet For Avaliable Fault Sources.
 *
 * \Example    Err=SetupPWMGen7Fault(INDEPENDENT_FAULT_MODE,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7Fault(IndependentFaultEN, FaultSource, FaultMode, _FaultData_H, _FaultData_L, _FaultInputActiveState)({\
															\
	unsigned int _Return=0;												\
	unsigned int FaultData_H=_FaultData_H;										\
	unsigned int FaultData_L=_FaultData_L;										\
	unsigned int FaultInputActiveState=_FaultInputActiveState;							\
	if(FaultSource		!= DONT_CHANGE) {FCLCON7bits.FLTSRC = FaultSource&0b1;}					\
	if(FaultMode		!= DONT_CHANGE) {FaultModePWM7 = FaultMode&0b11111;}					\
	if(FaultData_H		== DONT_CHANGE) {FaultData_H = IOCON7bits.FLTDAT1;}					\
	if(FaultData_L		== DONT_CHANGE) {FaultData_L = IOCON7bits.FLTDAT0;}					\
	if(FaultInputActiveState== DONT_CHANGE) {FaultInputActiveState = FCLCON7bits.FLTPOL;}				\
															\
	if(PTCONbits.PTEN == 0){	/*If The PWM Module is Disabled Then You Can Change This Configration Bits.*/	\
		IOCON7bits.FLTDAT1 = FaultData_H;									\
		IOCON7bits.FLTDAT0 = FaultData_L;									\
		FCLCON7bits.FLTPOL = FaultInputActiveState;								\
	} else {													\
		if (FaultInputActiveState != FCLCON7bits.FLTPOL){							\
			/*Set Error "PWM Fault Polarity Cant be Changed When The PWM Module is Enabled"*/		\
			_Return=1;											\
		}													\
		if (FaultData_H != IOCON7bits.FLTDAT1){									\
			/*Set Error "PWM Fault Data H Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=2;											\
		}													\
		if (FaultData_L != IOCON7bits.FLTDAT0){									\
			/*Set Error "PWM Fault Data L Cant be Changed When The PWM Module is Enabled"*/			\
			_Return=3;											\
		}													\
	}														\
	_Return;													\
})
#endif

/***********************************************************************************************************************
 * \Function		unsigned int SetupPWMGen7CurrentLimit(CurrentLimSource,CurrentLimData_H,CurrentLimData_L,CurrentLimInputActiveState)
 *
 * \Description		Configures PWM Generator7 Current Limit Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>CurrentLimSource:</u>\n
 *	- SOURCE_IS_FAULT1_PIN	\n
 *	- SOURCE_IS_FAULT2_PIN	\n
 *	- SOURCE_IS_FAULT3_PIN	\n
 *	- SOURCE_IS_FAULT4_PIN	\n
 *	- SOURCE_IS_FAULT5_PIN	\n
 *	- SOURCE_IS_FAULT6_PIN	\n
 *	- SOURCE_IS_FAULT7_PIN	\n
 *	- SOURCE_IS_COMPARATOR1	\n
 *	- SOURCE_IS_COMPARATOR2	\n
 *	- SOURCE_IS_COMPARATOR3	\n
 *	- SOURCE_IS_COMPARATOR4	\n
 *	- SOURCE_IS_COMPARATOR5	\n
 *	- SOURCE_IS_FAULT_32_CLASS_B \n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_H:</u>\n
 *	- SET_H_PIN_LOW		\n
 *	- SET_H_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimData_L:</u>\n
 *	- SET_L_PIN_LOW		\n
 *	- SET_L_PIN_HIGH	\n
 * 	- DONT_CHANGE
 *
 * <u>CurrentLimInputActiveState:</u>\n
 *	- CURRENT_LIM_INPUT_ACTIVE_HIGH	\n
 *	- CURRENT_LIM_INPUT_ACTIVE_LOW	\n
 * 	- DONT_CHANGE
 *
 * \Return      <b> 0: </b> Means That Every Thing is Ok and The PWM Generator7 Current Limit Function Configered Correctly.\n
 *		<b> 1: </b> Err1: An Attempt to Change Current Limit Polarity While PWM Module is Enabled.\n
 *		<b> 2: </b> Err2: An Attempt to Use Current Limit Data High/Low in Independent Fault Mode.
 *
 * \RelatedMacros	EnablePWM7CurrentLim()		\n
 *			DisablePWM7CurrentLim()		\n
 *			EnableIntPWM7CLim()		\n
 *			DisableIntPWM7CLim()		\n
 *			PWM7_CUR_LIM_INTERRUPT_IS_PENDING <i>/Software must clear this interrupt status bit in the corresponding IFS bit.</i>
 *
 * \Notes       1- CurrentLimInputActiveState should not be changed after the PWM is enabled (PTEN = 1).\n
 *		2- Current Limit Data High/Low is Not used in Independent Fault Mode.\n
 *		3- Refer to Device Datasheet For Avaliable Current Limit Sources.
 *
 * \Example    Err=SetupPWMGen7CurrentLimit(SOURCE_IS_COMPARATOR2,SET_H_PIN_LOW,SET_L_PIN_LOW,CURRENT_LIM_INPUT_ACTIVE_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7CurrentLimit(CurrentLimSource, CurrentLimData_H, CurrentLimData_L, CurrentLimInputActiveState)({			\
																	\
	unsigned int _Return=0;														\
	if(CurrentLimData_H != DONT_CHANGE) {												\
		IOCON7bits.CLDAT1 = CurrentLimData_H&0b1;										\
		if(FCLCON7bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																	\
	if(CurrentLimData_L != DONT_CHANGE) {												\
		IOCON7bits.CLDAT0 = CurrentLimData_L&0b1;										\
		if(FCLCON7bits.IFLTMOD){_Return=2;/*Set Warning: "Current Limit Data High/Low is Not used in Independent Fault Mode"*/}	\
	}																\
																 	\
	if(CurrentLimSource != DONT_CHANGE) {FCLCON7bits.CLSRC = CurrentLimSource&0b11111;}						\
	if(CurrentLimInputActiveState != DONT_CHANGE) {											\
		if(PTCONbits.PTEN == 0){												\
			FCLCON7bits.CLPOL = CurrentLimInputActiveState&0b1;								\
		} else {														\
			if (CurrentLimInputActiveState != FCLCON7bits.CLPOL) {								\
				_Return=1; /*Set Error "PWM Current Limit Polarity Cant be Changed When The PWM Module is Enabled"*/	\
			}														\
		}															\
	}																\
	_Return;															\
})

/***********************************************************************************************************************
 * \Function	void SetupPWMGen7LEB(PWMH_RisingEdgeBlankingEN,PWMH_FallingEdgeBlankingEN,PWML_RisingEdgeBlankingEN,PWML_FallingEdgeBlankingEN,LEB_Delay)
 *
 * \Description		Configures PWM Generator7 Leading Edge Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_RISING_EDGE_BLANK \n
 *	- ENABLE_PWMH_RISING_EDGE_BLANK	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWMH_FALLING_EDGE_BLANK \n
 *	- DONT_CHANGE
 *
 * <u>PWML_RisingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_RISING_EDGE_BLANK \n
 *	- ENABLE_PWML_RISING_EDGE_BLANK	\n
 * 	- DONT_CHANGE
 *
 * <u>PWML_FallingEdgeBlankingEN:</u>\n
 *	- DISABLE_PWML_FALLING_EDGE_BLANK \n
 *	- ENABLE_PWML_FALLING_EDGE_BLANK \n
 * 	- DONT_CHANGE
 *
 * <u>LEB_Delay:</u>\n
 *	- a Value Between 1 and 4096 Inclusivly. (LEB Delay = (0 to 4096) * PWM Clock Freq) \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM7FaultBlanking()	\n
 *			EnablePWM7CurrentLimBlanking()	\n
 *			DisablePWM7FaultBlanking()	\n
 *			DisablePWM7CurrentLimBlanking()
 *
 * \Notes		LEB Delay = (0 to 4096) * PWM Clock Freq.
 *
 * \Example
 * SetupPWMGen7LEB(ENABLE_PWMH_RISING_EDGE_BLANK,ENABLE_PWMH_FALLING_EDGE_BLANK,ENABLE_PWML_RISING_EDGE_BLANK,ENABLE_PWML_FALLING_EDGE_BLANK,990);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7LEB(PWMH_RisingEdgeBlankingEN, PWMH_FallingEdgeBlankingEN,  PWML_RisingEdgeBlankingEN, PWML_FallingEdgeBlankingEN, LEB_Delay)({	\
																			\
	if(PWMH_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON7bits.PHR = PWMH_RisingEdgeBlankingEN&0b1;}						\
	if(PWMH_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON7bits.PHF = PWMH_FallingEdgeBlankingEN&0b1;}						\
	if(PWML_RisingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON7bits.PLR = PWML_RisingEdgeBlankingEN&0b1;}						\
	if(PWML_FallingEdgeBlankingEN	!= DONT_CHANGE) {LEBCON7bits.PLF = PWML_FallingEdgeBlankingEN&0b1;}						\
	if(LEB_Delay			!= DONT_CHANGE) {LEBDLY7 = LEB_Delay&0b111111111111;}								\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen7StateBlanking(PWMH_BlankingState,PWML_BlankingState,SelectedSignalForStateBlank,SelectedSignalBlankingState)
 *
 * \Description		Configures PWM Generator7 State Blanking Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>PWMH_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWMH_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_LOW	\n
 *	- ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>PWML_BlankingState:</u>\n
 *	- DONT_CARE_ABOUT_PWML_STATE \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH \n
 *	- DONT_CHANGE
 *
 * <u>SelectedSignalForStateBlank:</u>\n
 *	- NO_SIGNAL_SOURCE_FOR_STATE_BLNKING \n
 *	- PWM7H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM2H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM3H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM4H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM5H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM6H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 *	- PWM7H_AS_STATE_BLANK_SIGNAL_SOURCE \n
 * 	- DONT_CHANGE
 *
 * <u>SelectedSignalBlankingState:</u>\n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_LOW \n
 *	- ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	EnablePWM7FaultBlanking()	\n
 *			EnablePWM7CurrentLimBlanking()	\n
 *			DisablePWM7FaultBlanking()	\n
 *			DisablePWM7CurrentLimBlanking()
 *
 * \Notes		None.
 *
 * \Example	SetupPWMGen7StateBlanking(ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH,ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH,
 *					  PWM7H_AS_STATE_BLANK_SIGNAL_SOURCE,ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7StateBlanking(PWMH_BlankingState, PWML_BlankingState, SelectedSignalForStateBlank, SelectedSignalBlankingState)({			\
																			\
	if(PWMH_BlankingState		!= DONT_CHANGE) {LEBCON7bits.BPHH = (PWMH_BlankingState&0b11)>>1; LEBCON7bits.BPHL = PWMH_BlankingState&0b1;}	\
	if(PWML_BlankingState		!= DONT_CHANGE) {LEBCON7bits.BPLH = (PWML_BlankingState&0b11)>>1; LEBCON7bits.BPLL = PWML_BlankingState&0b1;}	\
	if(SelectedSignalBlankingState	!= DONT_CHANGE) {LEBCON7bits.BCH  = (SelectedSignalBlankingState&0b11)>>1;					\
							 LEBCON7bits.BCL  =  SelectedSignalBlankingState&0b1;}						\
	if(SelectedSignalForStateBlank	!= DONT_CHANGE) {AUXCON7bits.BLANKSEL = SelectedSignalForStateBlank&0b111;}					\
})

/***********************************************************************************************************************
 * \Function		void SetupPWMGen7Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)
 *
 * \Description		Configures PWM Generator7 Chop Operation.
 *
 * \Preconditions	None.
 *
 * \Inputs
 * <u>ChopClkSource:</u> <i>/Refer to Device Datasheet For Avaliable Chop Clock Sources.</i>\n
 *	- CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE \n
 *	- PWM7H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM2H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM3H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM4H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM5H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM6H_AS_CHOP_CLOCK_SOURCE	\n
 *	- PWM7H_AS_CHOP_CLOCK_SOURCE	\n
 *	- DONT_CHANGE
 *
 * <u>PWMH_ChopEN:</u>\n
 *	- DISABLE_PWMH_CHOP \n
 *	- ENABLE_PWMH_CHOP \n
 *	- DONT_CHANGE
 *
 * <u>PWML_ChopEN:</u>\n
 *	- DISABLE_PWML_CHOP \n
 *	- ENABLE_PWML_CHOP \n
 * 	- DONT_CHANGE
 *
 * \Return		None.
 *
 * \RelatedMacros	None.
 *
 * \Notes		Refer to Device Datasheet For Avaliable Chop Clock Sources.
 *
 * \Example	SetupPWMGen7Chop(CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE, ENABLE_PWMH_CHOP, ENABLE_PWML_CHOP);
 *
 **********************************************************************************************************************/
#define SetupPWMGen7Chop(ChopClkSource, PWMH_ChopEN, PWML_ChopEN)({				\
												\
	if(ChopClkSource	!= DONT_CHANGE) {AUXCON7bits.CHOPSEL = ChopClkSource&0b111;}	\
	if(PWMH_ChopEN		!= DONT_CHANGE) {AUXCON7bits.CHOPHEN = PWMH_ChopEN&0b1;}	\
	if(PWML_ChopEN		!= DONT_CHANGE) {AUXCON7bits.CHOPLEN = PWML_ChopEN&0b1;}	\
})
/***/

// <editor-fold defaultstate="collapsed" desc="PWM7 Macros and Defines">

#ifdef PWMCAP7
/******************************************************************************************************
 * \brief	The value Returned From this Routine represents the captured PWM time base value when
 *		a leading edge is detected on the current-limit input.
 *
 * \Notes	1: The capture feature is only available on the primary output (PWMxH).
 *		2: The feature is only active after LEB processing on the current-limit input signal is complete.
 *
 *****************************************************************************************************/
#define CaptuerPWM7PriTB_Val()			PWMCAP7
#endif
/***/
#define EnablePWM7Fault()			FCLCON7bits.FLTMOD = FaultModePWM7
#define DisablePWM7Fault()			FCLCON7bits.FLTMOD = 0b11

#define EnablePWM7CurrentLim()			FCLCON7bits.CLMOD = 1
#define DisablePWM7CurrentLim()			FCLCON7bits.CLMOD = 0

#define EnablePWM7FaultBlanking()		LEBCON7bits.FLTLEBEN = 1
#define DisablePWM7FaultBlanking()		LEBCON7bits.FLTLEBEN = 0

#define EnablePWM7CurrentLimBlanking()		LEBCON7bits.CLLEBEN = 1
#define DisablePWM7CurrentLimBlanking()		LEBCON7bits.CLLEBEN = 0

#define SetDutyCyclePWM7(DC)			(PDC7=DC)	/*For Independent PWM DCMode With Complementary PWM Output Mode.*/
#ifdef STPER /* Devices That Have Secondary Time Base Will Have True Independent Duty Cycle Mode Option.*/
#define SetDutyCyclePWM7H(DC)			(PDC7=DC)	/*For Independent PWM DCMode*/
#define SetDutyCyclePWM7L(DC)			(SDC7=DC)	/*For Independent PWM DCMode*/
#endif
#define SetMasterDutyCycle(DC)			(MDC=DC)	/*When Using Master DCMode.*/

/** PWM Geneator1 Interrupt Handling */
#define EnableIntPWM7()				_PWM7IE = 1
#define DisableIntPWM7()			_PWM7IE = 0
#define SetIntPriorityPWM7(Priority)		_PWM7IP = (Priority)
#define PWM7_INT_FLAG				_PWM7IF

#define EnableIntPWM7Trig()			PWMCON7bits.TRGIEN = 1
#define DisableIntPWM7Trig()			PWMCON7bits.TRGIEN = 0
#define PWM7_TRIG_INTERRUPT_IS_PENDING		PWMCON7bits.TRGSTAT

#define	EnableIntPWM7Fault()			PWMCON7bits.FLTIEN = 1
#define DisableIntPWM7Fault()			PWMCON7bits.FLTIEN = 0
#define PWM7_FAULT_INTERRUPT_IS_PENDING		PWMCON7bits.FLTSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

#define	EnableIntPWM7CLim()			PWMCON7bits.CLIEN = 1
#define	DisableIntPWM7CLim()			PWMCON7bits.CLIEN = 0
#define PWM7_CUR_LIM_INTERRUPT_IS_PENDING	PWMCON7bits.CLSTAT	/* Software must clear this interrupt status bit in the corresponding IFS bit.*/

// </editor-fold>

#endif


// <editor-fold defaultstate="collapsed" desc="PWM Library Macros and Defines">

#ifndef _COMMON_DIRECTIVES_
#define _COMMON_DIRECTIVES_

#define DONT_CHANGE	0xFFFF
#define DONT_CARE	0
#define KEEP_DEFAULT	0
#define SET_DEFAULT	0

#endif


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// SetUpPWM_PrimaryMasterTB
// SetUpPWM_SecondaryMasterTB

/**ClockPrescaler*/
#define PWM_PRESCALER_DIV_1	0
#define PWM_PRESCALER_DIV_2	1
#define PWM_PRESCALER_DIV_4	2
#define PWM_PRESCALER_DIV_8	3
#define PWM_PRESCALER_DIV_16	4
#define PWM_PRESCALER_DIV_32	5
#define PWM_PRESCALER_DIV_64	6

/**SEVPostscaler*/
#define SEV_POSTSCALER_DIV_1	1
#define SEV_POSTSCALER_DIV_2	2
#define SEV_POSTSCALER_DIV_3	3
#define SEV_POSTSCALER_DIV_4	4
#define SEV_POSTSCALER_DIV_5	5
#define SEV_POSTSCALER_DIV_6	6
#define SEV_POSTSCALER_DIV_7	7
#define SEV_POSTSCALER_DIV_8	8
#define SEV_POSTSCALER_DIV_9	9
#define SEV_POSTSCALER_DIV_10	10
#define SEV_POSTSCALER_DIV_11	11
#define SEV_POSTSCALER_DIV_12	12
#define SEV_POSTSCALER_DIV_13	13
#define SEV_POSTSCALER_DIV_14	14
#define SEV_POSTSCALER_DIV_15	15
#define SEV_POSTSCALER_DIV_16	16

/**ENimmediatPeriodUpdate*/
#define DISABLE_IMMEDIAT_UPDATE	0
#define ENABLE_IMMEDIAT_UPDATE	1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// SetupPWMChopGenerator

/**ChopClockDivider*/
/* ChopClockDivider = 1 to 1023.
 * Chop Frequency = (FP/PCLKDIV) / (CHOPCLK<9:0> + 1)
 * As an example, for devices running at 60 MIPS, a value of all zeros will yield a 60 MHz chop clock
 * (period = 16.7 ns) with the PWM clock prescaler configured for fastest clock.*/

/**ChopClockGeneratorEN*/
#define DISABLE_CHOP_CLOCK_GENERATOR	0
#define ENABLE_CHOP_CLOCK_GENERATOR	1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// SetUpPWMGenx

/**PWM_Mode*/
#define COMPLEMENTARY_MODE	0
#define REDUNDANT_MODE		1
#define PUSHPULL_MODE		2
#ifdef STPER /* Devices That Have Secondary Time Base Will Have True Independent PWM Output Mode Option.*/
#define TRUE_INDEPENDENT_MODE	3
#endif

/**Aligment*/
#define EDGE_ALIGNED		0
#define CENTER_ALIGNED		1

/**TimeBaseSelect*/
#define PRIMARY_TIME_BASE	0
#ifdef STPER
#define SECONDARY_TIME_BASE	1
#endif
#define INDEPENDENT_TIME_BASE	2

/**DCMode*/
#define INDEPENDENT_DUTY_CYCLE	0
#define MASTER_DUTY_CYCLE	1

/**ENimmediatDCupdate*/
#define DISABLE_IMMEDIAT_DC_UPDATE	0
#define ENABLE_IMMEDIAT_DC_UPDATE	1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// SetUpPWMGenxPins

/**H_PinOwnership*/
#define GPIO_MODULE_CONTROL_H_PIN	0
#define PWM_MODULE_CONTROL_H_PIN	1

/**L_PinOwnership*/
#define GPIO_MODULE_CONTROL_L_PIN	0
#define PWM_MODULE_CONTROL_L_PIN	1

/**H_PinPolarity*/
#define H_PIN_ACTIVE_HIGH		0
#define H_PIN_ACTIVE_LOW		1

/**L_PinPolarity*/
#define L_PIN_ACTIVE_HIGH		0
#define L_PIN_ACTIVE_LOW		1

/**EN_PinSwap*/
#define H_L_PINS_NOT_SWAPED		0
#define H_L_PINS_SWAPED			1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// SetUpPWMGenxOverrid

/**H_PinOverEN*/
#define PWM_GEN_CONTROL_H_PIN	0
#define OVR_DAT_CONTROL_H_PIN	1

/**L_PinOverEN*/
#define PWM_GEN_CONTROL_L_PIN	0
#define OVR_DAT_CONTROL_L_PIN	1

/**OverDataForH_Pin*/
#define OVERRIDE_H_PIN_LOW	0
#define OVERRIDE_H_PIN_HIGH	1

/**OverDataForL_Pin*/
#define OVERRIDE_L_PIN_LOW	0
#define OVERRIDE_L_PIN_HIGH	1

/**SyncOverride*/
#define DISABLE_OVERRIDE_SYNC	0
#define ENABLE_OVERRIDE_SYNC	1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// SetUpPWMGenxDeadTime
/* Dead time compensation is available only for Complementary PWM output mode with Positive Dead Time mode */
/* Dead time vary (from 0 to 16383) * PWM Clock Freq */

/**DeadTimeMode*/
#define POSITIVE_DT_MODE		0
#define NEGATIVE_DT_MODE		1 // Not allowed in Center_Align mode
#define DISABLE_DT_MODE			2
#define NEGATIVE_COMPENSATION_DT_MODE	3 /*If DTCMPx = 0, PWMxH is shortened and PWMxL is lengthened.
					    If DTCMPx = 1, PWMxL is shortened and PWMxH is lengthened.*/
#define POSITIVE_COMPENSATION_DT_MODE	4 /*If DTCMPx = 0, PWMxL is shortened and PWMxH is lengthened.
					    If DTCMPx = 1, PWMxH is shortened and PWMxL is lengthened.*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// SetUpPWMGen1ADCTrigger
/* TriggerCompareVal vary from 0 to 65535 */

/**TrigStartWaitCycles*/
#define WAIT_0_PWM_CYC_BEFORE_FIRST_TRIG	0
#define WAIT_1_PWM_CYC_BEFORE_FIRST_TRIG	1
#define WAIT_2_PWM_CYC_BEFORE_FIRST_TRIG	2
#define WAIT_3_PWM_CYC_BEFORE_FIRST_TRIG	3
#define WAIT_4_PWM_CYC_BEFORE_FIRST_TRIG	4
#define WAIT_5_PWM_CYC_BEFORE_FIRST_TRIG	5
#define WAIT_6_PWM_CYC_BEFORE_FIRST_TRIG	6
#define WAIT_7_PWM_CYC_BEFORE_FIRST_TRIG	7
#define WAIT_8_PWM_CYC_BEFORE_FIRST_TRIG	8
#define WAIT_9_PWM_CYC_BEFORE_FIRST_TRIG	9
#define WAIT_10_PWM_CYC_BEFORE_FIRST_TRIG	10
#define WAIT_11_PWM_CYC_BEFORE_FIRST_TRIG	11
#define WAIT_12_PWM_CYC_BEFORE_FIRST_TRIG	12
#define WAIT_13_PWM_CYC_BEFORE_FIRST_TRIG	13
#define WAIT_14_PWM_CYC_BEFORE_FIRST_TRIG	14
#define WAIT_15_PWM_CYC_BEFORE_FIRST_TRIG	15
#define WAIT_16_PWM_CYC_BEFORE_FIRST_TRIG	16
#define WAIT_17_PWM_CYC_BEFORE_FIRST_TRIG	17
#define WAIT_18_PWM_CYC_BEFORE_FIRST_TRIG	18
#define WAIT_19_PWM_CYC_BEFORE_FIRST_TRIG	19
#define WAIT_20_PWM_CYC_BEFORE_FIRST_TRIG	20
#define WAIT_21_PWM_CYC_BEFORE_FIRST_TRIG	21
#define WAIT_22_PWM_CYC_BEFORE_FIRST_TRIG	22
#define WAIT_23_PWM_CYC_BEFORE_FIRST_TRIG	23
#define WAIT_24_PWM_CYC_BEFORE_FIRST_TRIG	24
#define WAIT_25_PWM_CYC_BEFORE_FIRST_TRIG	25
#define WAIT_26_PWM_CYC_BEFORE_FIRST_TRIG	26
#define WAIT_27_PWM_CYC_BEFORE_FIRST_TRIG	27
#define WAIT_28_PWM_CYC_BEFORE_FIRST_TRIG	28
#define WAIT_29_PWM_CYC_BEFORE_FIRST_TRIG	29
#define WAIT_30_PWM_CYC_BEFORE_FIRST_TRIG	30
#define WAIT_31_PWM_CYC_BEFORE_FIRST_TRIG	31
#define WAIT_32_PWM_CYC_BEFORE_FIRST_TRIG	32
#define WAIT_33_PWM_CYC_BEFORE_FIRST_TRIG	33
#define WAIT_34_PWM_CYC_BEFORE_FIRST_TRIG	34
#define WAIT_35_PWM_CYC_BEFORE_FIRST_TRIG	35
#define WAIT_36_PWM_CYC_BEFORE_FIRST_TRIG	36
#define WAIT_37_PWM_CYC_BEFORE_FIRST_TRIG	37
#define WAIT_38_PWM_CYC_BEFORE_FIRST_TRIG	38
#define WAIT_39_PWM_CYC_BEFORE_FIRST_TRIG	39
#define WAIT_40_PWM_CYC_BEFORE_FIRST_TRIG	40
#define WAIT_41_PWM_CYC_BEFORE_FIRST_TRIG	41
#define WAIT_42_PWM_CYC_BEFORE_FIRST_TRIG	42
#define WAIT_43_PWM_CYC_BEFORE_FIRST_TRIG	43
#define WAIT_44_PWM_CYC_BEFORE_FIRST_TRIG	44
#define WAIT_45_PWM_CYC_BEFORE_FIRST_TRIG	45
#define WAIT_46_PWM_CYC_BEFORE_FIRST_TRIG	46
#define WAIT_47_PWM_CYC_BEFORE_FIRST_TRIG	47
#define WAIT_48_PWM_CYC_BEFORE_FIRST_TRIG	48
#define WAIT_49_PWM_CYC_BEFORE_FIRST_TRIG	49
#define WAIT_50_PWM_CYC_BEFORE_FIRST_TRIG	50
#define WAIT_51_PWM_CYC_BEFORE_FIRST_TRIG	51
#define WAIT_52_PWM_CYC_BEFORE_FIRST_TRIG	52
#define WAIT_53_PWM_CYC_BEFORE_FIRST_TRIG	53
#define WAIT_54_PWM_CYC_BEFORE_FIRST_TRIG	54
#define WAIT_55_PWM_CYC_BEFORE_FIRST_TRIG	55
#define WAIT_56_PWM_CYC_BEFORE_FIRST_TRIG	56
#define WAIT_57_PWM_CYC_BEFORE_FIRST_TRIG	57
#define WAIT_58_PWM_CYC_BEFORE_FIRST_TRIG	58
#define WAIT_59_PWM_CYC_BEFORE_FIRST_TRIG	59
#define WAIT_60_PWM_CYC_BEFORE_FIRST_TRIG	60
#define WAIT_61_PWM_CYC_BEFORE_FIRST_TRIG	61
#define WAIT_62_PWM_CYC_BEFORE_FIRST_TRIG	62
#define WAIT_63_PWM_CYC_BEFORE_FIRST_TRIG	63

/**TrigOutputDiv*/
#define TRIGGER_ON_EVERY_TRIGGER_EVENT		0
#define TRIGGER_ON_EVERY_2ND_TRIGGER_EVENT	1
#define TRIGGER_ON_EVERY_3RD_TRIGGER_EVENT	2
#define TRIGGER_ON_EVERY_4TH_TRIGGER_EVENT	3
#define TRIGGER_ON_EVERY_5TH_TRIGGER_EVENT	4
#define TRIGGER_ON_EVERY_6TH_TRIGGER_EVENT	5
#define TRIGGER_ON_EVERY_7TH_TRIGGER_EVENT	6
#define TRIGGER_ON_EVERY_8TH_TRIGGER_EVENT	7
#define TRIGGER_ON_EVERY_9TH_TRIGGER_EVENT	8
#define TRIGGER_ON_EVERY_10TH_TRIGGER_EVENT	9
#define TRIGGER_ON_EVERY_11TH_TRIGGER_EVENT	10
#define TRIGGER_ON_EVERY_12TH_TRIGGER_EVENT	11
#define TRIGGER_ON_EVERY_13TH_TRIGGER_EVENT	12
#define TRIGGER_ON_EVERY_14TH_TRIGGER_EVENT	13
#define TRIGGER_ON_EVERY_15TH_TRIGGER_EVENT	14
#define TRIGGER_ON_EVERY_16TH_TRIGGER_EVENT	15


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// SetUpPWMGenxFault
/****************************************************************************************************
 * Fault Exit Sequence:
 *
 * If Cycle-by-Cycle Fault mode is selected, the fault is automatically reset on every PWM cycle. No
 * additional coding is needed to exit the Fault condition.
 * For the Latched Fault mode, however, the following sequence must be followed to exit the Fault condition:
 *	1. Poll the PWM fault source to determine, if the fault signal has been deasserted.
 *	2. If the PWM fault interrupt is not enabled, skip the following sub-steps and proceed to step (3),
 *	   If the PWM fault interrupt is enabled, perform the following sub-steps, and then proceed to step (4).
 *		a) Complete the PWM fault Interrupt Service Routine.
 *		b) Disable the PWM fault interrupt by clearing the FLTIEN bit (PWMCONx <12>).
 *		c) Enable the PWM fault interrupt by setting FLTMOD<1:0> (FCLCONx <1:0>) = 0b00.
 *	3. Disable PWM faults by setting FLTMOD<1:0> (FCLCONx <1:0>) = 0b11.
 *	4. Enable the latched PWM Fault mode by setting FLTMOD<1:0> (FCLCONx <1:0>) = 0b00.
*****************************************************************************************************/

/**IndependentFaultEN*/
#ifdef STPER /* Devices That Have Secondary Time Base Will Have Independent Fault Mode Option.*/
#define NORMAL_FAULT_MODE		0
#define INDEPENDENT_FAULT_MODE	1
#endif

/**FaultSource & CurrentLimSource*/
#define SOURCE_IS_FAULT1_PIN		0
#define SOURCE_IS_FAULT2_PIN		1
#define SOURCE_IS_FAULT3_PIN		2
#define SOURCE_IS_FAULT4_PIN		3
#define SOURCE_IS_FAULT5_PIN		4
#define SOURCE_IS_FAULT6_PIN		5
#define SOURCE_IS_FAULT7_PIN		6
#define SOURCE_IS_FAULT8_PIN		7
#define SOURCE_IS_COMPARATOR1		8
#define SOURCE_IS_COMPARATOR2		9
#define SOURCE_IS_COMPARATOR3		10
#define SOURCE_IS_COMPARATOR4		11
#define SOURCE_IS_COMPARATOR5		12
#define SOURCE_IS_FAULT_32_CLASS_B	31

/**FaultMode*/
#define LATCHED_FAULT_MODE	0
#define CYCLE_FAULT_MODE	1

/**FaultData_H & CurrentLimData_H*/
#define SET_H_PIN_LOW		0
#define SET_H_PIN_HIGH		1

/**General Use*/
#define SET_PIN_LOW			0
#define SET_PIN_HIGH		1

/**FaultData_L & CurrentLimData_L*/
#define SET_L_PIN_LOW		0
#define SET_L_PIN_HIGH		1

/**FaultInputActiveState*/
#define FAULT_INPUT_ACTIVE_LOW	0
#define FAULT_INPUT_ACTIVE_HIGH	1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// SetUpPWMGenxCurrentLimit

/**CurrentLimInputActiveState*/
#define CURRENT_LIM_INPUT_ACTIVE_HIGH	0
#define CURRENT_LIM_INPUT_ACTIVE_LOW	1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// SetUpPWMGenxLEB
/* LEB Delay Vary From (0 to 4096) * PWM Clock Freq */

/**PWMH_RisingEdgeBlankingEN*/
#define DISABLE_PWMH_RISING_EDGE_BLANK		0
#define ENABLE_PWMH_RISING_EDGE_BLANK		1

/**PWMH_FallingEdgeBlankingEN*/
#define DISABLE_PWMH_FALLING_EDGE_BLANK		0
#define ENABLE_PWMH_FALLING_EDGE_BLANK		1

/**PWML_RisingEdgeBlankingEN*/
#define DISABLE_PWML_RISING_EDGE_BLANK		0
#define ENABLE_PWML_RISING_EDGE_BLANK		1

/**PWML_FallingEdgeBlankingEN*/
#define DISABLE_PWML_FALLING_EDGE_BLANK		0
#define ENABLE_PWML_FALLING_EDGE_BLANK		1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// SetUpPWMGenxStateBlanking

/**PWMH_BlankingState*/
#define DONT_CARE_ABOUT_PWMH_STATE			0b00
#define ENABLE_BLANKING_JUST_WHEN_PWMH_IS_LOW		0b01
#define ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH		0b10

/**PWML_BlankingState*/
#define DONT_CARE_ABOUT_PWML_STATE			0b00
#define ENABLE_BLANKING_JUST_WHEN_PWML_IS_LOW		0b01
#define ENABLE_BLANKING_JUST_WHEN_PWML_IS_HIGH		0b10

/**SelectedSignalBlankingState*/
#define ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_LOW		0b01
#define ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH	0b10

/**SelectedSignalForStateBlank*/
#define NO_SIGNAL_SOURCE_FOR_STATE_BLNKING		0
#define PWM1H_AS_STATE_BLANK_SIGNAL_SOURCE		1
#define PWM2H_AS_STATE_BLANK_SIGNAL_SOURCE		2
#define PWM3H_AS_STATE_BLANK_SIGNAL_SOURCE		3
#define PWM4H_AS_STATE_BLANK_SIGNAL_SOURCE		4
#define PWM5H_AS_STATE_BLANK_SIGNAL_SOURCE		5
#define PWM6H_AS_STATE_BLANK_SIGNAL_SOURCE		6
#define PWM7H_AS_STATE_BLANK_SIGNAL_SOURCE		7

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// SetUpPWMGenxChop

/**ChopClkSource*/
#define CHOP_CLK_GEN_AS_CHOP_CLK_SOURCE	0
#define PWM1H_AS_CHOP_CLOCK_SOURCE	1
#define PWM2H_AS_CHOP_CLOCK_SOURCE	2
#define PWM3H_AS_CHOP_CLOCK_SOURCE	3
#define PWM4H_AS_CHOP_CLOCK_SOURCE	4
#define PWM5H_AS_CHOP_CLOCK_SOURCE	5
#define PWM6H_AS_CHOP_CLOCK_SOURCE	6
#define PWM7H_AS_CHOP_CLOCK_SOURCE	7

/**PWMH_ChopEN*/
#define DISABLE_PWMH_CHOP		0
#define ENABLE_PWMH_CHOP		1

/**PWML_ChopEN*/
#define DISABLE_PWML_CHOP		0
#define ENABLE_PWML_CHOP		1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// All PWM Module Interrupts And Status Bits.

//----------------------------------
/** Primary Time Base Interrupt Handling */
#define EnableIntPWMSEV()		({PTCONbits.SEIEN = 1; _PSEMIE = 1;})
#define DiableIntPWMSEV()		({PTCONbits.SEIEN = 0; _PSEMIE = 0;})
#define SetIntPriorityPWMSEV(Priority)	_PSEMIP = (Priority)
#define PWMSEV_INT_FLAG			_PSEMIF

//----------------------------------
/** Secoundary Time Base Interrupt Handling */
#define	EnableIntPWMSSEV()		({STCONbits.SEIEN = 1; _PSESMIE = 1;})
#define	DiableIntPWMSSEV()		({STCONbits.SEIEN = 0; _PSESMIE = 0;})
#define SetIntPriorityPWMSSEV(Priority)	_PSESMIP = (Priority)
#define PWMSSEV_INT_FLAG		_PSESMIF

// </editor-fold>
