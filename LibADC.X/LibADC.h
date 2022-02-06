
/* ADC Module Libary For All dsPIC33EP/PIC24EP Devices.
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
 * File: $Id: LibADC.h, V1.00 2015/03/12 AL-Moutaz Billah Tabbakha Exp $
 *
 * Functions in This Library : ("x" Denotes to ADC Number)
 *
 * void SetupADCx(	ADCResolution,
 *			ADCVoltageRef,
 *			SamplingMode,
 *			StartOfSampleSrc,
 *			StartOfConversionSrc,
 *			ConversionClockTAD,
 *			OutputFormat,
 *			IdelEN,
 *			BufferFillMode,
 *			ResultPerInt,
 *			DMATransferEN,
 *			DMABufferBuildMode,
 *			DMABLPerAnalogInput,
 *			DMAAddrIncrementRate)
 *
 * void SetupADCxChannels(	ChannelToConvert,
 *				AltInputModeEN,
 *				Ch0ScanEN,
 *				Ch0ScanInputs,
 *				Ch0PosInputMuxA,Ch0NegInputMuxA,
 *				Ch0PosInputMuxB,Ch0NegInputMuxB,
 *				Ch123PosInputsMuxA, Ch123NegInputsMuxA,
 *				Ch123PosInputsMuxB, Ch123NegInputsMuxB)
 *
 * void StartOperationADCx()
 * void StopOperationADCx()
 * void BeginSamplingADCx()
 * void BeginConvertingADCx()
 * void ADCxConversionCompleted()
 * void ADCxFillingSecBuffHalf()
 * void EnableIntADCx()
 * void DisableIntADCx()
 * void SetPriorityIntADCx(priority)
 *
 */

/*====================================================================================================================*/
#include <xc.h>
#include <libpic30.h>	     /* For __delay32() and __delay_us()*/
#include <math.h>	     /* For ceilf()*/

#ifndef _ADC_LIB_
#define _ADC_LIB_

#ifndef SYS_FREQ
#define SYS_FREQ 120000000UL
#endif

#ifndef FP
#define FP SYS_FREQ/2
#endif

#ifndef FCY
#define FCY SYS_FREQ/2
#endif

#ifndef TCY
#define TCY 1/FCY
#endif


// <editor-fold defaultstate="collapsed" desc="dsPIC33EP And PIC24EP Group Detecting">

// (dsPIC33EPXXX(GP/MC/MU)806/810/814 and PIC24EPXXX(GP/GU)810/814)  MCUs Group.
#if defined  (__dsPIC33EP256MU806__) || defined  (__dsPIC33EP256MU810__) || defined  (__dsPIC33EP256MU814__) ||	\
    defined  (__dsPIC33EP512GP806__) || defined  (__dsPIC33EP512MC806__) || defined  (__dsPIC33EP512MU810__) ||	\
    defined  (__dsPIC33EP512MU814__) || defined  (__PIC24EP256GU810__)   || defined  (__PIC24EP256GU814__)   ||	\
    defined  (__PIC24EP512GP806__)   || defined  (__PIC24EP512GU810__)   || defined  (__PIC24EP512GU814__)
#define GROUP1_DSPIC33E_PIC24E_FAMILY
#define GROUP1_dsPIC33EPXXX_GP_MC_MU_806_810_814_And_PIC24EPXXX_GP_GU_810_814
#endif

// (dsPIC33EPXXX(GM)3XX/6XX/7XX)  MCUs Group.
#if defined  (__dsPIC33EP128GM304__) || defined  (__dsPIC33EP128GM604__) || defined  (__dsPIC33EP256GM304__) ||	\
	defined  (__dsPIC33EP256GM604__) || defined  (__dsPIC33EP512GM304__) || defined  (__dsPIC33EP512GM604__) ||	\
	defined  (__dsPIC33EP128GM306__) || defined  (__dsPIC33EP128GM706__) || defined  (__dsPIC33EP256GM306__) ||	\
	defined  (__dsPIC33EP128GM310__) || defined  (__dsPIC33EP128GM710__) || defined  (__dsPIC33EP256GM310__) ||	\
	defined  (__dsPIC33EP256GM710__) || defined  (__dsPIC33EP512GM310__) || defined  (__dsPIC33EP512GM710__) ||	\
	defined  (__dsPIC33EP256GM706__) || defined  (__dsPIC33EP512GM306__) || defined  (__dsPIC33EP512GM706__)
#define GROUP2_DSPIC33E_PIC24E_FAMILY
#define GROUP2_dsPIC33EPXXX_GM_3XX_6XX_7XX
#endif

// (dsPIC33EPXXXGP50X, dsPIC33EPXXXMC20X/50X AND PIC24EPXXXGP/MC20X)  MCUs Group.
#if defined  (__PIC24EP32GP202__)    || defined  (__PIC24EP64GP202__)    || defined  (__PIC24EP128GP202__)   ||	\
	defined  (__PIC24EP256GP202__)   || defined  (__PIC24EP512GP202__)   || defined  (__PIC24EP32GP203__)    ||	\
	defined  (__PIC24EP64GP203__)    || defined  (__PIC24EP32GP204__)    || defined  (__PIC24EP64GP204__)    ||	\
	defined  (__PIC24EP128GP204__)   || defined  (__PIC24EP256GP204__)   || defined  (__PIC24EP512GP204__)   ||	\
	defined  (__PIC24EP64GP206__)    || defined  (__PIC24EP128GP206__)   || defined  (__PIC24EP256GP206__)   ||	\
	defined  (__PIC24EP512GP206__)   || defined  (__dsPIC33EP32GP502__)  || defined  (__dsPIC33EP64GP502__)  ||	\
	defined  (__dsPIC33EP128GP502__) || defined  (__dsPIC33EP256GP502__) || defined  (__dsPIC33EP512GP502__) ||	\
	defined  (__dsPIC33EP32GP503__)  || defined  (__dsPIC33EP64GP503__)  || defined  (__dsPIC33EP32GP504__)  ||	\
	defined  (__dsPIC33EP64GP504__)  || defined  (__dsPIC33EP128GP504__) || defined  (__dsPIC33EP256GP504__) ||	\
	defined  (__dsPIC33EP512GP504__) || defined  (__dsPIC33EP64GP506__)  || defined  (__dsPIC33EP128GP506__) ||	\
	defined  (__dsPIC33EP256GP506__) || defined  (__dsPIC33EP512GP506__) || defined  (__PIC24EP32MC202__)    ||	\
	defined  (__PIC24EP64MC202__)    || defined  (__PIC24EP128MC202__)   || defined  (__PIC24EP256MC202__)   ||	\
	defined  (__PIC24EP512MC202__)   || defined  (__PIC24EP32MC203__)    || defined  (__PIC24EP64MC203__)    ||	\
	defined  (__PIC24EP32MC204__)    || defined  (__PIC24EP64MC204__)    || defined  (__PIC24EP128MC204__)   ||	\
	defined  (__PIC24EP256MC204__)   || defined  (__PIC24EP512MC204__)   || defined  (__PIC24EP64MC206__)    ||	\
	defined  (__PIC24EP128MC206__)   || defined  (__PIC24EP256MC206__)   || defined  (__PIC24EP512MC206__)   ||	\
	defined  (__dsPIC33EP32MC202__)  || defined  (__dsPIC33EP64MC202__)  || defined  (__dsPIC33EP128MC202__) ||	\
	defined  (__dsPIC33EP256MC202__) || defined  (__dsPIC33EP512MC202__) || defined  (__dsPIC33EP32MC203__)  ||	\
	defined  (__dsPIC33EP64MC203__)  || defined  (__dsPIC33EP32MC204__)  || defined  (__dsPIC33EP64MC204__)  ||	\
	defined  (__dsPIC33EP128MC204__) || defined  (__dsPIC33EP256MC204__) || defined  (__dsPIC33EP512MC204__) ||	\
	defined  (__dsPIC33EP64MC206__)  || defined  (__dsPIC33EP128MC206__) || defined  (__dsPIC33EP256MC206__) ||	\
	defined  (__dsPIC33EP512MC206__) || defined  (__dsPIC33EP32MC502__)  || defined  (__dsPIC33EP64MC502__)  ||	\
	defined  (__dsPIC33EP128MC502__) || defined  (__dsPIC33EP256MC502__) || defined  (__dsPIC33EP512MC502__) ||	\
	defined  (__dsPIC33EP32MC503__)  || defined  (__dsPIC33EP64MC503__)  || defined  (__dsPIC33EP32MC504__)  ||	\
	defined  (__dsPIC33EP64MC504__)  || defined  (__dsPIC33EP128MC504__) || defined  (__dsPIC33EP256MC504__) ||	\
	defined  (__dsPIC33EP512MC504__) || defined  (__dsPIC33EP64MC506__)  || defined  (__dsPIC33EP128MC506__) ||	\
	defined  (__dsPIC33EP256MC506__) || defined  (__dsPIC33EP512MC506__)
#define GROUP3_DSPIC33E_PIC24E_FAMILY
#define GROUP3_dsPIC33EPXXX_GP_50X_dsPIC33EPXXX_MC_20X_50X_PIC24EPXXX_GP_MC_20X
#endif
// </editor-fold>

#define FASTEST_TAD_FOR_10bit_MODE		((int)ceilf(76/(1000000000ULL/FCY))-1)
#define FASTEST_TAD_FOR_12bit_MODE		((int)ceilf(118/(1000000000ULL/FCY))-1)
#define FASTEST_SAMPLE_TIME_FOR_12bit		SOC_SRC_AUTO_AFTER_3TAD
#define FASTEST_SAMPLE_TIME_FOR_10bit_NoOpAMP	SOC_SRC_AUTO_AFTER_2TAD
#define FASTEST_SAMPLE_TIME_FOR_10bit_OpAMP	SOC_SRC_AUTO_AFTER_4TAD

//#include "p33EP128GM304.h"
//#include "p33EP128GM710.h"

#ifndef _COMMON_DIRECTIVES_
#define _COMMON_DIRECTIVES_

#define DONT_CHANGE	0xFFFF
#define DONT_CARE	0
#define KEEP_DEFAULT	0
#define SET_DEFAULT	0

#endif


/**********************************************************************************************************************/
// Functions For (dsPIC33EPXXX(GP/MC/MU)806/810/814 and PIC24EPXXX(GP/GU)810/814)  MCUs Group.
/**********************************************************************************************************************/
#ifdef GROUP1_dsPIC33EPXXX_GP_MC_MU_806_810_814_And_PIC24EPXXX_GP_GU_810_814

/**********************************************************************************************************************/
/***************************************** ADC 1 Macro Functions & Defines ********************************************/
/**********************************************************************************************************************/
/***********************************************************************************************************************
 * \Function	void SetupADC1( ADCResolution,
 *                              ADCVoltageRef,
 *				SamplingMode,
 *				StartOfSampleSrc,
 *				StartOfConversionSrc,
 *				ConversionClockTAD,
 *				OutputFormat,
 *				IdelEN,
 *				BufferFillMode,
 *				ResultPerInt,
 *				DMATransferEN,
 *				DMABufferBuildMode,
 *				DMABLPerAnalogInput,
 *				DMAAddrIncrementRate)
 *
 * \Description		Configures ADC1 Operation Parameters.
 *
 * \PreCondition	You Must Set The SYS_FREQ and FCY in The Beginning of The Library Header File Before Using The Library.
 *
 * \Inputs
 * <u>ADCResolution:</u>\n
 *	- ADC_10BIT_MODE			
 *	- ADC_12BIT_MODE			
 *
 * <u>ADCVoltageRef:</u>\n
 *	- ADC_REF_AVDD_AVSS			<i>/ A/D Voltage reference configuration	VrefH is	 AVdd	and VrefL is AVss.</i> \n
 *	- ADC_REF_VREFP_AVSS		<i>/ A/D Voltage reference configuration	VrefH is Ext Vref+	and	VrefL is AVss.</i> \n
 *	- ADC_REF_AVDD_VREFN		<i>/ A/D Voltage reference configuration	VrefH is	 AVdd	and	VrefL is Ext Vref-.</i> \n
 *	- ADC_REF_VREFP_VREFN		<i>/ A/D Voltage reference configuration	VrefH is Ext Vref+	and	VrefL is Ext Vref-.</i> \n
 *
 * <u>SamplingMode:</u>\n
 *	- SEQUENTIAL_SAMPLING		<i>/ Samples multiple channels individually in sequence.</i>\n
 *	- SIMULTANEOUS_SAMPLING		<i>/ only applicable when more than one channel are converted.</i>\n
 *
 * <u>StartOfSampleSrc:</u>\n
 *	- MANUAL_SAMPLING		<i>/ Sampling begins when SAMP bit is set, OR Use BeginSamplingADC1() Function.</i>\n
 *	- AUTO_SAMPLING			<i>/ Sampling begins immediately after last conversion; SAMP bit is auto-set.</i>\n
 *
 * <u>StartOfConversionSrc:</u>\n
 *	- SOC_SRC_MANUAL		<i>/ Clearing the (SAMP) bit ends sampling and starts conversion (Manual mode), OR Use BeginConversionADC1() Function.</i>\n
 *	- SOC_SRC_INT0			<i>/ Active transition on the INT0 pin ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_TMR3			<i>/ Timer 3 Interrupt ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_TMR5			<i>/ Timer 5 Interrupt ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_AUTO_AFTER_0TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- SOC_SRC_AUTO_AFTER_1TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- .																											 \n
 *	- .																											 \n
 *	- SOC_SRC_AUTO_AFTER_31TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- SOC_SRC_AUTO_FASTEST_NoOpAMP_ADC1	<i>/ Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if No OpAmp Outputs is Used As inputs for The ADC Module.</i>\n
 *	- SOC_SRC_AUTO_FASTEST_OpAMP	<i>/ Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if Any OpAmp Output is Used As inputs for The ADC Module.</i>\n
 *	- SOC_SRC_PRI_SEVT		<i>/ (Only dsPICs) PWM primary Special Event Trigger ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_SEC_SEVT		<i>/ (Only dsPICs) PWM secondary Special Event Trigger ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_PWM1			<i>/ (Only dsPICs) PWM Generator 1 primary trigger compare ends sampling and starts conversion.</i> \n
 *	- .																																	\n
 *	- .																																	\n
 *	- SOC_SRC_PWM7			<i>/ (Only dsPICs) PWM Generator 7 primary trigger compare ends sampling and starts conversion.</i> \n
 *	- ADC_CLK_CTMU			<i>/ CTMU ends sampling and starts conversion (if CTMU Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO12		<i>/ PTGO12 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO13		<i>/ PTGO13 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO14		<i>/ PTGO14 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO15		<i>/ PTGO15 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *
 * <u>ConversionClockTAD:</u>\n
 *	- TAD_EQUAL_INTERNAL_tRC	<i>/ TAD = ADC internal RC clock Interval.</i>\n
 *	- TAD_EQUAL_FASTEST_FOR_ADC1		<i>/ Automatically Set The Fastest Time Of TAD Respecting (FCY & ADC Resolution).</i>\n
 *	- TAD_EQUAL_1Tcy		\n
 *	- TAD_EQUAL_2Tcy		\n
 *	- .				\n
 *	- .				\n
 *	- TAD_EQUAL_256Tcy
 *
 * <u>OutputFormat:</u>\n
 *	- OUTPUT_FORMAT_U_INT		<i>/ A/D data format integer.</i>\n
 *	- OUTPUT_FORMAT_S_INT		<i>/ A/D data format signed integer.</i>\n
 *	- OUTPUT_FORMAT_U_FRACT		<i>/ A/D data format fractional.</i>\n
 *	- OUTPUT_FORMAT_S_FRACT		<i>/ A/D data format signed fractional.</i>\n
 *
 * <u>IdelEN:</u>\n
 *	- ADC_CONTINUE_IN_IDEL		<i>/ Continue ADC in Idle mode.</i>\n
 *	- ADC_STOP_IN_IDEL			<i>/ Stop ADC in Idle mode.</i>\n
 *
 * <u>BufferFillMode:</u> Only if DMA Transfer is Disabled Else Put (DONT_CARE).\n
 *	- RESULTS_BUFFER_16LOC_MODE		<i>/ Always starts filling the buffer from the Start address.</i>\n
 *	- RESULTS_BUFFER_2X8LOC_ALT_MODE	<i>/ Starts buffer filling the first half of the buffer on the first interrupt
 *										   and the second half of the buffer on the next interrupt And So On.</i>\n
 *
 * <u>ResultPerInt:</u> Only if DMA Transfer is Disabled Else Put (DONT_CARE).\n
 *	- INT_WHEN_1_NEW_RESULT_IN_BUFF			\n
 *	- INT_WHEN_2_NEW_RESULT_IN_BUFF			\n
 *	- INT_WHEN_3_NEW_RESULT_IN_BUFF			\n
 *	- INT_WHEN_4_NEW_RESULT_IN_BUFF			\n
 *	- INT_WHEN_5_NEW_RESULT_IN_BUFF			\n
 *	- INT_WHEN_6_NEW_RESULT_IN_BUFF			\n
 *	- INT_WHEN_7_NEW_RESULT_IN_BUFF			\n
 *	- INT_WHEN_8_NEW_RESULT_IN_BUFF			\n
 *	- INT_WHEN_9_NEW_RESULT_IN_BUFF			\n
 *	- INT_WHEN_10_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_11_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_12_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_13_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_14_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_15_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_16_NEW_RESULT_IN_BUFF
 *
 *	- a Value From 1 to 16			<i>/ if SamplingMode = SEQUENTIAL_SAMPLING   & BufferFillMode = RESULTS_BUFFER_16LOC_MODE.</i>\n
 *	- a Value Equ 2,4,6,8,10,12,14,16	<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_16LOC_MODE & Converting 2 Ch.</i>\n
 *	- a Value Equ 4,8,12,16			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_16LOC_MODE & Converting 4 Ch.</i>\n
 *	- a Value From 1 to 8			<i>/ if SamplingMode = SEQUENTIAL_SAMPLING   & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE.</i>\n
 *	- a Value Equ 2,4,6,8			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE & Converting 2 Ch.</i>\n
 *	- a Value Equ 4,8			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE & Converting 4 Ch.</i>\n
 *
 * <u>DMATransferEN:</u>\n
 *	- DIS_DMA_TRANSFER				<i>/ Conversion results are stored in ADCxBUF0 through ADCxBUFF registers; DMA will not be used.</i>\n
 *	- EN_DMA_TRANSFER				<i>/ Conversion results are stored in ADCxBUF0 register for transferring to RAM using DMA.</i>\n
 *
 * <u>DMABufferBuildMode:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- DMA_BM_SCATTR				<i>/ DMA buffers are written in Scatter/Gather mode.</i>\n
 *	- DMA_BM_ORDER				<i>/ DMA buffers are written in the order of conversion.</i>\n
 *
 * <u>DMABLPerAnalogInput:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- EACH_ANALOG_INPUT_HAS_1_DMA_BUFF_LOC		<i>/ Allocates 1 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_2_DMA_BUFF_LOC		<i>/ Allocates 2 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_4_DMA_BUFF_LOC		<i>/ Allocates 4 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_8_DMA_BUFF_LOC		<i>/ Allocates 8 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_16_DMA_BUFF_LOC		<i>/ Allocates 16 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_32_DMA_BUFF_LOC		<i>/ Allocates 32 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_64_DMA_BUFF_LOC		<i>/ Allocates 64 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_128_DMA_BUFF_LOC	<i>/ Allocates 128 words of buffer to each analog input.</i>\n
 *
 * <u>DMAAddrIncrementRate:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- ADC_DMA_ADD_INC_1	<i>/ DMA address increment after conversion of each sample.</i> \n
 *	- .																						\n
 *	- .																								\n
 *	- ADC_DMA_ADD_INC_32	<i>/ DMA address increment after conversion of 32 samples.</i>  \n
 *
 * \Return	None
 *
 * \Notes	None
 *
 * \Example	SetupADC1(  ADC_10BIT_MODE,
 *                          ADC_REF_AVDD_AVSS,
 *                          SIMULTANEOUS_SAMPLING,
 *                          AUTO_SAMPLING,
 *                          SOC_SRC_AUTO_FASTEST_NoOpAMP_ADC1,
 *                          TAD_EQUAL_FASTEST_FOR_ADC1,
 *                          OUTPUT_FORMAT_U_INT,
 *                          ADC_CONTINUE_IN_IDEL,
 *                          RESULTS_BUFFER_16LOC_MODE,
 *                          INT_WHEN_16_NEW_RESULT_IN_BUFF,
 *                          DIS_DMA_TRANSFER,
 *                          DONT_CARE, DONT_CARE, DONT_CARE);
 *
 **********************************************************************************************************************/
#define SetupADC1(	ADCResolution,\
			ADCVoltageRef,\
			SamplingMode,\
			StartOfSampleSrc,\
			StartOfConversionSrc,\
			ConversionClockTAD,\
			OutputFormat,\
			IdelEN,\
			BufferFillMode,\
			ResultPerInt,\
			DMATransferEN,\
			DMABufferBuildMode,\
			DMABLPerAnalogInput,\
			DMAAddrIncrementRate)({\
						\
	static unsigned int ADC_Running;\
	ADC_Running = AD1CON1bits.ADON;\
	if(ADCResolution != AD1CON1bits.AD12B) {StopOperationADC1();}\
	AD1CON1bits.AD12B	= ADCResolution;\
	AD1CON2bits.VCFG	= ADCVoltageRef;\
	AD1CON1bits.SIMSAM	= SamplingMode;\
	AD1CON1bits.ASAM	= StartOfSampleSrc;\
	AD1CON1bits.SSRC	= StartOfConversionSrc&0b000000111;\
	AD1CON1bits.SSRCG	= (StartOfConversionSrc>>3)&0b000001;\
	AD1CON3bits.SAMC	= (StartOfConversionSrc>>4);\
	AD1CON3bits.ADCS	= ConversionClockTAD&0b011111111;\
	AD1CON3bits.ADRC	= (ConversionClockTAD>>8);\
	AD1CON1bits.FORM	= OutputFormat;\
	AD1CON1bits.ADSIDL	= IdelEN;\
	AD1CON2bits.BUFM	= BufferFillMode;\
	if(DMATransferEN) {\
		AD1CON2bits.SMPI = DMAAddrIncrementRate;\
		} else {\
			if(AD1CON1bits.SIMSAM){\
				AD1CON2bits.SMPI = ( (ResultPerInt)/(AD1CON2bits.CHPS+1) ) - 1;\
			} else {AD1CON2bits.SMPI = (ResultPerInt) - 1;}\
		}\
	AD1CON4bits.ADDMAEN	= DMATransferEN;\
	AD1CON1bits.ADDMABM	= DMABufferBuildMode;\
	AD1CON4bits.DMABL	= DMABLPerAnalogInput;\
	if(ADC_Running) StartOperationADC1();\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/***********************************************************************************************************************
 * \Function	void SetupADC1Channels(	ChannelToConvert,
 *					AltInputModeEN,
 *					Ch0ScanEN,
 *					Ch0ScanInputs,
 *					Ch0PosInputMuxA,Ch0NegInputMuxA,
 *					Ch0PosInputMuxB,Ch0NegInputMuxB,
 *					Ch123PosInputsMuxA, Ch123NegInputsMuxA,
 *					Ch123PosInputsMuxB, Ch123NegInputsMuxB)
 *
 * \Description		Configures ADC1 Channels Inputs.
 *
 * \PreCondition	None
 *
 * \Inputs
 * <u>ChannelToConvert:</u>\n
 *	- CONVERT_CH0_ONLY	\n
 *	- CONVERT_CH01		<i>/ Only if ADCResolution = ADC_10BIT_MODE.</i>\n
 *	- CONVERT_CH0123	<i>/ Only if ADCResolution = ADC_10BIT_MODE.</i>\n
 *
 * <u>AltInputModeEN:</u>\n
 *	- MUXA_ONLY		<i>/ Always uses channel input selects for MUXA.</i> \n
 *	- ALT_MUXA_MUXB		<i>/ Uses channel input selects for MUX A on the first sample and MUX B on the next sample.</i> \n
 *
 * <u>Ch0ScanEN:</u>\n
 *	- DIS_CH0_SCAN		<i>/ Does not scan inputs for CH0 (CH0 as Normal Channel).</i>\n
 *	- EN_CH0_SCAN		<i>/ Scans inputs for CH0 during Sample MUXA bit.</i>\n
 *
 * <u>Ch0ScanInputs:</u>\n
 *	- SCN_AN0		<i>/ Scan AN0.</i>	 \n
 *	- .						 \n
 *	- .						 \n
 *	- SCN_AN31		<i>/ Scan AN31.</i>	 \n <i>/ The AN16 through AN23 pins are not available for dsPIC33EP256MU806 (64-pin) devices.</i>\n
 *	- SCN_ALL_0_15		<i>/ Scan All Inputs From  AN0  To  AN15 inclusively.</i>\n
 *	- SCN_ALL_16_31		<i>/ Scan All Inputs From  AN16 To  AN31 inclusively.</i>\n
 *	- SCN_ALL_0_31		<i>/ Scan All Inputs From  AN0  To  AN31 inclusively.</i>\n
 *	- SCN_NONE		<i>/ Scan Nothing.
 *
 * <u>Ch0PosInputMuxA:</u>\n
 *	- ADC_MUXA_CH0_POS_AN0	<i>/ ADC MuxA CH0 Positive Input is AN0.</i>\n
 *	- .								\n
 *	- .								\n
 *	- ADC_MUXA_CH0_POS_AN31	<i>/ ADC MuxA CH0 Positive Input is AN31.</i>\n
 *
 * <u>Ch0NegInputMuxA:</u>\n
 *	- ADC_MUXA_CH0_NEG_VREFL	<i>/ ADC MuxA CH0 Negative Input is VRefL.</i>\n
 *	- ADC_MUXA_CH0_NEG_AN1		<i>/ ADC MuxA CH0 Negative Input is AN1.</i>\n
 *
 * <u>Ch0PosInputMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same as Ch0PosInputMuxA.
 *
 * <u>Ch0NegInputMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same as Ch0NegInputMuxA.
 *
 * <u>Ch123PosInputsMuxA:</u>\n
 *	- ADC_MUXA_CH123_POS_AN0_AN1_AN2	<i>/ ADC MuxA, AN0 => +CH1, AN1 => +CH2, AN2 => +CH2.</i>\n
 *	- ADC_MUXA_CH123_POS_AN3_AN4_AN5	<i>/ ADC MuxA, AN3 => +CH1, AN4 => +CH2, AN5 => +CH3.</i>\n
 *
 * <u>Ch123NegInputsMuxA:</u>\n
 *	- ADC_MUXA_CH123_NEG_VREFL		<i>/ ADC MuxA, VrefL => -CH1,-CH2,-CH3.</i>\n
 *	- ADC_MUXA_CH123_NEG_AN6_AN7_AN8	<i>/ ADC MuxA, AN6 => -CH1, AN7 => -CH2, AN8 => -CH3.</i>\n
 *	- ADC_MUXA_CH123_NEG_AN9_AN10_AN11	<i>/ ADC MuxA, AN9 => -CH1, AN10 => -CH2, AN11 => -CH3.</i>\n
 *
 * <u>Ch123PosInputsMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same As Ch123PosInputsMuxA.
 *
 * <u>Ch123NegInputsMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same As Ch123NegInputsMuxA.
 *
 * \Return	None
 *
 * \Notes	None
 *
 * \Example	SetupADC1Channels(CONVERT_CH0123,
 *				MUXA_ONLY,
 *				EN_CH0_SCAN,
 *				SCN_AN0 & SCN_AN1 & SCN_AN2 & SCN_AN7,
 *				DONT_CARE, ADC_MUXA_CH0_NEG_VREFL,
 *				DONT_CARE, DONT_CARE,
 *				ADC_MUXA_CH123_POS_3_4_5, ADC_MUXA_CH123_NEG_VREFL,
 *				DONT_CARE, DONT_CARE);
 *
 **********************************************************************************************************************/
#define SetupADC1Channels(	ChannelToConvert,\
				AltInputModeEN,\
				Ch0ScanEN,\
				Ch0ScanInputs,\
				Ch0PosInputMuxA,	Ch0NegInputMuxA,\
				Ch0PosInputMuxB,	Ch0NegInputMuxB,\
				Ch123PosInputsMuxA, Ch123NegInputsMuxA,\
				Ch123PosInputsMuxB, Ch123NegInputsMuxB)({\
									\
	AD1CON2bits.CHPS	= ChannelToConvert;\
	AD1CON2bits.ALTS	= AltInputModeEN;\
	AD1CON2bits.CSCNA	= Ch0ScanEN;\
	AD1CSSL			= (~(Ch0ScanInputs))&0x0000FFFF;\
	AD1CSSH			= (~(Ch0ScanInputs))>>16;\
	AD1CHS0bits.CH0SA	= Ch0PosInputMuxA;\
	AD1CHS0bits.CH0NA	= Ch0NegInputMuxA;\
	AD1CHS0bits.CH0SB	= Ch0PosInputMuxB;\
	AD1CHS0bits.CH0NB	= Ch0NegInputMuxB;\
	AD1CHS123bits.CH123SA	= Ch123PosInputsMuxA;\
	AD1CHS123bits.CH123NA	= Ch123NegInputsMuxA;\
	AD1CHS123bits.CH123SB	= Ch123PosInputsMuxB;\
	AD1CHS123bits.CH123NB	= Ch123NegInputsMuxB;\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define EnableIntADC1()			(_AD1IE = 1)
#define DisableIntADC1()		(_AD1IE = 0)
#define SetPriorityIntADC1(priority)	(_AD1IP = priority)
#define ADC1_INT_FLAG			_AD1IF

#define StartOperationADC1() ({\
	/* Enable ADC module and provide ADC stabilization delay */\
	static int SampMode;\
	SampMode = AD1CON1bits.ASAM;\
	AD1CON1bits.ASAM = MANUAL_SAMPLING;	/* To Prevent Any Conversions In The Stabilization Time */\
	AD1CON1bits.ADON = 1;\
	__delay_us(20);\
	AD1CON1bits.ASAM = SampMode;\
})

#define StopOperationADC1()		(AD1CON1bits.ADON = 0)	/* Turn Off ADC1 Module */

#define BeginSamplingADC1()		(AD1CON1bits.SAMP = 1)	/* If Manual Sampling Mode Is Selected */

#define BeginConvertingADC1()		(AD1CON1bits.SAMP = 0)	/* (End Sampling and Start Converting) If Manual Converting Mode Is Selected */

#define ADC1ConversionCompleted()	(AD1CON1bits.DONE)	/* Return 1 => ADC conversion cycle has completed */
								/* Return 0 => ADC conversion has not started or is in progress */

#define ADC1FillingSecBuffHalf()	(AD1CON2bits.BUFS)	/* This Macro is Used When BufferFillMode = RESULTS_BUFFER_2X8_ALT_MODE in SetupADCx */
								/* Return 1 => ADC is currently filling the second half of the buffer;
								 * the user application should access data in the first half of the buffer */
								/* Return 0 => ADC is currently filling the first half of the buffer;
								 * the user application should access data in the second half of the buffer */

/**********************************************************************************************************************/
/***************************************** ADC 2 Macro Functions & Defines ********************************************/
/**********************************************************************************************************************/
/***********************************************************************************************************************
 * \Function	void SetupADC2( ADCVoltageRef,
 *				SamplingMode,
 *				StartOfSampleSrc,
 *				StartOfConversionSrc,
 *				ConversionClockTAD,
 *				OutputFormat,
 *				IdelEN,
 *				BufferFillMode,
 *				ResultPerInt,
 *				DMATransferEN,
 *				DMABufferBuildMode,
 *				DMABLPerAnalogInput,
 *				DMAAddrIncrementRate)
 *
 * \Description		Configures ADC2 Operation Parameters.
 *
 * \PreCondition	You Must Set The SYS_FREQ and FCY in The Beginning of The Library Header File Before Using The Library.
 *
 * \Inputs
 * <u>ADCVoltageRef:</u>\n
 *	- ADC_REF_AVDD_AVSS	<i>/ A/D Voltage reference configuration	VrefH is	 AVdd	and VrefL is AVss.</i> \n
 *	- ADC_REF_VREFP_AVSS	<i>/ A/D Voltage reference configuration	VrefH is Ext Vref+	and	VrefL is AVss.</i> \n
 *	- ADC_REF_AVDD_VREFN	<i>/ A/D Voltage reference configuration	VrefH is	 AVdd	and	VrefL is Ext Vref-.</i> \n
 *	- ADC_REF_VREFP_VREFN	<i>/ A/D Voltage reference configuration	VrefH is Ext Vref+	and	VrefL is Ext Vref-.</i> \n
 *
 * <u>SamplingMode:</u>\n
 *	- SEQUENTIAL_SAMPLING	<i>/ Samples multiple channels individually in sequence.</i>\n
 *	- SIMULTANEOUS_SAMPLING	<i>/ only applicable when more than one channel are converted.</i>\n
 *
 * <u>StartOfSampleSrc:</u>\n
 *	- MANUAL_SAMPLING	<i>/ Sampling begins when SAMP bit is set, OR Use BeginSamplingADC1() Function.</i>\n
 *	- AUTO_SAMPLING		<i>/ Sampling begins immediately after last conversion; SAMP bit is auto-set.</i>\n
 *
 * <u>StartOfConversionSrc:</u>\n
 *	- SOC_SRC_MANUAL		<i>/ Clearing the (SAMP) bit ends sampling and starts conversion (Manual mode), OR Use BeginConversionADC1() Function.</i>\n
 *	- SOC_SRC_INT0			<i>/ Active transition on the INT0 pin ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_TMR3			<i>/ Timer 3 Interrupt ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_TMR5			<i>/ Timer 5 Interrupt ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_AUTO_AFTER_0TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- SOC_SRC_AUTO_AFTER_1TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- .																											 \n
 *	- .																											 \n
 *	- SOC_SRC_AUTO_AFTER_31TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- SOC_SRC_AUTO_FASTEST_NoOpAMP_ADC2	<i>/ Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if No OpAmp Outputs is Used As inputs for The ADC Module.</i>\n
 *	- SOC_SRC_AUTO_FASTEST_OpAMP	<i>/ Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if Any OpAmp Output is Used As inputs for The ADC Module.</i>\n
 *	- SOC_SRC_PRI_SEVT		<i>/ (Only dsPICs) PWM primary Special Event Trigger ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_SEC_SEVT		<i>/ (Only dsPICs) PWM secondary Special Event Trigger ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_PWM1			<i>/ (Only dsPICs) PWM Generator 1 primary trigger compare ends sampling and starts conversion.</i> \n
 *	- .				\n
 *	- .				\n
 *	- SOC_SRC_PWM7			<i>/ (Only dsPICs) PWM Generator 7 primary trigger compare ends sampling and starts conversion.</i> \n
 *	- ADC_CLK_CTMU			<i>/ CTMU ends sampling and starts conversion (if CTMU Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO12		<i>/ PTGO12 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO13		<i>/ PTGO13 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO14		<i>/ PTGO14 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO15		<i>/ PTGO15 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *
 * <u>ConversionClockTAD:</u>\n
 *	- TAD_EQUAL_INTERNAL_tRC	<i>/ TAD = ADC internal RC clock Interval.</i>\n
 *	- TAD_EQUAL_FASTEST_FOR_ADC2		<i>/ Automatically Set The Fastest Time Of TAD Respecting (FCY & ADC Resolution).</i>\n
 *	- TAD_EQUAL_1Tcy		\n
 *	- TAD_EQUAL_2Tcy		\n
 *	- .				\n
 *	- .				\n
 *	- TAD_EQUAL_256Tcy
 *
 * <u>OutputFormat:</u>\n
 *	- OUTPUT_FORMAT_U_INT		<i>/ A/D data format integer.</i>\n
 *	- OUTPUT_FORMAT_S_INT		<i>/ A/D data format signed integer.</i>\n
 *	- OUTPUT_FORMAT_U_FRACT		<i>/ A/D data format fractional.</i>\n
 *	- OUTPUT_FORMAT_S_FRACT		<i>/ A/D data format signed fractional.</i>\n
 *
 * <u>IdelEN:</u>\n
 *	- ADC_CONTINUE_IN_IDEL		<i>/ Continue QEI in Idle mode.</i>\n
 *	- ADC_STOP_IN_IDEL		<i>/ Stop QEI in Idle mode.</i>\n
 *
 * <u>BufferFillMode:</u> Only if DMA Transfer is Disabled Else Put (DONT_CARE).\n
 *	- RESULTS_BUFFER_16LOC_MODE		<i>/ Always starts filling the buffer from the Start address.</i>\n
 *	- RESULTS_BUFFER_2X8LOC_ALT_MODE	<i>/ Starts buffer filling the first half of the buffer on the first interrupt
 *										   and the second half of the buffer on the next interrupt And So On.</i>\n
 *
 * <u>ResultPerInt:</u> Only if DMA Transfer is Disabled Else Put (DONT_CARE).\n
 *	- INT_WHEN_1_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_2_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_3_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_4_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_5_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_6_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_7_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_8_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_9_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_10_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_11_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_12_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_13_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_14_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_15_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_16_NEW_RESULT_IN_BUFF
 *
 *	- a Value From 1 to 16			<i>/ if SamplingMode = SEQUENTIAL_SAMPLING   & BufferFillMode = RESULTS_BUFFER_16LOC_MODE.</i>\n
 *	- a Value Equ 2,4,6,8,10,12,14,16	<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_16LOC_MODE & Converting 2 Ch.</i>\n
 *	- a Value Equ 4,8,12,16			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_16LOC_MODE & Converting 4 Ch.</i>\n
 *	- a Value From 1 to 8			<i>/ if SamplingMode = SEQUENTIAL_SAMPLING   & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE.</i>\n
 *	- a Value Equ 2,4,6,8			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE & Converting 2 Ch.</i>\n
 *	- a Value Equ 4,8			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE & Converting 4 Ch.</i>\n
 *
 * <u>DMATransferEN:</u>\n
 *	- DIS_DMA_TRANSFER				<i>/ Conversion results are stored in ADCxBUF0 through ADCxBUFF registers; DMA will not be used.</i>\n
 *	- EN_DMA_TRANSFER				<i>/ Conversion results are stored in ADCxBUF0 register for transferring to RAM using DMA.</i>\n
 *
 * <u>DMABufferBuildMode:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- DMA_BM_SCATTR		<i>/ DMA buffers are written in Scatter/Gather mode.</i>\n
 *	- DMA_BM_ORDER		<i>/ DMA buffers are written in the order of conversion.</i>\n
 *
 * <u>DMABLPerAnalogInput:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- EACH_ANALOG_INPUT_HAS_1_DMA_BUFF_LOC		<i>/ Allocates 1 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_2_DMA_BUFF_LOC		<i>/ Allocates 2 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_4_DMA_BUFF_LOC		<i>/ Allocates 4 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_8_DMA_BUFF_LOC		<i>/ Allocates 8 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_16_DMA_BUFF_LOC		<i>/ Allocates 16 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_32_DMA_BUFF_LOC		<i>/ Allocates 32 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_64_DMA_BUFF_LOC		<i>/ Allocates 64 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_128_DMA_BUFF_LOC	<i>/ Allocates 128 words of buffer to each analog input.</i>\n
 *
 * <u>DMAAddrIncrementRate:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- ADC_DMA_ADD_INC_1				<i>/ DMA address increment after conversion of each sample.</i> \n
 *	- .						\n
 *	- .						\n
 *	- ADC_DMA_ADD_INC_16				<i>/ DMA address increment after conversion of 16 samples.</i>  \n
 *
 * \Return	None
 *
 * \Notes	The ADC2 module only supports 10-bit operation with 4 S&H.\n
 *			The AN16 through AN31 pins are not available for the ADC2 module.
 *
 * \Example	SetupADC2(	ADC_REF_AVDD_AVSS,
 *				SIMULTANEOUS_SAMPLING,
 *				AUTO_SAMPLING,
 *				SOC_SRC_AUTO_FASTEST_NoOpAMP_ADC2,
 *				TAD_EQUAL_FASTEST_FOR_ADC2,
 *				OUTPUT_FORMAT_U_INT,
 *				ADC_CONTINUE_IN_IDEL,
 *				RESULTS_BUFFER_16LOC_MODE,
 *				INT_WHEN_16_NEW_RESULT_IN_BUFF,
 *				DIS_DMA_TRANSFER,
 *				DONT_CARE, DONT_CARE, DONT_CARE);
 *
 **********************************************************************************************************************/
#define SetupADC2(	ADCVoltageRef,\
			SamplingMode,\
			StartOfSampleSrc,\
			StartOfConversionSrc,\
			ConversionClockTAD,\
			OutputFormat,\
			IdelEN,\
			BufferFillMode,\
			ResultPerInt,\
			DMATransferEN,\
			DMABufferBuildMode,\
			DMABLPerAnalogInput,\
			DMAAddrIncrementRate)({\
						\
	AD2CON2bits.VCFG	= ADCVoltageRef;\
	AD2CON1bits.SIMSAM	= SamplingMode;\
	AD2CON1bits.ASAM	= StartOfSampleSrc;\
	AD2CON1bits.SSRC	= StartOfConversionSrc&0b000000111;\
	AD2CON1bits.SSRCG	= (StartOfConversionSrc>>3)&0b000001;\
	AD2CON3bits.SAMC	= (StartOfConversionSrc>>4);\
	AD2CON3bits.ADCS	= ConversionClockTAD&0b011111111;\
	AD2CON3bits.ADRC	= (ConversionClockTAD>>8);\
	AD2CON1bits.FORM	= OutputFormat;\
	AD2CON1bits.ADSIDL	= IdelEN;\
	AD2CON2bits.BUFM	= BufferFillMode;\
	if(DMATransferEN) {\
		AD2CON2bits.SMPI = DMAAddrIncrementRate;\
		} else {\
			if(AD2CON1bits.SIMSAM){\
				AD2CON2bits.SMPI = ( (ResultPerInt)/(AD2CON2bits.CHPS+1) ) - 1;\
			} else {AD2CON2bits.SMPI = (ResultPerInt) - 1;}\
		}\
	AD2CON4bits.ADDMAEN	= DMATransferEN;\
	AD2CON1bits.ADDMABM	= DMABufferBuildMode;\
	AD2CON4bits.DMABL	= DMABLPerAnalogInput;\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/***********************************************************************************************************************
 * \Function	void SetupADC2Channels(	ChannelToConvert,
 *					AltInputModeEN,
 *					Ch0ScanEN,
 *					Ch0ScanInputs,
 *					Ch0PosInputMuxA,	Ch0NegInputMuxA,
 *					Ch0PosInputMuxB,	Ch0NegInputMuxB,
 *					Ch123PosInputsMuxA, Ch123NegInputsMuxA,
 *					Ch123PosInputsMuxB, Ch123NegInputsMuxB)
 *
 * \Description		Configures ADC2 Channels Inputs.
 *
 * \PreCondition	None
 *
 * \Inputs
 * <u>ChannelToConvert:</u>\n
 *	- CONVERT_CH0_ONLY		\n
 *	- CONVERT_CH01			<i>/ Only if ADCResolution = ADC_10BIT_MODE.</i>\n
 *	- CONVERT_CH0123		<i>/ Only if ADCResolution = ADC_10BIT_MODE.</i>\n
 *
 * <u>AltInputModeEN:</u>\n
 *	- MUXA_ONLY			<i>/ Always uses channel input selects for MUXA.</i> \n
 *	- ALT_MUXA_MUXB			<i>/ Uses channel input selects for MUX A on the first sample and MUX B on the next sample.</i> \n
 *
 * <u>Ch0ScanEN:</u>\n
 *	- DIS_CH0_SCAN			<i>/ Does not scan inputs for CH0 (CH0 as Normal Channel).</i>\n
 *	- EN_CH0_SCAN			<i>/ Scans inputs for CH0 during Sample MUXA bit.</i>\n
 *
 * <u>Ch0ScanInputs:</u>\n
 *	- SCN_AN0			<i>/ Scan AN0.</i>	 \n
 *	- .				\n
 *	- .				\n
 *	- SCN_AN15			<i>/ Scan AN15.</i>	 \n
 *	- SCN_ALL_0_15			<i>/ Scan All Inputs From  AN0  To  AN15 inclusively.</i>\n
 *	- SCN_NONE			<i>/ Scan Nothing.</i>\n
 *
 * <u>Ch0PosInputMuxA:</u>\n
 *	- ADC_MUXA_CH0_POS_AN0		<i>/ ADC MuxA CH0 Positive Input is AN0.</i>\n
 *	- .				\n
 *	- .				\n
 *	- ADC_MUXA_CH0_POS_AN15		<i>/ ADC MuxA CH0 Positive Input is AN15.</i>\n
 *
 * <u>Ch0NegInputMuxA:</u>\n
 *	- ADC_MUXA_CH0_NEG_VREFL	<i>/ ADC MuxA CH0 Negative Input is VRefL.</i>\n
 *	- ADC_MUXA_CH0_NEG_AN1		<i>/ ADC MuxA CH0 Negative Input is AN1.</i>\n
 *
 * <u>Ch0PosInputMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same as Ch0PosInputMuxA.
 *
 * <u>Ch0NegInputMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same as Ch0NegInputMuxA.
 *
 * <u>Ch123PosInputsMuxA:</u>\n
 *	- ADC_MUXA_CH123_POS_AN0_AN1_AN2	<i>/ ADC MuxA, AN0 => +CH1, AN1 => +CH2, AN2 => +CH2.</i>\n
 *	- ADC_MUXA_CH123_POS_AN3_AN4_AN5	<i>/ ADC MuxA, AN3 => +CH1, AN4 => +CH2, AN5 => +CH3.</i>\n
 *
 * <u>Ch123NegInputsMuxA:</u>\n
 *	- ADC_MUXA_CH123_NEG_VREFL		<i>/ ADC MuxA, VrefL => -CH1,-CH2,-CH3.</i>\n
 *	- ADC_MUXA_CH123_NEG_AN6_AN7_AN8	<i>/ ADC MuxA, AN6 => -CH1, AN7 => -CH2, AN8 => -CH3.</i>\n
 *	- ADC_MUXA_CH123_NEG_AN9_AN10_AN11	<i>/ ADC MuxA, AN9 => -CH1, AN10 => -CH2, AN11 => -CH3.</i>\n
 *
 * <u>Ch123PosInputsMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same As Ch123PosInputsMuxA.
 *
 * <u>Ch123NegInputsMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same As Ch123NegInputsMuxA.
 *
 * \Return	None
 *
 * \Notes	The AN16 through AN31 pins are not available for the ADC2 module.
 *
 * \Example	SetupADC2Channels(	CONVERT_CH0123,
 *					MUXA_ONLY,
 *					EN_CH0_SCAN,
 *					SCN_AN0 & SCN_AN1 & SCN_AN2 & SCN_AN7,
 *					DONT_CARE, ADC_MUXA_CH0_NEG_VREFL,
 *					DONT_CARE, DONT_CARE,
 *					ADC_MUXA_CH123_POS_3_4_5, ADC_MUXA_CH123_NEG_VREFL,
 *					DONT_CARE, DONT_CARE);
 *
 **********************************************************************************************************************/
#define SetupADC2Channels(	ChannelToConvert,\
				AltInputModeEN,\
				Ch0ScanEN,\
				Ch0ScanInputs,\
				Ch0PosInputMuxA,	Ch0NegInputMuxA,\
				Ch0PosInputMuxB,	Ch0NegInputMuxB,\
				Ch123PosInputsMuxA, Ch123NegInputsMuxA,\
				Ch123PosInputsMuxB, Ch123NegInputsMuxB)({\
									\
	AD2CON2bits.CHPS	= ChannelToConvert;\
	AD2CON2bits.ALTS	= AltInputModeEN;\
	AD2CON2bits.CSCNA	= Ch0ScanEN;\
	AD2CSSL			= (~(Ch0ScanInputs))&0x0000FFFF;\
	AD2CHS0bits.CH0SA	= Ch0PosInputMuxA;\
	AD2CHS0bits.CH0NA	= Ch0NegInputMuxA;\
	AD2CHS0bits.CH0SB	= Ch0PosInputMuxB;\
	AD2CHS0bits.CH0NB	= Ch0NegInputMuxB;\
	AD2CHS123bits.CH123SA	= Ch123PosInputsMuxA;\
	AD2CHS123bits.CH123NA	= Ch123NegInputsMuxA;\
	AD2CHS123bits.CH123SB	= Ch123PosInputsMuxB;\
	AD2CHS123bits.CH123NB	= Ch123NegInputsMuxB;\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define EnableIntADC2()                 (_AD2IE = 1)
#define DisableIntADC2()                (_AD2IE = 0)
#define SetPriorityIntADC2(priority)    (_AD2IP = priority)
#define ADC2_INT_FLAG			_AD2IF

#define StartOperationADC2() ({\
	/* Enable ADC module and provide ADC stabilization delay */\
	static int SampMode;\
	SampMode = AD2CON1bits.ASAM;\
	AD2CON1bits.ASAM = MANUAL_SAMPLING;	/* To Prevent Any Conversions In The Stabilization Time */\
	AD2CON1bits.ADON = 1;\
	__delay_us(20);\
	AD2CON1bits.ASAM = SampMode;\
})

#define StopOperationADC2()		(AD2CON1bits.ADON = 0)	/* Turn Off ADC2 Module */

#define BeginSamplingADC2()		(AD2CON1bits.SAMP = 1)	/* If Manual Sampling Mode Is Selected */

#define BeginConvertingADC2()		(AD2CON1bits.SAMP = 0)	/* (End Sampling and Start Converting) If Manual Converting Mode Is Selected */

#define ADC2ConversionCompleted()	(AD2CON1bits.DONE)	/* Return 1 => ADC conversion cycle has completed */
								/* Return 0 => ADC conversion has not started or is in progress */

#define ADC2FillingSecBuffHalf()	(AD2CON2bits.BUFS)	/* This Macro is Used When BufferFillMode = RESULTS_BUFFER_2X8_ALT_MODE in SetupADCx */
								/* Return 1 => ADC is currently filling the second half of the buffer;
								* the user application should access data in the first half of the buffer */
								/* Return 0 => ADC is currently filling the first half of the buffer;
								* the user application should access data in the second half of the buffer */
#endif

/**********************************************************************************************************************/
// Functions For (dsPIC33EPXXX(GM)3XX/6XX/7XX)  MCUs Group.
/**********************************************************************************************************************/
#ifdef GROUP2_dsPIC33EPXXX_GM_3XX_6XX_7XX

/**********************************************************************************************************************/
/***************************************** ADC 1 Macro Functions & Defines ********************************************/
/**********************************************************************************************************************/
/***********************************************************************************************************************
 * \Function	void SetupADC1( ADCResolution,
 *				ADCVoltageRef,
 *				SamplingMode,
 *				StartOfSampleSrc,
 *				StartOfConversionSrc,
 *				ConversionClockTAD,
 *				OutputFormat,
 *				IdelEN,
 *				BufferFillMode,
 *				ResultPerInt,
 *				DMATransferEN,
 *				DMABufferBuildMode,
 *				DMABLPerAnalogInput,
 *				DMAAddrIncrementRate)
 *
 * \Description		Configures ADC1 Operation Parameters.
 *
 * \PreCondition	You Must Set The SYS_FREQ and FCY in The Beginning of The Library Header File Before Using The Library.
 *
 * \Inputs
 * <u>ADCResolution:</u>\n
 *	- ADC_10BIT_MODE
 *	- ADC_12BIT_MODE
 *
 * <u>ADCVoltageRef:</u>\n
 *	- ADC_REF_AVDD_AVSS		<i>/ A/D Voltage reference configuration	VrefH is	 AVdd	and VrefL is AVss.</i> \n
 *	- ADC_REF_VREFP_AVSS		<i>/ A/D Voltage reference configuration	VrefH is Ext Vref+	and	VrefL is AVss.</i> \n
 *	- ADC_REF_AVDD_VREFN		<i>/ A/D Voltage reference configuration	VrefH is	 AVdd	and	VrefL is Ext Vref-.</i> \n
 *	- ADC_REF_VREFP_VREFN		<i>/ A/D Voltage reference configuration	VrefH is Ext Vref+	and	VrefL is Ext Vref-.</i> \n
 *
 * <u>SamplingMode:</u>\n
 *	- SEQUENTIAL_SAMPLING		<i>/ Samples multiple channels individually in sequence.</i>\n
 *	- SIMULTANEOUS_SAMPLING		<i>/ only applicable when more than one channel are converted.</i>\n
 *
 * <u>StartOfSampleSrc:</u>\n
 *	- MANUAL_SAMPLING		<i>/ Sampling begins when SAMP bit is set, OR Use BeginSamplingADC1() Function.</i>\n
 *	- AUTO_SAMPLING			<i>/ Sampling begins immediately after last conversion; SAMP bit is auto-set.</i>\n
 *
 * <u>StartOfConversionSrc:</u>\n
 *	- SOC_SRC_MANUAL		<i>/ Clearing the (SAMP) bit ends sampling and starts conversion (Manual mode), OR Use BeginConversionADC1() Function.</i>\n
 *	- SOC_SRC_INT0			<i>/ Active transition on the INT0 pin ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_TMR3			<i>/ Timer 3 Interrupt ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_TMR5			<i>/ Timer 5 Interrupt ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_AUTO_AFTER_0TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- SOC_SRC_AUTO_AFTER_1TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- .				\n
 *	- .				\n
 *	- SOC_SRC_AUTO_AFTER_31TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- SOC_SRC_AUTO_FASTEST_NoOpAMP_ADC1	<i>/ Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if No OpAmp Outputs is Used As inputs for The ADC Module.</i>\n
 *	- SOC_SRC_AUTO_FASTEST_OpAMP	<i>/ Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if Any OpAmp Output is Used As inputs for The ADC Module.</i>\n
 *	- SOC_SRC_PRI_SEVT		<i>/ (Only dsPICs) PWM primary Special Event Trigger ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_SEC_SEVT		<i>/ (Only dsPICs) PWM secondary Special Event Trigger ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_PWM1			<i>/ (Only dsPICs) PWM Generator 1 primary trigger compare ends sampling and starts conversion.</i> \n
 *	- .				\n
 *	- .				\n
 *	- SOC_SRC_PWM7			<i>/ (Only dsPICs) PWM Generator 7 primary trigger compare ends sampling and starts conversion.</i> \n
 *	- ADC_CLK_CTMU			<i>/ CTMU ends sampling and starts conversion (if CTMU Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO12		<i>/ PTGO12 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO13		<i>/ PTGO13 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO14		<i>/ PTGO14 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO15		<i>/ PTGO15 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *
 * <u>ConversionClockTAD:</u>\n
 *	- TAD_EQUAL_INTERNAL_tRC	<i>/ TAD = ADC internal RC clock Interval.</i>\n
 *	- TAD_EQUAL_FASTEST_FOR_ADC1		<i>/ Automatically Set The Fastest Time Of TAD Respecting (FCY & ADC Resolution).</i>\n
 *	- TAD_EQUAL_1Tcy		\n
 *	- TAD_EQUAL_2Tcy		\n
 *	- .				\n
 *	- .				\n
 *	- TAD_EQUAL_255Tcy
 *
 * <u>OutputFormat:</u>\n
 *	- OUTPUT_FORMAT_U_INT		<i>/ A/D data format integer.</i>\n
 *	- OUTPUT_FORMAT_S_INT		<i>/ A/D data format signed integer.</i>\n
 *	- OUTPUT_FORMAT_U_FRACT		<i>/ A/D data format fractional.</i>\n
 *	- OUTPUT_FORMAT_S_FRACT		<i>/ A/D data format signed fractional.</i>\n
 *
 * <u>IdelEN:</u>\n
 *	- ADC_CONTINUE_IN_IDEL		<i>/ Continue QEI in Idle mode.</i>\n
 *	- ADC_STOP_IN_IDEL		<i>/ Stop QEI in Idle mode.</i>\n
 *
 * <u>BufferFillMode:</u> Only if DMA Transfer is Disabled Else Put (DONT_CARE).\n
 *	- RESULTS_BUFFER_16LOC_MODE		<i>/ Always starts filling the buffer from the Start address.</i>\n
 *	- RESULTS_BUFFER_2X8LOC_ALT_MODE	<i>/ Starts buffer filling the first half of the buffer on the first interrupt
 *										   and the second half of the buffer on the next interrupt And So On.</i>\n
 *
 * <u>ResultPerInt:</u> Only if DMA Transfer is Disabled Else Put (DONT_CARE).\n
 *	- INT_WHEN_1_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_2_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_3_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_4_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_5_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_6_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_7_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_8_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_9_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_10_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_11_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_12_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_13_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_14_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_15_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_16_NEW_RESULT_IN_BUFF
 *
 *	- a Value From 1 to 16			<i>/ if SamplingMode = SEQUENTIAL_SAMPLING   & BufferFillMode = RESULTS_BUFFER_16LOC_MODE.</i>\n
 *	- a Value Equ 2,4,6,8,10,12,14,16	<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_16LOC_MODE & Converting 2 Ch.</i>\n
 *	- a Value Equ 4,8,12,16			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_16LOC_MODE & Converting 4 Ch.</i>\n
 *	- a Value From 1 to 8			<i>/ if SamplingMode = SEQUENTIAL_SAMPLING   & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE.</i>\n
 *	- a Value Equ 2,4,6,8			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE & Converting 2 Ch.</i>\n
 *	- a Value Equ 4,8			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE & Converting 2 Ch.</i>\n
 *
 * <u>DMATransferEN:</u>\n
 *	- DIS_DMA_TRANSFER	\n
 *	- EN_DMA_TRANSFER
 *
 * <u>DMABufferBuildMode:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- DMA_BM_SCATTR		<i>/ DMA buffers are written in Scatter/Gather mode.</i>\n
 *	- DMA_BM_ORDER		<i>/ DMA buffers are written in the order of conversion.</i>\n
 *
 * <u>DMABLPerAnalogInput:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- EACH_ANALOG_INPUT_HAS_1_DMA_BUFF_LOC		<i>/ Allocates 1 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_2_DMA_BUFF_LOC		<i>/ Allocates 2 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_4_DMA_BUFF_LOC		<i>/ Allocates 4 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_8_DMA_BUFF_LOC		<i>/ Allocates 8 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_16_DMA_BUFF_LOC		<i>/ Allocates 16 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_32_DMA_BUFF_LOC		<i>/ Allocates 32 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_64_DMA_BUFF_LOC		<i>/ Allocates 64 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_128_DMA_BUFF_LOC	<i>/ Allocates 128 words of buffer to each analog input.</i>\n
 *
 * <u>DMAAddrIncrementRate:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- ADC_DMA_ADD_INC_1				<i>/ DMA address increment after conversion of each sample.</i> \n
 *	- .						\n
 *	- .						\n
 *	- ADC_DMA_ADD_INC_32				<i>/ DMA address increment after conversion of 32 samples.</i>  \n
 *
 * \Return	None
 *
 * \Notes	None
 *
 * \Example	SetupADC1(	ADC_10BIT_MODE,
 *				ADC_REF_AVDD_AVSS,
 *				SIMULTANEOUS_SAMPLING,
 *				AUTO_SAMPLING,
 *				SOC_SRC_AUTO_FASTEST_NoOpAMP_ADC1,
 *				TAD_EQUAL_FASTEST_FOR_ADC1,
 *				OUTPUT_FORMAT_U_INT,
 *				ADC_CONTINUE_IN_IDEL,
 *				RESULTS_BUFFER_16LOC_MODE,
 *				INT_WHEN_16_NEW_RESULT_IN_BUFF,
 *				DIS_DMA_TRANSFER,
 *				DONT_CARE, DONT_CARE, DONT_CARE);
 *
 **********************************************************************************************************************/
#define SetupADC1(	ADCResolution,\
			ADCVoltageRef,\
			SamplingMode,\
			StartOfSampleSrc,\
			StartOfConversionSrc,\
			ConversionClockTAD,\
			OutputFormat,\
			IdelEN,\
			BufferFillMode,\
			ResultPerInt,\
			DMATransferEN,\
			DMABufferBuildMode,\
			DMABLPerAnalogInput,\
			DMAAddrIncrementRate)({\
						\
	static unsigned int ADC_Running;\
	ADC_Running = AD1CON1bits.ADON;\
	if(ADCResolution != AD1CON1bits.AD12B) {StopOperationADC1();}\
	AD1CON1bits.AD12B	= ADCResolution;\
	AD1CON2bits.VCFG	= ADCVoltageRef;\
	AD1CON1bits.SIMSAM	= SamplingMode;\
	AD1CON1bits.ASAM	= StartOfSampleSrc;\
	AD1CON1bits.SSRC	= StartOfConversionSrc&0b000000111;\
	AD1CON1bits.SSRCG	= (StartOfConversionSrc>>3)&0b000001;\
	AD1CON3bits.SAMC	= (StartOfConversionSrc>>4);\
	AD1CON3bits.ADCS	= ConversionClockTAD&0b011111111;\
	AD1CON3bits.ADRC	= (ConversionClockTAD>>8);\
	AD1CON1bits.FORM	= OutputFormat;\
	AD1CON1bits.ADSIDL	= IdelEN;\
	AD1CON2bits.BUFM	= BufferFillMode;\
	if(DMATransferEN) {\
		AD1CON2bits.SMPI = DMAAddrIncrementRate;\
		} else {\
			if(AD1CON1bits.SIMSAM){\
				AD1CON2bits.SMPI = ( (ResultPerInt)/(AD1CON2bits.CHPS+1) ) - 1;\
			} else {AD1CON2bits.SMPI = (ResultPerInt) - 1;}\
		}\
	AD1CON4bits.ADDMAEN	= DMATransferEN;\
	AD1CON1bits.ADDMABM	= DMABufferBuildMode;\
	AD1CON4bits.DMABL	= DMABLPerAnalogInput;\
	if(ADC_Running) StartOperationADC1();\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/***********************************************************************************************************************
 * \Function	void SetupADC1Channels(	ChannelToConvert,
 *					AltInputModeEN,
 *					Ch0ScanEN,
 *					Ch0ScanInputs,
 *					Ch0PosInputMuxA,Ch0NegInputMuxA,
 *					Ch0PosInputMuxB,Ch0NegInputMuxB,
 *					Ch123PosInputsMuxA, Ch123NegInputsMuxA,
 *					Ch123PosInputsMuxB, Ch123NegInputsMuxB)
 *
 * \Description		Configures ADC1 Channels Inputs.
 *
 * \PreCondition	None
 *
 * \Inputs
 * <u>ChannelToConvert:</u> \n
 *	- CONVERT_CH0_ONLY		\n
 *	- CONVERT_CH01			<i>/ Only if ADCResolution = ADC_10BIT_MODE.</i>\n
 *	- CONVERT_CH0123		<i>/ Only if ADCResolution = ADC_10BIT_MODE.</i>\n
 *
 * <u>AltInputModeEN:</u>\n
 *	- MUXA_ONLY			<i>/ Always uses channel input selects for MUXA.</i> \n
 *	- ALT_MUXA_MUXB			<i>/ Uses channel input selects for MUX A on the first sample and MUX B on the next sample.</i> \n
 *
 * <u>Ch0ScanEN:</u>\n
 *	- DIS_CH0_SCAN			<i>/ Does not scan inputs for CH0 (CH0 as Normal Channel).</i>\n
 *	- EN_CH0_SCAN			<i>/ Scans inputs for CH0 during Sample MUXA bit.</i>\n
 *
 * <u>Ch0ScanInputs:</u>\n
 *	- SCN_AN0			<i>/ Scan AN0.</i>	 \n
 *	- .				\n
 *	- .				\n
 *	- SCN_AN23			<i>/ Scan AN23.</i>	 \n
 *	- SCN_OA1AN3			<i>/ Scan AN3 or OpAmp1 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- SCN_OA2AN0			<i>/ Scan AN0 or OpAmp2 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- SCN_OA3AN6			<i>/ Scan AN6 or OpAmp3 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- SCN_AN27			<i>/ Scan AN27.</i>\n
 *	- SCN_AN28			<i>/ Scan AN28.</i>\n
 *	- SCN_AN29			<i>/ Scan AN29.</i>\n
 *	- SCN_AN30			<i>/ Scan AN30.</i>\n
 *	- SCN_AN31			<i>/ Scan AN31.</i>\n
 *	- SCN_ALL_0_15			<i>/ Scan All Inputs From  AN0  To  AN15 inclusively.</i>\n
 *	- SCN_ALL_16_31			<i>/ Scan All Inputs From  AN16 To  AN31 inclusively (Note: AN24=OA1AN3 & AN25=OA2AN0 & AN26=OA3AN6).</i>\n
 *	- SCN_ALL_0_31			<i>/ Scan All Inputs From  AN0  To  AN31 inclusively (Note: AN24=OA1AN3 & AN25=OA2AN0 & AN26=OA3AN6).</i>\n
 *	- SCN_NONE			<i>/ Scan Nothing.</i>\n
 *
 * <u>Ch0PosInputMuxA:</u>\n
 *	- ADC_MUXA_CH0_POS_OA2AN0	<i>/ ADC MuxA CH0 Positive Input is AN0 or OpAmp2 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- ADC_MUXA_CH0_POS_AN1		<i>/ ADC MuxA CH0 Positive Input is AN1.</i>\n
 *	- ADC_MUXA_CH0_POS_AN2		<i>/ ADC MuxA CH0 Positive Input is AN2.</i>\n
 *	- ADC_MUXA_CH0_POS_OA1AN3	<i>/ ADC MuxA CH0 Positive Input is AN3 or OpAmp1 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- ADC_MUXA_CH0_POS_AN4		<i>/ ADC MuxA CH0 Positive Input is AN4.</i>\n
 *	- ADC_MUXA_CH0_POS_AN5		<i>/ ADC MuxA CH0 Positive Input is AN5.</i>\n
 *	- ADC_MUXA_CH0_POS_OA3AN6	<i>/ ADC MuxA CH0 Positive Input is AN6 or OpAmp3 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- ADC_MUXA_CH0_POS_AN7		<i>/ ADC MuxA CH0 Positive Input is AN7.</i>\n
 *	- ADC_MUXA_CH0_POS_AN8		<i>/ ADC MuxA CH0 Positive Input is AN8.</i>\n
 *	- ADC_MUXA_CH0_POS_AN9		<i>/ ADC MuxA CH0 Positive Input is AN9.</i>\n
 *	- ADC_MUXA_CH0_POS_AN10		<i>/ ADC MuxA CH0 Positive Input is AN10.</i>\n
 *	- ADC_MUXA_CH0_POS_AN11		<i>/ ADC MuxA CH0 Positive Input is AN11.</i>\n
 *	- ADC_MUXA_CH0_POS_AN12		<i>/ ADC MuxA CH0 Positive Input is AN12.</i>\n
 *	- ADC_MUXA_CH0_POS_AN13		<i>/ ADC MuxA CH0 Positive Input is AN13.</i>\n
 *	- ADC_MUXA_CH0_POS_AN14		<i>/ ADC MuxA CH0 Positive Input is AN14.</i>\n
 *	- ADC_MUXA_CH0_POS_AN15		<i>/ ADC MuxA CH0 Positive Input is AN15.</i>\n
 *	- ADC_MUXA_CH0_POS_AN16		<i>/ ADC MuxA CH0 Positive Input is AN16.</i>\n
 *	- ADC_MUXA_CH0_POS_AN17		<i>/ ADC MuxA CH0 Positive Input is AN17.</i>\n
 *	- ADC_MUXA_CH0_POS_AN18		<i>/ ADC MuxA CH0 Positive Input is AN18.</i>\n
 *	- ADC_MUXA_CH0_POS_AN19		<i>/ ADC MuxA CH0 Positive Input is AN19.</i>\n
 *	- ADC_MUXA_CH0_POS_AN20		<i>/ ADC MuxA CH0 Positive Input is AN20.</i>\n
 *	- ADC_MUXA_CH0_POS_AN21		<i>/ ADC MuxA CH0 Positive Input is AN21.</i>\n
 *	- ADC_MUXA_CH0_POS_AN22		<i>/ ADC MuxA CH0 Positive Input is AN22.</i>\n
 *	- ADC_MUXA_CH0_POS_AN23		<i>/ ADC MuxA CH0 Positive Input is AN23.</i>\n
 *	- ADC_MUXA_CH0_POS_AN24		<i>/ ADC MuxA CH0 Positive Input is AN24.</i>\n
 *	- ADC_MUXA_CH0_POS_OA5AN25	<i>/ ADC MuxA CH0 Positive Input is AN25 or OpAmp5 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- ADC_MUXA_CH0_POS_AN26		<i>/ ADC MuxA CH0 Positive Input is AN26.</i>\n
 *	- ADC_MUXA_CH0_POS_AN27		<i>/ ADC MuxA CH0 Positive Input is AN27.</i>\n
 *	- ADC_MUXA_CH0_POS_AN28		<i>/ ADC MuxA CH0 Positive Input is AN28.</i>\n
 *	- ADC_MUXA_CH0_POS_AN29		<i>/ ADC MuxA CH0 Positive Input is AN29.</i>\n
 *	- ADC_MUXA_CH0_POS_AN30		<i>/ ADC MuxA CH0 Positive Input is AN30.</i>\n
 *	- ADC_MUXA_CH0_POS_AN31		<i>/ ADC MuxA CH0 Positive Input is AN31.</i>\n
 *	- ADC_MUXA_CH0_POS_AN32		<i>/ ADC MuxA CH0 Positive Input is AN32 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN33		<i>/ ADC MuxA CH0 Positive Input is AN33 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN34		<i>/ ADC MuxA CH0 Positive Input is AN34 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN35		<i>/ ADC MuxA CH0 Positive Input is AN35 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN36		<i>/ ADC MuxA CH0 Positive Input is AN36 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN37		<i>/ ADC MuxA CH0 Positive Input is AN37 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN38		<i>/ ADC MuxA CH0 Positive Input is AN38 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN39		<i>/ ADC MuxA CH0 Positive Input is AN39 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN40		<i>/ ADC MuxA CH0 Positive Input is AN40 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN41		<i>/ ADC MuxA CH0 Positive Input is AN41 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN42		<i>/ ADC MuxA CH0 Positive Input is AN42 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN43		<i>/ ADC MuxA CH0 Positive Input is AN43 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN44		<i>/ ADC MuxA CH0 Positive Input is AN44 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN45		<i>/ ADC MuxA CH0 Positive Input is AN45 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN46		<i>/ ADC MuxA CH0 Positive Input is AN46 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN47		<i>/ ADC MuxA CH0 Positive Input is AN47 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN48		<i>/ ADC MuxA CH0 Positive Input is AN48 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN49		<i>/ ADC MuxA CH0 Positive Input is AN49 (ADC1 Module & 10 bit ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN50		<i>/ ADC MuxA CH0 Positive Input is AN50 (ADC1 Module ONLY!) (reserved).</i>\n
 *	- ADC_MUXA_CH0_POS_AN51		<i>/ ADC MuxA CH0 Positive Input is AN51 (ADC1 Module ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN52		<i>/ ADC MuxA CH0 Positive Input is AN52 (ADC1 Module ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN53		<i>/ ADC MuxA CH0 Positive Input is AN53 (ADC1 Module ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN54		<i>/ ADC MuxA CH0 Positive Input is AN54 (ADC1 Module ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN55		<i>/ ADC MuxA CH0 Positive Input is AN55 (ADC1 Module ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN56		<i>/ ADC MuxA CH0 Positive Input is AN56 (ADC1 Module ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN57		<i>/ ADC MuxA CH0 Positive Input is AN57 (ADC1 Module ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN58		<i>/ ADC MuxA CH0 Positive Input is AN58 (ADC1 Module ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN59		<i>/ ADC MuxA CH0 Positive Input is AN59 (ADC1 Module ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN60		<i>/ ADC MuxA CH0 Positive Input is AN60 (ADC1 Module ONLY!).</i>\n
 *	- ADC_MUXA_CH0_POS_AN61		<i>/ ADC MuxA CH0 Positive Input is AN61 (ADC1 Module ONLY!) (reserved).</i>\n
 *	- ADC_MUXA_CH0_POS_AN62_CTMU_TEMP	<i>/ ADC MuxA CH0 Positive Input is AN62 (ADC1 Module ONLY!) CTMU temperature voltage.</i>\n
 *	- ADC_MUXA_CH0_POS_AN63_UNCONNECTED	<i>/ ADC MuxA CH0 Positive Input is AN63 (ADC1 Module ONLY!) unconnected.</i>\n
 *
 * <u>Ch0NegInputMuxA:</u>\n
 *	- ADC_MUXA_CH0_NEG_VREFL	<i>/ ADC MuxA CH0 Negative Input is VRefL.</i>\n
 *	- ADC_MUXA_CH0_NEG_AN1		<i>/ ADC MuxA CH0 Negative Input is AN1.</i>\n
 *
 * <u>Ch0PosInputMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same as Ch0PosInputMuxA.
 *
 * <u>Ch0NegInputMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same as Ch0NegInputMuxA.
 *
 * <u>Ch123PosInputsMuxA:</u>\n
 *	- ADC_MUXA_CH123_POS_AN0_AN1_AN2		<i>/ ADC MuxA, AN0 => +CH1, AN1 => +CH2, AN2 => +CH.</i>\n
 *	- ADC_MUXA_CH123_POS_AN3_AN4_AN5		<i>/ ADC MuxA, AN3 => +CH1, AN4 => +CH2, AN5 => +CH.</i>\n
 *	- ADC_MUXA_CH123_POS_OA1AN3_OA2AN0_OA3AN6	<i>/ ADC MuxA, AN3 or OpAmp1 => +CH1, AN0 or OpAmp2 => +CH2, AN6 or OpAmp3 => +CH.</i>\n
 *	- ADC_MUXA_CH123_POS_OA1AN3_OA2AN0_OA5AN25	<i>/ ADC MuxA, AN3 or OpAmp1 => +CH1, AN0 or OpAmp2 => +CH2, AN25 or OpAmp5 => +CH.</i>\n
 *	- ADC_MUXA_CH123_POS_OA2AN0_OA5AN25_OA3AN6	<i>/ ADC MuxA, AN0 or OpAmp2 => +CH1, AN25 or OpAmp5 => +CH2, AN6 or OpAmp6 => +CH.</i>\n
 *
 * <u>Ch123NegInputsMuxA:</u>\n
 *	- ADC_MUXA_CH123_NEG_VREFL		<i>/ ADC MuxA, VrefL => -CH1,-CH2,-CH3.</i>\n
 *	- ADC_MUXA_CH123_NEG_AN6_AN7_AN8	<i>/ ADC MuxA, AN6 => -CH1, AN7 => -CH2, AN8 => -CH3.</i>\n
 *	- ADC_MUXA_CH123_NEG_AN9_AN10_AN11	<i>/ ADC MuxA, AN9 => -CH1, AN10 => -CH2, AN11 => -CH3.</i>\n
 *
 * <u>Ch123PosInputsMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same As Ch123PosInputsMuxA.
 *
 * <u>Ch123NegInputsMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same As Ch123NegInputsMuxA.
 *
 * \Return	None
 *
 * \Notes	analog inputs are shared with op amp inputs and outputs, comparator inputs and external voltage references. When
 *		op amp/comparator functionality is enabled, or an external voltage reference is used, the analog input that shares
 *		that pin is no longer available.
 *
 * \Example	SetupADC1Channels(	CONVERT_CH0123,
 *					MUXA_ONLY,
 *					EN_CH0_SCAN,
 *					SCN_AN0 & SCN_AN1 & SCN_AN2 & SCN_AN7,
 *					DONT_CARE, ADC_MUXA_CH0_NEG_VREFL,
 *					DONT_CARE, DONT_CARE,
 *					ADC_MUXA_CH0_POS_OA3AN6, ADC_MUXA_CH123_NEG_VREFL,
 *					DONT_CARE, DONT_CARE);
 *
 **********************************************************************************************************************/
#define SetupADC1Channels(	ChannelToConvert,\
				AltInputModeEN,\
				Ch0ScanEN,\
				Ch0ScanInputs,\
				Ch0PosInputMuxA,Ch0NegInputMuxA,\
				Ch0PosInputMuxB,Ch0NegInputMuxB,\
				Ch123PosInputsMuxA, Ch123NegInputsMuxA,\
				Ch123PosInputsMuxB, Ch123NegInputsMuxB)({\
									\
	AD1CON2bits.CHPS	= ChannelToConvert;\
	AD1CON2bits.ALTS	= AltInputModeEN;\
	AD1CON2bits.CSCNA	= Ch0ScanEN;\
	AD1CSSL			= (~(Ch0ScanInputs))&0x0000FFFF;\
	AD1CSSH			= (~(Ch0ScanInputs))>>16;\
	AD1CHS0bits.CH0SA	= Ch0PosInputMuxA;\
	AD1CHS0bits.CH0NA	= Ch0NegInputMuxA;\
	AD1CHS0bits.CH0SB	= Ch0PosInputMuxB;\
	AD1CHS0bits.CH0NB	= Ch0NegInputMuxB;\
	AD1CHS123bits.CH123SA0	= (Ch123PosInputsMuxA)&0b001;\
	AD1CHS123bits.CH123SA1	= ((Ch123PosInputsMuxA)>>1)&0b01;\
	AD1CHS123bits.CH123SA2	= ((Ch123PosInputsMuxA)>>2);\
	AD1CHS123bits.CH123NA	= Ch123NegInputsMuxA;\
	AD1CHS123bits.CH123SB0	= (Ch123PosInputsMuxB)&0b001;\
	AD1CHS123bits.CH123SB1	= ((Ch123PosInputsMuxB)>>1)&0b01;\
	AD1CHS123bits.CH123SB2	= ((Ch123PosInputsMuxB)>>2);\
	AD1CHS123bits.CH123NB	= Ch123NegInputsMuxB;\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define EnableIntADC1()			(_AD1IE = 1)
#define DisableIntADC1()                (_AD1IE = 0)
#define SetPriorityIntADC1(priority)    (_AD1IP = priority)
#define ADC1_INT_FLAG			_AD1IF

#define StartOperationADC1() ({\
	/* Enable ADC module and provide ADC stabilization delay */\
	static int SampMode;\
	SampMode = AD1CON1bits.ASAM;\
	AD1CON1bits.ASAM = MANUAL_SAMPLING;	/* To Prevent Any Conversions In The Stabilization Time */\
	AD1CON1bits.ADON = 1;\
	__delay_us(20);\
	AD1CON1bits.ASAM = SampMode;\
})

#define StopOperationADC1()			(AD1CON1bits.ADON = 0)	/* Turn Off ADC1 Module */

#define BeginSamplingADC1()			(AD1CON1bits.SAMP = 1)	/* If Manual Sampling Mode Is Selected */

#define BeginConvertingADC1()			(AD1CON1bits.SAMP = 0)	/* (End Sampling and Start Converting) If Manual Converting Mode Is Selected */

#define EnterOffsetCalibrationModeADC1()	(AD1CON2bits.OFFCAL =1)	/* + and - inputs of channel Sample-and-Hold are connected to AVSS */

#define ExitOffsetCalibrationModeADC1()		(AD1CON2bits.OFFCAL =0)	/* + and - inputs of channel Sample-and-Hold Normal */

#define ADC1ConversionCompleted()		(AD1CON1bits.DONE)	/* Return 1 => ADC conversion cycle has completed */
									/* Return 0 => ADC conversion has not started or is in progress */

#define ADC1FillingSecBuffHalf()		(AD1CON2bits.BUFS)	/* This Macro is Used When BufferFillMode = RESULTS_BUFFER_2X8_ALT_MODE in SetupADCx */
									/* Return 1 => ADC is currently filling the second half of the buffer;
									* the user application should access data in the first half of the buffer */
									/* Return 0 => ADC is currently filling the first half of the buffer;
									* the user application should access data in the second half of the buffer */

/**********************************************************************************************************************/
/***************************************** ADC 2 Macro Functions & Defines ********************************************/
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * \Function	void SetupADC2( ADCResolution,
 *				ADCVoltageRef,
 *				SamplingMode,
 *				StartOfSampleSrc,
 *				StartOfConversionSrc,
 *				ConversionClockTAD,
 *				OutputFormat,
 *				IdelEN,
 *				BufferFillMode,
 *				ResultPerInt,
 *				DMATransferEN,
 *				DMABufferBuildMode,
 *				DMABLPerAnalogInput,
 *				DMAAddrIncrementRate)
 *
 * \Description		Configures ADC2 Operation Parameters.
 *
 * \PreCondition	You Must Set The SYS_FREQ and FCY in The Beginning of The Library Header File Before Using The Library.
 *
 * \Inputs
 * <u>ADCResolution:</u>\n
 *	- ADC_10BIT_MODE
 *	- ADC_12BIT_MODE
 *
 * <u>ADCVoltageRef:</u>\n
 *	- ADC_REF_AVDD_AVSS		<i>/ A/D Voltage reference configuration	VrefH is	 AVdd	and VrefL is AVss.</i> \n
 *	- ADC_REF_VREFP_AVSS		<i>/ A/D Voltage reference configuration	VrefH is Ext Vref+	and	VrefL is AVss.</i> \n
 *
 * <u>SamplingMode:</u>\n
 *	- SEQUENTIAL_SAMPLING		<i>/ Samples multiple channels individually in sequence.</i>\n
 *	- SIMULTANEOUS_SAMPLING		<i>/ Only applicable when more than one channel are converted.</i>\n
 *
 * <u>StartOfSampleSrc:</u>\n
 *	- MANUAL_SAMPLING		<i>/ Sampling begins when SAMP bit is set, OR Use BeginSamplingADC1() Function.</i>\n
 *	- AUTO_SAMPLING			<i>/ Sampling begins immediately after last conversion; SAMP bit is auto-set.</i>\n
 *
 * <u>StartOfConversionSrc:</u>\n
 *	- SOC_SRC_MANUAL		<i>/ Clearing the (SAMP) bit ends sampling and starts conversion (Manual mode), OR Use BeginConversionADC1() Function.</i>\n
 *	- SOC_SRC_INT0			<i>/ Active transition on the INT0 pin ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_TMR3			<i>/ Timer 3 Interrupt ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_TMR5			<i>/ Timer 5 Interrupt ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_AUTO_AFTER_0TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- SOC_SRC_AUTO_AFTER_1TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- .				\n
 *	- .				\n
 *	- SOC_SRC_AUTO_AFTER_31TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- SOC_SRC_AUTO_FASTEST_NoOpAMP_ADC2	<i>/ Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if No OpAmp Outputs is Used As inputs for The ADC Module.</i>\n
 *	- SOC_SRC_AUTO_FASTEST_OpAMP	<i>/ Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if Any OpAmp Output is Used As inputs for The ADC Module.</i>\n
 *	- SOC_SRC_PRI_SEVT		<i>/ (Only dsPICs) PWM primary Special Event Trigger ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_SEC_SEVT		<i>/ (Only dsPICs) PWM secondary Special Event Trigger ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_PWM1			<i>/ (Only dsPICs) PWM Generator 1 primary trigger compare ends sampling and starts conversion.</i> \n
 *	- .				\n
 *	- .				\n
 *	- SOC_SRC_PWM7			<i>/ (Only dsPICs) PWM Generator 7 primary trigger compare ends sampling and starts conversion.</i> \n
 *	- ADC_CLK_CTMU			<i>/ CTMU ends sampling and starts conversion (if CTMU Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO12		<i>/ PTGO12 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO13		<i>/ PTGO13 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO14		<i>/ PTGO14 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO15		<i>/ PTGO15 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *
 * <u>ConversionClockTAD:</u>\n
 *	- TAD_EQUAL_INTERNAL_tRC	<i>/ TAD = ADC internal RC clock Interval.</i>\n
 *	- TAD_EQUAL_FASTEST_FOR_ADC2		<i>/ Automatically Set The Fastest Time Of TAD Respecting (FCY & ADC Resolution).</i>\n
 *	- TAD_EQUAL_1Tcy		\n
 *	- TAD_EQUAL_2Tcy		\n
 *	- .				\n
 *	- .				\n
 *	- TAD_EQUAL_255Tcy
 *
 * <u>OutputFormat:</u>\n
 *	- OUTPUT_FORMAT_U_INT		<i>/ A/D data format integer.</i>\n
 *	- OUTPUT_FORMAT_S_INT		<i>/ A/D data format signed integer.</i>\n
 *	- OUTPUT_FORMAT_U_FRACT		<i>/ A/D data format fractional.</i>\n
 *	- OUTPUT_FORMAT_S_FRACT		<i>/ A/D data format signed fractional.</i>\n
 *
 * <u>IdelEN:</u>\n
 *	- ADC_CONTINUE_IN_IDEL		<i>/ Continue QEI in Idle mode.</i>\n
 *	- ADC_STOP_IN_IDEL		<i>/ Stop QEI in Idle mode.</i>\n
 *
 * <u>BufferFillMode:</u> Only if DMA Transfer is Disabled Else Put (DONT_CARE).\n
 *	- RESULTS_BUFFER_16LOC_MODE		<i>/ Always starts filling the buffer from the Start address.</i>\n
 *	- RESULTS_BUFFER_2X8LOC_ALT_MODE	<i>/ Starts buffer filling the first half of the buffer on the first interrupt
 *										   and the second half of the buffer on the next interrupt And So On.</i>\n
 *
 * <u>ResultPerInt:</u> Only if DMA Transfer is Disabled Else Put (DONT_CARE).\n
 *	- INT_WHEN_1_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_2_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_3_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_4_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_5_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_6_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_7_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_8_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_9_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_10_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_11_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_12_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_13_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_14_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_15_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_16_NEW_RESULT_IN_BUFF
 *
 *	- a Value From 1 to 16			<i>/ if SamplingMode = SEQUENTIAL_SAMPLING   & BufferFillMode = RESULTS_BUFFER_16LOC_MODE.</i>\n
 *	- a Value Equ 2,4,6,8,10,12,14,16	<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_16LOC_MODE & Converting 2 Ch.</i>\n
 *	- a Value Equ 4,8,12,16			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_16LOC_MODE & Converting 4 Ch.</i>\n
 *	- a Value From 1 to 8			<i>/ if SamplingMode = SEQUENTIAL_SAMPLING   & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE.</i>\n
 *	- a Value Equ 2,4,6,8			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE & Converting 2 Ch.</i>\n
 *	- a Value Equ 4,8			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE & Converting 2 Ch.</i>\n
 *
 * <u>DMATransferEN:</u>\n
 *	- DIS_DMA_TRANSFER	\n
 *	- EN_DMA_TRANSFER
 *
 * <u>DMABufferBuildMode:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- DMA_BM_SCATTR				<i>/ DMA buffers are written in Scatter/Gather mode.</i>\n
 *	- DMA_BM_ORDER				<i>/ DMA buffers are written in the order of conversion.</i>\n
 *
 * <u>DMABLPerAnalogInput:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- EACH_ANALOG_INPUT_HAS_1_DMA_BUFF_LOC		<i>/ Allocates 1 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_2_DMA_BUFF_LOC		<i>/ Allocates 2 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_4_DMA_BUFF_LOC		<i>/ Allocates 4 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_8_DMA_BUFF_LOC		<i>/ Allocates 8 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_16_DMA_BUFF_LOC		<i>/ Allocates 16 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_32_DMA_BUFF_LOC		<i>/ Allocates 32 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_64_DMA_BUFF_LOC		<i>/ Allocates 64 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_128_DMA_BUFF_LOC	<i>/ Allocates 128 words of buffer to each analog input.</i>\n
 *
 * <u>DMAAddrIncrementRate:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- ADC_DMA_ADD_INC_1	<i>/ DMA address increment after conversion of each sample.</i> \n
 *	- .			\n
 *	- .			\n
 *	- ADC_DMA_ADD_INC_32	<i>/ DMA address increment after conversion of 32 samples.</i>  \n
 *
 * \Return	None
 *
 * \Notes	The ADC2 module only supports 10-bit operation with 4 S&H.\n
 *			The AN16 through AN31 pins are not available for the ADC2 module.
 *
 * \Example	SetupADC2(	ADC_10BIT_MODE,
 *				ADC_REF_AVDD_AVSS,
 *				SIMULTANEOUS_SAMPLING,
 *				AUTO_SAMPLING,
 *				SOC_SRC_AUTO_FASTEST_NoOpAMP_ADC2,
 *				TAD_EQUAL_FASTEST_FOR_ADC2,
 *				OUTPUT_FORMAT_U_INT,
 *				ADC_CONTINUE_IN_IDEL,
 *				RESULTS_BUFFER_16LOC_MODE,
 *				INT_WHEN_16_NEW_RESULT_IN_BUFF,
 *				DIS_DMA_TRANSFER,
 *				DONT_CARE, DONT_CARE, DONT_CARE);
 *
 **********************************************************************************************************************/
#define SetupADC2(	ADCResolution,\
			ADCVoltageRef,\
			SamplingMode,\
			StartOfSampleSrc,\
			StartOfConversionSrc,\
			ConversionClockTAD,\
			OutputFormat,\
			IdelEN,\
			BufferFillMode,\
			ResultPerInt,\
			DMATransferEN,\
			DMABufferBuildMode,\
			DMABLPerAnalogInput,\
			DMAAddrIncrementRate)({\
						\
	static unsigned int ADC_Running;\
	ADC_Running = AD2CON1bits.ADON;\
	if(ADCResolution != AD2CON1bits.AD12B) {StopOperationADC2();}\
	AD2CON1bits.AD12B	= ADCResolution;\
	AD2CON2bits.VCFG	= ADCVoltageRef;\
	AD2CON1bits.SIMSAM	= SamplingMode;\
	AD2CON1bits.ASAM	= StartOfSampleSrc;\
	AD2CON1bits.SSRC	= StartOfConversionSrc&0b000000111;\
	AD2CON1bits.SSRCG	= (StartOfConversionSrc>>3)&0b000001;\
	AD2CON3bits.SAMC	= (StartOfConversionSrc>>4);\
	AD2CON3bits.ADCS	= ConversionClockTAD&0b011111111;\
	AD2CON3bits.ADRC	= (ConversionClockTAD>>8);\
	AD2CON1bits.FORM	= OutputFormat;\
	AD2CON1bits.ADSIDL	= IdelEN;\
	AD2CON2bits.BUFM	= BufferFillMode;\
	if(DMATransferEN) {\
		AD2CON2bits.SMPI = DMAAddrIncrementRate;\
		} else {\
			if(AD2CON1bits.SIMSAM){\
				AD2CON2bits.SMPI = ( (ResultPerInt)/(AD2CON2bits.CHPS+1) ) - 1;\
			} else {AD2CON2bits.SMPI = (ResultPerInt) - 1;}\
		}\
	AD2CON4bits.ADDMAEN	= DMATransferEN;\
	AD2CON1bits.ADDMABM	= DMABufferBuildMode;\
	AD2CON4bits.DMABL	= DMABLPerAnalogInput;\
	if(ADC_Running) StartOperationADC2();\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/***********************************************************************************************************************
 * \Function	void SetupADC2Channels(	ChannelToConvert,
 *					AltInputModeEN,
 *					Ch0ScanEN,
 *					Ch0ScanInputs,
 *					Ch0PosInputMuxA,Ch0NegInputMuxA,
 *					Ch0PosInputMuxB,Ch0NegInputMuxB,
 *					Ch123PosInputsMuxA, Ch123NegInputsMuxA,
 *					Ch123PosInputsMuxB, Ch123NegInputsMuxB)
 *
 * \Description		Configures ADC2 Channels Inputs.
 *
 * \PreCondition	None
 *
 * \Inputs
 * <u>ChannelToConvert:</u> \n
 *	- CONVERT_CH0_ONLY	\n
 *	- CONVERT_CH01		<i>/ Only if ADCResolution = ADC_10BIT_MODE.</i>\n
 *	- CONVERT_CH0123	<i>/ Only if ADCResolution = ADC_10BIT_MODE.</i>\n
 *
 * <u>AltInputModeEN:</u>\n
 *	- MUXA_ONLY		<i>/ Always uses channel input selects for MUXA.</i> \n
 *	- ALT_MUXA_MUXB		<i>/ Uses channel input selects for MUX A on the first sample and MUX B on the next sample.</i> \n
 *
 * <u>Ch0ScanEN:</u>\n
 *	- DIS_CH0_SCAN		<i>/ Does not scan inputs for CH0 (CH0 as Normal Channel).</i>\n
 *	- EN_CH0_SCAN		<i>/ Scans inputs for CH0 during Sample MUXA bit.</i>\n
 *
 * <u>Ch0ScanInputs:</u>\n
 *	- SCN_AN0		<i>/ Scan AN0.</i>	 \n
 *	- .			\n
 *	- .			\n
 *	- SCN_AN23		<i>/ Scan AN23.</i>	 \n
 *	- SCN_OA1AN3		<i>/ Scan AN3 or OpAmp1 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- SCN_OA2AN0		<i>/ Scan AN0 or OpAmp2 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- SCN_OA3AN6		<i>/ Scan AN6 or OpAmp3 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- SCN_AN27		<i>/ Scan AN27.</i>\n
 *	- SCN_AN28		<i>/ Scan AN28.</i>\n
 *	- SCN_AN29		<i>/ Scan AN29.</i>\n
 *	- SCN_AN30		<i>/ Scan AN30.</i>\n
 *	- SCN_AN31		<i>/ Scan AN31.</i>\n
 *	- SCN_ALL_0_15		<i>/ Scan All Inputs From  AN0  To  AN15 inclusively.</i>\n
 *	- SCN_ALL_16_31		<i>/ Scan All Inputs From  AN16 To  AN31 inclusively (Note: AN24=OA1AN3 & AN25=OA2AN0 & AN26=OA3AN6).</i>\n
 *	- SCN_ALL_0_31		<i>/ Scan All Inputs From  AN0  To  AN31 inclusively (Note: AN24=OA1AN3 & AN25=OA2AN0 & AN26=OA3AN6).</i>\n
 *	- SCN_NONE		<i>/ Scan Nothing.</i>\n
 *
 * <u>Ch0PosInputMuxA:</u>\n
 *	- ADC_MUXA_CH0_POS_OA2AN0	<i>/ ADC MuxA CH0 Positive Input is AN0 or OpAmp2 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- ADC_MUXA_CH0_POS_AN1		<i>/ ADC MuxA CH0 Positive Input is AN1.</i>\n
 *	- ADC_MUXA_CH0_POS_AN2		<i>/ ADC MuxA CH0 Positive Input is AN2.</i>\n
 *	- ADC_MUXA_CH0_POS_OA1AN3	<i>/ ADC MuxA CH0 Positive Input is AN3 or OpAmp1 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- ADC_MUXA_CH0_POS_AN4		<i>/ ADC MuxA CH0 Positive Input is AN4.</i>\n
 *	- ADC_MUXA_CH0_POS_AN5		<i>/ ADC MuxA CH0 Positive Input is AN5.</i>\n
 *	- ADC_MUXA_CH0_POS_OA3AN6	<i>/ ADC MuxA CH0 Positive Input is AN6 or OpAmp3 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- ADC_MUXA_CH0_POS_AN7		<i>/ ADC MuxA CH0 Positive Input is AN7.</i>\n
 *	- ADC_MUXA_CH0_POS_AN8		<i>/ ADC MuxA CH0 Positive Input is AN8.</i>\n
 *	- ADC_MUXA_CH0_POS_AN9		<i>/ ADC MuxA CH0 Positive Input is AN9.</i>\n
 *	- ADC_MUXA_CH0_POS_AN10		<i>/ ADC MuxA CH0 Positive Input is AN10.</i>\n
 *	- ADC_MUXA_CH0_POS_AN11		<i>/ ADC MuxA CH0 Positive Input is AN11.</i>\n
 *	- ADC_MUXA_CH0_POS_AN12		<i>/ ADC MuxA CH0 Positive Input is AN12.</i>\n
 *	- ADC_MUXA_CH0_POS_AN13		<i>/ ADC MuxA CH0 Positive Input is AN13.</i>\n
 *	- ADC_MUXA_CH0_POS_AN14		<i>/ ADC MuxA CH0 Positive Input is AN14.</i>\n
 *	- ADC_MUXA_CH0_POS_AN15		<i>/ ADC MuxA CH0 Positive Input is AN15.</i>\n
 *	- ADC_MUXA_CH0_POS_AN16		<i>/ ADC MuxA CH0 Positive Input is AN16.</i>\n
 *	- ADC_MUXA_CH0_POS_AN17		<i>/ ADC MuxA CH0 Positive Input is AN17.</i>\n
 *	- ADC_MUXA_CH0_POS_AN18		<i>/ ADC MuxA CH0 Positive Input is AN18.</i>\n
 *	- ADC_MUXA_CH0_POS_AN19		<i>/ ADC MuxA CH0 Positive Input is AN19.</i>\n
 *	- ADC_MUXA_CH0_POS_AN20		<i>/ ADC MuxA CH0 Positive Input is AN20.</i>\n
 *	- ADC_MUXA_CH0_POS_AN21		<i>/ ADC MuxA CH0 Positive Input is AN21.</i>\n
 *	- ADC_MUXA_CH0_POS_AN22		<i>/ ADC MuxA CH0 Positive Input is AN22.</i>\n
 *	- ADC_MUXA_CH0_POS_AN23		<i>/ ADC MuxA CH0 Positive Input is AN23.</i>\n
 *	- ADC_MUXA_CH0_POS_AN24		<i>/ ADC MuxA CH0 Positive Input is AN24.</i>\n
 *	- ADC_MUXA_CH0_POS_OA5AN25	<i>/ ADC MuxA CH0 Positive Input is AN25 or OpAmp5 output voltage if CMxCON.OPMODE=1.</i>\n
 *	- ADC_MUXA_CH0_POS_AN26		<i>/ ADC MuxA CH0 Positive Input is AN26.</i>\n
 *	- ADC_MUXA_CH0_POS_AN27		<i>/ ADC MuxA CH0 Positive Input is AN27.</i>\n
 *	- ADC_MUXA_CH0_POS_AN28		<i>/ ADC MuxA CH0 Positive Input is AN28.</i>\n
 *	- ADC_MUXA_CH0_POS_AN29		<i>/ ADC MuxA CH0 Positive Input is AN29.</i>\n
 *	- ADC_MUXA_CH0_POS_AN30		<i>/ ADC MuxA CH0 Positive Input is AN30.</i>\n
 *	- ADC_MUXA_CH0_POS_AN31		<i>/ ADC MuxA CH0 Positive Input is AN31.</i>\n
 *
 * <u>Ch0NegInputMuxA:</u>\n
 *	- ADC_MUXA_CH0_NEG_VREFL	<i>/ ADC MuxA CH0 Negative Input is VRefL.</i>\n
 *	- ADC_MUXA_CH0_NEG_AN1		<i>/ ADC MuxA CH0 Negative Input is AN1.</i>\n
 *
 * <u>Ch0PosInputMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same as Ch0PosInputMuxA.
 *
 * <u>Ch0NegInputMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same as Ch0NegInputMuxA.
 *
 * <u>Ch123PosInputsMuxA:</u>\n
 *	- ADC_MUXA_CH123_POS_AN0_AN1_AN2		<i>/ ADC MuxA, AN0 => +CH1, AN1 => +CH2, AN2 => +CH.</i>\n
 *	- ADC_MUXA_CH123_POS_AN3_AN4_AN5		<i>/ ADC MuxA, AN3 => +CH1, AN4 => +CH2, AN5 => +CH.</i>\n
 *	- ADC_MUXA_CH123_POS_OA1AN3_OA2AN0_OA3AN6	<i>/ ADC MuxA, AN3 or OpAmp1 => +CH1, AN0 or OpAmp2 => +CH2, AN6 or OpAmp3 => +CH.</i>\n
 *	- ADC_MUXA_CH123_POS_OA1AN3_OA2AN0_OA5AN25	<i>/ ADC MuxA, AN3 or OpAmp1 => +CH1, AN0 or OpAmp2 => +CH2, AN25 or OpAmp5 => +CH.</i>\n
 *	- ADC_MUXA_CH123_POS_OA2AN0_OA5AN25_OA3AN6	<i>/ ADC MuxA, AN0 or OpAmp2 => +CH1, AN25 or OpAmp5 => +CH2, AN6 or OpAmp6 => +CH.</i>\n
 *
 * <u>Ch123NegInputsMuxA:</u>\n
 *	- ADC_MUXA_CH123_NEG_VREFL			<i>/ ADC MuxA, VrefL => -CH1,-CH2,-CH3.</i>\n
 *	- ADC_MUXA_CH123_NEG_AN6_AN7_AN8		<i>/ ADC MuxA, AN6 => -CH1, AN7 => -CH2, AN8 => -CH3.</i>\n
 *	- ADC_MUXA_CH123_NEG_AN9_AN10_AN11		<i>/ ADC MuxA, AN9 => -CH1, AN10 => -CH2, AN11 => -CH3.</i>\n
 *
 * <u>Ch123PosInputsMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same As Ch123PosInputsMuxA.
 *
 * <u>Ch123NegInputsMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same As Ch123NegInputsMuxA.
 *
 * \Return	None
 *
 * \Notes	analog inputs are shared with op amp inputs and outputs, comparator inputs and external voltage references. When
 *		op amp/comparator functionality is enabled, or an external voltage reference is used, the analog input that shares
 *		that pin is no longer available.\n
 *			The AN16 through AN31 pins are not available for the ADC2 module.
 *
 * \Example	SetupADC2Channels(	CONVERT_CH0123,
 *					MUXA_ONLY,
 *					EN_CH0_SCAN,
 *					SCN_AN0 & SCN_AN1 & SCN_AN2 & SCN_AN7,
 *					DONT_CARE, ADC_MUXA_CH0_NEG_VREFL,
 *					DONT_CARE, DONT_CARE,
 *					ADC_MUXA_CH0_POS_OA3AN6, ADC_MUXA_CH123_NEG_VREFL,
 *					DONT_CARE, DONT_CARE);
 *
 **********************************************************************************************************************/
#define SetupADC2Channels(	ChannelToConvert,\
				AltInputModeEN,\
				Ch0ScanEN,\
				Ch0ScanInputs,\
				Ch0PosInputMuxA,Ch0NegInputMuxA,\
				Ch0PosInputMuxB,Ch0NegInputMuxB,\
				Ch123PosInputsMuxA, Ch123NegInputsMuxA,\
				Ch123PosInputsMuxB, Ch123NegInputsMuxB)({\
									\
	AD2CON2bits.CHPS	= ChannelToConvert;\
	AD2CON2bits.ALTS	= AltInputModeEN;\
	AD2CON2bits.CSCNA	= Ch0ScanEN;\
	AD2CSSL			= (~(Ch0ScanInputs))&0x0000FFFF;\
	AD2CSSH			= (~(Ch0ScanInputs))>>16;\
	AD2CHS0bits.CH0SA	= Ch0PosInputMuxA;\
	AD2CHS0bits.CH0NA	= Ch0NegInputMuxA;\
	AD2CHS0bits.CH0SB	= Ch0PosInputMuxB;\
	AD2CHS0bits.CH0NB	= Ch0NegInputMuxB;\
	AD2CHS123bits.CH123SA0	= (Ch123PosInputsMuxA)&0b001;\
	AD2CHS123bits.CH123SA1	= ((Ch123PosInputsMuxA)>>1)&0b01;\
	AD2CHS123bits.CH123SA2	= ((Ch123PosInputsMuxA)>>2);\
	AD2CHS123bits.CH123NA	= Ch123NegInputsMuxA;\
	AD2CHS123bits.CH123SB0	= (Ch123PosInputsMuxB)&0b001;\
	AD2CHS123bits.CH123SB1	= ((Ch123PosInputsMuxB)>>1)&0b01;\
	AD2CHS123bits.CH123SB2	= ((Ch123PosInputsMuxB)>>2);\
	AD2CHS123bits.CH123NB	= Ch123NegInputsMuxB;\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define EnableIntADC2()			(_AD2IE = 1)
#define DisableIntADC2()                (_AD2IE = 0)
#define SetPriorityIntADC2(priority)    (_AD2IP = priority)
#define ADC2_INT_FLAG			_AD2IF

#define StartOperationADC2() ({\
	/* Enable ADC module and provide ADC stabilization delay */\
	static int SampMode;\
	SampMode = AD2CON1bits.ASAM;\
	AD2CON1bits.ASAM = MANUAL_SAMPLING;	/* To Prevent Any Conversions In The Stabilization Time */\
	AD2CON1bits.ADON = 1;\
	__delay_us(20);\
	AD2CON1bits.ASAM = SampMode;\
})

#define StopOperationADC2()		(AD2CON1bits.ADON = 0)	/* Turn Off ADC2 Module */

#define BeginSamplingADC2()		(AD2CON1bits.SAMP = 1)	/* If Manual Sampling Mode Is Selected */

#define BeginConvertingADC2()		(AD2CON1bits.SAMP = 0)	/* (End Sampling and Start Converting) If Manual Converting Mode Is Selected */

#define EnterOffsetCalibrationModeADC2() (AD2CON2bits.OFFCAL = 1)/* + and - inputs of channel Sample-and-Hold are connected to AVSS */

#define ExitOffsetCalibrationModeADC2()	(AD2CON2bits.OFFCAL = 0)/* + and - inputs of channel Sample-and-Hold Normal */

#define ADC2ConversionCompleted()	(AD2CON1bits.DONE)	/* Return 1 => ADC conversion cycle has completed */
								/* Return 0 => ADC conversion has not started or is in progress */

#define ADC2FillingSecBuffHalf()	(AD2CON2bits.BUFS)	/* This Macro is Used When BufferFillMode = RESULTS_BUFFER_2X8_ALT_MODE in SetupADCx */
								/* Return 1 => ADC is currently filling the second half of the buffer;
								* the user application should access data in the first half of the buffer */
								/* Return 0 => ADC is currently filling the first half of the buffer;
								* the user application should access data in the second half of the buffer */

#endif

/**********************************************************************************************************************/
// Functions For (dsPIC33EPXXXGP50X, dsPIC33EPXXXMC20X/50X AND PIC24EPXXXGP/MC20X)  MCUs Group.
/**********************************************************************************************************************/
#ifdef GROUP3_dsPIC33EPXXX_GP_50X_dsPIC33EPXXX_MC_20X_50X_PIC24EPXXX_GP_MC_20X


/**********************************************************************************************************************/
/***************************************** ADC 1 Macro Functions & Defines ********************************************/
/**********************************************************************************************************************/

/***********************************************************************************************************************
 * \Function	void SetupADC1( ADCResolution,
 *				ADCVoltageRef,
 *				SamplingMode,
 *				StartOfSampleSrc,
 *				StartOfConversionSrc,
 *				ConversionClockTAD,
 *				OutputFormat,
 *				IdelEN,
 *				BufferFillMode,
 *				ResultPerInt,
 *				DMATransferEN,
 *				DMABufferBuildMode,
 *				DMABLPerAnalogInput,
 *				DMAAddrIncrementRate)
 *
 * \Description		Configures ADC1 Operation Parameters.
 *
 * \PreCondition	You Must Set The SYS_FREQ and FCY in The Beginning of The Library Header File Before Using The Library.
 *
 * \Inputs
 * <u>ADCResolution:</u>\n
 *	- ADC_10BIT_MODE	\n
 *	- ADC_12BIT_MODE
 *
 * <u>ADCVoltageRef:</u>\n
 *	- ADC_REF_AVDD_AVSS		<i>/ A/D Voltage reference configuration	VrefH is	 AVdd	and VrefL is AVss.</i> \n
 *	- ADC_REF_VREFP_AVSS		<i>/ A/D Voltage reference configuration	VrefH is Ext Vref+	and	VrefL is AVss.</i> \n
 *	- ADC_REF_AVDD_VREFN		<i>/ A/D Voltage reference configuration	VrefH is	 AVdd	and	VrefL is Ext Vref-.</i> \n
 *	- ADC_REF_VREFP_VREFN		<i>/ A/D Voltage reference configuration	VrefH is Ext Vref+	and	VrefL is Ext Vref-.</i> \n
 *
 * <u>SamplingMode:</u>\n
 *	- SEQUENTIAL_SAMPLING		<i>/ Samples multiple channels individually in sequence.</i>\n
 *	- SIMULTANEOUS_SAMPLING		<i>/ only applicable when more than one channel are converted.</i>\n
 *
 * <u>StartOfSampleSrc:</u>\n
 *	- MANUAL_SAMPLING		<i>/ Sampling begins when SAMP bit is set, OR Use BeginSamplingADC1() Function.</i>\n
 *	- AUTO_SAMPLING			<i>/ Sampling begins immediately after last conversion; SAMP bit is auto-set.</i>\n
 *
 * <u>StartOfConversionSrc:</u>\n
 *	- SOC_SRC_MANUAL		<i>/ Clearing the (SAMP) bit ends sampling and starts conversion (Manual mode), OR Use BeginConversionADC1() Function.</i>\n
 *	- SOC_SRC_INT0			<i>/ Active transition on the INT0 pin ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_TMR3			<i>/ Timer 3 Interrupt ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_TMR5			<i>/ Timer 5 Interrupt ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_AUTO_AFTER_0TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- SOC_SRC_AUTO_AFTER_1TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- .																											 \n
 *	- .																											 \n
 *	- SOC_SRC_AUTO_AFTER_31TAD	<i>/ Internal counter ends sampling and starts conversion (auto-convert).</i>\n
 *	- SOC_SRC_AUTO_FASTEST_NoOpAMP_ADC1	<i>/ Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if No OpAmp Outputs is Used As inputs for The ADC Module.</i>\n
 *	- SOC_SRC_AUTO_FASTEST_OpAMP	<i>/ Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if Any OpAmp Output is Used As inputs for The ADC Module.</i>\n
 *	- SOC_SRC_PRI_SEVT		<i>/ (Only dsPICs) PWM primary Special Event Trigger ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_SEC_SEVT		<i>/ (Only dsPICs) PWM secondary Special Event Trigger ends sampling and starts conversion.</i>\n
 *	- SOC_SRC_PWM1			<i>/ (Only dsPICs) PWM Generator 1 primary trigger compare ends sampling and starts conversion.</i> \n
 *	- .				\n
 *	- .				\n
 *	- SOC_SRC_PWM7			<i>/ (Only dsPICs) PWM Generator 7 primary trigger compare ends sampling and starts conversion.</i> \n
 *	- ADC_CLK_CTMU			<i>/ CTMU ends sampling and starts conversion (if CTMU Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO12		<i>/ PTGO12 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO13		<i>/ PTGO13 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO14		<i>/ PTGO14 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *	- SOC_SRC_PTGO15		<i>/ PTGO15 primary trigger compare ends sampling and starts conversion (if PTG Module is Avaliable in The MCU).</i>\n
 *
 * <u>ConversionClockTAD:</u>\n
 *	- TAD_EQUAL_INTERNAL_tRC	<i>/ TAD = ADC internal RC clock Interval.</i>\n
 *	- TAD_EQUAL_FASTEST_FOR_ADC1		<i>/ Automatically Set The Fastest Time Of TAD Respecting (FCY & ADC Resolution).</i>\n
 *	- TAD_EQUAL_1Tcy		\n
 *	- TAD_EQUAL_2Tcy		\n
 *	- .				\n
 *	- .				\n
 *	- TAD_EQUAL_255Tcy
 *
 * <u>OutputFormat:</u>\n
 *	- OUTPUT_FORMAT_U_INT		<i>/ A/D data format integer.</i>\n
 *	- OUTPUT_FORMAT_S_INT		<i>/ A/D data format signed integer.</i>\n
 *	- OUTPUT_FORMAT_U_FRACT		<i>/ A/D data format fractional.</i>\n
 *	- OUTPUT_FORMAT_S_FRACT		<i>/ A/D data format signed fractional.</i>\n
 *
 * <u>IdelEN:</u>\n
 *	- ADC_CONTINUE_IN_IDEL		<i>/ Continue QEI in Idle mode.</i>\n
 *	- ADC_STOP_IN_IDEL		<i>/ Stop QEI in Idle mode.</i>\n
 *
 * <u>BufferFillMode:</u> Only if DMA Transfer is Disabled Else Put (DONT_CARE).\n
 *	- RESULTS_BUFFER_16LOC_MODE		<i>/ Always starts filling the buffer from the Start address.</i>\n
 *	- RESULTS_BUFFER_2X8LOC_ALT_MODE	<i>/ Starts buffer filling the first half of the buffer on the first interrupt
 *							and the second half of the buffer on the next interrupt And So On.</i>\n
 *
 * <u>ResultPerInt:</u> Only if DMA Transfer is Disabled Else Put (DONT_CARE).\n
 *	- INT_WHEN_1_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_2_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_3_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_4_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_5_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_6_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_7_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_8_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_9_NEW_RESULT_IN_BUFF		\n
 *	- INT_WHEN_10_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_11_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_12_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_13_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_14_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_15_NEW_RESULT_IN_BUFF	\n
 *	- INT_WHEN_16_NEW_RESULT_IN_BUFF
 *
 *	- a Value From 1 to 16			<i>/ if SamplingMode = SEQUENTIAL_SAMPLING   & BufferFillMode = RESULTS_BUFFER_16LOC_MODE.</i>\n
 *	- a Value Equ 2,4,6,8,10,12,14,16	<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_16LOC_MODE & Converting 2 Ch.</i>\n
 *	- a Value Equ 4,8,12,16			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_16LOC_MODE & Converting 4 Ch.</i>\n
 *	- a Value From 1 to 8			<i>/ if SamplingMode = SEQUENTIAL_SAMPLING   & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE.</i>\n
 *	- a Value Equ 2,4,6,8			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE & Converting 2 Ch.</i>\n
 *	- a Value Equ 4,8			<i>/ if SamplingMode = SIMULTANEOUS_SAMPLING & BufferFillMode = RESULTS_BUFFER_2X8LOC_ALT_MODE & Converting 2 Ch.</i>\n
 *
 * <u>DMATransferEN:</u>\n
 *	- DIS_DMA_TRANSFER	\n
 *	- EN_DMA_TRANSFER
 *
 * <u>DMABufferBuildMode:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- DMA_BM_SCATTR		<i>/ DMA buffers are written in Scatter/Gather mode.</i>\n
 *	- DMA_BM_ORDER		<i>/ DMA buffers are written in the order of conversion.</i>\n
 *
 * <u>DMABLPerAnalogInput:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- EACH_ANALOG_INPUT_HAS_1_DMA_BUFF_LOC		<i>/ Allocates 1 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_2_DMA_BUFF_LOC		<i>/ Allocates 2 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_4_DMA_BUFF_LOC		<i>/ Allocates 4 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_8_DMA_BUFF_LOC		<i>/ Allocates 8 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_16_DMA_BUFF_LOC		<i>/ Allocates 16 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_32_DMA_BUFF_LOC		<i>/ Allocates 32 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_64_DMA_BUFF_LOC		<i>/ Allocates 64 words of buffer to each analog input.</i>\n
 *	- EACH_ANALOG_INPUT_HAS_128_DMA_BUFF_LOC	<i>/ Allocates 128 words of buffer to each analog input.</i>\n
 *
 * <u>DMAAddrIncrementRate:</u> Only if DMA Transfer is Enabled Else Put (DONT_CARE).\n
 *	- ADC_DMA_ADD_INC_1	<i>/ DMA address increment after conversion of each sample.</i> \n
 *	- .			\n
 *	- .			\n
 *	- ADC_DMA_ADD_INC_32	<i>/ DMA address increment after conversion of 32 samples.</i>  \n
 *
 * \Return	None
 *
 * \Notes	None
 *
 * \Example	SetupADC1(	ADC_10BIT_MODE,
 *				ADC_REF_AVDD_AVSS,
 *				SIMULTANEOUS_SAMPLING,
 *				AUTO_SAMPLING,
 *				SOC_SRC_AUTO_FASTEST_NoOpAMP_ADC1,
 *				TAD_EQUAL_FASTEST_FOR_ADC1,
 *				OUTPUT_FORMAT_U_INT,
 *				ADC_CONTINUE_IN_IDEL,
 *				RESULTS_BUFFER_16LOC_MODE,
 *				INT_WHEN_16_NEW_RESULT_IN_BUFF,
 *				DIS_DMA_TRANSFER,
 *				DONT_CARE, DONT_CARE, DONT_CARE);
 *
 **********************************************************************************************************************/
#define SetupADC1(	ADCResolution,\
			ADCVoltageRef,\
			SamplingMode,\
			StartOfSampleSrc,\
			StartOfConversionSrc,\
			ConversionClockTAD,\
			OutputFormat,\
			IdelEN,\
			BufferFillMode,\
			ResultPerInt,\
			DMATransferEN,\
			DMABufferBuildMode,\
			DMABLPerAnalogInput,\
			DMAAddrIncrementRate)({\
						\
	static unsigned int ADC_Running;\
	ADC_Running = AD1CON1bits.ADON;\
	if(ADCResolution != AD1CON1bits.AD12B) {StopOperationADC1();}\
	AD1CON1bits.AD12B	= ADCResolution;\
	AD1CON2bits.VCFG	= ADCVoltageRef;\
	AD1CON1bits.SIMSAM	= SamplingMode;\
	AD1CON1bits.ASAM	= StartOfSampleSrc;\
	AD1CON1bits.SSRC	= StartOfConversionSrc&0b000000111;\
	AD1CON1bits.SSRCG	= (StartOfConversionSrc>>3)&0b000001;\
	AD1CON3bits.SAMC	= (StartOfConversionSrc>>4);\
	AD1CON3bits.ADCS	= ConversionClockTAD&0b011111111;\
	AD1CON3bits.ADRC	= (ConversionClockTAD>>8);\
	AD1CON1bits.FORM	= OutputFormat;\
	AD1CON1bits.ADSIDL	= IdelEN;\
	AD1CON2bits.BUFM	= BufferFillMode;\
	if(DMATransferEN) {\
		AD1CON2bits.SMPI = DMAAddrIncrementRate;\
		} else {\
			if(AD1CON1bits.SIMSAM){\
				AD1CON2bits.SMPI = ( (ResultPerInt)/(AD1CON2bits.CHPS+1) ) - 1;\
			} else {AD1CON2bits.SMPI = (ResultPerInt) - 1;}\
		}\
	AD1CON4bits.ADDMAEN	= DMATransferEN;\
	AD1CON1bits.ADDMABM	= DMABufferBuildMode;\
	AD1CON4bits.DMABL	= DMABLPerAnalogInput;\
	if(ADC_Running) StartOperationADC1();\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/***********************************************************************************************************************
 * \Function	void SetupADC1Channels(	ChannelToConvert,
 *					AltInputModeEN,
 *					Ch0ScanEN,
 *					Ch0ScanInputs,
 *					Ch0PosInputMuxA,Ch0NegInputMuxA,
 *					Ch0PosInputMuxB,Ch0NegInputMuxB,
 *					Ch123PosInputsMuxA, Ch123NegInputsMuxA,
 *					Ch123PosInputsMuxB, Ch123NegInputsMuxB)
 *
 * \Description		Configures ADC1 Channels Inputs.
 *
 * \PreCondition	None
 *
 * \Inputs
 * <u>ChannelToConvert:</u>		\n
 *	- CONVERT_CH0_ONLY		\n
 *	- CONVERT_CH01			<i>/ Only if ADCResolution = ADC_10BIT_MODE.</i>\n
 *	- CONVERT_CH0123		<i>/ Only if ADCResolution = ADC_10BIT_MODE.</i>\n
 *
 * <u>AltInputModeEN:</u>\n
 *	- MUXA_ONLY			<i>/ Always uses channel input selects for MUXA.</i> \n
 *	- ALT_MUXA_MUXB			<i>/ Uses channel input selects for MUX A on the first sample and MUX B on the next sample.</i> \n
 *
 * <u>Ch0ScanEN:</u>\n
 *	- DIS_CH0_SCAN			<i>/ Does not scan inputs for CH0 (CH0 as Normal Channel).</i>\n
 *	- EN_CH0_SCAN			<i>/ Scans inputs for CH0 during Sample MUXA bit.</i>\n
 *
 * <u>Ch0ScanInputs:</u>\n
 *	- SCN_AN0			<i>/ Scan AN0.</i>	 \n
 *	- .				\n
 *	- .				\n
 *	- SCN_AN15			<i>/ Scan AN15.</i>	 \n
 *	- SCN_OA1AN3			<i>/ Scan AN3 or OpAmp1 output voltage if CMxCON.OPMODE = 1.</i>\n
 *	- SCN_OA2AN0			<i>/ Scan AN0 or OpAmp2 output voltage if CMxCON.OPMODE = 1.</i>\n
 *	- SCN_OA3AN6			<i>/ Scan AN6 or OpAmp3 output voltage if CMxCON.OPMODE = 1.</i>\n
 *	- SCN_CTMU_TEMP			<i>/ Scan CTMU Tempreture Diode Output.</i>\n
 *	- SCN_OPEN_CTMU_CAPTIME		<i>/ Scan CTMU capacitive and time measurement (Open).</i>\n
 *	- SCN_ALL_0_15			<i>/ Scan All Inputs From  AN0  To  AN15 inclusively.</i>\n
 *	- SCN_NONE			<i>/ Scan Nothing.</i>\n
 *
 * <u>Ch0PosInputMuxA:</u>\n
 *	- ADC_MUXA_CH0_POS_AN0			<i>/ ADC MuxA CH0 Positive Input is AN0.</i>\n
 *	- .					\n
 *	- .					\n
 *	- ADC_MUXA_CH0_POS_AN15			<i>/ ADC MuxA CH0 Positive Input is AN15.</i>\n
 *	- ADC_MUXA_CH0_POS_OA1AN3		<i>/ ADC MuxA CH0 Positive Input is AN3 or OpAmp1 output if CMxCON.OPMODE=1.</i>\n
 *	- ADC_MUXA_CH0_POS_OA2AN0		<i>/ ADC MuxA CH0 Positive Input is AN0 or OpAmp2 output if CMxCON.OPMODE=1.</i>\n
 *	- ADC_MUXA_CH0_POS_OA3AN6		<i>/ ADC MuxA CH0 Positive Input is AN6 or OpAmp3 output if CMxCON.OPMODE=1.</i>\n
 *	- ADC_MUXA_CH0_POS_CTMU_TEMP		<i>/ ADC MuxA CH0 Positive Input is CTMU Tempreture Diode.</i>\n
 *	- ADC_MUXA_CH0_POS_OPEN_CTMU_CAPTIME	<i>/ ADC MuxA CH0 Positive Input is Open; use this selection with CTMU capacitive and time measurement.</i>\n
 *
 * <u>Ch0NegInputMuxA:</u>\n
 *	- ADC_MUXA_CH0_NEG_VREFL	<i>/ ADC MuxA CH0 Negative Input is VRefL.</i>\n
 *	- ADC_MUXA_CH0_NEG_AN1		<i>/ ADC MuxA CH0 Negative Input is AN1.</i>\n
 *
 * <u>Ch0PosInputMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same as Ch0PosInputMuxA.
 *
 * <u>Ch0NegInputMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same as Ch0NegInputMuxA.
 *
 * <u>Ch123PosInputsMuxA:</u>\n
 *	- ADC_MUXA_CH123_POS_OA2AN0_AN1_AN2		<i>/ ADC MuxA, OpAmp2 or AN0 => +CH1, AN1 => +CH2, AN2 => +CH.</i>\n
 *	- ADC_MUXA_CH123_POS_OA1AN3_OA2AN0_OA3AN6	<i>/ ADC MuxA, OpAmp1 or AN3 => +CH1, OpAmp2 or AN4 => +CH2, OpAmp3 or AN5 => +CH.</i>\n

 * <u>Ch123NegInputsMuxA:</u>\n
 *	- ADC_MUXA_CH123_NEG_VREFL			<i>/ ADC MuxA, VrefL => -CH1,-CH2,-CH3.</i>\n
 *	- ADC_MUXA_CH123_NEG_OA3AN6_AN7_AN8		<i>/ ADC MuxA, OpAmp3 or AN6 => -CH1, AN7 => -CH2, AN8 => -CH3.</i>\n
 *	- ADC_MUXA_CH123_NEG_AN9_AN10_AN11		<i>/ ADC MuxA, AN9 => -CH1, AN10 => -CH2, AN11 => -CH3.</i>\n
 *
 * <u>Ch123PosInputsMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same As Ch123PosInputsMuxA.
 *
 * <u>Ch123NegInputsMuxB:</u> Only if Alt Input Mode is Enabled Else Put (DONT_CARE).\n
 *	- Same As Ch123NegInputsMuxA.
 *
 * \Return	None
 *
 * \Notes	analog inputs are shared with op amp inputs and outputs, comparator inputs and external voltage references. When
 *		op amp/comparator functionality is enabled, or an external voltage reference is used, the analog input that shares
 *		that pin is no longer available.
 *
 * \Example	SetupADC1Channels(	CONVERT_CH0123,
 *					MUXA_ONLY,
 *					EN_CH0_SCAN,
 *					SCN_AN0 & SCN_AN1 & SCN_AN2 & SCN_AN7,
 *					DONT_CARE, ADC_MUXA_CH0_NEG_VREFL,
 *					DONT_CARE, DONT_CARE,
 *					ADC_MUXA_CH123_POS_OA2AN0_AN1_AN2, ADC_MUXA_CH123_NEG_VREFL,
 *					DONT_CARE, DONT_CARE);
 *
 **********************************************************************************************************************/
#define SetupADC1Channels(	ChannelToConvert,\
				AltInputModeEN,\
				Ch0ScanEN,\
				Ch0ScanInputs,\
				Ch0PosInputMuxA,Ch0NegInputMuxA,\
				Ch0PosInputMuxB,Ch0NegInputMuxB,\
				Ch123PosInputsMuxA, Ch123NegInputsMuxA,\
				Ch123PosInputsMuxB, Ch123NegInputsMuxB)({\
									\
	AD1CON2bits.CHPS	= ChannelToConvert;\
	AD1CON2bits.ALTS	= AltInputModeEN;\
	AD1CON2bits.CSCNA	= Ch0ScanEN;\
	AD1CSSL			= (~(Ch0ScanInputs))&0x0000FFFF;\
	AD1CSSH			= (~(Ch0ScanInputs))>>16;\
	AD1CHS0bits.CH0SA	= Ch0PosInputMuxA;\
	AD1CHS0bits.CH0NA	= Ch0NegInputMuxA;\
	AD1CHS0bits.CH0SB	= Ch0PosInputMuxB;\
	AD1CHS0bits.CH0NB	= Ch0NegInputMuxB;\
	AD1CHS123bits.CH123SA	= Ch123PosInputsMuxA;\
	AD1CHS123bits.CH123NA	= Ch123NegInputsMuxA;\
	AD1CHS123bits.CH123SB	= Ch123PosInputsMuxB;\
	AD1CHS123bits.CH123NB	= Ch123NegInputsMuxB;\
})

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define EnableIntADC1()			(_AD1IE = 1)
#define DisableIntADC1()                (_AD1IE = 0)
#define SetPriorityIntADC1(priority)    (_AD1IP = priority)
#define ADC1_INT_FLAG			_AD1IF

#define StartOperationADC1() ({\
	/* Enable ADC module and provide ADC stabilization delay */\
	static int SampMode;\
	SampMode = AD1CON1bits.ASAM;\
	AD1CON1bits.ASAM = MANUAL_SAMPLING;	/* To Prevent Any Conversions In The Stabilization Time */\
	AD1CON1bits.ADON = 1;\
	__delay_us(20);\
	AD1CON1bits.ASAM = SampMode;\
})

#define StopOperationADC1()		(AD1CON1bits.ADON = 0)	/* Turn Off ADC1 Module */

#define BeginSamplingADC1()		(AD1CON1bits.SAMP = 1)	/* If Manual Sampling Mode Is Selected */

#define BeginConvertingADC1()		(AD1CON1bits.SAMP = 0)	/* (End Sampling and Start Converting) If Manual Converting Mode Is Selected */

#define ADC1ConversionCompleted()	(AD1CON1bits.DONE)	/* Return 1 => ADC conversion cycle has completed */
								/* Return 0 => ADC conversion has not started or is in progress */

#define ADC1FillingSecBuffHalf()	(AD1CON2bits.BUFS)	/* This Macro is Used When BufferFillMode = RESULTS_BUFFER_2X8_ALT_MODE in SetupADCx */
								/* Return 1 => ADC is currently filling the second half of the buffer;
								* the user application should access data in the first half of the buffer */
								/* Return 0 => ADC is currently filling the first half of the buffer;
								* the user application should access data in the second half of the buffer */

#endif

/**********************************************************************************************************************/
/**********************************************************************************************************************/
// <editor-fold defaultstate="collapsed" desc="ADC Library Arguments">

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* SetupADCx */

// ADCResolution
#define ADC_10BIT_MODE		0
#define ADC_12BIT_MODE		1

// ADCVoltageRef
#define ADC_REF_AVDD_AVSS	0	/* A/D Voltage reference configuration	VrefH is AVdd		and	VrefL is AVss */
#define ADC_REF_VREFP_AVSS	1	/* A/D Voltage reference configuration	VrefH is Ext Vref+	and	VrefL is AVss */
#define ADC_REF_AVDD_VREFN	2	/* A/D Voltage reference configuration	VrefH is AVdd		and	VrefL is Ext Vref- */
#define ADC_REF_VREFP_VREFN	3	/* A/D Voltage reference configuration	VrefH is Ext Vref+	and	VrefL is Ext Vref- */

// SamplingMode
#define SEQUENTIAL_SAMPLING	0	/* Samples multiple channels individually in sequence. */
#define	SIMULTANEOUS_SAMPLING	1	/* only applicable when more than one channel are converted. */

// StartOfSampleSrc
#define MANUAL_SAMPLING		0	/* Sampling begins when SAMP bit is set */
#define AUTO_SAMPLING		1	/* Sampling begins immediately after last conversion; SAMP bit is auto-set */

// StartOfConversionSrc
// <editor-fold defaultstate="collapsed" desc="StartOfConversionSrc Args">
#define SOC_SRC_MANUAL			0b000000000 /* SSRC = 000 & SSRCG = 0 Clearing the Sample bit (SAMP) ends sampling and starts conversion (Manual mode) */
#define SOC_SRC_INT0			0b000000001 /* SSRC = 001 & SSRCG = 0 Active transition on the INT0 pin ends sampling and starts conversion */
#define SOC_SRC_TMR3			0b000000010 /* SSRC = 010 & SSRCG = 0 Timer 3 Interrupt ends sampling and starts conversion */
#define SOC_SRC_TMR5			0b000000100 /* SSRC = 100 & SSRCG = 0 Timer 5 Interrupt ends sampling and starts conversion */
#define SOC_SRC_AUTO_AFTER_0TAD		0b000000111 /* SSRC = 111 & SSRCG = 0 & SAMC = 00000 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_1TAD		0b000010111 /* SSRC = 111 & SSRCG = 0 & SAMC = 00001 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_2TAD		0b000100111 /* SSRC = 111 & SSRCG = 0 & SAMC = 00010 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_3TAD		0b000110111 /* SSRC = 111 & SSRCG = 0 & SAMC = 00011 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_4TAD		0b001000111 /* SSRC = 111 & SSRCG = 0 & SAMC = 00100 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_5TAD		0b001010111 /* SSRC = 111 & SSRCG = 0 & SAMC = 00101 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_6TAD		0b001100111 /* SSRC = 111 & SSRCG = 0 & SAMC = 00110 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_7TAD		0b001110111 /* SSRC = 111 & SSRCG = 0 & SAMC = 00111 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_8TAD		0b010000111 /* SSRC = 111 & SSRCG = 0 & SAMC = 01000 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_9TAD		0b010010111 /* SSRC = 111 & SSRCG = 0 & SAMC = 01001 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_10TAD	0b010100111 /* SSRC = 111 & SSRCG = 0 & SAMC = 01010 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_11TAD	0b010110111 /* SSRC = 111 & SSRCG = 0 & SAMC = 01011 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_12TAD	0b011000111 /* SSRC = 111 & SSRCG = 0 & SAMC = 01100 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_13TAD	0b011010111 /* SSRC = 111 & SSRCG = 0 & SAMC = 01101 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_14TAD	0b011100111 /* SSRC = 111 & SSRCG = 0 & SAMC = 01110 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_15TAD	0b011110111 /* SSRC = 111 & SSRCG = 0 & SAMC = 01111 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_16TAD	0b100000111 /* SSRC = 111 & SSRCG = 0 & SAMC = 10000 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_17TAD	0b100010111 /* SSRC = 111 & SSRCG = 0 & SAMC = 10001 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_18TAD	0b100100111 /* SSRC = 111 & SSRCG = 0 & SAMC = 10010 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_19TAD	0b100110111 /* SSRC = 111 & SSRCG = 0 & SAMC = 10011 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_20TAD	0b101000111 /* SSRC = 111 & SSRCG = 0 & SAMC = 10100 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_21TAD	0b101010111 /* SSRC = 111 & SSRCG = 0 & SAMC = 10101 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_22TAD	0b101100111 /* SSRC = 111 & SSRCG = 0 & SAMC = 10110 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_23TAD	0b101110111 /* SSRC = 111 & SSRCG = 0 & SAMC = 10111 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_24TAD	0b110000111 /* SSRC = 111 & SSRCG = 0 & SAMC = 11000 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_25TAD	0b110010111 /* SSRC = 111 & SSRCG = 0 & SAMC = 11001 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_26TAD	0b110100111 /* SSRC = 111 & SSRCG = 0 & SAMC = 11010 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_27TAD	0b110110111 /* SSRC = 111 & SSRCG = 0 & SAMC = 11011 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_28TAD	0b111000111 /* SSRC = 111 & SSRCG = 0 & SAMC = 11100 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_29TAD	0b111010111 /* SSRC = 111 & SSRCG = 0 & SAMC = 11101 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_30TAD	0b111100111 /* SSRC = 111 & SSRCG = 0 & SAMC = 11110 Internal counter ends sampling and starts conversion (auto-convert) */
#define SOC_SRC_AUTO_AFTER_31TAD	0b111110111 /* SSRC = 111 & SSRCG = 0 & SAMC = 11111 Internal counter ends sampling and starts conversion (auto-convert) */
/* Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if No OpAmp Outputs is Used As inputs for The ADC1 Module */
#define SOC_SRC_AUTO_FASTEST_NoOpAMP_ADC1	({\
		static unsigned int _Return;\
		if(AD1CON1bits.AD12B)  {_Return = FASTEST_SAMPLE_TIME_FOR_12bit;}\
		if(!AD1CON1bits.AD12B) {_Return = FASTEST_SAMPLE_TIME_FOR_10bit_NoOpAMP;}\
		_Return;\
})
/* Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if Any OpAmp Output is Used As inputs for The ADC1 Module */
#define SOC_SRC_AUTO_FASTEST_OpAMP_ADC1	({\
		static unsigned int _Return;\
		if(AD1CON1bits.AD12B)  {_Return = FASTEST_SAMPLE_TIME_FOR_12bit;}\
		if(!AD1CON1bits.AD12B) {_Return = FASTEST_SAMPLE_TIME_FOR_10bit_OpAMP;}\
		_Return;\
})
/* Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if No OpAmp Outputs is Used As inputs for The ADC2 Module */
#define SOC_SRC_AUTO_FASTEST_NoOpAMP_ADC2	FASTEST_SAMPLE_TIME_FOR_10bit_NoOpAMP
/* Automaticaly Sellect The Fastest Time For ends sampling and starts conversion if Any OpAmp Output is Used As inputs for The ADC2 Module */
#define SOC_SRC_AUTO_FASTEST_OpAMP_ADC2		FASTEST_SAMPLE_TIME_FOR_10bit_OpAMP

#ifdef _PSEMIF
#define SOC_SRC_PRI_SEVT	0b000000011 /* SSRC = 011 & SSRCG = 0 PWM primary Special Event Trigger ends sampling and starts conversion */
#endif
#ifdef _PSESMIF
#define SOC_SRC_SEC_SEVT	0b000000101 /* SSRC = 101 & SSRCG = 0 PWM secondary Special Event Trigger ends sampling and starts conversion */
#endif
#ifdef _CTMUIF
#define ADC_CLK_CTMU		0b000000110 /* SSRC = 110 & SSRCG = 0 CTMU ends sampling and starts conversion */
#endif
#ifdef _PWM1IF
#define SOC_SRC_PWM1		0b000001000 /* SSRC = 000 & SSRCG = 1 PWM Generator 1 primary trigger compare ends sampling and starts conversion */
#define SOC_SRC_PWM2		0b000001001 /* SSRC = 001 & SSRCG = 1 PWM Generator 2 primary trigger compare ends sampling and starts conversion */
#define SOC_SRC_PWM3		0b000001010 /* SSRC = 010 & SSRCG = 1 PWM Generator 3 primary trigger compare ends sampling and starts conversion */
#endif
#ifdef _PTG0IF
#define SOC_SRC_PTGO12		0b000001011 /* SSRC = 011 & SSRCG = 1 PTGO12 primary trigger compare ends sampling and starts conversion */
#define SOC_SRC_PTGO13		0b000001100 /* SSRC = 100 & SSRCG = 1 PTGO13 primary trigger compare ends sampling and starts conversion */
#define SOC_SRC_PTGO14		0b000001101 /* SSRC = 101 & SSRCG = 1 PTGO14 primary trigger compare ends sampling and starts conversion */
#define SOC_SRC_PTGO15		0b000001110 /* SSRC = 110 & SSRCG = 1 PTGO15 primary trigger compare ends sampling and starts conversion */
#elif defined _PWM4IF
#define SOC_SRC_PWM4		0b000001011 /* SSRC = 011 & SSRCG = 1 PWM Generator 4 primary trigger compare ends sampling and starts conversion */
#define SOC_SRC_PWM5		0b000001100 /* SSRC = 100 & SSRCG = 1 PWM Generator 5 primary trigger compare ends sampling and starts conversion */
#define SOC_SRC_PWM6		0b000001101 /* SSRC = 101 & SSRCG = 1 PWM Generator 6 primary trigger compare ends sampling and starts conversion */
#define SOC_SRC_PWM7		0b000001110 /* SSRC = 110 & SSRCG = 1 PWM Generator 7 primary trigger compare ends sampling and starts conversion */
#endif
// </editor-fold>

// ConversionClockTAD
// <editor-fold defaultstate="collapsed" desc="ConversionClockTAD Args">
#define TAD_EQUAL_INTERNAL_tRC   256
#define TAD_EQUAL_FASTEST_FOR_ADC1   ({\
		static unsigned int _Return;\
		if(AD1CON1bits.AD12B)  _Return=FASTEST_TAD_FOR_12bit_MODE;\
		if(!AD1CON1bits.AD12B) _Return=FASTEST_TAD_FOR_10bit_MODE;\
		_Return;\
})
#define TAD_EQUAL_FASTEST_FOR_ADC2	FASTEST_TAD_FOR_10bit_MODE

#define TAD_EQUAL_1Tcy	   0
#define TAD_EQUAL_2Tcy     1
#define TAD_EQUAL_3Tcy     2
#define TAD_EQUAL_4Tcy     3
#define TAD_EQUAL_5Tcy     4
#define TAD_EQUAL_6Tcy     5
#define TAD_EQUAL_7Tcy     6
#define TAD_EQUAL_8Tcy     7
#define TAD_EQUAL_9Tcy     8
#define TAD_EQUAL_10Tcy    9
#define TAD_EQUAL_11Tcy    10
#define TAD_EQUAL_12Tcy    11
#define TAD_EQUAL_13Tcy    12
#define TAD_EQUAL_14Tcy    13
#define TAD_EQUAL_15Tcy    14
#define TAD_EQUAL_16Tcy    15
#define TAD_EQUAL_17Tcy    16
#define TAD_EQUAL_18Tcy    17
#define TAD_EQUAL_19Tcy    18
#define TAD_EQUAL_20Tcy    19
#define TAD_EQUAL_21Tcy    20
#define TAD_EQUAL_22Tcy    21
#define TAD_EQUAL_23Tcy    22
#define TAD_EQUAL_24Tcy    23
#define TAD_EQUAL_25Tcy    24
#define TAD_EQUAL_26Tcy    25
#define TAD_EQUAL_27Tcy    26
#define TAD_EQUAL_28Tcy    27
#define TAD_EQUAL_29Tcy    28
#define TAD_EQUAL_30Tcy    29
#define TAD_EQUAL_31Tcy    30
#define TAD_EQUAL_32Tcy    31
#define TAD_EQUAL_33Tcy    32
#define TAD_EQUAL_34Tcy    33
#define TAD_EQUAL_35Tcy    34
#define TAD_EQUAL_36Tcy    35
#define TAD_EQUAL_37Tcy    36
#define TAD_EQUAL_38Tcy    37
#define TAD_EQUAL_39Tcy    38
#define TAD_EQUAL_40Tcy    39
#define TAD_EQUAL_41Tcy    40
#define TAD_EQUAL_42Tcy    41
#define TAD_EQUAL_43Tcy    42
#define TAD_EQUAL_44Tcy    43
#define TAD_EQUAL_45Tcy    44
#define TAD_EQUAL_46Tcy    45
#define TAD_EQUAL_47Tcy    46
#define TAD_EQUAL_48Tcy    47
#define TAD_EQUAL_49Tcy    48
#define TAD_EQUAL_50Tcy    49
#define TAD_EQUAL_51Tcy    50
#define TAD_EQUAL_52Tcy    51
#define TAD_EQUAL_53Tcy    52
#define TAD_EQUAL_54Tcy    53
#define TAD_EQUAL_55Tcy    54
#define TAD_EQUAL_56Tcy    55
#define TAD_EQUAL_57Tcy    56
#define TAD_EQUAL_58Tcy    57
#define TAD_EQUAL_59Tcy    58
#define TAD_EQUAL_60Tcy    59
#define TAD_EQUAL_61Tcy    60
#define TAD_EQUAL_62Tcy    61
#define TAD_EQUAL_63Tcy    62
#define TAD_EQUAL_64Tcy    63
#define TAD_EQUAL_65Tcy    64
#define TAD_EQUAL_66Tcy    65
#define TAD_EQUAL_67Tcy    66
#define TAD_EQUAL_68Tcy    67
#define TAD_EQUAL_69Tcy    68
#define TAD_EQUAL_70Tcy    69
#define TAD_EQUAL_71Tcy    70
#define TAD_EQUAL_72Tcy    71
#define TAD_EQUAL_73Tcy    72
#define TAD_EQUAL_74Tcy    73
#define TAD_EQUAL_75Tcy    74
#define TAD_EQUAL_76Tcy    75
#define TAD_EQUAL_77Tcy    76
#define TAD_EQUAL_78Tcy    77
#define TAD_EQUAL_79Tcy    78
#define TAD_EQUAL_80Tcy    79
#define TAD_EQUAL_81Tcy    80
#define TAD_EQUAL_82Tcy    81
#define TAD_EQUAL_83Tcy    82
#define TAD_EQUAL_84Tcy    83
#define TAD_EQUAL_85Tcy    84
#define TAD_EQUAL_86Tcy    85
#define TAD_EQUAL_87Tcy    86
#define TAD_EQUAL_88Tcy    87
#define TAD_EQUAL_89Tcy    88
#define TAD_EQUAL_90Tcy    89
#define TAD_EQUAL_91Tcy    90
#define TAD_EQUAL_92Tcy    91
#define TAD_EQUAL_93Tcy    92
#define TAD_EQUAL_94Tcy    93
#define TAD_EQUAL_95Tcy    94
#define TAD_EQUAL_96Tcy    95
#define TAD_EQUAL_97Tcy    96
#define TAD_EQUAL_98Tcy    97
#define TAD_EQUAL_99Tcy    98
#define TAD_EQUAL_100Tcy    99
#define TAD_EQUAL_101Tcy    100
#define TAD_EQUAL_102Tcy    101
#define TAD_EQUAL_103Tcy    102
#define TAD_EQUAL_104Tcy    103
#define TAD_EQUAL_105Tcy    104
#define TAD_EQUAL_106Tcy    105
#define TAD_EQUAL_107Tcy    106
#define TAD_EQUAL_108Tcy    107
#define TAD_EQUAL_109Tcy    108
#define TAD_EQUAL_110Tcy    109
#define TAD_EQUAL_111Tcy    110
#define TAD_EQUAL_112Tcy    111
#define TAD_EQUAL_113Tcy    112
#define TAD_EQUAL_114Tcy    113
#define TAD_EQUAL_115Tcy    114
#define TAD_EQUAL_116Tcy    115
#define TAD_EQUAL_117Tcy    116
#define TAD_EQUAL_118Tcy    117
#define TAD_EQUAL_119Tcy    118
#define TAD_EQUAL_120Tcy    119
#define TAD_EQUAL_121Tcy    120
#define TAD_EQUAL_122Tcy    121
#define TAD_EQUAL_123Tcy    122
#define TAD_EQUAL_124Tcy    123
#define TAD_EQUAL_125Tcy    124
#define TAD_EQUAL_126Tcy    125
#define TAD_EQUAL_127Tcy    126
#define TAD_EQUAL_128Tcy    127
#define TAD_EQUAL_129Tcy    128
#define TAD_EQUAL_130Tcy    129
#define TAD_EQUAL_131Tcy    130
#define TAD_EQUAL_132Tcy    131
#define TAD_EQUAL_133Tcy    132
#define TAD_EQUAL_134Tcy    133
#define TAD_EQUAL_135Tcy    134
#define TAD_EQUAL_136Tcy    135
#define TAD_EQUAL_137Tcy    136
#define TAD_EQUAL_138Tcy    137
#define TAD_EQUAL_139Tcy    138
#define TAD_EQUAL_140Tcy    139
#define TAD_EQUAL_141Tcy    140
#define TAD_EQUAL_142Tcy    141
#define TAD_EQUAL_143Tcy    142
#define TAD_EQUAL_144Tcy    143
#define TAD_EQUAL_145Tcy    144
#define TAD_EQUAL_146Tcy    145
#define TAD_EQUAL_147Tcy    146
#define TAD_EQUAL_148Tcy    147
#define TAD_EQUAL_149Tcy    148
#define TAD_EQUAL_150Tcy    149
#define TAD_EQUAL_151Tcy    150
#define TAD_EQUAL_152Tcy    151
#define TAD_EQUAL_153Tcy    152
#define TAD_EQUAL_154Tcy    153
#define TAD_EQUAL_155Tcy    154
#define TAD_EQUAL_156Tcy    155
#define TAD_EQUAL_157Tcy    156
#define TAD_EQUAL_158Tcy    157
#define TAD_EQUAL_159Tcy    158
#define TAD_EQUAL_160Tcy    159
#define TAD_EQUAL_161Tcy    160
#define TAD_EQUAL_162Tcy    161
#define TAD_EQUAL_163Tcy    162
#define TAD_EQUAL_164Tcy    163
#define TAD_EQUAL_165Tcy    164
#define TAD_EQUAL_166Tcy    165
#define TAD_EQUAL_167Tcy    166
#define TAD_EQUAL_168Tcy    167
#define TAD_EQUAL_169Tcy    168
#define TAD_EQUAL_170Tcy    169
#define TAD_EQUAL_171Tcy    170
#define TAD_EQUAL_172Tcy    171
#define TAD_EQUAL_173Tcy    172
#define TAD_EQUAL_174Tcy    173
#define TAD_EQUAL_175Tcy    174
#define TAD_EQUAL_176Tcy    175
#define TAD_EQUAL_177Tcy    176
#define TAD_EQUAL_178Tcy    177
#define TAD_EQUAL_179Tcy    178
#define TAD_EQUAL_180Tcy    179
#define TAD_EQUAL_181Tcy    180
#define TAD_EQUAL_182Tcy    181
#define TAD_EQUAL_183Tcy    182
#define TAD_EQUAL_184Tcy    183
#define TAD_EQUAL_185Tcy    184
#define TAD_EQUAL_186Tcy    185
#define TAD_EQUAL_187Tcy    186
#define TAD_EQUAL_188Tcy    187
#define TAD_EQUAL_189Tcy    188
#define TAD_EQUAL_190Tcy    189
#define TAD_EQUAL_191Tcy    190
#define TAD_EQUAL_192Tcy    191
#define TAD_EQUAL_193Tcy    192
#define TAD_EQUAL_194Tcy    193
#define TAD_EQUAL_195Tcy    194
#define TAD_EQUAL_196Tcy    195
#define TAD_EQUAL_197Tcy    196
#define TAD_EQUAL_198Tcy    197
#define TAD_EQUAL_199Tcy    198
#define TAD_EQUAL_200Tcy    199
#define TAD_EQUAL_201Tcy    200
#define TAD_EQUAL_202Tcy    201
#define TAD_EQUAL_203Tcy    202
#define TAD_EQUAL_204Tcy    203
#define TAD_EQUAL_205Tcy    204
#define TAD_EQUAL_206Tcy    205
#define TAD_EQUAL_207Tcy    206
#define TAD_EQUAL_208Tcy    207
#define TAD_EQUAL_209Tcy    208
#define TAD_EQUAL_210Tcy    209
#define TAD_EQUAL_211Tcy    210
#define TAD_EQUAL_212Tcy    211
#define TAD_EQUAL_213Tcy    212
#define TAD_EQUAL_214Tcy    213
#define TAD_EQUAL_215Tcy    214
#define TAD_EQUAL_216Tcy    215
#define TAD_EQUAL_217Tcy    216
#define TAD_EQUAL_218Tcy    217
#define TAD_EQUAL_219Tcy    218
#define TAD_EQUAL_220Tcy    219
#define TAD_EQUAL_221Tcy    220
#define TAD_EQUAL_222Tcy    221
#define TAD_EQUAL_223Tcy    222
#define TAD_EQUAL_224Tcy    223
#define TAD_EQUAL_225Tcy    224
#define TAD_EQUAL_226Tcy    225
#define TAD_EQUAL_227Tcy    226
#define TAD_EQUAL_228Tcy    227
#define TAD_EQUAL_229Tcy    228
#define TAD_EQUAL_230Tcy    229
#define TAD_EQUAL_231Tcy    230
#define TAD_EQUAL_232Tcy    231
#define TAD_EQUAL_233Tcy    232
#define TAD_EQUAL_234Tcy    233
#define TAD_EQUAL_235Tcy    234
#define TAD_EQUAL_236Tcy    235
#define TAD_EQUAL_237Tcy    236
#define TAD_EQUAL_238Tcy    237
#define TAD_EQUAL_239Tcy    238
#define TAD_EQUAL_240Tcy    239
#define TAD_EQUAL_241Tcy    240
#define TAD_EQUAL_242Tcy    241
#define TAD_EQUAL_243Tcy    242
#define TAD_EQUAL_244Tcy    243
#define TAD_EQUAL_245Tcy    244
#define TAD_EQUAL_246Tcy    245
#define TAD_EQUAL_247Tcy    246
#define TAD_EQUAL_248Tcy    247
#define TAD_EQUAL_249Tcy    248
#define TAD_EQUAL_250Tcy    249
#define TAD_EQUAL_251Tcy    250
#define TAD_EQUAL_252Tcy    251
#define TAD_EQUAL_253Tcy    252
#define TAD_EQUAL_254Tcy    253
#define TAD_EQUAL_255Tcy    254
#define TAD_EQUAL_256Tcy    255
// </editor-fold>

// BufferFillMode
#define RESULTS_BUFFER_16LOC_MODE	0	/* Always starts filling the buffer from the Start address. */
#define RESULTS_BUFFER_2X8LOC_ALT_MODE	1	/* Starts buffer filling the first half of the buffer on the first interrupt
						   and the second half of the buffer on the next interrupt AND SO ON*/
// OutputFormat
#define OUTPUT_FORMAT_U_INT			0	/* A/D data format integer */
#define OUTPUT_FORMAT_S_INT			1	/* A/D data format signed integer */
#define OUTPUT_FORMAT_U_FRACT			2	/* A/D data format fractional */
#define OUTPUT_FORMAT_S_FRACT			3	/* A/D data format signed fractional */

// IdelEN
#define	ADC_CONTINUE_IN_IDEL			0	/* Continue QEI in Idle mode */
#define	ADC_STOP_IN_IDEL			1	/* Stop QEI in Idle mode */

// DMATransferEN
#define	DIS_DMA_TRANSFER			0	/*Conversion results are stored in ADCxBUF0 through ADCxBUFF registers; DMA will not be used*/
#define	EN_DMA_TRANSFER				1	/*Conversion results are stored in ADCxBUF0 register for transferring to RAM using DMA*/

// DMABufferBuildMode
#define DMA_BM_SCATTR				0	/* DMA buffers are written in Scatter/Gather mode */
#define DMA_BM_ORDER				1	/* DMA buffers are written in the order of conversion */

// DMABLPerAnalogInput
#define EACH_ANALOG_INPUT_HAS_1_DMA_BUFF_LOC	0	/* Allocates 1 words of buffer to each analog input */
#define EACH_ANALOG_INPUT_HAS_2_DMA_BUFF_LOC	1	/* Allocates 2 words of buffer to each analog input */
#define EACH_ANALOG_INPUT_HAS_4_DMA_BUFF_LOC	2	/* Allocates 4 words of buffer to each analog input */
#define EACH_ANALOG_INPUT_HAS_8_DMA_BUFF_LOC	3	/* Allocates 8 words of buffer to each analog input */
#define EACH_ANALOG_INPUT_HAS_16_DMA_BUFF_LOC	4	/* Allocates 16 words of buffer to each analog input */
#define EACH_ANALOG_INPUT_HAS_32_DMA_BUFF_LOC	5	/* Allocates 32 words of buffer to each analog input */
#define EACH_ANALOG_INPUT_HAS_64_DMA_BUFF_LOC	6	/* Allocates 64 words of buffer to each analog input */
#define EACH_ANALOG_INPUT_HAS_128_DMA_BUFF_LOC	7	/* Allocates 128 words of buffer to each analog input */

// DMAAddrIncrementRate
// <editor-fold defaultstate="collapsed" desc="DMAAddrIncrementRate Args">
#define ADC_DMA_ADD_INC_1       0 /* DMA address increment after conversion of each sample */
#define ADC_DMA_ADD_INC_2       1 /* DMA address increment after conversion of 2 samples */
#define ADC_DMA_ADD_INC_3       2 /* DMA address increment after conversion of 3 samples */
#define ADC_DMA_ADD_INC_4       3 /* DMA address increment after conversion of 4 samples */
#define ADC_DMA_ADD_INC_5       4 /* DMA address increment after conversion of 5 samples */
#define ADC_DMA_ADD_INC_6       5 /* DMA address increment after conversion of 6 samples */
#define ADC_DMA_ADD_INC_7       6 /* DMA address increment after conversion of 7 samples */
#define ADC_DMA_ADD_INC_8       7 /* DMA address increment after conversion of 8 samples */
#define ADC_DMA_ADD_INC_9       8 /* DMA address increment after conversion of 9 samples */
#define ADC_DMA_ADD_INC_10      9 /* DMA address increment after conversion of 10 samples */
#define ADC_DMA_ADD_INC_11      10 /* DMA address increment after conversion of 11 samples */
#define ADC_DMA_ADD_INC_12      11 /* DMA address increment after conversion of 12 samples */
#define ADC_DMA_ADD_INC_13      12 /* DMA address increment after conversion of 13 samples */
#define ADC_DMA_ADD_INC_14      13 /* DMA address increment after conversion of 14 samples */
#define ADC_DMA_ADD_INC_15      14 /* DMA address increment after conversion of 15 samples */
#define ADC_DMA_ADD_INC_16      15 /* DMA address increment after conversion of 16 samples */
#define ADC_DMA_ADD_INC_17      16 /* DMA address increment after conversion of 17 samples */
#define ADC_DMA_ADD_INC_18      17 /* DMA address increment after conversion of 18 samples */
#define ADC_DMA_ADD_INC_19      18 /* DMA address increment after conversion of 19 samples */
#define ADC_DMA_ADD_INC_20      19 /* DMA address increment after conversion of 20 samples */
#define ADC_DMA_ADD_INC_21      20 /* DMA address increment after conversion of 21 samples */
#define ADC_DMA_ADD_INC_22      21 /* DMA address increment after conversion of 22 samples */
#define ADC_DMA_ADD_INC_23      22 /* DMA address increment after conversion of 23 samples */
#define ADC_DMA_ADD_INC_24      23 /* DMA address increment after conversion of 24 samples */
#define ADC_DMA_ADD_INC_25      24 /* DMA address increment after conversion of 25 samples */
#define ADC_DMA_ADD_INC_26      25 /* DMA address increment after conversion of 26 samples */
#define ADC_DMA_ADD_INC_27      26 /* DMA address increment after conversion of 27 samples */
#define ADC_DMA_ADD_INC_28      27 /* DMA address increment after conversion of 28 samples */
#define ADC_DMA_ADD_INC_29      28 /* DMA address increment after conversion of 29 samples */
#define ADC_DMA_ADD_INC_30      29 /* DMA address increment after conversion of 30 samples */
#define ADC_DMA_ADD_INC_31      30 /* DMA address increment after conversion of 31 samples */
#define ADC_DMA_ADD_INC_32      31 /* DMA address increment after conversion of 32 samples */
// </editor-fold>

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* SetupADCxChannels */

// ChannelToConvert
#define CONVERT_CH0_ONLY	0
#define CONVERT_CH01		1
#define CONVERT_CH0123		3

// AltInputModeEN
#define	MUXA_ONLY		0	/* Always uses channel input selects for MUX A */
#define	ALT_MUXA_MUXB		1	/* Uses channel input selects for MUX A on the first sample and MUX B on the next sample */

// Ch0ScanEN
#define	DIS_CH0_SCAN		0	/* Does not scan inputs for CH0 (CH0 as Normal Channel) */
#define	EN_CH0_SCAN		1	/* Scans inputs for CH0 during Sample MUXA bit */

/*************************************************************************/
// Channels Sellect Options For (dsPIC33EPXXX(GP/MC/MU)806/810/814 and PIC24EPXXX(GP/GU)810/814)  MCUs Group.
#ifdef GROUP1_DSPIC33E_PIC24E_FAMILY

// Ch0ScanInputs
// <editor-fold defaultstate="collapsed" desc="Ch0ScanInputs Args">
#define SCN_AN0		0xFFFFFFFE	/* Scan AN0 */
#define SCN_AN1		0xFFFFFFFD	/* Scan AN1 */
#define SCN_AN2		0xFFFFFFFB	/* Scan AN2 */
#define SCN_AN3		0xFFFFFFF7	/* Scan AN3 */
#define SCN_AN4		0xFFFFFFEF	/* Scan AN4 */
#define SCN_AN5		0xFFFFFFDF	/* Scan AN5 */
#define SCN_AN6		0xFFFFFFBF	/* Scan AN6 */
#define SCN_AN7		0xFFFFFF7F	/* Scan AN7 */
#define SCN_AN8		0xFFFFFEFF	/* Scan AN8 */
#define SCN_AN9		0xFFFFFDFF	/* Scan AN9 */
#define SCN_AN10	0xFFFFFBFF	/* Scan AN10 */
#define SCN_AN11	0xFFFFF7FF	/* Scan AN11 */
#define SCN_AN12	0xFFFFEFFF	/* Scan AN12 */
#define SCN_AN13	0xFFFFDFFF	/* Scan AN13 */
#define SCN_AN14	0xFFFFBFFF	/* Scan AN14 */
#define SCN_AN15	0xFFFF7FFF	/* Scan AN15 */
#define SCN_AN16	0xFFFEFFFF	/* Scan AN16 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define SCN_AN17	0xFFFDFFFF	/* Scan AN17 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define SCN_AN18	0xFFFBFFFF	/* Scan AN18 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define SCN_AN19	0xFFF7FFFF	/* Scan AN19 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define SCN_AN20	0xFFEFFFFF	/* Scan AN20 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define SCN_AN21	0xFFDFFFFF	/* Scan AN21 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define SCN_AN22	0xFFBFFFFF	/* Scan AN22 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define SCN_AN23	0xFF7FFFFF	/* Scan AN23 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define SCN_AN24	0xFEFFFFFF	/* Scan AN24 (ADC1 Module ONLY!) */
#define SCN_AN25	0xFDFFFFFF	/* Scan AN25 (ADC1 Module ONLY!) */
#define SCN_AN26	0xFBFFFFFF	/* Scan AN26 (ADC1 Module ONLY!) */
#define SCN_AN27	0xF7FFFFFF	/* Scan AN27 (ADC1 Module ONLY!) */
#define SCN_AN28	0xEFFFFFFF	/* Scan AN28 (ADC1 Module ONLY!) */
#define SCN_AN29	0xDFFFFFFF	/* Scan AN29 (ADC1 Module ONLY!) */
#define SCN_AN30	0xBFFFFFFF	/* Scan AN30 (ADC1 Module ONLY!) */
#define SCN_AN31	0x7FFFFFFF	/* Scan AN31 (ADC1 Module ONLY!) */
#define SCN_ALL_0_15	0xFFFF0000	/* Scan All Inputs From  AN0  To  AN15 inclusively */
#define SCN_ALL_16_31	0x0000FFFF	/* Scan All Inputs From  AN16 To  AN31 inclusively (ADC1 Module ONLY!) */
#define SCN_ALL_0_31	0x00000000	/* Scan All Inputs From  AN0  To  AN31 inclusively (ADC1 Module ONLY!) */
#define SCN_NONE	0xFFFFFFFF	/* Scan Nothing */
// </editor-fold>

// Ch0PosInputMuxA
// <editor-fold defaultstate="collapsed" desc="Ch0PosInputMuxA defines">
#define ADC_MUXA_CH0_POS_AN0	0	/* ADC MuxA CH0 Positive Input is AN0 */
#define ADC_MUXA_CH0_POS_AN1	1	/* ADC MuxA CH0 Positive Input is AN1 */
#define ADC_MUXA_CH0_POS_AN2	2	/* ADC MuxA CH0 Positive Input is AN2 */
#define ADC_MUXA_CH0_POS_AN3	3	/* ADC MuxA CH0 Positive Input is AN3 */
#define ADC_MUXA_CH0_POS_AN4	4	/* ADC MuxA CH0 Positive Input is AN4 */
#define ADC_MUXA_CH0_POS_AN5	5	/* ADC MuxA CH0 Positive Input is AN5 */
#define ADC_MUXA_CH0_POS_AN6	6	/* ADC MuxA CH0 Positive Input is AN6 */
#define ADC_MUXA_CH0_POS_AN7	7	/* ADC MuxA CH0 Positive Input is AN7 */
#define ADC_MUXA_CH0_POS_AN8	8	/* ADC MuxA CH0 Positive Input is AN8 */
#define ADC_MUXA_CH0_POS_AN9	9	/* ADC MuxA CH0 Positive Input is AN9 */
#define ADC_MUXA_CH0_POS_AN10	10	/* ADC MuxA CH0 Positive Input is AN10 */
#define ADC_MUXA_CH0_POS_AN11	11	/* ADC MuxA CH0 Positive Input is AN11 */
#define ADC_MUXA_CH0_POS_AN12	12	/* ADC MuxA CH0 Positive Input is AN12 */
#define ADC_MUXA_CH0_POS_AN13	13	/* ADC MuxA CH0 Positive Input is AN13 */
#define ADC_MUXA_CH0_POS_AN14	14	/* ADC MuxA CH0 Positive Input is AN14 */
#define ADC_MUXA_CH0_POS_AN15	15	/* ADC MuxA CH0 Positive Input is AN15 */
#define ADC_MUXA_CH0_POS_AN16	16	/* ADC MuxA CH0 Positive Input is AN16 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define ADC_MUXA_CH0_POS_AN17	17	/* ADC MuxA CH0 Positive Input is AN17 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define ADC_MUXA_CH0_POS_AN18	18	/* ADC MuxA CH0 Positive Input is AN18 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define ADC_MUXA_CH0_POS_AN19	19	/* ADC MuxA CH0 Positive Input is AN19 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define ADC_MUXA_CH0_POS_AN20	20	/* ADC MuxA CH0 Positive Input is AN20 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define ADC_MUXA_CH0_POS_AN21	21	/* ADC MuxA CH0 Positive Input is AN21 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define ADC_MUXA_CH0_POS_AN22	22	/* ADC MuxA CH0 Positive Input is AN22 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define ADC_MUXA_CH0_POS_AN23	23	/* ADC MuxA CH0 Positive Input is AN23 (ADC1 Module ONLY! & Not Avaliable For (64-pin) devices.) */
#define ADC_MUXA_CH0_POS_AN24	24	/* ADC MuxA CH0 Positive Input is AN24 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN25	25	/* ADC MuxA CH0 Positive Input is AN25 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN26	26	/* ADC MuxA CH0 Positive Input is AN26 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN27	27	/* ADC MuxA CH0 Positive Input is AN27 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN28	28	/* ADC MuxA CH0 Positive Input is AN28 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN29	29	/* ADC MuxA CH0 Positive Input is AN29 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN30	30	/* ADC MuxA CH0 Positive Input is AN30 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN31	31	/* ADC MuxA CH0 Positive Input is AN31 (ADC1 Module ONLY!) */
// </editor-fold>

// Ch0NegInputMuxA
#define ADC_MUXA_CH0_NEG_VREFL	0	/* ADC MuxA CH0 Negative Input is VRefL */
#define ADC_MUXA_CH0_NEG_AN1	1	/* ADC MuxA CH0 Negative Input is AN1   */

// Ch0PosInputMuxB
// <editor-fold defaultstate="collapsed" desc="Ch0PosInputMuxB defines">
#define ADC_MUXB_CH0_POS_AN0	0	/* ADC MuxB CH0 Positive Input is AN0 */
#define ADC_MUXB_CH0_POS_AN1	1	/* ADC MuxB CH0 Positive Input is AN1 */
#define ADC_MUXB_CH0_POS_AN2	2	/* ADC MuxB CH0 Positive Input is AN2 */
#define ADC_MUXB_CH0_POS_AN3	3	/* ADC MuxB CH0 Positive Input is AN3 */
#define ADC_MUXB_CH0_POS_AN4	4	/* ADC MuxB CH0 Positive Input is AN4 */
#define ADC_MUXB_CH0_POS_AN5	5	/* ADC MuxB CH0 Positive Input is AN5 */
#define ADC_MUXB_CH0_POS_AN6	6	/* ADC MuxB CH0 Positive Input is AN6 */
#define ADC_MUXB_CH0_POS_AN7	7	/* ADC MuxB CH0 Positive Input is AN7 */
#define ADC_MUXB_CH0_POS_AN8	8	/* ADC MuxB CH0 Positive Input is AN8 */
#define ADC_MUXB_CH0_POS_AN9	9	/* ADC MuxB CH0 Positive Input is AN9 */
#define ADC_MUXB_CH0_POS_AN10	10	/* ADC MuxB CH0 Positive Input is AN10 */
#define ADC_MUXB_CH0_POS_AN11	11	/* ADC MuxB CH0 Positive Input is AN11 */
#define ADC_MUXB_CH0_POS_AN12	12	/* ADC MuxB CH0 Positive Input is AN12 */
#define ADC_MUXB_CH0_POS_AN13	13	/* ADC MuxB CH0 Positive Input is AN13 */
#define ADC_MUXB_CH0_POS_AN14	14	/* ADC MuxB CH0 Positive Input is AN14 */
#define ADC_MUXB_CH0_POS_AN15	15	/* ADC MuxB CH0 Positive Input is AN15 */
#define ADC_MUXB_CH0_POS_AN16	16	/* ADC MuxB CH0 Positive Input is AN16 */
#define ADC_MUXB_CH0_POS_AN17	17	/* ADC MuxB CH0 Positive Input is AN17 */
#define ADC_MUXB_CH0_POS_AN18	18	/* ADC MuxB CH0 Positive Input is AN18 */
#define ADC_MUXB_CH0_POS_AN19	19	/* ADC MuxB CH0 Positive Input is AN19 */
#define ADC_MUXB_CH0_POS_AN20	20	/* ADC MuxB CH0 Positive Input is AN20 */
#define ADC_MUXB_CH0_POS_AN21	21	/* ADC MuxB CH0 Positive Input is AN21 */
#define ADC_MUXB_CH0_POS_AN22	22	/* ADC MuxB CH0 Positive Input is AN22 */
#define ADC_MUXB_CH0_POS_AN23	23	/* ADC MuxB CH0 Positive Input is AN23 */
#define ADC_MUXB_CH0_POS_AN24	24	/* ADC MuxB CH0 Positive Input is AN24 */
#define ADC_MUXB_CH0_POS_AN25	25	/* ADC MuxB CH0 Positive Input is AN25 */
#define ADC_MUXB_CH0_POS_AN26	26	/* ADC MuxB CH0 Positive Input is AN26 */
#define ADC_MUXB_CH0_POS_AN27	27	/* ADC MuxB CH0 Positive Input is AN27 */
#define ADC_MUXB_CH0_POS_AN28	28	/* ADC MuxB CH0 Positive Input is AN28 */
#define ADC_MUXB_CH0_POS_AN29	29	/* ADC MuxB CH0 Positive Input is AN29 */
#define ADC_MUXB_CH0_POS_AN30	30	/* ADC MuxB CH0 Positive Input is AN30 */
#define ADC_MUXB_CH0_POS_AN31	31	/* ADC MuxB CH0 Positive Input is AN31 */
// </editor-fold>

// Ch0NegInputMuxB
#define ADC_MUXB_CH0_NEG_VREFL	0	/* ADC MuxB CH0 Negative Input is VRefL */
#define ADC_MUXB_CH0_NEG_AN1	1	/* ADC MuxB CH0 Negative Input is AN1   */

// Ch123PosInputsMuxA
#define ADC_MUXA_CH123_POS_AN0_AN1_AN2		0 /* ADC MuxA, AN0 => +CH1, AN1 => +CH2, AN2 => +CH2*/
#define ADC_MUXA_CH123_POS_AN3_AN4_AN5		1 /* ADC MuxA, AN3 => +CH1, AN4 => +CH2, AN5 => +CH3*/

// Ch123NegInputsMuxA
#define ADC_MUXA_CH123_NEG_VREFL		0 /* ADC MuxA, VrefL => -CH1,-CH2,-CH3*/
#define ADC_MUXA_CH123_NEG_AN6_AN7_AN8		2 /* ADC MuxA, AN6 => -CH1, AN7 => -CH2, AN8 => -CH3*/
#define ADC_MUXA_CH123_NEG_AN9_AN10_AN11	3 /* ADC MuxA, AN9 => -CH1, AN10 => -CH2, AN11 => -CH3*/

// Ch123PosInputsMuxB
#define ADC_MUXB_CH123_POS_AN0_AN1_AN2		0 /* ADC MuxB, AN0 => +CH1, AN1 => +CH2, AN2 => +CH2*/
#define ADC_MUXB_CH123_POS_AN3_AN4_AN5		1 /* ADC MuxB, AN3 => +CH1, AN4 => +CH2, AN5 => +CH3*/

// Ch123NegInputsMuxB
#define ADC_MUXB_CH123_NEG_VREFL		0 /* ADC MuxB, VrefL => -CH1,-CH2,-CH3*/
#define ADC_MUXB_CH123_NEG_AN6_AN7_AN8		2 /* ADC MuxB, AN6 => -CH1, AN7 => -CH2, AN8 => -CH3*/
#define ADC_MUXB_CH123_NEG_AN9_AN10_AN11	3 /* ADC MuxB, AN9 => -CH1, AN10 => -CH2, AN11 => -CH3*/

#endif

/*************************************************************************/
// Channels Sellect Options For (dsPIC33EPXXX(GM)3XX/6XX/7XX)  MCUs Group.
#ifdef GROUP2_DSPIC33E_PIC24E_FAMILY

// Ch0ScanInputs
// <editor-fold defaultstate="collapsed" desc="Ch0ScanInputs Args">
#define SCN_AN0		0xFFFFFFFE	/* Scan AN0 */
#define SCN_AN1		0xFFFFFFFD	/* Scan AN1 */
#define SCN_AN2		0xFFFFFFFB	/* Scan AN2 */
#define SCN_AN3		0xFFFFFFF7	/* Scan AN3 */
#define SCN_AN4		0xFFFFFFEF	/* Scan AN4 */
#define SCN_AN5		0xFFFFFFDF	/* Scan AN5 */
#define SCN_AN6		0xFFFFFFBF	/* Scan AN6 */
#define SCN_AN7		0xFFFFFF7F	/* Scan AN7 */
#define SCN_AN8		0xFFFFFEFF	/* Scan AN8 */
#define SCN_AN9		0xFFFFFDFF	/* Scan AN9 */
#define SCN_AN10	0xFFFFFBFF	/* Scan AN10 */
#define SCN_AN11	0xFFFFF7FF	/* Scan AN11 */
#define SCN_AN12	0xFFFFEFFF	/* Scan AN12 */
#define SCN_AN13	0xFFFFDFFF	/* Scan AN13 */
#define SCN_AN14	0xFFFFBFFF	/* Scan AN14 */
#define SCN_AN15	0xFFFF7FFF	/* Scan AN15 */
#define SCN_AN16	0xFFFEFFFF	/* Scan AN16 */
#define SCN_AN17	0xFFFDFFFF	/* Scan AN17 */
#define SCN_AN18	0xFFFBFFFF	/* Scan AN18 */
#define SCN_AN19	0xFFF7FFFF	/* Scan AN19 */
#define SCN_AN20	0xFFEFFFFF	/* Scan AN20 */
#define SCN_AN21	0xFFDFFFFF	/* Scan AN21 */
#define SCN_AN22	0xFFBFFFFF	/* Scan AN22 */
#define SCN_AN23	0xFF7FFFFF	/* Scan AN23 */
#define SCN_OA1AN3	0xFEFFFFFF	/* Scan AN3 or OpAmp1 output voltage if CMxCON.OPMODE=1 */
#define SCN_OA2AN0	0xFDFFFFFF	/* Scan AN0 or OpAmp2 output voltage if CMxCON.OPMODE=1 */
#define SCN_OA3AN6	0xFBFFFFFF	/* Scan AN6 or OpAmp3 output voltage if CMxCON.OPMODE=1 */
#define SCN_AN27	0xF7FFFFFF	/* Scan AN27 */
#define SCN_AN28	0xEFFFFFFF	/* Scan AN28 */
#define SCN_AN29	0xDFFFFFFF	/* Scan AN29 */
#define SCN_AN30	0xBFFFFFFF	/* Scan AN30 */
#define SCN_AN31	0x7FFFFFFF	/* Scan AN31 */
#define SCN_ALL_0_15	0xFFFF0000	/* Scan All Inputs From  AN0  To  AN15 inclusively */
#define SCN_ALL_16_31	0x0000FFFF	/* Scan All Inputs From  AN16 To  AN31 inclusively (Note: AN24=OA1AN3 & AN25=OA2AN0 & AN26=OA3AN6)*/
#define SCN_ALL_0_31	0x00000000	/* Scan All Inputs From  AN0  To  AN31 inclusively (Note: AN24=OA1AN3 & AN25=OA2AN0 & AN26=OA3AN6)*/
#define SCN_NONE	0xFFFFFFFF	/* Scan AN31 */
// </editor-fold>

// Ch0PosInputMuxA
// <editor-fold defaultstate="collapsed" desc="Ch0PosInputMuxA Args">
#define ADC_MUXA_CH0_POS_OA2AN0		0	/* ADC MuxA CH0 Positive Input is AN0 or OpAmp2 output voltage if CMxCON.OPMODE=1 */
#define ADC_MUXA_CH0_POS_AN1		1	/* ADC MuxA CH0 Positive Input is AN1 */
#define ADC_MUXA_CH0_POS_AN2		2	/* ADC MuxA CH0 Positive Input is AN2 */
#define ADC_MUXA_CH0_POS_OA1AN3		3	/* ADC MuxA CH0 Positive Input is AN3 or OpAmp1 output voltage if CMxCON.OPMODE=1 */
#define ADC_MUXA_CH0_POS_AN4		4	/* ADC MuxA CH0 Positive Input is AN4 */
#define ADC_MUXA_CH0_POS_AN5		5	/* ADC MuxA CH0 Positive Input is AN5 */
#define ADC_MUXA_CH0_POS_OA3AN6		6	/* ADC MuxA CH0 Positive Input is AN6 or OpAmp3 output voltage if CMxCON.OPMODE=1 */
#define ADC_MUXA_CH0_POS_AN7		7	/* ADC MuxA CH0 Positive Input is AN7 */
#define ADC_MUXA_CH0_POS_AN8		8	/* ADC MuxA CH0 Positive Input is AN8 */
#define ADC_MUXA_CH0_POS_AN9		9	/* ADC MuxA CH0 Positive Input is AN9 */
#define ADC_MUXA_CH0_POS_AN10		10	/* ADC MuxA CH0 Positive Input is AN10 */
#define ADC_MUXA_CH0_POS_AN11		11	/* ADC MuxA CH0 Positive Input is AN11 */
#define ADC_MUXA_CH0_POS_AN12		12	/* ADC MuxA CH0 Positive Input is AN12 */
#define ADC_MUXA_CH0_POS_AN13		13	/* ADC MuxA CH0 Positive Input is AN13 */
#define ADC_MUXA_CH0_POS_AN14		14	/* ADC MuxA CH0 Positive Input is AN14 */
#define ADC_MUXA_CH0_POS_AN15		15	/* ADC MuxA CH0 Positive Input is AN15 */
#define ADC_MUXA_CH0_POS_AN16		16	/* ADC MuxA CH0 Positive Input is AN16 */
#define ADC_MUXA_CH0_POS_AN17		17	/* ADC MuxA CH0 Positive Input is AN17 */
#define ADC_MUXA_CH0_POS_AN18		18	/* ADC MuxA CH0 Positive Input is AN18 */
#define ADC_MUXA_CH0_POS_AN19		19	/* ADC MuxA CH0 Positive Input is AN19 */
#define ADC_MUXA_CH0_POS_AN20		20	/* ADC MuxA CH0 Positive Input is AN20 */
#define ADC_MUXA_CH0_POS_AN21		21	/* ADC MuxA CH0 Positive Input is AN21 */
#define ADC_MUXA_CH0_POS_AN22		22	/* ADC MuxA CH0 Positive Input is AN22 */
#define ADC_MUXA_CH0_POS_AN23		23	/* ADC MuxA CH0 Positive Input is AN23 */
#define ADC_MUXA_CH0_POS_AN24		24	/* ADC MuxA CH0 Positive Input is AN24 */
#define ADC_MUXA_CH0_POS_OA5AN25	25	/* ADC MuxA CH0 Positive Input is AN25 or OpAmp5 output voltage if CMxCON.OPMODE=1 */
#define ADC_MUXA_CH0_POS_AN26		26	/* ADC MuxA CH0 Positive Input is AN26 */
#define ADC_MUXA_CH0_POS_AN27		27	/* ADC MuxA CH0 Positive Input is AN27 */
#define ADC_MUXA_CH0_POS_AN28		28	/* ADC MuxA CH0 Positive Input is AN28 */
#define ADC_MUXA_CH0_POS_AN29		29	/* ADC MuxA CH0 Positive Input is AN29 */
#define ADC_MUXA_CH0_POS_AN30		30	/* ADC MuxA CH0 Positive Input is AN30 */
#define ADC_MUXA_CH0_POS_AN31		31	/* ADC MuxA CH0 Positive Input is AN31 */
#define ADC_MUXA_CH0_POS_AN32		32	/* ADC MuxA CH0 Positive Input is AN32 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN33		33	/* ADC MuxA CH0 Positive Input is AN33 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN34		34	/* ADC MuxA CH0 Positive Input is AN34 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN35		35	/* ADC MuxA CH0 Positive Input is AN35 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN36		36	/* ADC MuxA CH0 Positive Input is AN36 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN37		37	/* ADC MuxA CH0 Positive Input is AN37 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN38		38	/* ADC MuxA CH0 Positive Input is AN38 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN39		39	/* ADC MuxA CH0 Positive Input is AN39 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN40		40	/* ADC MuxA CH0 Positive Input is AN40 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN41		41	/* ADC MuxA CH0 Positive Input is AN41 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN42		42	/* ADC MuxA CH0 Positive Input is AN42 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN43		43	/* ADC MuxA CH0 Positive Input is AN43 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN44		44	/* ADC MuxA CH0 Positive Input is AN44 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN45		45	/* ADC MuxA CH0 Positive Input is AN45 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN46		46	/* ADC MuxA CH0 Positive Input is AN46 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN47		47	/* ADC MuxA CH0 Positive Input is AN47 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN48		48	/* ADC MuxA CH0 Positive Input is AN48 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN49		49	/* ADC MuxA CH0 Positive Input is AN49 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXA_CH0_POS_AN50		50	/* ADC MuxA CH0 Positive Input is AN50 (ADC1 Module ONLY!) (reserved) */
#define ADC_MUXA_CH0_POS_AN51		51	/* ADC MuxA CH0 Positive Input is AN51 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN52		52	/* ADC MuxA CH0 Positive Input is AN52 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN53		53	/* ADC MuxA CH0 Positive Input is AN53 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN54		54	/* ADC MuxA CH0 Positive Input is AN54 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN55		55	/* ADC MuxA CH0 Positive Input is AN55 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN56		56	/* ADC MuxA CH0 Positive Input is AN56 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN57		57	/* ADC MuxA CH0 Positive Input is AN57 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN58		58	/* ADC MuxA CH0 Positive Input is AN58 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN59		59	/* ADC MuxA CH0 Positive Input is AN59 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN60		60	/* ADC MuxA CH0 Positive Input is AN60 (ADC1 Module ONLY!) */
#define ADC_MUXA_CH0_POS_AN61		61	/* ADC MuxA CH0 Positive Input is AN61 (ADC1 Module ONLY!) (reserved) */
#define ADC_MUXA_CH0_POS_AN62_CTMU_TEMP		62	/* ADC MuxA CH0 Positive Input is AN62 (ADC1 Module ONLY!) CTMU temperature voltage */
#define ADC_MUXA_CH0_POS_AN63_UNCONNECTED	63	/* ADC MuxA CH0 Positive Input is AN63 (ADC1 Module ONLY!) unconnected */
// </editor-fold>

// Ch0NegInputMuxA
#define ADC_MUXA_CH0_NEG_VREFL		0	/* ADC MuxA CH0 Negative Input is VRefL */
#define ADC_MUXA_CH0_NEG_AN1		1	/* ADC MuxA CH0 Negative Input is AN1   */

// Ch0PosInputMuxB
// <editor-fold defaultstate="collapsed" desc="Ch0PosInputMuxB Args">
#define ADC_MUXB_CH0_POS_AN0	0	/* ADC MuxB CH0 Positive Input is AN0 or OpAmp 2 output voltage if CMxCON.OPMODE=1 */
#define ADC_MUXB_CH0_POS_AN1	1	/* ADC MuxB CH0 Positive Input is AN1 */
#define ADC_MUXB_CH0_POS_AN2	2	/* ADC MuxB CH0 Positive Input is AN2 */
#define ADC_MUXB_CH0_POS_AN3	3	/* ADC MuxB CH0 Positive Input is AN3 or OpAmp 1 output voltage if CMxCON.OPMODE=1 */
#define ADC_MUXB_CH0_POS_AN4	4	/* ADC MuxB CH0 Positive Input is AN4 */
#define ADC_MUXB_CH0_POS_AN5	5	/* ADC MuxB CH0 Positive Input is AN5 */
#define ADC_MUXB_CH0_POS_AN6	6	/* ADC MuxB CH0 Positive Input is AN6 or OpAmp 3 output voltage if CMxCON.OPMODE=1 */
#define ADC_MUXB_CH0_POS_AN7	7	/* ADC MuxB CH0 Positive Input is AN7 */
#define ADC_MUXB_CH0_POS_AN8	8	/* ADC MuxB CH0 Positive Input is AN8 */
#define ADC_MUXB_CH0_POS_AN9	9	/* ADC MuxB CH0 Positive Input is AN9 */
#define ADC_MUXB_CH0_POS_AN10	10	/* ADC MuxB CH0 Positive Input is AN10 */
#define ADC_MUXB_CH0_POS_AN11	11	/* ADC MuxB CH0 Positive Input is AN11 */
#define ADC_MUXB_CH0_POS_AN12	12	/* ADC MuxB CH0 Positive Input is AN12 */
#define ADC_MUXB_CH0_POS_AN13	13	/* ADC MuxB CH0 Positive Input is AN13 */
#define ADC_MUXB_CH0_POS_AN14	14	/* ADC MuxB CH0 Positive Input is AN14 */
#define ADC_MUXB_CH0_POS_AN15	15	/* ADC MuxB CH0 Positive Input is AN15 */
#define ADC_MUXB_CH0_POS_AN16	16	/* ADC MuxB CH0 Positive Input is AN16 */
#define ADC_MUXB_CH0_POS_AN17	17	/* ADC MuxB CH0 Positive Input is AN17 */
#define ADC_MUXB_CH0_POS_AN18	18	/* ADC MuxB CH0 Positive Input is AN18 */
#define ADC_MUXB_CH0_POS_AN19	19	/* ADC MuxB CH0 Positive Input is AN19 */
#define ADC_MUXB_CH0_POS_AN20	20	/* ADC MuxB CH0 Positive Input is AN20 */
#define ADC_MUXB_CH0_POS_AN21	21	/* ADC MuxB CH0 Positive Input is AN21 */
#define ADC_MUXB_CH0_POS_AN22	22	/* ADC MuxB CH0 Positive Input is AN22 */
#define ADC_MUXB_CH0_POS_AN23	23	/* ADC MuxB CH0 Positive Input is AN23 */
#define ADC_MUXB_CH0_POS_AN24	24	/* ADC MuxB CH0 Positive Input is AN24 */
#define ADC_MUXB_CH0_POS_AN25	25	/* ADC MuxB CH0 Positive Input is AN25 or OpAmp 5 output voltage if CMxCON.OPMODE=1 */
#define ADC_MUXB_CH0_POS_AN26	26	/* ADC MuxB CH0 Positive Input is AN26 */
#define ADC_MUXB_CH0_POS_AN27	27	/* ADC MuxB CH0 Positive Input is AN27 */
#define ADC_MUXB_CH0_POS_AN28	28	/* ADC MuxB CH0 Positive Input is AN28 */
#define ADC_MUXB_CH0_POS_AN29	29	/* ADC MuxB CH0 Positive Input is AN29 */
#define ADC_MUXB_CH0_POS_AN30	30	/* ADC MuxB CH0 Positive Input is AN30 */
#define ADC_MUXB_CH0_POS_AN31	31	/* ADC MuxB CH0 Positive Input is AN31 */
#define ADC_MUXB_CH0_POS_AN32	32	/* ADC MuxB CH0 Positive Input is AN32 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN33	33	/* ADC MuxB CH0 Positive Input is AN33 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN34	34	/* ADC MuxB CH0 Positive Input is AN34 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN35	35	/* ADC MuxB CH0 Positive Input is AN35 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN36	36	/* ADC MuxB CH0 Positive Input is AN36 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN37	37	/* ADC MuxB CH0 Positive Input is AN37 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN38	38	/* ADC MuxB CH0 Positive Input is AN38 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN39	39	/* ADC MuxB CH0 Positive Input is AN39 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN40	40	/* ADC MuxB CH0 Positive Input is AN40 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN41	41	/* ADC MuxB CH0 Positive Input is AN41 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN42	42	/* ADC MuxB CH0 Positive Input is AN42 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN43	43	/* ADC MuxB CH0 Positive Input is AN43 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN44	44	/* ADC MuxB CH0 Positive Input is AN44 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN45	45	/* ADC MuxB CH0 Positive Input is AN45 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN46	46	/* ADC MuxB CH0 Positive Input is AN46 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN47	47	/* ADC MuxB CH0 Positive Input is AN47 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN48	48	/* ADC MuxB CH0 Positive Input is AN48 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN49	49	/* ADC MuxB CH0 Positive Input is AN49 (ADC1 Module & 10 bit ONLY!) */
#define ADC_MUXB_CH0_POS_AN50	50	/* ADC MuxB CH0 Positive Input is AN50 (ADC1 Module ONLY!) (reserved) */
#define ADC_MUXB_CH0_POS_AN51	51	/* ADC MuxB CH0 Positive Input is AN51 (ADC1 Module ONLY!) */
#define ADC_MUXB_CH0_POS_AN52	52	/* ADC MuxB CH0 Positive Input is AN52 (ADC1 Module ONLY!) */
#define ADC_MUXB_CH0_POS_AN53	53	/* ADC MuxB CH0 Positive Input is AN53 (ADC1 Module ONLY!) */
#define ADC_MUXB_CH0_POS_AN54	54	/* ADC MuxB CH0 Positive Input is AN54 (ADC1 Module ONLY!) */
#define ADC_MUXB_CH0_POS_AN55	55	/* ADC MuxB CH0 Positive Input is AN55 (ADC1 Module ONLY!) */
#define ADC_MUXB_CH0_POS_AN56	56	/* ADC MuxB CH0 Positive Input is AN56 (ADC1 Module ONLY!) */
#define ADC_MUXB_CH0_POS_AN57	57	/* ADC MuxB CH0 Positive Input is AN57 (ADC1 Module ONLY!) */
#define ADC_MUXB_CH0_POS_AN58	58	/* ADC MuxB CH0 Positive Input is AN58 (ADC1 Module ONLY!) */
#define ADC_MUXB_CH0_POS_AN59	59	/* ADC MuxB CH0 Positive Input is AN59 (ADC1 Module ONLY!) */
#define ADC_MUXB_CH0_POS_AN60	60	/* ADC MuxB CH0 Positive Input is AN60 (ADC1 Module ONLY!) */
#define ADC_MUXB_CH0_POS_AN61	61	/* ADC MuxB CH0 Positive Input is AN61 (ADC1 Module ONLY!) (reserved) */
#define ADC_MUXB_CH0_POS_AN62	62	/* ADC MuxB CH0 Positive Input is AN62 (ADC1 Module ONLY!) CTMU temperature voltage */
#define ADC_MUXB_CH0_POS_AN63	63	/* ADC MuxB CH0 Positive Input is AN63 (ADC1 Module ONLY!) unconnected */
// </editor-fold>

// Ch0NegInputMuxB
#define ADC_MUXB_CH0_NEG_VREFL		0	/* ADC MuxB CH0 Negative Input is VRefL */
#define ADC_MUXB_CH0_NEG_AN1		1	/* ADC MuxB CH0 Negative Input is AN1   */

// Ch123PosInputsMuxA
#define ADC_MUXA_CH123_POS_AN0_AN1_AN2			0 /* ADC MuxA, AN0 => +CH1, AN1 => +CH2, AN2 => +CH2*/
#define ADC_MUXA_CH123_POS_AN3_AN4_AN5			1 /* ADC MuxA, AN3 => +CH1, AN4 => +CH2, AN5 => +CH3*/
#define ADC_MUXA_CH123_POS_OA1AN3_OA2AN0_OA3AN6		2 /* ADC MuxA, AN3 or OpAmp1 => +CH1, AN0 or OpAmp2 => +CH2, AN6 or OpAmp3 => +CH3*/
#define ADC_MUXA_CH123_POS_OA1AN3_OA2AN0_OA5AN25	3 /* ADC MuxA, AN3 or OpAmp1 => +CH1, AN0 or OpAmp2 => +CH2, AN25 or OpAmp5 => +CH3*/
#define ADC_MUXA_CH123_POS_OA2AN0_OA5AN25_OA3AN6	4 /* ADC MuxA, AN0 or OpAmp2 => +CH1, AN25 or OpAmp5 => +CH2, AN6 or OpAmp6 => +CH3*/

// Ch123NegInputsMuxA
#define ADC_MUXA_CH123_NEG_VREFL			0 /* ADC MuxA, VrefL => -CH1,-CH2,-CH3*/
#define ADC_MUXA_CH123_NEG_AN6_AN7_AN8			2 /* ADC MuxA, AN6 => -CH1, AN7 => -CH2, AN8 => -CH3*/
#define ADC_MUXA_CH123_NEG_AN9_AN10_AN11		3 /* ADC MuxA, AN9 => -CH1, AN10 => -CH2, AN11 => -CH3*/

// Ch123PosInputsMuxB
#define ADC_MUXB_CH123_POS_AN0_AN1_AN2			0 /* ADC MuxB, AN0 => +CH1, AN1 => +CH2, AN2 => +CH2*/
#define ADC_MUXB_CH123_POS_AN3_AN4_AN5			1 /* ADC MuxB, AN3 => +CH1, AN4 => +CH2, AN5 => +CH3*/
#define ADC_MUXB_CH123_POS_OA1AN3_OA2AN0_OA3AN6		2 /* ADC MuxB, AN3 or OpAmp1 => +CH1, AN0 or OpAmp2 => +CH2, AN6 or OpAmp3 => +CH3*/
#define ADC_MUXB_CH123_POS_OA1AN3_OA2AN0_OA5AN25	3 /* ADC MuxB, AN3 or OpAmp1 => +CH1, AN0 or OpAmp2 => +CH2, AN25 or OpAmp5 => +CH3*/
#define ADC_MUXB_CH123_POS_OA2AN0_OA5AN25_OA3AN6	4 /* ADC MuxB, AN0 or OpAmp2 => +CH1, AN25 or OpAmp5 => +CH2, AN6 or OpAmp6 => +CH3*/

// Ch123NegInputsMuxB
#define ADC_MUXB_CH123_NEG_VREFL			0 /* ADC MuxB, VrefL => -CH1,-CH2,-CH3*/
#define ADC_MUXB_CH123_NEG_AN6_AN7_AN8			2 /* ADC MuxB, AN6 => -CH1, AN7 => -CH2, AN8 => -CH3*/
#define ADC_MUXB_CH123_NEG_AN9_AN10_AN11		3 /* ADC MuxB, AN9 => -CH1, AN10 => -CH2, AN11 => -CH3*/

#endif

/*************************************************************************/
// Channels Sellect Options For (dsPIC33EPXXXGP50X, dsPIC33EPXXXMC20X/50X AND PIC24EPXXXGP/MC20X)  MCUs Group.
#ifdef GROUP3_DSPIC33E_PIC24E_FAMILY

// Ch0ScanInputs
// <editor-fold defaultstate="collapsed" desc="Ch0ScanInputs Args">
#define SCN_AN0			0xFFFFFFFE	/* Scan AN0 */
#define SCN_AN1			0xFFFFFFFD	/* Scan AN1 */
#define SCN_AN2			0xFFFFFFFB	/* Scan AN2 */
#define SCN_AN3			0xFFFFFFF7	/* Scan AN3 */
#define SCN_AN4			0xFFFFFFEF	/* Scan AN4 */
#define SCN_AN5			0xFFFFFFDF	/* Scan AN5 */
#define SCN_AN6			0xFFFFFFBF	/* Scan AN6 */
#define SCN_AN7			0xFFFFFF7F	/* Scan AN7 */
#define SCN_AN8			0xFFFFFEFF	/* Scan AN8 */
#define SCN_AN9			0xFFFFFDFF	/* Scan AN9 */
#define SCN_AN10		0xFFFFFBFF	/* Scan AN10 */
#define SCN_AN11		0xFFFFF7FF	/* Scan AN11 */
#define SCN_AN12		0xFFFFEFFF	/* Scan AN12 */
#define SCN_AN13		0xFFFFDFFF	/* Scan AN13 */
#define SCN_AN14		0xFFFFBFFF	/* Scan AN14 */
#define SCN_AN15		0xFFFF7FFF	/* Scan AN15 */
#define SCN_OA1AN3		0xFEFFFFFF	/* Scan AN3 or OpAmp1 output voltage if CMxCON.OPMODE = 1 */
#define SCN_OA2AN0		0xFDFFFFFF	/* Scan AN0 or OpAmp2 output voltage if CMxCON.OPMODE = 1 */
#define SCN_OA3AN6		0xFBFFFFFF	/* Scan AN6 or OpAmp3 output voltage if CMxCON.OPMODE = 1 */
#define SCN_CTMU_TEMP		0xBFFFFFFF	/* Scan CTMU Tempreture Diode Output */
#define SCN_OPEN_CTMU_CAPTIME	0x7FFFFFFF	/* Scan CTMU capacitive and time measurement (Open) */
#define SCN_ALL_0_15		0xFFFF0000	/* Scan All Inputs From  AN0  To  AN15 inclusively */
#define SCN_NONE		0xFFFFFFFF	/* Scan Nothing */
// </editor-fold>

// Ch0PosInputMuxA
// <editor-fold defaultstate="collapsed" desc="Ch0PosInputMuxA defines">
#define ADC_MUXA_CH0_POS_AN0			0	/* ADC MuxA CH0 Positive Input is AN0 */
#define ADC_MUXA_CH0_POS_AN1			1	/* ADC MuxA CH0 Positive Input is AN1 */
#define ADC_MUXA_CH0_POS_AN2			2	/* ADC MuxA CH0 Positive Input is AN2 */
#define ADC_MUXA_CH0_POS_AN3			3	/* ADC MuxA CH0 Positive Input is AN3 */
#define ADC_MUXA_CH0_POS_AN4			4	/* ADC MuxA CH0 Positive Input is AN4 */
#define ADC_MUXA_CH0_POS_AN5			5	/* ADC MuxA CH0 Positive Input is AN5 */
#define ADC_MUXA_CH0_POS_AN6			6	/* ADC MuxA CH0 Positive Input is AN6 */
#define ADC_MUXA_CH0_POS_AN7			7	/* ADC MuxA CH0 Positive Input is AN7 */
#define ADC_MUXA_CH0_POS_AN8			8	/* ADC MuxA CH0 Positive Input is AN8 */
#define ADC_MUXA_CH0_POS_AN9			9	/* ADC MuxA CH0 Positive Input is AN9 */
#define ADC_MUXA_CH0_POS_AN10			10	/* ADC MuxA CH0 Positive Input is AN10 */
#define ADC_MUXA_CH0_POS_AN11			11	/* ADC MuxA CH0 Positive Input is AN11 */
#define ADC_MUXA_CH0_POS_AN12			12	/* ADC MuxA CH0 Positive Input is AN12 */
#define ADC_MUXA_CH0_POS_AN13			13	/* ADC MuxA CH0 Positive Input is AN13 */
#define ADC_MUXA_CH0_POS_AN14			14	/* ADC MuxA CH0 Positive Input is AN14 */
#define ADC_MUXA_CH0_POS_AN15			15	/* ADC MuxA CH0 Positive Input is AN15 */
#define ADC_MUXA_CH0_POS_OA1AN3			48	/* ADC MuxA CH0 Positive Input is AN3 or OpAmp1 output if CMxCON.OPMODE=1 */
#define ADC_MUXA_CH0_POS_OA2AN0			49	/* ADC MuxA CH0 Positive Input is AN0 or OpAmp2 output if CMxCON.OPMODE=1 */
#define ADC_MUXA_CH0_POS_OA3AN6			50	/* ADC MuxA CH0 Positive Input is AN6 or OpAmp3 output if CMxCON.OPMODE=1 */
#define ADC_MUXA_CH0_POS_CTMU_TEMP		62	/* ADC MuxA CH0 Positive Input is CTMU Tempreture Diode */
#define ADC_MUXA_CH0_POS_OPEN_CTMU_CAPTIME	63	/* ADC MuxA CH0 Positive Input is Open; use this selection with CTMU capacitive and time measurement */

// </editor-fold>

// Ch0NegInputMuxA
#define ADC_MUXA_CH0_NEG_VREFL		0	/* ADC MuxA CH0 Negative Input is VRefL */
#define ADC_MUXA_CH0_NEG_AN1		1	/* ADC MuxA CH0 Negative Input is AN1   */

// Ch0PosInputMuxB
// <editor-fold defaultstate="collapsed" desc="Ch0PosInputMuxB defines">
#define ADC_MUXB_CH0_POS_AN0		0	/* ADC MuxB CH0 Positive Input is AN0 */
#define ADC_MUXB_CH0_POS_AN1		1	/* ADC MuxB CH0 Positive Input is AN1 */
#define ADC_MUXB_CH0_POS_AN2		2	/* ADC MuxB CH0 Positive Input is AN2 */
#define ADC_MUXB_CH0_POS_AN3		3	/* ADC MuxB CH0 Positive Input is AN3 */
#define ADC_MUXB_CH0_POS_AN4		4	/* ADC MuxB CH0 Positive Input is AN4 */
#define ADC_MUXB_CH0_POS_AN5		5	/* ADC MuxB CH0 Positive Input is AN5 */
#define ADC_MUXB_CH0_POS_AN6		6	/* ADC MuxB CH0 Positive Input is AN6 */
#define ADC_MUXB_CH0_POS_AN7		7	/* ADC MuxB CH0 Positive Input is AN7 */
#define ADC_MUXB_CH0_POS_AN8		8	/* ADC MuxB CH0 Positive Input is AN8 */
#define ADC_MUXB_CH0_POS_AN9		9	/* ADC MuxB CH0 Positive Input is AN9 */
#define ADC_MUXB_CH0_POS_AN10		10	/* ADC MuxB CH0 Positive Input is AN10 */
#define ADC_MUXB_CH0_POS_AN11		11	/* ADC MuxB CH0 Positive Input is AN11 */
#define ADC_MUXB_CH0_POS_AN12		12	/* ADC MuxB CH0 Positive Input is AN12 */
#define ADC_MUXB_CH0_POS_AN13		13	/* ADC MuxB CH0 Positive Input is AN13 */
#define ADC_MUXB_CH0_POS_AN14		14	/* ADC MuxB CH0 Positive Input is AN14 */
#define ADC_MUXB_CH0_POS_AN15		15	/* ADC MuxB CH0 Positive Input is AN15 */
#define ADC_MUXB_CH0_POS_OA1AN3		48	/* ADC MuxB CH0 Positive Input is AN3 or OpAmp1 output if CMxCON.OPMODE=1 */
#define ADC_MUXB_CH0_POS_OA2AN0		49	/* ADC MuxB CH0 Positive Input is AN0 or OpAmp2 output if CMxCON.OPMODE=1 */
#define ADC_MUXB_CH0_POS_OA3AN6		50	/* ADC MuxB CH0 Positive Input is AN6 or OpAmp3 output if CMxCON.OPMODE=1 */
#define ADC_MUXB_CH0_POS_CTMU_TEMP	62	/* ADC MuxB CH0 Positive Input is CTMU Tempreture Diode */
#define ADC_MUXB_CH0_POS_OPEN_CTMU_CAP	63	/* ADC MuxB CH0 Positive Input is Open; use this selection with CTMU capacitive and time measurement */
// </editor-fold>

// Ch0NegInputMuxB
#define ADC_MUXB_CH0_NEG_VREFL		0	/* ADC MuxB CH0 Negative Input is VRefL */
#define ADC_MUXB_CH0_NEG_AN1		1	/* ADC MuxB CH0 Negative Input is AN1   */

// Ch123PosInputsMuxA
#define ADC_MUXA_CH123_POS_OA2AN0_AN1_AN2	0 /* ADC MuxA, OpAmp2 or AN0 => +CH1, AN1 => +CH2, AN2 => +CH2*/
#define ADC_MUXA_CH123_POS_OA1AN3_OA2AN0_OA3AN6	1 /* ADC MuxA, OpAmp1 or AN3 => +CH1, OpAmp2 or AN4 => +CH2, OpAmp3 or AN5 => +CH3*/

// Ch123NegInputsMuxA
#define ADC_MUXA_CH123_NEG_VREFL		0 /* ADC MuxA, VrefL => -CH1,-CH2,-CH3*/
#define ADC_MUXA_CH123_NEG_OA3AN6_AN7_AN8	2 /* ADC MuxA, OpAmp3 or AN6 => -CH1, AN7 => -CH2, AN8 => -CH3*/
#define ADC_MUXA_CH123_NEG_AN9_AN10_AN11	3 /* ADC MuxA, AN9 => -CH1, AN10 => -CH2, AN11 => -CH3*/

// Ch123PosInputsMuxB
#define ADC_MUXB_CH123_POS_OA2AN0_AN1_AN2	0 /* ADC MuxB, OpAmp2 or AN0 => +CH1, AN1 => +CH2, AN2 => +CH2*/
#define ADC_MUXB_CH123_POS_OA1AN3_OA2AN0_OA3AN6	1 /* ADC MuxB, OpAmp1 or AN3 => +CH1, OpAmp2 or AN4 => +CH2, OpAmp3 or AN5 => +CH3*/

// Ch123NegInputsMuxB
#define ADC_MUXB_CH123_NEG_VREFL		0 /* ADC MuxB, VrefL => -CH1,-CH2,-CH3*/
#define ADC_MUXB_CH123_NEG_OA3AN6_AN7_AN8	2 /* ADC MuxB, OpAmp3 or AN6 => -CH1, AN7 => -CH2, AN8 => -CH3*/
#define ADC_MUXB_CH123_NEG_AN9_AN10_AN11	3 /* ADC MuxB, AN9 => -CH1, AN10 => -CH2, AN11 => -CH3*/

#endif

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* SetupADC1InterruptNoDMA */

// ResultPerInt
#define	INT_WHEN_1_NEW_RESULT_IN_BUFF		1
#define	INT_WHEN_2_NEW_RESULT_IN_BUFF		2
#define	INT_WHEN_3_NEW_RESULT_IN_BUFF		3
#define	INT_WHEN_4_NEW_RESULT_IN_BUFF		4
#define	INT_WHEN_5_NEW_RESULT_IN_BUFF		5
#define	INT_WHEN_6_NEW_RESULT_IN_BUFF		6
#define	INT_WHEN_7_NEW_RESULT_IN_BUFF		7
#define	INT_WHEN_8_NEW_RESULT_IN_BUFF		8
#define	INT_WHEN_9_NEW_RESULT_IN_BUFF		9
#define	INT_WHEN_10_NEW_RESULT_IN_BUFF		10
#define	INT_WHEN_11_NEW_RESULT_IN_BUFF		11
#define	INT_WHEN_12_NEW_RESULT_IN_BUFF		12
#define	INT_WHEN_13_NEW_RESULT_IN_BUFF		13
#define	INT_WHEN_14_NEW_RESULT_IN_BUFF		14
#define	INT_WHEN_15_NEW_RESULT_IN_BUFF		15
#define	INT_WHEN_16_NEW_RESULT_IN_BUFF		16

// </editor-fold>

#endif