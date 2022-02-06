
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

#include <stdint.h>
#include "System.h"        /* System funct/params, like osc/peripheral config */
#include "LibADC.h"
#include "LibPorts.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/


/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
	Setup_OSC();

	InitPinA6(ANA_IN,NO_PULL,CN_DIS);
	InitPinA7(ANA_IN,NO_PULL,CN_DIS);
	InitPinG6(ANA_IN,NO_PULL,CN_DIS);


	SetupADC1(	ADC_10BIT_MODE,
			ADC_REF_AVDD_AVSS,
			SIMULTANEOUS_SAMPLING,
			AUTO_SAMPLING,
			SOC_SRC_AUTO_FASTEST_NoOpAMP,
			TAD_EQUAL_FASTEST,
			OUTPUT_FORMAT_U_INT,
			ADC_CONTINUE_IN_IDEL,
			RESULTS_BUFFER_16LOC_MODE,
			INT_WHEN_16_NEW_RESULT_IN_BUFF,
			DIS_DMA_TRANSFER,
			DONT_CARE, DONT_CARE, DONT_CARE);

	SetupADC1Channels(	CONVERT_CH0123,
				MUXA_ONLY,
				DIS_CH0_SCAN,
				SCN_NONE,
				ADC_MUXA_CH0_POS_AN0,
				ADC_MUXA_CH0_NEG_VREFL,
				DONT_CARE, DONT_CARE,
				ADC_MUXA_CH123_POS_AN3_AN4_AN5, ADC_MUXA_CH123_NEG_VREFL,
				DONT_CARE, DONT_CARE);

	SetupADC1Channels(	CONVERT_CH0123,
				MUXA_ONLY,
				EN_CH0_SCAN,
				SCN_AN0 & SCN_AN1 & SCN_AN2 & SCN_AN7,
				DONT_CARE, ADC_MUXA_CH0_NEG_VREFL,
				DONT_CARE, DONT_CARE,
				ADC_MUXA_CH123_POS_AN3_AN4_AN5, ADC_MUXA_CH123_NEG_VREFL,
				DONT_CARE, DONT_CARE);

	EnableIntADC1();
	DisableIntADC1();
	SetPriorityIntADC1(5);
	StartOperationADC1();
	StopOperationADC1();
	BeginSamplingADC1();
	BeginConvertingADC1();

//	InitPinB6(ANA_IN,NO_PULL,CN_DIS);
//	InitPinB7(ANA_IN,NO_PULL,CN_DIS);
//	InitPinC1(ANA_IN,NO_PULL,CN_DIS);
//
//	SetupADC2(	ADC_REF_AVDD_AVSS,
//			SIMULTANEOUS_SAMPLING,
//			AUTO_SAMPLING,
//			SOC_SRC_AUTO_FASTEST_NoOpAMP,
//			TAD_EQUAL_FASTEST,
//			OUTPUT_FORMAT_U_INT,
//			ADC_CONTINUE_IN_IDEL,
//			RESULTS_BUFFER_16LOC_MODE,
//			INT_WHEN_16_NEW_RESULT_IN_BUFF,
//			DIS_DMA_TRANSFER,
//			DONT_CARE, DONT_CARE, DONT_CARE);
//
//	SetupADC2Channels(	CONVERT_CH0123,
//				MUXA_ONLY,
//				DIS_CH0_SCAN,
//				SCN_NONE,
//				ADC_MUXA_CH0_POS_AN0,
//				ADC_MUXA_CH0_NEG_VREFL,
//				DONT_CARE, DONT_CARE,
//				ADC_MUXA_CH123_POS_AN3_AN4_AN5, ADC_MUXA_CH123_NEG_VREFL,
//				DONT_CARE, DONT_CARE);
//
//	SetupADC2Channels(	CONVERT_CH0123,
//				MUXA_ONLY,
//				EN_CH0_SCAN,
//				SCN_AN0 & SCN_AN1 & SCN_AN2 & SCN_AN7,
//				DONT_CARE, ADC_MUXA_CH0_NEG_VREFL,
//				DONT_CARE, DONT_CARE,
//				ADC_MUXA_CH123_POS_AN3_AN4_AN5, ADC_MUXA_CH123_NEG_VREFL,
//				DONT_CARE, DONT_CARE);
//
//	EnableIntADC2();
//	DisableIntADC2();
//	SetPriorityIntADC2(5);
//	StartOperationADC2();
//	StopOperationADC1();
//	BeginSamplingADC2();
//	BeginConvertingADC2();
//
//

	while(1)
	{
		Nop();
	}
    
}

void __attribute__((interrupt, no_auto_psv)) _AD1Interrupt(void){

	_AD1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _AD2Interrupt(void){

	_AD2IF = 0;
}