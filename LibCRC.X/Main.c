
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


#include <libpic30.h>
#include "System.h"        /* System funct/params, like osc/peripheral config */
#include "LibCRC.h"
/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
unsigned char Data[]={1,2,3,4,5,6,7,8,9};
unsigned long CRC_Result;

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/
int main(void)
{
	Setup_OSC();

	/* Uncomment The Configration Group You Want To Try it. */
	
//	SetupCRC_8Bit();
//	CRC_Result = CRC8_HW_Calculate(Data,9,CRC_SEED_8BIT,DONT_REFLECT_RESULT,0);

	
//	SetupCRC_CCITT();
//	CRC_Result = CRC16_HW_Calculate(Data,9,CRC_SEED_CCITT,DONT_REFLECT_RESULT,0);

	
	SetupCRC_32Bit();
	CRC_Result = CRC32_HW_Calculate(Data,9,CRC_SEED_32BIT,REFLECT_RESULT,0xFFFFFFFF);

	
//	SetupCRC_8Bit();
//	CRC_Result = CRC_HW_Calculate(Data,9,CRC_SEED_8BIT,DONT_REFLECT_RESULT,0);

	
//	SetupCRC_CCITT();
//	CRC_Result = CRC_HW_Calculate(Data,9,CRC_SEED_CCITT,DONT_REFLECT_RESULT,0);

	
//	SetupCRC_32Bit();
//	CRC_Result = CRC_HW_Calculate(Data,9,CRC_SEED_32BIT,REFLECT_RESULT,0xFFFFFFFF);

	
//	SetupCRC(5, 0xB, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA);
//	CRC_Result = CRC_HW_Calculate(Data,9,0,REFLECT_RESULT,0XA);

	
//	SetupCRC(10, 0xBB, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA);
//	CRC_Result = CRC_HW_Calculate(Data,9,0,REFLECT_RESULT,0XAA);

	
//	SetupCRC(12, 0xBBB, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA);
//	CRC_Result = CRC_HW_Calculate(Data,9,0,REFLECT_RESULT,0XAAA);

	
//	SetupCRC(17, 0xBBBB, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA);
//	CRC_Result = CRC_HW_Calculate(Data,9,0,REFLECT_RESULT,0XAAAA);

	
//	SetupCRC(24, 0xBBBBBB, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA);
//	CRC_Result = CRC_HW_Calculate(Data,9,0,REFLECT_RESULT,0XAAAAAA);

	
//	SetupCRC(30, 0xBBBBBBB, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA);
//	CRC_Result = CRC_HW_Calculate(Data,9,0,REFLECT_RESULT,0XAAAAAAA);
 
	while(1)
	{
		Nop();
	}
}


void __attribute__((interrupt, no_auto_psv)) _CRCInterrupt(void){
	
	CRC_INT_FLAG = 0;
	Nop();

//	CRC_Result = GetCRC_ResultWORD();
}
