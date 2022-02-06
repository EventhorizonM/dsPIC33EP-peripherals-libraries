
/*
 * CRC Module Libary For All dsPIC33EP/PIC24EP Devices.
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
 * IMPLIED WARRBNTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRBNTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRBCT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: LibCRC.h, V1.00 2015/03/27 AL-Moutaz Billah Tabbakha Exp $
 *
 * Macro Functions in This Library:
 *
 * void SetupCRC(PolyLength, Polynomial, DataLength, DataShiftDirection)
 * void SetupCRCInterrupt(InterruptCondition, InterruptPriority, IntEN)
 * void CloseCRC()
 *
 * void StartCalculatingCRC()
 * void StopCalculatingCRC()
 *
 * uint16_t GetTheNumOfBytesInFIFO()
 * uint16_t GetTheNumOfWordInFIFO()
 * uint16_t GetTheNumOfDWordInFIFO()
 *
 * void WriteCRC_DataByte(data)
 * void WriteCRC_DataWord(data)
 * void WriteCRC_DataDWord(data)
 * void WriteCRC_DataDWordL(data_lo)
 * void WriteCRC_DataDWordH(data_hi)
 *
 * void SetCRC_PreviousResult(data)
 * void SetCRC_PreviousResultLow(data_lo)
 * void SetCRC_PreviousResultHigh(data_hi)
 *
 * uint16_t GetCRC_ResultWord()
 * uint32_t GetCRC_ResultDWord()
 * uint16_t GetCRC_ResultDWordL()
 * uint16_t GetCRC_ResultDWordH()
 *
 * void EnableIntCRC()
 * void DisableIntCRC()
 * void SetIntPriorityCRC(priority)
 *
 */

/* Important Note: Do Not Use The Simulator To Simulate the CRC HW Module Becoz it is Unspported By The MPLAB X, Just it's Registers are Supported */

// Note 1: For data width less than or equal to 8 bits, the user should feed the input data through byte operations into the CRCDATL register.
// Note 2: The only problem with the CRC computation is that it cannot have a foolproof mechanism for leading zeros.
// To avoid this scenario, the CRC is first initialized to a particular value and then the computed checksum is complemented.
// The result is then appended to the message stream and transmitted. In this way, errors due to leading zeros can be avoided.
// Note 3: For correct calculation of the CRC, PLEN + 1 number of zeros are to be appended to the data stream.
// Note 4: At least one instruction cycle must pass after a write to the CRCDATL register before a read of the VWORD bits is done.
// Note 5: When the CPU writes the shift registers directly though the CRCWDATL register, the CRCGO bit must be ?0?.

/* XMODEM CRC Standard is NOT Applicable On This CRC Module Becoz its Poly is 0x8408 So The (LSB=0) And The CRC Module Will
 * Change it To 0x8409 (LSB=1) Automaticaly, Corresponding to The Section 27 Manual Page 10 They Said: The 0 bit required
 * by the poly equation is always XORed; thus, X0 is a "don't care bit" and its always considered = 1. */


#ifndef _LIBCRC_
#define _LIBCRC_
/******************************************************************************/
/* CRC Module Library Defines                                                 */
/******************************************************************************/

// <editor-fold defaultstate="collapsed" desc="CRC Module Library Defines">

#ifndef _COMMON_DIRECTIVES_
#define _COMMON_DIRECTIVES_

#define DONT_CHANGE     0xFFFF
#define DONT_CARE       0
#define KEEP_DEFAULT    0
#define SET_DEFAULT     0

#endif
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* Setup_CRC */

/** PolyLegnth */ 
#define POLY_LEN_8_BIT		8
#define POLY_LEN_10_BIT		10
#define POLY_LEN_12_BIT		12
#define POLY_LEN_15_BIT		15
#define POLY_LEN_16_BIT		16
#define POLY_LEN_17_BIT		17
#define POLY_LEN_24_BIT		24
#define POLY_LEN_32_BIT		32

/** DataLength */ 
#define DATA_LEN_8_BIT		8
#define DATA_LEN_16_BIT		16
#define DATA_LEN_24_BIT		24
#define DATA_LEN_32_BIT		32

/** DataShiftDirection */ 
#define DONT_REFLECT_INPUT_DATA		0
#define SHIFT_MSB_FIRST			0
#define BIG_ENDIAN			0
#define REFLECT_INPUT_DATA		1
#define SHIFT_LSB_FIRST			1
#define LITTLE_ENDIAN			1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// Setup_CRCInterrupt
/** InterruptCondition */
#define WHEN_SHIFT_COMPLETE	0
#define WHEN_FIFO_EMPTY		1

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// CRC_HW_Calculate
/** RefOut */
#define DONT_REFLECT_RESULT	0
#define REFLECT_RESULT		1
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/** SetCRC_PreviousResult */
#define CRC_SEED_8BIT		0x00
#define CRC_SEED_10BIT		0x00
#define CRC_SEED_12BIT		0x00
#define CRC_SEED_16BIT		0x00
#define CRC_SEED_32BIT		0x46AF6449
#define CRC_SEED_CCITT		0x84CF
#define CRC_SEED_ANSI		0x00
#define CRC_SEED_MODBUS		0xEAA8
#define CRC_SEED_USB		0x00
#define CRC_SEED_ETHERNET	0x46AF6449
#define CRC_SEED_PKZIP		0x46AF6449
/***/

#define CRC_FIFO_IsEmpty()	_CRCMPT
#define CRC_FIFO_IsFull()	_CRCFUL

#define CRC_LOW_POLY		CRCXORL
#define CRC_HIGH_POLY		CRCXORH

#define CRC_LOW_INPUT		CRCDATL
#define CRC_HIGH_INPUT		CRCDATH

#define CRC_LOW_OUTPUT		CRCWDATL
#define CRC_HIGH_OUTPUT		CRCWDATH

/** Macro to Start CRC Calculation */
#define StartCalculatingCRC()	(_CRCGO = 1)
/** Macro to Stop CRC Calculation */
#define StopCalculatingCRC()	(_CRCGO = 0)


/** Macro to Get The Number of Data Bytes In CRC FIFO if The DataLength is Less Or Equal 8, The Maximum Value is 16*/
#define GetTheNumOfBytesInFIFO()	_VWORD
/** Macro to Get The Number of Data Bytes In CRC FIFO if The DataLength is Less Or Equal 16, The Maximum Value is 8*/
#define GetTheNumOfWordsInFIFO()	_VWORD
/** Macro to Get The Number of Data Bytes In CRC FIFO if The DataLength is Less Or Equal 32, The Maximum Value is 4*/
#define GetTheNumOfDWordsInFIFO()	_VWORD


/** Macro to Write 8 bit Value to The CRCDATL Register */
#define WriteCRC_DataByte(data)		(*(unsigned char*)&CRCDATL = data)
/** Macro to Write 16 bit Value to The CRCDATL Register */
#define WriteCRC_DataWord(data)		(CRCDATL = data)
/** Macro to Write 32 bit Value to The CRCDATL/CRCDATH Register */
#define WriteCRC_DataDWord(data)	({static unsigned long __data=data; CRCDATL = (unsigned int)__data; CRCDATH = (unsigned int)(__data>>16);})
/** Macro to Write The Lower Word Of 32 bit Value to The CRCDATL Register (Must Be Writen BEFOR CRCDATH)*/
#define WriteCRC_DataDWordL(data_lo)	(CRCDATL = data_lo)
/** Macro to Write The Higher Word Of 32 bit Value to The CRCDATH Register (Must Be Writen AFTER CRCDATL)*/
#define WriteCRC_DataDWordH(data_hi)	(CRCDATH = data_hi)


/** Macro to Set The Previous CRC Result (In The Begining Of New CRC Calculation)*/
#define SetCRC_PreviousResult(data)	({static unsigned long __data=data; CRCWDATL = (unsigned int)__data; CRCWDATH = (unsigned int)(__data>>16);})
/** Macro to Set The Previous Low Word CRC Result (In The Begining Of New CRC Calculation)*/
#define SetCRC_PreviousResultLow(data_lo)	(CRCWDATL = data_lo)
/** Macro to Set The Previous High Word CRC Result (In The Begining Of New CRC Calculation)*/
#define SetCRC_PreviousResultHigh(data_hi)	(CRCWDATH = data_hi)


/** Macro to Get The Final CRC (8 or 16) Bit Result (Checksum)*/
#define GetCRC_ResultWord()	(CRCWDATL)
/** Macro to Get The Final CRC 32 Bit Result (Checksum)*/
#define GetCRC_ResultDWord()	((unsigned long)(CRCWDATL+((unsigned long)CRCWDATH<<16)))
/** Macro to Get The Final CRC Lower Word Of 32 bit Result (Checksum)*/
#define GetCRC_ResultDWordL()	(CRCWDATL)
/** Macro to Get The Final CRC Higher Word Of 32 bit Result (Checksum)*/
#define GetCRC_ResultDWordH()	(CRCWDATH)
/***/

#define CRC_INT_FLAG			_CRCIF
#define EnableIntCRC()			(_CRCIE = 1)
#define DisableIntCRC()			(_CRCIE = 0)
#define SetIntPriorityCRC(priority)     (_CRCIP = priority)

/**********************************************************************************************************************/
/************************ A bunch Of Preconfigered CRC Standards For CRC Module Initialization ************************/
/**********************************************************************************************************************/

/** Macro Function to Setup The CRC Module To Operate as CRC-8-CCITT 8Bit Standard. \n
 * \Note You Have To Use Pointer Operation To Fill The FIFO or Simply Use WriteCRC_DataBYTE(data) Macro Function. */
#define SetupCRC_8Bit()		SetupCRC(POLY_LEN_8_BIT, 0x07, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA); SetCRC_PreviousResultLow(0x00)

/** Macro Function to Setup The CRC Module To Operate as CRC-10 10Bit Standard. \n
 * \Note You Have To Use Pointer Operation To Fill The FIFO or Simply Use WriteCRC_DataBYTE(data) Macro Function. */
#define SetupCRC_10Bit()	SetupCRC(POLY_LEN_10_BIT, 0x233, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA); SetCRC_PreviousResultLow(0x000)

/** Macro Function to Setup The CRC Module To Operate as CRC-12 12Bit Standard. \n
 * \Note You Have To Use Pointer Operation To Fill The FIFO or Simply Use WriteCRC_DataBYTE(data) Macro Function. */
#define SetupCRC_12Bit()	SetupCRC(POLY_LEN_12_BIT, 0x80F, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA); SetCRC_PreviousResultLow(0x000)

/** Macro Function to Setup The CRC Module To Operate as CRC-16-CCITT Standard */
#define SetupCRC_CCITT()	SetupCRC(POLY_LEN_16_BIT, 0x1021, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA); SetCRC_PreviousResultLow(0x84CF)

/** Macro Function to Setup The CRC Module To Operate as CRC-16-ANSI Standard. \n
 * \Note You Have To Reflect The Result Before Using it. */
#define SetupCRC_ANSI()		SetupCRC(POLY_LEN_16_BIT, 0x8005, DATA_LEN_8_BIT, REFLECT_INPUT_DATA); SetCRC_PreviousResultLow(0x0000)
#define SetupCRC_16Bit()	SetupCRC(POLY_LEN_16_BIT, 0x8005, DATA_LEN_8_BIT, REFLECT_INPUT_DATA); SetCRC_PreviousResultLow(0x0000)

/** Macro Function to Setup The CRC Module To Operate as CRC-16-IBM Modbus Standard. \n
 * \Note You Have To Reflect The Result Before Using it. */
#define SetupCRC_Modbus()	SetupCRC(POLY_LEN_16_BIT, 0x8005, DATA_LEN_8_BIT, REFLECT_INPUT_DATA); SetCRC_PreviousResultLow(0xEAA8)

/** Macro Function to Setup The CRC Module To Operate as CRC-16-IBM USB Standard */
#define SetupCRC_USB()		SetupCRC(POLY_LEN_16_BIT, 0xA001, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA); SetCRC_PreviousResultLow(0x0000)

/** Macro Function to Setup The CRC Module To Operate as CRC-32 Ethernet/PKZip Standard. \n
 * \Note You Have To Reflect The Result AND XOR it With 0xFFFFFFFF Before Using it. */
#define SetupCRC_Ethernet()	SetupCRC(POLY_LEN_32_BIT, 0x04C11DB7, DATA_LEN_8_BIT, REFLECT_INPUT_DATA); SetCRC_PreviousResult(0x46AF6449)
#define SetupCRC_PKZip()	SetupCRC(POLY_LEN_32_BIT, 0x04C11DB7, DATA_LEN_8_BIT, REFLECT_INPUT_DATA); SetCRC_PreviousResult(0x46AF6449)
#define SetupCRC_32Bit()	SetupCRC(POLY_LEN_32_BIT, 0x04C11DB7, DATA_LEN_8_BIT, REFLECT_INPUT_DATA); SetCRC_PreviousResult(0x46AF6449)
/***/

/* XMODEM CRC Standard is NOT Applicable On This CRC Module Becoz its Poly is 0x8408 So The (LSB=0) And The CRC Module Will
 * Change it To 0x8409 (LSB=1) Automaticaly, Corresponding to The Section 27 Manual Page 10 They Said: The 0 bit required 
 * by the poly equation is always XORed; thus, X0 is a "don't care bit" and its always considered = 1. */
/**********************************************************************************************************************/

// </editor-fold>

/******************************************************************************/
/* Library Macro Functions Prototypes                                         */
/******************************************************************************/
// <editor-fold defaultstate="collapsed" desc="CRC Module Library Macro Functions ">

/***********************************************************************************************************************
 * \Function        void SetupCRC(PolyLength, Polynomial, DataLength, DataShiftDirection)
 *
 * \Description     This Function Configures CRC Module.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>PolyLength:</u> <i>/This Parameter Sets The Length Of The Polynomial (Which Reflects The Highest Exponent in The Equation).</i>\n
 *	- POLY_LEN_8_BIT	\n
 *	- POLY_LEN_16_BIT	\n
 *	- POLY_LEN_24_BIT	\n
 *	- POLY_LEN_32_BIT	\n
 *	- Any Number Between 1 to 32 inclusively.
 * 
 * <u>Polynomial:</u>\n
 *	- The Value Of The Polynomial (Each Bit in the "Polynomial Value" controls which exponent terms are included in the equation).
 *
 * <u>DataLength:</u>\n
 *	- DATA_LEN_8_BIT	\n
 *	- DATA_LEN_16_BIT	\n
 *	- DATA_LEN_24_BIT	\n
 *	- DATA_LEN_32_BIT	\n
 *	- Any Number Between 1 to 32 inclusively.
 *
 * <u>DataShiftDirection:</u>	<i>/This setting allows better integration with various communication schemes and removes the overhead
 *				of reversing the bit order in software, Note that this only changes the direction the data is shifted into the engine.
 *				The result of the CRC calculation will still be a normal CRC result, not a reverse CRC result.</i>\n
 *	- SHIFT_MSB_FIRST (or) DONT_REFLECT_INPUT_DATA (or) BIG_ENDIAN\n
 *	- SHIFT_LSB_FIRST (or) REFLECT_INPUT_DATA (or) LITTLE_ENDIAN
 *
 * \Return      None
 *
 * \Notes       The Data Length Determin The CRC FIFO Deep:\n
 *		The FIFO is:
 *		. 16 words deep when DataLength is Less Or Equal 8  (data words, 8 bits wide or less)
 *		. 8  words deep when DataLength is Less Or Equal 16 (data words from 9 to 16 bits wide)
 *		. 4  words deep when DataLength is Less Or Equal 32 (data words from 17 to 32 bits wide)
 *
 * \Example     SetupCRC(POLY_LEN_16_BIT,0xA001,DATA_LEN_8_BIT,LITTLE_ENDIAN);	\n
 *		SetupCRC(16,0xA001,8,SHIFT_LSB_FIRST);
 *
 **********************************************************************************************************************/
#define SetupCRC(PolyLength, Polynomial, DataLength, DataShiftDirection)({	\
										\
	_CRCEN	= 1;								\
	_PLEN	= PolyLength-1;							\
	CRCXORL = (unsigned int)Polynomial;					\
	CRCXORH = (unsigned long)Polynomial>>16;				\
	_DWIDTH = DataLength-1;							\
	_LENDIAN = DataShiftDirection;						\
	_CRCGO = 1;								\
	while(_CRCMPT==0);							\
	Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();	\
	Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();	\
	_CRCGO = 0;								\
	CRCWDATL = 0;								\
	CRCWDATH = 0;								\
})

/***********************************************************************************************************************
 * \Function        void SetupCRCInterrupt(InterruptSource, InterruptPriority)
 *
 * \Description     This Function Configures CRC Module Interrupt.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptSource:</u> \n
 *	- WHEN_SHIFT_COMPLETE	\n
 *	- WHEN_FIFO_EMPTY
 *
 * <u>InterruptPriority:</u>\n
 *	- Any Value Between 1 to 7 inclusively.
 *
 * \Return      None
 *
 * \Notes       None
 *
 * \Example     SetupCRCInterrupt(WHEN_SHIFT_COMPLETE,3);
 *
 **********************************************************************************************************************/
#define SetupCRCInterrupt(InterruptCondition, InterruptPriority, IntEN)({	\
										\
	_CRCISEL = InterruptCondition;						\
	_CRCIP   = InterruptPriority;						\
	_CRCIF = 0;								\
	_CRCIE = IntEN;								\
})

/***********************************************************************************************************************
 * \Function		void CloseCRC()
 *
 * \Description		This Function Shut Of The CRC Module And Empty the FIFO And Disable All its Interrupts .
 *
 * \PreCondition	None
 *
 * \Inputs		None
 *
 * \Return		None
 *
 * \Notes		None
 *
 * \Example		CloseCRC();
 *
 **********************************************************************************************************************/
#define CloseCRC()({								\
										\
	_CRCGO = 1;								\
	while(_CRCMPT==0);							\
	Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();	\
	Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();	\
	_CRCGO = 0;								\
	CRCWDATL = 0;								\
	CRCWDATH = 0;								\
	_CRCIF = 0;								\
	_CRCIE = 0;								\
	_CRCEN = 0;								\
})
// </editor-fold>

/******************************************************************************/
/* Library Functions Prototypes                                               */
/******************************************************************************/

/***********************************************************************************************************************
 * \Function        unsigned long Direct2NondirectSeed(unsigned long Poly,unsigned long InitVal,unsigned char PolyLen)
 *
 * \Description     This Function Convert The Initial CRC Value (Seed) From Direct Form to Indirect Form That Can Be Used In This PIC24EP/dsPIC33EP CRC Module.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>Poly:</u>		<i>/The CRC Polynomial Used With This Seed.</i>\n
 *	- Any Value Ranging From 8bit to 32bit.
 *
 * <u>InitVal:</u>	<i>/The CRC Direct Initial Value (Direct Seed) That Will Be Converted To Nondirect Initial Value (Nondirect Seed).</i>\n
 *	- Any Value Ranging From 8bit to 32bit but Equal to Poly Range.
 *
 * <u>PolyLen:</u>	<i>/The CRC Polynomial Length in bits (From 1 to 32).</i>
 * 
 * \Return      Nondirect Initial Value (Nondirect Seed) of The Input.
 *
 * \Notes       None
 *
 * \Example     IndirectSeed32 = Direct2NondirectSeed(0x04C11DB7,0xFFFFFFFF,32); // The Result Will Be 0x46AF6449.
 *		IndirectSeedCCITT = Direct2NondirectSeed(0x1021,0xFFFF,16);	 // The Result Will Be 0x84CF.
 *
 **********************************************************************************************************************/
unsigned long Direct2NondirectSeed(unsigned long Poly,unsigned long InitVal,unsigned char PolyLen);


/***********************************************************************************************************************
 * \Function        unsigned long Reflect(unsigned long Data,unsigned char DataLen)
 *
 * \Description     This Function Reflect Any Value About it's Center, And it's Very Important in CRC Calculations .
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>Data:</u>		<i>/The Value you Want To Reflect.</i>\n
 *	- Any Value Ranging From 8bit to 32bit.
 *
 * <u>DataLen:</u>	<i>/The Value Length in bits (From 1 to 32).</i>
 *
 * \Return      The Reflective of The Input.
 *
 * \Notes       None
 *
 * \Example     Reflective32 = Reflect(0xABCDEF12,32);	// The Result Will Be 0x21FEDCBA.
 *		Reflective16 = Reflect(0xFF00,16);	// The Result Will Be 0x00FF.
 *
 **********************************************************************************************************************/
unsigned long Reflect(unsigned long Data,unsigned char DataLen);


/***********************************************************************************************************************
 * \Function        unsigned char CRC8_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned int Seed,unsigned char RefOut,unsigned char LastXorValue);
 *
 * \Description     This Function Calculate The New CRC Value Of The Input Buffer Using PIC24EP/dsPIC33EP CRC Module.
 *
 * \PreCondition    You Must Enable and Intialise The CRC Module First e.g: Polynomial, PolyLength(8bit), DataLength(8bit), DataShiftDirection.
 *
 * \Inputs
 * <u>*DataBuffer:</u>	<i>/A Pointer to Data Buffer That you Want to Get it's Checksum.</i>
 *
 * <u>BufferLen:</u>	<i>/How Many Bytes in The Buffer.</i>
 *
 * <u>Seed:</u>		<i>/The CRC Nondirect Initial Value (Nondirect Seed) For This Checksum.</i>
 *
 * <u>RefOut:</u>	<i>/This Option is For Reflect the CRC Result Before XORing it With Final XOR Value (Like CRC32 Standard).</i>\n
 *	- SHIFT_MSB_FIRST (or) DONT_REFLECT_INPUT_DATA	\n
 *	- SHIFT_LSB_FIRST (or) REFLECT_INPUT_DATA
 *
 * <u>LastXorValue:</u>	<i>/XOR This Value With The CRC Final Result Befor Returning it (Leave it Zero if You Wont XOR Any Thing).</i>
 *
 * \Return      0 if Any Error In Input Parameters, Else it Returns The Input Buffer CRC Checksum Value.
 *
 * \Notes       This Function is For 8bit Polynomial length Only And The Input Data Buffer Must be A BYTE Data Array Only.\n
 *		This Function is Faster Than Universal Polynomial Function "CRC_HW_Calculate" So If Your Poly is 8bit Wide Make Sure to Use This One.
 *
 * \Example     unsigned char Data[]={1,2,3,4,5,6,7,8,9};	\n
 *		unsigned char CRC_Result;			\n
 *
 *		SetupCRC_8Bit();				\n
 *		CRC_Result = CRC8_HW_Calculate(Data,9,0x00,DONT_REFLECT_RESULT,0x00);
 *
 **********************************************************************************************************************/
unsigned char CRC8_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned int Seed,unsigned char RefOut,unsigned char LastXorValue);


/***********************************************************************************************************************
 * \Function        unsigned int  CRC16_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned int Seed,unsigned char RefOut,unsigned int LastXorValue);
 *
 * \Description     This Function Calculate The New CRC Value Of The Input Buffer Using PIC24EP/dsPIC33EP CRC Module.
 *
 * \PreCondition    You Must Enable and Intialise The CRC Module First e.g: Polynomial, PolyLength(16bit), DataLength(8bit), DataShiftDirection.
 *
 * \Inputs
 * <u>*DataBuffer:</u>	<i>/A Pointer to Data Buffer That you Want to Get it's Checksum.</i>
 *
 * <u>BufferLen:</u>	<i>/How Many Bytes in The Buffer.</i>
 *
 * <u>Seed:</u>		<i>/The CRC Nondirect Initial Value (Nondirect Seed) For This Checksum.</i>
 *
 * <u>RefOut:</u>	<i>/This Option is For Reflect the CRC Result Before XORing it With Final XOR Value (Like CRC32 Standard).</i>\n
 *	- SHIFT_MSB_FIRST (or) DONT_REFLECT_INPUT_DATA	\n
 *	- SHIFT_LSB_FIRST (or) REFLECT_INPUT_DATA
 *
 * <u>LastXorValue:</u>	<i>/XOR This Value With The CRC Final Result Befor Returning it (Leave it Zero if You Wont XOR Any Thing).</i>
 *
 * \Return      0 if Any Error In Input Parameters, Else it Returns The Input Buffer CRC Checksum Value.
 *
 * \Notes       This Function is For 16bit Polynomial length Only And The Input Data Buffer Must be A BYTE Data Array Only.\n
 *		This Function is Faster Than Universal Polynomial Function "CRC_HW_Calculate" So If Your Poly is 8bit Wide Make Sure to Use This One.
 *
 * \Example     unsigned char Data[]={1,2,3,4,5,6,7,8,9};	\n
 *		unsigned int CRC_Result;			\n
 *
 *		SetupCRC_CCITT();				\n
 *		CRC_Result = CRC8_HW_Calculate(Data,9,0x00,DONT_REFLECT_RESULT,0x00);
 *
 **********************************************************************************************************************/
unsigned int  CRC16_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned int Seed,unsigned char RefOut,unsigned int LastXorValue);


/***********************************************************************************************************************
 * \Function        unsigned long CRC32_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned long Seed,unsigned char RefOut,unsigned long LastXorValue);
 *
 * \Description     This Function Calculate The New CRC Value Of The Input Buffer Using PIC24EP/dsPIC33EP CRC Module.
 *
 * \PreCondition    You Must Enable and Intialise The CRC Module First e.g: Polynomial, PolyLength(32bit), DataLength(8bit), DataShiftDirection.
 *
 * \Inputs
 * <u>*DataBuffer:</u>	<i>/A Pointer to Data Buffer That you Want to Get it's Checksum.</i>
 *
 * <u>BufferLen:</u>	<i>/How Many Bytes in The Buffer.</i>
 *
 * <u>Seed:</u>		<i>/The CRC Nondirect Initial Value (Nondirect Seed) For This Checksum.</i>
 *
 * <u>RefOut:</u>	<i>/This Option is For Reflect the CRC Result Before XORing it With Final XOR Value (Like CRC32 Standard).</i>\n
 *	- SHIFT_MSB_FIRST (or) DONT_REFLECT_INPUT_DATA	\n
 *	- SHIFT_LSB_FIRST (or) REFLECT_INPUT_DATA
 *
 * <u>LastXorValue:</u>	<i>/XOR This Value With The CRC Final Result Befor Returning it (Leave it Zero if You Wont XOR Any Thing).</i>
 *
 * \Return      0 if Any Error In Input Parameters, Else it Returns The Input Buffer CRC Checksum Value.
 *
 * \Notes       This Function is For 32bit Polynomial length Only And The Input Data Buffer Must be A BYTE Data Array Only.
 *		This Function is Faster Than Universal Polynomial Function "CRC_HW_Calculate" So If Your Poly is 8bit Wide Make Sure to Use This One.
 *
 * \Example     unsigned char Data[]={1,2,3,4,5,6,7,8,9};	\n
 *		unsigned long CRC_Result;			\n
 *
 *		SetupCRC_Ethernet();				\n
 *		CRC_Result = CRC32_HW_Calculate(Data,9,CRC_SEED_32BIT,REFLECT_RESULT,0xFFFFFFFF);
 *
 **********************************************************************************************************************/
unsigned long CRC32_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned long Seed,unsigned char RefOut,unsigned long LastXorValue);


/***********************************************************************************************************************
 * \Function        unsigned long CRC_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned long Seed,unsigned char RefOut,unsigned long LastXorValue);
 *
 * \Description     This Function Calculate The New CRC Value Of The Input Buffer Using PIC24EP/dsPIC33EP CRC HW Module.
 *
 * \PreCondition    You Must Enable and Intialise The CRC Module First e.g: Polynomial, PolyLength(1 to 32bit), DataLength(8bit), DataShiftDirection.
 *
 * \Inputs
 * <u>*DataBuffer:</u>	<i>/A Pointer to Data Buffer That you Want to Get it's Checksum.</i>
 *
 * <u>BufferLen:</u>	<i>/How Many Bytes in The Buffer.</i>
 *
 * <u>Seed:</u>		<i>/The CRC Nondirect Initial Value (Nondirect Seed) For This Checksum.</i>
 *
 * <u>RefOut:</u>	<i>/This Option is For Reflect the CRC Result Before XORing it With Final XOR Value (Like CRC32 Standard).</i>\n
 *	- SHIFT_MSB_FIRST (or) DONT_REFLECT_INPUT_DATA	\n
 *	- SHIFT_LSB_FIRST (or) REFLECT_INPUT_DATA
 *
 * <u>LastXorValue:</u>	<i>/XOR This Value With The CRC Final Result Befor Returning it (Leave it Zero if You Wont XOR Any Thing).</i>
 *
 * \Return      0 if Any Error In Input Parameters, Else it Returns The Input Buffer CRC Checksum Value.
 *
 * \Notes       This Function is For Any Polynomial length (But it's Slower Than The Dedicated PolyLength Functions Listed Before). \n
 *		The Input Data Buffer Must be A BYTE Data Array Only.	\n
 *		You Can Use This Function To Calculate The Checksum Of Any Standard And Nonstandard Polynomial Length Like 5,7,9,10,12,17,20,25,30 bit Poly.
 *
 * \Example     unsigned char Data[]={1,2,3,4,5,6,7,8,9};	\n
 *		unsigned long CRC_Result;			\n
 *
 *		SetupCRC_Ethernet();				\n
 *		CRC_Result = CRC_HW_Calculate(Data,9,CRC_SEED_32BIT,REFLECT_RESULT,0xFFFFFFFF); \n
 *
 *		SetupCRC(5, 0xB, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA);		\n
 *		CRC_Result = CRC_HW_Calculate(Data,9,0,REFLECT_RESULT,0XA);		\n
 *
 *		SetupCRC(17, 0xBBBB, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA);		\n
 *		CRC_Result = CRC_HW_Calculate(Data,9,0,REFLECT_RESULT,0XAAAA);		\n
 *
 *		SetupCRC(24, 0xBBBBBB, DATA_LEN_8_BIT, DONT_REFLECT_INPUT_DATA);	\n
 *		CRC_Result = CRC_HW_Calculate(Data,9,0,REFLECT_RESULT,0XAAAAAA);
 *
 **********************************************************************************************************************/
unsigned long CRC_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned long Seed,unsigned char RefOut,unsigned long LastXorValue);

#endif /*_LIBCRC_*/
