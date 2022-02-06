
/*
 * CRC Libary For All dsPIC33EP/PIC24EP Devices.
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
 * File: $Id: LibCRC.c, V1.00 2015/03/27 AL-Moutaz Billah Tabbakha Exp $
 *
 * Functions in This Library:
 *
 * unsigned long Direct2NondirectSeed(unsigned long Poly,unsigned long InitVal,unsigned char PolyLen)
 * unsigned long Reflect(unsigned long Data,unsigned char DataLen)
 * unsigned char CRC8_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned int Seed,unsigned char RefOut,unsigned char LastXorValue)
 * unsigned int  CRC16_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned int Seed,unsigned char RefOut,unsigned int LastXorValue)
 * unsigned long CRC32_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned long Seed,unsigned char RefOut,unsigned long LastXorValue)
 * unsigned long CRC_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned long Seed,unsigned char RefOut,unsigned long LastXorValue)
 *
 */

/* Important Note: Do Not Use The Simulator To Simulate the CRC HW Module Becoz it is Unspported By The MPLAB X, Just it's Registers are Supported */

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <xc.h>              //Device header file
#include "LibCRC.h"

/******************************************************************************/
/* Global Variables.	                                                      */
/******************************************************************************/

/******************************************************************************/
/****************************** User Functions ********************************/
/******************************************************************************/
unsigned long Direct2NondirectSeed(unsigned long Poly,unsigned long InitVal,unsigned char PolyLen){
	unsigned char i;
	unsigned long K = (unsigned long)1<<(PolyLen-1);
	for(i=0;i<PolyLen;i++){
		if(InitVal&1){
			InitVal ^= Poly;
			InitVal >>= 1;
			InitVal |= K;
		}
		else{
			InitVal >>= 1;
		}
	}
	return InitVal;
}

unsigned long Reflect(unsigned long Data,unsigned char DataLen){

	unsigned long Reflection=0;
	unsigned char Bit;

	for(Bit=0;Bit<DataLen;Bit++){	// Reflect The Data About it's Center.
		if(Data&0x01){		// If The LSB Bit is Set, Then Set The Reflection Of it, ELSE The Reflection of This Bit is Zero.
			Reflection |= (unsigned long)1<<((DataLen-1)-Bit);
		}
		Data = (Data>>1);
	}
	return Reflection;
}

unsigned char CRC8_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned int Seed,unsigned char RefOut,unsigned char LastXorValue){

	unsigned int i,Carry;
	unsigned char OddBytesFlag;

	CRCWDATL = Seed;
	_CRCIF = _CRCIE = 0;

	if(BufferLen==0)	return 0;		// if The Buffer Length is Zero So Set Error (return 0;)
	if(_PLEN!=7)		return 0;		// if The Polynomial is Not 8bit Wide So Set Error (return 0;)

	if(((BufferLen+1)%2)==1) OddBytesFlag = 1;	// if The (Buffer + Must Appended Zeros) Bytes Count is Odd.
	else			 OddBytesFlag = 0;
	
	for(i=0;i<BufferLen;i++){
		WriteCRC_DataByte(*DataBuffer++);	// write data into FIFO.
		if(CRC_FIFO_IsFull()){			// check if FIFO is full.
			StartCalculatingCRC();		// start CRC engine.
			while(!CRC_FIFO_IsEmpty());	// check if FIFO is empty.
		}
	}
	StartCalculatingCRC();
	while(_CRCMPT!=1);		// check if FIFO is empty

	if(OddBytesFlag==1){		// Odd Bytes Count So You Have to Shift&XOR The Last Byte Manulay Becoz The Module Can Shift Only 16bit at Once.
		Nop(); Nop(); Nop(); Nop();
		StopCalculatingCRC();
		for(i = 0; i < 8; i ++){ // Append The Last 8 Zeros And Manualy Shift&XOR Them.
			Carry =(CRCWDATL & 0x0080);
			CRCWDATL <<= 1;
			if(Carry)
				CRCWDATL ^= (CRCXORL+1);
		}
	}else{	// Even Bytes Count Number.

		WriteCRC_DataByte(0x00); // Append (Polynmial Length -8) Zeros to The Massage and The Remaining 8 Zeros Will Be Appended in The Last Manual Shift&XOR Opperation.
		StartCalculatingCRC();
		while(_CRCMPT!=1);	// check if FIFO is empty
		Nop(); Nop(); Nop(); Nop(); Nop();
		StopCalculatingCRC();	// stop CRC engine
		Nop();
	}

	if(RefOut){	// reverse CRC result before Final XOR
		return Reflect((CRCWDATL&0x00FF),8)^LastXorValue;
	}else{
		return ((CRCWDATL&0x00FF)^LastXorValue);
	}
}

unsigned int  CRC16_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned int Seed,unsigned char RefOut,unsigned int LastXorValue){

	unsigned int i,Carry;
	unsigned char OddBytesFlag;

	CRCWDATL = Seed;
	_CRCIF = _CRCIE = 0;

	if(BufferLen==0)	return 0;		// if The Buffer Length is Zero So Set Error (return 0;)
	if(_PLEN!=15)		return 0;		// if The Polynomial is Not 16bit Wide So Set Error (return 0;)

	if((BufferLen%2)==1)	OddBytesFlag = 1;	// if The (Buffer + Must Appended Zeros) Bytes Count is Odd.
	else			OddBytesFlag = 0;

	for(i=0;i<BufferLen;i++){
		WriteCRC_DataByte(*DataBuffer++);	// write data into FIFO.
		if(CRC_FIFO_IsFull()){			// check if FIFO is full.
			StartCalculatingCRC();		// start CRC engine.
			while(!CRC_FIFO_IsEmpty());	// check if FIFO is empty.
		}
	}
	StartCalculatingCRC();
	while(_CRCMPT!=1);		// check if FIFO is empty

	if(OddBytesFlag==1){		// Odd Bytes Count So You Have to Shift&XOR The Last Byte Manulay Becoz The Module Can Shift Only 16bit at Once.
		WriteCRC_DataByte(0x00); // Append (Polynmial Length -8) Zeros to The Massage and The Remaining 8 Zeros Will Be Appended in The Last Manual Shift&XOR Opperation.
		StartCalculatingCRC();
		while(_CRCMPT!=1);	// check if FIFO is empty
		Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
		StopCalculatingCRC();	// stop CRC engine
		Nop();
		for(i = 0; i < 8; i ++){ // Append The Last 8 Zeros And Manualy Shift&XOR Them.
			Carry =( CRCWDATL & 0x8000);
			CRCWDATL <<= 1;
			if(Carry)
				CRCWDATL ^= (CRCXORL+1);
		}
	}else{	// Even Bytes Count.

		WriteCRC_DataByte(0x00);
		WriteCRC_DataByte(0x00);
		StartCalculatingCRC();
		while(_CRCMPT!=1);	// check if FIFO is empty
		Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
		StopCalculatingCRC();	// stop CRC engine
		Nop();
	}

	if(RefOut){	// reverse CRC result before Final XOR
		return Reflect(CRCWDATL,16)^LastXorValue;
	}else{
		return (CRCWDATL^LastXorValue);
	}
}

unsigned long CRC32_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned long Seed,unsigned char RefOut,unsigned long LastXorValue){

	unsigned long Polynomial,PreResult,Carry;
	unsigned int i;
	unsigned char OddBytesFlag;

	CRCWDATL = (unsigned int)Seed;
	CRCWDATH = Seed>>16;
	_CRCIF = _CRCIE = 0;

	if(BufferLen==0)	return 0;		// if The Buffer Length is Zero So Set Error (return 0;)
	if(_PLEN!=31)		return 0;		// if The Polynomial is Not 32bit Wide So Set Error (return 0;)

	if((BufferLen%2)==1)	OddBytesFlag = 1;	// if The (Buffer + Must Appended Zeros) Bytes Count is Odd.
	else			OddBytesFlag = 0;

	for(i=0;i<BufferLen;i++){
		WriteCRC_DataByte(*DataBuffer++);	// write data into FIFO.
		if(CRC_FIFO_IsFull()){			// check if FIFO is full.
			StartCalculatingCRC();		// start CRC engine.
			while(!CRC_FIFO_IsEmpty());	// check if FIFO is empty.
		}
	}
	StartCalculatingCRC();
	while(_CRCMPT!=1);	// check if FIFO is empty

	if(OddBytesFlag==1){	// Odd Bytes Count So You Have to Shift&XOR The Last Byte Manulay Becoz The Module Can Shift Only 16bit at Once.
		WriteCRC_DataByte(0x00); // Append (Polynmial Length -8) Zeros to The Massage and The Remaining 8 Zeros Will Be Appended in The Last Manual Shift&XOR Opperation.
		WriteCRC_DataByte(0x00);
		WriteCRC_DataByte(0x00);
		StartCalculatingCRC();
		while(_CRCMPT!=1);	// check if FIFO is empty
		Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
		Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
		StopCalculatingCRC();	// stop CRC engine

		Polynomial = CRCXORH;
		Polynomial = (Polynomial<<16)+(CRCXORL+1);

		PreResult = CRCWDATH;
		PreResult = (PreResult<<16)+CRCWDATL;

		for(i = 0; i < 8; i++){	// Append The Last 8 Zeros And Manualy Shift&XOR Them.
			Carry = (PreResult & 0x80000000);
			PreResult <<= 1;
			if(Carry)
				PreResult ^= Polynomial;
		}
	}else{	// Even Bytes Count.

		WriteCRC_DataByte(0x00);
		WriteCRC_DataByte(0x00);
		WriteCRC_DataByte(0x00);
		WriteCRC_DataByte(0x00);
		StartCalculatingCRC();
		while(_CRCMPT!=1);	// check if FIFO is empty
		Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
		Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
		StopCalculatingCRC();	// stop CRC engine
		Nop();

		PreResult = CRCWDATH;
		PreResult = (PreResult<<16)+CRCWDATL;
	}

	if(RefOut){	// reverse CRC result before Final XOR
		return (Reflect(PreResult,32)^LastXorValue);
	}else{
		return (PreResult^LastXorValue);
	}
}

unsigned long CRC_HW_Calculate(unsigned char *DataBuffer,unsigned int BufferLen,unsigned long Seed,unsigned char RefOut,unsigned long LastXorValue){

	unsigned long Polynomial,PreResult=0,Carry,ResultMSB_Mask,ResultRange;
	unsigned int i,PolyLen;
	unsigned char OddBytesFlag,PolyMod8;

	CRCWDATL = (unsigned int)Seed;
	CRCWDATH = Seed>>16;
	_CRCIF = _CRCIE = 0;

	ResultMSB_Mask = (unsigned long)1<<(_PLEN);	// Most Significant Bit Mask (Position) in The Polynomial.
	PolyLen =_PLEN+1;				// Polynomial Length in Bits.
	PolyMod8 = (PolyLen)%8;				//
	if(PolyMod8==0) PolyMod8=8;			//
	ResultRange = ((ResultMSB_Mask-1)*2)+1;	// if Poly is 8bit Then ResultRange = 11111111 and if poly is 12bit Then ResultRange = 111111111111 ets.

	if(BufferLen==0)	return 0;		// if The Buffer Length is Zero So Set Error (return 0;)

	if((PolyLen>0)&&(PolyLen<=8))		PolyLen=1;	// Polynomial Length in Bytes.
	else if((PolyLen>8 )&&(PolyLen<=16))	PolyLen=2;	// Polynomial Length in Bytes.
	else if((PolyLen>16)&&(PolyLen<=24))	PolyLen=3;	// Polynomial Length in Bytes.
	else if((PolyLen>24)&&(PolyLen<=32))	PolyLen=4;	// Polynomial Length in Bytes.
	else return 0;

	if(((BufferLen+PolyLen)%2)==1)	OddBytesFlag = 1; // if The (Buffer + Must Appended Zeros) Bytes Count is Odd.
	else				OddBytesFlag = 0;

	for(i=0;i<BufferLen;i++){
		WriteCRC_DataByte(*DataBuffer++);	// write data into FIFO.
		if(CRC_FIFO_IsFull()){			// check if FIFO is full.
			StartCalculatingCRC();		// start CRC engine.
			while(!CRC_FIFO_IsEmpty());	// check if FIFO is empty.
		}
	}
	StartCalculatingCRC();
	while(_CRCMPT!=1);		// Check if FIFO is Empty

	if(OddBytesFlag==1){		// Odd Bytes Count So You Have to Shift&XOR The Last Byte Manulay Becoz The Module Can Shift Only 16bit at Once.
		if(PolyLen==1){
			Nop(); Nop(); Nop(); Nop();
			StopCalculatingCRC();
			for(i = 0; i < PolyMod8; i ++){ // Append The Last (PolyMod8) Zeros And Manualy Shift&XOR Them.
				Carry =(CRCWDATL & ResultMSB_Mask);
				CRCWDATL <<= 1;
				if(Carry)
					CRCWDATL ^= (CRCXORL+1);
			}
			PreResult = CRCWDATL;
		}
		if(PolyLen==2){
			WriteCRC_DataByte(0x00); // Append (Polynmial Length - PolyMod8) Zeros to The Massage and The Remaining (PolyMod8) Zeros Will Be Appended in The Last Manual Shift&XOR Opperation.
			StartCalculatingCRC();
			while(_CRCMPT!=1);	// check if FIFO is empty
			Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
			StopCalculatingCRC();	// stop CRC engine
			Nop();
			for(i = 0; i < PolyMod8; i ++){	// Append The Last (PolyMod8) Zeros And Manualy Shift&XOR Them.
				Carry =( CRCWDATL & ResultMSB_Mask);
				CRCWDATL <<= 1;
				if(Carry)
					CRCWDATL ^= (CRCXORL+1);
			}
			PreResult = CRCWDATL;
		}
		if(PolyLen==3){
			WriteCRC_DataByte(0x00); // Append (Polynmial Length - PolyMod8) Zeros to The Massage and The Remaining (PolyMod8) Zeros Will Be Appended in The Last Manual Shift&XOR Opperation.
			WriteCRC_DataByte(0x00);
			StartCalculatingCRC();
			while(_CRCMPT!=1);	// check if FIFO is empty
			Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
			Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
			StopCalculatingCRC();	// stop CRC engine

			Polynomial = CRCXORH;
			Polynomial = (Polynomial<<16)+(CRCXORL+1);

			PreResult = CRCWDATH;
			PreResult = (PreResult<<16)+CRCWDATL;
			PreResult = PreResult&ResultRange;

			for(i = 0; i < PolyMod8; i ++){	// Append The Last (PolyMod8) Zeros And Manualy Shift&XOR Them.
				Carry =( PreResult & ResultMSB_Mask);
				PreResult <<= 1;
				if(Carry)
					PreResult ^= Polynomial;
			}
		}
		if(PolyLen==4){
			WriteCRC_DataByte(0x00); // Append (Polynmial Length - PolyMod8) Zeros to The Massage and The Remaining (PolyMod8) Zeros Will Be Appended in The Last Manual Shift&XOR Opperation.
			WriteCRC_DataByte(0x00);
			WriteCRC_DataByte(0x00);
			StartCalculatingCRC();
			while(_CRCMPT!=1);	// check if FIFO is empty
			Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
			Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
			StopCalculatingCRC();	// stop CRC engine

			Polynomial = CRCXORH;
			Polynomial = (Polynomial<<16)+(CRCXORL+1);

			PreResult = CRCWDATH;
			PreResult = (PreResult<<16)+CRCWDATL;
			PreResult = PreResult&ResultRange;

			for(i = 0; i < PolyMod8; i ++){	// Append The Last (PolyMod8) Zeros And Manualy Shift&XOR Them.
				Carry =( PreResult & ResultMSB_Mask);
				PreResult <<= 1;
				if(Carry)
					PreResult ^= Polynomial;
			}
		}
	}else{					// Data Bytes is Even.
		if( ((_PLEN+1)%8)==0 ){		// If Polynomial Length is a Multiple of 8bit (Byte).
			for(i = 0; i < PolyLen; i ++){
				WriteCRC_DataByte(0x00);
			}

			StartCalculatingCRC();
			while(_CRCMPT!=1);	// check if FIFO is empty
			Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
			Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
			StopCalculatingCRC();	// stop CRC engine
			Nop();

			PreResult = CRCWDATH;
			PreResult = (PreResult<<16)+CRCWDATL;

		}else{	// If Polynomial Length is NOT a Multiple of 8bit e.g: 12bit or 17bit or 20bit etc.

			for(i = 0; i < (PolyLen-1); i++){ // Append (Polynmial Length - PolyMod8) Zeros to The Massage and The Remaining (PolyMod8) Zeros Will Be Appended in The Last Manual Shift&XOR Opperation.
				WriteCRC_DataByte(0x00);
			}

			StartCalculatingCRC();
			while(_CRCMPT!=1);	// check if FIFO is empty
			Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
			Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop();
			StopCalculatingCRC();	// stop CRC engine

			Polynomial = CRCXORH;
			Polynomial = (Polynomial<<16)+(CRCXORL+1);

			PreResult = CRCWDATH;
			PreResult = (PreResult<<16)+CRCWDATL;
			PreResult = PreResult&ResultRange;

			for(i = 0; i < PolyMod8; i++){
				Carry =( PreResult & ResultMSB_Mask);
				PreResult <<= 1;
				if(Carry)
					PreResult ^= Polynomial;
			}
		}
	}

	if(RefOut){	// reverse CRC result before Final XOR
		return Reflect((PreResult&ResultRange),(_PLEN+1))^LastXorValue;
	}else{
		return ((PreResult&ResultRange)^LastXorValue);
	}
}