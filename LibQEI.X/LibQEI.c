
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
 * unsigned long Read32bitQEIxPositionCounter(void);
 * unsigned long Read32bitQEIxCapture(void);	
 * unsigned long Read32bitQEIxIndexCounter(void);
 * unsigned long Read32bitQEIxIntervalTimer(void);
 * unsigned int Read32bitQEIxVelocityCounter(void);
 * void Write32bitQEIxGreaterEqual(qeiCounter *ptr);
 * void Write32bitQEIxIndexCounter(qeiCounter *ptr);
 * void Write32bitQEIxInitialization(qeiCounter *ptr);
 * void Write32bitQEIxIntervalTimer(qeiCounter *ptr);
 * void Write32bitQEIxLesserEqual(qeiCounter *ptr);
 * void Write32bitQEIxPositionCounter(qeiCounter *ptr);
 * void Close32bitQEIx(void);
 *
 */

/* Important Note: Do Not Use The Simulator To Simulate the CRC HW Module Becoz it is Unspported By The MPLAB X, Just it's Registers are Supported */

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <xc.h>              //Device header file
#include "LibQEI.h"

/******************************************************************************/
/* Global Variables.	                                                      */
/******************************************************************************/

/******************************************************************************/
/****************************** User Functions ********************************/
/******************************************************************************/

#ifdef _QEI1IF

/*********************************************************************
* Function Name     : Read32bitQEI1Capture()
* Description       : This function reads capture 
                      register
* Parameters        : None
* Return Value      : 32-bit value of capture register
*********************************************************************/

unsigned long Read32bitQEI1Capture(void)
{
	uQEI_Counter Temp;

	Temp.f.LoWord = QEI1ICL;
	Temp.f.HiWord = QEI1ICH;

	return Temp.l;
}

/*********************************************************************
* Function Name     : Read32bitQEI1PositionCounter()
* Description       : This function reads position counter 
                      register
* Parameters        : None
* Return Value      : 32-bit value of position counter
*********************************************************************/

unsigned long Read32bitQEI1PositionCounter(void)
{
	uQEI_Counter Temp;

	Temp.f.LoWord = POS1CNTL;
	Temp.f.HiWord = POS1HLD;

	return Temp.l;
}

/*********************************************************************
* Function Name     : Read32bitQEI1IntervalTimer()
* Description       : This function reads interval timer 
                      register
* Parameters        : None
* Return Value      : 32-bit value of interval timer hold register
*********************************************************************/

unsigned long Read32bitQEI1IntervalTimer(void)
{
	uQEI_Counter Temp;

	Temp.f.LoWord = INT1HLDL;
	Temp.f.HiWord = INT1HLDH;

	return Temp.l;
}

/*********************************************************************
* Function Name     : Read32bitQEI1IndexCounter()
* Description       : This function reads index counter 
                      register
* Parameters        : None
* Return Value      : 32-bit value of index counter
*********************************************************************/

unsigned long Read32bitQEI1IndexCounter(void)
{
	uQEI_Counter Temp;

	Temp.f.LoWord = INDX1CNTL;
	Temp.f.HiWord = INDX1HLD;

	return Temp.l;
}

/*********************************************************************
* Function Name     : Read32bitQEI1VelocityCounter()
* Description       : This function reads velocity counter 
                      register
* Parameters        : None
* Return Value      : 16-bit value of velocity counter
*********************************************************************/

unsigned int Read32bitQEI1VelocityCounter(void)
{
	return VEL1CNT;
}

/*********************************************************************
* Function Name     : Write32bitQEI1LesserEqual()
* Description       : This function writes to lesser than or 
                      equal register
* Parameters        : Pointer to union qeiCounter, which has the
		      value to be written to lesser than or
 		      equal register
* Return Value      : None
*********************************************************************/

void Write32bitQEI1LesserEqual(uQEI_Counter *ptr)
{
	QEI1LECH  = ptr->f.HiWord;
	QEI1LECL = ptr->f.LoWord;
}

/*********************************************************************
* Function Name     : Write32bitQEI1PositionCounter()
* Description       : This function writes to position counter 
                      register
* Parameters        : Pointer to union qeiCounter, which has the
		      value to be written to position counter
* Return Value      : None
*********************************************************************/

void Write32bitQEI1PositionCounter(uQEI_Counter *ptr)
{
	POS1HLD  = ptr->f.HiWord;
	POS1CNTL = ptr->f.LoWord;
}

/*********************************************************************
* Function Name     : Write32bitQEI1GreaterEqual()
* Description       : This function writes to greater than or 
                      equal register
* Parameters        : Pointer to union qeiCounter, which has the
		      value to be written to greater than or
 		      equal register
* Return Value      : None
*********************************************************************/

void Write32bitQEI1GreaterEqual(uQEI_Counter *ptr)
{
	QEI1GECH  = ptr->f.HiWord;
	QEI1GECL = ptr->f.LoWord;
}

/*********************************************************************
* Function Name     : Write32bitQEI1Initialization()
* Description       : This function writes to initialization 
                      register
* Parameters        : Pointer to union qeiCounter, which has the
		      value to be written to initialization register
* Return Value      : None
*********************************************************************/

void Write32bitQEI1Initialization(uQEI_Counter *ptr)
{
	QEI1ICH  = ptr->f.HiWord;
	QEI1ICL  = ptr->f.LoWord;
}

/*********************************************************************
* Function Name     : Write32bitQEI1IntervalTimer()
* Description       : This function writes to Interval Timer 
                      register
* Parameters        : Pointer to union qeiCounter, which has the
		      value to be written to interval timer
* Return Value      : None
*********************************************************************/

void Write32bitQEI1IntervalTimer(uQEI_Counter *ptr)
{
	INT1TMRH  = ptr->f.HiWord;
	INT1TMRL = ptr->f.LoWord;
}

/*********************************************************************
* Function Name     : Write32bitQEI1IndexCounter()
* Description       : This function writes to index counter 
                      register
* Parameters        : Pointer to union qeiCounter, which has the
		      value to be written to index counter
* Return Value      : None
*********************************************************************/

void Write32bitQEI1IndexCounter(uQEI_Counter *ptr)
{
	INDX1HLD  = ptr->f.HiWord;
	INDX1CNTL = ptr->f.LoWord;
}

#else
#warning "Does not build on this target"
#endif

#ifdef _QEI2IF

/*********************************************************************
* Function Name     : Read32bitQEI2Capture()
* Description       : This function reads capture 
                      register
* Parameters        : None
* Return Value      : 32-bit value of capture register
*********************************************************************/

unsigned long Read32bitQEI2Capture(void)
{
	uQEI_Counter Temp;

	Temp.f.LoWord = QEI2ICL;
	Temp.f.HiWord = QEI2ICH;

	return Temp.l;
}

/*********************************************************************
* Function Name     : Read32bitQEI2PositionCounter()
* Description       : This function reads position counter 
                      register
* Parameters        : None
* Return Value      : 32-bit value of position counter
*********************************************************************/

unsigned long Read32bitQEI2PositionCounter(void)
{
	uQEI_Counter Temp;

	Temp.f.LoWord = POS2CNTL;
	Temp.f.HiWord = POS2HLD;

	return Temp.l;
}

/*********************************************************************
* Function Name     : Read32bitQEI2IntervalTimer()
* Description       : This function reads interval timer 
                      register
* Parameters        : None
* Return Value      : 32-bit value of interval timer hold register
*********************************************************************/

unsigned long Read32bitQEI2IntervalTimer(void)
{
	uQEI_Counter Temp;

	Temp.f.LoWord = INT2HLDL;
	Temp.f.HiWord = INT2HLDH;

	return Temp.l;
}

/*********************************************************************
* Function Name     : Read32bitQEI2IndexCounter()
* Description       : This function reads index counter 
                      register
* Parameters        : None
* Return Value      : 32-bit value of index counter
*********************************************************************/

unsigned long Read32bitQEI2IndexCounter(void)
{
	uQEI_Counter Temp;

	Temp.f.LoWord = INDX2CNTL;
	Temp.f.HiWord = INDX2HLD;

	return Temp.l;
}

/*********************************************************************
* Function Name     : Read32bitQEI2VelocityCounter()
* Description       : This function reads velocity counter 
                      register
* Parameters        : None
* Return Value      : 16-bit value of velocity counter
*********************************************************************/

unsigned int Read32bitQEI2VelocityCounter(void)
{
	return VEL2CNT;
}

/*********************************************************************
* Function Name     : Write32bitQEI2LesserEqual()
* Description       : This function writes to lesser than or 
                      equal register
* Parameters        : Pointer to union qeiCounter, which has the
		      value to be written to lesser than or
 		      equal register
* Return Value      : None
*********************************************************************/

void Write32bitQEI2LesserEqual(uQEI_Counter *ptr)
{
	QEI2LECH  = ptr->f.HiWord;
	QEI2LECL = ptr->f.LoWord;
}

/*********************************************************************
* Function Name     : Write32bitQEI2PositionCounter()
* Description       : This function writes to position counter 
                      register
* Parameters        : Pointer to union qeiCounter, which has the
		      value to be written to position counter
* Return Value      : None
*********************************************************************/

void Write32bitQEI2PositionCounter(uQEI_Counter *ptr)
{
	POS2HLD  = ptr->f.HiWord;
	POS2CNTL = ptr->f.LoWord;
}

/*********************************************************************
* Function Name     : Write32bitQEI2GreaterEqual()
* Description       : This function writes to greater than or 
                      equal register
* Parameters        : Pointer to union qeiCounter, which has the
		      value to be written to greater than or
 		      equal register
* Return Value      : None
*********************************************************************/

void Write32bitQEI2GreaterEqual(uQEI_Counter *ptr)
{
	QEI2GECH  = ptr->f.HiWord;
	QEI2GECL = ptr->f.LoWord;
}

/*********************************************************************
* Function Name     : Write32bitQEI2Initialization()
* Description       : This function writes to initialization 
                      register
* Parameters        : Pointer to union qeiCounter, which has the
		      value to be written to initialization register
* Return Value      : None
*********************************************************************/

void Write32bitQEI2Initialization(uQEI_Counter *ptr)
{
	QEI2ICH  = ptr->f.HiWord;
	QEI2ICL  = ptr->f.LoWord;
}

/*********************************************************************
* Function Name     : Write32bitQEI2IntervalTimer()
* Description       : This function writes to Interval Timer 
                      register
* Parameters        : Pointer to union qeiCounter, which has the
		      value to be written to interval timer
* Return Value      : None
*********************************************************************/

void Write32bitQEI2IntervalTimer(uQEI_Counter *ptr)
{
	INT2TMRH  = ptr->f.HiWord;
	INT2TMRL = ptr->f.LoWord;
}

/*********************************************************************
* Function Name     : Write32bitQEI2IndexCounter()
* Description       : This function writes to index counter 
                      register
* Parameters        : Pointer to union qeiCounter, which has the
		      value to be written to index counter
* Return Value      : None
*********************************************************************/

void Write32bitQEI2IndexCounter(uQEI_Counter *ptr)
{
	INDX2HLD  = ptr->f.HiWord;
	INDX2CNTL = ptr->f.LoWord;
}

#else
#warning "Does not build on this target"
#endif