
/*
 * Perepheral Pin Select Module (PPS) And Ports Module Libary For All dsPIC33EP/PIC24EP Devices.
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
 * File: $Id: LibPorts.h, V1.00 2015/03/7 AL-Moutaz Billah Tabbakha Exp $
 *
 * Functions in This Library:   ("X" Denotes to Port Name AND "Y" Denotes To Pin Number)
 *
 * void InitAnalogPinsPortX(AnalogPins)
 * void InitPinXY(PinMode,PullUpDn,CNI_EN)
 * BOOL ReadPinXY()
 * void SetPinXY(Value)
 * uint16 ReadPortX()
 * void SetPORTX(Value)
 * void PPS_Mapping(RemappablePinName, PeripheralIO_Name)
 * void PPS_Unlock()
 * void PPS_Lock()
 * 
 */

#ifndef LIBPORTS_H
#define	LIBPORTS_H

/* Important Notes For IO_Ports Module Use:
1-  One instruction cycle is required between a port direction change or port write operation and a read operation
    of the same port. Typically this instruction would be an NOP,

2-  Remappable peripherals never take priority over any analog functions associated with the pin.

3-  For input only, Peripheral Pin Select functionality does not have priority over TRISx settings. Therefore, when configuring RPn/RPIn pin for
    input, the corresponding bit in the TRISx register must also be configured for input (set to "1").

4-  PPS Mapping Lwas:
 a) Only one "output" function can be active on a given pin at any time regardless if it is a dedicated or remappable function (one pin, one output).
 b) It is possible to assign a "remappable output" function to multiple pins and externally short or tie them together for increased current drive.
 c) If any "dedicated output" function is enabled on a pin, it will take precedence over any remappable "output" function.
 d) If any "dedicated digital" (input or output) function is enabled on a pin, any number of "input" remappable functions can be mapped to the same pin.
 e) If any "dedicated analog" function(s) are enabled on a given pin, "digital input(s)" of any kind will all be disabled, although a single "digital
    output", at the user"s cautionary discretion, can be enabled and active as long as there is no signal contention with an external analog
    input signal. For example, it is possible for the ADC to convert the digital output logic level, or to toggle a digital output on a comparator or
    ADC input, provided there is no external analog input, such as for a built-in self test.
 f) Any number of "input" remappable functions can be mapped to the same pin(s) at the same time, including any pin with a single output from
    either a dedicated or remappable "output".
*/

// <editor-fold defaultstate="collapsed" desc="dsPIC33EP And PIC24EP Group Detecting">

// (dsPIC33EPXXX(GP/MC/MU)806/810/814 and PIC24EPXXX(GP/GU)810/814)  MCUs Group.
#if defined  (__dsPIC33EP256MU806__) || defined  (__dsPIC33EP256MU810__) || defined  (__dsPIC33EP256MU814__) ||	\
    defined  (__dsPIC33EP512GP806__) || defined  (__dsPIC33EP512MC806__) || defined  (__dsPIC33EP512MU810__) ||	\
    defined  (__dsPIC33EP512MU814__) || defined  (__PIC24EP256GU810__)   || defined  (__PIC24EP256GU814__)   ||	\
    defined  (__PIC24EP512GP806__)   || defined  (__PIC24EP512GU810__)   || defined  (__PIC24EP512GU814__)
#define GROUP1_DSPIC33E_PIC24E_FAMILY
#define GROUP1_dsPIC33EPXXX_GP_MC_MU_806_810_814_And_PIC24EPXXX_GP_GU_810_814

// (dsPIC33EPXXX(GM)3XX/6XX/7XX)  MCUs Group.
#elif   defined  (__dsPIC33EP128GM304__) || defined  (__dsPIC33EP128GM604__) || defined  (__dsPIC33EP256GM304__) ||	\
	defined  (__dsPIC33EP256GM604__) || defined  (__dsPIC33EP512GM304__) || defined  (__dsPIC33EP512GM604__) ||	\
	defined  (__dsPIC33EP128GM306__) || defined  (__dsPIC33EP128GM706__) || defined  (__dsPIC33EP256GM306__) ||	\
	defined  (__dsPIC33EP128GM310__) || defined  (__dsPIC33EP128GM710__) || defined  (__dsPIC33EP256GM310__) ||	\
	defined  (__dsPIC33EP256GM710__) || defined  (__dsPIC33EP512GM310__) || defined  (__dsPIC33EP512GM710__) ||	\
	defined  (__dsPIC33EP256GM706__) || defined  (__dsPIC33EP512GM306__) || defined  (__dsPIC33EP512GM706__)
#define GROUP2_DSPIC33E_PIC24E_FAMILY
#define GROUP2_dsPIC33EPXXX_GM_3XX_6XX_7XX

// (dsPIC33EPXXXGP50X, dsPIC33EPXXXMC20X/50X AND PIC24EPXXXGP/MC20X)  MCUs Group.
#elif   defined  (__PIC24EP32GP202__)    || defined  (__PIC24EP64GP202__)    || defined  (__PIC24EP128GP202__)   ||	\
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

#else
#error  "The target divice is not supportd in this library version"
#endif
// </editor-fold>


// <editor-fold defaultstate="collapsed" desc="Ports Defines And Functions">

#ifdef  PORTA

#define ReadPORTA()           (PORTA)
#define SetPORTA(Value)     LATA = (Value)

#ifdef ANSELA
/***********************************************************************************************************************
 * \Function            void InitAnalogPinsPortA(AnalogPins)
 *
 * \Description         Set PortA Individual Pins Mode (Analog OR Digital).
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>AnalogPins:</u>\n
 *  - RA0AN to RA15AN   <i>/ The Corresponding Pin is Analog Pin.</i>   \n
 *  - All_PINS_DIG      <i>/ All PortA Pins is Digital.</i>   \n
 *  - All_PINS_AN       <i>/ All PortA Pins is Analog.</i>
 *
 * \Return      None
 *
 * \Notes       None
 *
 * \Example     InitAnalogPinsPortA(All_PINS_DIG & RA6AN & RA7AN); \n
 * \Example     InitAnalogPinsPortG(All_PINS_AN);
 *
 **********************************************************************************************************************/
#define InitAnalogPinsPortA(AnalogPins)    ANSELA = ~(AnalogPins)
#endif
#ifdef _ANSA0
#define RA0AN	0b1111111111111110	/* RA0 bin is Analog bin */
#define _InitPinA0AN(PinMode)    _ANSA0 = (PinMode>>2);
#else
#define _InitPinA0AN(PinMode)
#endif
#ifdef _ANSA1
#define RA1AN	0b1111111111111101	/* RA1 bin is Analog bin */
#define _InitPinA1AN(PinMode)    _ANSA1 = (PinMode>>2);
#else
#define _InitPinA1AN(PinMode)
#endif
#ifdef _ANSA2
#define RA2AN	0b1111111111111011	/* RA2 bin is Analog bin */
#define _InitPinA2AN(PinMode)    _ANSA2 = (PinMode>>2);
#else
#define _InitPinA2AN(PinMode)
#endif
#ifdef _ANSA3
#define RA3AN	0b1111111111110111	/* RA3 bin is Analog bin */
#define _InitPinA3AN(PinMode)    _ANSA3 = (PinMode>>2);
#else
#define _InitPinA3AN(PinMode)
#endif
#ifdef _ANSA4
#define RA4AN	0b1111111111101111	/* RA4 bin is Analog bin */
#define _InitPinA4AN(PinMode)    _ANSA4 = (PinMode>>2);
#else
#define _InitPinA4AN(PinMode)
#endif
#ifdef _ANSA5
#define RA5AN	0b1111111111011111	/* RA5 bin is Analog bin */
#define _InitPinA5AN(PinMode)    _ANSA5 = (PinMode>>2);
#else
#define _InitPinA5AN(PinMode)
#endif
#ifdef _ANSA6
#define RA6AN	0b1111111110111111	/* RA6 bin is Analog bin */
#define _InitPinA6AN(PinMode)    _ANSA6 = (PinMode>>2);
#else
#define _InitPinA6AN(PinMode)
#endif
#ifdef _ANSA7
#define RA7AN	0b1111111101111111	/* RA7 bin is Analog bin */
#define _InitPinA7AN(PinMode)    _ANSA7 = (PinMode>>2);
#else
#define _InitPinA7AN(PinMode)
#endif
#ifdef _ANSA8
#define RA8AN	0b1111111011111111	/* RA8 bin is Analog bin */
#define ____InitPinA8AN(PinMode)    _ANSA8 = (PinMode>>2);
#else
#define ____InitPinA8AN(PinMode)
#endif
#ifdef _ANSA9
#define RA9AN	0b1111110111111111	/* RA9 bin is Analog bin */
#define _InitPinA9AN(PinMode)    _ANSA9 = (PinMode>>2);
#else
#define _InitPinA9AN(PinMode)
#endif
#ifdef _ANSA10
#define RA10AN	0b1111101111111111	/* RA10 bin is Analog bin */
#define _InitPinA10AN(PinMode)    _ANSA10 = (PinMode>>2);
#else
#define _InitPinA10AN(PinMode)
#endif
#ifdef _ANSA11
#define RA11AN	0b1111011111111111	/* RA11 bin is Analog bin */
#define _InitPinA11AN(PinMode)    _ANSA11 = (PinMode>>2);
#else
#define _InitPinA11AN(PinMode)
#endif
#ifdef _ANSA12
#define RA12AN	0b1110111111111111	/* RA12 bin is Analog bin */
#define _InitPinA12AN(PinMode)    _ANSA12 = (PinMode>>2);
#else
#define _InitPinA12AN(PinMode)
#endif
#ifdef _ANSA13
#define RA13AN	0b1101111111111111	/* RA13 bin is Analog bin */
#define _InitPinA13AN(PinMode)    _ANSA13 = (PinMode>>2);
#else
#define _InitPinA13AN(PinMode)
#endif
#ifdef _ANSA14
#define RA14AN	0b1011111111111111	/* RA14 bin is Analog bin */
#define _InitPinA14AN(PinMode)    _ANSA14 = (PinMode>>2);
#else
#define _InitPinA14AN(PinMode)
#endif
#ifdef _ANSA15
#define RA15AN	0b0111111111111111	/* RA15 bin is Analog bin */
#define _InitPinA15AN(PinMode)    _ANSA15 = (PinMode>>2);
#else
#define _InitPinA15AN(PinMode)
#endif

#ifdef _ODCA0
#define _InitPinA0OD(PinMode)    _ODCA0 = (PinMode&0b001);
#else
#define _InitPinA0OD(PinMode)
#endif
#ifdef _ODCA1
#define _InitPinA1OD(PinMode)    _ODCA1 = (PinMode&0b001);
#else
#define _InitPinA1OD(PinMode)
#endif
#ifdef _ODCA2
#define _InitPinA2OD(PinMode)    _ODCA2 = (PinMode&0b001);
#else
#define _InitPinA2OD(PinMode)
#endif
#ifdef _ODCA3
#define _InitPinA3OD(PinMode)    _ODCA3 = (PinMode&0b001);
#else
#define _InitPinA3OD(PinMode)
#endif
#ifdef _ODCA4
#define _InitPinA4OD(PinMode)    _ODCA4 = (PinMode&0b001);
#else
#define _InitPinA4OD(PinMode)
#endif
#ifdef _ODCA5
#define _InitPinA5OD(PinMode)    _ODCA5 = (PinMode&0b001);
#else
#define _InitPinA5OD(PinMode)
#endif
#ifdef _ODCA6
#define _InitPinA6OD(PinMode)    _ODCA6 = (PinMode&0b001);
#else
#define _InitPinA6OD(PinMode)
#endif
#ifdef _ODCA7
#define _InitPinA7OD(PinMode)    _ODCA7 = (PinMode&0b001);
#else
#define _InitPinA7OD(PinMode)
#endif
#ifdef _ODCA8
#define _InitPinA8OD(PinMode)    _ODCA8 = (PinMode&0b001);
#else
#define _InitPinA8OD(PinMode)
#endif
#ifdef _ODCA9
#define _InitPinA9OD(PinMode)    _ODCA9 = (PinMode&0b001);
#else
#define _InitPinA9OD(PinMode)
#endif
#ifdef _ODCA10
#define _InitPinA10OD(PinMode)    _ODCA10 = (PinMode&0b001);
#else
#define _InitPinA10OD(PinMode)
#endif
#ifdef _ODCA11
#define _InitPinA11OD(PinMode)    _ODCA11 = (PinMode&0b001);
#else
#define _InitPinA11OD(PinMode)
#endif
#ifdef _ODCA12
#define _InitPinA12OD(PinMode)    _ODCA12 = (PinMode&0b001);
#else
#define _InitPinA12OD(PinMode)
#endif
#ifdef _ODCA13
#define _InitPinA13OD(PinMode)    _ODCA13 = (PinMode&0b001);
#else
#define _InitPinA13OD(PinMode)
#endif
#ifdef _ODCA14
#define _InitPinA14OD(PinMode)    _ODCA14 = (PinMode&0b001);
#else
#define _InitPinA14OD(PinMode)
#endif
#ifdef _ODCA15
#define _InitPinA15OD(PinMode)    _ODCA15 = (PinMode&0b001);
#else
#define _InitPinA15OD(PinMode)
#endif

#ifdef _CNIEA0
#define _InitPinA0PUD(PullUpDn)    _CNPUA0 = (PullUpDn&0b01); _CNPDA0 = (PullUpDn>>1);
#define _InitPinA0CN(CNI_EN)          _CNIEA0 = (CNI_EN);
#else
#define _InitPinA0PUD(PullUpDn)
#define _InitPinA0CN(CNI_EN)
#endif
#ifdef _CNIEA1
#define _InitPinA1PUD(PullUpDn)    _CNPUA1 = (PullUpDn&0b01); _CNPDA1 = (PullUpDn>>1);
#define _InitPinA1CN(CNI_EN)          _CNIEA1 = (CNI_EN);
#else
#define _InitPinA1PUD(PullUpDn)
#define _InitPinA1CN(CNI_EN)
#endif
#ifdef _CNIEA2
#define _InitPinA2PUD(PullUpDn)    _CNPUA2 = (PullUpDn&0b01); _CNPDA2 = (PullUpDn>>1);
#define _InitPinA2CN(CNI_EN)          _CNIEA2 = (CNI_EN);
#else
#define _InitPinA2PUD(PullUpDn)
#define _InitPinA2CN(CNI_EN)
#endif
#ifdef _CNIEA3
#define _InitPinA3PUD(PullUpDn)    _CNPUA3 = (PullUpDn&0b01); _CNPDA3 = (PullUpDn>>1);
#define _InitPinA3CN(CNI_EN)          _CNIEA3 = (CNI_EN);
#else
#define _InitPinA3PUD(PullUpDn)
#define _InitPinA3CN(CNI_EN)
#endif
#ifdef _CNIEA4
#define _InitPinA4PUD(PullUpDn)    _CNPUA4 = (PullUpDn&0b01); _CNPDA4 = (PullUpDn>>1);
#define _InitPinA4CN(CNI_EN)          _CNIEA4 = (CNI_EN);
#else
#define _InitPinA4PUD(PullUpDn)
#define _InitPinA4CN(CNI_EN)
#endif
#ifdef _CNIEA5
#define _InitPinA5PUD(PullUpDn)    _CNPUA5 = (PullUpDn&0b01); _CNPDA5 = (PullUpDn>>1);
#define _InitPinA5CN(CNI_EN)          _CNIEA5 = (CNI_EN);
#else
#define _InitPinA5PUD(PullUpDn)
#define _InitPinA5CN(CNI_EN)
#endif
#ifdef _CNIEA6
#define _InitPinA6PUD(PullUpDn)    _CNPUA6 = (PullUpDn&0b01); _CNPDA6 = (PullUpDn>>1);
#define _InitPinA6CN(CNI_EN)          _CNIEA6 = (CNI_EN);
#else
#define _InitPinA6PUD(PullUpDn)
#define _InitPinA6CN(CNI_EN)
#endif
#ifdef _CNIEA7
#define _InitPinA7PUD(PullUpDn)    _CNPUA7 = (PullUpDn&0b01); _CNPDA7 = (PullUpDn>>1);
#define _InitPinA7CN(CNI_EN)          _CNIEA7 = (CNI_EN);
#else
#define _InitPinA7PUD(PullUpDn)
#define _InitPinA7CN(CNI_EN)
#endif
#ifdef _CNIEA8
#define _InitPinA8PUD(PullUpDn)    _CNPUA8 = (PullUpDn&0b01); _CNPDA8 = (PullUpDn>>1);
#define _InitPinA8CN(CNI_EN)          _CNIEA8 = (CNI_EN);
#else
#define _InitPinA8PUD(PullUpDn)
#define _InitPinA8CN(CNI_EN)
#endif
#ifdef _CNIEA9
#define _InitPinA9PUD(PullUpDn)    _CNPUA9 = (PullUpDn&0b01); _CNPDA9 = (PullUpDn>>1);
#define _InitPinA9CN(CNI_EN)          _CNIEA9 = (CNI_EN);
#else
#define _InitPinA9PUD(PullUpDn)
#define _InitPinA9CN(CNI_EN)
#endif
#ifdef _CNIEA10
#define _InitPinA10PUD(PullUpDn)    _CNPUA10 = (PullUpDn&0b01); _CNPDA10 = (PullUpDn>>1);
#define _InitPinA10CN(CNI_EN)          _CNIEA10 = (CNI_EN);
#else
#define _InitPinA10PUD(PullUpDn)
#define _InitPinA10CN(CNI_EN)
#endif
#ifdef _CNIEA11
#define _InitPinA11PUD(PullUpDn)    _CNPUA11 = (PullUpDn&0b01); _CNPDA11 = (PullUpDn>>1);
#define _InitPinA11CN(CNI_EN)          _CNIEA11 = (CNI_EN);
#else
#define _InitPinA11PUD(PullUpDn)
#define _InitPinA11CN(CNI_EN)
#endif
#ifdef _CNIEA12
#define _InitPinA12PUD(PullUpDn)    _CNPUA12 = (PullUpDn&0b01); _CNPDA12 = (PullUpDn>>1);
#define _InitPinA12CN(CNI_EN)          _CNIEA12 = (CNI_EN);
#else
#define _InitPinA12PUD(PullUpDn)
#define _InitPinA12CN(CNI_EN)
#endif
#ifdef _CNIEA13
#define _InitPinA13PUD(PullUpDn)    _CNPUA13 = (PullUpDn&0b01); _CNPDA13 = (PullUpDn>>1);
#define _InitPinA13CN(CNI_EN)          _CNIEA13 = (CNI_EN);
#else
#define _InitPinA13PUD(PullUpDn)
#define _InitPinA13CN(CNI_EN)
#endif
#ifdef _CNIEA14
#define _InitPinA14PUD(PullUpDn)    _CNPUA14 = (PullUpDn&0b01); _CNPDA14 = (PullUpDn>>1);
#define _InitPinA14CN(CNI_EN)          _CNIEA14 = (CNI_EN);
#else
#define _InitPinA14PUD(PullUpDn)
#define _InitPinA14CN(CNI_EN)
#endif
#ifdef _CNIEA15
#define _InitPinA15PUD(PullUpDn)    _CNPUA15 = (PullUpDn&0b01); _CNPDA15 = (PullUpDn>>1);
#define _InitPinA15CN(CNI_EN)          _CNIEA15 = (CNI_EN);
#else
#define _InitPinA15PUD(PullUpDn)
#define _InitPinA15CN(CNI_EN)
#endif

#ifdef _RA0
/***********************************************************************************************************************
 * \Function            void InitPinA0(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA0 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA0(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA0(PinMode,PullUpDn,CNI_EN)  ({_TRISA0 = ((PinMode>>1)&0b01); _InitPinA0AN(PinMode) _InitPinA0OD(PinMode) _InitPinA0PUD(PullUpDn) _InitPinA0CN(CNI_EN)})
#define ReadPinA0()           (PORTAbits.RA0)
#define SetPinA0(Value)     LATAbits.LATA0 = (Value)
#endif
#ifdef _RA1
/***********************************************************************************************************************
 * \Function            void InitPinA1(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA1 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA1(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA1(PinMode,PullUpDn,CNI_EN)  ({_TRISA1 = ((PinMode>>1)&0b01); _InitPinA1AN(PinMode)  _InitPinA1OD(PinMode) _InitPinA1PUD(PullUpDn) _InitPinA1CN(CNI_EN)})
#define ReadPinA1()           (PORTAbits.RA1)
#define SetPinA1(Value)     LATAbits.LATA1 = (Value)
#endif
#ifdef _RA2
/***********************************************************************************************************************
 * \Function            void InitPinA2(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA2 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA2(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA2(PinMode,PullUpDn,CNI_EN)  ({_TRISA2 = ((PinMode>>1)&0b01); _InitPinA2AN(PinMode)  _InitPinA2OD(PinMode) _InitPinA2PUD(PullUpDn) _InitPinA2CN(CNI_EN)})
#define ReadPinA2()           (PORTAbits.RA2)
#define SetPinA2(Value)     LATAbits.LATA2 = (Value)
#endif
#ifdef _RA3
/***********************************************************************************************************************
 * \Function            void InitPinA3(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA3 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA3(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA3(PinMode,PullUpDn,CNI_EN)  ({_TRISA3 = ((PinMode>>1)&0b01); _InitPinA3AN(PinMode)  _InitPinA3OD(PinMode) _InitPinA3PUD(PullUpDn) _InitPinA3CN(CNI_EN)})
#define ReadPinA3()           (PORTAbits.RA3)
#define SetPinA3(Value)     LATAbits.LATA3 = (Value)
#endif
#ifdef _RA4
/***********************************************************************************************************************
 * \Function            void InitPinA4(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA4 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA4(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA4(PinMode,PullUpDn,CNI_EN)  ({_TRISA4 = ((PinMode>>1)&0b01); _InitPinA4AN(PinMode)  _InitPinA4OD(PinMode) _InitPinA4PUD(PullUpDn) _InitPinA4CN(CNI_EN)})
#define ReadPinA4()           (PORTAbits.RA4)
#define SetPinA4(Value)     LATAbits.LATA4 = (Value)
#endif
#ifdef _RA5
/***********************************************************************************************************************
 * \Function            void InitPinA5(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA5 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA5(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA5(PinMode,PullUpDn,CNI_EN)  ({_TRISA5 = ((PinMode>>1)&0b01); _InitPinA5AN(PinMode)  _InitPinA5OD(PinMode) _InitPinA5PUD(PullUpDn) _InitPinA5CN(CNI_EN)})
#define ReadPinA5()           (PORTAbits.RA5)
#define SetPinA5(Value)     LATAbits.LATA5 = (Value)
#endif
#ifdef _RA6
/***********************************************************************************************************************
 * \Function            void InitPinA6(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA6 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA6(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA6(PinMode,PullUpDn,CNI_EN)  ({_TRISA6 = ((PinMode>>1)&0b01); _InitPinA6AN(PinMode)  _InitPinA6OD(PinMode) _InitPinA6PUD(PullUpDn) _InitPinA6CN(CNI_EN)})
#define ReadPinA6()           (PORTAbits.RA6)
#define SetPinA6(Value)     LATAbits.LATA6 = (Value)
#endif
#ifdef _RA7
/***********************************************************************************************************************
 * \Function            void InitPinA7(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA7 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA7(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA7(PinMode,PullUpDn,CNI_EN)  ({_TRISA7 = ((PinMode>>1)&0b01); _InitPinA7AN(PinMode)  _InitPinA7OD(PinMode) _InitPinA7PUD(PullUpDn) _InitPinA7CN(CNI_EN)})
#define ReadPinA7()           (PORTAbits.RA7)
#define SetPinA7(Value)     LATAbits.LATA7 = (Value)
#endif
#ifdef _RA8
/***********************************************************************************************************************
 * \Function            void InitPinA8(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA8 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA8(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA8(PinMode,PullUpDn,CNI_EN)  ({_TRISA8 = ((PinMode>>1)&0b01); ____InitPinA8AN(PinMode)  _InitPinA8OD(PinMode) _InitPinA8PUD(PullUpDn) _InitPinA8CN(CNI_EN)})
#define ReadPinA8()           (PORTAbits.RA8)
#define SetPinA8(Value)     LATAbits.LATA8 = (Value)
#endif
#ifdef _RA9
/***********************************************************************************************************************
 * \Function            void InitPinA9(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA9 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA9(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA9(PinMode,PullUpDn,CNI_EN)  ({_TRISA9 = ((PinMode>>1)&0b01); _InitPinA9AN(PinMode)  _InitPinA9OD(PinMode) _InitPinA9PUD(PullUpDn) _InitPinA9CN(CNI_EN)})
#define ReadPinA9()           (PORTAbits.RA9)
#define SetPinA9(Value)     LATAbits.LATA9 = (Value)
#endif
#ifdef _RA10
/***********************************************************************************************************************
 * \Function            void InitPinA10(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA10 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA10(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA10(PinMode,PullUpDn,CNI_EN)  ({_TRISA10 = ((PinMode>>1)&0b01); _InitPinA10AN(PinMode)  _InitPinA10OD(PinMode) _InitPinA10PUD(PullUpDn) _InitPinA10CN(CNI_EN)})
#define ReadPinA10()          (PORTAbits.RA10)
#define SetPinA10(Value)     LATAbits.LATA10 = (Value)
#endif
#ifdef _RA11
/***********************************************************************************************************************
 * \Function            void InitPinA11(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA11 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA11(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA11(PinMode,PullUpDn,CNI_EN)  ({_TRISA11 = ((PinMode>>1)&0b01); _InitPinA11AN(PinMode)  _InitPinA11OD(PinMode) _InitPinA11PUD(PullUpDn) _InitPinA11CN(CNI_EN)})
#define ReadPinA11()          (PORTAbits.RA11)
#define SetPinA11(Value)     LATAbits.LATA11 = (Value)
#endif
#ifdef _RA12
/***********************************************************************************************************************
 * \Function            void InitPinA12(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA12 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA12(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA12(PinMode,PullUpDn,CNI_EN)  ({_TRISA12 = ((PinMode>>1)&0b01); _InitPinA12AN(PinMode)  _InitPinA12OD(PinMode) _InitPinA12PUD(PullUpDn) _InitPinA12CN(CNI_EN)})
#define ReadPinA12()          (PORTAbits.RA12)
#define SetPinA12(Value)     LATAbits.LATA12 = (Value)
#endif
#ifdef _RA13
/***********************************************************************************************************************
 * \Function            void InitPinA13(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA13 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA13(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA13(PinMode,PullUpDn,CNI_EN)  ({_TRISA13 = ((PinMode>>1)&0b01); _InitPinA13AN(PinMode)  _InitPinA13OD(PinMode) _InitPinA13PUD(PullUpDn) _InitPinA13CN(CNI_EN)})
#define ReadPinA13()          (PORTAbits.RA13)
#define SetPinA13(Value)     LATAbits.LATA13 = (Value)
#endif
#ifdef _RA14
/***********************************************************************************************************************
 * \Function            void InitPinA14(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA14 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA14(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA14(PinMode,PullUpDn,CNI_EN)  ({_TRISA14 = ((PinMode>>1)&0b01); _InitPinA14AN(PinMode)  _InitPinA14OD(PinMode) _InitPinA14PUD(PullUpDn) _InitPinA14CN(CNI_EN)})
#define ReadPinA14()          (PORTAbits.RA14)
#define SetPinA14(Value)     LATAbits.LATA14 = (Value)
#endif
#ifdef _RA15
/***********************************************************************************************************************
 * \Function            void InitPinA15(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTA.RA15 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinA15(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinA15(PinMode,PullUpDn,CNI_EN)  ({_TRISA15 = ((PinMode>>1)&0b01); _InitPinA15AN(PinMode)  _InitPinA15OD(PinMode) _InitPinA15PUD(PullUpDn) _InitPinA15CN(CNI_EN)})
#define ReadPinA15()          (PORTAbits.RA15)
#define SetPinA15(Value)     LATAbits.LATA15 = (Value)
#endif

#endif
#ifdef  PORTB

#define ReadPORTB()           (PORTB)
#define SetPORTB(Value)     LATB = (Value)

#ifdef ANSELB
/***********************************************************************************************************************
 * \Function            void InitAnalogPinsPortB(AnalogPins)
 *
 * \Description         Set PortB Individual Pins Mode (Analog OR Digital).
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>AnalogPins:</u>\n
 *  - RB0AN to RB15AN   <i>/ The Corresponding Pin is Analog Pin.</i>   \n
 *  - All_PINS_DIG      <i>/ All PortB Pins is Digital.</i>   \n
 *  - All_PINS_AN       <i>/ All PortB Pins is Analog.</i>
 *
 * \Return      None
 *
 * \Notes       None
 *
 * \Example     InitAnalogPinsPortB(All_PINS_DIG & RB6AN & RB7AN); \n
 * \Example     InitAnalogPinsPortG(All_PINS_AN);
 *
 **********************************************************************************************************************/
#define InitAnalogPinsPortB(AnalogPins)    ANSELB = ~(AnalogPins)
#endif
#ifdef _ANSB0
#define RB0AN	0b1111111111111110	/* RB0 bin is Analog bin */
#define _InitPinB0AN(PinMode)    _ANSB0 = (PinMode>>2);
#else
#define _InitPinB0AN(PinMode)
#endif
#ifdef _ANSB1
#define RB1AN	0b1111111111111101	/* RB1 bin is Analog bin */
#define _InitPinB1AN(PinMode)    _ANSB1 = (PinMode>>2);
#else
#define _InitPinB1AN(PinMode)
#endif
#ifdef _ANSB2
#define RB2AN	0b1111111111111011	/* RB2 bin is Analog bin */
#define _InitPinB2AN(PinMode)    _ANSB2 = (PinMode>>2);
#else
#define _InitPinB2AN(PinMode)
#endif
#ifdef _ANSB3
#define RB3AN	0b1111111111110111	/* RB3 bin is Analog bin */
#define _InitPinB3AN(PinMode)    _ANSB3 = (PinMode>>2);
#else
#define _InitPinB3AN(PinMode)
#endif
#ifdef _ANSB4
#define RB4AN	0b1111111111101111	/* RB4 bin is Analog bin */
#define _InitPinB4AN(PinMode)    _ANSB4 = (PinMode>>2);
#else
#define _InitPinB4AN(PinMode)
#endif
#ifdef _ANSB5
#define RB5AN	0b1111111111011111	/* RB5 bin is Analog bin */
#define _InitPinB5AN(PinMode)    _ANSB5 = (PinMode>>2);
#else
#define _InitPinB5AN(PinMode)
#endif
#ifdef _ANSB6
#define RB6AN	0b1111111110111111	/* RB6 bin is Analog bin */
#define _InitPinB6AN(PinMode)    _ANSB6 = (PinMode>>2);
#else
#define _InitPinB6AN(PinMode)
#endif
#ifdef _ANSB7
#define RB7AN	0b1111111101111111	/* RB7 bin is Analog bin */
#define _InitPinB7AN(PinMode)    _ANSB7 = (PinMode>>2);
#else
#define _InitPinB7AN(PinMode)
#endif
#ifdef _ANSB8
#define RB8AN	0b1111111011111111	/* RB8 bin is Analog bin */
#define _InitPinB8AN(PinMode)    _ANSB8 = (PinMode>>2);
#else
#define _InitPinB8AN(PinMode)
#endif
#ifdef _ANSB9
#define RB9AN	0b1111110111111111	/* RB9 bin is Analog bin */
#define _InitPinB9AN(PinMode)    _ANSB9 = (PinMode>>2);
#else
#define _InitPinB9AN(PinMode)
#endif
#ifdef _ANSB10
#define RB10AN	0b1111101111111111	/* RB10 bin is Analog bin */
#define _InitPinB10AN(PinMode)    _ANSB10 = (PinMode>>2);
#else
#define _InitPinB10AN(PinMode)
#endif
#ifdef _ANSB11
#define RB11AN	0b1111011111111111	/* RB11 bin is Analog bin */
#define _InitPinB11AN(PinMode)    _ANSB11 = (PinMode>>2);
#else
#define _InitPinB11AN(PinMode)
#endif
#ifdef _ANSB12
#define RB12AN	0b1110111111111111	/* RB12 bin is Analog bin */
#define _InitPinB12AN(PinMode)    _ANSB12 = (PinMode>>2);
#else
#define _InitPinB12AN(PinMode)
#endif
#ifdef _ANSB13
#define RB13AN	0b1101111111111111	/* RB13 bin is Analog bin */
#define _InitPinB13AN(PinMode)    _ANSB13 = (PinMode>>2);
#else
#define _InitPinB13AN(PinMode)
#endif
#ifdef _ANSB14
#define RB14AN	0b1011111111111111	/* RB14 bin is Analog bin */
#define _InitPinB14AN(PinMode)    _ANSB14 = (PinMode>>2);
#else
#define _InitPinB14AN(PinMode)
#endif
#ifdef _ANSB15
#define RB15AN	0b0111111111111111	/* RB15 bin is Analog bin */
#define _InitPinB15AN(PinMode)    _ANSB15 = (PinMode>>2);
#else
#define _InitPinB15AN(PinMode)
#endif

#ifdef _ODCB0
#define _InitPinB0OD(PinMode)    _ODCB0 = (PinMode&0b001);
#else
#define _InitPinB0OD(PinMode)
#endif
#ifdef _ODCB1
#define _InitPinB1OD(PinMode)    _ODCB1 = (PinMode&0b001);
#else
#define _InitPinB1OD(PinMode)
#endif
#ifdef _ODCB2
#define _InitPinB2OD(PinMode)    _ODCB2 = (PinMode&0b001);
#else
#define _InitPinB2OD(PinMode)
#endif
#ifdef _ODCB3
#define _InitPinB3OD(PinMode)    _ODCB3 = (PinMode&0b001);
#else
#define _InitPinB3OD(PinMode)
#endif
#ifdef _ODCB4
#define _InitPinB4OD(PinMode)    _ODCB4 = (PinMode&0b001);
#else
#define _InitPinB4OD(PinMode)
#endif
#ifdef _ODCB5
#define _InitPinB5OD(PinMode)    _ODCB5 = (PinMode&0b001);
#else
#define _InitPinB5OD(PinMode)
#endif
#ifdef _ODCB6
#define _InitPinB6OD(PinMode)    _ODCB6 = (PinMode&0b001);
#else
#define _InitPinB6OD(PinMode)
#endif
#ifdef _ODCB7
#define _InitPinB7OD(PinMode)    _ODCB7 = (PinMode&0b001);
#else
#define _InitPinB7OD(PinMode)
#endif
#ifdef _ODCB8
#define _InitPinB8OD(PinMode)    _ODCB8 = (PinMode&0b001);
#else
#define _InitPinB8OD(PinMode)
#endif
#ifdef _ODCB9
#define _InitPinB9OD(PinMode)    _ODCB9 = (PinMode&0b001);
#else
#define _InitPinB9OD(PinMode)
#endif
#ifdef _ODCB10
#define _InitPinB10OD(PinMode)    _ODCB10 = (PinMode&0b001);
#else
#define _InitPinB10OD(PinMode)
#endif
#ifdef _ODCB11
#define _InitPinB11OD(PinMode)    _ODCB11 = (PinMode&0b001);
#else
#define _InitPinB11OD(PinMode)
#endif
#ifdef _ODCB12
#define _InitPinB12OD(PinMode)    _ODCB12 = (PinMode&0b001);
#else
#define _InitPinB12OD(PinMode)
#endif
#ifdef _ODCB13
#define _InitPinB13OD(PinMode)    _ODCB13 = (PinMode&0b001);
#else
#define _InitPinB13OD(PinMode)
#endif
#ifdef _ODCB14
#define _InitPinB14OD(PinMode)    _ODCB14 = (PinMode&0b001);
#else
#define _InitPinB14OD(PinMode)
#endif
#ifdef _ODCB15
#define _InitPinB15OD(PinMode)    _ODCB15 = (PinMode&0b001);
#else
#define _InitPinB15OD(PinMode)
#endif

#ifdef _CNIEB0
#define _InitPinB0PUD(PullUpDn)    _CNPUB0 = (PullUpDn&0b01); _CNPDB0 = (PullUpDn>>1);
#define _InitPinB0CN(CNI_EN)          _CNIEB0 = (CNI_EN);
#else
#define _InitPinB0PUD(PullUpDn)
#define _InitPinB0CN(CNI_EN)
#endif
#ifdef _CNIEB1
#define _InitPinB1PUD(PullUpDn)    _CNPUB1 = (PullUpDn&0b01); _CNPDB1 = (PullUpDn>>1);
#define _InitPinB1CN(CNI_EN)          _CNIEB1 = (CNI_EN);
#else
#define _InitPinB1PUD(PullUpDn)
#define _InitPinB1CN(CNI_EN)
#endif
#ifdef _CNIEB2
#define _InitPinB2PUD(PullUpDn)    _CNPUB2 = (PullUpDn&0b01); _CNPDB2 = (PullUpDn>>1);
#define _InitPinB2CN(CNI_EN)          _CNIEB2 = (CNI_EN);
#else
#define _InitPinB2PUD(PullUpDn)
#define _InitPinB2CN(CNI_EN)
#endif
#ifdef _CNIEB3
#define _InitPinB3PUD(PullUpDn)    _CNPUB3 = (PullUpDn&0b01); _CNPDB3 = (PullUpDn>>1);
#define _InitPinB3CN(CNI_EN)          _CNIEB3 = (CNI_EN);
#else
#define _InitPinB3PUD(PullUpDn)
#define _InitPinB3CN(CNI_EN)
#endif
#ifdef _CNIEB4
#define _InitPinB4PUD(PullUpDn)    _CNPUB4 = (PullUpDn&0b01); _CNPDB4 = (PullUpDn>>1);
#define _InitPinB4CN(CNI_EN)          _CNIEB4 = (CNI_EN);
#else
#define _InitPinB4PUD(PullUpDn)
#define _InitPinB4CN(CNI_EN)
#endif
#ifdef _CNIEB5
#define _InitPinB5PUD(PullUpDn)    _CNPUB5 = (PullUpDn&0b01); _CNPDB5 = (PullUpDn>>1);
#define _InitPinB5CN(CNI_EN)          _CNIEB5 = (CNI_EN);
#else
#define _InitPinB5PUD(PullUpDn)
#define _InitPinB5CN(CNI_EN)
#endif
#ifdef _CNIEB6
#define _InitPinB6PUD(PullUpDn)    _CNPUB6 = (PullUpDn&0b01); _CNPDB6 = (PullUpDn>>1);
#define _InitPinB6CN(CNI_EN)          _CNIEB6 = (CNI_EN);
#else
#define _InitPinB6PUD(PullUpDn)
#define _InitPinB6CN(CNI_EN)
#endif
#ifdef _CNIEB7
#define _InitPinB7PUD(PullUpDn)    _CNPUB7 = (PullUpDn&0b01); _CNPDB7 = (PullUpDn>>1);
#define _InitPinB7CN(CNI_EN)          _CNIEB7 = (CNI_EN);
#else
#define _InitPinB7PUD(PullUpDn)
#define _InitPinB7CN(CNI_EN)
#endif
#ifdef _CNIEB8
#define _InitPinB8PUD(PullUpDn)    _CNPUB8 = (PullUpDn&0b01); _CNPDB8 = (PullUpDn>>1);
#define _InitPinB8CN(CNI_EN)          _CNIEB8 = (CNI_EN);
#else
#define _InitPinB8PUD(PullUpDn)
#define _InitPinB8CN(CNI_EN)
#endif
#ifdef _CNIEB9
#define _InitPinB9PUD(PullUpDn)    _CNPUB9 = (PullUpDn&0b01); _CNPDB9 = (PullUpDn>>1);
#define _InitPinB9CN(CNI_EN)          _CNIEB9 = (CNI_EN);
#else
#define _InitPinB9PUD(PullUpDn)
#define _InitPinB9CN(CNI_EN)
#endif
#ifdef _CNIEB10
#define _InitPinB10PUD(PullUpDn)    _CNPUB10 = (PullUpDn&0b01); _CNPDB10 = (PullUpDn>>1);
#define _InitPinB10CN(CNI_EN)          _CNIEB10 = (CNI_EN);
#else
#define _InitPinB10PUD(PullUpDn)
#define _InitPinB10CN(CNI_EN)
#endif
#ifdef _CNIEB11
#define _InitPinB11PUD(PullUpDn)    _CNPUB11 = (PullUpDn&0b01); _CNPDB11 = (PullUpDn>>1);
#define _InitPinB11CN(CNI_EN)          _CNIEB11 = (CNI_EN);
#else
#define _InitPinB11PUD(PullUpDn)
#define _InitPinB11CN(CNI_EN)
#endif
#ifdef _CNIEB12
#define _InitPinB12PUD(PullUpDn)    _CNPUB12 = (PullUpDn&0b01); _CNPDB12 = (PullUpDn>>1);
#define _InitPinB12CN(CNI_EN)          _CNIEB12 = (CNI_EN);
#else
#define _InitPinB12PUD(PullUpDn)
#define _InitPinB12CN(CNI_EN)
#endif
#ifdef _CNIEB13
#define _InitPinB13PUD(PullUpDn)    _CNPUB13 = (PullUpDn&0b01); _CNPDB13 = (PullUpDn>>1);
#define _InitPinB13CN(CNI_EN)          _CNIEB13 = (CNI_EN);
#else
#define _InitPinB13PUD(PullUpDn)
#define _InitPinB13CN(CNI_EN)
#endif
#ifdef _CNIEB14
#define _InitPinB14PUD(PullUpDn)    _CNPUB14 = (PullUpDn&0b01); _CNPDB14 = (PullUpDn>>1);
#define _InitPinB14CN(CNI_EN)          _CNIEB14 = (CNI_EN);
#else
#define _InitPinB14PUD(PullUpDn)
#define _InitPinB14CN(CNI_EN)
#endif
#ifdef _CNIEB15
#define _InitPinB15PUD(PullUpDn)    _CNPUB15 = (PullUpDn&0b01); _CNPDB15 = (PullUpDn>>1);
#define _InitPinB15CN(CNI_EN)          _CNIEB15 = (CNI_EN);
#else
#define _InitPinB15PUD(PullUpDn)
#define _InitPinB15CN(CNI_EN)
#endif

#ifdef _RB0
/***********************************************************************************************************************
 * \Function            void InitPinB0(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB0 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB0(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB0(PinMode,PullUpDn,CNI_EN)  ({_TRISB0 = ((PinMode>>1)&0b01); _InitPinB0AN(PinMode) _InitPinB0OD(PinMode) _InitPinB0PUD(PullUpDn) _InitPinB0CN(CNI_EN)})
#define ReadPinB0()           (PORTBbits.RB0)
#define SetPinB0(Value)     LATBbits.LATB0 = (Value)
#endif
#ifdef _RB1
/***********************************************************************************************************************
 * \Function            void InitPinB1(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB1 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB1(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB1(PinMode,PullUpDn,CNI_EN)  ({_TRISB1 = ((PinMode>>1)&0b01); _InitPinB1AN(PinMode)  _InitPinB1OD(PinMode) _InitPinB1PUD(PullUpDn) _InitPinB1CN(CNI_EN)})
#define ReadPinB1()           (PORTBbits.RB1)
#define SetPinB1(Value)     LATBbits.LATB1 = (Value)
#endif
#ifdef _RB2
/***********************************************************************************************************************
 * \Function            void InitPinB2(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB2 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB2(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB2(PinMode,PullUpDn,CNI_EN)  ({_TRISB2 = ((PinMode>>1)&0b01); _InitPinB2AN(PinMode)  _InitPinB2OD(PinMode) _InitPinB2PUD(PullUpDn) _InitPinB2CN(CNI_EN)})
#define ReadPinB2()           (PORTBbits.RB2)
#define SetPinB2(Value)     LATBbits.LATB2 = (Value)
#endif
#ifdef _RB3
/***********************************************************************************************************************
 * \Function            void InitPinB3(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB3 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB3(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB3(PinMode,PullUpDn,CNI_EN)  ({_TRISB3 = ((PinMode>>1)&0b01); _InitPinB3AN(PinMode)  _InitPinB3OD(PinMode) _InitPinB3PUD(PullUpDn) _InitPinB3CN(CNI_EN)})
#define ReadPinB3()           (PORTBbits.RB3)
#define SetPinB3(Value)     LATBbits.LATB3 = (Value)
#endif
#ifdef _RB4
/***********************************************************************************************************************
 * \Function            void InitPinB4(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB4 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB4(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB4(PinMode,PullUpDn,CNI_EN)  ({_TRISB4 = ((PinMode>>1)&0b01); _InitPinB4AN(PinMode)  _InitPinB4OD(PinMode) _InitPinB4PUD(PullUpDn) _InitPinB4CN(CNI_EN)})
#define ReadPinB4()           (PORTBbits.RB4)
#define SetPinB4(Value)     LATBbits.LATB4 = (Value)
#endif
#ifdef _RB5
/***********************************************************************************************************************
 * \Function            void InitPinB5(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB5 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB5(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB5(PinMode,PullUpDn,CNI_EN)  ({_TRISB5 = ((PinMode>>1)&0b01); _InitPinB5AN(PinMode)  _InitPinB5OD(PinMode) _InitPinB5PUD(PullUpDn) _InitPinB5CN(CNI_EN)})
#define ReadPinB5()           (PORTBbits.RB5)
#define SetPinB5(Value)     LATBbits.LATB5 = (Value)
#endif
#ifdef _RB6
/***********************************************************************************************************************
 * \Function            void InitPinB6(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB6 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB6(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB6(PinMode,PullUpDn,CNI_EN)  ({_TRISB6 = ((PinMode>>1)&0b01); _InitPinB6AN(PinMode)  _InitPinB6OD(PinMode) _InitPinB6PUD(PullUpDn) _InitPinB6CN(CNI_EN)})
#define ReadPinB6()           (PORTBbits.RB6)
#define SetPinB6(Value)     LATBbits.LATB6 = (Value)
#endif
#ifdef _RB7
/***********************************************************************************************************************
 * \Function            void InitPinB7(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB7 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB7(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB7(PinMode,PullUpDn,CNI_EN)  ({_TRISB7 = ((PinMode>>1)&0b01); _InitPinB7AN(PinMode)  _InitPinB7OD(PinMode) _InitPinB7PUD(PullUpDn) _InitPinB7CN(CNI_EN)})
#define ReadPinB7()           (PORTBbits.RB7)
#define SetPinB7(Value)     LATBbits.LATB7 = (Value)
#endif
#ifdef _RB8
/***********************************************************************************************************************
 * \Function            void InitPinB8(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB8 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB8(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB8(PinMode,PullUpDn,CNI_EN)  ({_TRISB8 = ((PinMode>>1)&0b01); _InitPinB8AN(PinMode)  _InitPinB8OD(PinMode) _InitPinB8PUD(PullUpDn) _InitPinB8CN(CNI_EN)})
#define ReadPinB8()           (PORTBbits.RB8)
#define SetPinB8(Value)     LATBbits.LATB8 = (Value)
#endif
#ifdef _RB9
/***********************************************************************************************************************
 * \Function            void InitPinB9(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB9 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB9(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB9(PinMode,PullUpDn,CNI_EN)  ({_TRISB9 = ((PinMode>>1)&0b01); _InitPinB9AN(PinMode)  _InitPinB9OD(PinMode) _InitPinB9PUD(PullUpDn) _InitPinB9CN(CNI_EN)})
#define ReadPinB9()           (PORTBbits.RB9)
#define SetPinB9(Value)     LATBbits.LATB9 = (Value)
#endif
#ifdef _RB10
/***********************************************************************************************************************
 * \Function            void InitPinB10(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB10 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB10(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB10(PinMode,PullUpDn,CNI_EN)  ({_TRISB10 = ((PinMode>>1)&0b01); _InitPinB10AN(PinMode)  _InitPinB10OD(PinMode) _InitPinB10PUD(PullUpDn) _InitPinB10CN(CNI_EN)})
#define ReadPinB10()          (PORTBbits.RB10)
#define SetPinB10(Value)     LATBbits.LATB10 = (Value)
#endif
#ifdef _RB11
/***********************************************************************************************************************
 * \Function            void InitPinB11(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB11 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB11(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB11(PinMode,PullUpDn,CNI_EN)  ({_TRISB11 = ((PinMode>>1)&0b01); _InitPinB11AN(PinMode)  _InitPinB11OD(PinMode) _InitPinB11PUD(PullUpDn) _InitPinB11CN(CNI_EN)})
#define LATKB11          (PORTBbits.RB11)
#define SetPinB11(Value)     LATBbits.LATB11 = (Value)
#endif
#ifdef _RB12
/***********************************************************************************************************************
 * \Function            void InitPinB12(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB12 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB12(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB12(PinMode,PullUpDn,CNI_EN)  ({_TRISB12 = ((PinMode>>1)&0b01); _InitPinB12AN(PinMode)  _InitPinB12OD(PinMode) _InitPinB12PUD(PullUpDn) _InitPinB12CN(CNI_EN)})
#define ReadPinB12()          (PORTBbits.RB12)
#define SetPinB12(Value)     LATBbits.LATB12 = (Value)
#endif
#ifdef _RB13
/***********************************************************************************************************************
 * \Function            void InitPinB13(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB13 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB13(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB13(PinMode,PullUpDn,CNI_EN)  ({_TRISB13 = ((PinMode>>1)&0b01); _InitPinB13AN(PinMode)  _InitPinB13OD(PinMode) _InitPinB13PUD(PullUpDn) _InitPinB13CN(CNI_EN)})
#define LATKB13          (PORTBbits.RB13)
#define SetPinB13(Value)     LATBbits.LATB13 = (Value)
#endif
#ifdef _RB14
/***********************************************************************************************************************
 * \Function            void InitPinB14(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB14 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB14(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB14(PinMode,PullUpDn,CNI_EN)  ({_TRISB14 = ((PinMode>>1)&0b01); _InitPinB14AN(PinMode)  _InitPinB14OD(PinMode) _InitPinB14PUD(PullUpDn) _InitPinB14CN(CNI_EN)})
#define ReadPinB14()          (PORTBbits.RB14)
#define SetPinB14(Value)     LATBbits.LATB14 = (Value)
#endif
#ifdef _RB15
/***********************************************************************************************************************
 * \Function            void InitPinB15(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTB.RB15 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinB15(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinB15(PinMode,PullUpDn,CNI_EN)  ({_TRISB15 = ((PinMode>>1)&0b01); _InitPinB15AN(PinMode)  _InitPinB15OD(PinMode) _InitPinB15PUD(PullUpDn) _InitPinB15CN(CNI_EN)})
#define ReadPinB15()          (PORTBbits.RB15)
#define SetPinB15(Value)     LATBbits.LATB15 = (Value)
#endif

#endif
#ifdef  PORTC

#define ReadPORTC()           (PORTC)
#define SetPORTC(Value)     LATC = (Value)

#ifdef ANSELC
/***********************************************************************************************************************
 * \Function            void InitAnalogPinsPortC(AnalogPins)
 *
 * \Description         Set PortC Individual Pins Mode (Analog OR Digital).
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>AnalogPins:</u>\n
 *  - RC0AN to RC15AN   <i>/ The Corresponding Pin is Analog Pin.</i>   \n
 *  - All_PINS_DIG      <i>/ All PortC Pins is Digital.</i>   \n
 *  - All_PINS_AN       <i>/ All PortC Pins is Analog.</i>
 *
 * \Return      None
 *
 * \Notes       None
 *
 * \Example     InitAnalogPinsPortC(All_PINS_DIG & RC6AN & RC7AN); \n
 * \Example     InitAnalogPinsPortG(All_PINS_AN);
 *
 **********************************************************************************************************************/
#define InitAnalogPinsPortC(AnalogPins)    ANSELC = ~(AnalogPins)
#endif
#ifdef _ANSC0
#define RC0AN	0b1111111111111110	/* RC0 bin is Analog bin */
#define _InitPinC0AN(PinMode)    _ANSC0 = (PinMode>>2);
#else
#define _InitPinC0AN(PinMode)
#endif
#ifdef _ANSC1
#define RC1AN	0b1111111111111101	/* RC1 bin is Analog bin */
#define _InitPinC1AN(PinMode)    _ANSC1 = (PinMode>>2);
#else
#define _InitPinC1AN(PinMode)
#endif
#ifdef _ANSC2
#define RC2AN	0b1111111111111011	/* RC2 bin is Analog bin */
#define _InitPinC2AN(PinMode)    _ANSC2 = (PinMode>>2);
#else
#define _InitPinC2AN(PinMode)
#endif
#ifdef _ANSC3
#define RC3AN	0b1111111111110111	/* RC3 bin is Analog bin */
#define _InitPinC3AN(PinMode)    _ANSC3 = (PinMode>>2);
#else
#define _InitPinC3AN(PinMode)
#endif
#ifdef _ANSC4
#define RC4AN	0b1111111111101111	/* RC4 bin is Analog bin */
#define _InitPinC4AN(PinMode)    _ANSC4 = (PinMode>>2);
#else
#define _InitPinC4AN(PinMode)
#endif
#ifdef _ANSC5
#define RC5AN	0b1111111111011111	/* RC5 bin is Analog bin */
#define _InitPinC5AN(PinMode)    _ANSC5 = (PinMode>>2);
#else
#define _InitPinC5AN(PinMode)
#endif
#ifdef _ANSC6
#define RC6AN	0b1111111110111111	/* RC6 bin is Analog bin */
#define _InitPinC6AN(PinMode)    _ANSC6 = (PinMode>>2);
#else
#define _InitPinC6AN(PinMode)
#endif
#ifdef _ANSC7
#define RC7AN	0b1111111101111111	/* RC7 bin is Analog bin */
#define _InitPinC7AN(PinMode)    _ANSC7 = (PinMode>>2);
#else
#define _InitPinC7AN(PinMode)
#endif
#ifdef _ANSC8
#define RC8AN	0b1111111011111111	/* RC8 bin is Analog bin */
#define _InitPinC8AN(PinMode)    _ANSC8 = (PinMode>>2);
#else
#define _InitPinC8AN(PinMode)
#endif
#ifdef _ANSC9
#define RC9AN	0b1111110111111111	/* RC9 bin is Analog bin */
#define _InitPinC9AN(PinMode)    _ANSC9 = (PinMode>>2);
#else
#define _InitPinC9AN(PinMode)
#endif
#ifdef _ANSC10
#define RC10AN	0b1111101111111111	/* RC10 bin is Analog bin */
#define _InitPinC10AN(PinMode)    _ANSC10 = (PinMode>>2);
#else
#define _InitPinC10AN(PinMode)
#endif
#ifdef _ANSC11
#define RC11AN	0b1111011111111111	/* RC11 bin is Analog bin */
#define _InitPinC11AN(PinMode)    _ANSC11 = (PinMode>>2);
#else
#define _InitPinC11AN(PinMode)
#endif
#ifdef _ANSC12
#define RC12AN	0b1110111111111111	/* RC12 bin is Analog bin */
#define _InitPinC12AN(PinMode)    _ANSC12 = (PinMode>>2);
#else
#define _InitPinC12AN(PinMode)
#endif
#ifdef _ANSC13
#define RC13AN	0b1101111111111111	/* RC13 bin is Analog bin */
#define _InitPinC13AN(PinMode)    _ANSC13 = (PinMode>>2);
#else
#define _InitPinC13AN(PinMode)
#endif
#ifdef _ANSC14
#define RC14AN	0b1011111111111111	/* RC14 bin is Analog bin */
#define _InitPinC14AN(PinMode)    _ANSC14 = (PinMode>>2);
#else
#define _InitPinC14AN(PinMode)
#endif
#ifdef _ANSC15
#define RC15AN	0b0111111111111111	/* RC15 bin is Analog bin */
#define _InitPinC15AN(PinMode)    _ANSC15 = (PinMode>>2);
#else
#define _InitPinC15AN(PinMode)
#endif

#ifdef _ODCC0
#define _InitPinC0OD(PinMode)    _ODCC0 = (PinMode&0b001);
#else
#define _InitPinC0OD(PinMode)
#endif
#ifdef _ODCC1
#define _InitPinC1OD(PinMode)    _ODCC1 = (PinMode&0b001);
#else
#define _InitPinC1OD(PinMode)
#endif
#ifdef _ODCC2
#define _InitPinC2OD(PinMode)    _ODCC2 = (PinMode&0b001);
#else
#define _InitPinC2OD(PinMode)
#endif
#ifdef _ODCC3
#define _InitPinC3OD(PinMode)    _ODCC3 = (PinMode&0b001);
#else
#define _InitPinC3OD(PinMode)
#endif
#ifdef _ODCC4
#define _InitPinC4OD(PinMode)    _ODCC4 = (PinMode&0b001);
#else
#define _InitPinC4OD(PinMode)
#endif
#ifdef _ODCC5
#define _InitPinC5OD(PinMode)    _ODCC5 = (PinMode&0b001);
#else
#define _InitPinC5OD(PinMode)
#endif
#ifdef _ODCC6
#define _InitPinC6OD(PinMode)    _ODCC6 = (PinMode&0b001);
#else
#define _InitPinC6OD(PinMode)
#endif
#ifdef _ODCC7
#define _InitPinC7OD(PinMode)    _ODCC7 = (PinMode&0b001);
#else
#define _InitPinC7OD(PinMode)
#endif
#ifdef _ODCC8
#define _InitPinC8OD(PinMode)    _ODCC8 = (PinMode&0b001);
#else
#define _InitPinC8OD(PinMode)
#endif
#ifdef _ODCC9
#define _InitPinC9OD(PinMode)    _ODCC9 = (PinMode&0b001);
#else
#define _InitPinC9OD(PinMode)
#endif
#ifdef _ODCC10
#define _InitPinC10OD(PinMode)    _ODCC10 = (PinMode&0b001);
#else
#define _InitPinC10OD(PinMode)
#endif
#ifdef _ODCC11
#define _InitPinC11OD(PinMode)    _ODCC11 = (PinMode&0b001);
#else
#define _InitPinC11OD(PinMode)
#endif
#ifdef _ODCC12
#define _InitPinC12OD(PinMode)    _ODCC12 = (PinMode&0b001);
#else
#define _InitPinC12OD(PinMode)
#endif
#ifdef _ODCC13
#define _InitPinC13OD(PinMode)    _ODCC13 = (PinMode&0b001);
#else
#define _InitPinC13OD(PinMode)
#endif
#ifdef _ODCC14
#define _InitPinC14OD(PinMode)    _ODCC14 = (PinMode&0b001);
#else
#define _InitPinC14OD(PinMode)
#endif
#ifdef _ODCC15
#define _InitPinC15OD(PinMode)    _ODCC15 = (PinMode&0b001);
#else
#define _InitPinC15OD(PinMode)
#endif

#ifdef _CNIEC0
#define _InitPinC0PUD(PullUpDn)    _CNPUC0 = (PullUpDn&0b01); _CNPDC0 = (PullUpDn>>1);
#define _InitPinC0CN(CNI_EN)          _CNIEC0 = (CNI_EN);
#else
#define _InitPinC0PUD(PullUpDn)
#define _InitPinC0CN(CNI_EN)
#endif
#ifdef _CNIEC1
#define _InitPinC1PUD(PullUpDn)    _CNPUC1 = (PullUpDn&0b01); _CNPDC1 = (PullUpDn>>1);
#define _InitPinC1CN(CNI_EN)          _CNIEC1 = (CNI_EN);
#else
#define _InitPinC1PUD(PullUpDn)
#define _InitPinC1CN(CNI_EN)
#endif
#ifdef _CNIEC2
#define _InitPinC2PUD(PullUpDn)    _CNPUC2 = (PullUpDn&0b01); _CNPDC2 = (PullUpDn>>1);
#define _InitPinC2CN(CNI_EN)          _CNIEC2 = (CNI_EN);
#else
#define _InitPinC2PUD(PullUpDn)
#define _InitPinC2CN(CNI_EN)
#endif
#ifdef _CNIEC3
#define _InitPinC3PUD(PullUpDn)    _CNPUC3 = (PullUpDn&0b01); _CNPDC3 = (PullUpDn>>1);
#define _InitPinC3CN(CNI_EN)          _CNIEC3 = (CNI_EN);
#else
#define _InitPinC3PUD(PullUpDn)
#define _InitPinC3CN(CNI_EN)
#endif
#ifdef _CNIEC4
#define _InitPinC4PUD(PullUpDn)    _CNPUC4 = (PullUpDn&0b01); _CNPDC4 = (PullUpDn>>1);
#define _InitPinC4CN(CNI_EN)          _CNIEC4 = (CNI_EN);
#else
#define _InitPinC4PUD(PullUpDn)
#define _InitPinC4CN(CNI_EN)
#endif
#ifdef _CNIEC5
#define _InitPinC5PUD(PullUpDn)    _CNPUC5 = (PullUpDn&0b01); _CNPDC5 = (PullUpDn>>1);
#define _InitPinC5CN(CNI_EN)          _CNIEC5 = (CNI_EN);
#else
#define _InitPinC5PUD(PullUpDn)
#define _InitPinC5CN(CNI_EN)
#endif
#ifdef _CNIEC6
#define _InitPinC6PUD(PullUpDn)    _CNPUC6 = (PullUpDn&0b01); _CNPDC6 = (PullUpDn>>1);
#define _InitPinC6CN(CNI_EN)          _CNIEC6 = (CNI_EN);
#else
#define _InitPinC6PUD(PullUpDn)
#define _InitPinC6CN(CNI_EN)
#endif
#ifdef _CNIEC7
#define _InitPinC7PUD(PullUpDn)    _CNPUC7 = (PullUpDn&0b01); _CNPDC7 = (PullUpDn>>1);
#define _InitPinC7CN(CNI_EN)          _CNIEC7 = (CNI_EN);
#else
#define _InitPinC7PUD(PullUpDn)
#define _InitPinC7CN(CNI_EN)
#endif
#ifdef _CNIEC8
#define _InitPinC8PUD(PullUpDn)    _CNPUC8 = (PullUpDn&0b01); _CNPDC8 = (PullUpDn>>1);
#define _InitPinC8CN(CNI_EN)          _CNIEC8 = (CNI_EN);
#else
#define _InitPinC8PUD(PullUpDn)
#define _InitPinC8CN(CNI_EN)
#endif
#ifdef _CNIEC9
#define _InitPinC9PUD(PullUpDn)    _CNPUC9 = (PullUpDn&0b01); _CNPDC9 = (PullUpDn>>1);
#define _InitPinC9CN(CNI_EN)          _CNIEC9 = (CNI_EN);
#else
#define _InitPinC9PUD(PullUpDn)
#define _InitPinC9CN(CNI_EN)
#endif
#ifdef _CNIEC10
#define _InitPinC10PUD(PullUpDn)    _CNPUC10 = (PullUpDn&0b01); _CNPDC10 = (PullUpDn>>1);
#define _InitPinC10CN(CNI_EN)          _CNIEC10 = (CNI_EN);
#else
#define _InitPinC10PUD(PullUpDn)
#define _InitPinC10CN(CNI_EN)
#endif
#ifdef _CNIEC11
#define _InitPinC11PUD(PullUpDn)    _CNPUC11 = (PullUpDn&0b01); _CNPDC11 = (PullUpDn>>1);
#define _InitPinC11CN(CNI_EN)          _CNIEC11 = (CNI_EN);
#else
#define _InitPinC11PUD(PullUpDn)
#define _InitPinC11CN(CNI_EN)
#endif
#ifdef _CNIEC12
#define _InitPinC12PUD(PullUpDn)    _CNPUC12 = (PullUpDn&0b01); _CNPDC12 = (PullUpDn>>1);
#define _InitPinC12CN(CNI_EN)          _CNIEC12 = (CNI_EN);
#else
#define _InitPinC12PUD(PullUpDn)
#define _InitPinC12CN(CNI_EN)
#endif
#ifdef _CNIEC13
#define _InitPinC13PUD(PullUpDn)    _CNPUC13 = (PullUpDn&0b01); _CNPDC13 = (PullUpDn>>1);
#define _InitPinC13CN(CNI_EN)          _CNIEC13 = (CNI_EN);
#else
#define _InitPinC13PUD(PullUpDn)
#define _InitPinC13CN(CNI_EN)
#endif
#ifdef _CNIEC14
#define _InitPinC14PUD(PullUpDn)    _CNPUC14 = (PullUpDn&0b01); _CNPDC14 = (PullUpDn>>1);
#define _InitPinC14CN(CNI_EN)          _CNIEC14 = (CNI_EN);
#else
#define _InitPinC14PUD(PullUpDn)
#define _InitPinC14CN(CNI_EN)
#endif
#ifdef _CNIEC15
#define _InitPinC15PUD(PullUpDn)    _CNPUC15 = (PullUpDn&0b01); _CNPDC15 = (PullUpDn>>1);
#define _InitPinC15CN(CNI_EN)          _CNIEC15 = (CNI_EN);
#else
#define _InitPinC15PUD(PullUpDn)
#define _InitPinC15CN(CNI_EN)
#endif

#ifdef _RC0
/***********************************************************************************************************************
 * \Function            void InitPinC0(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC0 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC0(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC0(PinMode,PullUpDn,CNI_EN)  ({_TRISC0 = ((PinMode>>1)&0b01); _InitPinC0AN(PinMode) _InitPinC0OD(PinMode) _InitPinC0PUD(PullUpDn) _InitPinC0CN(CNI_EN)})
#define ReadPinC0()           (PORTCbits.RC0)
#define SetPinC0(Value)     LATCbits.LATC0 = (Value)
#endif
#ifdef _RC1
/***********************************************************************************************************************
 * \Function            void InitPinC1(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC1 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC1(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC1(PinMode,PullUpDn,CNI_EN)  ({_TRISC1 = ((PinMode>>1)&0b01); _InitPinC1AN(PinMode)  _InitPinC1OD(PinMode) _InitPinC1PUD(PullUpDn) _InitPinC1CN(CNI_EN)})
#define ReadPinC1()           (PORTCbits.RC1)
#define SetPinC1(Value)     LATCbits.LATC1 = (Value)
#endif
#ifdef _RC2
/***********************************************************************************************************************
 * \Function            void InitPinC2(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC2 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC2(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC2(PinMode,PullUpDn,CNI_EN)  ({_TRISC2 = ((PinMode>>1)&0b01); _InitPinC2AN(PinMode)  _InitPinC2OD(PinMode) _InitPinC2PUD(PullUpDn) _InitPinC2CN(CNI_EN)})
#define ReadPinC2()           (PORTCbits.RC2)
#define SetPinC2(Value)     LATCbits.LATC2 = (Value)
#endif
#ifdef _RC3
/***********************************************************************************************************************
 * \Function            void InitPinC3(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC3 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC3(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC3(PinMode,PullUpDn,CNI_EN)  ({_TRISC3 = ((PinMode>>1)&0b01); _InitPinC3AN(PinMode)  _InitPinC3OD(PinMode) _InitPinC3PUD(PullUpDn) _InitPinC3CN(CNI_EN)})
#define ReadPinC3()           (PORTCbits.RC3)
#define SetPinC3(Value)     LATCbits.LATC3 = (Value)
#endif
#ifdef _RC4
/***********************************************************************************************************************
 * \Function            void InitPinC4(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC4 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC4(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC4(PinMode,PullUpDn,CNI_EN)  ({_TRISC4 = ((PinMode>>1)&0b01); _InitPinC4AN(PinMode)  _InitPinC4OD(PinMode) _InitPinC4PUD(PullUpDn) _InitPinC4CN(CNI_EN)})
#define ReadPinC4()           (PORTCbits.RC4)
#define SetPinC4(Value)     LATCbits.LATC4 = (Value)
#endif
#ifdef _RC5
/***********************************************************************************************************************
 * \Function            void InitPinC5(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC5 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC5(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC5(PinMode,PullUpDn,CNI_EN)  ({_TRISC5 = ((PinMode>>1)&0b01); _InitPinC5AN(PinMode)  _InitPinC5OD(PinMode) _InitPinC5PUD(PullUpDn) _InitPinC5CN(CNI_EN)})
#define ReadPinC5()           (PORTCbits.RC5)
#define SetPinC5(Value)     LATCbits.LATC5 = (Value)
#endif
#ifdef _RC6
/***********************************************************************************************************************
 * \Function            void InitPinC6(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC6 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC6(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC6(PinMode,PullUpDn,CNI_EN)  ({_TRISC6 = ((PinMode>>1)&0b01); _InitPinC6AN(PinMode)  _InitPinC6OD(PinMode) _InitPinC6PUD(PullUpDn) _InitPinC6CN(CNI_EN)})
#define ReadPinC6()           (PORTCbits.RC6)
#define SetPinC6(Value)     LATCbits.LATC6 = (Value)
#endif
#ifdef _RC7
/***********************************************************************************************************************
 * \Function            void InitPinC7(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC7 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC7(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC7(PinMode,PullUpDn,CNI_EN)  ({_TRISC7 = ((PinMode>>1)&0b01); _InitPinC7AN(PinMode)  _InitPinC7OD(PinMode) _InitPinC7PUD(PullUpDn) _InitPinC7CN(CNI_EN)})
#define ReadPinC7()           (PORTCbits.RC7)
#define SetPinC7(Value)     LATCbits.LATC7 = (Value)
#endif
#ifdef _RC8
/***********************************************************************************************************************
 * \Function            void InitPinC8(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC8 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC8(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC8(PinMode,PullUpDn,CNI_EN)  ({_TRISC8 = ((PinMode>>1)&0b01); _InitPinC8AN(PinMode)  _InitPinC8OD(PinMode) _InitPinC8PUD(PullUpDn) _InitPinC8CN(CNI_EN)})
#define ReadPinC8()           (PORTCbits.RC8)
#define SetPinC8(Value)     LATCbits.LATC8 = (Value)
#endif
#ifdef _RC9
/***********************************************************************************************************************
 * \Function            void InitPinC9(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC9 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC9(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC9(PinMode,PullUpDn,CNI_EN)  ({_TRISC9 = ((PinMode>>1)&0b01); _InitPinC9AN(PinMode)  _InitPinC9OD(PinMode) _InitPinC9PUD(PullUpDn) _InitPinC9CN(CNI_EN)})
#define ReadPinC9()           (PORTCbits.RC9)
#define SetPinC9(Value)     LATCbits.LATC9 = (Value)
#endif
#ifdef _RC10
/***********************************************************************************************************************
 * \Function            void InitPinC10(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC10 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC10(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC10(PinMode,PullUpDn,CNI_EN)  ({_TRISC10 = ((PinMode>>1)&0b01); _InitPinC10AN(PinMode)  _InitPinC10OD(PinMode) _InitPinC10PUD(PullUpDn) _InitPinC10CN(CNI_EN)})
#define ReadPinC10()          (PORTCbits.RC10)
#define SetPinC10(Value)     LATCbits.LATC10 = (Value)
#endif
#ifdef _RC11
/***********************************************************************************************************************
 * \Function            void InitPinC11(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC11 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC11(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC11(PinMode,PullUpDn,CNI_EN)  ({_TRISC11 = ((PinMode>>1)&0b01); _InitPinC11AN(PinMode)  _InitPinC11OD(PinMode) _InitPinC11PUD(PullUpDn) _InitPinC11CN(CNI_EN)})
#define ReadPinC11()          (PORTCbits.RC11)
#define SetPinC11(Value)     LATCbits.LATC11 = (Value)
#endif
#ifdef _RC12
/***********************************************************************************************************************
 * \Function            void InitPinC12(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC12 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC12(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC12(PinMode,PullUpDn,CNI_EN)  ({_TRISC12 = ((PinMode>>1)&0b01); _InitPinC12AN(PinMode)  _InitPinC12OD(PinMode) _InitPinC12PUD(PullUpDn) _InitPinC12CN(CNI_EN)})
#define ReadPinC12()          (PORTCbits.RC12)
#define SetPinC12(Value)     LATCbits.LATC12 = (Value)
#endif
#ifdef _RC13
/***********************************************************************************************************************
 * \Function            void InitPinC13(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC13 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC13(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC13(PinMode,PullUpDn,CNI_EN)  ({_TRISC13 = ((PinMode>>1)&0b01); _InitPinC13AN(PinMode)  _InitPinC13OD(PinMode) _InitPinC13PUD(PullUpDn) _InitPinC13CN(CNI_EN)})
#define ReadPinC13()          (PORTCbits.RC13)
#define SetPinC13(Value)     LATCbits.LATC13 = (Value)
#endif
#ifdef _RC14
/***********************************************************************************************************************
 * \Function            void InitPinC14(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC14 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC14(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC14(PinMode,PullUpDn,CNI_EN)  ({_TRISC14 = ((PinMode>>1)&0b01); _InitPinC14AN(PinMode)  _InitPinC14OD(PinMode) _InitPinC14PUD(PullUpDn) _InitPinC14CN(CNI_EN)})
#define ReadPinC14()          (PORTCbits.RC14)
#define SetPinC14(Value)     LATCbits.LATC14 = (Value)
#endif
#ifdef _RC15
/***********************************************************************************************************************
 * \Function            void InitPinC15(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTC.RC15 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinC15(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinC15(PinMode,PullUpDn,CNI_EN)  ({_TRISC15 = ((PinMode>>1)&0b01); _InitPinC15AN(PinMode)  _InitPinC15OD(PinMode) _InitPinC15PUD(PullUpDn) _InitPinC15CN(CNI_EN)})
#define ReadPinC15()          (PORTCbits.RC15)
#define SetPinC15(Value)     LATCbits.LATC15 = (Value)
#endif

#endif
#ifdef  PORTD

#define ReadPORTD()           (PORTD)
#define SetPORTD(Value)     LATD = (Value)

#ifdef ANSELD
/***********************************************************************************************************************
 * \Function            void InitAnalogPinsPortD(AnalogPins)
 *
 * \Description         Set PortD Individual Pins Mode (Analog OR Digital).
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>AnalogPins:</u>\n
 *  - RD0AN to RD15AN   <i>/ The Corresponding Pin is Analog Pin.</i>   \n
 *  - All_PINS_DIG      <i>/ All PortD Pins is Digital.</i>   \n
 *  - All_PINS_AN       <i>/ All PortD Pins is Analog.</i>
 *
 * \Return      None
 *
 * \Notes       None
 *
 * \Example     InitAnalogPinsPortD(All_PINS_DIG & RD6AN & RD7AN); \n
 * \Example     InitAnalogPinsPortG(All_PINS_AN);
 *
 **********************************************************************************************************************/
#define InitAnalogPinsPortD(AnalogPins)    ANSELD = ~(AnalogPins)
#endif
#ifdef _ANSD0
#define RD0AN	0b1111111111111110	/* RD0 bin is Analog bin */
#define _InitPinD0AN(PinMode)    _ANSD0 = (PinMode>>2);
#else
#define _InitPinD0AN(PinMode)
#endif
#ifdef _ANSD1
#define RD1AN	0b1111111111111101	/* RD1 bin is Analog bin */
#define _InitPinD1AN(PinMode)    _ANSD1 = (PinMode>>2);
#else
#define _InitPinD1AN(PinMode)
#endif
#ifdef _ANSD2
#define RD2AN	0b1111111111111011	/* RD2 bin is Analog bin */
#define _InitPinD2AN(PinMode)    _ANSD2 = (PinMode>>2);
#else
#define _InitPinD2AN(PinMode)
#endif
#ifdef _ANSD3
#define RD3AN	0b1111111111110111	/* RD3 bin is Analog bin */
#define _InitPinD3AN(PinMode)    _ANSD3 = (PinMode>>2);
#else
#define _InitPinD3AN(PinMode)
#endif
#ifdef _ANSD4
#define RD4AN	0b1111111111101111	/* RD4 bin is Analog bin */
#define _InitPinD4AN(PinMode)    _ANSD4 = (PinMode>>2);
#else
#define _InitPinD4AN(PinMode)
#endif
#ifdef _ANSD5
#define RD5AN	0b1111111111011111	/* RD5 bin is Analog bin */
#define _InitPinD5AN(PinMode)    _ANSD5 = (PinMode>>2);
#else
#define _InitPinD5AN(PinMode)
#endif
#ifdef _ANSD6
#define RD6AN	0b1111111110111111	/* RD6 bin is Analog bin */
#define _InitPinD6AN(PinMode)    _ANSD6 = (PinMode>>2);
#else
#define _InitPinD6AN(PinMode)
#endif
#ifdef _ANSD7
#define RD7AN	0b1111111101111111	/* RD7 bin is Analog bin */
#define _InitPinD7AN(PinMode)    _ANSD7 = (PinMode>>2);
#else
#define _InitPinD7AN(PinMode)
#endif
#ifdef _ANSD8
#define RD8AN	0b1111111011111111	/* RD8 bin is Analog bin */
#define _InitPinD8AN(PinMode)    _ANSD8 = (PinMode>>2);
#else
#define _InitPinD8AN(PinMode)
#endif
#ifdef _ANSD9
#define RD9AN	0b1111110111111111	/* RD9 bin is Analog bin */
#define _InitPinD9AN(PinMode)    _ANSD9 = (PinMode>>2);
#else
#define _InitPinD9AN(PinMode)
#endif
#ifdef _ANSD10
#define RD10AN	0b1111101111111111	/* RD10 bin is Analog bin */
#define _InitPinD10AN(PinMode)    _ANSD10 = (PinMode>>2);
#else
#define _InitPinD10AN(PinMode)
#endif
#ifdef _ANSD11
#define RD11AN	0b1111011111111111	/* RD11 bin is Analog bin */
#define _InitPinD11AN(PinMode)    _ANSD11 = (PinMode>>2);
#else
#define _InitPinD11AN(PinMode)
#endif
#ifdef _ANSD12
#define RD12AN	0b1110111111111111	/* RD12 bin is Analog bin */
#define _InitPinD12AN(PinMode)    _ANSD12 = (PinMode>>2);
#else
#define _InitPinD12AN(PinMode)
#endif
#ifdef _ANSD13
#define RD13AN	0b1101111111111111	/* RD13 bin is Analog bin */
#define _InitPinD13AN(PinMode)    _ANSD13 = (PinMode>>2);
#else
#define _InitPinD13AN(PinMode)
#endif
#ifdef _ANSD14
#define RD14AN	0b1011111111111111	/* RD14 bin is Analog bin */
#define _InitPinD14AN(PinMode)    _ANSD14 = (PinMode>>2);
#else
#define _InitPinD14AN(PinMode)
#endif
#ifdef _ANSD15
#define RD15AN	0b0111111111111111	/* RD15 bin is Analog bin */
#define _InitPinD15AN(PinMode)    _ANSD15 = (PinMode>>2);
#else
#define _InitPinD15AN(PinMode)
#endif

#ifdef _ODCD0
#define _InitPinD0OD(PinMode)    _ODCD0 = (PinMode&0b001);
#else
#define _InitPinD0OD(PinMode)
#endif
#ifdef _ODCD1
#define _InitPinD1OD(PinMode)    _ODCD1 = (PinMode&0b001);
#else
#define _InitPinD1OD(PinMode)
#endif
#ifdef _ODCD2
#define _InitPinD2OD(PinMode)    _ODCD2 = (PinMode&0b001);
#else
#define _InitPinD2OD(PinMode)
#endif
#ifdef _ODCD3
#define _InitPinD3OD(PinMode)    _ODCD3 = (PinMode&0b001);
#else
#define _InitPinD3OD(PinMode)
#endif
#ifdef _ODCD4
#define _InitPinD4OD(PinMode)    _ODCD4 = (PinMode&0b001);
#else
#define _InitPinD4OD(PinMode)
#endif
#ifdef _ODCD5
#define _InitPinD5OD(PinMode)    _ODCD5 = (PinMode&0b001);
#else
#define _InitPinD5OD(PinMode)
#endif
#ifdef _ODCD6
#define _InitPinD6OD(PinMode)    _ODCD6 = (PinMode&0b001);
#else
#define _InitPinD6OD(PinMode)
#endif
#ifdef _ODCD7
#define _InitPinD7OD(PinMode)    _ODCD7 = (PinMode&0b001);
#else
#define _InitPinD7OD(PinMode)
#endif
#ifdef _ODCD8
#define _InitPinD8OD(PinMode)    _ODCD8 = (PinMode&0b001);
#else
#define _InitPinD8OD(PinMode)
#endif
#ifdef _ODCD9
#define _InitPinD9OD(PinMode)    _ODCD9 = (PinMode&0b001);
#else
#define _InitPinD9OD(PinMode)
#endif
#ifdef _ODCD10
#define _InitPinD10OD(PinMode)    _ODCD10 = (PinMode&0b001);
#else
#define _InitPinD10OD(PinMode)
#endif
#ifdef _ODCD11
#define _InitPinD11OD(PinMode)    _ODCD11 = (PinMode&0b001);
#else
#define _InitPinD11OD(PinMode)
#endif
#ifdef _ODCD12
#define _InitPinD12OD(PinMode)    _ODCD12 = (PinMode&0b001);
#else
#define _InitPinD12OD(PinMode)
#endif
#ifdef _ODCD13
#define _InitPinD13OD(PinMode)    _ODCD13 = (PinMode&0b001);
#else
#define _InitPinD13OD(PinMode)
#endif
#ifdef _ODCD14
#define _InitPinD14OD(PinMode)    _ODCD14 = (PinMode&0b001);
#else
#define _InitPinD14OD(PinMode)
#endif
#ifdef _ODCD15
#define _InitPinD15OD(PinMode)    _ODCD15 = (PinMode&0b001);
#else
#define _InitPinD15OD(PinMode)
#endif

#ifdef _CNIED0
#define _InitPinD0PUD(PullUpDn)    _CNPUD0 = (PullUpDn&0b01); _CNPDD0 = (PullUpDn>>1);
#define _InitPinD0CN(CNI_EN)          _CNIED0 = (CNI_EN);
#else
#define _InitPinD0PUD(PullUpDn)
#define _InitPinD0CN(CNI_EN)
#endif
#ifdef _CNIED1
#define _InitPinD1PUD(PullUpDn)    _CNPUD1 = (PullUpDn&0b01); _CNPDD1 = (PullUpDn>>1);
#define _InitPinD1CN(CNI_EN)          _CNIED1 = (CNI_EN);
#else
#define _InitPinD1PUD(PullUpDn)
#define _InitPinD1CN(CNI_EN)
#endif
#ifdef _CNIED2
#define _InitPinD2PUD(PullUpDn)    _CNPUD2 = (PullUpDn&0b01); _CNPDD2 = (PullUpDn>>1);
#define _InitPinD2CN(CNI_EN)          _CNIED2 = (CNI_EN);
#else
#define _InitPinD2PUD(PullUpDn)
#define _InitPinD2CN(CNI_EN)
#endif
#ifdef _CNIED3
#define _InitPinD3PUD(PullUpDn)    _CNPUD3 = (PullUpDn&0b01); _CNPDD3 = (PullUpDn>>1);
#define _InitPinD3CN(CNI_EN)          _CNIED3 = (CNI_EN);
#else
#define _InitPinD3PUD(PullUpDn)
#define _InitPinD3CN(CNI_EN)
#endif
#ifdef _CNIED4
#define _InitPinD4PUD(PullUpDn)    _CNPUD4 = (PullUpDn&0b01); _CNPDD4 = (PullUpDn>>1);
#define _InitPinD4CN(CNI_EN)          _CNIED4 = (CNI_EN);
#else
#define _InitPinD4PUD(PullUpDn)
#define _InitPinD4CN(CNI_EN)
#endif
#ifdef _CNIED5
#define _InitPinD5PUD(PullUpDn)    _CNPUD5 = (PullUpDn&0b01); _CNPDD5 = (PullUpDn>>1);
#define _InitPinD5CN(CNI_EN)          _CNIED5 = (CNI_EN);
#else
#define _InitPinD5PUD(PullUpDn)
#define _InitPinD5CN(CNI_EN)
#endif
#ifdef _CNIED6
#define _InitPinD6PUD(PullUpDn)    _CNPUD6 = (PullUpDn&0b01); _CNPDD6 = (PullUpDn>>1);
#define _InitPinD6CN(CNI_EN)          _CNIED6 = (CNI_EN);
#else
#define _InitPinD6PUD(PullUpDn)
#define _InitPinD6CN(CNI_EN)
#endif
#ifdef _CNIED7
#define _InitPinD7PUD(PullUpDn)    _CNPUD7 = (PullUpDn&0b01); _CNPDD7 = (PullUpDn>>1);
#define _InitPinD7CN(CNI_EN)          _CNIED7 = (CNI_EN);
#else
#define _InitPinD7PUD(PullUpDn)
#define _InitPinD7CN(CNI_EN)
#endif
#ifdef _CNIED8
#define _InitPinD8PUD(PullUpDn)    _CNPUD8 = (PullUpDn&0b01); _CNPDD8 = (PullUpDn>>1);
#define _InitPinD8CN(CNI_EN)          _CNIED8 = (CNI_EN);
#else
#define _InitPinD8PUD(PullUpDn)
#define _InitPinD8CN(CNI_EN)
#endif
#ifdef _CNIED9
#define _InitPinD9PUD(PullUpDn)    _CNPUD9 = (PullUpDn&0b01); _CNPDD9 = (PullUpDn>>1);
#define _InitPinD9CN(CNI_EN)          _CNIED9 = (CNI_EN);
#else
#define _InitPinD9PUD(PullUpDn)
#define _InitPinD9CN(CNI_EN)
#endif
#ifdef _CNIED10
#define _InitPinD10PUD(PullUpDn)    _CNPUD10 = (PullUpDn&0b01); _CNPDD10 = (PullUpDn>>1);
#define _InitPinD10CN(CNI_EN)          _CNIED10 = (CNI_EN);
#else
#define _InitPinD10PUD(PullUpDn)
#define _InitPinD10CN(CNI_EN)
#endif
#ifdef _CNIED11
#define _InitPinD11PUD(PullUpDn)    _CNPUD11 = (PullUpDn&0b01); _CNPDD11 = (PullUpDn>>1);
#define _InitPinD11CN(CNI_EN)          _CNIED11 = (CNI_EN);
#else
#define _InitPinD11PUD(PullUpDn)
#define _InitPinD11CN(CNI_EN)
#endif
#ifdef _CNIED12
#define _InitPinD12PUD(PullUpDn)    _CNPUD12 = (PullUpDn&0b01); _CNPDD12 = (PullUpDn>>1);
#define _InitPinD12CN(CNI_EN)          _CNIED12 = (CNI_EN);
#else
#define _InitPinD12PUD(PullUpDn)
#define _InitPinD12CN(CNI_EN)
#endif
#ifdef _CNIED13
#define _InitPinD13PUD(PullUpDn)    _CNPUD13 = (PullUpDn&0b01); _CNPDD13 = (PullUpDn>>1);
#define _InitPinD13CN(CNI_EN)          _CNIED13 = (CNI_EN);
#else
#define _InitPinD13PUD(PullUpDn)
#define _InitPinD13CN(CNI_EN)
#endif
#ifdef _CNIED14
#define _InitPinD14PUD(PullUpDn)    _CNPUD14 = (PullUpDn&0b01); _CNPDD14 = (PullUpDn>>1);
#define _InitPinD14CN(CNI_EN)          _CNIED14 = (CNI_EN);
#else
#define _InitPinD14PUD(PullUpDn)
#define _InitPinD14CN(CNI_EN)
#endif
#ifdef _CNIED15
#define _InitPinD15PUD(PullUpDn)    _CNPUD15 = (PullUpDn&0b01); _CNPDD15 = (PullUpDn>>1);
#define _InitPinD15CN(CNI_EN)          _CNIED15 = (CNI_EN);
#else
#define _InitPinD15PUD(PullUpDn)
#define _InitPinD15CN(CNI_EN)
#endif

#ifdef _RD0
/***********************************************************************************************************************
 * \Function            void InitPinD0(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD0 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD0(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD0(PinMode,PullUpDn,CNI_EN)  ({_TRISD0 = ((PinMode>>1)&0b01); _InitPinD0AN(PinMode) _InitPinD0OD(PinMode) _InitPinD0PUD(PullUpDn) _InitPinD0CN(CNI_EN)})
#define ReadPinD0()           (PORTDbits.RD0)
#define SetPinD0(Value)     LATDbits.LATD0 = (Value)
#endif
#ifdef _RD1
/***********************************************************************************************************************
 * \Function            void InitPinD1(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD1 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD1(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD1(PinMode,PullUpDn,CNI_EN)  ({_TRISD1 = ((PinMode>>1)&0b01); _InitPinD1AN(PinMode)  _InitPinD1OD(PinMode) _InitPinD1PUD(PullUpDn) _InitPinD1CN(CNI_EN)})
#define ReadPinD1()           (PORTDbits.RD1)
#define SetPinD1(Value)     LATDbits.LATD1 = (Value)
#endif
#ifdef _RD2
/***********************************************************************************************************************
 * \Function            void InitPinD2(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD2 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD2(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD2(PinMode,PullUpDn,CNI_EN)  ({_TRISD2 = ((PinMode>>1)&0b01); _InitPinD2AN(PinMode)  _InitPinD2OD(PinMode) _InitPinD2PUD(PullUpDn) _InitPinD2CN(CNI_EN)})
#define ReadPinD2()           (PORTDbits.RD2)
#define SetPinD2(Value)     LATDbits.LATD2 = (Value)
#endif
#ifdef _RD3
/***********************************************************************************************************************
 * \Function            void InitPinD3(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD3 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD3(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD3(PinMode,PullUpDn,CNI_EN)  ({_TRISD3 = ((PinMode>>1)&0b01); _InitPinD3AN(PinMode)  _InitPinD3OD(PinMode) _InitPinD3PUD(PullUpDn) _InitPinD3CN(CNI_EN)})
#define ReadPinD3()           (PORTDbits.RD3)
#define SetPinD3(Value)     LATDbits.LATD3 = (Value)
#endif
#ifdef _RD4
/***********************************************************************************************************************
 * \Function            void InitPinD4(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD4 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD4(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD4(PinMode,PullUpDn,CNI_EN)  ({_TRISD4 = ((PinMode>>1)&0b01); _InitPinD4AN(PinMode)  _InitPinD4OD(PinMode) _InitPinD4PUD(PullUpDn) _InitPinD4CN(CNI_EN)})
#define ReadPinD4()           (PORTDbits.RD4)
#define SetPinD4(Value)     LATDbits.LATD4 = (Value)
#endif
#ifdef _RD5
/***********************************************************************************************************************
 * \Function            void InitPinD5(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD5 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD5(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD5(PinMode,PullUpDn,CNI_EN)  ({_TRISD5 = ((PinMode>>1)&0b01); _InitPinD5AN(PinMode)  _InitPinD5OD(PinMode) _InitPinD5PUD(PullUpDn) _InitPinD5CN(CNI_EN)})
#define ReadPinD5()           (PORTDbits.RD5)
#define SetPinD5(Value)     LATDbits.LATD5 = (Value)
#endif
#ifdef _RD6
/***********************************************************************************************************************
 * \Function            void InitPinD6(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD6 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD6(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD6(PinMode,PullUpDn,CNI_EN)  ({_TRISD6 = ((PinMode>>1)&0b01); _InitPinD6AN(PinMode)  _InitPinD6OD(PinMode) _InitPinD6PUD(PullUpDn) _InitPinD6CN(CNI_EN)})
#define ReadPinD6()           (PORTDbits.RD6)
#define SetPinD6(Value)     LATDbits.LATD6 = (Value)
#endif
#ifdef _RD7
/***********************************************************************************************************************
 * \Function            void InitPinD7(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD7 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD7(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD7(PinMode,PullUpDn,CNI_EN)  ({_TRISD7 = ((PinMode>>1)&0b01); _InitPinD7AN(PinMode)  _InitPinD7OD(PinMode) _InitPinD7PUD(PullUpDn) _InitPinD7CN(CNI_EN)})
#define ReadPinD7()           (PORTDbits.RD7)
#define SetPinD7(Value)     LATDbits.LATD7 = (Value)
#endif
#ifdef _RD8
/***********************************************************************************************************************
 * \Function            void InitPinD8(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD8 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD8(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD8(PinMode,PullUpDn,CNI_EN)  ({_TRISD8 = ((PinMode>>1)&0b01); _InitPinD8AN(PinMode)  _InitPinD8OD(PinMode) _InitPinD8PUD(PullUpDn) _InitPinD8CN(CNI_EN)})
#define ReadPinD8()           (PORTDbits.RD8)
#define SetPinD8(Value)     LATDbits.LATD8 = (Value)
#endif
#ifdef _RD9
/***********************************************************************************************************************
 * \Function            void InitPinD9(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD9 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD9(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD9(PinMode,PullUpDn,CNI_EN)  ({_TRISD9 = ((PinMode>>1)&0b01); _InitPinD9AN(PinMode)  _InitPinD9OD(PinMode) _InitPinD9PUD(PullUpDn) _InitPinD9CN(CNI_EN)})
#define ReadPinD9()           (PORTDbits.RD9)
#define SetPinD9(Value)     LATDbits.LATD9 = (Value)
#endif
#ifdef _RD10
/***********************************************************************************************************************
 * \Function            void InitPinD10(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD10 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD10(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD10(PinMode,PullUpDn,CNI_EN)  ({_TRISD10 = ((PinMode>>1)&0b01); _InitPinD10AN(PinMode)  _InitPinD10OD(PinMode) _InitPinD10PUD(PullUpDn) _InitPinD10CN(CNI_EN)})
#define ReadPinD10()          (PORTDbits.RD10)
#define SetPinD10(Value)     LATDbits.LATD10 = (Value)
#endif
#ifdef _RD11
/***********************************************************************************************************************
 * \Function            void InitPinD11(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD11 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD11(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD11(PinMode,PullUpDn,CNI_EN)  ({_TRISD11 = ((PinMode>>1)&0b01); _InitPinD11AN(PinMode)  _InitPinD11OD(PinMode) _InitPinD11PUD(PullUpDn) _InitPinD11CN(CNI_EN)})
#define ReadPinD11()          (PORTDbits.RD11)
#define SetPinD11(Value)     LATDbits.LATD11 = (Value)
#endif
#ifdef _RD12
/***********************************************************************************************************************
 * \Function            void InitPinD12(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD12 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD12(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD12(PinMode,PullUpDn,CNI_EN)  ({_TRISD12 = ((PinMode>>1)&0b01); _InitPinD12AN(PinMode)  _InitPinD12OD(PinMode) _InitPinD12PUD(PullUpDn) _InitPinD12CN(CNI_EN)})
#define ReadPinD12()          (PORTDbits.RD12)
#define SetPinD12(Value)     LATDbits.LATD12 = (Value)
#endif
#ifdef _RD13
/***********************************************************************************************************************
 * \Function            void InitPinD13(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD13 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD13(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD13(PinMode,PullUpDn,CNI_EN)  ({_TRISD13 = ((PinMode>>1)&0b01); _InitPinD13AN(PinMode)  _InitPinD13OD(PinMode) _InitPinD13PUD(PullUpDn) _InitPinD13CN(CNI_EN)})
#define ReadPinD13()          (PORTDbits.RD13)
#define SetPinD13(Value)     LATDbits.LATD13 = (Value)
#endif
#ifdef _RD14
/***********************************************************************************************************************
 * \Function            void InitPinD14(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD14 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD14(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD14(PinMode,PullUpDn,CNI_EN)  ({_TRISD14 = ((PinMode>>1)&0b01); _InitPinD14AN(PinMode)  _InitPinD14OD(PinMode) _InitPinD14PUD(PullUpDn) _InitPinD14CN(CNI_EN)})
#define ReadPinD14()          (PORTDbits.RD14)
#define SetPinD14(Value)     LATDbits.LATD14 = (Value)
#endif
#ifdef _RD15
/***********************************************************************************************************************
 * \Function            void InitPinD15(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTD.RD15 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinD15(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinD15(PinMode,PullUpDn,CNI_EN)  ({_TRISD15 = ((PinMode>>1)&0b01); _InitPinD15AN(PinMode)  _InitPinD15OD(PinMode) _InitPinD15PUD(PullUpDn) _InitPinD15CN(CNI_EN)})
#define ReadPinD15()          (PORTDbits.RD15)
#define SetPinD15(Value)     LATDbits.LATD15 = (Value)
#endif

#endif
#ifdef  PORTE

#define ReadPORTE()           (PORTE)
#define SetPORTE(Value)     LATE = (Value)

#ifdef ANSELE
/***********************************************************************************************************************
 * \Function            void InitAnalogPinsPortE(AnalogPins)
 *
 * \Description         Set PortE Individual Pins Mode (Analog OR Digital).
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>AnalogPins:</u>\n
 *  - RE0AN to RE15AN   <i>/ The Corresponding Pin is Analog Pin.</i>   \n
 *  - All_PINS_DIG      <i>/ All PortE Pins is Digital.</i>   \n
 *  - All_PINS_AN       <i>/ All PortE Pins is Analog.</i>
 *
 * \Return      None
 *
 * \Notes       None
 *
 * \Example     InitAnalogPinsPortE(All_PINS_DIG & RE6AN & RE7AN); \n
 * \Example     InitAnalogPinsPortG(All_PINS_AN);
 *
 **********************************************************************************************************************/
#define InitAnalogPinsPortE(AnalogPins)    ANSELE = ~(AnalogPins)
#endif
#ifdef _ANSE0
#define RE0AN	0b1111111111111110	/* RE0 bin is Analog bin */
#define _InitPinE0AN(PinMode)    _ANSE0 = (PinMode>>2);
#else
#define _InitPinE0AN(PinMode)
#endif
#ifdef _ANSE1
#define RE1AN	0b1111111111111101	/* RE1 bin is Analog bin */
#define _InitPinE1AN(PinMode)    _ANSE1 = (PinMode>>2);
#else
#define _InitPinE1AN(PinMode)
#endif
#ifdef _ANSE2
#define RE2AN	0b1111111111111011	/* RE2 bin is Analog bin */
#define _InitPinE2AN(PinMode)    _ANSE2 = (PinMode>>2);
#else
#define _InitPinE2AN(PinMode)
#endif
#ifdef _ANSE3
#define RE3AN	0b1111111111110111	/* RE3 bin is Analog bin */
#define _InitPinE3AN(PinMode)    _ANSE3 = (PinMode>>2);
#else
#define _InitPinE3AN(PinMode)
#endif
#ifdef _ANSE4
#define RE4AN	0b1111111111101111	/* RE4 bin is Analog bin */
#define _InitPinE4AN(PinMode)    _ANSE4 = (PinMode>>2);
#else
#define _InitPinE4AN(PinMode)
#endif
#ifdef _ANSE5
#define RE5AN	0b1111111111011111	/* RE5 bin is Analog bin */
#define _InitPinE5AN(PinMode)    _ANSE5 = (PinMode>>2);
#else
#define _InitPinE5AN(PinMode)
#endif
#ifdef _ANSE6
#define RE6AN	0b1111111110111111	/* RE6 bin is Analog bin */
#define _InitPinE6AN(PinMode)    _ANSE6 = (PinMode>>2);
#else
#define _InitPinE6AN(PinMode)
#endif
#ifdef _ANSE7
#define RE7AN	0b1111111101111111	/* RE7 bin is Analog bin */
#define _InitPinE7AN(PinMode)    _ANSE7 = (PinMode>>2);
#else
#define _InitPinE7AN(PinMode)
#endif
#ifdef _ANSE8
#define RE8AN	0b1111111011111111	/* RE8 bin is Analog bin */
#define _InitPinE8AN(PinMode)    _ANSE8 = (PinMode>>2);
#else
#define _InitPinE8AN(PinMode)
#endif
#ifdef _ANSE9
#define RE9AN	0b1111110111111111	/* RE9 bin is Analog bin */
#define _InitPinE9AN(PinMode)    _ANSE9 = (PinMode>>2);
#else
#define _InitPinE9AN(PinMode)
#endif
#ifdef _ANSE10
#define RE10AN	0b1111101111111111	/* RE10 bin is Analog bin */
#define _InitPinE10AN(PinMode)    _ANSE10 = (PinMode>>2);
#else
#define _InitPinE10AN(PinMode)
#endif
#ifdef _ANSE11
#define RE11AN	0b1111011111111111	/* RE11 bin is Analog bin */
#define _InitPinE11AN(PinMode)    _ANSE11 = (PinMode>>2);
#else
#define _InitPinE11AN(PinMode)
#endif
#ifdef _ANSE12
#define RE12AN	0b1110111111111111	/* RE12 bin is Analog bin */
#define _InitPinE12AN(PinMode)    _ANSE12 = (PinMode>>2);
#else
#define _InitPinE12AN(PinMode)
#endif
#ifdef _ANSE13
#define RE13AN	0b1101111111111111	/* RE13 bin is Analog bin */
#define _InitPinE13AN(PinMode)    _ANSE13 = (PinMode>>2);
#else
#define _InitPinE13AN(PinMode)
#endif
#ifdef _ANSE14
#define RE14AN	0b1011111111111111	/* RE14 bin is Analog bin */
#define _InitPinE14AN(PinMode)    _ANSE14 = (PinMode>>2);
#else
#define _InitPinE14AN(PinMode)
#endif
#ifdef _ANSE15
#define RE15AN	0b0111111111111111	/* RE15 bin is Analog bin */
#define _InitPinE15AN(PinMode)    _ANSE15 = (PinMode>>2);
#else
#define _InitPinE15AN(PinMode)
#endif

#ifdef _ODCE0
#define _InitPinE0OD(PinMode)    _ODCE0 = (PinMode&0b001);
#else
#define _InitPinE0OD(PinMode)
#endif
#ifdef _ODCE1
#define _InitPinE1OD(PinMode)    _ODCE1 = (PinMode&0b001);
#else
#define _InitPinE1OD(PinMode)
#endif
#ifdef _ODCE2
#define _InitPinE2OD(PinMode)    _ODCE2 = (PinMode&0b001);
#else
#define _InitPinE2OD(PinMode)
#endif
#ifdef _ODCE3
#define _InitPinE3OD(PinMode)    _ODCE3 = (PinMode&0b001);
#else
#define _InitPinE3OD(PinMode)
#endif
#ifdef _ODCE4
#define _InitPinE4OD(PinMode)    _ODCE4 = (PinMode&0b001);
#else
#define _InitPinE4OD(PinMode)
#endif
#ifdef _ODCE5
#define _InitPinE5OD(PinMode)    _ODCE5 = (PinMode&0b001);
#else
#define _InitPinE5OD(PinMode)
#endif
#ifdef _ODCE6
#define _InitPinE6OD(PinMode)    _ODCE6 = (PinMode&0b001);
#else
#define _InitPinE6OD(PinMode)
#endif
#ifdef _ODCE7
#define _InitPinE7OD(PinMode)    _ODCE7 = (PinMode&0b001);
#else
#define _InitPinE7OD(PinMode)
#endif
#ifdef _ODCE8
#define _InitPinE8OD(PinMode)    _ODCE8 = (PinMode&0b001);
#else
#define _InitPinE8OD(PinMode)
#endif
#ifdef _ODCE9
#define _InitPinE9OD(PinMode)    _ODCE9 = (PinMode&0b001);
#else
#define _InitPinE9OD(PinMode)
#endif
#ifdef _ODCE10
#define _InitPinE10OD(PinMode)    _ODCE10 = (PinMode&0b001);
#else
#define _InitPinE10OD(PinMode)
#endif
#ifdef _ODCE11
#define _InitPinE11OD(PinMode)    _ODCE11 = (PinMode&0b001);
#else
#define _InitPinE11OD(PinMode)
#endif
#ifdef _ODCE12
#define _InitPinE12OD(PinMode)    _ODCE12 = (PinMode&0b001);
#else
#define _InitPinE12OD(PinMode)
#endif
#ifdef _ODCE13
#define _InitPinE13OD(PinMode)    _ODCE13 = (PinMode&0b001);
#else
#define _InitPinE13OD(PinMode)
#endif
#ifdef _ODCE14
#define _InitPinE14OD(PinMode)    _ODCE14 = (PinMode&0b001);
#else
#define _InitPinE14OD(PinMode)
#endif
#ifdef _ODCE15
#define _InitPinE15OD(PinMode)    _ODCE15 = (PinMode&0b001);
#else
#define _InitPinE15OD(PinMode)
#endif

#ifdef _CNIEE0
#define _InitPinE0PUD(PullUpDn)    _CNPUE0 = (PullUpDn&0b01); _CNPDE0 = (PullUpDn>>1);
#define _InitPinE0CN(CNI_EN)          _CNIEE0 = (CNI_EN);
#else
#define _InitPinE0PUD(PullUpDn)
#define _InitPinE0CN(CNI_EN)
#endif
#ifdef _CNIEE1
#define _InitPinE1PUD(PullUpDn)    _CNPUE1 = (PullUpDn&0b01); _CNPDE1 = (PullUpDn>>1);
#define _InitPinE1CN(CNI_EN)          _CNIEE1 = (CNI_EN);
#else
#define _InitPinE1PUD(PullUpDn)
#define _InitPinE1CN(CNI_EN)
#endif
#ifdef _CNIEE2
#define _InitPinE2PUD(PullUpDn)    _CNPUE2 = (PullUpDn&0b01); _CNPDE2 = (PullUpDn>>1);
#define _InitPinE2CN(CNI_EN)          _CNIEE2 = (CNI_EN);
#else
#define _InitPinE2PUD(PullUpDn)
#define _InitPinE2CN(CNI_EN)
#endif
#ifdef _CNIEE3
#define _InitPinE3PUD(PullUpDn)    _CNPUE3 = (PullUpDn&0b01); _CNPDE3 = (PullUpDn>>1);
#define _InitPinE3CN(CNI_EN)          _CNIEE3 = (CNI_EN);
#else
#define _InitPinE3PUD(PullUpDn)
#define _InitPinE3CN(CNI_EN)
#endif
#ifdef _CNIEE4
#define _InitPinE4PUD(PullUpDn)    _CNPUE4 = (PullUpDn&0b01); _CNPDE4 = (PullUpDn>>1);
#define _InitPinE4CN(CNI_EN)          _CNIEE4 = (CNI_EN);
#else
#define _InitPinE4PUD(PullUpDn)
#define _InitPinE4CN(CNI_EN)
#endif
#ifdef _CNIEE5
#define _InitPinE5PUD(PullUpDn)    _CNPUE5 = (PullUpDn&0b01); _CNPDE5 = (PullUpDn>>1);
#define _InitPinE5CN(CNI_EN)          _CNIEE5 = (CNI_EN);
#else
#define _InitPinE5PUD(PullUpDn)
#define _InitPinE5CN(CNI_EN)
#endif
#ifdef _CNIEE6
#define _InitPinE6PUD(PullUpDn)    _CNPUE6 = (PullUpDn&0b01); _CNPDE6 = (PullUpDn>>1);
#define _InitPinE6CN(CNI_EN)          _CNIEE6 = (CNI_EN);
#else
#define _InitPinE6PUD(PullUpDn)
#define _InitPinE6CN(CNI_EN)
#endif
#ifdef _CNIEE7
#define _InitPinE7PUD(PullUpDn)    _CNPUE7 = (PullUpDn&0b01); _CNPDE7 = (PullUpDn>>1);
#define _InitPinE7CN(CNI_EN)          _CNIEE7 = (CNI_EN);
#else
#define _InitPinE7PUD(PullUpDn)
#define _InitPinE7CN(CNI_EN)
#endif
#ifdef _CNIEE8
#define _InitPinE8PUD(PullUpDn)    _CNPUE8 = (PullUpDn&0b01); _CNPDE8 = (PullUpDn>>1);
#define _InitPinE8CN(CNI_EN)          _CNIEE8 = (CNI_EN);
#else
#define _InitPinE8PUD(PullUpDn)
#define _InitPinE8CN(CNI_EN)
#endif
#ifdef _CNIEE9
#define _InitPinE9PUD(PullUpDn)    _CNPUE9 = (PullUpDn&0b01); _CNPDE9 = (PullUpDn>>1);
#define _InitPinE9CN(CNI_EN)          _CNIEE9 = (CNI_EN);
#else
#define _InitPinE9PUD(PullUpDn)
#define _InitPinE9CN(CNI_EN)
#endif
#ifdef _CNIEE10
#define _InitPinE10PUD(PullUpDn)    _CNPUE10 = (PullUpDn&0b01); _CNPDE10 = (PullUpDn>>1);
#define _InitPinE10CN(CNI_EN)          _CNIEE10 = (CNI_EN);
#else
#define _InitPinE10PUD(PullUpDn)
#define _InitPinE10CN(CNI_EN)
#endif
#ifdef _CNIEE11
#define _InitPinE11PUD(PullUpDn)    _CNPUE11 = (PullUpDn&0b01); _CNPDE11 = (PullUpDn>>1);
#define _InitPinE11CN(CNI_EN)          _CNIEE11 = (CNI_EN);
#else
#define _InitPinE11PUD(PullUpDn)
#define _InitPinE11CN(CNI_EN)
#endif
#ifdef _CNIEE12
#define _InitPinE12PUD(PullUpDn)    _CNPUE12 = (PullUpDn&0b01); _CNPDE12 = (PullUpDn>>1);
#define _InitPinE12CN(CNI_EN)          _CNIEE12 = (CNI_EN);
#else
#define _InitPinE12PUD(PullUpDn)
#define _InitPinE12CN(CNI_EN)
#endif
#ifdef _CNIEE13
#define _InitPinE13PUD(PullUpDn)    _CNPUE13 = (PullUpDn&0b01); _CNPDE13 = (PullUpDn>>1);
#define _InitPinE13CN(CNI_EN)          _CNIEE13 = (CNI_EN);
#else
#define _InitPinE13PUD(PullUpDn)
#define _InitPinE13CN(CNI_EN)
#endif
#ifdef _CNIEE14
#define _InitPinE14PUD(PullUpDn)    _CNPUE14 = (PullUpDn&0b01); _CNPDE14 = (PullUpDn>>1);
#define _InitPinE14CN(CNI_EN)          _CNIEE14 = (CNI_EN);
#else
#define _InitPinE14PUD(PullUpDn)
#define _InitPinE14CN(CNI_EN)
#endif
#ifdef _CNIEE15
#define _InitPinE15PUD(PullUpDn)    _CNPUE15 = (PullUpDn&0b01); _CNPDE15 = (PullUpDn>>1);
#define _InitPinE15CN(CNI_EN)          _CNIEE15 = (CNI_EN);
#else
#define _InitPinE15PUD(PullUpDn)
#define _InitPinE15CN(CNI_EN)
#endif

#ifdef _RE0
/***********************************************************************************************************************
 * \Function            void InitPinE0(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE0 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE0(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE0(PinMode,PullUpDn,CNI_EN)  ({_TRISE0 = ((PinMode>>1)&0b01); _InitPinE0AN(PinMode) _InitPinE0OD(PinMode) _InitPinE0PUD(PullUpDn) _InitPinE0CN(CNI_EN)})
#define ReadPinE0()           (PORTEbits.RE0)
#define SetPinE0(Value)     LATEbits.LATE0 = (Value)
#endif
#ifdef _RE1
/***********************************************************************************************************************
 * \Function            void InitPinE1(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE1 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE1(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE1(PinMode,PullUpDn,CNI_EN)  ({_TRISE1 = ((PinMode>>1)&0b01); _InitPinE1AN(PinMode)  _InitPinE1OD(PinMode) _InitPinE1PUD(PullUpDn) _InitPinE1CN(CNI_EN)})
#define ReadPinE1()           (PORTEbits.RE1)
#define SetPinE1(Value)     LATEbits.LATE1 = (Value)
#endif
#ifdef _RE2
/***********************************************************************************************************************
 * \Function            void InitPinE2(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE2 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE2(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE2(PinMode,PullUpDn,CNI_EN)  ({_TRISE2 = ((PinMode>>1)&0b01); _InitPinE2AN(PinMode)  _InitPinE2OD(PinMode) _InitPinE2PUD(PullUpDn) _InitPinE2CN(CNI_EN)})
#define ReadPinE2()           (PORTEbits.RE2)
#define SetPinE2(Value)     LATEbits.LATE2 = (Value)
#endif
#ifdef _RE3
/***********************************************************************************************************************
 * \Function            void InitPinE3(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE3 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE3(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE3(PinMode,PullUpDn,CNI_EN)  ({_TRISE3 = ((PinMode>>1)&0b01); _InitPinE3AN(PinMode)  _InitPinE3OD(PinMode) _InitPinE3PUD(PullUpDn) _InitPinE3CN(CNI_EN)})
#define ReadPinE3()           (PORTEbits.RE3)
#define SetPinE3(Value)     LATEbits.LATE3 = (Value)
#endif
#ifdef _RE4
/***********************************************************************************************************************
 * \Function            void InitPinE4(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE4 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE4(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE4(PinMode,PullUpDn,CNI_EN)  ({_TRISE4 = ((PinMode>>1)&0b01); _InitPinE4AN(PinMode)  _InitPinE4OD(PinMode) _InitPinE4PUD(PullUpDn) _InitPinE4CN(CNI_EN)})
#define ReadPinE4()           (PORTEbits.RE4)
#define SetPinE4(Value)     LATEbits.LATE4 = (Value)
#endif
#ifdef _RE5
/***********************************************************************************************************************
 * \Function            void InitPinE5(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE5 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE5(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE5(PinMode,PullUpDn,CNI_EN)  ({_TRISE5 = ((PinMode>>1)&0b01); _InitPinE5AN(PinMode)  _InitPinE5OD(PinMode) _InitPinE5PUD(PullUpDn) _InitPinE5CN(CNI_EN)})
#define ReadPinE5()           (PORTEbits.RE5)
#define SetPinE5(Value)     LATEbits.LATE5 = (Value)
#endif
#ifdef _RE6
/***********************************************************************************************************************
 * \Function            void InitPinE6(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE6 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE6(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE6(PinMode,PullUpDn,CNI_EN)  ({_TRISE6 = ((PinMode>>1)&0b01); _InitPinE6AN(PinMode)  _InitPinE6OD(PinMode) _InitPinE6PUD(PullUpDn) _InitPinE6CN(CNI_EN)})
#define ReadPinE6()           (PORTEbits.RE6)
#define SetPinE6(Value)     LATEbits.LATE6 = (Value)
#endif
#ifdef _RE7
/***********************************************************************************************************************
 * \Function            void InitPinE7(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE7 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE7(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE7(PinMode,PullUpDn,CNI_EN)  ({_TRISE7 = ((PinMode>>1)&0b01); _InitPinE7AN(PinMode)  _InitPinE7OD(PinMode) _InitPinE7PUD(PullUpDn) _InitPinE7CN(CNI_EN)})
#define ReadPinE7()           (PORTEbits.RE7)
#define SetPinE7(Value)     LATEbits.LATE7 = (Value)
#endif
#ifdef _RE8
/***********************************************************************************************************************
 * \Function            void InitPinE8(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE8 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE8(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE8(PinMode,PullUpDn,CNI_EN)  ({_TRISE8 = ((PinMode>>1)&0b01); _InitPinE8AN(PinMode)  _InitPinE8OD(PinMode) _InitPinE8PUD(PullUpDn) _InitPinE8CN(CNI_EN)})
#define ReadPinE8()           (PORTEbits.RE8)
#define SetPinE8(Value)     LATEbits.LATE8 = (Value)
#endif
#ifdef _RE9
/***********************************************************************************************************************
 * \Function            void InitPinE9(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE9 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE9(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE9(PinMode,PullUpDn,CNI_EN)  ({_TRISE9 = ((PinMode>>1)&0b01); _InitPinE9AN(PinMode)  _InitPinE9OD(PinMode) _InitPinE9PUD(PullUpDn) _InitPinE9CN(CNI_EN)})
#define ReadPinE9()           (PORTEbits.RE9)
#define SetPinE9(Value)     LATEbits.LATE9 = (Value)
#endif
#ifdef _RE10
/***********************************************************************************************************************
 * \Function            void InitPinE10(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE10 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE10(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE10(PinMode,PullUpDn,CNI_EN)  ({_TRISE10 = ((PinMode>>1)&0b01); _InitPinE10AN(PinMode)  _InitPinE10OD(PinMode) _InitPinE10PUD(PullUpDn) _InitPinE10CN(CNI_EN)})
#define ReadPinE10()          (PORTEbits.RE10)
#define SetPinE10(Value)     LATEbits.LATE10 = (Value)
#endif
#ifdef _RE11
/***********************************************************************************************************************
 * \Function            void InitPinE11(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE11 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE11(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE11(PinMode,PullUpDn,CNI_EN)  ({_TRISE11 = ((PinMode>>1)&0b01); _InitPinE11AN(PinMode)  _InitPinE11OD(PinMode) _InitPinE11PUD(PullUpDn) _InitPinE11CN(CNI_EN)})
#define ReadPinE11()          (PORTEbits.RE11)
#define SetPinE11(Value)     LATEbits.LATE11 = (Value)
#endif
#ifdef _RE12
/***********************************************************************************************************************
 * \Function            void InitPinE12(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE12 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE12(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE12(PinMode,PullUpDn,CNI_EN)  ({_TRISE12 = ((PinMode>>1)&0b01); _InitPinE12AN(PinMode)  _InitPinE12OD(PinMode) _InitPinE12PUD(PullUpDn) _InitPinE12CN(CNI_EN)})
#define ReadPinE12()          (PORTEbits.RE12)
#define SetPinE12(Value)     LATEbits.LATE12 = (Value)
#endif
#ifdef _RE13
/***********************************************************************************************************************
 * \Function            void InitPinE13(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE13 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE13(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE13(PinMode,PullUpDn,CNI_EN)  ({_TRISE13 = ((PinMode>>1)&0b01); _InitPinE13AN(PinMode)  _InitPinE13OD(PinMode) _InitPinE13PUD(PullUpDn) _InitPinE13CN(CNI_EN)})
#define ReadPinE13()          (PORTEbits.RE13)
#define SetPinE13(Value)     LATEbits.LATE13 = (Value)
#endif
#ifdef _RE14
/***********************************************************************************************************************
 * \Function            void InitPinE14(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE14 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE14(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE14(PinMode,PullUpDn,CNI_EN)  ({_TRISE14 = ((PinMode>>1)&0b01); _InitPinE14AN(PinMode)  _InitPinE14OD(PinMode) _InitPinE14PUD(PullUpDn) _InitPinE14CN(CNI_EN)})
#define ReadPinE14()          (PORTEbits.RE14)
#define SetPinE14(Value)     LATEbits.LATE14 = (Value)
#endif
#ifdef _RE15
/***********************************************************************************************************************
 * \Function            void InitPinE15(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTE.RE15 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinE15(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinE15(PinMode,PullUpDn,CNI_EN)  ({_TRISE15 = ((PinMode>>1)&0b01); _InitPinE15AN(PinMode)  _InitPinE15OD(PinMode) _InitPinE15PUD(PullUpDn) _InitPinE15CN(CNI_EN)})
#define ReadPinE15()          (PORTEbits.RE15)
#define SetPinE15(Value)     LATEbits.LATE15 = (Value)
#endif

#endif
#ifdef  PORTF

#define ReadPORTF()           (PORTF)
#define SetPORTF(Value)     LATF = (Value)

#ifdef ANSELF
/***********************************************************************************************************************
 * \Function            void InitAnalogPinsPortF(AnalogPins)
 *
 * \Description         Set PortF Individual Pins Mode (Analog OR Digital).
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>AnalogPins:</u>\n
 *  - RF0AN to RF15AN   <i>/ The Corresponding Pin is Analog Pin.</i>   \n
 *  - All_PINS_DIG      <i>/ All PortF Pins is Digital.</i>   \n
 *  - All_PINS_AN       <i>/ All PortF Pins is Analog.</i>
 *
 * \Return      None
 *
 * \Notes       None
 *
 * \Example     InitAnalogPinsPortF(All_PINS_DIG & RF6AN & RF7AN); \n
 * \Example     InitAnalogPinsPortG(All_PINS_AN);
 *
 **********************************************************************************************************************/
#define InitAnalogPinsPortF(AnalogPins)    ANSELF = ~(AnalogPins)
#endif
#ifdef _ANSF0
#define RF0AN	0b1111111111111110	/* RF0 bin is Analog bin */
#define _InitPinF0AN(PinMode)    _ANSF0 = (PinMode>>2);
#else
#define _InitPinF0AN(PinMode)
#endif
#ifdef _ANSF1
#define RF1AN	0b1111111111111101	/* RF1 bin is Analog bin */
#define _InitPinF1AN(PinMode)    _ANSF1 = (PinMode>>2);
#else
#define _InitPinF1AN(PinMode)
#endif
#ifdef _ANSF2
#define RF2AN	0b1111111111111011	/* RF2 bin is Analog bin */
#define _InitPinF2AN(PinMode)    _ANSF2 = (PinMode>>2);
#else
#define _InitPinF2AN(PinMode)
#endif
#ifdef _ANSF3
#define RF3AN	0b1111111111110111	/* RF3 bin is Analog bin */
#define _InitPinF3AN(PinMode)    _ANSF3 = (PinMode>>2);
#else
#define _InitPinF3AN(PinMode)
#endif
#ifdef _ANSF4
#define RF4AN	0b1111111111101111	/* RF4 bin is Analog bin */
#define _InitPinF4AN(PinMode)    _ANSF4 = (PinMode>>2);
#else
#define _InitPinF4AN(PinMode)
#endif
#ifdef _ANSF5
#define RF5AN	0b1111111111011111	/* RF5 bin is Analog bin */
#define _InitPinF5AN(PinMode)    _ANSF5 = (PinMode>>2);
#else
#define _InitPinF5AN(PinMode)
#endif
#ifdef _ANSF6
#define RF6AN	0b1111111110111111	/* RF6 bin is Analog bin */
#define _InitPinF6AN(PinMode)    _ANSF6 = (PinMode>>2);
#else
#define _InitPinF6AN(PinMode)
#endif
#ifdef _ANSF7
#define RF7AN	0b1111111101111111	/* RF7 bin is Analog bin */
#define _InitPinF7AN(PinMode)    _ANSF7 = (PinMode>>2);
#else
#define _InitPinF7AN(PinMode)
#endif
#ifdef _ANSF8
#define RF8AN	0b1111111011111111	/* RF8 bin is Analog bin */
#define _InitPinF8AN(PinMode)    _ANSF8 = (PinMode>>2);
#else
#define _InitPinF8AN(PinMode)
#endif
#ifdef _ANSF9
#define RF9AN	0b1111110111111111	/* RF9 bin is Analog bin */
#define _InitPinF9AN(PinMode)    _ANSF9 = (PinMode>>2);
#else
#define _InitPinF9AN(PinMode)
#endif
#ifdef _ANSF10
#define RF10AN	0b1111101111111111	/* RF10 bin is Analog bin */
#define _InitPinF10AN(PinMode)    _ANSF10 = (PinMode>>2);
#else
#define _InitPinF10AN(PinMode)
#endif
#ifdef _ANSF11
#define RF11AN	0b1111011111111111	/* RF11 bin is Analog bin */
#define _InitPinF11AN(PinMode)    _ANSF11 = (PinMode>>2);
#else
#define _InitPinF11AN(PinMode)
#endif
#ifdef _ANSF12
#define RF12AN	0b1110111111111111	/* RF12 bin is Analog bin */
#define _InitPinF12AN(PinMode)    _ANSF12 = (PinMode>>2);
#else
#define _InitPinF12AN(PinMode)
#endif
#ifdef _ANSF13
#define RF13AN	0b1101111111111111	/* RF13 bin is Analog bin */
#define _InitPinF13AN(PinMode)    _ANSF13 = (PinMode>>2);
#else
#define _InitPinF13AN(PinMode)
#endif
#ifdef _ANSF14
#define RF14AN	0b1011111111111111	/* RF14 bin is Analog bin */
#define _InitPinF14AN(PinMode)    _ANSF14 = (PinMode>>2);
#else
#define _InitPinF14AN(PinMode)
#endif
#ifdef _ANSF15
#define RF15AN	0b0111111111111111	/* RF15 bin is Analog bin */
#define _InitPinF15AN(PinMode)    _ANSF15 = (PinMode>>2);
#else
#define _InitPinF15AN(PinMode)
#endif

#ifdef _ODCF0
#define _InitPinF0OD(PinMode)    _ODCF0 = (PinMode&0b001);
#else
#define _InitPinF0OD(PinMode)
#endif
#ifdef _ODCF1
#define _InitPinF1OD(PinMode)    _ODCF1 = (PinMode&0b001);
#else
#define _InitPinF1OD(PinMode)
#endif
#ifdef _ODCF2
#define _InitPinF2OD(PinMode)    _ODCF2 = (PinMode&0b001);
#else
#define _InitPinF2OD(PinMode)
#endif
#ifdef _ODCF3
#define _InitPinF3OD(PinMode)    _ODCF3 = (PinMode&0b001);
#else
#define _InitPinF3OD(PinMode)
#endif
#ifdef _ODCF4
#define _InitPinF4OD(PinMode)    _ODCF4 = (PinMode&0b001);
#else
#define _InitPinF4OD(PinMode)
#endif
#ifdef _ODCF5
#define _InitPinF5OD(PinMode)    _ODCF5 = (PinMode&0b001);
#else
#define _InitPinF5OD(PinMode)
#endif
#ifdef _ODCF6
#define _InitPinF6OD(PinMode)    _ODCF6 = (PinMode&0b001);
#else
#define _InitPinF6OD(PinMode)
#endif
#ifdef _ODCF7
#define _InitPinF7OD(PinMode)    _ODCF7 = (PinMode&0b001);
#else
#define _InitPinF7OD(PinMode)
#endif
#ifdef _ODCF8
#define _InitPinF8OD(PinMode)    _ODCF8 = (PinMode&0b001);
#else
#define _InitPinF8OD(PinMode)
#endif
#ifdef _ODCF9
#define _InitPinF9OD(PinMode)    _ODCF9 = (PinMode&0b001);
#else
#define _InitPinF9OD(PinMode)
#endif
#ifdef _ODCF10
#define _InitPinF10OD(PinMode)    _ODCF10 = (PinMode&0b001);
#else
#define _InitPinF10OD(PinMode)
#endif
#ifdef _ODCF11
#define _InitPinF11OD(PinMode)    _ODCF11 = (PinMode&0b001);
#else
#define _InitPinF11OD(PinMode)
#endif
#ifdef _ODCF12
#define _InitPinF12OD(PinMode)    _ODCF12 = (PinMode&0b001);
#else
#define _InitPinF12OD(PinMode)
#endif
#ifdef _ODCF13
#define _InitPinF13OD(PinMode)    _ODCF13 = (PinMode&0b001);
#else
#define _InitPinF13OD(PinMode)
#endif
#ifdef _ODCF14
#define _InitPinF14OD(PinMode)    _ODCF14 = (PinMode&0b001);
#else
#define _InitPinF14OD(PinMode)
#endif
#ifdef _ODCF15
#define _InitPinF15OD(PinMode)    _ODCF15 = (PinMode&0b001);
#else
#define _InitPinF15OD(PinMode)
#endif

#ifdef _CNIEF0
#define _InitPinF0PUD(PullUpDn)    _CNPUF0 = (PullUpDn&0b01); _CNPDF0 = (PullUpDn>>1);
#define _InitPinF0CN(CNI_EN)          _CNIEF0 = (CNI_EN);
#else
#define _InitPinF0PUD(PullUpDn)
#define _InitPinF0CN(CNI_EN)
#endif
#ifdef _CNIEF1
#define _InitPinF1PUD(PullUpDn)    _CNPUF1 = (PullUpDn&0b01); _CNPDF1 = (PullUpDn>>1);
#define _InitPinF1CN(CNI_EN)          _CNIEF1 = (CNI_EN);
#else
#define _InitPinF1PUD(PullUpDn)
#define _InitPinF1CN(CNI_EN)
#endif
#ifdef _CNIEF2
#define _InitPinF2PUD(PullUpDn)    _CNPUF2 = (PullUpDn&0b01); _CNPDF2 = (PullUpDn>>1);
#define _InitPinF2CN(CNI_EN)          _CNIEF2 = (CNI_EN);
#else
#define _InitPinF2PUD(PullUpDn)
#define _InitPinF2CN(CNI_EN)
#endif
#ifdef _CNIEF3
#define _InitPinF3PUD(PullUpDn)    _CNPUF3 = (PullUpDn&0b01); _CNPDF3 = (PullUpDn>>1);
#define _InitPinF3CN(CNI_EN)          _CNIEF3 = (CNI_EN);
#else
#define _InitPinF3PUD(PullUpDn)
#define _InitPinF3CN(CNI_EN)
#endif
#ifdef _CNIEF4
#define _InitPinF4PUD(PullUpDn)    _CNPUF4 = (PullUpDn&0b01); _CNPDF4 = (PullUpDn>>1);
#define _InitPinF4CN(CNI_EN)          _CNIEF4 = (CNI_EN);
#else
#define _InitPinF4PUD(PullUpDn)
#define _InitPinF4CN(CNI_EN)
#endif
#ifdef _CNIEF5
#define _InitPinF5PUD(PullUpDn)    _CNPUF5 = (PullUpDn&0b01); _CNPDF5 = (PullUpDn>>1);
#define _InitPinF5CN(CNI_EN)          _CNIEF5 = (CNI_EN);
#else
#define _InitPinF5PUD(PullUpDn)
#define _InitPinF5CN(CNI_EN)
#endif
#ifdef _CNIEF6
#define _InitPinF6PUD(PullUpDn)    _CNPUF6 = (PullUpDn&0b01); _CNPDF6 = (PullUpDn>>1);
#define _InitPinF6CN(CNI_EN)          _CNIEF6 = (CNI_EN);
#else
#define _InitPinF6PUD(PullUpDn)
#define _InitPinF6CN(CNI_EN)
#endif
#ifdef _CNIEF7
#define _InitPinF7PUD(PullUpDn)    _CNPUF7 = (PullUpDn&0b01); _CNPDF7 = (PullUpDn>>1);
#define _InitPinF7CN(CNI_EN)          _CNIEF7 = (CNI_EN);
#else
#define _InitPinF7PUD(PullUpDn)
#define _InitPinF7CN(CNI_EN)
#endif
#ifdef _CNIEF8
#define _InitPinF8PUD(PullUpDn)    _CNPUF8 = (PullUpDn&0b01); _CNPDF8 = (PullUpDn>>1);
#define _InitPinF8CN(CNI_EN)          _CNIEF8 = (CNI_EN);
#else
#define _InitPinF8PUD(PullUpDn)
#define _InitPinF8CN(CNI_EN)
#endif
#ifdef _CNIEF9
#define _InitPinF9PUD(PullUpDn)    _CNPUF9 = (PullUpDn&0b01); _CNPDF9 = (PullUpDn>>1);
#define _InitPinF9CN(CNI_EN)          _CNIEF9 = (CNI_EN);
#else
#define _InitPinF9PUD(PullUpDn)
#define _InitPinF9CN(CNI_EN)
#endif
#ifdef _CNIEF10
#define _InitPinF10PUD(PullUpDn)    _CNPUF10 = (PullUpDn&0b01); _CNPDF10 = (PullUpDn>>1);
#define _InitPinF10CN(CNI_EN)          _CNIEF10 = (CNI_EN);
#else
#define _InitPinF10PUD(PullUpDn)
#define _InitPinF10CN(CNI_EN)
#endif
#ifdef _CNIEF11
#define _InitPinF11PUD(PullUpDn)    _CNPUF11 = (PullUpDn&0b01); _CNPDF11 = (PullUpDn>>1);
#define _InitPinF11CN(CNI_EN)          _CNIEF11 = (CNI_EN);
#else
#define _InitPinF11PUD(PullUpDn)
#define _InitPinF11CN(CNI_EN)
#endif
#ifdef _CNIEF12
#define _InitPinF12PUD(PullUpDn)    _CNPUF12 = (PullUpDn&0b01); _CNPDF12 = (PullUpDn>>1);
#define _InitPinF12CN(CNI_EN)          _CNIEF12 = (CNI_EN);
#else
#define _InitPinF12PUD(PullUpDn)
#define _InitPinF12CN(CNI_EN)
#endif
#ifdef _CNIEF13
#define _InitPinF13PUD(PullUpDn)    _CNPUF13 = (PullUpDn&0b01); _CNPDF13 = (PullUpDn>>1);
#define _InitPinF13CN(CNI_EN)          _CNIEF13 = (CNI_EN);
#else
#define _InitPinF13PUD(PullUpDn)
#define _InitPinF13CN(CNI_EN)
#endif
#ifdef _CNIEF14
#define _InitPinF14PUD(PullUpDn)    _CNPUF14 = (PullUpDn&0b01); _CNPDF14 = (PullUpDn>>1);
#define _InitPinF14CN(CNI_EN)          _CNIEF14 = (CNI_EN);
#else
#define _InitPinF14PUD(PullUpDn)
#define _InitPinF14CN(CNI_EN)
#endif
#ifdef _CNIEF15
#define _InitPinF15PUD(PullUpDn)    _CNPUF15 = (PullUpDn&0b01); _CNPDF15 = (PullUpDn>>1);
#define _InitPinF15CN(CNI_EN)          _CNIEF15 = (CNI_EN);
#else
#define _InitPinF15PUD(PullUpDn)
#define _InitPinF15CN(CNI_EN)
#endif

#ifdef _RF0
/***********************************************************************************************************************
 * \Function            void InitPinF0(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF0 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF0(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF0(PinMode,PullUpDn,CNI_EN)  ({_TRISF0 = ((PinMode>>1)&0b01); _InitPinF0AN(PinMode) _InitPinF0OD(PinMode) _InitPinF0PUD(PullUpDn) _InitPinF0CN(CNI_EN)})
#define ReadPinF0()           (PORTFbits.RF0)
#define SetPinF0(Value)     LATFbits.LATF0 = (Value)
#endif
#ifdef _RF1
/***********************************************************************************************************************
 * \Function            void InitPinF1(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF1 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF1(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF1(PinMode,PullUpDn,CNI_EN)  ({_TRISF1 = ((PinMode>>1)&0b01); _InitPinF1AN(PinMode)  _InitPinF1OD(PinMode) _InitPinF1PUD(PullUpDn) _InitPinF1CN(CNI_EN)})
#define ReadPinF1()           (PORTFbits.RF1)
#define SetPinF1(Value)     LATFbits.LATF1 = (Value)
#endif
#ifdef _RF2
/***********************************************************************************************************************
 * \Function            void InitPinF2(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF2 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF2(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF2(PinMode,PullUpDn,CNI_EN)  ({_TRISF2 = ((PinMode>>1)&0b01); _InitPinF2AN(PinMode)  _InitPinF2OD(PinMode) _InitPinF2PUD(PullUpDn) _InitPinF2CN(CNI_EN)})
#define ReadPinF2()           (PORTFbits.RF2)
#define SetPinF2(Value)     LATFbits.LATF2 = (Value)
#endif
#ifdef _RF3
/***********************************************************************************************************************
 * \Function            void InitPinF3(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF3 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF3(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF3(PinMode,PullUpDn,CNI_EN)  ({_TRISF3 = ((PinMode>>1)&0b01); _InitPinF3AN(PinMode)  _InitPinF3OD(PinMode) _InitPinF3PUD(PullUpDn) _InitPinF3CN(CNI_EN)})
#define ReadPinF3()           (PORTFbits.RF3)
#define SetPinF3(Value)     LATFbits.LATF3 = (Value)
#endif
#ifdef _RF4
/***********************************************************************************************************************
 * \Function            void InitPinF4(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF4 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF4(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF4(PinMode,PullUpDn,CNI_EN)  ({_TRISF4 = ((PinMode>>1)&0b01); _InitPinF4AN(PinMode)  _InitPinF4OD(PinMode) _InitPinF4PUD(PullUpDn) _InitPinF4CN(CNI_EN)})
#define ReadPinF4()           (PORTFbits.RF4)
#define SetPinF4(Value)     LATFbits.LATF4 = (Value)
#endif
#ifdef _RF5
/***********************************************************************************************************************
 * \Function            void InitPinF5(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF5 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF5(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF5(PinMode,PullUpDn,CNI_EN)  ({_TRISF5 = ((PinMode>>1)&0b01); _InitPinF5AN(PinMode)  _InitPinF5OD(PinMode) _InitPinF5PUD(PullUpDn) _InitPinF5CN(CNI_EN)})
#define ReadPinF5()           (PORTFbits.RF5)
#define SetPinF5(Value)     LATFbits.LATF5 = (Value)
#endif
#ifdef _RF6
/***********************************************************************************************************************
 * \Function            void InitPinF6(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF6 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF6(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF6(PinMode,PullUpDn,CNI_EN)  ({_TRISF6 = ((PinMode>>1)&0b01); _InitPinF6AN(PinMode)  _InitPinF6OD(PinMode) _InitPinF6PUD(PullUpDn) _InitPinF6CN(CNI_EN)})
#define ReadPinF6()           (PORTFbits.RF6)
#define SetPinF6(Value)     LATFbits.LATF6 = (Value)
#endif
#ifdef _RF7
/***********************************************************************************************************************
 * \Function            void InitPinF7(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF7 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF7(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF7(PinMode,PullUpDn,CNI_EN)  ({_TRISF7 = ((PinMode>>1)&0b01); _InitPinF7AN(PinMode)  _InitPinF7OD(PinMode) _InitPinF7PUD(PullUpDn) _InitPinF7CN(CNI_EN)})
#define ReadPinF7()           (PORTFbits.RF7)
#define SetPinF7(Value)     LATFbits.LATF7 = (Value)
#endif
#ifdef _RF8
/***********************************************************************************************************************
 * \Function            void InitPinF8(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF8 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF8(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF8(PinMode,PullUpDn,CNI_EN)  ({_TRISF8 = ((PinMode>>1)&0b01); _InitPinF8AN(PinMode)  _InitPinF8OD(PinMode) _InitPinF8PUD(PullUpDn) _InitPinF8CN(CNI_EN)})
#define ReadPinF8()           (PORTFbits.RF8)
#define SetPinF8(Value)     LATFbits.LATF8 = (Value)
#endif
#ifdef _RF9
/***********************************************************************************************************************
 * \Function            void InitPinF9(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF9 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF9(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF9(PinMode,PullUpDn,CNI_EN)  ({_TRISF9 = ((PinMode>>1)&0b01); _InitPinF9AN(PinMode)  _InitPinF9OD(PinMode) _InitPinF9PUD(PullUpDn) _InitPinF9CN(CNI_EN)})
#define ReadPinF9()           (PORTFbits.RF9)
#define SetPinF9(Value)     LATFbits.LATF9 = (Value)
#endif
#ifdef _RF10
/***********************************************************************************************************************
 * \Function            void InitPinF10(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF10 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF10(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF10(PinMode,PullUpDn,CNI_EN)  ({_TRISF10 = ((PinMode>>1)&0b01); _InitPinF10AN(PinMode)  _InitPinF10OD(PinMode) _InitPinF10PUD(PullUpDn) _InitPinF10CN(CNI_EN)})
#define ReadPinF10()          (PORTFbits.RF10)
#define SetPinF10(Value)     LATFbits.LATF10 = (Value)
#endif
#ifdef _RF11
/***********************************************************************************************************************
 * \Function            void InitPinF11(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF11 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF11(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF11(PinMode,PullUpDn,CNI_EN)  ({_TRISF11 = ((PinMode>>1)&0b01); _InitPinF11AN(PinMode)  _InitPinF11OD(PinMode) _InitPinF11PUD(PullUpDn) _InitPinF11CN(CNI_EN)})
#define ReadPinF11()          (PORTFbits.RF11)
#define SetPinF11(Value)     LATFbits.LATF11 = (Value)
#endif
#ifdef _RF12
/***********************************************************************************************************************
 * \Function            void InitPinF12(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF12 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF12(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF12(PinMode,PullUpDn,CNI_EN)  ({_TRISF12 = ((PinMode>>1)&0b01); _InitPinF12AN(PinMode)  _InitPinF12OD(PinMode) _InitPinF12PUD(PullUpDn) _InitPinF12CN(CNI_EN)})
#define ReadPinF12()          (PORTFbits.RF12)
#define SetPinF12(Value)     LATFbits.LATF12 = (Value)
#endif
#ifdef _RF13
/***********************************************************************************************************************
 * \Function            void InitPinF13(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF13 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF13(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF13(PinMode,PullUpDn,CNI_EN)  ({_TRISF13 = ((PinMode>>1)&0b01); _InitPinF13AN(PinMode)  _InitPinF13OD(PinMode) _InitPinF13PUD(PullUpDn) _InitPinF13CN(CNI_EN)})
#define ReadPinF13()          (PORTFbits.RF13)
#define SetPinF13(Value)     LATFbits.LATF13 = (Value)
#endif
#ifdef _RF14
/***********************************************************************************************************************
 * \Function            void InitPinF14(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF14 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF14(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF14(PinMode,PullUpDn,CNI_EN)  ({_TRISF14 = ((PinMode>>1)&0b01); _InitPinF14AN(PinMode)  _InitPinF14OD(PinMode) _InitPinF14PUD(PullUpDn) _InitPinF14CN(CNI_EN)})
#define ReadPinF14()          (PORTFbits.RF14)
#define SetPinF14(Value)     LATFbits.LATF14 = (Value)
#endif
#ifdef _RF15
/***********************************************************************************************************************
 * \Function            void InitPinF15(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTF.RF15 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinF15(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinF15(PinMode,PullUpDn,CNI_EN)  ({_TRISF15 = ((PinMode>>1)&0b01); _InitPinF15AN(PinMode)  _InitPinF15OD(PinMode) _InitPinF15PUD(PullUpDn) _InitPinF15CN(CNI_EN)})
#define ReadPinF15()          (PORTFbits.RF15)
#define SetPinF15(Value)     LATFbits.LATF15 = (Value)
#endif

#endif
#ifdef  PORTG

#define ReadPORTG()           (PORTG)
#define SetPORTG(Value)     LATG = (Value)

#ifdef ANSELG
/***********************************************************************************************************************
 * \Function            void InitAnalogPinsPortG(AnalogPins)
 *
 * \Description         Set PortG Individual Pins Mode (Analog OR Digital).
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>AnalogPins:</u>\n
 *  - RG0AN to RG15AN   <i>/ The Corresponding Pin is Analog Pin.</i>   \n
 *  - All_PINS_DIG      <i>/ All PortG Pins is Digital.</i>   \n
 *  - All_PINS_AN       <i>/ All PortG Pins is Analog.</i>
 *
 * \Return      None
 *
 * \Notes       None
 *
 * \Example     InitAnalogPinsPortG(All_PINS_DIG & RG6AN & RG7AN); \n
 * \Example     InitAnalogPinsPortG(All_PINS_AN);
 *
 **********************************************************************************************************************/
#define InitAnalogPinsPortG(AnalogPins)    ANSELG = ~(AnalogPins)
#endif
#ifdef _ANSG0
#define RG0AN	0b1111111111111110	/* RG0 bin is Analog bin */
#define _InitPinG0AN(PinMode)    _ANSG0 = (PinMode>>2);
#else
#define _InitPinG0AN(PinMode)
#endif
#ifdef _ANSG1
#define RG1AN	0b1111111111111101	/* RG1 bin is Analog bin */
#define _InitPinG1AN(PinMode)    _ANSG1 = (PinMode>>2);
#else
#define _InitPinG1AN(PinMode)
#endif
#ifdef _ANSG2
#define RG2AN	0b1111111111111011	/* RG2 bin is Analog bin */
#define _InitPinG2AN(PinMode)    _ANSG2 = (PinMode>>2);
#else
#define _InitPinG2AN(PinMode)
#endif
#ifdef _ANSG3
#define RG3AN	0b1111111111110111	/* RG3 bin is Analog bin */
#define _InitPinG3AN(PinMode)    _ANSG3 = (PinMode>>2);
#else
#define _InitPinG3AN(PinMode)
#endif
#ifdef _ANSG4
#define RG4AN	0b1111111111101111	/* RG4 bin is Analog bin */
#define _InitPinG4AN(PinMode)    _ANSG4 = (PinMode>>2);
#else
#define _InitPinG4AN(PinMode)
#endif
#ifdef _ANSG5
#define RG5AN	0b1111111111011111	/* RG5 bin is Analog bin */
#define _InitPinG5AN(PinMode)    _ANSG5 = (PinMode>>2);
#else
#define _InitPinG5AN(PinMode)
#endif
#ifdef _ANSG6
#define RG6AN	0b1111111110111111	/* RG6 bin is Analog bin */
#define _InitPinG6AN(PinMode)    _ANSG6 = (PinMode>>2);
#else
#define _InitPinG6AN(PinMode)
#endif
#ifdef _ANSG7
#define RG7AN	0b1111111101111111	/* RG7 bin is Analog bin */
#define _InitPinG7AN(PinMode)    _ANSG7 = (PinMode>>2);
#else
#define _InitPinG7AN(PinMode)
#endif
#ifdef _ANSG8
#define RG8AN	0b1111111011111111	/* RG8 bin is Analog bin */
#define _InitPinG8AN(PinMode)    _ANSG8 = (PinMode>>2);
#else
#define _InitPinG8AN(PinMode)
#endif
#ifdef _ANSG9
#define RG9AN	0b1111110111111111	/* RG9 bin is Analog bin */
#define _InitPinG9AN(PinMode)    _ANSG9 = (PinMode>>2);
#else
#define _InitPinG9AN(PinMode)
#endif
#ifdef _ANSG10
#define RG10AN	0b1111101111111111	/* RG10 bin is Analog bin */
#define _InitPinG10AN(PinMode)    _ANSG10 = (PinMode>>2);
#else
#define _InitPinG10AN(PinMode)
#endif
#ifdef _ANSG11
#define RG11AN	0b1111011111111111	/* RG11 bin is Analog bin */
#define _InitPinG11AN(PinMode)    _ANSG11 = (PinMode>>2);
#else
#define _InitPinG11AN(PinMode)
#endif
#ifdef _ANSG12
#define RG12AN	0b1110111111111111	/* RG12 bin is Analog bin */
#define _InitPinG12AN(PinMode)    _ANSG12 = (PinMode>>2);
#else
#define _InitPinG12AN(PinMode)
#endif
#ifdef _ANSG13
#define RG13AN	0b1101111111111111	/* RG13 bin is Analog bin */
#define _InitPinG13AN(PinMode)    _ANSG13 = (PinMode>>2);
#else
#define _InitPinG13AN(PinMode)
#endif
#ifdef _ANSG14
#define RG14AN	0b1011111111111111	/* RG14 bin is Analog bin */
#define _InitPinG14AN(PinMode)    _ANSG14 = (PinMode>>2);
#else
#define _InitPinG14AN(PinMode)
#endif
#ifdef _ANSG15
#define RG15AN	0b0111111111111111	/* RG15 bin is Analog bin */
#define _InitPinG15AN(PinMode)    _ANSG15 = (PinMode>>2);
#else
#define _InitPinG15AN(PinMode)
#endif

#ifdef _ODCG0
#define _InitPinG0OD(PinMode)    _ODCG0 = (PinMode&0b001);
#else
#define _InitPinG0OD(PinMode)
#endif
#ifdef _ODCG1
#define _InitPinG1OD(PinMode)    _ODCG1 = (PinMode&0b001);
#else
#define _InitPinG1OD(PinMode)
#endif
#ifdef _ODCG2
#define _InitPinG2OD(PinMode)    _ODCG2 = (PinMode&0b001);
#else
#define _InitPinG2OD(PinMode)
#endif
#ifdef _ODCG3
#define _InitPinG3OD(PinMode)    _ODCG3 = (PinMode&0b001);
#else
#define _InitPinG3OD(PinMode)
#endif
#ifdef _ODCG4
#define _InitPinG4OD(PinMode)    _ODCG4 = (PinMode&0b001);
#else
#define _InitPinG4OD(PinMode)
#endif
#ifdef _ODCG5
#define _InitPinG5OD(PinMode)    _ODCG5 = (PinMode&0b001);
#else
#define _InitPinG5OD(PinMode)
#endif
#ifdef _ODCG6
#define _InitPinG6OD(PinMode)    _ODCG6 = (PinMode&0b001);
#else
#define _InitPinG6OD(PinMode)
#endif
#ifdef _ODCG7
#define _InitPinG7OD(PinMode)    _ODCG7 = (PinMode&0b001);
#else
#define _InitPinG7OD(PinMode)
#endif
#ifdef _ODCG8
#define _InitPinG8OD(PinMode)    _ODCG8 = (PinMode&0b001);
#else
#define _InitPinG8OD(PinMode)
#endif
#ifdef _ODCG9
#define _InitPinG9OD(PinMode)    _ODCG9 = (PinMode&0b001);
#else
#define _InitPinG9OD(PinMode)
#endif
#ifdef _ODCG10
#define _InitPinG10OD(PinMode)    _ODCG10 = (PinMode&0b001);
#else
#define _InitPinG10OD(PinMode)
#endif
#ifdef _ODCG11
#define _InitPinG11OD(PinMode)    _ODCG11 = (PinMode&0b001);
#else
#define _InitPinG11OD(PinMode)
#endif
#ifdef _ODCG12
#define _InitPinG12OD(PinMode)    _ODCG12 = (PinMode&0b001);
#else
#define _InitPinG12OD(PinMode)
#endif
#ifdef _ODCG13
#define _InitPinG13OD(PinMode)    _ODCG13 = (PinMode&0b001);
#else
#define _InitPinG13OD(PinMode)
#endif
#ifdef _ODCG14
#define _InitPinG14OD(PinMode)    _ODCG14 = (PinMode&0b001);
#else
#define _InitPinG14OD(PinMode)
#endif
#ifdef _ODCG15
#define _InitPinG15OD(PinMode)    _ODCG15 = (PinMode&0b001);
#else
#define _InitPinG15OD(PinMode)
#endif

#ifdef _CNIEG0
#define _InitPinG0PUD(PullUpDn)    _CNPUG0 = (PullUpDn&0b01); _CNPDG0 = (PullUpDn>>1);
#define _InitPinG0CN(CNI_EN)          _CNIEG0 = (CNI_EN);
#else
#define _InitPinG0PUD(PullUpDn)
#define _InitPinG0CN(CNI_EN)
#endif
#ifdef _CNIEG1
#define _InitPinG1PUD(PullUpDn)    _CNPUG1 = (PullUpDn&0b01); _CNPDG1 = (PullUpDn>>1);
#define _InitPinG1CN(CNI_EN)          _CNIEG1 = (CNI_EN);
#else
#define _InitPinG1PUD(PullUpDn)
#define _InitPinG1CN(CNI_EN)
#endif
#ifdef _CNPUG2
#define _InitPinG2PUD(PullUpDn)    _CNPUG2 = (PullUpDn&0b01); _CNPDG2 = (PullUpDn>>1);
#define _InitPinG2CN(CNI_EN)          _CNIEG2 = (CNI_EN);
#else
#define _InitPinG2PUD(PullUpDn)
#define _InitPinG2CN(CNI_EN)
#endif
#ifdef _CNPUG3
#define _InitPinG3PUD(PullUpDn)    _CNPUG3 = (PullUpDn&0b01); _CNPDG3 = (PullUpDn>>1);
#define _InitPinG3CN(CNI_EN)          _CNIEG3 = (CNI_EN);
#else
#define _InitPinG3PUD(PullUpDn)
#define _InitPinG3CN(CNI_EN)
#endif
#ifdef _CNIEG4
#define _InitPinG4PUD(PullUpDn)    _CNPUG4 = (PullUpDn&0b01); _CNPDG4 = (PullUpDn>>1);
#define _InitPinG4CN(CNI_EN)          _CNIEG4 = (CNI_EN);
#else
#define _InitPinG4PUD(PullUpDn)
#define _InitPinG4CN(CNI_EN)
#endif
#ifdef _CNIEG5
#define _InitPinG5PUD(PullUpDn)    _CNPUG5 = (PullUpDn&0b01); _CNPDG5 = (PullUpDn>>1);
#define _InitPinG5CN(CNI_EN)          _CNIEG5 = (CNI_EN);
#else
#define _InitPinG5PUD(PullUpDn)
#define _InitPinG5CN(CNI_EN)
#endif
#ifdef _CNIEG6
#define _InitPinG6PUD(PullUpDn)    _CNPUG6 = (PullUpDn&0b01); _CNPDG6 = (PullUpDn>>1);
#define _InitPinG6CN(CNI_EN)          _CNIEG6 = (CNI_EN);
#else
#define _InitPinG6PUD(PullUpDn)
#define _InitPinG6CN(CNI_EN)
#endif
#ifdef _CNIEG7
#define _InitPinG7PUD(PullUpDn)    _CNPUG7 = (PullUpDn&0b01); _CNPDG7 = (PullUpDn>>1);
#define _InitPinG7CN(CNI_EN)          _CNIEG7 = (CNI_EN);
#else
#define _InitPinG7PUD(PullUpDn)
#define _InitPinG7CN(CNI_EN)
#endif
#ifdef _CNIEG8
#define _InitPinG8PUD(PullUpDn)    _CNPUG8 = (PullUpDn&0b01); _CNPDG8 = (PullUpDn>>1);
#define _InitPinG8CN(CNI_EN)          _CNIEG8 = (CNI_EN);
#else
#define _InitPinG8PUD(PullUpDn)
#define _InitPinG8CN(CNI_EN)
#endif
#ifdef _CNIEG9
#define _InitPinG9PUD(PullUpDn)    _CNPUG9 = (PullUpDn&0b01); _CNPDG9 = (PullUpDn>>1);
#define _InitPinG9CN(CNI_EN)          _CNIEG9 = (CNI_EN);
#else
#define _InitPinG9PUD(PullUpDn)
#define _InitPinG9CN(CNI_EN)
#endif
#ifdef _CNIEG10
#define _InitPinG10PUD(PullUpDn)    _CNPUG10 = (PullUpDn&0b01); _CNPDG10 = (PullUpDn>>1);
#define _InitPinG10CN(CNI_EN)          _CNIEG10 = (CNI_EN);
#else
#define _InitPinG10PUD(PullUpDn)
#define _InitPinG10CN(CNI_EN)
#endif
#ifdef _CNIEG11
#define _InitPinG11PUD(PullUpDn)    _CNPUG11 = (PullUpDn&0b01); _CNPDG11 = (PullUpDn>>1);
#define _InitPinG11CN(CNI_EN)          _CNIEG11 = (CNI_EN);
#else
#define _InitPinG11PUD(PullUpDn)
#define _InitPinG11CN(CNI_EN)
#endif
#ifdef _CNIEG12
#define _InitPinG12PUD(PullUpDn)    _CNPUG12 = (PullUpDn&0b01); _CNPDG12 = (PullUpDn>>1);
#define _InitPinG12CN(CNI_EN)          _CNIEG12 = (CNI_EN);
#else
#define _InitPinG12PUD(PullUpDn)
#define _InitPinG12CN(CNI_EN)
#endif
#ifdef _CNIEG13
#define _InitPinG13PUD(PullUpDn)    _CNPUG13 = (PullUpDn&0b01); _CNPDG13 = (PullUpDn>>1);
#define _InitPinG13CN(CNI_EN)          _CNIEG13 = (CNI_EN);
#else
#define _InitPinG13PUD(PullUpDn)
#define _InitPinG13CN(CNI_EN)
#endif
#ifdef _CNIEG14
#define _InitPinG14PUD(PullUpDn)    _CNPUG14 = (PullUpDn&0b01); _CNPDG14 = (PullUpDn>>1);
#define _InitPinG14CN(CNI_EN)          _CNIEG14 = (CNI_EN);
#else
#define _InitPinG14PUD(PullUpDn)
#define _InitPinG14CN(CNI_EN)
#endif
#ifdef _CNIEG15
#define _InitPinG15PUD(PullUpDn)    _CNPUG15 = (PullUpDn&0b01); _CNPDG15 = (PullUpDn>>1);
#define _InitPinG15CN(CNI_EN)          _CNIEG15 = (CNI_EN);
#else
#define _InitPinG15PUD(PullUpDn)
#define _InitPinG15CN(CNI_EN)
#endif

#ifdef _RG0
/***********************************************************************************************************************
 * \Function            void InitPinG0(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG0 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG0(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG0(PinMode,PullUpDn,CNI_EN)  ({_TRISG0 = ((PinMode>>1)&0b01); _InitPinG0AN(PinMode) _InitPinG0OD(PinMode) _InitPinG0PUD(PullUpDn) _InitPinG0CN(CNI_EN)})
#define ReadPinG0()           (PORTGbits.RG0)
#define SetPinG0(Value)     LATGbits.LATG0 = (Value)
#endif
#ifdef _RG1
/***********************************************************************************************************************
 * \Function            void InitPinG1(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG1 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG1(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG1(PinMode,PullUpDn,CNI_EN)  ({_TRISG1 = ((PinMode>>1)&0b01); _InitPinG1AN(PinMode)  _InitPinG1OD(PinMode) _InitPinG1PUD(PullUpDn) _InitPinG1CN(CNI_EN)})
#define ReadPinG1()           (PORTGbits.RG1)
#define SetPinG1(Value)     LATGbits.LATG1 = (Value)
#endif
#ifdef _RG2
/***********************************************************************************************************************
 * \Function            void InitPinG2(CNI_EN)
 *
 * \Description         Set Up PORTG.RG2 Pin Mode (CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - This PIN can Be Only Input. \n
 *				- If RG2 and RG3 are used as general purpose inputs, the VUSB3V3 pin must be connected to VDD. 
 *
 * \Example     InitPinG2(CN_DIS);
 *
 **********************************************************************************************************************/
	#ifdef _TRISG2
		#define InitPinG2(PinMode,PullUpDn,CNI_EN)  ({_TRISG2 = ((PinMode>>1)&0b01); _InitPinG2AN(PinMode)  _InitPinG2OD(PinMode) _InitPinG2PUD(PullUpDn) _InitPinG2CN(CNI_EN)})
		#define SetPinG2(Value)     LATGbits.LATG2 = (Value)
		#define ReadPinG2()           (PORTGbits.RG2)
	#else
		#define InitPinG2(CNI_EN)  ({_InitPinG2CN(CNI_EN)})
		#define ReadPinG2()           (PORTGbits.RG2)
	#endif
#endif

#ifdef _RG3
/***********************************************************************************************************************
 * \Function            void InitPinG3(CNI_EN)
 *
 * \Description         Set Up PORTG.RG3 Pin Mode (CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - This PIN can Be Only Input. \n
 *				- If RG2 and RG3 are used as general purpose inputs, the VUSB3V3 pin must be connected to VDD. 
 *
 * \Example     InitPinG3(CN_DIS);
 *
 **********************************************************************************************************************/
	#ifdef _TRISG2
		#define InitPinG3(PinMode,PullUpDn,CNI_EN)  ({_TRISG3 = ((PinMode>>1)&0b01); _InitPinG3AN(PinMode)  _InitPinG3OD(PinMode) _InitPinG3PUD(PullUpDn) _InitPinG3CN(CNI_EN)})
		#define ReadPinG3()           (PORTGbits.RG3)
		#define SetPinG3(Value)     LATGbits.LATG3 = (Value)
	#else
		#define InitPinG3(CNI_EN)  ({_InitPinG3CN(CNI_EN)})
		#define ReadPinG3()           (PORTGbits.RG3)
	#endif
#endif
#ifdef _RG4
/***********************************************************************************************************************
 * \Function            void InitPinG4(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG4 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG4(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG4(PinMode,PullUpDn,CNI_EN)  ({_TRISG4 = ((PinMode>>1)&0b01); _InitPinG4AN(PinMode)  _InitPinG4OD(PinMode) _InitPinG4PUD(PullUpDn) _InitPinG4CN(CNI_EN)})
#define ReadPinG4()           (PORTGbits.RG4)
#define SetPinG4(Value)     LATGbits.LATG4 = (Value)
#endif
#ifdef _RG5
/***********************************************************************************************************************
 * \Function            void InitPinG5(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG5 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG5(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG5(PinMode,PullUpDn,CNI_EN)  ({_TRISG5 = ((PinMode>>1)&0b01); _InitPinG5AN(PinMode)  _InitPinG5OD(PinMode) _InitPinG5PUD(PullUpDn) _InitPinG5CN(CNI_EN)})
#define ReadPinG5()           (PORTGbits.RG5)
#define SetPinG5(Value)     LATGbits.LATG5 = (Value)
#endif
#ifdef _RG6
/***********************************************************************************************************************
 * \Function            void InitPinG6(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG6 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG6(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG6(PinMode,PullUpDn,CNI_EN)  ({_TRISG6 = ((PinMode>>1)&0b01); _InitPinG6AN(PinMode)  _InitPinG6OD(PinMode) _InitPinG6PUD(PullUpDn) _InitPinG6CN(CNI_EN)})
#define ReadPinG6()           (PORTGbits.RG6)
#define SetPinG6(Value)     LATGbits.LATG6 = (Value)
#endif
#ifdef _RG7
/***********************************************************************************************************************
 * \Function            void InitPinG7(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG7 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG7(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG7(PinMode,PullUpDn,CNI_EN)  ({_TRISG7 = ((PinMode>>1)&0b01); _InitPinG7AN(PinMode)  _InitPinG7OD(PinMode) _InitPinG7PUD(PullUpDn) _InitPinG7CN(CNI_EN)})
#define ReadPinG7()           (PORTGbits.RG7)
#define SetPinG7(Value)     LATGbits.LATG7 = (Value)
#endif
#ifdef _RG8
/***********************************************************************************************************************
 * \Function            void InitPinG8(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG8 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG8(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG8(PinMode,PullUpDn,CNI_EN)  ({_TRISG8 = ((PinMode>>1)&0b01); _InitPinG8AN(PinMode)  _InitPinG8OD(PinMode) _InitPinG8PUD(PullUpDn) _InitPinG8CN(CNI_EN)})
#define ReadPinG8()           (PORTGbits.RG8)
#define SetPinG8(Value)     LATGbits.LATG8 = (Value)
#endif
#ifdef _RG9
/***********************************************************************************************************************
 * \Function            void InitPinG9(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG9 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG9(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG9(PinMode,PullUpDn,CNI_EN)  ({_TRISG9 = ((PinMode>>1)&0b01); _InitPinG9AN(PinMode)  _InitPinG9OD(PinMode) _InitPinG9PUD(PullUpDn) _InitPinG9CN(CNI_EN)})
#define ReadPinG9()           (PORTGbits.RG9)
#define SetPinG9(Value)     LATGbits.LATG9 = (Value)
#endif
#ifdef _RG10
/***********************************************************************************************************************
 * \Function            void InitPinG10(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG10 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG10(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG10(PinMode,PullUpDn,CNI_EN)  ({_TRISG10 = ((PinMode>>1)&0b01); _InitPinG10AN(PinMode)  _InitPinG10OD(PinMode) _InitPinG10PUD(PullUpDn) _InitPinG10CN(CNI_EN)})
#define ReadPinG10()          (PORTGbits.RG10)
#define SetPinG10(Value)     LATGbits.LATG10 = (Value)
#endif
#ifdef _RG11
/***********************************************************************************************************************
 * \Function            void InitPinG11(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG11 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG11(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG11(PinMode,PullUpDn,CNI_EN)  ({_TRISG11 = ((PinMode>>1)&0b01); _InitPinG11AN(PinMode)  _InitPinG11OD(PinMode) _InitPinG11PUD(PullUpDn) _InitPinG11CN(CNI_EN)})
#define ReadPinG11()          (PORTGbits.RG11)
#define SetPinG11(Value)     LATGbits.LATG11 = (Value)
#endif
#ifdef _RG12
/***********************************************************************************************************************
 * \Function            void InitPinG12(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG12 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG12(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG12(PinMode,PullUpDn,CNI_EN)  ({_TRISG12 = ((PinMode>>1)&0b01); _InitPinG12AN(PinMode)  _InitPinG12OD(PinMode) _InitPinG12PUD(PullUpDn) _InitPinG12CN(CNI_EN)})
#define ReadPinG12()          (PORTGbits.RG12)
#define SetPinG12(Value)     LATGbits.LATG12 = (Value)
#endif
#ifdef _RG13
/***********************************************************************************************************************
 * \Function            void InitPinG13(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG13 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG13(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG13(PinMode,PullUpDn,CNI_EN)  ({_TRISG13 = ((PinMode>>1)&0b01); _InitPinG13AN(PinMode)  _InitPinG13OD(PinMode) _InitPinG13PUD(PullUpDn) _InitPinG13CN(CNI_EN)})
#define ReadPinG13()          (PORTGbits.RG13)
#define SetPinG13(Value)     LATGbits.LATG13 = (Value)
#endif
#ifdef _RG14
/***********************************************************************************************************************
 * \Function            void InitPinG14(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG14 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG14(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG14(PinMode,PullUpDn,CNI_EN)  ({_TRISG14 = ((PinMode>>1)&0b01); _InitPinG14AN(PinMode)  _InitPinG14OD(PinMode) _InitPinG14PUD(PullUpDn) _InitPinG14CN(CNI_EN)})
#define ReadPinG14()          (PORTGbits.RG14)
#define SetPinG14(Value)     LATGbits.LATG14 = (Value)
#endif
#ifdef _RG15
/***********************************************************************************************************************
 * \Function            void InitPinG15(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTG.RG15 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinG15(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinG15(PinMode,PullUpDn,CNI_EN)  ({_TRISG15 = ((PinMode>>1)&0b01); _InitPinG15AN(PinMode)  _InitPinG15OD(PinMode) _InitPinG15PUD(PullUpDn) _InitPinG15CN(CNI_EN)})
#define ReadPinG15()          (PORTGbits.RG15)
#define SetPinG15(Value)     LATGbits.LATG15 = (Value)
#endif

#endif
#ifdef  PORTH

#define ReadPORTH()           (PORTH)
#define SetPORTH(Value)     LATH = (Value)

#ifdef ANSELH
/***********************************************************************************************************************
 * \Function            void InitAnalogPinsPortH(AnalogPins)
 *
 * \Description         Set PortH Individual Pins Mode (Analog OR Digital).
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>AnalogPins:</u>\n
 *  - RH0AN to RH15AN   <i>/ The Corresponding Pin is Analog Pin.</i>   \n
 *  - All_PINS_DIG      <i>/ All PortH Pins is Digital.</i>   \n
 *  - All_PINS_AN       <i>/ All PortH Pins is Analog.</i>
 *
 * \Return      None
 *
 * \Notes       None
 *
 * \Example     InitAnalogPinsPortH(All_PINS_DIG & RH6AN & RH7AN); \n
 * \Example     InitAnalogPinsPortG(All_PINS_AN);
 *
 **********************************************************************************************************************/
#define InitAnalogPinsPortH(AnalogPins)    ANSELH = ~(AnalogPins)
#endif
#ifdef _ANSH0
#define RH0AN	0b1111111111111110	/* RH0 bin is Analog bin */
#define _InitPinH0AN(PinMode)    _ANSH0 = (PinMode>>2);
#else
#define _InitPinH0AN(PinMode)
#endif
#ifdef _ANSH1
#define RH1AN	0b1111111111111101	/* RH1 bin is Analog bin */
#define _InitPinH1AN(PinMode)    _ANSH1 = (PinMode>>2);
#else
#define _InitPinH1AN(PinMode)
#endif
#ifdef _ANSH2
#define RH2AN	0b1111111111111011	/* RH2 bin is Analog bin */
#define _InitPinH2AN(PinMode)    _ANSH2 = (PinMode>>2);
#else
#define _InitPinH2AN(PinMode)
#endif
#ifdef _ANSH3
#define RH3AN	0b1111111111110111	/* RH3 bin is Analog bin */
#define _InitPinH3AN(PinMode)    _ANSH3 = (PinMode>>2);
#else
#define _InitPinH3AN(PinMode)
#endif
#ifdef _ANSH4
#define RH4AN	0b1111111111101111	/* RH4 bin is Analog bin */
#define _InitPinH4AN(PinMode)    _ANSH4 = (PinMode>>2);
#else
#define _InitPinH4AN(PinMode)
#endif
#ifdef _ANSH5
#define RH5AN	0b1111111111011111	/* RH5 bin is Analog bin */
#define _InitPinH5AN(PinMode)    _ANSH5 = (PinMode>>2);
#else
#define _InitPinH5AN(PinMode)
#endif
#ifdef _ANSH6
#define RH6AN	0b1111111110111111	/* RH6 bin is Analog bin */
#define _InitPinH6AN(PinMode)    _ANSH6 = (PinMode>>2);
#else
#define _InitPinH6AN(PinMode)
#endif
#ifdef _ANSH7
#define RH7AN	0b1111111101111111	/* RH7 bin is Analog bin */
#define _InitPinH7AN(PinMode)    _ANSH7 = (PinMode>>2);
#else
#define _InitPinH7AN(PinMode)
#endif
#ifdef _ANSH8
#define RH8AN	0b1111111011111111	/* RH8 bin is Analog bin */
#define _InitPinH8AN(PinMode)    _ANSH8 = (PinMode>>2);
#else
#define _InitPinH8AN(PinMode)
#endif
#ifdef _ANSH9
#define RH9AN	0b1111110111111111	/* RH9 bin is Analog bin */
#define _InitPinH9AN(PinMode)    _ANSH9 = (PinMode>>2);
#else
#define _InitPinH9AN(PinMode)
#endif
#ifdef _ANSH10
#define RH10AN	0b1111101111111111	/* RH10 bin is Analog bin */
#define _InitPinH10AN(PinMode)    _ANSH10 = (PinMode>>2);
#else
#define _InitPinH10AN(PinMode)
#endif
#ifdef _ANSH11
#define RH11AN	0b1111011111111111	/* RH11 bin is Analog bin */
#define _InitPinH11AN(PinMode)    _ANSH11 = (PinMode>>2);
#else
#define _InitPinH11AN(PinMode)
#endif
#ifdef _ANSH12
#define RH12AN	0b1110111111111111	/* RH12 bin is Analog bin */
#define _InitPinH12AN(PinMode)    _ANSH12 = (PinMode>>2);
#else
#define _InitPinH12AN(PinMode)
#endif
#ifdef _ANSH13
#define RH13AN	0b1101111111111111	/* RH13 bin is Analog bin */
#define _InitPinH13AN(PinMode)    _ANSH13 = (PinMode>>2);
#else
#define _InitPinH13AN(PinMode)
#endif
#ifdef _ANSH14
#define RH14AN	0b1011111111111111	/* RH14 bin is Analog bin */
#define _InitPinH14AN(PinMode)    _ANSH14 = (PinMode>>2);
#else
#define _InitPinH14AN(PinMode)
#endif
#ifdef _ANSH15
#define RH15AN	0b0111111111111111	/* RH15 bin is Analog bin */
#define _InitPinH15AN(PinMode)    _ANSH15 = (PinMode>>2);
#else
#define _InitPinH15AN(PinMode)
#endif

#ifdef _ODCH0
#define _InitPinH0OD(PinMode)    _ODCH0 = (PinMode&0b001);
#else
#define _InitPinH0OD(PinMode)
#endif
#ifdef _ODCH1
#define _InitPinH1OD(PinMode)    _ODCH1 = (PinMode&0b001);
#else
#define _InitPinH1OD(PinMode)
#endif
#ifdef _ODCH2
#define _InitPinH2OD(PinMode)    _ODCH2 = (PinMode&0b001);
#else
#define _InitPinH2OD(PinMode)
#endif
#ifdef _ODCH3
#define _InitPinH3OD(PinMode)    _ODCH3 = (PinMode&0b001);
#else
#define _InitPinH3OD(PinMode)
#endif
#ifdef _ODCH4
#define _InitPinH4OD(PinMode)    _ODCH4 = (PinMode&0b001);
#else
#define _InitPinH4OD(PinMode)
#endif
#ifdef _ODCH5
#define _InitPinH5OD(PinMode)    _ODCH5 = (PinMode&0b001);
#else
#define _InitPinH5OD(PinMode)
#endif
#ifdef _ODCH6
#define _InitPinH6OD(PinMode)    _ODCH6 = (PinMode&0b001);
#else
#define _InitPinH6OD(PinMode)
#endif
#ifdef _ODCH7
#define _InitPinH7OD(PinMode)    _ODCH7 = (PinMode&0b001);
#else
#define _InitPinH7OD(PinMode)
#endif
#ifdef _ODCH8
#define _InitPinH8OD(PinMode)    _ODCH8 = (PinMode&0b001);
#else
#define _InitPinH8OD(PinMode)
#endif
#ifdef _ODCH9
#define _InitPinH9OD(PinMode)    _ODCH9 = (PinMode&0b001);
#else
#define _InitPinH9OD(PinMode)
#endif
#ifdef _ODCH10
#define _InitPinH10OD(PinMode)    _ODCH10 = (PinMode&0b001);
#else
#define _InitPinH10OD(PinMode)
#endif
#ifdef _ODCH11
#define _InitPinH11OD(PinMode)    _ODCH11 = (PinMode&0b001);
#else
#define _InitPinH11OD(PinMode)
#endif
#ifdef _ODCH12
#define _InitPinH12OD(PinMode)    _ODCH12 = (PinMode&0b001);
#else
#define _InitPinH12OD(PinMode)
#endif
#ifdef _ODCH13
#define _InitPinH13OD(PinMode)    _ODCH13 = (PinMode&0b001);
#else
#define _InitPinH13OD(PinMode)
#endif
#ifdef _ODCH14
#define _InitPinH14OD(PinMode)    _ODCH14 = (PinMode&0b001);
#else
#define _InitPinH14OD(PinMode)
#endif
#ifdef _ODCH15
#define _InitPinH15OD(PinMode)    _ODCH15 = (PinMode&0b001);
#else
#define _InitPinH15OD(PinMode)
#endif

#ifdef _CNIEH0
#define _InitPinH0PUD(PullUpDn)    _CNPUH0 = (PullUpDn&0b01); _CNPDH0 = (PullUpDn>>1);
#define _InitPinH0CN(CNI_EN)          _CNIEH0 = (CNI_EN);
#else
#define _InitPinH0PUD(PullUpDn)
#define _InitPinH0CN(CNI_EN)
#endif
#ifdef _CNIEH1
#define _InitPinH1PUD(PullUpDn)    _CNPUH1 = (PullUpDn&0b01); _CNPDH1 = (PullUpDn>>1);
#define _InitPinH1CN(CNI_EN)          _CNIEH1 = (CNI_EN);
#else
#define _InitPinH1PUD(PullUpDn)
#define _InitPinH1CN(CNI_EN)
#endif
#ifdef _CNIEH2
#define _InitPinH2PUD(PullUpDn)    _CNPUH2 = (PullUpDn&0b01); _CNPDH2 = (PullUpDn>>1);
#define _InitPinH2CN(CNI_EN)          _CNIEH2 = (CNI_EN);
#else
#define _InitPinH2PUD(PullUpDn)
#define _InitPinH2CN(CNI_EN)
#endif
#ifdef _CNIEH3
#define _InitPinH3PUD(PullUpDn)    _CNPUH3 = (PullUpDn&0b01); _CNPDH3 = (PullUpDn>>1);
#define _InitPinH3CN(CNI_EN)          _CNIEH3 = (CNI_EN);
#else
#define _InitPinH3PUD(PullUpDn)
#define _InitPinH3CN(CNI_EN)
#endif
#ifdef _CNIEH4
#define _InitPinH4PUD(PullUpDn)    _CNPUH4 = (PullUpDn&0b01); _CNPDH4 = (PullUpDn>>1);
#define _InitPinH4CN(CNI_EN)          _CNIEH4 = (CNI_EN);
#else
#define _InitPinH4PUD(PullUpDn)
#define _InitPinH4CN(CNI_EN)
#endif
#ifdef _CNIEH5
#define _InitPinH5PUD(PullUpDn)    _CNPUH5 = (PullUpDn&0b01); _CNPDH5 = (PullUpDn>>1);
#define _InitPinH5CN(CNI_EN)          _CNIEH5 = (CNI_EN);
#else
#define _InitPinH5PUD(PullUpDn)
#define _InitPinH5CN(CNI_EN)
#endif
#ifdef _CNIEH6
#define _InitPinH6PUD(PullUpDn)    _CNPUH6 = (PullUpDn&0b01); _CNPDH6 = (PullUpDn>>1);
#define _InitPinH6CN(CNI_EN)          _CNIEH6 = (CNI_EN);
#else
#define _InitPinH6PUD(PullUpDn)
#define _InitPinH6CN(CNI_EN)
#endif
#ifdef _CNIEH7
#define _InitPinH7PUD(PullUpDn)    _CNPUH7 = (PullUpDn&0b01); _CNPDH7 = (PullUpDn>>1);
#define _InitPinH7CN(CNI_EN)          _CNIEH7 = (CNI_EN);
#else
#define _InitPinH7PUD(PullUpDn)
#define _InitPinH7CN(CNI_EN)
#endif
#ifdef _CNIEH8
#define _InitPinH8PUD(PullUpDn)    _CNPUH8 = (PullUpDn&0b01); _CNPDH8 = (PullUpDn>>1);
#define _InitPinH8CN(CNI_EN)          _CNIEH8 = (CNI_EN);
#else
#define _InitPinH8PUD(PullUpDn)
#define _InitPinH8CN(CNI_EN)
#endif
#ifdef _CNIEH9
#define _InitPinH9PUD(PullUpDn)    _CNPUH9 = (PullUpDn&0b01); _CNPDH9 = (PullUpDn>>1);
#define _InitPinH9CN(CNI_EN)          _CNIEH9 = (CNI_EN);
#else
#define _InitPinH9PUD(PullUpDn)
#define _InitPinH9CN(CNI_EN)
#endif
#ifdef _CNIEH10
#define _InitPinH10PUD(PullUpDn)    _CNPUH10 = (PullUpDn&0b01); _CNPDH10 = (PullUpDn>>1);
#define _InitPinH10CN(CNI_EN)          _CNIEH10 = (CNI_EN);
#else
#define _InitPinH10PUD(PullUpDn)
#define _InitPinH10CN(CNI_EN)
#endif
#ifdef _CNIEH11
#define _InitPinH11PUD(PullUpDn)    _CNPUH11 = (PullUpDn&0b01); _CNPDH11 = (PullUpDn>>1);
#define _InitPinH11CN(CNI_EN)          _CNIEH11 = (CNI_EN);
#else
#define _InitPinH11PUD(PullUpDn)
#define _InitPinH11CN(CNI_EN)
#endif
#ifdef _CNIEH12
#define _InitPinH12PUD(PullUpDn)    _CNPUH12 = (PullUpDn&0b01); _CNPDH12 = (PullUpDn>>1);
#define _InitPinH12CN(CNI_EN)          _CNIEH12 = (CNI_EN);
#else
#define _InitPinH12PUD(PullUpDn)
#define _InitPinH12CN(CNI_EN)
#endif
#ifdef _CNIEH13
#define _InitPinH13PUD(PullUpDn)    _CNPUH13 = (PullUpDn&0b01); _CNPDH13 = (PullUpDn>>1);
#define _InitPinH13CN(CNI_EN)          _CNIEH13 = (CNI_EN);
#else
#define _InitPinH13PUD(PullUpDn)
#define _InitPinH13CN(CNI_EN)
#endif
#ifdef _CNIEH14
#define _InitPinH14PUD(PullUpDn)    _CNPUH14 = (PullUpDn&0b01); _CNPDH14 = (PullUpDn>>1);
#define _InitPinH14CN(CNI_EN)          _CNIEH14 = (CNI_EN);
#else
#define _InitPinH14PUD(PullUpDn)
#define _InitPinH14CN(CNI_EN)
#endif
#ifdef _CNIEH15
#define _InitPinH15PUD(PullUpDn)    _CNPUH15 = (PullUpDn&0b01); _CNPDH15 = (PullUpDn>>1);
#define _InitPinH15CN(CNI_EN)          _CNIEH15 = (CNI_EN);
#else
#define _InitPinH15PUD(PullUpDn)
#define _InitPinH15CN(CNI_EN)
#endif

#ifdef _RH0
/***********************************************************************************************************************
 * \Function            void InitPinH0(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH0 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH0(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH0(PinMode,PullUpDn,CNI_EN)  ({_TRISH0 = ((PinMode>>1)&0b01); _InitPinH0AN(PinMode) _InitPinH0OD(PinMode) _InitPinH0PUD(PullUpDn) _InitPinH0CN(CNI_EN)})
#define ReadPinH0()           (PORTHbits.RH0)
#define SetPinH0(Value)     LATHbits.LATH0 = (Value)
#endif
#ifdef _RH1
/***********************************************************************************************************************
 * \Function            void InitPinH1(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH1 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH1(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH1(PinMode,PullUpDn,CNI_EN)  ({_TRISH1 = ((PinMode>>1)&0b01); _InitPinH1AN(PinMode)  _InitPinH1OD(PinMode) _InitPinH1PUD(PullUpDn) _InitPinH1CN(CNI_EN)})
#define ReadPinH1()           (PORTHbits.RH1)
#define SetPinH1(Value)     LATHbits.LATH1 = (Value)
#endif
#ifdef _RH2
/***********************************************************************************************************************
 * \Function            void InitPinH2(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH2 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH2(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH2(PinMode,PullUpDn,CNI_EN)  ({_TRISH2 = ((PinMode>>1)&0b01); _InitPinH2AN(PinMode)  _InitPinH2OD(PinMode) _InitPinH2PUD(PullUpDn) _InitPinH2CN(CNI_EN)})
#define ReadPinH2()           (PORTHbits.RH2)
#define SetPinH2(Value)     LATHbits.LATH2 = (Value)
#endif
#ifdef _RH3
/***********************************************************************************************************************
 * \Function            void InitPinH3(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH3 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH3(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH3(PinMode,PullUpDn,CNI_EN)  ({_TRISH3 = ((PinMode>>1)&0b01); _InitPinH3AN(PinMode)  _InitPinH3OD(PinMode) _InitPinH3PUD(PullUpDn) _InitPinH3CN(CNI_EN)})
#define ReadPinH3()           (PORTHbits.RH3)
#define SetPinH3(Value)     LATHbits.LATH3 = (Value)
#endif
#ifdef _RH4
/***********************************************************************************************************************
 * \Function            void InitPinH4(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH4 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH4(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH4(PinMode,PullUpDn,CNI_EN)  ({_TRISH4 = ((PinMode>>1)&0b01); _InitPinH4AN(PinMode)  _InitPinH4OD(PinMode) _InitPinH4PUD(PullUpDn) _InitPinH4CN(CNI_EN)})
#define ReadPinH4()           (PORTHbits.RH4)
#define SetPinH4(Value)     LATHbits.LATH4 = (Value)
#endif
#ifdef _RH5
/***********************************************************************************************************************
 * \Function            void InitPinH5(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH5 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH5(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH5(PinMode,PullUpDn,CNI_EN)  ({_TRISH5 = ((PinMode>>1)&0b01); _InitPinH5AN(PinMode)  _InitPinH5OD(PinMode) _InitPinH5PUD(PullUpDn) _InitPinH5CN(CNI_EN)})
#define ReadPinH5()           (PORTHbits.RH5)
#define SetPinH5(Value)     LATHbits.LATH5 = (Value)
#endif
#ifdef _RH6
/***********************************************************************************************************************
 * \Function            void InitPinH6(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH6 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH6(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH6(PinMode,PullUpDn,CNI_EN)  ({_TRISH6 = ((PinMode>>1)&0b01); _InitPinH6AN(PinMode)  _InitPinH6OD(PinMode) _InitPinH6PUD(PullUpDn) _InitPinH6CN(CNI_EN)})
#define ReadPinH6()           (PORTHbits.RH6)
#define SetPinH6(Value)     LATHbits.LATH6 = (Value)
#endif
#ifdef _RH7
/***********************************************************************************************************************
 * \Function            void InitPinH7(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH7 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH7(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH7(PinMode,PullUpDn,CNI_EN)  ({_TRISH7 = ((PinMode>>1)&0b01); _InitPinH7AN(PinMode)  _InitPinH7OD(PinMode) _InitPinH7PUD(PullUpDn) _InitPinH7CN(CNI_EN)})
#define ReadPinH7()           (PORTHbits.RH7)
#define SetPinH7(Value)     LATHbits.LATH7 = (Value)
#endif
#ifdef _RH8
/***********************************************************************************************************************
 * \Function            void InitPinH8(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH8 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH8(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH8(PinMode,PullUpDn,CNI_EN)  ({_TRISH8 = ((PinMode>>1)&0b01); _InitPinH8AN(PinMode)  _InitPinH8OD(PinMode) _InitPinH8PUD(PullUpDn) _InitPinH8CN(CNI_EN)})
#define ReadPinH8()           (PORTHbits.RH8)
#define SetPinH8(Value)     LATHbits.LATH8 = (Value)
#endif
#ifdef _RH9
/***********************************************************************************************************************
 * \Function            void InitPinH9(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH9 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH9(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH9(PinMode,PullUpDn,CNI_EN)  ({_TRISH9 = ((PinMode>>1)&0b01); _InitPinH9AN(PinMode)  _InitPinH9OD(PinMode) _InitPinH9PUD(PullUpDn) _InitPinH9CN(CNI_EN)})
#define ReadPinH9()           (PORTHbits.RH9)
#define SetPinH9(Value)     LATHbits.LATH9 = (Value)
#endif
#ifdef _RH10
/***********************************************************************************************************************
 * \Function            void InitPinH10(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH10 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH10(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH10(PinMode,PullUpDn,CNI_EN)  ({_TRISH10 = ((PinMode>>1)&0b01); _InitPinH10AN(PinMode)  _InitPinH10OD(PinMode) _InitPinH10PUD(PullUpDn) _InitPinH10CN(CNI_EN)})
#define ReadPinH10()          (PORTHbits.RH10)
#define SetPinH10(Value)     LATHbits.LATH10 = (Value)
#endif
#ifdef _RH11
/***********************************************************************************************************************
 * \Function            void InitPinH11(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH11 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH11(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH11(PinMode,PullUpDn,CNI_EN)  ({_TRISH11 = ((PinMode>>1)&0b01); _InitPinH11AN(PinMode)  _InitPinH11OD(PinMode) _InitPinH11PUD(PullUpDn) _InitPinH11CN(CNI_EN)})
#define ReadPinH11()          (PORTHbits.RH11)
#define SetPinH11(Value)     LATHbits.LATH11 = (Value)
#endif
#ifdef _RH12
/***********************************************************************************************************************
 * \Function            void InitPinH12(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH12 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH12(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH12(PinMode,PullUpDn,CNI_EN)  ({_TRISH12 = ((PinMode>>1)&0b01); _InitPinH12AN(PinMode)  _InitPinH12OD(PinMode) _InitPinH12PUD(PullUpDn) _InitPinH12CN(CNI_EN)})
#define ReadPinH12()          (PORTHbits.RH12)
#define SetPinH12(Value)     LATHbits.LATH12 = (Value)
#endif
#ifdef _RH13
/***********************************************************************************************************************
 * \Function            void InitPinH13(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH13 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH13(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH13(PinMode,PullUpDn,CNI_EN)  ({_TRISH13 = ((PinMode>>1)&0b01); _InitPinH13AN(PinMode)  _InitPinH13OD(PinMode) _InitPinH13PUD(PullUpDn) _InitPinH13CN(CNI_EN)})
#define ReadPinH13()          (PORTHbits.RH13)
#define SetPinH13(Value)     LATHbits.LATH13 = (Value)
#endif
#ifdef _RH14
/***********************************************************************************************************************
 * \Function            void InitPinH14(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH14 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH14(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH14(PinMode,PullUpDn,CNI_EN)  ({_TRISH14 = ((PinMode>>1)&0b01); _InitPinH14AN(PinMode)  _InitPinH14OD(PinMode) _InitPinH14PUD(PullUpDn) _InitPinH14CN(CNI_EN)})
#define ReadPinH14()          (PORTHbits.RH14)
#define SetPinH14(Value)     LATHbits.LATH14 = (Value)
#endif
#ifdef _RH15
/***********************************************************************************************************************
 * \Function            void InitPinH15(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTH.RH15 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinH15(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinH15(PinMode,PullUpDn,CNI_EN)  ({_TRISH15 = ((PinMode>>1)&0b01); _InitPinH15AN(PinMode)  _InitPinH15OD(PinMode) _InitPinH15PUD(PullUpDn) _InitPinH15CN(CNI_EN)})
#define ReadPinH15()          (PORTHbits.RH15)
#define SetPinH15(Value)     LATHbits.LATH15 = (Value)
#endif

#endif
#ifdef  PORTJ

#define ReadPORTJ()           (PORTJ)
#define SetPORTJ(Value)     LATJ = (Value)

#ifdef ANSELJ
/***********************************************************************************************************************
 * \Function            void InitAnalogPinsPortJ(AnalogPins)
 *
 * \Description         Set PortJ Individual Pins Mode (Analog OR Digital).
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>AnalogPins:</u>\n
 *  - RJ0AN to RJ15AN   <i>/ The Corresponding Pin is Analog Pin.</i>   \n
 *  - All_PINS_DIG      <i>/ All PortJ Pins is Digital.</i>   \n
 *  - All_PINS_AN       <i>/ All PortJ Pins is Analog.</i>
 *
 * \Return      None
 *
 * \Notes       None
 *
 * \Example     InitAnalogPinsPortJ(All_PINS_DIG & RJ6AN & RJ7AN); \n
 * \Example     InitAnalogPinsPortG(All_PINS_AN);
 *
 **********************************************************************************************************************/
#define InitAnalogPinsPortJ(AnalogPins)    ANSELJ = ~(AnalogPins)
#endif
#ifdef _ANSJ0
#define RJ0AN	0b1111111111111110	/* RJ0 bin is Analog bin */
#define _InitPinJ0AN(PinMode)    _ANSJ0 = (PinMode>>2);
#else
#define _InitPinJ0AN(PinMode)
#endif
#ifdef _ANSJ1
#define RJ1AN	0b1111111111111101	/* RJ1 bin is Analog bin */
#define _InitPinJ1AN(PinMode)    _ANSJ1 = (PinMode>>2);
#else
#define _InitPinJ1AN(PinMode)
#endif
#ifdef _ANSJ2
#define RJ2AN	0b1111111111111011	/* RJ2 bin is Analog bin */
#define _InitPinJ2AN(PinMode)    _ANSJ2 = (PinMode>>2);
#else
#define _InitPinJ2AN(PinMode)
#endif
#ifdef _ANSJ3
#define RJ3AN	0b1111111111110111	/* RJ3 bin is Analog bin */
#define _InitPinJ3AN(PinMode)    _ANSJ3 = (PinMode>>2);
#else
#define _InitPinJ3AN(PinMode)
#endif
#ifdef _ANSJ4
#define RJ4AN	0b1111111111101111	/* RJ4 bin is Analog bin */
#define _InitPinJ4AN(PinMode)    _ANSJ4 = (PinMode>>2);
#else
#define _InitPinJ4AN(PinMode)
#endif
#ifdef _ANSJ5
#define RJ5AN	0b1111111111011111	/* RJ5 bin is Analog bin */
#define _InitPinJ5AN(PinMode)    _ANSJ5 = (PinMode>>2);
#else
#define _InitPinJ5AN(PinMode)
#endif
#ifdef _ANSJ6
#define RJ6AN	0b1111111110111111	/* RJ6 bin is Analog bin */
#define _InitPinJ6AN(PinMode)    _ANSJ6 = (PinMode>>2);
#else
#define _InitPinJ6AN(PinMode)
#endif
#ifdef _ANSJ7
#define RJ7AN	0b1111111101111111	/* RJ7 bin is Analog bin */
#define _InitPinJ7AN(PinMode)    _ANSJ7 = (PinMode>>2);
#else
#define _InitPinJ7AN(PinMode)
#endif
#ifdef _ANSJ8
#define RJ8AN	0b1111111011111111	/* RJ8 bin is Analog bin */
#define _InitPinJ8AN(PinMode)    _ANSJ8 = (PinMode>>2);
#else
#define _InitPinJ8AN(PinMode)
#endif
#ifdef _ANSJ9
#define RJ9AN	0b1111110111111111	/* RJ9 bin is Analog bin */
#define _InitPinJ9AN(PinMode)    _ANSJ9 = (PinMode>>2);
#else
#define _InitPinJ9AN(PinMode)
#endif
#ifdef _ANSJ10
#define RJ10AN	0b1111101111111111	/* RJ10 bin is Analog bin */
#define _InitPinJ10AN(PinMode)    _ANSJ10 = (PinMode>>2);
#else
#define _InitPinJ10AN(PinMode)
#endif
#ifdef _ANSJ11
#define RJ11AN	0b1111011111111111	/* RJ11 bin is Analog bin */
#define _InitPinJ11AN(PinMode)    _ANSJ11 = (PinMode>>2);
#else
#define _InitPinJ11AN(PinMode)
#endif
#ifdef _ANSJ12
#define RJ12AN	0b1110111111111111	/* RJ12 bin is Analog bin */
#define _InitPinJ12AN(PinMode)    _ANSJ12 = (PinMode>>2);
#else
#define _InitPinJ12AN(PinMode)
#endif
#ifdef _ANSJ13
#define RJ13AN	0b1101111111111111	/* RJ13 bin is Analog bin */
#define _InitPinJ13AN(PinMode)    _ANSJ13 = (PinMode>>2);
#else
#define _InitPinJ13AN(PinMode)
#endif
#ifdef _ANSJ14
#define RJ14AN	0b1011111111111111	/* RJ14 bin is Analog bin */
#define _InitPinJ14AN(PinMode)    _ANSJ14 = (PinMode>>2);
#else
#define _InitPinJ14AN(PinMode)
#endif
#ifdef _ANSJ15
#define RJ15AN	0b0111111111111111	/* RJ15 bin is Analog bin */
#define _InitPinJ15AN(PinMode)    _ANSJ15 = (PinMode>>2);
#else
#define _InitPinJ15AN(PinMode)
#endif

#ifdef _ODCJ0
#define _InitPinJ0OD(PinMode)    _ODCJ0 = (PinMode&0b001);
#else
#define _InitPinJ0OD(PinMode)
#endif
#ifdef _ODCJ1
#define _InitPinJ1OD(PinMode)    _ODCJ1 = (PinMode&0b001);
#else
#define _InitPinJ1OD(PinMode)
#endif
#ifdef _ODCJ2
#define _InitPinJ2OD(PinMode)    _ODCJ2 = (PinMode&0b001);
#else
#define _InitPinJ2OD(PinMode)
#endif
#ifdef _ODCJ3
#define _InitPinJ3OD(PinMode)    _ODCJ3 = (PinMode&0b001);
#else
#define _InitPinJ3OD(PinMode)
#endif
#ifdef _ODCJ4
#define _InitPinJ4OD(PinMode)    _ODCJ4 = (PinMode&0b001);
#else
#define _InitPinJ4OD(PinMode)
#endif
#ifdef _ODCJ5
#define _InitPinJ5OD(PinMode)    _ODCJ5 = (PinMode&0b001);
#else
#define _InitPinJ5OD(PinMode)
#endif
#ifdef _ODCJ6
#define _InitPinJ6OD(PinMode)    _ODCJ6 = (PinMode&0b001);
#else
#define _InitPinJ6OD(PinMode)
#endif
#ifdef _ODCJ7
#define _InitPinJ7OD(PinMode)    _ODCJ7 = (PinMode&0b001);
#else
#define _InitPinJ7OD(PinMode)
#endif
#ifdef _ODCJ8
#define _InitPinJ8OD(PinMode)    _ODCJ8 = (PinMode&0b001);
#else
#define _InitPinJ8OD(PinMode)
#endif
#ifdef _ODCJ9
#define _InitPinJ9OD(PinMode)    _ODCJ9 = (PinMode&0b001);
#else
#define _InitPinJ9OD(PinMode)
#endif
#ifdef _ODCJ10
#define _InitPinJ10OD(PinMode)    _ODCJ10 = (PinMode&0b001);
#else
#define _InitPinJ10OD(PinMode)
#endif
#ifdef _ODCJ11
#define _InitPinJ11OD(PinMode)    _ODCJ11 = (PinMode&0b001);
#else
#define _InitPinJ11OD(PinMode)
#endif
#ifdef _ODCJ12
#define _InitPinJ12OD(PinMode)    _ODCJ12 = (PinMode&0b001);
#else
#define _InitPinJ12OD(PinMode)
#endif
#ifdef _ODCJ13
#define _InitPinJ13OD(PinMode)    _ODCJ13 = (PinMode&0b001);
#else
#define _InitPinJ13OD(PinMode)
#endif
#ifdef _ODCJ14
#define _InitPinJ14OD(PinMode)    _ODCJ14 = (PinMode&0b001);
#else
#define _InitPinJ14OD(PinMode)
#endif
#ifdef _ODCJ15
#define _InitPinJ15OD(PinMode)    _ODCJ15 = (PinMode&0b001);
#else
#define _InitPinJ15OD(PinMode)
#endif

#ifdef _CNIEJ0
#define _InitPinJ0PUD(PullUpDn)    _CNPUJ0 = (PullUpDn&0b01); _CNPDJ0 = (PullUpDn>>1);
#define _InitPinJ0CN(CNI_EN)          _CNIEJ0 = (CNI_EN);
#else
#define _InitPinJ0PUD(PullUpDn)
#define _InitPinJ0CN(CNI_EN)
#endif
#ifdef _CNIEJ1
#define _InitPinJ1PUD(PullUpDn)    _CNPUJ1 = (PullUpDn&0b01); _CNPDJ1 = (PullUpDn>>1);
#define _InitPinJ1CN(CNI_EN)          _CNIEJ1 = (CNI_EN);
#else
#define _InitPinJ1PUD(PullUpDn)
#define _InitPinJ1CN(CNI_EN)
#endif
#ifdef _CNIEJ2
#define _InitPinJ2PUD(PullUpDn)    _CNPUJ2 = (PullUpDn&0b01); _CNPDJ2 = (PullUpDn>>1);
#define _InitPinJ2CN(CNI_EN)          _CNIEJ2 = (CNI_EN);
#else
#define _InitPinJ2PUD(PullUpDn)
#define _InitPinJ2CN(CNI_EN)
#endif
#ifdef _CNIEJ3
#define _InitPinJ3PUD(PullUpDn)    _CNPUJ3 = (PullUpDn&0b01); _CNPDJ3 = (PullUpDn>>1);
#define _InitPinJ3CN(CNI_EN)          _CNIEJ3 = (CNI_EN);
#else
#define _InitPinJ3PUD(PullUpDn)
#define _InitPinJ3CN(CNI_EN)
#endif
#ifdef _CNIEJ4
#define _InitPinJ4PUD(PullUpDn)    _CNPUJ4 = (PullUpDn&0b01); _CNPDJ4 = (PullUpDn>>1);
#define _InitPinJ4CN(CNI_EN)          _CNIEJ4 = (CNI_EN);
#else
#define _InitPinJ4PUD(PullUpDn)
#define _InitPinJ4CN(CNI_EN)
#endif
#ifdef _CNIEJ5
#define _InitPinJ5PUD(PullUpDn)    _CNPUJ5 = (PullUpDn&0b01); _CNPDJ5 = (PullUpDn>>1);
#define _InitPinJ5CN(CNI_EN)          _CNIEJ5 = (CNI_EN);
#else
#define _InitPinJ5PUD(PullUpDn)
#define _InitPinJ5CN(CNI_EN)
#endif
#ifdef _CNIEJ6
#define _InitPinJ6PUD(PullUpDn)    _CNPUJ6 = (PullUpDn&0b01); _CNPDJ6 = (PullUpDn>>1);
#define _InitPinJ6CN(CNI_EN)          _CNIEJ6 = (CNI_EN);
#else
#define _InitPinJ6PUD(PullUpDn)
#define _InitPinJ6CN(CNI_EN)
#endif
#ifdef _CNIEJ7
#define _InitPinJ7PUD(PullUpDn)    _CNPUJ7 = (PullUpDn&0b01); _CNPDJ7 = (PullUpDn>>1);
#define _InitPinJ7CN(CNI_EN)          _CNIEJ7 = (CNI_EN);
#else
#define _InitPinJ7PUD(PullUpDn)
#define _InitPinJ7CN(CNI_EN)
#endif
#ifdef _CNIEJ8
#define _InitPinJ8PUD(PullUpDn)    _CNPUJ8 = (PullUpDn&0b01); _CNPDJ8 = (PullUpDn>>1);
#define _InitPinJ8CN(CNI_EN)          _CNIEJ8 = (CNI_EN);
#else
#define _InitPinJ8PUD(PullUpDn)
#define _InitPinJ8CN(CNI_EN)
#endif
#ifdef _CNIEJ9
#define _InitPinJ9PUD(PullUpDn)    _CNPUJ9 = (PullUpDn&0b01); _CNPDJ9 = (PullUpDn>>1);
#define _InitPinJ9CN(CNI_EN)          _CNIEJ9 = (CNI_EN);
#else
#define _InitPinJ9PUD(PullUpDn)
#define _InitPinJ9CN(CNI_EN)
#endif
#ifdef _CNIEJ10
#define _InitPinJ10PUD(PullUpDn)    _CNPUJ10 = (PullUpDn&0b01); _CNPDJ10 = (PullUpDn>>1);
#define _InitPinJ10CN(CNI_EN)          _CNIEJ10 = (CNI_EN);
#else
#define _InitPinJ10PUD(PullUpDn)
#define _InitPinJ10CN(CNI_EN)
#endif
#ifdef _CNIEJ11
#define _InitPinJ11PUD(PullUpDn)    _CNPUJ11 = (PullUpDn&0b01); _CNPDJ11 = (PullUpDn>>1);
#define _InitPinJ11CN(CNI_EN)          _CNIEJ11 = (CNI_EN);
#else
#define _InitPinJ11PUD(PullUpDn)
#define _InitPinJ11CN(CNI_EN)
#endif
#ifdef _CNIEJ12
#define _InitPinJ12PUD(PullUpDn)    _CNPUJ12 = (PullUpDn&0b01); _CNPDJ12 = (PullUpDn>>1);
#define _InitPinJ12CN(CNI_EN)          _CNIEJ12 = (CNI_EN);
#else
#define _InitPinJ12PUD(PullUpDn)
#define _InitPinJ12CN(CNI_EN)
#endif
#ifdef _CNIEJ13
#define _InitPinJ13PUD(PullUpDn)    _CNPUJ13 = (PullUpDn&0b01); _CNPDJ13 = (PullUpDn>>1);
#define _InitPinJ13CN(CNI_EN)          _CNIEJ13 = (CNI_EN);
#else
#define _InitPinJ13PUD(PullUpDn)
#define _InitPinJ13CN(CNI_EN)
#endif
#ifdef _CNIEJ14
#define _InitPinJ14PUD(PullUpDn)    _CNPUJ14 = (PullUpDn&0b01); _CNPDJ14 = (PullUpDn>>1);
#define _InitPinJ14CN(CNI_EN)          _CNIEJ14 = (CNI_EN);
#else
#define _InitPinJ14PUD(PullUpDn)
#define _InitPinJ14CN(CNI_EN)
#endif
#ifdef _CNIEJ15
#define _InitPinJ15PUD(PullUpDn)    _CNPUJ15 = (PullUpDn&0b01); _CNPDJ15 = (PullUpDn>>1);
#define _InitPinJ15CN(CNI_EN)          _CNIEJ15 = (CNI_EN);
#else
#define _InitPinJ15PUD(PullUpDn)
#define _InitPinJ15CN(CNI_EN)
#endif

#ifdef _RJ0
/***********************************************************************************************************************
 * \Function            void InitPinJ0(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ0 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ0(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ0(PinMode,PullUpDn,CNI_EN)  ({_TRISJ0 = ((PinMode>>1)&0b01); _InitPinJ0AN(PinMode) _InitPinJ0OD(PinMode) _InitPinJ0PUD(PullUpDn) _InitPinJ0CN(CNI_EN)})
#define ReadPinJ0()           (PORTJbits.RJ0)
#define SetPinJ0(Value)     LATJbits.LATJ0 = (Value)
#endif
#ifdef _RJ1
/***********************************************************************************************************************
 * \Function            void InitPinJ1(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ1 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ1(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ1(PinMode,PullUpDn,CNI_EN)  ({_TRISJ1 = ((PinMode>>1)&0b01); _InitPinJ1AN(PinMode)  _InitPinJ1OD(PinMode) _InitPinJ1PUD(PullUpDn) _InitPinJ1CN(CNI_EN)})
#define ReadPinJ1()           (PORTJbits.RJ1)
#define SetPinJ1(Value)     LATJbits.LATJ1 = (Value)
#endif
#ifdef _RJ2
/***********************************************************************************************************************
 * \Function            void InitPinJ2(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ2 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ2(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ2(PinMode,PullUpDn,CNI_EN)  ({_TRISJ2 = ((PinMode>>1)&0b01); _InitPinJ2AN(PinMode)  _InitPinJ2OD(PinMode) _InitPinJ2PUD(PullUpDn) _InitPinJ2CN(CNI_EN)})
#define ReadPinJ2()           (PORTJbits.RJ2)
#define SetPinJ2(Value)     LATJbits.LATJ2 = (Value)
#endif
#ifdef _RJ3
/***********************************************************************************************************************
 * \Function            void InitPinJ3(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ3 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ3(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ3(PinMode,PullUpDn,CNI_EN)  ({_TRISJ3 = ((PinMode>>1)&0b01); _InitPinJ3AN(PinMode)  _InitPinJ3OD(PinMode) _InitPinJ3PUD(PullUpDn) _InitPinJ3CN(CNI_EN)})
#define ReadPinJ3()           (PORTJbits.RJ3)
#define SetPinJ3(Value)     LATJbits.LATJ3 = (Value)
#endif
#ifdef _RJ4
/***********************************************************************************************************************
 * \Function            void InitPinJ4(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ4 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ4(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ4(PinMode,PullUpDn,CNI_EN)  ({_TRISJ4 = ((PinMode>>1)&0b01); _InitPinJ4AN(PinMode)  _InitPinJ4OD(PinMode) _InitPinJ4PUD(PullUpDn) _InitPinJ4CN(CNI_EN)})
#define ReadPinJ4()           (PORTJbits.RJ4)
#define SetPinJ4(Value)     LATJbits.LATJ4 = (Value)
#endif
#ifdef _RJ5
/***********************************************************************************************************************
 * \Function            void InitPinJ5(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ5 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ5(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ5(PinMode,PullUpDn,CNI_EN)  ({_TRISJ5 = ((PinMode>>1)&0b01); _InitPinJ5AN(PinMode)  _InitPinJ5OD(PinMode) _InitPinJ5PUD(PullUpDn) _InitPinJ5CN(CNI_EN)})
#define ReadPinJ5()           (PORTJbits.RJ5)
#define SetPinJ5(Value)     LATJbits.LATJ5 = (Value)
#endif
#ifdef _RJ6
/***********************************************************************************************************************
 * \Function            void InitPinJ6(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ6 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ6(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ6(PinMode,PullUpDn,CNI_EN)  ({_TRISJ6 = ((PinMode>>1)&0b01); _InitPinJ6AN(PinMode)  _InitPinJ6OD(PinMode) _InitPinJ6PUD(PullUpDn) _InitPinJ6CN(CNI_EN)})
#define ReadPinJ6()           (PORTJbits.RJ6)
#define SetPinJ6(Value)     LATJbits.LATJ6 = (Value)
#endif
#ifdef _RJ7
/***********************************************************************************************************************
 * \Function            void InitPinJ7(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ7 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ7(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ7(PinMode,PullUpDn,CNI_EN)  ({_TRISJ7 = ((PinMode>>1)&0b01); _InitPinJ7AN(PinMode)  _InitPinJ7OD(PinMode) _InitPinJ7PUD(PullUpDn) _InitPinJ7CN(CNI_EN)})
#define ReadPinJ7()           (PORTJbits.RJ7)
#define SetPinJ7(Value)     LATJbits.LATJ7 = (Value)
#endif
#ifdef _RJ8
/***********************************************************************************************************************
 * \Function            void InitPinJ8(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ8 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ8(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ8(PinMode,PullUpDn,CNI_EN)  ({_TRISJ8 = ((PinMode>>1)&0b01); _InitPinJ8AN(PinMode)  _InitPinJ8OD(PinMode) _InitPinJ8PUD(PullUpDn) _InitPinJ8CN(CNI_EN)})
#define ReadPinJ8()           (PORTJbits.RJ8)
#define SetPinJ8(Value)     LATJbits.LATJ8 = (Value)
#endif
#ifdef _RJ9
/***********************************************************************************************************************
 * \Function            void InitPinJ9(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ9 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ9(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ9(PinMode,PullUpDn,CNI_EN)  ({_TRISJ9 = ((PinMode>>1)&0b01); _InitPinJ9AN(PinMode)  _InitPinJ9OD(PinMode) _InitPinJ9PUD(PullUpDn) _InitPinJ9CN(CNI_EN)})
#define ReadPinJ9()           (PORTJbits.RJ9)
#define SetPinJ9(Value)     LATJbits.LATJ9 = (Value)
#endif
#ifdef _RJ10
/***********************************************************************************************************************
 * \Function            void InitPinJ10(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ10 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ10(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ10(PinMode,PullUpDn,CNI_EN)  ({_TRISJ10 = ((PinMode>>1)&0b01); _InitPinJ10AN(PinMode)  _InitPinJ10OD(PinMode) _InitPinJ10PUD(PullUpDn) _InitPinJ10CN(CNI_EN)})
#define ReadPinJ10()          (PORTJbits.RJ10)
#define SetPinJ10(Value)     LATJbits.LATJ10 = (Value)
#endif
#ifdef _RJ11
/***********************************************************************************************************************
 * \Function            void InitPinJ11(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ11 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ11(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ11(PinMode,PullUpDn,CNI_EN)  ({_TRISJ11 = ((PinMode>>1)&0b01); _InitPinJ11AN(PinMode)  _InitPinJ11OD(PinMode) _InitPinJ11PUD(PullUpDn) _InitPinJ11CN(CNI_EN)})
#define ReadPinJ11()          (PORTJbits.RJ11)
#define SetPinJ11(Value)     LATJbits.LATJ11 = (Value)
#endif
#ifdef _RJ12
/***********************************************************************************************************************
 * \Function            void InitPinJ12(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ12 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ12(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ12(PinMode,PullUpDn,CNI_EN)  ({_TRISJ12 = ((PinMode>>1)&0b01); _InitPinJ12AN(PinMode)  _InitPinJ12OD(PinMode) _InitPinJ12PUD(PullUpDn) _InitPinJ12CN(CNI_EN)})
#define ReadPinJ12()          (PORTJbits.RJ12)
#define SetPinJ12(Value)     LATJbits.LATJ12 = (Value)
#endif
#ifdef _RJ13
/***********************************************************************************************************************
 * \Function            void InitPinJ13(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ13 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ13(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ13(PinMode,PullUpDn,CNI_EN)  ({_TRISJ13 = ((PinMode>>1)&0b01); _InitPinJ13AN(PinMode)  _InitPinJ13OD(PinMode) _InitPinJ13PUD(PullUpDn) _InitPinJ13CN(CNI_EN)})
#define ReadPinJ13()          (PORTJbits.RJ13)
#define SetPinJ13(Value)     LATJbits.LATJ13 = (Value)
#endif
#ifdef _RJ14
/***********************************************************************************************************************
 * \Function            void InitPinJ14(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ14 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ14(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ14(PinMode,PullUpDn,CNI_EN)  ({_TRISJ14 = ((PinMode>>1)&0b01); _InitPinJ14AN(PinMode)  _InitPinJ14OD(PinMode) _InitPinJ14PUD(PullUpDn) _InitPinJ14CN(CNI_EN)})
#define ReadPinJ14()          (PORTJbits.RJ14)
#define SetPinJ14(Value)     LATJbits.LATJ14 = (Value)
#endif
#ifdef _RJ15
/***********************************************************************************************************************
 * \Function            void InitPinJ15(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTJ.RJ15 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinJ15(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinJ15(PinMode,PullUpDn,CNI_EN)  ({_TRISJ15 = ((PinMode>>1)&0b01); _InitPinJ15AN(PinMode)  _InitPinJ15OD(PinMode) _InitPinJ15PUD(PullUpDn) _InitPinJ15CN(CNI_EN)})
#define ReadPinJ15()          (PORTJbits.RJ15)
#define SetPinJ15(Value)     LATJbits.LATJ15 = (Value)
#endif

#endif
#ifdef  PORTK

#define ReadPORTK()           (PORTK)
#define SetPORTK(Value)     LATK = (Value)

#ifdef ANSELK
/***********************************************************************************************************************
 * \Function            void InitAnalogPinsPortK(AnalogPins)
 *
 * \Description         Set PortK Individual Pins Mode (Analog OR Digital).
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>AnalogPins:</u>\n
 *  - RK0AN to RK15AN   <i>/ The Corresponding Pin is Analog Pin.</i>   \n
 *  - All_PINS_DIG      <i>/ All PortK Pins is Digital.</i>   \n
 *  - All_PINS_AN       <i>/ All PortK Pins is Analog.</i>
 *
 * \Return      None
 *
 * \Notes       None
 *
 * \Example     InitAnalogPinsPortK(All_PINS_DIG & RK6AN & RK7AN); \n
 * \Example     InitAnalogPinsPortG(All_PINS_AN);
 *
 **********************************************************************************************************************/
#define InitAnalogPinsPortK(AnalogPins)    ANSELK = ~(AnalogPins)
#endif
#ifdef _ANSK0
#define RK0AN	0b1111111111111110	/* RK0 bin is Analog bin */
#define _InitPinK0AN(PinMode)    _ANSK0 = (PinMode>>2);
#else
#define _InitPinK0AN(PinMode)
#endif
#ifdef _ANSK1
#define RK1AN	0b1111111111111101	/* RK1 bin is Analog bin */
#define _InitPinK1AN(PinMode)    _ANSK1 = (PinMode>>2);
#else
#define _InitPinK1AN(PinMode)
#endif
#ifdef _ANSK2
#define RK2AN	0b1111111111111011	/* RK2 bin is Analog bin */
#define _InitPinK2AN(PinMode)    _ANSK2 = (PinMode>>2);
#else
#define _InitPinK2AN(PinMode)
#endif
#ifdef _ANSK3
#define RK3AN	0b1111111111110111	/* RK3 bin is Analog bin */
#define _InitPinK3AN(PinMode)    _ANSK3 = (PinMode>>2);
#else
#define _InitPinK3AN(PinMode)
#endif
#ifdef _ANSK4
#define RK4AN	0b1111111111101111	/* RK4 bin is Analog bin */
#define _InitPinK4AN(PinMode)    _ANSK4 = (PinMode>>2);
#else
#define _InitPinK4AN(PinMode)
#endif
#ifdef _ANSK5
#define RK5AN	0b1111111111011111	/* RK5 bin is Analog bin */
#define _InitPinK5AN(PinMode)    _ANSK5 = (PinMode>>2);
#else
#define _InitPinK5AN(PinMode)
#endif
#ifdef _ANSK6
#define RK6AN	0b1111111110111111	/* RK6 bin is Analog bin */
#define _InitPinK6AN(PinMode)    _ANSK6 = (PinMode>>2);
#else
#define _InitPinK6AN(PinMode)
#endif
#ifdef _ANSK7
#define RK7AN	0b1111111101111111	/* RK7 bin is Analog bin */
#define _InitPinK7AN(PinMode)    _ANSK7 = (PinMode>>2);
#else
#define _InitPinK7AN(PinMode)
#endif
#ifdef _ANSK8
#define RK8AN	0b1111111011111111	/* RK8 bin is Analog bin */
#define _InitPinK8AN(PinMode)    _ANSK8 = (PinMode>>2);
#else
#define _InitPinK8AN(PinMode)
#endif
#ifdef _ANSK9
#define RK9AN	0b1111110111111111	/* RK9 bin is Analog bin */
#define _InitPinK9AN(PinMode)    _ANSK9 = (PinMode>>2);
#else
#define _InitPinK9AN(PinMode)
#endif
#ifdef _ANSK10
#define RK10AN	0b1111101111111111	/* RK10 bin is Analog bin */
#define _InitPinK10AN(PinMode)    _ANSK10 = (PinMode>>2);
#else
#define _InitPinK10AN(PinMode)
#endif
#ifdef _ANSK11
#define RK11AN	0b1111011111111111	/* RK11 bin is Analog bin */
#define _InitPinK11AN(PinMode)    _ANSK11 = (PinMode>>2);
#else
#define _InitPinK11AN(PinMode)
#endif
#ifdef _ANSK12
#define RK12AN	0b1110111111111111	/* RK12 bin is Analog bin */
#define _InitPinK12AN(PinMode)    _ANSK12 = (PinMode>>2);
#else
#define _InitPinK12AN(PinMode)
#endif
#ifdef _ANSK13
#define RK13AN	0b1101111111111111	/* RK13 bin is Analog bin */
#define _InitPinK13AN(PinMode)    _ANSK13 = (PinMode>>2);
#else
#define _InitPinK13AN(PinMode)
#endif
#ifdef _ANSK14
#define RK14AN	0b1011111111111111	/* RK14 bin is Analog bin */
#define _InitPinK14AN(PinMode)    _ANSK14 = (PinMode>>2);
#else
#define _InitPinK14AN(PinMode)
#endif
#ifdef _ANSK15
#define RK15AN	0b0111111111111111	/* RK15 bin is Analog bin */
#define _InitPinK15AN(PinMode)    _ANSK15 = (PinMode>>2);
#else
#define _InitPinK15AN(PinMode)
#endif

#ifdef _ODCK0
#define _InitPinK0OD(PinMode)    _ODCK0 = (PinMode&0b001);
#else
#define _InitPinK0OD(PinMode)
#endif
#ifdef _ODCK1
#define _InitPinK1OD(PinMode)    _ODCK1 = (PinMode&0b001);
#else
#define _InitPinK1OD(PinMode)
#endif
#ifdef _ODCK2
#define _InitPinK2OD(PinMode)    _ODCK2 = (PinMode&0b001);
#else
#define _InitPinK2OD(PinMode)
#endif
#ifdef _ODCK3
#define _InitPinK3OD(PinMode)    _ODCK3 = (PinMode&0b001);
#else
#define _InitPinK3OD(PinMode)
#endif
#ifdef _ODCK4
#define _InitPinK4OD(PinMode)    _ODCK4 = (PinMode&0b001);
#else
#define _InitPinK4OD(PinMode)
#endif
#ifdef _ODCK5
#define _InitPinK5OD(PinMode)    _ODCK5 = (PinMode&0b001);
#else
#define _InitPinK5OD(PinMode)
#endif
#ifdef _ODCK6
#define _InitPinK6OD(PinMode)    _ODCK6 = (PinMode&0b001);
#else
#define _InitPinK6OD(PinMode)
#endif
#ifdef _ODCK7
#define _InitPinK7OD(PinMode)    _ODCK7 = (PinMode&0b001);
#else
#define _InitPinK7OD(PinMode)
#endif
#ifdef _ODCK8
#define _InitPinK8OD(PinMode)    _ODCK8 = (PinMode&0b001);
#else
#define _InitPinK8OD(PinMode)
#endif
#ifdef _ODCK9
#define _InitPinK9OD(PinMode)    _ODCK9 = (PinMode&0b001);
#else
#define _InitPinK9OD(PinMode)
#endif
#ifdef _ODCK10
#define _InitPinK10OD(PinMode)    _ODCK10 = (PinMode&0b001);
#else
#define _InitPinK10OD(PinMode)
#endif
#ifdef _ODCK11
#define _InitPinK11OD(PinMode)    _ODCK11 = (PinMode&0b001);
#else
#define _InitPinK11OD(PinMode)
#endif
#ifdef _ODCK12
#define _InitPinK12OD(PinMode)    _ODCK12 = (PinMode&0b001);
#else
#define _InitPinK12OD(PinMode)
#endif
#ifdef _ODCK13
#define _InitPinK13OD(PinMode)    _ODCK13 = (PinMode&0b001);
#else
#define _InitPinK13OD(PinMode)
#endif
#ifdef _ODCK14
#define _InitPinK14OD(PinMode)    _ODCK14 = (PinMode&0b001);
#else
#define _InitPinK14OD(PinMode)
#endif
#ifdef _ODCK15
#define _InitPinK15OD(PinMode)    _ODCK15 = (PinMode&0b001);
#else
#define _InitPinK15OD(PinMode)
#endif

#ifdef _CNIEK0
#define _InitPinK0PUD(PullUpDn)    _CNPUK0 = (PullUpDn&0b01); _CNPDK0 = (PullUpDn>>1);
#define _InitPinK0CN(CNI_EN)          _CNIEK0 = (CNI_EN);
#else
#define _InitPinK0PUD(PullUpDn)
#define _InitPinK0CN(CNI_EN)
#endif
#ifdef _CNIEK1
#define _InitPinK1PUD(PullUpDn)    _CNPUK1 = (PullUpDn&0b01); _CNPDK1 = (PullUpDn>>1);
#define _InitPinK1CN(CNI_EN)          _CNIEK1 = (CNI_EN);
#else
#define _InitPinK1PUD(PullUpDn)
#define _InitPinK1CN(CNI_EN)
#endif
#ifdef _CNIEK2
#define _InitPinK2PUD(PullUpDn)    _CNPUK2 = (PullUpDn&0b01); _CNPDK2 = (PullUpDn>>1);
#define _InitPinK2CN(CNI_EN)          _CNIEK2 = (CNI_EN);
#else
#define _InitPinK2PUD(PullUpDn)
#define _InitPinK2CN(CNI_EN)
#endif
#ifdef _CNIEK3
#define _InitPinK3PUD(PullUpDn)    _CNPUK3 = (PullUpDn&0b01); _CNPDK3 = (PullUpDn>>1);
#define _InitPinK3CN(CNI_EN)          _CNIEK3 = (CNI_EN);
#else
#define _InitPinK3PUD(PullUpDn)
#define _InitPinK3CN(CNI_EN)
#endif
#ifdef _CNIEK4
#define _InitPinK4PUD(PullUpDn)    _CNPUK4 = (PullUpDn&0b01); _CNPDK4 = (PullUpDn>>1);
#define _InitPinK4CN(CNI_EN)          _CNIEK4 = (CNI_EN);
#else
#define _InitPinK4PUD(PullUpDn)
#define _InitPinK4CN(CNI_EN)
#endif
#ifdef _CNIEK5
#define _InitPinK5PUD(PullUpDn)    _CNPUK5 = (PullUpDn&0b01); _CNPDK5 = (PullUpDn>>1);
#define _InitPinK5CN(CNI_EN)          _CNIEK5 = (CNI_EN);
#else
#define _InitPinK5PUD(PullUpDn)
#define _InitPinK5CN(CNI_EN)
#endif
#ifdef _CNIEK6
#define _InitPinK6PUD(PullUpDn)    _CNPUK6 = (PullUpDn&0b01); _CNPDK6 = (PullUpDn>>1);
#define _InitPinK6CN(CNI_EN)          _CNIEK6 = (CNI_EN);
#else
#define _InitPinK6PUD(PullUpDn)
#define _InitPinK6CN(CNI_EN)
#endif
#ifdef _CNIEK7
#define _InitPinK7PUD(PullUpDn)    _CNPUK7 = (PullUpDn&0b01); _CNPDK7 = (PullUpDn>>1);
#define _InitPinK7CN(CNI_EN)          _CNIEK7 = (CNI_EN);
#else
#define _InitPinK7PUD(PullUpDn)
#define _InitPinK7CN(CNI_EN)
#endif
#ifdef _CNIEK8
#define _InitPinK8PUD(PullUpDn)    _CNPUK8 = (PullUpDn&0b01); _CNPDK8 = (PullUpDn>>1);
#define _InitPinK8CN(CNI_EN)          _CNIEK8 = (CNI_EN);
#else
#define _InitPinK8PUD(PullUpDn)
#define _InitPinK8CN(CNI_EN)
#endif
#ifdef _CNIEK9
#define _InitPinK9PUD(PullUpDn)    _CNPUK9 = (PullUpDn&0b01); _CNPDK9 = (PullUpDn>>1);
#define _InitPinK9CN(CNI_EN)          _CNIEK9 = (CNI_EN);
#else
#define _InitPinK9PUD(PullUpDn)
#define _InitPinK9CN(CNI_EN)
#endif
#ifdef _CNIEK10
#define _InitPinK10PUD(PullUpDn)    _CNPUK10 = (PullUpDn&0b01); _CNPDK10 = (PullUpDn>>1);
#define _InitPinK10CN(CNI_EN)          _CNIEK10 = (CNI_EN);
#else
#define _InitPinK10PUD(PullUpDn)
#define _InitPinK10CN(CNI_EN)
#endif
#ifdef _CNIEK11
#define _InitPinK11PUD(PullUpDn)    _CNPUK11 = (PullUpDn&0b01); _CNPDK11 = (PullUpDn>>1);
#define _InitPinK11CN(CNI_EN)          _CNIEK11 = (CNI_EN);
#else
#define _InitPinK11PUD(PullUpDn)
#define _InitPinK11CN(CNI_EN)
#endif
#ifdef _CNIEK12
#define _InitPinK12PUD(PullUpDn)    _CNPUK12 = (PullUpDn&0b01); _CNPDK12 = (PullUpDn>>1);
#define _InitPinK12CN(CNI_EN)          _CNIEK12 = (CNI_EN);
#else
#define _InitPinK12PUD(PullUpDn)
#define _InitPinK12CN(CNI_EN)
#endif
#ifdef _CNIEK13
#define _InitPinK13PUD(PullUpDn)    _CNPUK13 = (PullUpDn&0b01); _CNPDK13 = (PullUpDn>>1);
#define _InitPinK13CN(CNI_EN)          _CNIEK13 = (CNI_EN);
#else
#define _InitPinK13PUD(PullUpDn)
#define _InitPinK13CN(CNI_EN)
#endif
#ifdef _CNIEK14
#define _InitPinK14PUD(PullUpDn)    _CNPUK14 = (PullUpDn&0b01); _CNPDK14 = (PullUpDn>>1);
#define _InitPinK14CN(CNI_EN)          _CNIEK14 = (CNI_EN);
#else
#define _InitPinK14PUD(PullUpDn)
#define _InitPinK14CN(CNI_EN)
#endif
#ifdef _CNIEK15
#define _InitPinK15PUD(PullUpDn)    _CNPUK15 = (PullUpDn&0b01); _CNPDK15 = (PullUpDn>>1);
#define _InitPinK15CN(CNI_EN)          _CNIEK15 = (CNI_EN);
#else
#define _InitPinK15PUD(PullUpDn)
#define _InitPinK15CN(CNI_EN)
#endif

#ifdef _RK0
/***********************************************************************************************************************
 * \Function            void InitPinK0(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK0 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK0(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK0(PinMode,PullUpDn,CNI_EN)  ({_TRISK0 = ((PinMode>>1)&0b01); _InitPinK0AN(PinMode) _InitPinK0OD(PinMode) _InitPinK0PUD(PullUpDn) _InitPinK0CN(CNI_EN)})
#define ReadPinK0()           (PORTKbits.RK0)
#define SetPinK0(Value)     LATKbits.LATK0 = (Value)
#endif
#ifdef _RK1
/***********************************************************************************************************************
 * \Function            void InitPinK1(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK1 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK1(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK1(PinMode,PullUpDn,CNI_EN)  ({_TRISK1 = ((PinMode>>1)&0b01); _InitPinK1AN(PinMode)  _InitPinK1OD(PinMode) _InitPinK1PUD(PullUpDn) _InitPinK1CN(CNI_EN)})
#define ReadPinK1()           (PORTKbits.RK1)
#define SetPinK1(Value)     LATKbits.LATK1 = (Value)
#endif
#ifdef _RK2
/***********************************************************************************************************************
 * \Function            void InitPinK2(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK2 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK2(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK2(PinMode,PullUpDn,CNI_EN)  ({_TRISK2 = ((PinMode>>1)&0b01); _InitPinK2AN(PinMode)  _InitPinK2OD(PinMode) _InitPinK2PUD(PullUpDn) _InitPinK2CN(CNI_EN)})
#define ReadPinK2()           (PORTKbits.RK2)
#define SetPinK2(Value)     LATKbits.LATK2 = (Value)
#endif
#ifdef _RK3
/***********************************************************************************************************************
 * \Function            void InitPinK3(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK3 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK3(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK3(PinMode,PullUpDn,CNI_EN)  ({_TRISK3 = ((PinMode>>1)&0b01); _InitPinK3AN(PinMode)  _InitPinK3OD(PinMode) _InitPinK3PUD(PullUpDn) _InitPinK3CN(CNI_EN)})
#define ReadPinK3()           (PORTKbits.RK3)
#define SetPinK3(Value)     LATKbits.LATK3 = (Value)
#endif
#ifdef _RK4
/***********************************************************************************************************************
 * \Function            void InitPinK4(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK4 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK4(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK4(PinMode,PullUpDn,CNI_EN)  ({_TRISK4 = ((PinMode>>1)&0b01); _InitPinK4AN(PinMode)  _InitPinK4OD(PinMode) _InitPinK4PUD(PullUpDn) _InitPinK4CN(CNI_EN)})
#define ReadPinK4()           (PORTKbits.RK4)
#define SetPinK4(Value)     LATKbits.LATK4 = (Value)
#endif
#ifdef _RK5
/***********************************************************************************************************************
 * \Function            void InitPinK5(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK5 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK5(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK5(PinMode,PullUpDn,CNI_EN)  ({_TRISK5 = ((PinMode>>1)&0b01); _InitPinK5AN(PinMode)  _InitPinK5OD(PinMode) _InitPinK5PUD(PullUpDn) _InitPinK5CN(CNI_EN)})
#define ReadPinK5()           (PORTKbits.RK5)
#define SetPinK5(Value)     LATKbits.LATK5 = (Value)
#endif
#ifdef _RK6
/***********************************************************************************************************************
 * \Function            void InitPinK6(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK6 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK6(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK6(PinMode,PullUpDn,CNI_EN)  ({_TRISK6 = ((PinMode>>1)&0b01); _InitPinK6AN(PinMode)  _InitPinK6OD(PinMode) _InitPinK6PUD(PullUpDn) _InitPinK6CN(CNI_EN)})
#define ReadPinK6()           (PORTKbits.RK6)
#define SetPinK6(Value)     LATKbits.LATK6 = (Value)
#endif
#ifdef _RK7
/***********************************************************************************************************************
 * \Function            void InitPinK7(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK7 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK7(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK7(PinMode,PullUpDn,CNI_EN)  ({_TRISK7 = ((PinMode>>1)&0b01); _InitPinK7AN(PinMode)  _InitPinK7OD(PinMode) _InitPinK7PUD(PullUpDn) _InitPinK7CN(CNI_EN)})
#define ReadPinK7()           (PORTKbits.RK7)
#define SetPinK7(Value)     LATKbits.LATK7 = (Value)
#endif
#ifdef _RK8
/***********************************************************************************************************************
 * \Function            void InitPinK8(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK8 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK8(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK8(PinMode,PullUpDn,CNI_EN)  ({_TRISK8 = ((PinMode>>1)&0b01); _InitPinK8AN(PinMode)  _InitPinK8OD(PinMode) _InitPinK8PUD(PullUpDn) _InitPinK8CN(CNI_EN)})
#define ReadPinK8()           (PORTKbits.RK8)
#define SetPinK8(Value)     LATKbits.LATK8 = (Value)
#endif
#ifdef _RK9
/***********************************************************************************************************************
 * \Function            void InitPinK9(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK9 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK9(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK9(PinMode,PullUpDn,CNI_EN)  ({_TRISK9 = ((PinMode>>1)&0b01); _InitPinK9AN(PinMode)  _InitPinK9OD(PinMode) _InitPinK9PUD(PullUpDn) _InitPinK9CN(CNI_EN)})
#define ReadPinK9()           (PORTKbits.RK9)
#define SetPinK9(Value)     LATKbits.LATK9 = (Value)
#endif
#ifdef _RK10
/***********************************************************************************************************************
 * \Function            void InitPinK10(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK10 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK10(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK10(PinMode,PullUpDn,CNI_EN)  ({_TRISK10 = ((PinMode>>1)&0b01); _InitPinK10AN(PinMode)  _InitPinK10OD(PinMode) _InitPinK10PUD(PullUpDn) _InitPinK10CN(CNI_EN)})
#define ReadPinK10()          (PORTKbits.RK10)
#define SetPinK10(Value)     LATKbits.LATK10 = (Value)
#endif
#ifdef _RK11
/***********************************************************************************************************************
 * \Function            void InitPinK11(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK11 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK11(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK11(PinMode,PullUpDn,CNI_EN)  ({_TRISK11 = ((PinMode>>1)&0b01); _InitPinK11AN(PinMode)  _InitPinK11OD(PinMode) _InitPinK11PUD(PullUpDn) _InitPinK11CN(CNI_EN)})
#define ReadPinK11()          (PORTKbits.RK11)
#define SetPinK11(Value)     LATKbits.LATK11 = (Value)
#endif
#ifdef _RK12
/***********************************************************************************************************************
 * \Function            void InitPinK12(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK12 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK12(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK12(PinMode,PullUpDn,CNI_EN)  ({_TRISK12 = ((PinMode>>1)&0b01); _InitPinK12AN(PinMode)  _InitPinK12OD(PinMode) _InitPinK12PUD(PullUpDn) _InitPinK12CN(CNI_EN)})
#define ReadPinK12()          (PORTKbits.RK12)
#define SetPinK12(Value)     LATKbits.LATK12 = (Value)
#endif
#ifdef _RK13
/***********************************************************************************************************************
 * \Function            void InitPinK13(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK13 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK13(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK13(PinMode,PullUpDn,CNI_EN)  ({_TRISK13 = ((PinMode>>1)&0b01); _InitPinK13AN(PinMode)  _InitPinK13OD(PinMode) _InitPinK13PUD(PullUpDn) _InitPinK13CN(CNI_EN)})
#define ReadPinK13()          (PORTKbits.RK13)
#define SetPinK13(Value)     LATKbits.LATK13 = (Value)
#endif
#ifdef _RK14
/***********************************************************************************************************************
 * \Function            void InitPinK14(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK14 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK14(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK14(PinMode,PullUpDn,CNI_EN)  ({_TRISK14 = ((PinMode>>1)&0b01); _InitPinK14AN(PinMode)  _InitPinK14OD(PinMode) _InitPinK14PUD(PullUpDn) _InitPinK14CN(CNI_EN)})
#define ReadPinK14()          (PORTKbits.RK14)
#define SetPinK14(Value)     LATKbits.LATK14 = (Value)
#endif
#ifdef _RK15
/***********************************************************************************************************************
 * \Function            void InitPinK15(PinMode,PullUpDn,CNI_EN)
 *
 * \Description         Set Up PORTK.RK15 Pin Mode (Analog/Digita & Input/Output & Totem Pole/Open Drain & Pulling & CN interrupt)
 *
 * \PreCondition        None
 *
 * \Inputs
 * <u>PinMode:</u>\n
 *  - ANA_INPUT         <i>/ Set The Pin as Analog Input (DEFUALT).</i>     \n
 *  - DIG_INPUT         <i>/ Set The Pin as Digital Input.</i>              \n
 *  - DIG_OUTPUT        <i>/ Set The Pin as Digital Totem Pole Output.</i>  \n
 *  - DIG_OUTPUT_OD     <i>/ Set The Pin as Digital Open Drain Output.</i>
 *
 * <u>PullUpDn:</u>\n
 *  - PULL_UP           <i>/ Pull the Pin Up To VCC By Weak Resistor.</i>   \n
 *  - PULL_DN           <i>/ Pull the Pin Down To GND By Weak Resistor.</i> \n
 *  - NO_PULL           <i>/ Dont Pull this Pin (DEFUALT).</i>
 *
 * <u>CNI_EN:</u>\n
 *  - CN_EN             <i>/ Enable Change Notification Interrupt for this Pin.</i>   \n
 *  - CN_DIS            <i>/ Disable Change Notification Interrupt for this Pin.</i>
 *
 * \Return      None
 *
 * \Notes       - If the pin is set as analog input and it was not an analog pin it will be set automatically to digital input.\n
 *              - The open-drain I/O feature is not supported on pins that are not 5V tolerant.
 *
 * \Example     InitPinK15(ANA_INPUT,NO_PULL,CN_DIS);
 *
 **********************************************************************************************************************/
#define InitPinK15(PinMode,PullUpDn,CNI_EN)  ({_TRISK15 = ((PinMode>>1)&0b01); _InitPinK15AN(PinMode)  _InitPinK15OD(PinMode) _InitPinK15PUD(PullUpDn) _InitPinK15CN(CNI_EN)})
#define ReadPinK15()          (PORTKbits.RK15)
#define SetPinK15(Value)     LATKbits.LATK15 = (Value)
#endif

#endif

#define CN_EnableInt()                  (_CNIE = 1)
#define CN_DisableInt()                 (_CNIE = 0)
#define CN_SetPriorityInt(priority)     (_CNIP = priority)
#define CN_INT_FLAG                     (_CNIF)

// <editor-fold defaultstate="collapsed" desc="Ports Function Parameters">

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*InitPinXY*/

// PinMode.
#define ANA_IN       0b110   /* The pin is anallog input (DEFUALT) */
#define DIG_IN       0b010   /* The pin is digital input */
#define DIG_OUT      0b000   /* The pin is totem pole digital output */
#define DIG_OUT_OD   0b001   /* The pin is open drain digital output */
// PullUpDn.
#define PULL_UP         0b01    /* Pull the Pin Up To VCC By Weak Resistor */
#define PULL_DN         0b10    /* Pull the Pin Down To GND By Weak Resistor */
#define NO_PULL         0b00    /* Dont Pull this Pin (DEFUALT) */
// CNI_EN.
#define CN_EN           0b1     /* Enable Change Notification Interrupt for this Pin */
#define CN_DIS          0b0     /* Disable Change Notification Interrupt for this Pin (DEFUALT) */

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*InitAnalogPinsPortX*/

// AnalogPins
#define All_PINS_DIG	0xFFFF	/* All Port Pins Is Digital */
#define All_PINS_AN	0x0000	/* All Port Pins Is Analog */
// </editor-fold>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="PPS Defines And Functions">
/***********************************************************************************************************************
 * \Function            void PPS_Mapping(const RemappablePinName, const PeripheralIO_Name)
 *
 * \Description         Sets desired internal MCU module to be mapped on the requested pins.
 *
 * \PreCondition        User must call PPS_Unlock() before this function call and PPS_Lock() after it.
 *
 * \Inputs
 * <u>RemappablePinName:</u>\n
 *  - CONNECT_RP20_TO .... CONNECT_RP127_TO      <i>/ RPn For Peripheral Input and Output .</i>    \n
 *  - CONNECT_RPI16_TO .. CONNECT_RPI127_TO      <i>/ RPIn For Peripheral Input Only.(if You Assign It For Output NOTHING WILL BE ASSIGNED)</i>\n
 *  - CONNECT_INTERNAL_VSS_TO          <i>/ Virtual Connect to VSS.</i>                    \n
 *  - CONNECT_INTERNAL_C1OUT_TO        <i>/ Virtual Connect to Comparator 1 Output.</i>    \n
 *  - CONNECT_INTERNAL_C2OUT_TO        <i>/ Virtual Connect to Comparator 2 Output.</i>    \n
 *  - CONNECT_INTERNAL_C3OUT_TO        <i>/ Virtual Connect to Comparator 3 Output.</i>    \n
 *  - CONNECT_INTERNAL_C4OUT_TO        <i>/ Virtual Connect to Comparator 4 Output.</i>    \n
 *  - CONNECT_INTERNAL_C5OUT_TO        <i>/ Virtual Connect to Comparator 5 Output.</i>    \n
 *  - CONNECT_INTERNAL_PTGO30_TO        <i>/ Virtual Connect to Peripheral Trigar Generator 30 Output.</i>    \n
 *  - CONNECT_INTERNAL_PTGO31_TO        <i>/ Virtual Connect to Peripheral Trigar Generator 31 Output.</i>    \n
 *  - CONNECT_INTERNAL_FINDX1_TO        <i>/ Virtual Connect to QEI1 Filterd Index Output.</i>     \n
 *  - CONNECT_INTERNAL_FHOME1_TO        <i>/ Virtual Connect to QEI1 Filterd Home Output.</i>      \n
 *  - CONNECT_INTERNAL_FINDX2_TO        <i>/ Virtual Connect to QEI2 Filterd Index Output.</i>     \n
 *  - CONNECT_INTERNAL_FHOME2_TO        <i>/ Virtual Connect to QEI2 Filterd Home Output.</i>      \n
 *
 *
 * <u>PeripheralIO_Name:</u>\n
 *  - INT1_INPUT            <i>/ Assign External Interrupt 1 (INTR1) to the corresponding RPn pin.</i>   \n
 *  - INT2_INPUT            <i>/ Assign External Interrupt 2 (INTR2) to the corresponding RPn pin.</i>   \n
 *  - INT3_INPUT            <i>/ Assign External Interrupt 3 (INTR3) to the corresponding RPn pin.</i>   \n
 *  - INT4_INPUT            <i>/ Assign External Interrupt 4 (INTR4) to the corresponding RPn pin.</i>   \n
 *  - T1CK_INPUT            <i>/ Assign Timer1 External Clock (T1CK) to the corresponding RPn pin.</i>   \n
 *  - T2CK_INPUT            <i>/ Assign Timer2 External Clock (T2CK) to the corresponding RPn pin.</i>   \n
 *  - T3CK_INPUT            <i>/ Assign Timer3 External Clock (T3CK) to the corresponding RPn pin.</i>   \n
 *  - T4CK_INPUT            <i>/ Assign Timer4 External Clock (T4CK) to the corresponding RPn pin.</i>   \n
 *  - T5CK_INPUT            <i>/ Assign Timer5 External Clock (T5CK) to the corresponding RPn pin.</i>   \n
 *  - T6CK_INPUT            <i>/ Assign Timer5 External Clock (T5CK) to the corresponding RPn pin.</i>   \n
 *  - T7CK_INPUT            <i>/ Assign Timer5 External Clock (T5CK) to the corresponding RPn pin.</i>   \n
 *  - T8CK_INPUT            <i>/ Assign Timer5 External Clock (T5CK) to the corresponding RPn pin.</i>   \n
 *  - T9CK_INPUT            <i>/ Assign Timer5 External Clock (T5CK) to the corresponding RPn pin.</i>   \n
 *  - IC1_INPUT             <i>/ Assign Input Capture 1 (IC1) to the corresponding RPn pin.</i>   \n
 *  - IC2_INPUT             <i>/ Assign Input Capture 2 (IC2) to the corresponding RPn pin.</i>   \n
 *  - IC3_INPUT             <i>/ Assign Input Capture 3 (IC3) to the corresponding RPn pin.</i>   \n
 *  - IC4_INPUT             <i>/ Assign Input Capture 4 (IC4) to the corresponding RPn pin.</i>   \n
 *  - IC5_INPUT             <i>/ Assign Input Capture 5 (IC5) to the corresponding RPn pin.</i>   \n
 *  - IC6_INPUT             <i>/ Assign Input Capture 6 (IC6) to the corresponding RPn pin.</i>   \n
 *  - IC7_INPUT             <i>/ Assign Input Capture 7 (IC7) to the corresponding RPn pin.</i>   \n
 *  - IC8_INPUT             <i>/ Assign Input Capture 8 (IC8) to the corresponding RPn pin.</i>   \n
 *  - IC9_INPUT             <i>/ Assign Input Capture 9 (IC9) to the corresponding RPn pin.</i> \n
 *  - IC10_INPUT            <i>/ Assign Input Capture 10 (IC10) to the corresponding RPn pin.</i>   \n
 *  - IC11_INPUT            <i>/ Assign Input Capture 11 (IC11) to the corresponding RPn pin.</i>   \n
 *  - IC12_INPUT            <i>/ Assign Input Capture 12 (IC12) to the corresponding RPn pin.</i>   \n
 *  - IC13_INPUT            <i>/ Assign Input Capture 13 (IC13) to the corresponding RPn pin.</i>   \n
 *  - IC14_INPUT            <i>/ Assign Input Capture 14 (IC14) to the corresponding RPn pin.</i>   \n
 *  - IC15_INPUT            <i>/ Assign Input Capture 15 (IC15) to the corresponding RPn pin.</i>   \n
 *  - IC16_INPUT            <i>/ Assign Input Capture 16 (IC16) to the corresponding RPn pin.</i>   \n
 *  - SDI1_INPUT            <i>/ Assign SPI1 Data Input (SDI1) to the corresponding RPn pin.</i>   \n
 *  - SCK1_INPUT            <i>/ Assign SPI1 Clock Input (SCK1IN) to the corresponding RPn pin.</i>   \n
 *  - SS1_INPUT             <i>/ Assign SPI1 Slave Select Input (SS1IN) to the corresponding RPn pin.</i>   \n
 *  - SDI2_INPUT            <i>/ Assign SPI2 Data Input (SDI2) to the corresponding RPn pin.</i>   \n
 *  - SCK2_INPUT            <i>/ Assign SPI2 Clock Input (SCK2IN) to the corresponding RPn pin.</i>   \n
 *  - SS2_INPUT             <i>/ Assign SPI2 Slave Select Input (SS2IN) to the corresponding RPn pin.</i>   \n
 *  - SDI3_INPUT            <i>/ Assign SPI3 Data Input (SDI3) to the corresponding RPn pin.</i>    \n
 *  - SCK3_INPUT            <i>/ Assign SPI3 Clock Input (SCK3IN) to the corresponding RPn pin.</i> \n
 *  - SS3_INPUT             <i>/ Assign SPI3 Slave Select Input (SS3IN) to the corresponding RPn pin.</i>   \n
 *  - SDI4_INPUT            <i>/ Assign SPI4 Data Input (SDI4) to the corresponding RPn pin.</i>    \n
 *  - SCK4_INPUT            <i>/ Assign SPI4 Clock Input (SCK4IN) to the corresponding RPn pin.</i> \n
 *  - SS4_INPUT             <i>/ Assign SPI4 Slave Select Input (SS4IN) to the corresponding RPn pin.</i>   \n
 *  - OCFA_INPUT            <i>/ Assign Output Capture A (OCFA) to the corresponding RPn pin.</i>   \n
 *  - OCFB_INPUT            <i>/ Assign Output Capture B (OCFB) to the corresponding RPn pin.</i>   \n
 *  - FLT1_INPUT            <i>/ Assign PWM1 Fault (FLT1) to the corresponding RPn pin.</i>   \n
 *  - FLT2_INPUT            <i>/ Assign PWM2 Fault (FLT2) to the corresponding RPn pin.</i>   \n
 *  - FLT3_INPUT            <i>/ Assign PWM3 Fault (FLT3) to the corresponding RPn pin.</i>   \n
 *  - FLT4_INPUT            <i>/ Assign PWM4 Fault (FLT4) to the corresponding RPn pin.</i>   \n
 *  - FLT5_INPUT            <i>/ Assign PWM5 Fault (FLT5) to the corresponding RPn pin.</i> \n
 *  - FLT6_INPUT            <i>/ Assign PWM6 Fault (FLT6) to the corresponding RPn pin.</i> \n
 *  - FLT7_INPUT            <i>/ Assign PWM7 Fault (FLT7) to the corresponding RPn pin.</i> \n
 *  - DTCMP1_INPUT          <i>/ Assign PWM Dead Time Compensation 1 (DTCMP1) to the corresponding RPn pin.</i> \n
 *  - DTCMP2_INPUT          <i>/ Assign PWM Dead Time Compensation 2 (DTCMP2) to the corresponding RPn pin.</i> \n
 *  - DTCMP3_INPUT          <i>/ Assign PWM Dead Time Compensation 3 (DTCMP3) to the corresponding RPn pin.</i> \n
 *  - DTCMP4_INPUT          <i>/ Assign PWM Dead Time Compensation 4 (DTCMP4) to the corresponding RPn pin.</i> \n
 *  - DTCMP5_INPUT          <i>/ Assign PWM Dead Time Compensation 5 (DTCMP5) to the corresponding RPn pin.</i> \n
 *  - DTCMP6_INPUT          <i>/ Assign PWM Dead Time Compensation 6 (DTCMP6) to the corresponding RPn pin.</i> \n
 *  - DTCMP7_INPUT          <i>/ Assign PWM Dead Time Compensation 7 (DTCMP7) to the corresponding RPn pin.</i> \n
 *  - SYNCI1_INPUT          <i>/ Assign PWM Sync Input 1 (SYNCI1) to the corresponding RPn pin.</i> \n
 *  - SYNCI2_INPUT          <i>/ Assign PWM Sync Input 2 (SYNCI2) to the corresponding RPn pin.</i> \n
 *  - QEA1_INPUT            <i>/ Assign A (QEA) to the corresponding pin.</i>   \n
 *  - QEB1_INPUT            <i>/ Assign B (QEB) to the corresponding pin.</i>   \n
 *  - QEI1_INPUT            <i>/ Assign QEI INDEX (INDX) to the corresponding RPn pin.</i>   \n
 *  - QEH1_INPUT            <i>/ Assign QEI HOME to the corresponding RPn pin.</i>   \n
 *  - QEA2_INPUT            <i>/ Assign A (QEA) to the corresponding pin.</i>   \n
 *  - QEB2_INPUT            <i>/ Assign B (QEB) to the corresponding pin.</i>   \n
 *  - QEI2_INPUT            <i>/ Assign QEI INDEX (INDX) to the corresponding RPn pin.</i>   \n
 *  - QEH2_INPUT            <i>/ Assign QEI HOME to the corresponding RPn pin.</i>   \n
 *  - U1RX_INPUT            <i>/ Assign UART1 Receive (U1RX) to the corresponding RPn pin.</i>   \n
 *  - U2RX_INPUT            <i>/ Assign UART2 Receive (U2RX) to the corresponding RPn pin.</i>   \n
 *  - U3RX_INPUT            <i>/ Assign UART3 Receive (U3RX) to the corresponding RPn pin.</i>  \n
 *  - U4RX_INPUT            <i>/ Assign UART4 Receive (U4RX) to the corresponding RPn pin.</i>  \n
 *  - U1CTS_INPUT           <i>/ Assign UART1 Clear to Send (U1CTS) to the corresponding RPn pin.</i>   \n
 *  - U2CTS_INPUT           <i>/ Assign UART2 Clear to Send (U2CTS) to the corresponding RPn pin.</i>   \n
 *  - U3CTS_INPUT           <i>/ Assign UART3 Clear to Send (U3CTS) to the corresponding RPn pin.</i>   \n
 *  - U4CTS_INPUT           <i>/ Assign UART4 Clear to Send (U4CTS) to the corresponding RPn pin.</i>   \n
 *  - CSDI_INPUT            <i>/ Assign DCI Serial Data Input (CSDIN) to the corresponding RPn pin.</i> \n
 *  - CSCK_INPUT            <i>/ Assign DCI Serial Clock Input (CSCKIN) to the corresponding RPn pin.</i>   \n
 *  - COFS_INPUT            <i>/ Assign DCI Frame Sync Input (COFSIN) to the corresponding RPn pin.</i> \n
 *  - C1RX_INPUT            <i>/ Assign ECAN1 Receive Input (C1RX) to the corresponding RPn pin.</i>    \n
 *  - C2RX_INPUT            <i>/ Assign ECAN2 Receive Input (C2RX) to the corresponding RPn pin.</i>    \n
 *  - OCFC_INPUT            <i>/ Assign Output Capture Compare Fault C (OCFC) to the corresponding RPn pin.</i>   \n
 *  - SENT1_INPUT           <i>/ Assign SENT module input 1 (SENT1) to the corresponding RPn pin.</i>   \n
 *  - SENT2_INPUT           <i>/ Assign SENT module input 2 (SENT2) to the corresponding RPn pin.</i>   \n
 *
 *  - NULL_OUTPUT           <i>/ RPn tied to default port pin .</i>  \n
 *  - U1TX_OUTPUT           <i>/ RPn tied to UART1 Transmit .</i>    \n
 *  - U1RTS_OUTPUT          <i>/ RPn tied to UART1 Ready To Send .</i>   \n
 *  - U2TX_OUTPUT           <i>/ RPn tied to UART2 Transmit .</i>    \n
 *  - U2RTS_OUTPUT          <i>/ RPn tied to UART2 Ready To Send .</i>   \n
 *  - U3TX_OUTPUT           <i>/ RPn tied to UART3 Transmit .</i>    \n
 *  - U3RTS_OUTPUT          <i>/ RPn tied to UART3 Ready To Send .</i>   \n
 *  - U4TX_OUTPUT           <i>/ RPn tied to UART4 Transmit .</i>    \n
 *  - U4RTS_OUTPUT          <i>/ RPn tied to UART4 Ready To Send .</i>   \n
 *  - SDO1_OUTPUT           <i>/ RPn tied to SPI1 Data Output .</i>  \n
 *  - SCK1_OUTPUT           <i>/ RPn tied to SPI1 Clock Output .</i> \n
 *  - SS1_OUTPUT            <i>/ RPn tied to SPI1 Slave Select Output .</i>  \n
 *  - SDO2_OUTPUT           <i>/ RPn tied to SPI2 Data Output .</i>  \n
 *  - SCK2_OUTPUT           <i>/ RPn tied to SPI2 Clock Output .</i> \n
 *  - SS2_OUTPUT            <i>/ RPn tied to SPI2 Slave Select Output .</i>  \n
 *  - SDO3_OUTPUT           <i>/ RPn tied to SPI3 Data Output .</i>  \n
 *  - SCK3_OUTPUT           <i>/ RPn tied to SPI3 Clock Output .</i> \n
 *  - SS3_OUTPUT            <i>/ RPn tied to SPI3 Slave Select Output .</i>  \n
 *  - SDO4_OUTPUT           <i>/ RPn tied to SPI4 Data Output .</i>  \n
 *  - SCK4_OUTPUT           <i>/ RPn tied to SPI4 Clock Output .</i> \n
 *  - SS4_OUTPUT            <i>/ RPn tied to SPI4 Slave Select Output .</i>  \n
 *  - CSDO_OUTPUT           <i>/ RPn tied to DCI Serial Data Output.</i> \n
 *  - CSCKOUT_OUTPUT        <i>/ RPn tied to DCI Serial Clock Output.</i>    \n
 *  - COFSOUT_OUTPUT        <i>/ RPn tied to DCI Frame Sync Output.</i>  \n
 *  - C1TX_OUTPUT           <i>/ RPn tied to ECAN1 Transmit .</i>    \n
 *  - C2TX_OUTPUT           <i>/ RPn tied to ECAN2 Transmit .</i>    \n
 *  - OC1_OUTPUT            <i>/ RPn tied to Output Compare 1 .</i>  \n
 *  - OC2_OUTPUT            <i>/ RPn tied to Output Compare 2 .</i>  \n
 *  - OC3_OUTPUT            <i>/ RPn tied to Output Compare 3 .</i>  \n
 *  - OC4_OUTPUT            <i>/ RPn tied to Output Compare 4 .</i>  \n
 *  - OC5_OUTPUT            <i>/ RPn tied to Output Compare 5 .</i>  \n
 *  - OC6_OUTPUT            <i>/ RPn tied to Output Compare 6 .</i>  \n
 *  - OC7_OUTPUT            <i>/ RPn tied to Output Compare 7 .</i>  \n
 *  - OC8_OUTPUT            <i>/ RPn tied to Output Compare 8 .</i>  \n
 *  - OC9_OUTPUT            <i>/ RPn tied to Output Compare 9 .</i>  \n
 *  - OC10_OUTPUT           <i>/ RPn tied to Output Compare 10 .</i> \n
 *  - OC11_OUTPUT           <i>/ RPn tied to Output Compare 11 .</i> \n
 *  - OC12_OUTPUT           <i>/ RPn tied to Output Compare 12 .</i> \n
 *  - OC13_OUTPUT           <i>/ RPn tied to Output Compare 13 .</i> \n
 *  - OC14_OUTPUT           <i>/ RPn tied to Output Compare 14 .</i> \n
 *  - OC15_OUTPUT           <i>/ RPn tied to Output Compare 15 .</i> \n
 *  - OC16_OUTPUT           <i>/ RPn tied to Output Compare 16 .</i> \n
 *  - C1OUT_OUTPUT          <i>/ RPn tied to Comparator1 Output .</i>    \n
 *  - C2OUT_OUTPUT          <i>/ RPn tied to Comparator2 Output .</i>    \n
 *  - C3OUT_OUTPUT          <i>/ RPn tied to Comparator3 Output .</i>    \n
 *  - SYNCO1_OUTPUT         <i>/ RPn tied to Output SYNCO1 .</i> \n
 *  - SYNCO2_OUTPUT         <i>/ RPn tied to Output SYNCO2 .</i> \n
 *  - QEI1CCMP_OUTPUT       <i>/ RPn tied to Output QEI1CCMP .</i>   \n
 *  - QEI2CCMP_OUTPUT       <i>/ RPn tied to Output QEI2CCMP .</i>   \n
 *  - REFCLKO_OUTPUT        <i>/ RPn tied to Output REFCLKO .</i>    \n
 *  - SENT1_OUTPUT          <i>/ RPn tied to SENT1 Output .</i>  \n
 *  - SENT2_OUTPUT          <i>/ RPn tied to SENT2 Output .</i>  \n
 *  - UPDN1_OUTPUT          <i>/ RPn tied to QEI1 UPDN Output .</i>  \n
 *  - UPDN2_OUTPUT          <i>/ RPn tied to QEI2 UPDN Output .</i>  \n
 *
 * \Return      None
 *
 * \Notes
 * 0- Look at Device Datasheet For Avaliable PR and RPI pins.
 *
 * 1- Remappable peripherals never take priority over any analog functions associated with the pin.
 *
 * 2- For input only, Peripheral Pin Select functionality does not have priority over TRISx settings. Therefore, when configuring RPn/RPIn pin for
 * input, the corresponding bit in the TRISx register must also be configured for input (set to "1").
 *
 * 3- Remappable peripherals Laws:\n
 * a) Only one "output" function can be active on a given pin at any time regardless if it is a dedicated or remappable function (one pin, one output).
 *
 * b) It is possible to assign a "remappable output" function to multiple pins and externally short or tie them together for increased current drive.
 *
 * c) If any "dedicated output" function is enabled on a pin, it will take precedence over any remappable "output" function.
 *
 * d) If any "dedicated digital" (input or output) function is enabled on a pin, any number of "input" remappable functions can be mapped to the same pin.
 *
 * e) If any "dedicated analog" function(s) are enabled on a given pin, "digital input(s)" of any kind will all be disabled, although a single "digital
 * output", at the user"s cautionary discretion, can be enabled and active as long as there is no signal contention with an external analog
 * input signal. For example, it is possible for the ADC to convert the digital output logic level, or to toggle a digital output on a comparator or
 * ADC input, provided there is no external analog input, such as for a built-in self test.
 *
 * f) Any number of "input" remappable functions can be mapped to the same pin(s) at the same time, including any pin with a single output from
 * either a dedicated or remappable "output".
 *
 * \Examples    PPS_Mapping(CONNECT_RPI16_TO, SDI1_INPUT);				\n
 *              PPS_Mapping(CONNECT_RP64_TO, SDO1_OUTPUT);				\n
 *              PPS_Mapping(CONNECT_RP100_TO, IC7_INPUT);				\n
 *              PPS_Mapping(CONNECT_INTERNAL_C3OUT_TO, FLT1_INPUT);			\n
 *              PPS_Mapping(CONNECT_INTERNAL_VSS_TO, U1CTS_INPUT);			\n
 *		<b>(Wrong Example)</b>PPS_Mapping(CONNECT_RPI20_TO, SDO1_OUTPUT);	\n
 *		Because I Assigned RPIn Pin To Peripheral Output So Nothing Will Happen.
 **********************************************************************************************************************/
#define PPS_Mapping(RemappablePinName, PeripheralIO_Name) ({					\
static unsigned int FakeReg;	FakeReg=0;							\
if(PeripheralIO_Name < 0x1000)	{SHADOW_##PeripheralIO_Name = RemappablePinName; }		\
else				{SHADOW_##RemappablePinName = (PeripheralIO_Name-0x1000); }	\
})

/*CONNECT_RPI16_TO, T2CK_INPUT*/    /*16, RPINR3bits.T2CKR*/    /*RPINR3bits.T2CKR = 16*/
/*
 * SHADOW_T2CK_INPUT = 16;
 * SHADOW_CONNECT_RPI16_TO = (RPINR3bits.T2CKR-0x1000); (IMPOSSIBLE)
 *************************************************************************************/

/*CONNECT_RP64_TO, T2CK_INPUT*/     /*64, RPINR3bits.T2CKR*/    /*RPINR3bits.T2CKR = 64*/
/*
 * SHADOW_T2CK_INPUT = 64;
 * SHADOW_CONNECT_RP64_TO = (RPINR3bits.T2CKR-0x1000);  (IMPOSSIBLE)
 *************************************************************************************/

/*CONNECT_RP64_TO, U1TX_OUTPUT*/    /*64, 0x1001*/              /*_RP64R = 0x0001*/
/*
 * SHADOW_U1TX_OUTPUT = 64;  (IMPOSSIBLE)
 * SHADOW_CONNECT_RP64_TO = (0x1001-0x1000);
 *************************************************************************************/

/*CONNECT_RPI16_TO, U1TX_OUTPUT*/   /*#error "Cannot Assign RPIn Pin As Output Pin"*/
/*************************************************************************************/

/** Unlocks I/O pins for Peripheral Pin Mapping. */
#define  PPS_Unlock()                   __builtin_write_OSCCONL(OSCCON & 0xbf)
/** Locks I/O pins for Peripheral Pin Mapping. */
#define  PPS_Lock()                     __builtin_write_OSCCONL(OSCCON | 0x40)

// <editor-fold defaultstate="collapsed" desc="PPS_Mapping Parameters">

#if defined (GROUP1_dsPIC33EPXXX_GP_MC_MU_806_810_814_And_PIC24EPXXX_GP_GU_810_814)

/*----------------------------------------------------------------------------------------------------*/
/*  Input Pin Selection For Selectable Input Sources: 																	  */
/*----------------------------------------------------------------------------------------------------*/
#define CONNECT_INTERNAL_VSS_TO			0
#define CONNECT_INTERNAL_C1OUT_TO		1
#define CONNECT_INTERNAL_C2OUT_TO		2
#define CONNECT_INTERNAL_C3OUT_TO		3
//#define CONNECT_INTERNAL_C4OUT_TO		4
//#define CONNECT_INTERNAL_PTGO30_TO		6
//#define CONNECT_INTERNAL_PTGO31_TO		7
#define CONNECT_INTERNAL_FINDX1_TO		8
#define CONNECT_INTERNAL_FHOME1_TO		9
#define CONNECT_INTERNAL_FINDX2_TO		10
#define CONNECT_INTERNAL_FHOME2_TO		11
//#define CONNECT_INTERNAL_C5OUT_TO             12
#define CONNECT_RPI16_TO			16
#define CONNECT_RPI17_TO			17
#define CONNECT_RPI18_TO			18
#define CONNECT_RPI19_TO			19
#define CONNECT_RPI20_TO			20
#define CONNECT_RPI21_TO			21
#define CONNECT_RPI22_TO			22
#define CONNECT_RPI23_TO			23
//#define CONNECT_RPI24_TO			24
//#define CONNECT_RPI25_TO			25
//#define CONNECT_RPI27_TO			27
//#define CONNECT_RPI28_TO			28
//#define CONNECT_RPI29_TO			29
#define CONNECT_RPI30_TO			30
#define CONNECT_RPI31_TO			31
#define CONNECT_RPI32_TO			32
#define CONNECT_RPI33_TO			33
#define CONNECT_RPI34_TO			34
#define CONNECT_RPI35_TO			35
#define CONNECT_RPI36_TO			36
#define CONNECT_RPI37_TO			37
#define CONNECT_RPI38_TO			38
#define CONNECT_RPI39_TO			39
#define CONNECT_RPI40_TO			40
#define CONNECT_RPI41_TO			41
#define CONNECT_RPI42_TO			42
#define CONNECT_RPI43_TO			43
#define CONNECT_RPI44_TO			44
#define CONNECT_RPI45_TO			45
#define CONNECT_RPI46_TO			46
#define CONNECT_RPI47_TO			47
//#define CONNECT_RPI48_TO			48
#define CONNECT_RPI49_TO			49
#define CONNECT_RPI50_TO			50
#define CONNECT_RPI51_TO			51
#define CONNECT_RPI52_TO			52
//#define CONNECT_RPI53_TO			53
//#define CONNECT_RPI54_TO			54
//#define CONNECT_RPI55_TO			55
//#define CONNECT_RPI56_TO			56
//#define CONNECT_RPI57_TO			57
//#define CONNECT_RPI58_TO			58
//#define CONNECT_RPI59_TO			59
#define CONNECT_RPI60_TO			60
#define CONNECT_RPI61_TO			61
#define CONNECT_RPI62_TO			62
//#define CONNECT_RPI63_TO			63
//#define CONNECT_RPI64_TO			64
//#define CONNECT_RPI65_TO			65
//#define CONNECT_RPI66_TO			66
//#define CONNECT_RPI67_TO			67
//#define CONNECT_RPI68_TO			68
//#define CONNECT_RPI69_TO			69
//#define CONNECT_RPI70_TO			70
//#define CONNECT_RPI71_TO			71
#define CONNECT_RPI72_TO			72
#define CONNECT_RPI73_TO			73
#define CONNECT_RPI74_TO			74
#define CONNECT_RPI75_TO			75
#define CONNECT_RPI76_TO			76
#define CONNECT_RPI77_TO			77
#define CONNECT_RPI78_TO			78
//#define CONNECT_RPI79_TO			79
//#define CONNECT_RPI80_TO			80
#define CONNECT_RPI81_TO			81
//#define CONNECT_RPI82_TO			82
#define CONNECT_RPI83_TO			83
//#define CONNECT_RPI84_TO			84
//#define CONNECT_RPI85_TO			85
#define CONNECT_RPI86_TO			86
//#define CONNECT_RPI87_TO			87
#define CONNECT_RPI88_TO			88
#define CONNECT_RPI89_TO			89
//#define CONNECT_RPI90_TO			90
//#define CONNECT_RPI91_TO			91
//#define CONNECT_RPI92_TO			92
//#define CONNECT_RPI93_TO			93
//#define CONNECT_RPI94_TO			94
//#define CONNECT_RPI95_TO			95
//#define CONNECT_RPI96_TO			96
//#define CONNECT_RPI97_TO			97
//#define CONNECT_RPI98_TO			98
//#define CONNECT_RPI99_TO			99
//#define CONNECT_RPI100_TO			100
//#define CONNECT_RPI101_TO			101
//#define CONNECT_RPI102_TO			102
//#define CONNECT_RPI103_TO			103
//#define CONNECT_RPI104_TO			104
//#define CONNECT_RPI105_TO			105
//#define CONNECT_RPI106_TO			106
//#define CONNECT_RPI107_TO			107
//#define CONNECT_RPI108_TO			108
//#define CONNECT_RPI109_TO			109
//#define CONNECT_RPI110_TO			110
//#define CONNECT_RPI111_TO			111
//#define CONNECT_RPI112_TO			112
//#define CONNECT_RPI113_TO			113
//#define CONNECT_RPI114_TO			114
//#define CONNECT_RPI115_TO			115
//#define CONNECT_RPI116_TO			116
//#define CONNECT_RPI117_TO			117
//#define CONNECT_RPI118_TO			118
#define CONNECT_RPI119_TO			119
//#define CONNECT_RPI120_TO			120
#define CONNECT_RPI121_TO			121
//#define CONNECT_RPI122_TO			122
//#define CONNECT_RPI123_TO			123
#define CONNECT_RPI124_TO			124
//#define CONNECT_RPI125_TO			125
//#define CONNECT_RPI126_TO			126
//#define CONNECT_RPI127_TO			127
//#define CONNECT_RPI176_TO			176
//#define CONNECT_RPI177_TO			177
//#define CONNECT_RPI178_TO			178
//#define CONNECT_RPI179_TO			179
//#define CONNECT_RPI180_TO			180
//#define CONNECT_RPI181_TO			181

/*----------------------------------------------------------------------------------------------------*/
/*  Output Selection For Remappable Pins (RPn ONLY!): 																	  */
/*----------------------------------------------------------------------------------------------------*/
#define NULL_OUTPUT			0x1000				/* RPn tied to default port pin */
#define U1TX_OUTPUT			0x1001				/* RPn tied to UART1 Transmit */
#define U1RTS_OUTPUT                    0x1002				/* RPn tied to UART1 Ready To Send */
#define U2TX_OUTPUT			0x1003				/* RPn tied to UART2 Transmit */
#define U2RTS_OUTPUT                    0x1004				/* RPn tied to UART2 Ready To Send */
#define SDO1_OUTPUT			0x1005				/* RPn tied to SPI1 Data Output */
#define SCK1_OUTPUT			0x1006				/* RPn tied to SPI1 Clock Output */
#define SS1_OUTPUT			0x1007				/* RPn tied to SPI1 Slave Select Output */
//#define SDO2_OUTPUT			0x1008				/* RPn tied to SPI2 Data Output */
//#define SCK2_OUTPUT			0x1009				/* RPn tied to SPI2 Clock Output */
#define SS2_OUTPUT			0x100A				/* RPn tied to SPI2 Slave Select Output */
#define CSDO_OUTPUT			0x100B				/* RPn tied to DCI Serial Data Output*/
#define CSCKOUT_OUTPUT                  0x100C				/* RPn tied to DCI Serial Clock Output*/
#define COFSOUT_OUTPUT                  0x100D				/* RPn tied to DCI Frame Sync Output*/
#define C1TX_OUTPUT			0x100E				/* RPn tied to ECAN1 Transmit */
#define C2TX_OUTPUT			0x100F				/* RPn tied to ECAN2 Transmit */
#define OC1_OUTPUT			0x1010				/* RPn tied to Output Compare 1 */
#define OC2_OUTPUT			0x1011				/* RPn tied to Output Compare 2 */
#define OC3_OUTPUT			0x1012				/* RPn tied to Output Compare 3 */
#define OC4_OUTPUT			0x1013				/* RPn tied to Output Compare 4 */
#define OC5_OUTPUT			0x1014				/* RPn tied to Output Compare 5 */
#define OC6_OUTPUT			0x1015				/* RPn tied to Output Compare 6 */
#define OC7_OUTPUT			0x1016				/* RPn tied to Output Compare 7 */
#define OC8_OUTPUT			0x1017				/* RPn tied to Output Compare 8 */
#define C1OUT_OUTPUT			0x1018				/* RPn tied to Comparator1 Output */
#define C2OUT_OUTPUT			0x1019				/* RPn tied to Comparator2 Output */
#define C3OUT_OUTPUT			0x101A				/* RPn tied to Comparator3 Output */
#define U3TX_OUTPUT			0x101B				/* RPn tied to UART3 Transmit */
#define U3RTS_OUTPUT			0x101C				/* RPn tied to UART3 Ready To Send */
#define U4TX_OUTPUT			0x101D				/* RPn tied to UART4 Transmit */
#define U4RTS_OUTPUT			0x101E				/* RPn tied to UART4 Ready To Send */
#define SDO3_OUTPUT			0x101F				/* RPn tied to SPI3 Data Output */
#define SCK3_OUTPUT			0x1020				/* RPn tied to SPI3 Clock Output */
#define SS3_OUTPUT			0x1021				/* RPn tied to SPI3 Slave Select Output */
#define SDO4_OUTPUT			0x1022				/* RPn tied to SPI4 Data Output */
#define SCK4_OUTPUT			0x1023				/* RPn tied to SPI4 Clock Output */
#define SS4_OUTPUT			0x1024				/* RPn tied to SPI4 Slave Select Output */
#define OC9_OUTPUT			0x1025				/* RPn tied to Output Compare 9 */
#define OC10_OUTPUT			0x1026				/* RPn tied to Output Compare 10 */
#define OC11_OUTPUT			0x1027				/* RPn tied to Output Compare 11 */
#define OC12_OUTPUT			0x1028				/* RPn tied to Output Compare 12 */
#define OC13_OUTPUT			0x1029				/* RPn tied to Output Compare 13 */
#define OC14_OUTPUT			0x102A				/* RPn tied to Output Compare 14 */
#define OC15_OUTPUT			0x102B				/* RPn tied to Output Compare 15 */
#define OC16_OUTPUT			0x102C				/* RPn tied to Output Compare 16 */
#define REFCLKO_OUTPUT                  0x1031				/* RPn tied to Output REFCLKO */
//#define C4OUT_OUTPUT			0x1032				/* RPn tied to Comparator4 Output */
//#define C5OUT_OUTPUT			0x1033				/* RPn tied to Comparator5 Output */
//#define SENT1_OUTPUT			0x1039				/* RPn tied to SENT1 Output */
//#define SENT2_OUTPUT			0x103A				/* RPn tied to SENT2 Output */

#if defined  (__dsPIC33EP256MU806__) || defined  (__dsPIC33EP256MU810__) || defined  (__dsPIC33EP256MU814__) ||	\
    defined  (__dsPIC33EP512MU814__) || defined  (__dsPIC33EP512MC806__) || defined  (__dsPIC33EP512MU810__)
#define SYNCO1_OUTPUT			0x102D				/* RPn tied to Output SYNCO1 */
#define SYNCO2_OUTPUT			0x102E				/* RPn tied to Output SYNCO2 */
#define QEI1CCMP_OUTPUT			0x102F				/* RPn tied to Output QEI1CCMP */
#define QEI2CCMP_OUTPUT			0x1030				/* RPn tied to Output QEI2CCMP */
#endif

#endif

#if defined (GROUP2_dsPIC33EPXXX_GM_3XX_6XX_7XX)

/*----------------------------------------------------------------------------------------------------*/
/*  Input Pin Selection For Selectable Input Sources: 																	  */
/*----------------------------------------------------------------------------------------------------*/
#define CONNECT_INTERNAL_VSS_TO			0
#define CONNECT_INTERNAL_C1OUT_TO		1
#define CONNECT_INTERNAL_C2OUT_TO		2
#define CONNECT_INTERNAL_C3OUT_TO		3
#define CONNECT_INTERNAL_C4OUT_TO		4
#define CONNECT_INTERNAL_PTGO30_TO		6
#define CONNECT_INTERNAL_PTGO31_TO		7
#define CONNECT_INTERNAL_INDX1_TO		8
#define CONNECT_INTERNAL_HOME1_TO		9
#define CONNECT_INTERNAL_INDX2_TO		10
#define CONNECT_INTERNAL_HOME2_TO		11
#define CONNECT_INTERNAL_C5OUT_TO		12
#define CONNECT_RPI16_TO			16
#define CONNECT_RPI17_TO			17
#define CONNECT_RPI18_TO			18
#define CONNECT_RPI19_TO			19
//#define CONNECT_RPI20_TO			20
//#define CONNECT_RPI21_TO			21
//#define CONNECT_RPI22_TO			22
//#define CONNECT_RPI23_TO			23
#define CONNECT_RPI24_TO			24
#define CONNECT_RPI25_TO			25
#define CONNECT_RPI27_TO			27
#define CONNECT_RPI28_TO			28
//#define CONNECT_RPI29_TO			29
//#define CONNECT_RPI30_TO			30
//#define CONNECT_RPI31_TO			31
#define CONNECT_RPI32_TO			32
#define CONNECT_RPI33_TO			33
#define CONNECT_RPI34_TO			34
//#define CONNECT_RPI35_TO			35
//#define CONNECT_RPI36_TO			36
//#define CONNECT_RPI37_TO			37
//#define CONNECT_RPI38_TO			38
//#define CONNECT_RPI39_TO			39
//#define CONNECT_RPI40_TO			40
//#define CONNECT_RPI41_TO			41
//#define CONNECT_RPI42_TO			42
//#define CONNECT_RPI43_TO			43
#define CONNECT_RPI44_TO			44
#define CONNECT_RPI45_TO			45
#define CONNECT_RPI46_TO			46
#define CONNECT_RPI47_TO			47
//#define CONNECT_RPI48_TO			48
//#define CONNECT_RPI49_TO			49
#define CONNECT_RPI50_TO			50
#define CONNECT_RPI51_TO			51
#define CONNECT_RPI52_TO			52
#define CONNECT_RPI53_TO			53
//#define CONNECT_RPI54_TO			54
//#define CONNECT_RPI55_TO			55
//#define CONNECT_RPI56_TO			56
//#define CONNECT_RPI57_TO			57
#define CONNECT_RPI58_TO			58
//#define CONNECT_RPI59_TO			59
#define CONNECT_RPI60_TO			60
#define CONNECT_RPI61_TO			61
//#define CONNECT_RPI62_TO			62
#define CONNECT_RPI63_TO			63
//#define CONNECT_RPI64_TO			64
//#define CONNECT_RPI65_TO			65
//#define CONNECT_RPI66_TO			66
//#define CONNECT_RPI67_TO			67
//#define CONNECT_RPI68_TO			68
//#define CONNECT_RPI69_TO			69
//#define CONNECT_RPI70_TO			70
//#define CONNECT_RPI71_TO			71
#define CONNECT_RPI72_TO			72
//#define CONNECT_RPI73_TO			73
//#define CONNECT_RPI74_TO			74
//#define CONNECT_RPI75_TO			75
#define CONNECT_RPI76_TO			76
#define CONNECT_RPI77_TO			77
//#define CONNECT_RPI78_TO			78
//#define CONNECT_RPI79_TO			79
#define CONNECT_RPI80_TO			80
//#define CONNECT_RPI81_TO			81
//#define CONNECT_RPI82_TO			82
//#define CONNECT_RPI83_TO			83
//#define CONNECT_RPI84_TO			84
//#define CONNECT_RPI85_TO			85
//#define CONNECT_RPI86_TO			86
//#define CONNECT_RPI87_TO			87
//#define CONNECT_RPI88_TO			88
//#define CONNECT_RPI89_TO			89
//#define CONNECT_RPI90_TO			90
//#define CONNECT_RPI91_TO			91
//#define CONNECT_RPI92_TO			92
//#define CONNECT_RPI93_TO			93
#define CONNECT_RPI94_TO			94
#define CONNECT_RPI95_TO			95
#define CONNECT_RPI96_TO			96
//#define CONNECT_RPI97_TO			97
//#define CONNECT_RPI98_TO			98
//#define CONNECT_RPI99_TO			99
//#define CONNECT_RPI100_TO			100
//#define CONNECT_RPI101_TO			101
//#define CONNECT_RPI102_TO			102
//#define CONNECT_RPI103_TO			103
//#define CONNECT_RPI104_TO			104
//#define CONNECT_RPI105_TO			105
//#define CONNECT_RPI106_TO			106
//#define CONNECT_RPI107_TO			107
//#define CONNECT_RPI108_TO			108
//#define CONNECT_RPI109_TO			109
//#define CONNECT_RPI110_TO			110
//#define CONNECT_RPI111_TO			111
#define CONNECT_RPI112_TO			112
//#define CONNECT_RPI113_TO			113
//#define CONNECT_RPI114_TO			114
//#define CONNECT_RPI115_TO			115
//#define CONNECT_RPI116_TO			116
//#define CONNECT_RPI117_TO			117
//#define CONNECT_RPI118_TO			118
#define CONNECT_RPI119_TO			119
//#define CONNECT_RPI120_TO			120
#define CONNECT_RPI121_TO			121
//#define CONNECT_RPI122_TO			122
//#define CONNECT_RPI123_TO			123
#define CONNECT_RPI124_TO			124
//#define CONNECT_RPI125_TO			125
//#define CONNECT_RPI126_TO			126
//#define CONNECT_RPI127_TO			127
//#define CONNECT_RPI176_TO			176
//#define CONNECT_RPI177_TO			177
//#define CONNECT_RPI178_TO			178
//#define CONNECT_RPI179_TO			179
//#define CONNECT_RPI180_TO			180
//#define CONNECT_RPI181_TO			181

/*----------------------------------------------------------------------------------------------------*/
/*  Output Selection For Remappable Pins (RPn ONLY!): 																	  */
/*----------------------------------------------------------------------------------------------------*/
#define NULL_OUTPUT			0x1000				/* RPn tied to default port pin */
#define U1TX_OUTPUT			0x1001				/* RPn tied to UART1 Transmit */
//#define U1RTS_OUTPUT			0x1002				/* RPn tied to UART1 Ready To Send */
#define U2TX_OUTPUT			0x1003				/* RPn tied to UART2 Transmit */
//#define U2RTS_OUTPUT			0x1004				/* RPn tied to UART2 Ready To Send */
//#define SDO1_OUTPUT			0x1005				/* RPn tied to SPI1 Data Output */
//#define SCK1_OUTPUT			0x1006				/* RPn tied to SPI1 Clock Output */
//#define SS1_OUTPUT			0x1007				/* RPn tied to SPI1 Slave Select Output */
#define SDO2_OUTPUT			0x1008				/* RPn tied to SPI2 Data Output */
#define SCK2_OUTPUT			0x1009				/* RPn tied to SPI2 Clock Output */
#define SS2_OUTPUT			0x100A				/* RPn tied to SPI2 Slave Select Output */
#define CSDO_OUTPUT			0x100B				/* RPn tied to DCI Serial Data Output*/
#define CSCK_OUTPUT			0x100C				/* RPn tied to DCI Serial Clock Output*/
#define COFS_OUTPUT			0x100D				/* RPn tied to DCI Frame Sync Output*/
#define C1TX_OUTPUT			0x100E				/* RPn tied to ECAN1 Transmit */
#define C2TX_OUTPUT			0x100F				/* RPn tied to ECAN2 Transmit */
#define OC1_OUTPUT			0x1010				/* RPn tied to Output Compare 1 */
#define OC2_OUTPUT			0x1011				/* RPn tied to Output Compare 2 */
#define OC3_OUTPUT			0x1012				/* RPn tied to Output Compare 3 */
#define OC4_OUTPUT			0x1013				/* RPn tied to Output Compare 4 */
#define OC5_OUTPUT			0x1014				/* RPn tied to Output Compare 5 */
#define OC6_OUTPUT			0x1015				/* RPn tied to Output Compare 6 */
#define OC7_OUTPUT			0x1016				/* RPn tied to Output Compare 7 */
#define OC8_OUTPUT			0x1017				/* RPn tied to Output Compare 8 */
#define C1OUT_OUTPUT			0x1018				/* RPn tied to Comparator1 Output */
#define C2OUT_OUTPUT			0x1019				/* RPn tied to Comparator2 Output */
#define C3OUT_OUTPUT			0x101A				/* RPn tied to Comparator3 Output */
#define U3TX_OUTPUT			0x101B				/* RPn tied to UART3 Transmit */
#define U3RTS_OUTPUT			0x101C				/* RPn tied to UART3 Ready To Send */
#define U4TX_OUTPUT			0x101D				/* RPn tied to UART4 Transmit */
#define U4RTS_OUTPUT			0x101E				/* RPn tied to UART4 Ready To Send */
#define SDO3_OUTPUT			0x101F				/* RPn tied to SPI3 Data Output */
#define SCK3_OUTPUT			0x1020				/* RPn tied to SPI3 Clock Output */
#define SS3_OUTPUT			0x1021				/* RPn tied to SPI3 Slave Select Output */
//#define SDO4_OUTPUT			0x1022				/* RPn tied to SPI4 Data Output */
//#define SCK4_OUTPUT			0x1023				/* RPn tied to SPI4 Clock Output */
//#define SS4_OUTPUT			0x1024				/* RPn tied to SPI4 Slave Select Output */
//#define OC9_OUTPUT			0x1025				/* RPn tied to Output Compare 9 */
//#define OC10_OUTPUT			0x1026				/* RPn tied to Output Compare 10 */
//#define OC11_OUTPUT			0x1027				/* RPn tied to Output Compare 11 */
//#define OC12_OUTPUT			0x1028				/* RPn tied to Output Compare 12 */
//#define OC13_OUTPUT			0x1029				/* RPn tied to Output Compare 13 */
//#define OC14_OUTPUT			0x102A				/* RPn tied to Output Compare 14 */
//#define OC15_OUTPUT			0x102B				/* RPn tied to Output Compare 15 */
//#define OC16_OUTPUT			0x102C				/* RPn tied to Output Compare 16 */
#define SYNCO1_OUTPUT			0x102D				/* RPn tied to Output SYNCO1 */
#define SYNCO2_OUTPUT			0x102E				/* RPn tied to Output SYNCO2 */
#define QEI1CCMP_OUTPUT			0x102F				/* RPn tied to Output QEI1CCMP */
#define QEI2CCMP_OUTPUT			0x1030				/* RPn tied to Output QEI2CCMP */
#define REFCLKO_OUTPUT                  0x1031				/* RPn tied to Output REFCLKO */
#define C4OUT_OUTPUT			0x1032				/* RPn tied to Comparator4 Output */
#define C5OUT_OUTPUT			0x1033				/* RPn tied to Comparator5 Output */
//#define SENT1_OUTPUT			0x1039				/* RPn tied to SENT1 Output */
//#define SENT2_OUTPUT			0x103A				/* RPn tied to SENT2 Output */

#endif

#if defined (GROUP3_dsPIC33EPXXX_GP_50X_dsPIC33EPXXX_MC_20X_50X_PIC24EPXXX_GP_MC_20X)

/*----------------------------------------------------------------------------------------------------*/
/*  Input Pin Selection For Selectable Input Sources: 																	  */
/*----------------------------------------------------------------------------------------------------*/
#define CONNECT_INTERNAL_VSS_TO			0
#define CONNECT_INTERNAL_C1OUT_TO		1
#define CONNECT_INTERNAL_C2OUT_TO		2
#define CONNECT_INTERNAL_C3OUT_TO		3
#define CONNECT_INTERNAL_C4OUT_TO		4
#define CONNECT_INTERNAL_PTGO30_TO		6
#define CONNECT_INTERNAL_PTGO31_TO		7
#define CONNECT_INTERNAL_FINDX1_TO		8
#define CONNECT_INTERNAL_FHOME1_TO		9
//#define CONNECT_INTERNAL_FINDX2_TO		10
//#define CONNECT_INTERNAL_FHOME2_TO		11
//#define CONNECT_INTERNAL_C5OUT_TO             12
//#define CONNECT_RPI16_TO			16
//#define CONNECT_RPI17_TO			17
//#define CONNECT_RPI18_TO			18
//#define CONNECT_RPI19_TO			19
//#define CONNECT_RPI20_TO			20
//#define CONNECT_RPI21_TO			21
//#define CONNECT_RPI22_TO			22
//#define CONNECT_RPI23_TO			23
#define CONNECT_RPI24_TO			24
#define CONNECT_RPI25_TO			25
#define CONNECT_RPI27_TO			27
#define CONNECT_RPI28_TO			28
//#define CONNECT_RPI29_TO			29
//#define CONNECT_RPI30_TO			30
//#define CONNECT_RPI31_TO			31
#define CONNECT_RPI32_TO			32
#define CONNECT_RPI33_TO			33
#define CONNECT_RPI34_TO			34
//#define CONNECT_RPI35_TO			35
//#define CONNECT_RPI36_TO			36
//#define CONNECT_RPI37_TO			37
//#define CONNECT_RPI38_TO			38
//#define CONNECT_RPI39_TO			39
//#define CONNECT_RPI40_TO			40
//#define CONNECT_RPI41_TO			41
//#define CONNECT_RPI42_TO			42
//#define CONNECT_RPI43_TO			43
#define CONNECT_RPI44_TO			44
#define CONNECT_RPI45_TO			45
#define CONNECT_RPI46_TO			46
#define CONNECT_RPI47_TO			47
//#define CONNECT_RPI48_TO			48
//#define CONNECT_RPI49_TO			49
//#define CONNECT_RPI50_TO			50
#define CONNECT_RPI51_TO			51
#define CONNECT_RPI52_TO			52
#define CONNECT_RPI53_TO			53
//#define CONNECT_RPI54_TO			54
//#define CONNECT_RPI55_TO			55
//#define CONNECT_RPI56_TO			56
//#define CONNECT_RPI57_TO			57
#define CONNECT_RPI58_TO			58
//#define CONNECT_RPI59_TO			59
//#define CONNECT_RPI60_TO			60
//#define CONNECT_RPI61_TO			61
//#define CONNECT_RPI62_TO			62
//#define CONNECT_RPI63_TO			63
//#define CONNECT_RPI64_TO			64
//#define CONNECT_RPI65_TO			65
//#define CONNECT_RPI66_TO			66
//#define CONNECT_RPI67_TO			67
//#define CONNECT_RPI68_TO			68
//#define CONNECT_RPI69_TO			69
//#define CONNECT_RPI70_TO			70
//#define CONNECT_RPI71_TO			71
//#define CONNECT_RPI72_TO			72
//#define CONNECT_RPI73_TO			73
//#define CONNECT_RPI74_TO			74
//#define CONNECT_RPI75_TO			75
//#define CONNECT_RPI76_TO			76
//#define CONNECT_RPI77_TO			77
//#define CONNECT_RPI78_TO			78
//#define CONNECT_RPI79_TO			79
//#define CONNECT_RPI80_TO			80
//#define CONNECT_RPI81_TO			81
//#define CONNECT_RPI82_TO			82
//#define CONNECT_RPI83_TO			83
//#define CONNECT_RPI84_TO			84
//#define CONNECT_RPI85_TO			85
//#define CONNECT_RPI86_TO			86
//#define CONNECT_RPI87_TO			87
//#define CONNECT_RPI88_TO			88
//#define CONNECT_RPI89_TO			89
//#define CONNECT_RPI90_TO			90
//#define CONNECT_RPI91_TO			91
//#define CONNECT_RPI92_TO			92
//#define CONNECT_RPI93_TO			93
#define CONNECT_RPI94_TO			94
#define CONNECT_RPI95_TO			95
#define CONNECT_RPI96_TO			96
//#define CONNECT_RPI97_TO			97
//#define CONNECT_RPI98_TO			98
//#define CONNECT_RPI99_TO			99
//#define CONNECT_RPI100_TO			100
//#define CONNECT_RPI101_TO			101
//#define CONNECT_RPI102_TO			102
//#define CONNECT_RPI103_TO			103
//#define CONNECT_RPI104_TO			104
//#define CONNECT_RPI105_TO			105
//#define CONNECT_RPI106_TO			106
//#define CONNECT_RPI107_TO			107
//#define CONNECT_RPI108_TO			108
//#define CONNECT_RPI109_TO			109
//#define CONNECT_RPI110_TO			110
//#define CONNECT_RPI111_TO			111
//#define CONNECT_RPI112_TO			112
//#define CONNECT_RPI113_TO			113
//#define CONNECT_RPI114_TO			114
//#define CONNECT_RPI115_TO			115
//#define CONNECT_RPI116_TO			116
//#define CONNECT_RPI117_TO			117
//#define CONNECT_RPI118_TO			118
#define CONNECT_RPI119_TO			119
//#define CONNECT_RPI120_TO			120
#define CONNECT_RPI121_TO			121
//#define CONNECT_RPI122_TO			122
//#define CONNECT_RPI123_TO			123
//#define CONNECT_RPI124_TO			124
//#define CONNECT_RPI125_TO			125
//#define CONNECT_RPI126_TO			126
//#define CONNECT_RPI127_TO			127
//#define CONNECT_RPI176_TO			176
//#define CONNECT_RPI177_TO			177
//#define CONNECT_RPI178_TO			178
//#define CONNECT_RPI179_TO			179
//#define CONNECT_RPI180_TO			180
//#define CONNECT_RPI181_TO			181

/*----------------------------------------------------------------------------------------------------*/
/*  Output Selection For Remappable Pins (RPn ONLY!): 																	  */
/*----------------------------------------------------------------------------------------------------*/
#define NULL_OUTPUT			0x1000				/* RPn tied to default port pin */
#define U1TX_OUTPUT			0x1001				/* RPn tied to UART1 Transmit */
//#define U1RTS_OUTPUT                  0x1002				/* RPn tied to UART1 Ready To Send */
#define U2TX_OUTPUT			0x1003				/* RPn tied to UART2 Transmit */
//#define U2RTS_OUTPUT                  0x1004				/* RPn tied to UART2 Ready To Send */
//#define SDO1_OUTPUT			0x1005				/* RPn tied to SPI1 Data Output */
//#define SCK1_OUTPUT			0x1006				/* RPn tied to SPI1 Clock Output */
//#define SS1_OUTPUT			0x1007				/* RPn tied to SPI1 Slave Select Output */
#define SDO2_OUTPUT			0x1008				/* RPn tied to SPI2 Data Output */
#define SCK2_OUTPUT			0x1009				/* RPn tied to SPI2 Clock Output */
#define SS2_OUTPUT			0x100A				/* RPn tied to SPI2 Slave Select Output */
//#define CSDO_OUTPUT			0x100B				/* RPn tied to DCI Serial Data Output*/
//#define CSCKOUT_OUTPUT                0x100C				/* RPn tied to DCI Serial Clock Output*/
//#define COFSOUT_OUTPUT                0x100D				/* RPn tied to DCI Frame Sync Output*/
#define C1TX_OUTPUT			0x100E				/* RPn tied to ECAN1 Transmit */
//#define C2TX_OUTPUT			0x100F				/* RPn tied to ECAN2 Transmit */
#define OC1_OUTPUT			0x1010				/* RPn tied to Output Compare 1 */
#define OC2_OUTPUT			0x1011				/* RPn tied to Output Compare 2 */
#define OC3_OUTPUT			0x1012				/* RPn tied to Output Compare 3 */
#define OC4_OUTPUT			0x1013				/* RPn tied to Output Compare 4 */
//#define OC5_OUTPUT			0x1014				/* RPn tied to Output Compare 5 */
//#define OC6_OUTPUT			0x1015				/* RPn tied to Output Compare 6 */
//#define OC7_OUTPUT			0x1016				/* RPn tied to Output Compare 7 */
//#define OC8_OUTPUT			0x1017				/* RPn tied to Output Compare 8 */
#define C1OUT_OUTPUT			0x1018				/* RPn tied to Comparator1 Output */
#define C2OUT_OUTPUT			0x1019				/* RPn tied to Comparator2 Output */
#define C3OUT_OUTPUT			0x101A				/* RPn tied to Comparator3 Output */
//#define U3TX_OUTPUT			0x101B				/* RPn tied to UART3 Transmit */
//#define U3RTS_OUTPUT			0x101C				/* RPn tied to UART3 Ready To Send */
//#define U4TX_OUTPUT			0x101D				/* RPn tied to UART4 Transmit */
//#define U4RTS_OUTPUT			0x101E				/* RPn tied to UART4 Ready To Send */
//#define SDO3_OUTPUT			0x101F				/* RPn tied to SPI3 Data Output */
//#define SCK3_OUTPUT			0x1020				/* RPn tied to SPI3 Clock Output */
//#define SS3_OUTPUT			0x1021				/* RPn tied to SPI3 Slave Select Output */
//#define SDO4_OUTPUT			0x1022				/* RPn tied to SPI4 Data Output */
//#define SCK4_OUTPUT			0x1023				/* RPn tied to SPI4 Clock Output */
//#define SS4_OUTPUT			0x1024				/* RPn tied to SPI4 Slave Select Output */
//#define OC9_OUTPUT			0x1025				/* RPn tied to Output Compare 9 */
//#define OC10_OUTPUT			0x1026				/* RPn tied to Output Compare 10 */
//#define OC11_OUTPUT			0x1027				/* RPn tied to Output Compare 11 */
//#define OC12_OUTPUT			0x1028				/* RPn tied to Output Compare 12 */
//#define OC13_OUTPUT			0x1029				/* RPn tied to Output Compare 13 */
//#define OC14_OUTPUT			0x102A				/* RPn tied to Output Compare 14 */
//#define OC15_OUTPUT			0x102B				/* RPn tied to Output Compare 15 */
//#define OC16_OUTPUT			0x102C				/* RPn tied to Output Compare 16 */
#define SYNCO1_OUTPUT			0x102D				/* RPn tied to Output SYNCO1 */
//#define SYNCO2_OUTPUT			0x102E				/* RPn tied to Output SYNCO2 */
#define QEI1CCMP_OUTPUT			0x102F				/* RPn tied to Output QEI1CCMP */
//#define QEI2CCMP_OUTPUT		0x1030				/* RPn tied to Output QEI2CCMP */
#define REFCLKO_OUTPUT                  0x1031				/* RPn tied to Output REFCLKO */
#define C4OUT_OUTPUT			0x1032				/* RPn tied to Comparator4 Output */
//#define C5OUT_OUTPUT			0x1033				/* RPn tied to Comparator5 Output */
//#define SENT1_OUTPUT			0x1039				/* RPn tied to SENT1 Output */
//#define SENT2_OUTPUT			0x103A				/* RPn tied to SENT2 Output */

#endif


#define SHADOW_CONNECT_INTERNAL_VSS_TO		FakeReg
#define SHADOW_CONNECT_INTERNAL_C1OUT_TO	FakeReg
#define SHADOW_CONNECT_INTERNAL_C2OUT_TO	FakeReg
#define SHADOW_CONNECT_INTERNAL_C3OUT_TO	FakeReg
#define SHADOW_CONNECT_INTERNAL_C4OUT_TO	FakeReg
#define SHADOW_CONNECT_INTERNAL_PTGO30_TO	FakeReg
#define SHADOW_CONNECT_INTERNAL_PTGO31_TO	FakeReg
#define SHADOW_CONNECT_INTERNAL_FINDX1_TO	FakeReg
#define SHADOW_CONNECT_INTERNAL_FHOME1_TO	FakeReg
#define SHADOW_CONNECT_INTERNAL_FINDX2_TO	FakeReg
#define SHADOW_CONNECT_INTERNAL_FHOME2_TO	FakeReg
#define SHADOW_CONNECT_INTERNAL_C5OUT_TO	FakeReg
#define SHADOW_CONNECT_RPI16_TO			FakeReg
#define SHADOW_CONNECT_RPI17_TO			FakeReg
#define SHADOW_CONNECT_RPI18_TO			FakeReg
#define SHADOW_CONNECT_RPI19_TO			FakeReg
#define SHADOW_CONNECT_RPI20_TO			FakeReg
#define SHADOW_CONNECT_RPI21_TO			FakeReg
#define SHADOW_CONNECT_RPI22_TO			FakeReg
#define SHADOW_CONNECT_RPI23_TO			FakeReg
#define SHADOW_CONNECT_RPI24_TO			FakeReg
#define SHADOW_CONNECT_RPI25_TO			FakeReg
#define SHADOW_CONNECT_RPI27_TO			FakeReg
#define SHADOW_CONNECT_RPI28_TO			FakeReg
#define SHADOW_CONNECT_RPI29_TO			FakeReg
#define SHADOW_CONNECT_RPI30_TO			FakeReg
#define SHADOW_CONNECT_RPI31_TO			FakeReg
#define SHADOW_CONNECT_RPI32_TO			FakeReg
#define SHADOW_CONNECT_RPI33_TO			FakeReg
#define SHADOW_CONNECT_RPI34_TO			FakeReg
#define SHADOW_CONNECT_RPI35_TO			FakeReg
#define SHADOW_CONNECT_RPI36_TO			FakeReg
#define SHADOW_CONNECT_RPI37_TO			FakeReg
#define SHADOW_CONNECT_RPI38_TO			FakeReg
#define SHADOW_CONNECT_RPI39_TO			FakeReg
#define SHADOW_CONNECT_RPI40_TO			FakeReg
#define SHADOW_CONNECT_RPI41_TO			FakeReg
#define SHADOW_CONNECT_RPI42_TO			FakeReg
#define SHADOW_CONNECT_RPI43_TO			FakeReg
#define SHADOW_CONNECT_RPI44_TO			FakeReg
#define SHADOW_CONNECT_RPI45_TO			FakeReg
#define SHADOW_CONNECT_RPI46_TO			FakeReg
#define SHADOW_CONNECT_RPI47_TO			FakeReg
#define SHADOW_CONNECT_RPI48_TO			FakeReg
#define SHADOW_CONNECT_RPI49_TO			FakeReg
#define SHADOW_CONNECT_RPI50_TO			FakeReg
#define SHADOW_CONNECT_RPI51_TO			FakeReg
#define SHADOW_CONNECT_RPI52_TO			FakeReg
#define SHADOW_CONNECT_RPI53_TO			FakeReg
#define SHADOW_CONNECT_RPI54_TO			FakeReg
#define SHADOW_CONNECT_RPI55_TO			FakeReg
#define SHADOW_CONNECT_RPI56_TO			FakeReg
#define SHADOW_CONNECT_RPI57_TO			FakeReg
#define SHADOW_CONNECT_RPI58_TO			FakeReg
#define SHADOW_CONNECT_RPI59_TO			FakeReg
#define SHADOW_CONNECT_RPI60_TO			FakeReg
#define SHADOW_CONNECT_RPI61_TO			FakeReg
#define SHADOW_CONNECT_RPI62_TO			FakeReg
#define SHADOW_CONNECT_RPI63_TO			FakeReg
#define SHADOW_CONNECT_RPI64_TO			FakeReg
#define SHADOW_CONNECT_RPI65_TO			FakeReg
#define SHADOW_CONNECT_RPI66_TO			FakeReg
#define SHADOW_CONNECT_RPI67_TO			FakeReg
#define SHADOW_CONNECT_RPI68_TO			FakeReg
#define SHADOW_CONNECT_RPI69_TO			FakeReg
#define SHADOW_CONNECT_RPI70_TO			FakeReg
#define SHADOW_CONNECT_RPI71_TO			FakeReg
#define SHADOW_CONNECT_RPI72_TO			FakeReg
#define SHADOW_CONNECT_RPI73_TO			FakeReg
#define SHADOW_CONNECT_RPI74_TO			FakeReg
#define SHADOW_CONNECT_RPI75_TO			FakeReg
#define SHADOW_CONNECT_RPI76_TO			FakeReg
#define SHADOW_CONNECT_RPI77_TO			FakeReg
#define SHADOW_CONNECT_RPI78_TO			FakeReg
#define SHADOW_CONNECT_RPI79_TO			FakeReg
#define SHADOW_CONNECT_RPI80_TO			FakeReg
#define SHADOW_CONNECT_RPI81_TO			FakeReg
#define SHADOW_CONNECT_RPI82_TO			FakeReg
#define SHADOW_CONNECT_RPI83_TO			FakeReg
#define SHADOW_CONNECT_RPI84_TO			FakeReg
#define SHADOW_CONNECT_RPI85_TO			FakeReg
#define SHADOW_CONNECT_RPI86_TO			FakeReg
#define SHADOW_CONNECT_RPI87_TO			FakeReg
#define SHADOW_CONNECT_RPI88_TO			FakeReg
#define SHADOW_CONNECT_RPI89_TO			FakeReg
#define SHADOW_CONNECT_RPI90_TO			FakeReg
#define SHADOW_CONNECT_RPI91_TO			FakeReg
#define SHADOW_CONNECT_RPI92_TO			FakeReg
#define SHADOW_CONNECT_RPI93_TO			FakeReg
#define SHADOW_CONNECT_RPI94_TO			FakeReg
#define SHADOW_CONNECT_RPI95_TO			FakeReg
#define SHADOW_CONNECT_RPI96_TO			FakeReg
#define SHADOW_CONNECT_RPI97_TO			FakeReg
#define SHADOW_CONNECT_RPI98_TO			FakeReg
#define SHADOW_CONNECT_RPI99_TO			FakeReg
#define SHADOW_CONNECT_RPI100_TO		FakeReg
#define SHADOW_CONNECT_RPI101_TO		FakeReg
#define SHADOW_CONNECT_RPI102_TO		FakeReg
#define SHADOW_CONNECT_RPI103_TO		FakeReg
#define SHADOW_CONNECT_RPI104_TO		FakeReg
#define SHADOW_CONNECT_RPI105_TO		FakeReg
#define SHADOW_CONNECT_RPI106_TO		FakeReg
#define SHADOW_CONNECT_RPI107_TO		FakeReg
#define SHADOW_CONNECT_RPI108_TO		FakeReg
#define SHADOW_CONNECT_RPI109_TO		FakeReg
#define SHADOW_CONNECT_RPI110_TO		FakeReg
#define SHADOW_CONNECT_RPI111_TO		FakeReg
#define SHADOW_CONNECT_RPI112_TO		FakeReg
#define SHADOW_CONNECT_RPI113_TO		FakeReg
#define SHADOW_CONNECT_RPI114_TO		FakeReg
#define SHADOW_CONNECT_RPI115_TO		FakeReg
#define SHADOW_CONNECT_RPI116_TO		FakeReg
#define SHADOW_CONNECT_RPI117_TO		FakeReg
#define SHADOW_CONNECT_RPI118_TO		FakeReg
#define SHADOW_CONNECT_RPI119_TO		FakeReg
#define SHADOW_CONNECT_RPI120_TO		FakeReg
#define SHADOW_CONNECT_RPI121_TO		FakeReg
#define SHADOW_CONNECT_RPI122_TO		FakeReg
#define SHADOW_CONNECT_RPI123_TO		FakeReg
#define SHADOW_CONNECT_RPI124_TO		FakeReg
#define SHADOW_CONNECT_RPI125_TO		FakeReg
#define SHADOW_CONNECT_RPI126_TO		FakeReg
#define SHADOW_CONNECT_RPI127_TO		FakeReg
#define SHADOW_CONNECT_RPI176_TO		FakeReg
#define SHADOW_CONNECT_RPI177_TO		FakeReg
#define SHADOW_CONNECT_RPI178_TO		FakeReg
#define SHADOW_CONNECT_RPI179_TO		FakeReg
#define SHADOW_CONNECT_RPI180_TO		FakeReg
#define SHADOW_CONNECT_RPI181_TO		FakeReg


#define SHADOW_NULL_OUTPUT			FakeReg			/* RPn tied to default port pin */
#define SHADOW_U1TX_OUTPUT			FakeReg			/* RPn tied to UART1 Transmit */
#define SHADOW_U1RTS_OUTPUT			FakeReg			/* RPn tied to UART1 Ready To Send */
#define SHADOW_U2TX_OUTPUT			FakeReg			/* RPn tied to UART2 Transmit */
#define SHADOW_U2RTS_OUTPUT			FakeReg			/* RPn tied to UART2 Ready To Send */
#define SHADOW_SDO1_OUTPUT			FakeReg			/* RPn tied to SPI1 Data Output */
#define SHADOW_SCK1_OUTPUT			FakeReg			/* RPn tied to SPI1 Clock Output */
#define SHADOW_SS1_OUTPUT			FakeReg			/* RPn tied to SPI1 Slave Select Output */
#define SHADOW_SDO2_OUTPUT			FakeReg			/* RPn tied to SPI2 Data Output */
#define SHADOW_SCK2_OUTPUT			FakeReg			/* RPn tied to SPI2 Clock Output */
#define SHADOW_SS2_OUTPUT			FakeReg			/* RPn tied to SPI2 Slave Select Output */
#define SHADOW_CSDO_OUTPUT			FakeReg			/* RPn tied to DCI Serial Data Output*/
#define SHADOW_CSCKOUT_OUTPUT			FakeReg			/* RPn tied to DCI Serial Clock Output*/
#define SHADOW_COFSOUT_OUTPUT			FakeReg			/* RPn tied to DCI Frame Sync Output*/
#define SHADOW_C1TX_OUTPUT			FakeReg			/* RPn tied to ECAN1 Transmit */
#define SHADOW_C2TX_OUTPUT			FakeReg			/* RPn tied to ECAN2 Transmit */
#define SHADOW_OC1_OUTPUT			FakeReg			/* RPn tied to Output Compare 1 */
#define SHADOW_OC2_OUTPUT			FakeReg			/* RPn tied to Output Compare 2 */
#define SHADOW_OC3_OUTPUT			FakeReg			/* RPn tied to Output Compare 3 */
#define SHADOW_OC4_OUTPUT			FakeReg			/* RPn tied to Output Compare 4 */
#define SHADOW_OC5_OUTPUT			FakeReg			/* RPn tied to Output Compare 5 */
#define SHADOW_OC6_OUTPUT			FakeReg			/* RPn tied to Output Compare 6 */
#define SHADOW_OC7_OUTPUT			FakeReg			/* RPn tied to Output Compare 7 */
#define SHADOW_OC8_OUTPUT			FakeReg			/* RPn tied to Output Compare 8 */
#define SHADOW_C1OUT_OUTPUT			FakeReg			/* RPn tied to Comparator1 Output */
#define SHADOW_C2OUT_OUTPUT			FakeReg			/* RPn tied to Comparator2 Output */
#define SHADOW_C3OUT_OUTPUT			FakeReg			/* RPn tied to Comparator3 Output */
#define SHADOW_U3TX_OUTPUT			FakeReg			/* RPn tied to UART3 Transmit */
#define SHADOW_U3RTS_OUTPUT			FakeReg			/* RPn tied to UART3 Ready To Send */
#define SHADOW_U4TX_OUTPUT			FakeReg			/* RPn tied to UART4 Transmit */
#define SHADOW_U4RTS_OUTPUT			FakeReg			/* RPn tied to UART4 Ready To Send */
#define SHADOW_SDO3_OUTPUT			FakeReg			/* RPn tied to SPI3 Data Output */
#define SHADOW_SCK3_OUTPUT			FakeReg			/* RPn tied to SPI3 Clock Output */
#define SHADOW_SS3_OUTPUT			FakeReg			/* RPn tied to SPI3 Slave Select Output */
#define SHADOW_SDO4_OUTPUT			FakeReg			/* RPn tied to SPI4 Data Output */
#define SHADOW_SCK4_OUTPUT			FakeReg			/* RPn tied to SPI4 Clock Output */
#define SHADOW_SS4_OUTPUT			FakeReg			/* RPn tied to SPI4 Slave Select Output */
#define SHADOW_OC9_OUTPUT			FakeReg			/* RPn tied to Output Compare 9 */
#define SHADOW_OC10_OUTPUT			FakeReg			/* RPn tied to Output Compare 10 */
#define SHADOW_OC11_OUTPUT			FakeReg			/* RPn tied to Output Compare 11 */
#define SHADOW_OC12_OUTPUT			FakeReg			/* RPn tied to Output Compare 12 */
#define SHADOW_OC13_OUTPUT			FakeReg			/* RPn tied to Output Compare 13 */
#define SHADOW_OC14_OUTPUT			FakeReg			/* RPn tied to Output Compare 14 */
#define SHADOW_OC15_OUTPUT			FakeReg			/* RPn tied to Output Compare 15 */
#define SHADOW_OC16_OUTPUT			FakeReg			/* RPn tied to Output Compare 16 */
#define SHADOW_SYNCO1_OUTPUT			FakeReg			/* RPn tied to Output SYNCO1 */
#define SHADOW_SYNCO2_OUTPUT			FakeReg			/* RPn tied to Output SYNCO2 */
#define SHADOW_QEI1CCMP_OUTPUT			FakeReg			/* RPn tied to Output QEI1CCMP */
#define SHADOW_QEI2CCMP_OUTPUT			FakeReg			/* RPn tied to Output QEI2CCMP */
#define SHADOW_REFCLKO_OUTPUT			FakeReg			/* RPn tied to Output REFCLKO */
#define SHADOW_C4OUT_OUTPUT			FakeReg			/* RPn tied to Comparator4 Output */
#define SHADOW_C5OUT_OUTPUT			FakeReg			/* RPn tied to Comparator5 Output */
#define SHADOW_SENT1_OUTPUT			FakeReg			/* RPn tied to SENT1 Output */
#define SHADOW_SENT2_OUTPUT			FakeReg			/* RPn tied to SENT2 Output */

/*----------------------------------------------------------------------------------------------------*/
/*  Remappable Peripheral Inputs: 																	  */
/*----------------------------------------------------------------------------------------------------*/
#ifdef _INT1R
#define INT1_INPUT			_INT1R          /* Assign External Interrupt 1 (INTR1) to the corresponding RPn pin*/
#define SHADOW_INT1_INPUT		_INT1R          /* Assign External Interrupt 1 (INTR1) to the corresponding RPn pin*/
#endif
#ifdef _INT2R
#define INT2_INPUT			_INT2R          /* Assign External Interrupt 2 (INTR2) to the corresponding RPn pin*/
#define SHADOW_INT2_INPUT		_INT2R          /* Assign External Interrupt 2 (INTR2) to the corresponding RPn pin*/
#endif
#ifdef _INT3R
#define INT3_INPUT			_INT3R          /* Assign External Interrupt 3 (INTR3) to the corresponding RPn pin*/
#define SHADOW_INT3_INPUT		_INT3R          /* Assign External Interrupt 3 (INTR3) to the corresponding RPn pin*/
#endif
#ifdef _INT4R
#define INT4_INPUT			_INT4R          /* Assign External Interrupt 4 (INTR4) to the corresponding RPn pin*/
#define SHADOW_INT4_INPUT		_INT4R          /* Assign External Interrupt 4 (INTR4) to the corresponding RPn pin*/
#endif
#ifdef _T1CKR
#define T1CK_INPUT			_T1CKR          /* Assign Timer1 External Clock (T1CK) to the corresponding RPn pin*/
#define SHADOW_T1CK_INPUT		_T1CKR          /* Assign Timer1 External Clock (T1CK) to the corresponding RPn pin*/
#endif
#ifdef _T2CKR
#define T2CK_INPUT			_T2CKR          /* Assign Timer2 External Clock (T2CK) to the corresponding RPn pin*/
#define SHADOW_T2CK_INPUT		_T2CKR          /* Assign Timer2 External Clock (T2CK) to the corresponding RPn pin*/
#endif
#ifdef _T3CKR
#define T3CK_INPUT			_T3CKR          /* Assign Timer3 External Clock (T3CK) to the corresponding RPn pin*/
#define SHADOW_T3CK_INPUT		_T3CKR          /* Assign Timer3 External Clock (T3CK) to the corresponding RPn pin*/
#endif
#ifdef _T4CKR
#define T4CK_INPUT			_T4CKR          /* Assign Timer4 External Clock (T4CK) to the corresponding RPn pin*/
#define SHADOW_T4CK_INPUT		_T4CKR          /* Assign Timer4 External Clock (T4CK) to the corresponding RPn pin*/
#endif
#ifdef _T5CKR
#define T5CK_INPUT			_T5CKR          /* Assign Timer5 External Clock (T5CK) to the corresponding RPn pin*/
#define SHADOW_T5CK_INPUT		_T5CKR          /* Assign Timer5 External Clock (T5CK) to the corresponding RPn pin*/
#endif
#ifdef _T6CKR
#define T6CK_INPUT			_T6CKR          /* Assign Timer6 External Clock (T6CK) to the corresponding RPn pin*/
#define SHADOW_T6CK_INPUT		_T6CKR          /* Assign Timer6 External Clock (T6CK) to the corresponding RPn pin*/
#endif
#ifdef _T7CKR
#define T7CK_INPUT			_T7CKR          /* Assign Timer7 External Clock (T7CK) to the corresponding RPn pin*/
#define SHADOW_T7CK_INPUT		_T7CKR          /* Assign Timer7 External Clock (T7CK) to the corresponding RPn pin*/
#endif
#ifdef _T8CKR
#define T8CK_INPUT			_T8CKR          /* Assign Timer8 External Clock (T8CK) to the corresponding RPn pin*/
#define SHADOW_T8CK_INPUT		_T8CKR          /* Assign Timer8 External Clock (T8CK) to the corresponding RPn pin*/
#endif
#ifdef _T9CKR
#define T9CK_INPUT			_T9CKR          /* Assign Timer9 External Clock (T9CK) to the corresponding RPn pin*/
#define SHADOW_T9CK_INPUT		_T9CKR          /* Assign Timer9 External Clock (T9CK) to the corresponding RPn pin*/
#endif
#ifdef _IC1R
#define IC1_INPUT			_IC1R           /* Assign Input Capture 1 (IC1) to the corresponding RPn pin*/
#define SHADOW_IC1_INPUT		_IC1R           /* Assign Input Capture 1 (IC1) to the corresponding RPn pin*/
#endif
#ifdef _IC2R
#define IC2_INPUT			_IC2R           /* Assign Input Capture 2 (IC2) to the corresponding RPn pin*/
#define SHADOW_IC2_INPUT		_IC2R           /* Assign Input Capture 2 (IC2) to the corresponding RPn pin*/
#endif
#ifdef _IC3R
#define IC3_INPUT			_IC3R           /* Assign Input Capture 3 (IC3) to the corresponding RPn pin*/
#define SHADOW_IC3_INPUT		_IC3R           /* Assign Input Capture 3 (IC3) to the corresponding RPn pin*/
#endif
#ifdef _IC4R
#define IC4_INPUT			_IC4R           /* Assign Input Capture 4 (IC4) to the corresponding RPn pin*/
#define SHADOW_IC4_INPUT		_IC4R           /* Assign Input Capture 4 (IC4) to the corresponding RPn pin*/
#endif
#ifdef _IC5R
#define IC5_INPUT			_IC5R           /* Assign Input Capture 5 (IC5) to the corresponding RPn pin*/
#define SHADOW_IC5_INPUT		_IC5R           /* Assign Input Capture 5 (IC5) to the corresponding RPn pin*/
#endif
#ifdef _IC6R
#define IC6_INPUT			_IC6R           /* Assign Input Capture 6 (IC6) to the corresponding RPn pin*/
#define SHADOW_IC6_INPUT		_IC6R           /* Assign Input Capture 6 (IC6) to the corresponding RPn pin*/
#endif
#ifdef _IC7R
#define IC7_INPUT			_IC7R           /* Assign Input Capture 7 (IC7) to the corresponding RPn pin*/
#define SHADOW_IC7_INPUT		_IC7R           /* Assign Input Capture 7 (IC7) to the corresponding RPn pin*/
#endif
#ifdef _IC8R
#define IC8_INPUT			_IC8R           /* Assign Input Capture 8 (IC8) to the corresponding RPn pin*/
#define SHADOW_IC8_INPUT		_IC8R           /* Assign Input Capture 8 (IC8) to the corresponding RPn pin*/
#endif
#ifdef _OCFAR
#define OCFA_INPUT			_OCFAR          /* Assign Output Capture A (OCFA) to the corresponding RPn pin*/
#define SHADOW_OCFA_INPUT		_OCFAR          /* Assign Output Capture A (OCFA) to the corresponding RPn pin*/
#endif
#ifdef _OCFBR
#define OCFB_INPUT			_OCFBR          /* Assign Output Capture B (OCFB) to the corresponding RPn pin*/
#define SHADOW_OCFB_INPUT		_OCFBR          /* Assign Output Capture B (OCFB) to the corresponding RPn pin*/
#endif
#ifdef _FLT1R
#define FLT1_INPUT			_FLT1R          /* Assign PWM1 Fault (FLT1) to the corresponding RPn pin*/
#define SHADOW_FLT1_INPUT		_FLT1R          /* Assign PWM1 Fault (FLT1) to the corresponding RPn pin*/
#endif
#ifdef _FLT2R
#define FLT2_INPUT			_FLT2R          /* Assign PWM2 Fault (FLT2) to the corresponding RPn pin*/
#define SHADOW_FLT2_INPUT		_FLT2R          /* Assign PWM2 Fault (FLT2) to the corresponding RPn pin*/
#endif
#ifdef _FLT3R
#define FLT3_INPUT			_FLT3R          /* Assign PWM3 Fault (FLT3) to the corresponding RPn pin*/
#define SHADOW_FLT3_INPUT		_FLT3R          /* Assign PWM3 Fault (FLT3) to the corresponding RPn pin*/
#endif
#ifdef _FLT4R
#define FLT4_INPUT			_FLT4R          /* Assign PWM4 Fault (FLT4) to the corresponding RPn pin*/
#define SHADOW_FLT4_INPUT		_FLT4R          /* Assign PWM4 Fault (FLT4) to the corresponding RPn pin*/
#endif
#ifdef _QEA1R
#define QEA1_INPUT			_QEA1R          /* Assign A (QEA) to the corresponding pin*/
#define SHADOW_QEA1_INPUT		_QEA1R          /* Assign A (QEA) to the corresponding pin*/
#endif
#ifdef _QEB1R
#define QEB1_INPUT			_QEB1R          /* Assign B (QEB) to the corresponding pin*/
#define SHADOW_QEB1_INPUT		_QEB1R          /* Assign B (QEB) to the corresponding pin*/
#endif
#ifdef _INDX1R
#define QEI1_INPUT			_INDX1R         /* Assign QEI INDEX (INDX) to the corresponding RPn pin*/
#define SHADOW_QEI1_INPUT		_INDX1R         /* Assign QEI INDEX (INDX) to the corresponding RPn pin*/
#endif
#ifdef _HOME1R
#define QEH1_INPUT			_HOME1R         /* Assign QEI HOME to the corresponding RPn pin*/
#define SHADOW_QEH1_INPUT		_HOME1R         /* Assign QEI HOME to the corresponding RPn pin*/
#endif
#ifdef _QEA2R
#define QEA2_INPUT			_QEA2R          /* Assign A (QEA) to the corresponding pin*/
#define SHADOW_QEA2_INPUT		_QEA2R          /* Assign A (QEA) to the corresponding pin*/
#endif
#ifdef _QEB2R
#define QEB2_INPUT			_QEB2R          /* Assign B (QEB) to the corresponding pin*/
#define SHADOW_QEB2_INPUT		_QEB2R          /* Assign B (QEB) to the corresponding pin*/
#endif
#ifdef _INDX2R
#define QEI2_INPUT			_INDX2R         /* Assign QEI INDEX (INDX) to the corresponding RPn pin*/
#define SHADOW_QEI2_INPUT		_INDX2R         /* Assign QEI INDEX (INDX) to the corresponding RPn pin*/
#endif
#ifdef _HOME2R
#define QEH2_INPUT			_HOME2R         /* Assign QEI HOME to the corresponding RPn pin*/
#define SHADOW_QEH2_INPUT		_HOME2R         /* Assign QEI HOME to the corresponding RPn pin*/
#endif
#ifdef _U1RXR
#define U1RX_INPUT			_U1RXR          /* Assign UART1 Receive (U1RX) to the corresponding RPn pin*/
#define SHADOW_U1RX_INPUT		_U1RXR          /* Assign UART1 Receive (U1RX) to the corresponding RPn pin*/
#endif
#ifdef _U1CTSR
#define U1CTS_INPUT			_U1CTSR         /* Assign UART1 Clear to Send (U1CTS) to the corresponding RPn pin*/
#define SHADOW_U1CTS_INPUT		_U1CTSR         /* Assign UART1 Clear to Send (U1CTS) to the corresponding RPn pin*/
#endif
#ifdef _U2RXR
#define U2RX_INPUT			_U2RXR          /* Assign UART2 Receive (U2RX) to the corresponding RPn pin*/
#define SHADOW_U2RX_INPUT		_U2RXR          /* Assign UART2 Receive (U2RX) to the corresponding RPn pin*/
#endif
#ifdef _U2CTSR
#define U2CTS_INPUT			_U2CTSR         /* Assign UART2 Clear to Send (U2CTS) to the corresponding RPn pin*/
#define SHADOW_U2CTS_INPUT		_U2CTSR         /* Assign UART2 Clear to Send (U2CTS) to the corresponding RPn pin*/
#endif
#ifdef _SDI1R
#define SDI1_INPUT			_SDI1R          /* Assign SPI1 Data Input (SDI1) to the corresponding RPn pin*/
#define SHADOW_SDI1_INPUT		_SDI1R          /* Assign SPI1 Data Input (SDI1) to the corresponding RPn pin*/
#endif
#ifdef _SCK1R
#define SCK1_INPUT			_SCK1R          /* Assign SPI1 Clock Input (SCK1IN) to the corresponding RPn pin*/
#define SHADOW_SCK1_INPUT		_SCK1R          /* Assign SPI1 Clock Input (SCK1IN) to the corresponding RPn pin*/
#endif
#ifdef _SS1R
#define SS1_INPUT			_SS1R           /* Assign SPI1 Slave Select Input (SS1IN) to the corresponding RPn pin*/
#define SHADOW_SS1_INPUT		_SS1R           /* Assign SPI1 Slave Select Input (SS1IN) to the corresponding RPn pin*/
#endif
#ifdef _SDI2R
#define SDI2_INPUT			_SDI2R          /* Assign SPI2 Data Input (SDI2) to the corresponding RPn pin*/
#define SHADOW_SDI2_INPUT		_SDI2R          /* Assign SPI2 Data Input (SDI2) to the corresponding RPn pin*/
#endif
#ifdef _SCK2R
#define SCK2_INPUT      		_SCK2R          /* Assign SPI2 Clock Input (SCK2IN) to the corresponding RPn pin*/
#define SHADOW_SCK2_INPUT               _SCK2R          /* Assign SPI2 Clock Input (SCK2IN) to the corresponding RPn pin*/
#endif
#ifdef _SS2R
#define SS2_INPUT			_SS2R           /* Assign SPI2 Slave Select Input (SS2IN) to the corresponding RPn pin*/
#define SHADOW_SS2_INPUT		_SS2R           /* Assign SPI2 Slave Select Input (SS2IN) to the corresponding RPn pin*/
#endif
#ifdef _CSDIR
#define CSDI_INPUT			_CSDIR          /* Assign DCI Serial Data Input (CSDIN) to the corresponding RPn pin*/
#define SHADOW_CSDI_INPUT		_CSDIR          /* Assign DCI Serial Data Input (CSDIN) to the corresponding RPn pin*/
#endif
#ifdef _CSCKR
#define CSCK_INPUT			_CSCKR          /* Assign DCI Serial Clock Input (CSCKIN) to the corresponding RPn pin*/
#define SHADOW_CSCK_INPUT		_CSCKR          /* Assign DCI Serial Clock Input (CSCKIN) to the corresponding RPn pin*/
#endif
#ifdef _COFSR
#define COFS_INPUT			_COFSR          /* Assign DCI Frame Sync Input (COFSIN) to the corresponding RPn pin*/
#define SHADOW_COFS_INPUT		_COFSR          /* Assign DCI Frame Sync Input (COFSIN) to the corresponding RPn pin*/
#endif
#ifdef _C1RXR
#define C1RX_INPUT			_C1RXR          /* Assign ECAN1 Receive Input (C1RX) to the corresponding RPn pin*/
#define SHADOW_C1RX_INPUT		_C1RXR          /* Assign ECAN1 Receive Input (C1RX) to the corresponding RPn pin*/
#endif
#ifdef _C2RXR
#define C2RX_INPUT			_C2RXR          /* Assign ECAN2 Receive Input (C2RX) to the corresponding RPn pin*/
#define SHADOW_C2RX_INPUT		_C2RXR          /* Assign ECAN2 Receive Input (C2RX) to the corresponding RPn pin*/
#endif
#ifdef _U3RXR
#define U3RX_INPUT			_U3RXR          /* Assign UART3 Receive (U3RX) to the corresponding RPn pin*/
#define SHADOW_U3RX_INPUT		_U3RXR          /* Assign UART3 Receive (U3RX) to the corresponding RPn pin*/
#endif
#ifdef _U3CTSR
#define U3CTS_INPUT			_U3CTSR         /* Assign UART3 Clear to Send (U3CTS) to the corresponding RPn pin*/
#define SHADOW_U3CTS_INPUT		_U3CTSR         /* Assign UART3 Clear to Send (U3CTS) to the corresponding RPn pin*/
#endif
#ifdef _U4RXR
#define U4RX_INPUT			_U4RXR          /* Assign UART4 Receive (U4RX) to the corresponding RPn pin*/
#define SHADOW_U4RX_INPUT		_U4RXR          /* Assign UART4 Receive (U4RX) to the corresponding RPn pin*/
#endif
#ifdef _U4CTSR
#define U4CTS_INPUT			_U4CTSR         /* Assign UART4 Clear to Send (U4CTS) to the corresponding RPn pin*/
#define SHADOW_U4CTS_INPUT		_U4CTSR         /* Assign UART4 Clear to Send (U4CTS) to the corresponding RPn pin*/
#endif
#ifdef _SDI3R
#define SDI3_INPUT			_SDI3R          /* Assign SPI3 Data Input (SDI3) to the corresponding RPn pin*/
#define SHADOW_SDI3_INPUT		_SDI3R          /* Assign SPI3 Data Input (SDI3) to the corresponding RPn pin*/
#endif
#ifdef _SCK3R
#define SCK3_INPUT			_SCK3R          /* Assign SPI3 Clock Input (SCK3IN) to the corresponding RPn pin*/
#define SHADOW_SCK3_INPUT		_SCK3R          /* Assign SPI3 Clock Input (SCK3IN) to the corresponding RPn pin*/
#endif
#ifdef _SS3R
#define SS3_INPUT			_SS3R           /* Assign SPI3 Slave Select Input (SS3IN) to the corresponding RPn pin*/
#define SHADOW_SS3_INPUT		_SS3R           /* Assign SPI3 Slave Select Input (SS3IN) to the corresponding RPn pin*/
#endif
#ifdef _SDI4R
#define SDI4_INPUT			_SDI4R          /* Assign SPI4 Data Input (SDI4) to the corresponding RPn pin*/
#define SHADOW_SDI4_INPUT		_SDI4R          /* Assign SPI4 Data Input (SDI4) to the corresponding RPn pin*/
#endif
#ifdef _SCK4R
#define SCK4_INPUT			_SCK4R          /* Assign SPI4 Clock Input (SCK4IN) to the corresponding RPn pin*/
#define SHADOW_SCK4_INPUT		_SCK4R          /* Assign SPI4 Clock Input (SCK4IN) to the corresponding RPn pin*/
#endif
#ifdef _SS4R
#define SS4_INPUT			_SS4R           /* Assign SPI4 Slave Select Input (SS4IN) to the corresponding RPn pin*/
#define SHADOW_SS4_INPUT		_SS4R           /* Assign SPI4 Slave Select Input (SS4IN) to the corresponding RPn pin*/
#endif
#ifdef _IC9R
#define IC9_INPUT			_IC9R           /* Assign Input Capture 9 (IC9) to the corresponding RPn pin*/
#define SHADOW_IC9_INPUT		_IC9R           /* Assign Input Capture 9 (IC9) to the corresponding RPn pin*/
#endif
#ifdef _IC10R
#define IC10_INPUT			_IC10R          /* Assign Input Capture 10 (IC10) to the corresponding RPn pin*/
#define SHADOW_IC10_INPUT		_IC10R          /* Assign Input Capture 10 (IC10) to the corresponding RPn pin*/
#endif
#ifdef _IC11R
#define IC11_INPUT			_IC11R          /* Assign Input Capture 11 (IC11) to the corresponding RPn pin*/
#define SHADOW_IC11_INPUT		_IC11R          /* Assign Input Capture 11 (IC11) to the corresponding RPn pin*/
#endif
#ifdef _IC12R
#define IC12_INPUT			_IC12R          /* Assign Input Capture 12 (IC12) to the corresponding RPn pin*/
#define SHADOW_IC12_INPUT		_IC12R          /* Assign Input Capture 12 (IC12) to the corresponding RPn pin*/
#endif
#ifdef _IC13R
#define IC13_INPUT			_IC13R          /* Assign Input Capture 13 (IC13) to the corresponding RPn pin*/
#define SHADOW_IC13_INPUT		_IC13R          /* Assign Input Capture 13 (IC13) to the corresponding RPn pin*/
#endif
#ifdef _IC14R
#define IC14_INPUT			_IC14R          /* Assign Input Capture 14 (IC14) to the corresponding RPn pin*/
#define SHADOW_IC14_INPUT		_IC14R          /* Assign Input Capture 14 (IC14) to the corresponding RPn pin*/
#endif
#ifdef _IC15R
#define IC15_INPUT			_IC15R          /* Assign Input Capture 15 (IC15) to the corresponding RPn pin*/
#define SHADOW_IC15_INPUT		_IC15R          /* Assign Input Capture 15 (IC15) to the corresponding RPn pin*/
#endif
#ifdef _IC16R
#define IC16_INPUT			_IC16R          /* Assign Input Capture 16 (IC16) to the corresponding RPn pin*/
#define SHADOW_IC16_INPUT		_IC16R          /* Assign Input Capture 16 (IC16) to the corresponding RPn pin*/
#endif
#ifdef _OCFCR
#define OCFC_INPUT			_OCFCR          /* Assign Output Compare Fault C (OCFC) to the corresponding RPn pin*/
#define SHADOW_OCFC_INPUT		_OCFCR          /* Assign Output Compare Fault C (OCFC) to the corresponding RPn pin*/
#endif
#ifdef _SYNCI1R
#define SYNCI1_INPUT			_SYNCI1R        /* Assign PWM Sync Input 1 (SYNCI1) to the corresponding RPn pin*/
#define SHADOW_SYNCI1_INPUT		_SYNCI1R        /* Assign PWM Sync Input 1 (SYNCI1) to the corresponding RPn pin*/
#endif
#ifdef _SYNCI2R
#define SYNCI2_INPUT			_SYNCI2R        /* Assign PWM Sync Input 2 (SYNCI2) to the corresponding RPn pin*/
#define SHADOW_SYNCI2_INPUT		_SYNCI2R        /* Assign PWM Sync Input 2 (SYNCI2) to the corresponding RPn pin*/
#endif
#ifdef _DTCMP1R
#define DTCMP1_INPUT			_DTCMP1R	/* Assign PWM Dead Time Compensation 1 (DTCMP1) to the corresponding RPn pin*/
#define SHADOW_DTCMP1_INPUT		_DTCMP1R	/* Assign PWM Dead Time Compensation 1 (DTCMP1) to the corresponding RPn pin*/
#endif
#ifdef _DTCMP2R
#define DTCMP2_INPUT			_DTCMP2R	/* Assign PWM Dead Time Compensation 2 (DTCMP2) to the corresponding RPn pin*/
#define SHADOW_DTCMP2_INPUT		_DTCMP2R	/* Assign PWM Dead Time Compensation 2 (DTCMP2) to the corresponding RPn pin*/
#endif
#ifdef _DTCMP3R
#define DTCMP3_INPUT			_DTCMP3R	/* Assign PWM Dead Time Compensation 3 (DTCMP3) to the corresponding RPn pin*/
#define SHADOW_DTCMP3_INPUT		_DTCMP3R	/* Assign PWM Dead Time Compensation 3 (DTCMP3) to the corresponding RPn pin*/
#endif
#ifdef _DTCMP4R
#define DTCMP4_INPUT			_DTCMP4R	/* Assign PWM Dead Time Compensation 4 (DTCMP4) to the corresponding RPn pin*/
#define SHADOW_DTCMP4_INPUT		_DTCMP4R	/* Assign PWM Dead Time Compensation 4 (DTCMP4) to the corresponding RPn pin*/
#endif
#ifdef _DTCMP5R
#define DTCMP5_INPUT			_DTCMP5R	/* Assign PWM Dead Time Compensation 5 (DTCMP5) to the corresponding RPn pin*/
#define SHADOW_DTCMP5_INPUT		_DTCMP5R	/* Assign PWM Dead Time Compensation 5 (DTCMP5) to the corresponding RPn pin*/
#endif
#ifdef _DTCMP6R
#define DTCMP6_INPUT			_DTCMP6R	/* Assign PWM Dead Time Compensation 6 (DTCMP6) to the corresponding RPn pin*/
#define SHADOW_DTCMP6_INPUT		_DTCMP6R	/* Assign PWM Dead Time Compensation 6 (DTCMP6) to the corresponding RPn pin*/
#endif
#ifdef _DTCMP7R
#define DTCMP7_INPUT			_DTCMP7R	/* Assign PWM Dead Time Compensation 7 (DTCMP7) to the corresponding RPn pin*/
#define SHADOW_DTCMP7_INPUT		_DTCMP7R	/* Assign PWM Dead Time Compensation 7 (DTCMP7) to the corresponding RPn pin*/
#endif
#ifdef _FLT5R
#define FLT5_INPUT			_FLT5R          /* Assign PWM5 Fault (FLT5) to the corresponding RPn pin*/
#define SHADOW_FLT5_INPUT		_FLT5R          /* Assign PWM5 Fault (FLT5) to the corresponding RPn pin*/
#endif
#ifdef _FLT6R
#define FLT6_INPUT			_FLT6R          /* Assign PWM6 Fault (FLT6) to the corresponding RPn pin*/
#define SHADOW_FLT6_INPUT		_FLT6R          /* Assign PWM6 Fault (FLT6) to the corresponding RPn pin*/
#endif
#ifdef _FLT7R
#define FLT7_INPUT                      _FLT7R          /* Assign PWM7 Fault (FLT7) to the corresponding RPn pin*/
#define SHADOW_FLT7_INPUT               _FLT7R          /* Assign PWM7 Fault (FLT7) to the corresponding RPn pin*/
#endif
#ifdef _SENT1R
#define SENT1_INPUT			_SENT1R         /* Assign SENT module input 1 (SENT1) to the corresponding RPn pin*/
#define SHADOW_SENT1_INPUT		_SENT1R         /* Assign SENT module input 1 (SENT1) to the corresponding RPn pin*/
#endif
#ifdef _SENT2R
#define SENT2_INPUT			_SENT2R         /* Assign SENT module input 2 (SENT2) to the corresponding RPn pin*/
#define SHADOW_SENT2_INPUT		_SENT2R         /* Assign SENT module input 2 (SENT2) to the corresponding RPn pin*/
#endif

/**********************************************************************************************************************/
/**********************************************************************************************************************/

#ifdef  _RP20R
#define CONNECT_RP20_TO			20		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP20_TO		_RP20R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP35R
#define CONNECT_RP35_TO			35		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP35_TO		_RP35R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP36R
#define CONNECT_RP36_TO			36		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP36_TO		_RP36R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP37R
#define CONNECT_RP37_TO			37		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP37_TO		_RP37R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP38R
#define CONNECT_RP38_TO			38		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP38_TO		_RP38R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP39R
#define CONNECT_RP39_TO			39		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP39_TO		_RP39R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP40R
#define CONNECT_RP40_TO			40		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP40_TO		_RP40R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP41R
#define CONNECT_RP41_TO			41		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP41_TO		_RP41R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP42R
#define CONNECT_RP42_TO			42		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP42_TO		_RP42R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP43R
#define CONNECT_RP43_TO			43		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP43_TO		_RP43R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP44R
#define CONNECT_RP44_TO			44		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP44_TO		_RP44R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP45R
#define CONNECT_RP45_TO			45		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP45_TO		_RP45R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP46R
#define CONNECT_RP46_TO			46		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP46_TO		_RP46R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP47R
#define CONNECT_RP47_TO			47		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP47_TO		_RP47R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP48R
#define CONNECT_RP48_TO			48		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP48_TO		_RP48R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP49R
#define CONNECT_RP49_TO			49		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP49_TO		_RP49R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP50R
#define CONNECT_RP50_TO			50		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP50_TO		_RP50R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP51R
#define CONNECT_RP51_TO			51		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP51_TO		_RP51R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP52R
#define CONNECT_RP52_TO			52		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP52_TO		_RP52R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP53R
#define CONNECT_RP53_TO			53		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP53_TO		_RP53R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP54R
#define CONNECT_RP54_TO			54		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP54_TO		_RP54R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP55R
#define CONNECT_RP55_TO			55		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP55_TO		_RP55R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP56R
#define CONNECT_RP56_TO			56		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP56_TO		_RP56R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP57R
#define CONNECT_RP57_TO			57		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP57_TO		_RP57R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP58R
#define CONNECT_RP58_TO			58		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP58_TO		_RP58R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP59R
#define CONNECT_RP59_TO			59		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP59_TO		_RP59R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP60R
#define CONNECT_RP60_TO			60		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP60_TO		_RP60R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP61R
#define CONNECT_RP61_TO			61		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP61_TO		_RP61R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP62R
#define CONNECT_RP62_TO			62		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP62_TO		_RP62R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP63R
#define CONNECT_RP63_TO			63		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP63_TO		_RP63R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP64R
#define CONNECT_RP64_TO			64		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP64_TO		_RP64R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP65R
#define CONNECT_RP65_TO			65		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP65_TO		_RP65R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP66R
#define CONNECT_RP66_TO			66		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP66_TO		_RP66R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP67R
#define CONNECT_RP67_TO			67		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP67_TO		_RP67R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP68R
#define CONNECT_RP68_TO			68		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP68_TO		_RP68R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP69R
#define CONNECT_RP69_TO			69		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP69_TO		_RP69R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP70R
#define CONNECT_RP70_TO			70		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP70_TO		_RP70R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP71R
#define CONNECT_RP71_TO			71		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP71_TO		_RP71R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP72R
#define CONNECT_RP72_TO			72		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP72_TO		_RP72R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP73R
#define CONNECT_RP73_TO			73		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP73_TO		_RP73R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP74R
#define CONNECT_RP74_TO			74		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP74_TO		_RP74R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP75R
#define CONNECT_RP75_TO			75		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP75_TO		_RP75R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP76R
#define CONNECT_RP76_TO			76		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP76_TO		_RP76R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP77R
#define CONNECT_RP77_TO			77		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP77_TO		_RP77R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP78R
#define CONNECT_RP78_TO			78		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP78_TO		_RP78R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP79R
#define CONNECT_RP79_TO			79		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP79_TO		_RP79R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP80R
#define CONNECT_RP80_TO			80		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP80_TO		_RP80R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP81R
#define CONNECT_RP81_TO			81		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP81_TO		_RP81R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP82R
#define CONNECT_RP82_TO			82		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP82_TO		_RP82R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP83R
#define CONNECT_RP83_TO			83		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP83_TO		_RP83R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP84R
#define CONNECT_RP84_TO			84		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP84_TO		_RP84R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP85R
#define CONNECT_RP85_TO			85		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP85_TO		_RP85R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP86R
#define CONNECT_RP86_TO			86		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP86_TO		_RP86R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP87R
#define CONNECT_RP87_TO			87		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP87_TO		_RP87R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP88R
#define CONNECT_RP88_TO			88		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP88_TO		_RP88R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP89R
#define CONNECT_RP89_TO			89		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP89_TO		_RP89R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP90R
#define CONNECT_RP90_TO			90		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP90_TO		_RP90R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP91R
#define CONNECT_RP91_TO			91		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP91_TO		_RP91R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP92R
#define CONNECT_RP92_TO			92		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP92_TO		_RP92R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP93R
#define CONNECT_RP93_TO			93		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP93_TO		_RP93R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP94R
#define CONNECT_RP94_TO			94		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP94_TO		_RP94R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP95R
#define CONNECT_RP95_TO			95		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP95_TO		_RP95R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP96R
#define CONNECT_RP96_TO			96		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP96_TO		_RP96R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP97R
#define CONNECT_RP97_TO			97		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP97_TO		_RP97R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP98R
#define CONNECT_RP98_TO			98		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP98_TO		_RP98R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP99R
#define CONNECT_RP99_TO			99		/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP99_TO		_RP99R		/* Assign RPn as Output Pin */
#endif
#ifdef  _RP100R
#define CONNECT_RP100_TO		100	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP100_TO		_RP100R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP101R
#define CONNECT_RP101_TO		101	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP101_TO		_RP101R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP102R
#define CONNECT_RP102_TO		102	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP102_TO		_RP102R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP103R
#define CONNECT_RP103_TO		103	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP103_TO		_RP103R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP104R
#define CONNECT_RP104_TO		104	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP104_TO		_RP104R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP105R
#define CONNECT_RP105_TO		105	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP105_TO		_RP105R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP106R
#define CONNECT_RP106_TO		106	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP106_TO		_RP106R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP107R
#define CONNECT_RP107_TO		107	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP107_TO		_RP107R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP108R
#define CONNECT_RP108_TO		108	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP108_TO		_RP108R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP109R
#define CONNECT_RP109_TO		109	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP109_TO		_RP109R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP110R
#define CONNECT_RP110_TO		110	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP110_TO		_RP110R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP111R
#define CONNECT_RP111_TO		111	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP111_TO		_RP111R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP112R
#define CONNECT_RP112_TO		112	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP112_TO		_RP112R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP113R
#define CONNECT_RP113_TO		113	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP113_TO		_RP113R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP114R
#define CONNECT_RP114_TO		114	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP114_TO		_RP114R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP115R
#define CONNECT_RP115_TO		115	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP115_TO		_RP115R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP116R
#define CONNECT_RP116_TO		116	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP116_TO		_RP116R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP117R
#define CONNECT_RP117_TO		117	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP117_TO		_RP117R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP118R
#define CONNECT_RP118_TO		118	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP118_TO		_RP118R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP119R
#define CONNECT_RP119_TO		119	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP119_TO		_RP119R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP120R
#define CONNECT_RP120_TO		120	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP120_TO		_RP120R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP121R
#define CONNECT_RP121_TO		121	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP121_TO		_RP121R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP122R
#define CONNECT_RP122_TO		122	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP122_TO		_RP122R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP123R
#define CONNECT_RP123_TO		123	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP123_TO		_RP123R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP124R
#define CONNECT_RP124_TO		124	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP124_TO		_RP124R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP125R
#define CONNECT_RP125_TO		125	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP125_TO		_RP125R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP126R
#define CONNECT_RP126_TO		126	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP126_TO		_RP126R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP127R
#define CONNECT_RP127_TO		127	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP127_TO		_RP127R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP176R
#define CONNECT_RP176_TO		176	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP176_TO		_RP176R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP177R
#define CONNECT_RP177_TO		177	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP177_TO		_RP177R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP178R
#define CONNECT_RP178_TO		178	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP178_TO		_RP178R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP179R
#define CONNECT_RP179_TO		179	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP179_TO		_RP179R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP180R
#define CONNECT_RP180_TO		180	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP180_TO		_RP180R	/* Assign RPn as Output Pin */
#endif
#ifdef  _RP181R
#define CONNECT_RP181_TO		181	/* Assign RPn as Output Pin */
#define SHADOW_CONNECT_RP181_TO		_RP181R	/* Assign RPn as Output Pin */
#endif


// </editor-fold>

// </editor-fold>

#endif