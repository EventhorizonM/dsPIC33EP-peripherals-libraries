
/*
 * DMA Libary For All dsPIC33EP/PIC24EP Devices.
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
 * File: $Id: LibDMA.h, V1.00 2015/03/12 AL-Moutaz Billah Tabbakha Exp $
 *
 * Functions in This Library:   ("x" Denotes to DMA Channel Number)
 *
 * void SetupDMAxChannel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)
 * void SetupDMAx_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 * void SetupDMAxInterrupt(InterruptMode, uint8_t InterruptPriority)
 * void EnableChannelDMAx()
 * void DisableChannelDMAx()
 * BOOL WriteCollisionPeripheralDMAx()
 * BOOL RequestCollisionDMAx()
 * BOOL PingPongStatusDMAx()
 * uint32_t LastAddressAccessedByDMA()
 * uint8_t LastDMAActive()
 * void EnableIntDMAx()
 * void DisableIntDMAx()
 * void SetIntPriorityDMAx(priority)
 *
 */

#ifndef _LIBDMA_
#define _LIBDMA_

/*****************************************************************************/
/**************************** DMA Ch0 Functions ******************************/
/*****************************************************************************/
#if defined _DMA0IE

/***********************************************************************************************************************
 * \Function        void SetupDMA0Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA0 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 * 
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *				- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA0Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA0Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA0Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA0CONbits.SIZE  = DataTransferSize;\
		DMA0CONbits.DIR   = DataTransferDir;\
		DMA0CONbits.NULLW = DataTransferDir>>1;\
		DMA0PAD = PeripheralBuffer;\
		DMA0REQ = PeripheralIRQ;\
		DMA0CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA0_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA0 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example  \n   
 *			__eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
			__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA0_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA0_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA0_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA0CONbits.MODE  = OperatingMode;\
		DMA0CONbits.AMODE = AdressingMode;\
		DMA0STAL = Buf_A_StartAddress;\
		DMA0STAH = 0x0000;\
		DMA0STBL = Buf_B_StartAddress;\
		DMA0STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA0Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA0 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA0Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA0Interrupt(InterruptMode,InterruptPriority)({\
	DMA0CONbits.HALF = InterruptMode;\
	_DMA0IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA0()             DMA0CONbits.CHEN = 1
#define DisableChannelDMA0()            DMA0CONbits.CHEN = 0

/** Returns DMA0 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA0()     _PWCOL0
/** Returns DMA0 Channel Request Collision detect flag bit */
#define RequestCollisionDMA0()          _RQCOL0
/** Returns the register selected for ping-pong :\n 1 -> DMA0STB Reg \n 0 -> DMA0STA Reg*/
#define PingPongStatusDMA0()            _PPST0

#define EnableIntDMA0()                 _DMA0IE = 1
#define DisableIntDMA0()                _DMA0IE = 0
#define SetIntPriorityDMA0(priority)    _DMA0IP = (priority)
#define DMA0_INT_FLAG                   _DMA0IF

/* Setting the priority of DMA0 interrupt */
#define DMA0_INT_PRI_0                   0
#define DMA0_INT_PRI_1                   1
#define DMA0_INT_PRI_2                   2
#define DMA0_INT_PRI_3                   3
#define DMA0_INT_PRI_4                   4
#define DMA0_INT_PRI_5                   5
#define DMA0_INT_PRI_6                   6
#define DMA0_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch1 Functions ******************************/
/*****************************************************************************/
#if defined _DMA1IE

/***********************************************************************************************************************
 * \Function        void SetupDMA1Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA1 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA1Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA1Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA1Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA1CONbits.SIZE  = DataTransferSize;\
		DMA1CONbits.DIR   = DataTransferDir;\
		DMA1CONbits.NULLW = DataTransferDir>>1;\
		DMA1PAD = PeripheralBuffer;\
		DMA1REQ = PeripheralIRQ;\
		DMA1CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA1_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA1 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA1_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA1_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA1_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA1CONbits.MODE  = OperatingMode;\
		DMA1CONbits.AMODE = AdressingMode;\
		DMA1STAL = Buf_A_StartAddress;\
		DMA1STAH = 0x0000;\
		DMA1STBL = Buf_B_StartAddress;\
		DMA1STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA1Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA1 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA1Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA1Interrupt(InterruptMode,InterruptPriority)({\
	DMA1CONbits.HALF = InterruptMode;\
	_DMA1IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA1()             DMA1CONbits.CHEN = 1
#define DisableChannelDMA1()            DMA1CONbits.CHEN = 0

/** Returns DMA1 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA1()     _PWCOL1
/** Returns DMA1 Channel Request Collision detect flag bit */
#define RequestCollisionDMA1()          _RQCOL1
/** Returns the register selected for ping-pong :\n 1 -> DMA1STB Reg \n 0 -> DMA1STA Reg*/
#define PingPongStatusDMA1()            _PPST1

#define EnableIntDMA1()                 _DMA1IE = 1
#define DisableIntDMA1()                _DMA1IE = 0
#define SetIntPriorityDMA1(priority)    _DMA1IP = (priority)
#define DMA1_INT_FLAG                   _DMA1IF

/* Setting the priority of DMA1 interrupt */
#define DMA1_INT_PRI_0                   0
#define DMA1_INT_PRI_1                   1
#define DMA1_INT_PRI_2                   2
#define DMA1_INT_PRI_3                   3
#define DMA1_INT_PRI_4                   4
#define DMA1_INT_PRI_5                   5
#define DMA1_INT_PRI_6                   6
#define DMA1_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch2 Functions ******************************/
/*****************************************************************************/
#if defined _DMA2IE

/***********************************************************************************************************************
 * \Function        void SetupDMA2Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA2 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA2Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA2Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA2Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA2CONbits.SIZE  = DataTransferSize;\
		DMA2CONbits.DIR   = DataTransferDir;\
		DMA2CONbits.NULLW = DataTransferDir>>1;\
		DMA2PAD = PeripheralBuffer;\
		DMA2REQ = PeripheralIRQ;\
		DMA2CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA2_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA2 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA2_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA2_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA2_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA2CONbits.MODE  = OperatingMode;\
		DMA2CONbits.AMODE = AdressingMode;\
		DMA2STAL = Buf_A_StartAddress;\
		DMA2STAH = 0x0000;\
		DMA2STBL = Buf_B_StartAddress;\
		DMA2STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA2Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA2 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA2Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA2Interrupt(InterruptMode,InterruptPriority)({\
	DMA2CONbits.HALF = InterruptMode;\
	_DMA2IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA2()             DMA2CONbits.CHEN = 1
#define DisableChannelDMA2()            DMA2CONbits.CHEN = 0

/** Returns DMA2 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA2()     _PWCOL2
/** Returns DMA2 Channel Request Collision detect flag bit */
#define RequestCollisionDMA2()          _RQCOL2
/** Returns the register selected for ping-pong :\n 1 -> DMA2STB Reg \n 0 -> DMA2STA Reg*/
#define PingPongStatusDMA2()            _PPST2

#define EnableIntDMA2()                 _DMA2IE = 1
#define DisableIntDMA2()                _DMA2IE = 0
#define SetIntPriorityDMA2(priority)    _DMA2IP = (priority)
#define DMA2_INT_FLAG                   _DMA2IF

/* Setting the priority of DMA2 interrupt */
#define DMA2_INT_PRI_0                   0
#define DMA2_INT_PRI_1                   1
#define DMA2_INT_PRI_2                   2
#define DMA2_INT_PRI_3                   3
#define DMA2_INT_PRI_4                   4
#define DMA2_INT_PRI_5                   5
#define DMA2_INT_PRI_6                   6
#define DMA2_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch3 Functions ******************************/
/*****************************************************************************/
#if defined _DMA3IE

/***********************************************************************************************************************
 * \Function        void SetupDMA3Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA3 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA3Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA3Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA3Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA3CONbits.SIZE  = DataTransferSize;\
		DMA3CONbits.DIR   = DataTransferDir;\
		DMA3CONbits.NULLW = DataTransferDir>>1;\
		DMA3PAD = PeripheralBuffer;\
		DMA3REQ = PeripheralIRQ;\
		DMA3CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA3_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA3 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA3_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA3_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA3_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA3CONbits.MODE  = OperatingMode;\
		DMA3CONbits.AMODE = AdressingMode;\
		DMA3STAL = Buf_A_StartAddress;\
		DMA3STAH = 0x0000;\
		DMA3STBL = Buf_B_StartAddress;\
		DMA3STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA3Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA3 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA3Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA3Interrupt(InterruptMode,InterruptPriority)({\
	DMA3CONbits.HALF = InterruptMode;\
	_DMA3IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA3()             DMA3CONbits.CHEN = 1
#define DisableChannelDMA3()            DMA3CONbits.CHEN = 0

/** Returns DMA3 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA3()     _PWCOL3
/** Returns DMA3 Channel Request Collision detect flag bit */
#define RequestCollisionDMA3()          _RQCOL3
/** Returns the register selected for ping-pong :\n 1 -> DMA3STB Reg \n 0 -> DMA3STA Reg*/
#define PingPongStatusDMA3()            _PPST3

#define EnableIntDMA3()                 _DMA3IE = 1
#define DisableIntDMA3()                _DMA3IE = 0
#define SetIntPriorityDMA3(priority)    _DMA3IP = (priority)
#define DMA3_INT_FLAG                   _DMA3IF

/* Setting the priority of DMA3 interrupt */
#define DMA3_INT_PRI_0                   0
#define DMA3_INT_PRI_1                   1
#define DMA3_INT_PRI_2                   2
#define DMA3_INT_PRI_3                   3
#define DMA3_INT_PRI_4                   4
#define DMA3_INT_PRI_5                   5
#define DMA3_INT_PRI_6                   6
#define DMA3_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch4 Functions ******************************/
/*****************************************************************************/
#if defined _DMA4IE

/***********************************************************************************************************************
 * \Function        void SetupDMA4Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA4 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA4Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA4Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA4Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA4CONbits.SIZE  = DataTransferSize;\
		DMA4CONbits.DIR   = DataTransferDir;\
		DMA4CONbits.NULLW = DataTransferDir>>1;\
		DMA4PAD = PeripheralBuffer;\
		DMA4REQ = PeripheralIRQ;\
		DMA4CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA4_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA4 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA4_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA4_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA4_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA4CONbits.MODE  = OperatingMode;\
		DMA4CONbits.AMODE = AdressingMode;\
		DMA4STAL = Buf_A_StartAddress;\
		DMA4STAH = 0x0000;\
		DMA4STBL = Buf_B_StartAddress;\
		DMA4STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA4Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA4 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA4Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA4Interrupt(InterruptMode,InterruptPriority)({\
	DMA4CONbits.HALF = InterruptMode;\
	_DMA4IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA4()             DMA4CONbits.CHEN = 1
#define DisableChannelDMA4()            DMA4CONbits.CHEN = 0

/** Returns DMA4 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA4()     _PWCOL4
/** Returns DMA4 Channel Request Collision detect flag bit */
#define RequestCollisionDMA4()          _RQCOL4
/** Returns the register selected for ping-pong :\n 1 -> DMA4STB Reg \n 0 -> DMA4STA Reg*/
#define PingPongStatusDMA4()            _PPST4

#define EnableIntDMA4()                 _DMA4IE = 1
#define DisableIntDMA4()                _DMA4IE = 0
#define SetIntPriorityDMA4(priority)    _DMA4IP = (priority)
#define DMA4_INT_FLAG                   _DMA4IF

/* Setting the priority of DMA4 interrupt */
#define DMA4_INT_PRI_0                   0
#define DMA4_INT_PRI_1                   1
#define DMA4_INT_PRI_2                   2
#define DMA4_INT_PRI_3                   3
#define DMA4_INT_PRI_4                   4
#define DMA4_INT_PRI_5                   5
#define DMA4_INT_PRI_6                   6
#define DMA4_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch5 Functions ******************************/
/*****************************************************************************/
#if defined _DMA5IE

/***********************************************************************************************************************
 * \Function        void SetupDMA5Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA5 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA5Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA5Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA5Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA5CONbits.SIZE  = DataTransferSize;\
		DMA5CONbits.DIR   = DataTransferDir;\
		DMA5CONbits.NULLW = DataTransferDir>>1;\
		DMA5PAD = PeripheralBuffer;\
		DMA5REQ = PeripheralIRQ;\
		DMA5CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA5_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA5 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA5_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA5_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA5_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA5CONbits.MODE  = OperatingMode;\
		DMA5CONbits.AMODE = AdressingMode;\
		DMA5STAL = Buf_A_StartAddress;\
		DMA5STAH = 0x0000;\
		DMA5STBL = Buf_B_StartAddress;\
		DMA5STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA5Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA5 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA5Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA5Interrupt(InterruptMode,InterruptPriority)({\
	DMA5CONbits.HALF = InterruptMode;\
	_DMA5IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA5()             DMA5CONbits.CHEN = 1
#define DisableChannelDMA5()            DMA5CONbits.CHEN = 0

/** Returns DMA5 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA5()     _PWCOL5
/** Returns DMA5 Channel Request Collision detect flag bit */
#define RequestCollisionDMA5()          _RQCOL5
/** Returns the register selected for ping-pong :\n 1 -> DMA5STB Reg \n 0 -> DMA5STA Reg*/
#define PingPongStatusDMA5()            _PPST5

#define EnableIntDMA5()                 _DMA5IE = 1
#define DisableIntDMA5()                _DMA5IE = 0
#define SetIntPriorityDMA5(priority)    _DMA5IP = (priority)
#define DMA5_INT_FLAG                   _DMA5IF

/* Setting the priority of DMA5 interrupt */
#define DMA5_INT_PRI_0                   0
#define DMA5_INT_PRI_1                   1
#define DMA5_INT_PRI_2                   2
#define DMA5_INT_PRI_3                   3
#define DMA5_INT_PRI_4                   4
#define DMA5_INT_PRI_5                   5
#define DMA5_INT_PRI_6                   6
#define DMA5_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch6 Functions ******************************/
/*****************************************************************************/
#if defined _DMA6IE

/***********************************************************************************************************************
 * \Function        void SetupDMA6Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA6 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA6Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA6Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA6Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA6CONbits.SIZE  = DataTransferSize;\
		DMA6CONbits.DIR   = DataTransferDir;\
		DMA6CONbits.NULLW = DataTransferDir>>1;\
		DMA6PAD = PeripheralBuffer;\
		DMA6REQ = PeripheralIRQ;\
		DMA6CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA6_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA6 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA6_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA6_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA6_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA6CONbits.MODE  = OperatingMode;\
		DMA6CONbits.AMODE = AdressingMode;\
		DMA6STAL = Buf_A_StartAddress;\
		DMA6STAH = 0x0000;\
		DMA6STBL = Buf_B_StartAddress;\
		DMA6STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA6Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA6 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA6Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA6Interrupt(InterruptMode,InterruptPriority)({\
	DMA6CONbits.HALF = InterruptMode;\
	_DMA6IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA6()             DMA6CONbits.CHEN = 1
#define DisableChannelDMA6()            DMA6CONbits.CHEN = 0

/** Returns DMA6 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA6()     _PWCOL6
/** Returns DMA6 Channel Request Collision detect flag bit */
#define RequestCollisionDMA6()          _RQCOL6
/** Returns the register selected for ping-pong :\n 1 -> DMA6STB Reg \n 0 -> DMA6STA Reg*/
#define PingPongStatusDMA6()            _PPST6

#define EnableIntDMA6()                 _DMA6IE = 1
#define DisableIntDMA6()                _DMA6IE = 0
#define SetIntPriorityDMA6(priority)    _DMA6IP = (priority)
#define DMA6_INT_FLAG                   _DMA6IF

/* Setting the priority of DMA6 interrupt */
#define DMA6_INT_PRI_0                   0
#define DMA6_INT_PRI_1                   1
#define DMA6_INT_PRI_2                   2
#define DMA6_INT_PRI_3                   3
#define DMA6_INT_PRI_4                   4
#define DMA6_INT_PRI_5                   5
#define DMA6_INT_PRI_6                   6
#define DMA6_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch7 Functions ******************************/
/*****************************************************************************/
#if defined _DMA7IE

/***********************************************************************************************************************
 * \Function        void SetupDMA7Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA7 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA7Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA7Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA7Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA7CONbits.SIZE  = DataTransferSize;\
		DMA7CONbits.DIR   = DataTransferDir;\
		DMA7CONbits.NULLW = DataTransferDir>>1;\
		DMA7PAD = PeripheralBuffer;\
		DMA7REQ = PeripheralIRQ;\
		DMA7CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA7_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA7 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA7_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA7_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA7_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA7CONbits.MODE  = OperatingMode;\
		DMA7CONbits.AMODE = AdressingMode;\
		DMA7STAL = Buf_A_StartAddress;\
		DMA7STAH = 0x0000;\
		DMA7STBL = Buf_B_StartAddress;\
		DMA7STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA7Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA7 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA7Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA7Interrupt(InterruptMode,InterruptPriority)({\
	DMA7CONbits.HALF = InterruptMode;\
	_DMA7IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA7()             DMA7CONbits.CHEN = 1
#define DisableChannelDMA7()            DMA7CONbits.CHEN = 0

/** Returns DMA7 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA7()     _PWCOL7
/** Returns DMA7 Channel Request Collision detect flag bit */
#define RequestCollisionDMA7()          _RQCOL7
/** Returns the register selected for ping-pong :\n 1 -> DMA7STB Reg \n 0 -> DMA7STA Reg*/
#define PingPongStatusDMA7()            _PPST7

#define EnableIntDMA7()                 _DMA7IE = 1
#define DisableIntDMA7()                _DMA7IE = 0
#define SetIntPriorityDMA7(priority)    _DMA7IP = (priority)
#define DMA7_INT_FLAG                   _DMA7IF

/* Setting the priority of DMA7 interrupt */
#define DMA7_INT_PRI_0                   0
#define DMA7_INT_PRI_1                   1
#define DMA7_INT_PRI_2                   2
#define DMA7_INT_PRI_3                   3
#define DMA7_INT_PRI_4                   4
#define DMA7_INT_PRI_5                   5
#define DMA7_INT_PRI_6                   6
#define DMA7_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch8 Functions ******************************/
/*****************************************************************************/
#if defined _DMA8IE

/***********************************************************************************************************************
 * \Function        void SetupDMA8Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA8 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA8Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA8Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA8Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA8CONbits.SIZE  = DataTransferSize;\
		DMA8CONbits.DIR   = DataTransferDir;\
		DMA8CONbits.NULLW = DataTransferDir>>1;\
		DMA8PAD = PeripheralBuffer;\
		DMA8REQ = PeripheralIRQ;\
		DMA8CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA8_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA8 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA8_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA8_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA8_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA8CONbits.MODE  = OperatingMode;\
		DMA8CONbits.AMODE = AdressingMode;\
		DMA8STAL = Buf_A_StartAddress;\
		DMA8STAH = 0x0000;\
		DMA8STBL = Buf_B_StartAddress;\
		DMA8STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA8Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA8 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA8Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA8Interrupt(InterruptMode,InterruptPriority)({\
	DMA8CONbits.HALF = InterruptMode;\
	_DMA8IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA8()             DMA8CONbits.CHEN = 1
#define DisableChannelDMA8()            DMA8CONbits.CHEN = 0

/** Returns DMA8 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA8()     _PWCOL8
/** Returns DMA8 Channel Request Collision detect flag bit */
#define RequestCollisionDMA8()          _RQCOL8
/** Returns the register selected for ping-pong :\n 1 -> DMA8STB Reg \n 0 -> DMA8STA Reg*/
#define PingPongStatusDMA8()            _PPST8

#define EnableIntDMA8()                 _DMA8IE = 1
#define DisableIntDMA8()                _DMA8IE = 0
#define SetIntPriorityDMA8(priority)    _DMA8IP = (priority)
#define DMA8_INT_FLAG                   _DMA8IF

/* Setting the priority of DMA8 interrupt */
#define DMA8_INT_PRI_0                   0
#define DMA8_INT_PRI_1                   1
#define DMA8_INT_PRI_2                   2
#define DMA8_INT_PRI_3                   3
#define DMA8_INT_PRI_4                   4
#define DMA8_INT_PRI_5                   5
#define DMA8_INT_PRI_6                   6
#define DMA8_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch9 Functions ******************************/
/*****************************************************************************/
#if defined _DMA9IE

/***********************************************************************************************************************
 * \Function        void SetupDMA9Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA9 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA9Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA9Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA9Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA9CONbits.SIZE  = DataTransferSize;\
		DMA9CONbits.DIR   = DataTransferDir;\
		DMA9CONbits.NULLW = DataTransferDir>>1;\
		DMA9PAD = PeripheralBuffer;\
		DMA9REQ = PeripheralIRQ;\
		DMA9CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA9_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA9 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA9_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA9_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA9_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA9CONbits.MODE  = OperatingMode;\
		DMA9CONbits.AMODE = AdressingMode;\
		DMA9STAL = Buf_A_StartAddress;\
		DMA9STAH = 0x0000;\
		DMA9STBL = Buf_B_StartAddress;\
		DMA9STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA9Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA9 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA9Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA9Interrupt(InterruptMode,InterruptPriority)({\
	DMA9CONbits.HALF = InterruptMode;\
	_DMA9IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA9()             DMA9CONbits.CHEN = 1
#define DisableChannelDMA9()            DMA9CONbits.CHEN = 0

/** Returns DMA9 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA9()     _PWCOL9
/** Returns DMA9 Channel Request Collision detect flag bit */
#define RequestCollisionDMA9()          _RQCOL9
/** Returns the register selected for ping-pong :\n 1 -> DMA9STB Reg \n 0 -> DMA9STA Reg*/
#define PingPongStatusDMA9()            _PPST9

#define EnableIntDMA9()                 _DMA9IE = 1
#define DisableIntDMA9()                _DMA9IE = 0
#define SetIntPriorityDMA9(priority)    _DMA9IP = (priority)
#define DMA9_INT_FLAG                   _DMA9IF

/* Setting the priority of DMA9 interrupt */
#define DMA9_INT_PRI_0                   0
#define DMA9_INT_PRI_1                   1
#define DMA9_INT_PRI_2                   2
#define DMA9_INT_PRI_3                   3
#define DMA9_INT_PRI_4                   4
#define DMA9_INT_PRI_5                   5
#define DMA9_INT_PRI_6                   6
#define DMA9_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch10 Functions *****************************/
/*****************************************************************************/
#if defined _DMA10IE

/***********************************************************************************************************************
 * \Function        void SetupDMA10Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA10 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA10Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA10Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA10Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA10CONbits.SIZE  = DataTransferSize;\
		DMA10CONbits.DIR   = DataTransferDir;\
		DMA10CONbits.NULLW = DataTransferDir>>1;\
		DMA10PAD = PeripheralBuffer;\
		DMA10REQ = PeripheralIRQ;\
		DMA10CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA10_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA10 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA10_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA10_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA10_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA10CONbits.MODE  = OperatingMode;\
		DMA10CONbits.AMODE = AdressingMode;\
		DMA10STAL = Buf_A_StartAddress;\
		DMA10STAH = 0x0000;\
		DMA10STBL = Buf_B_StartAddress;\
		DMA10STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA10Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA10 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA10Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA10Interrupt(InterruptMode,InterruptPriority)({\
	DMA10CONbits.HALF = InterruptMode;\
	_DMA10IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA10()             DMA10CONbits.CHEN = 1
#define DisableChannelDMA10()            DMA10CONbits.CHEN = 0

/** Returns DMA10 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA10()     _PWCOL10
/** Returns DMA10 Channel Request Collision detect flag bit */
#define RequestCollisionDMA10()          _RQCOL10
/** Returns the register selected for ping-pong :\n 1 -> DMA10STB Reg \n 0 -> DMA10STA Reg*/
#define PingPongStatusDMA10()            _PPST10

#define EnableIntDMA10()                 _DMA10IE = 1
#define DisableIntDMA10()                _DMA10IE = 0
#define SetIntPriorityDMA10(priority)    _DMA10IP = (priority)
#define DMA10_INT_FLAG                   _DMA10IF

/* Setting the priority of DMA10 interrupt */
#define DMA10_INT_PRI_0                   0
#define DMA10_INT_PRI_1                   1
#define DMA10_INT_PRI_2                   2
#define DMA10_INT_PRI_3                   3
#define DMA10_INT_PRI_4                   4
#define DMA10_INT_PRI_5                   5
#define DMA10_INT_PRI_6                   6
#define DMA10_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch11 Functions *****************************/
/*****************************************************************************/
#if defined _DMA11IE

/***********************************************************************************************************************
 * \Function        void SetupDMA11Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA11 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA11Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA11Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA11Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA11CONbits.SIZE  = DataTransferSize;\
		DMA11CONbits.DIR   = DataTransferDir;\
		DMA11CONbits.NULLW = DataTransferDir>>1;\
		DMA11PAD = PeripheralBuffer;\
		DMA11REQ = PeripheralIRQ;\
		DMA11CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA11_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA11 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA11_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA11_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA11_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA11CONbits.MODE  = OperatingMode;\
		DMA11CONbits.AMODE = AdressingMode;\
		DMA11STAL = Buf_A_StartAddress;\
		DMA11STAH = 0x0000;\
		DMA11STBL = Buf_B_StartAddress;\
		DMA11STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA11Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA11 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA11Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA11Interrupt(InterruptMode,InterruptPriority)({\
	DMA11CONbits.HALF = InterruptMode;\
	_DMA11IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA11()             DMA11CONbits.CHEN = 1
#define DisableChannelDMA11()            DMA11CONbits.CHEN = 0

/** Returns DMA11 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA11()     _PWCOL11
/** Returns DMA11 Channel Request Collision detect flag bit */
#define RequestCollisionDMA11()          _RQCOL11
/** Returns the register selected for ping-pong :\n 1 -> DMA11STB Reg \n 0 -> DMA11STA Reg*/
#define PingPongStatusDMA11()            _PPST11

#define EnableIntDMA11()                 _DMA11IE = 1
#define DisableIntDMA11()                _DMA11IE = 0
#define SetIntPriorityDMA11(priority)    _DMA11IP = (priority)
#define DMA11_INT_FLAG                   _DMA11IF

/* Setting the priority of DMA11 interrupt */
#define DMA11_INT_PRI_0                   0
#define DMA11_INT_PRI_1                   1
#define DMA11_INT_PRI_2                   2
#define DMA11_INT_PRI_3                   3
#define DMA11_INT_PRI_4                   4
#define DMA11_INT_PRI_5                   5
#define DMA11_INT_PRI_6                   6
#define DMA11_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch12 Functions *****************************/
/*****************************************************************************/
#if defined _DMA12IE

/***********************************************************************************************************************
 * \Function        void SetupDMA12Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA12 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA12Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA12Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA12Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA12CONbits.SIZE  = DataTransferSize;\
		DMA12CONbits.DIR   = DataTransferDir;\
		DMA12CONbits.NULLW = DataTransferDir>>1;\
		DMA12PAD = PeripheralBuffer;\
		DMA12REQ = PeripheralIRQ;\
		DMA12CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA12_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA12 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA12_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA12_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA12_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA12CONbits.MODE  = OperatingMode;\
		DMA12CONbits.AMODE = AdressingMode;\
		DMA12STAL = Buf_A_StartAddress;\
		DMA12STAH = 0x0000;\
		DMA12STBL = Buf_B_StartAddress;\
		DMA12STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA12Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA12 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA12Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA12Interrupt(InterruptMode,InterruptPriority)({\
	DMA12CONbits.HALF = InterruptMode;\
	_DMA12IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA12()             DMA12CONbits.CHEN = 1
#define DisableChannelDMA12()            DMA12CONbits.CHEN = 0

/** Returns DMA12 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA12()     _PWCOL12
/** Returns DMA12 Channel Request Collision detect flag bit */
#define RequestCollisionDMA12()          _RQCOL12
/** Returns the register selected for ping-pong :\n 1 -> DMA12STB Reg \n 0 -> DMA12STA Reg*/
#define PingPongStatusDMA12()            _PPST12

#define EnableIntDMA12()                 _DMA12IE = 1
#define DisableIntDMA12()                _DMA12IE = 0
#define SetIntPriorityDMA12(priority)    _DMA12IP = (priority)
#define DMA12_INT_FLAG                   _DMA12IF

/* Setting the priority of DMA12 interrupt */
#define DMA12_INT_PRI_0                   0
#define DMA12_INT_PRI_1                   1
#define DMA12_INT_PRI_2                   2
#define DMA12_INT_PRI_3                   3
#define DMA12_INT_PRI_4                   4
#define DMA12_INT_PRI_5                   5
#define DMA12_INT_PRI_6                   6
#define DMA12_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch13 Functions *****************************/
/*****************************************************************************/
#if defined _DMA13IE

/***********************************************************************************************************************
 * \Function        void SetupDMA13Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA13 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA13Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA13Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA13Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA13CONbits.SIZE  = DataTransferSize;\
		DMA13CONbits.DIR   = DataTransferDir;\
		DMA13CONbits.NULLW = DataTransferDir>>1;\
		DMA13PAD = PeripheralBuffer;\
		DMA13REQ = PeripheralIRQ;\
		DMA13CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA13_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA13 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA13_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA13_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA13_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA13CONbits.MODE  = OperatingMode;\
		DMA13CONbits.AMODE = AdressingMode;\
		DMA13STAL = Buf_A_StartAddress;\
		DMA13STAH = 0x0000;\
		DMA13STBL = Buf_B_StartAddress;\
		DMA13STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA13Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA13 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA13Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA13Interrupt(InterruptMode,InterruptPriority)({\
	DMA13CONbits.HALF = InterruptMode;\
	_DMA13IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA13()             DMA13CONbits.CHEN = 1
#define DisableChannelDMA13()            DMA13CONbits.CHEN = 0

/** Returns DMA13 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA13()     _PWCOL13
/** Returns DMA13 Channel Request Collision detect flag bit */
#define RequestCollisionDMA13()          _RQCOL13
/** Returns the register selected for ping-pong :\n 1 -> DMA13STB Reg \n 0 -> DMA13STA Reg*/
#define PingPongStatusDMA13()            _PPST13

#define EnableIntDMA13()                 _DMA13IE = 1
#define DisableIntDMA13()                _DMA13IE = 0
#define SetIntPriorityDMA13(priority)    _DMA13IP = (priority)
#define DMA13_INT_FLAG                   _DMA13IF

/* Setting the priority of DMA13 interrupt */
#define DMA13_INT_PRI_0                   0
#define DMA13_INT_PRI_1                   1
#define DMA13_INT_PRI_2                   2
#define DMA13_INT_PRI_3                   3
#define DMA13_INT_PRI_4                   4
#define DMA13_INT_PRI_5                   5
#define DMA13_INT_PRI_6                   6
#define DMA13_INT_PRI_7                   7
#endif

/*****************************************************************************/
/**************************** DMA Ch14 Functions *****************************/
/*****************************************************************************/
#if defined _DMA14IE

/***********************************************************************************************************************
 * \Function        void SetupDMA14Channel(DataTransferSize,  TransferCount,  DataTransferDir,  PeripheralBuffer,  PeripheralIRQ)
 *
 * \Description     Configures DMA14 Channel
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>DataTransferSize:</u>\n
 *  - DATA_SIZE_16_BIT  \n
 *  - DATA_SIZE_8_BIT
 *
 * <u>TransferCount:</u>\n
 *  - A value between 1 - 16383 inclusive.  <i>/ For DMA Interrupt Requst.</i>
 *
 * <u>DataTransferDir:</u>\n
 *  - PERIPHERAL_TO_RAM             <i>/ Move Data From The Peripheral to The RAM On Peripheral Interrupt Rquest.</i>\n
 *  - RAM_TO_PERIPHERAL             <i>/ Move Data From The RAM to The Peripheral On Peripheral Interrupt Rquest.</i>\n
 *  - PERIPHERAL_TO_RAM_NULL_WR     <i>/ Move Data From The Peripheral to The RAM And Write-Back Null Data to Peripheral. (SPI Special)</i>\n
 *
 * <u>PeripheralBuffer:</u>\n
 *  - READ_FROM_IC1BUF        <i>/ IC1 : Input Capture 1.</i>\n
 *  - READ_FROM_IC2BUF        <i>/ IC2 : Input Capture 2.</i>\n
 *  - READ_FROM_IC3BUF        <i>/ IC3 : Input Capture 3.</i>\n
 *  - READ_FROM_IC4BUF        <i>/ IC4 : Input Capture 4.</i>\n
 *  - READ_FROM_SPI1BUF       <i>/ SPI1 Transfer Register.</i>\n
 *  - READ_FROM_SPI2BUF       <i>/ SPI2 Transfer Register.</i>\n
 *  - READ_FROM_SPI3BUF       <i>/ SPI3 Transfer Register.</i>\n
 *  - READ_FROM_SPI4BUF       <i>/ SPI4 Transfer Register.</i>\n
 *  - READ_FROM_U1RXREG       <i>/ UART1 Receiver.</i>\n
 *  - READ_FROM_U2RXREG       <i>/ UART2 Receiver.</i>\n
 *  - READ_FROM_U3RXREG       <i>/ UART3 Receiver.</i>\n
 *  - READ_FROM_U4RXREG       <i>/ UART4 Receiver.</i>\n
 *  - READ_FROM_C1RXD         <i>/ ECAN1 : RX Data.</i>\n
 *  - READ_FROM_C2RXD         <i>/ ECAN2 : RX Data.</i>\n
 *  - READ_FROM_RXBUF0        <i>/ DCI Receiver.</i>\n
 *  - READ_FROM_ADC1BUF0      <i>/ ADC1 Result Buffer.</i>\n
 *  - READ_FROM_ADC2BUF0      <i>/ ADC2 Result Buffer.</i>\n
 *  - READ_FROM_PMDIN1        <i>/ Parallel Master Port Data.</i>\n
 *  - WRITE_TO_OC1R           <i>/ OC1 : Output Compare 1 Register.</i>\n
 *  - WRITE_TO_OC1RS          <i>/ OC1 : Output Compare 1 Reset Register.</i>\n
 *  - WRITE_TO_OC2R           <i>/ OC2 : Output Compare 2 Register.</i>\n
 *  - WRITE_TO_OC2RS          <i>/ OC2 : Output Compare 2 Reset Register.</i>\n
 *  - WRITE_TO_OC3R           <i>/ OC3 : Output Compare 3 Register.</i>\n
 *  - WRITE_TO_OC3RS          <i>/ OC3 : Output Compare 3 Reset Register.</i>\n
 *  - WRITE_TO_OC4R           <i>/ OC4 : Output Compare 4 Register.</i>\n
 *  - WRITE_TO_OC4RS          <i>/ OC4 : Output Compare 4 Reset Register.</i>\n
 *  - WRITE_TO_SPI1BUF        <i>/ SPI1 Transfer Register.</i>\n
 *  - WRITE_TO_SPI2BUF        <i>/ SPI2 Transfer Register.</i>\n
 *  - WRITE_TO_SPI3BUF        <i>/ SPI3 Transfer Register.</i>\n
 *  - WRITE_TO_SPI4BUF        <i>/ SPI4 Transfer Register.</i>\n
 *  - WRITE_TO_U1TXREG        <i>/ UART1 Transmitter.</i>\n
 *  - WRITE_TO_U2TXREG        <i>/ UART2 Transmitter.</i>\n
 *  - WRITE_TO_U3TXREG        <i>/ UART3 Transmitter.</i>\n
 *  - WRITE_TO_U4TXREG        <i>/ UART4 Transmitter.</i>\n
 *  - WRITE_TO_C1TXD          <i>/ ECAN1 : TX Data.</i>\n
 *  - WRITE_TO_C2TXD          <i>/ ECAN2 : TX Data.</i>\n
 *  - WRITE_TO_TXBUF0         <i>/ DCI Transmitter.</i>\n
 *  - WRITE_TO_PMDIN1         <i>/ Parallel Master Port Data.</i>\n
 *
 * <u>PeripheralIRQ:</u> <i>/ Which Peripheral Interrupt Requst Will Start a Data Transfer.</i>\n
 *
 *  - INT0_IRQ          <i>/ External Interrupt 0.</i>\n
 *  - IC1_IRQ           <i>/ Input Capture 1.</i>\n
 *  - OC1_IRQ           <i>/ Output Compare 1.</i>\n
 *  - IC2_IRQ           <i>/ Input Capture 2.</i>\n
 *  - OC2_IRQ           <i>/ Output Compare 2.</i>\n
 *  - TMR2_IRQ          <i>/ Timer2.</i>\n
 *  - TMR3_IRQ          <i>/ Timer3.</i>\n
 *  - SPI1_IRQ          <i>/ Transfer done.</i>\n
 *  - UART1RX_IRQ       <i>/ UART1 Receiver.</i>\n
 *  - UART1TX_IRQ       <i>/ UART1 Transmitter.</i>\n
 *  - ADC1_IRQ          <i>/ ADC1 convert done.</i>\n
 *  - ADC2_IRQ          <i>/ ADC2 convert done.</i>\n
 *  - OC3_IRQ           <i>/ Output Compare 3.</i>\n
 *  - OC4_IRQ           <i>/ Output Compare 4.</i>\n
 *  - TMR4_IRQ          <i>/ Timer4.</i>\n
 *  - TMR5_IRQ          <i>/ Timer5.</i>\n
 *  - UART2RX_IRQ       <i>/ UART2 Receiver.</i>\n
 *  - UART2TX_IRQ       <i>/ UART2 Transmitter.</i>\n
 *  - SPI2_IRQ          <i>/ Transfer done.</i>\n
 *  - ECAN1RX_IRQ       <i>/ RX data ready.</i>\n
 *  - IC3_IRQ           <i>/ Input Capture 3.</i>\n
 *  - IC4_IRQ           <i>/ Input Capture 4.</i>\n
 *  - PMP_IRQ           <i>/ Data mode.</i>\n
 *  - ECAN2RX_IRQ       <i>/ RX data ready.</i>\n
 *  - DCI_IRQ           <i>/ DCI transfer done.</i>\n
 *  - ECAN1TX_IRQ       <i>/ TX data request.</i>\n
 *  - ECAN2TX_IRQ       <i>/ TX data request.</i>\n
 *  - UART3RX_IRQ       <i>/ UART3 Receiver.</i>\n
 *  - UART3TX_IRQ       <i>/ UART3 Transmitter.</i>\n
 *  - UART4RX_IRQ       <i>/ UART4 Receiver.</i>\n
 *  - UART4TX_IRQ       <i>/ UART4 Transmitter.</i>\n
 *  - SPI3_IRQ          <i>/ Transfer done.</i>\n
 *  - SPI4_IRQ          <i>/ Transfer done.</i>
 *
 * \Return      None
 *
 * \Notes       - Word data can only be moved to and from aligned (even) addresses. But byte data can be moved to or
 *              from any (legal) address.\n
 *		- Look Into Device Datasheet For DMA Supported Peripherals.
 *
 * \Example     SetupDMA14Channel(DATA_SIZE_16_BIT,4,PERIPHERAL_TO_RAM,READ_FROM_ADC1BUF0,TMR3_IRQ);	\n
 *              SetupDMA14Channel(DATA_SIZE_16_BIT,4,RAM_TO_PERIPHERAL,WRITE_TO_U1TXREG,TMR3_IRQ);
 *
 **********************************************************************************************************************/
#define SetupDMA14Channel(DataTransferSize,TransferCount,DataTransferDir,PeripheralBuffer,PeripheralIRQ)({   \
		DMA14CONbits.SIZE  = DataTransferSize;\
		DMA14CONbits.DIR   = DataTransferDir;\
		DMA14CONbits.NULLW = DataTransferDir>>1;\
		DMA14PAD = PeripheralBuffer;\
		DMA14REQ = PeripheralIRQ;\
		DMA14CNT = (TransferCount)-1;})

/***********************************************************************************************************************
 * \Function        void SetupDMA14_RAM_Buffers(OperatingMode,  AdressingMode, uint16_t* Buf_A_StartAddress, uint16_t* Buf_B_StartAddress)
 *
 * \Description     Configures DMA14 RAM Buffers.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>OperatingMode:</u> \n
 *  - CONTINUOUS_NORMAL_TRANSFER    <i>/ Continuous mode is used when repetitive data transfer is required throughout the life of the program.</i>\n
 *  - ONE_SHOT_NORMAL_TRANSFER      \n
 *  - CONTINUOUS_PINGPONG_TRANSFER  <i>/ Ping-Pong mode allows the CPU to process one buffer while a second buffer operates with the DMA channel.</i>\n
 *  - ONE_SHOT_PINGPONG_TRANSFER
 *
 * <u>AdressingMode:</u>\n
 *  - REG_ADDR_POST_INC   <i>/ Register Indirect with Post-Increment Addressing mode is used to move blocks of data by incrementing the DPSRAM/RAM address after each transfer.</i>\n
 *  - REG_ADDR_NO_INC     <i>/ Register Indirect without Post-Increment Addressing mode is used to move blocks of data without incrementing the starting address of the data buffer after each transfer.</i>\n
 *  - PERIPHERAL_ADDR     <i>/ Peripheral Indirect Addressing mode is a special addressing mode where the peripheral, not the DMA channel, provides the variable part of the DPSRAM/RAM address.</i>
 *
 * <u>Buf_A_StartAddress:</u>\n
 *  - Pointer to Buffer A.
 *
 * <u>Buf_B_StartAddress:</u>\n
 *  - Pointer to Buffer B if PingPong Mode is Used.
 *
 * \Return      None
 *
 * \Notes       - If the DMA RAM (DPSRAM) is in the extended data space, then __eds__tag must be used to declare the
 *              buffers (Refer To the Device Datasheet).\n
 *              - If the DMA module attempts to access any unimplemented memory address, a DMA Address Error Trap is
 *              issued, and the DMA Address Error Soft Trap Status bit (DAE) is set.
 *
 * \Example     __eds__ unsigned int BufferA[10] __attribute__((eds,space(dma)));	\n
		__eds__ unsigned int Buffer __attribute__((eds,space(dma)));
 *
 *		SetupDMA14_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_POST_INC,__builtin_dmaoffset(BufferA),0); \n
 *		SetupDMA14_RAM_Buffers(CONTINUOUS_NORMAL_TRANSFER,REG_ADDRESS_NO_INC,__builtin_dmaoffset(&Buffer),0);
 *
 **********************************************************************************************************************/
#define SetupDMA14_RAM_Buffers(OperatingMode,AdressingMode,Buf_A_StartAddress,Buf_B_StartAddress)({\
		DMA14CONbits.MODE  = OperatingMode;\
		DMA14CONbits.AMODE = AdressingMode;\
		DMA14STAL = Buf_A_StartAddress;\
		DMA14STAH = 0x0000;\
		DMA14STBL = Buf_B_StartAddress;\
		DMA14STBH = 0x0000;})

/***********************************************************************************************************************
 * \Function        void SetupDMA14Interrupt(InterruptMode, uint8_t InterruptPriority)
 *
 * \Description     Configures DMA14 Interrupts.
 *
 * \PreCondition    None
 *
 * \Inputs
 * <u>InterruptMode:</u> \n
 *  - WHEN_ALL_DATA_MOVED       <i>/ The Data Has Moved a "TransferCount" Times.</i>\n
 *  - WHEN_HALF_DATA_MOVED      <i>/ The Data Has Moved a "TransferCount/2" Times.</i>
 *
 * <u>InterruptPriority:</u>\n
 *  - A Value Between 0 and 7.
 *
 * \Return      None
 *
 * \Example     SetupDMA14Interrupt(WHEN_ALL_DATA_MOVED,3);
 *
 **********************************************************************************************************************/
#define SetupDMA14Interrupt(InterruptMode,InterruptPriority)({\
	DMA14CONbits.HALF = InterruptMode;\
	_DMA14IP  = InterruptPriority;})

/**********************************************************************************************************************/
/** Enable The DMA Channel After Completing All its Configuration Setup*/
#define EnableChannelDMA14()             DMA14CONbits.CHEN = 1
#define DisableChannelDMA14()            DMA14CONbits.CHEN = 0

/** Returns DMA14 channel peripheral write collision flag bit*/
#define WriteCollisionPeripheralDMA14()     _PWCOL14
/** Returns DMA14 Channel Request Collision detect flag bit */
#define RequestCollisionDMA14()          _RQCOL14
/** Returns the register selected for ping-pong :\n 1 -> DMA14STB Reg \n 0 -> DMA14STA Reg*/
#define PingPongStatusDMA14()            _PPST14

#define EnableIntDMA14()                 _DMA14IE = 1
#define DisableIntDMA14()                _DMA14IE = 0
#define SetIntPriorityDMA14(priority)    _DMA14IP = (priority)
#define DMA14_INT_FLAG                   _DMA14IF

/* Setting the priority of DMA14 interrupt */
#define DMA14_INT_PRI_0                   0
#define DMA14_INT_PRI_1                   1
#define DMA14_INT_PRI_2                   2
#define DMA14_INT_PRI_3                   3
#define DMA14_INT_PRI_4                   4
#define DMA14_INT_PRI_5                   5
#define DMA14_INT_PRI_6                   6
#define DMA14_INT_PRI_7                   7
#endif

/*****************************************************************************/
/*************************** DMA General Functions ***************************/
/*****************************************************************************/

/** Returns the Last Address Accessed By DMA */
#define LastAddressAccessedByDMA()	((unsigned long) (DSADRL + ((unsigned long) DSADRH<<16)))
/** Returns The last DMA Active Channel Number\  If Returned Value = 15 So No DMA transfer has occurred since system Reset*/
#define LastDMAActive()			_LSTCH

/**********************************************************************************************************************/
// <editor-fold defaultstate="collapsed" desc="DMA Library Defines">

#ifndef _COMMON_DIRECTIVES_
#define _COMMON_DIRECTIVES_

#define DONT_CHANGE     0xFFFF
#define DONT_CARE       0
#define KEEP_DEFAULT    0
#define SET_DEFAULT     0

#endif

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* SetupDMAx_RAM_Buffers */

// AdressingMode
#define REG_ADDR_POST_INC	0
#define REG_ADDR_NO_INC		1
#define PERIPHERAL_ADDR		2

// OperatingMode
#define CONTINUOUS_NORMAL_TRANSFER	0
#define ONE_SHOT_NORMAL_TRANSFER	1
#define CONTINUOUS_PINGPONG_TRANSFER	2
#define ONE_SHOT_PINGPONG_TRANSFER	3

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* SetupDMAx */

// DataTransferSize
#define DATA_SIZE_16_BIT	0
#define DATA_SIZE_8_BIT		1

/*TransferCount = 1 TO 16383*/ 

// DataTransferDir
#define PERIPHERAL_TO_RAM		0
#define RAM_TO_PERIPHERAL		1
#define PERIPHERAL_TO_RAM_NULL_WR	2

// PeripheralBuffer
#ifdef IC1BUF
#define READ_FROM_IC1BUF        0x0144
#endif
#ifdef IC2BUF
#define READ_FROM_IC2BUF        0x014C
#endif
#ifdef IC3BUF
#define READ_FROM_IC3BUF        0x0154
#endif
#ifdef IC4BUF
#define READ_FROM_IC4BUF        0x015C
#endif
#ifdef OC1R
#define WRITE_TO_OC1R           0x0906
#define WRITE_TO_OC1RS          0x0904
#endif
#ifdef OC2R
#define WRITE_TO_OC2R           0x0910
#define WRITE_TO_OC2RS          0x090E
#endif
#ifdef OC3R
#define WRITE_TO_OC3R           0x091A
#define WRITE_TO_OC3RS          0x0918
#endif
#ifdef OC4R
#define WRITE_TO_OC4R           0x0924
#define WRITE_TO_OC4RS          0x0922
#endif
#ifdef SPI1BUF
#define READ_FROM_SPI1BUF       0x0248
#define WRITE_TO_SPI1BUF        0x0248
#endif
#ifdef SPI2BUF
#define READ_FROM_SPI2BUF       0x0268
#define WRITE_TO_SPI2BUF        0x0268
#endif
#ifdef SPI3BUF
#define READ_FROM_SPI3BUF       0x02A8
#define WRITE_TO_SPI3BUF        0x02A8
#endif
#ifdef SPI4BUF
#define READ_FROM_SPI4BUF       0x02C8
#define WRITE_TO_SPI4BUF        0x02C8
#endif
#ifdef U1RXREG
#define READ_FROM_U1RXREG       0x0226
#define WRITE_TO_U1TXREG        0x0224
#endif
#ifdef U2RXREG
#define READ_FROM_U2RXREG       0x0236
#define WRITE_TO_U2TXREG        0x0234
#endif
#ifdef U3RXREG
#define READ_FROM_U3RXREG       0x0256
#define WRITE_TO_U3TXREG        0x0254
#endif
#ifdef U4RXREG
#define READ_FROM_U4RXREG       0x02B6
#define WRITE_TO_U4TXREG        0x02B4
#endif
#ifdef C1RXD
#define READ_FROM_C1RXD         0x0440  // ECAN1 ? RX
#define WRITE_TO_C1TXD          0x0442
#endif
#ifdef C2RXD
#define READ_FROM_C2RXD         0x0540
#define WRITE_TO_C2TXD          0x0542
#endif
#ifdef RXBUF0
#define READ_FROM_RXBUF0        0x0290
#define WRITE_TO_TXBUF0         0x0298	// DCI ? CODEC Transfer Done
#endif
#ifdef ADC1BUF0
#define READ_FROM_ADC1BUF0      0x0300
#endif
#ifdef ADC2BUF0
#define READ_FROM_ADC2BUF0      0x0340
#endif
#ifdef PMDIN1
#define READ_FROM_PMDIN1        0x0608
#define WRITE_TO_PMDIN1         0x0608
#endif

// PeripheralIRQ
#define INT0_IRQ	0b00000000 // External Interrupt 0
#define IC1_IRQ		0b00000001 // Input Capture 1
#define OC1_IRQ		0b00000010 // Output Compare 1
#define IC2_IRQ		0b00000101 // Input Capture 2
#define OC2_IRQ		0b00000110 // Output Compare 2
#define TMR2_IRQ	0b00000111 // Timer2
#define TMR3_IRQ	0b00001000 // Timer3
#define SPI1_IRQ	0b00001010 // Transfer done
#define UART1RX_IRQ	0b00001011 // UART1 Receiver
#define UART1TX_IRQ	0b00001100 // UART1 Transmitter
#define ADC1_IRQ	0b00001101 // ADC1 convert done
#define ADC2_IRQ	0b00010101 // ADC2 convert done
#define OC3_IRQ		0b00011001 // Output Compare 3
#define OC4_IRQ		0b00011010 // Output Compare 4
#define TMR4_IRQ	0b00011011 // Timer4
#define TMR5_IRQ	0b00011100 // Timer5
#define UART2RX_IRQ	0b00011110 // UART2 Receiver
#define UART2TX_IRQ	0b00011111 // UART2 Transmitter
#define SPI2_IRQ	0b00100001 // Transfer done
#define ECAN1RX_IRQ	0b00100010 // RX data ready
#define IC3_IRQ		0b00100101 // Input Capture 3
#define IC4_IRQ		0b00100110 // Input Capture 4
#define PMP_IRQ		0b00101101 // Data mode
#define ECAN2RX_IRQ	0b00110111 // RX data ready
#define DCI_IRQ		0b00111100 // DCI transfer done
#define ECAN1TX_IRQ	0b01000110 // TX data request
#define ECAN2TX_IRQ	0b01000111 // TX data request
#define UART3RX_IRQ	0b01010010 // UART3 Receiver
#define UART3TX_IRQ	0b01010011 // UART3 Transmitter
#define UART4RX_IRQ	0b01011000 // UART4 Receiver
#define UART4TX_IRQ	0b01011001 // UART4 Transmitter
#define SPI3_IRQ	0b01011011 // Transfer done
#define SPI4_IRQ	0b01111011 // Transfer done

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/* SetupDMAxInterrupt */

// InterruptMode
#define WHEN_ALL_DATA_MOVED		0
#define WHEN_HALF_DATA_MOVED	1

// </editor-fold>
#endif
