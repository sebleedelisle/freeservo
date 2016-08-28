//---------------------------------------------------------------------
//
//							 Software License Agreement
//
// The software supplied herewith by Microchip Technology Incorporated 
// (the “Company”) for its PICmicro® Microcontroller is intended and 
// supplied to you, the Company’s customer, for use solely and 
// exclusively on Microchip PICmicro Microcontroller products. The 
// software is owned by the Company and/or its supplier, and is 
// protected under applicable copyright laws. All rights are reserved. 
//  Any use in violation of the foregoing restrictions may subject the 
// user to criminal sanctions under applicable laws, as well as to 
// civil liability for the breach of the terms and conditions of this 
// license.
//
// THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES, 
// WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
// TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
// PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT, 
// IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR 
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
//
//---------------------------------------------------------------------
//	File:		ClosedLoopSenBLDC.c
//
//	Written By:		Stan D'Souza, Microchip Technology
//						
// 
// The following files should be included in the MPLAB project:
//
//		ClosedLoopSenBLDC.c		-- Main source code file
//		p30f2010.gld	-- Linker script file
//				
//
//---------------------------------------------------------------------
//
// Revision History
//
// 10/01/04 -- first version 
//---------------------------------------------------------------------- 


#define __dsPIC30F2010__
#include "c:\pic30_tools\support\h\p30F2010.h"


#define FCY  10000000			// xtal = 5.0Mhz; PLLx8
#define MILLISEC FCY/10000			// 1 mSec delay constant
#define FPWM 39000
#define POLEPAIRS	4		// number of pole pairs
#define INDEX	1			// Hall sensor position index

#define S2	!PORTCbits.RC14
#define S3	!PORTCbits.RC13

#define CR	0x0D
#define LF 	0x0A
#define BAUD 19200
#define SPEEDMULT	2343750
#define OFFSET 8


void InitADC10(void);
void AverageADC(void);
void DelayNmSec(unsigned int N);
void InitMCPWM(void);
void InitUART(void);
void SendCRLF(void);
void SendSpeed(void);
void InitTMR3(void);
void SendMsg(void);



struct {
			unsigned RunMotor : 	1;
			unsigned SndSpeed :		1;
			unsigned CheckRX :		1;
			unsigned SendTX :		1;
			unsigned unused 	:	14;
		}	Flags;

unsigned int HallValue;
unsigned int timer3value;
unsigned int timer3avg;
unsigned char polecount;
unsigned char *TXPtr;
unsigned char *RXPtr;

unsigned char InData[] = {"000000"};
unsigned char OutData[] = {"Speed = 00000 rpm\r"};


/*************************************************************
	Low side driver table is as below.  In the StateLoTableClk
	and the StateLoTableAntiClk tables, the Low side driver is
	PWM while the high side driver is either on or off.  
*************************************************************/

unsigned int StateLoTableClk[] = {0x0000, 0x0210, 0x2004, 0x0204,
									0x0801, 0x0810, 0x2001, 0x0000};
unsigned int StateLoTableAntiClk[] = {0x0000, 0x2001, 0x0810, 0x0801,
									0x0204, 0x2004, 0x0210, 0x0000};

/****************************************************************
Interrupt vector for Change Notification CN5, 6 and 7 is as below.
When a Hall sensor changes states, an interrupt will be
caused which will vector to the routine below
The program then reads PORTB, mask bits 3, 4 and 5,
shift and adjust the value to read as 1, 2 ... 6.  This 
value is then used as an offset in the lookup table StateLoTableClk
or StateLoTableAntiClk to determine the value loaded in the OCDCON 
register.  This routine also reads the Hall sensors and counts
up to the POLEPAIRS to determine the time for one mechanical
revolution using the fact that 1 mech rev = POLEPAIR*(1 elect. rev)
*****************************************************************/ 
void __attribute__((__interrupt__)) _CNInterrupt (void)
{
	IFS0bits.CNIF = 0;				// clear flag
	HallValue = PORTB & 0x0038;	// mask RB3,4 & 5
	HallValue = HallValue >> 3;	// shift right 3 times
	OVDCON = StateLoTableAntiClk[HallValue];
	if (HallValue == INDEX)	// has the same position been sensed?	
		if (polecount++ == POLEPAIRS)	//has one mech rev elasped?
		{								// yes then read timer3
		timer3value = TMR3;
		TMR3 = 0;
		timer3avg = ((timer3avg + timer3value) >> 1);
		polecount = 1;

		} 
}
//---------------------------------------------------------------------

void __attribute__((__interrupt__)) _U1TXInterrupt(void)
{
	IFS0bits.U1TXIF = 0;	// clear interrupt flag
}

void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
	IFS0bits.U1RXIF = 0;	// clear interrupt flag
	*RXPtr = U1RXREG;
	if (*RXPtr == CR)
		{Flags.CheckRX = 1;RXPtr = &InData[0];}
	else *RXPtr++;
}
  


/*********************************************************************
The ADC interrupt loads the PDCx registers with the demand pot
value.  This is only done when the motor is running.

*********************************************************************/

void __attribute__((__interrupt__)) _ADCInterrupt (void)
{
	IFS0bits.ADIF = 0;
	if (Flags.RunMotor)
	{
		PDC1 = ADCBUF0 >> 1;					// get value ...
		PDC2 = PDC1;					// and load all three PWMs ...
		PDC3 = PDC1;					// duty cycles
		Flags.SndSpeed = 1;			// send speed info serially
	}
}

int main(void)
{
	LATE = 0x0000;
	TRISE = 0xFFC0;		// PWMs are outputs
	CNEN1 = 0x00E0;		// CN5,6 and 7 enabled
	CNPU1 = 0x00E0;		// enable internal pullups
	IFS0bits.CNIF = 0;	// clear CNIF
	IEC0bits.CNIE = 1;	// enable CN interrupt
	InitMCPWM();
	InitADC10();
	InitUART();
	InitTMR3();
	timer3avg = 0;
	while(1)
	{		while (!S2);				// wait for start key hit
		while (S2)					// wait till key is released
			DelayNmSec(10);
		// read hall position sensors on PORTB
		HallValue = PORTB & 0x0038;			// mask RB3,4 & 5
		HallValue = HallValue >> 3;			// shift right to get value 1, 2 ... 6
		OVDCON = StateLoTableAntiClk[HallValue];	// Load the overide control register
		PWMCON1 = 0x0777;					// enable PWM outputs
		Flags.RunMotor = 1;					// set flag
		T3CONbits.TON = 1;				// start tmr3
		polecount = 1;
		U1STAbits.UTXEN = 1;			// start transmission
		U1TXREG = CR;
		U1TXREG = LF;
		DelayNmSec(100);
		while (Flags.RunMotor)				// while motor is running
			{
			if (S2)							// if S2 is pressed
				{
				PWMCON1 = 0x0700;			// disable PWM outputs
  				OVDCON = 0x0000;			// overide PWM low.
				U1STA = 0x0000;				// clear transmit
				Flags.RunMotor = 0;			// reset run flag
				while (S2)					// wait for key release
					DelayNmSec(10);
				}
			if (Flags.SndSpeed)
				SendSpeed();
			}
	}	// end of while (1)
	

}
	
/*******************************************************************
		Below is the code required to setup the ADC registers for :
		1. 1 channel conversion (in this case RB2/AN2)
		2. PWM trigger starts conversion
		3. Pot is connected to CH0 and RB2
		4. Manual Stop Sampling and start converting
		5. Manual check of Conversion complete 
																
*********************************************************************/
void InitADC10(void)
{

 ADPCFG = 0xFFF8;				// all PORTB = Digital;RB0 to RB2 = analog
 ADCON1 = 0x0064;				// PWM starts conversion
 ADCON2 =  0x0200;				// simulataneous sample 4 channels									
 ADCHS = 0x0002;				// Connect RB2/AN2 as CH0 = pot  ..
								// ch1 = Vbus, Ch2 = Motor, Ch3 = pot
 ADCON3 = 0x0005;				// Tad = 3Tcy
 IFS0bits.ADIF = 0;
 IEC0bits.ADIE = 1;

 ADCON1bits.ADON = 1;			// turn ADC ON
}

/********************************************************************
InitMCPWM, intializes the PWM as follows:
1. FPWM = 16000 hz
2. Independant PWMs
3. Control outputs using OVDCON
4.	Set Duty Cycle with the ADC value read from pot
5. Set ADC to be triggered by PWM special trigger
*********************************************************************/

void InitMCPWM(void)
{
	PTPER = FCY/FPWM - 1;

	PWMCON1 = 0x0700;			// disable PWMs
	OVDCON = 0x0000;			// allow control using OVD
	PDC1 = 100;					// init PWM 1, 2 and 3 to 100
	PDC2 = 100;
	PDC3 = 100;
	SEVTCMP = PTPER;
	PWMCON2 = 0x0F00;		// 16 postscale values
	PTCON = 0x8000;			// start PWM
	 
}

/************************************************************************
Tmr3 is used to determine the rotor speed so it is set to count using Tcy/256

*************************************************************************/


void InitTMR3(void)
{
	T3CON = 0x0030;			// internal Tcy/256 clock
	TMR3 = 0;
	PR3 = 0xFFFF;
}

void InitUART(void)
{
//---------------------------------------------------------------------
// Initialize the UART1 for BAUD = 19,200 
 U1MODE = 0x8000;
 U1STA = 0x0000;
 U1BRG = ((FCY/16)/BAUD) - 1;	// set baud to 19200
 //IEC0bits.U1RXIE = 1;
 RXPtr = &InData[0];		// point to first char in string
 Flags.CheckRX = 0;
 Flags.SendTX = 0;

// U1STAbits.UTXEN = 1;           // Initiate transmission
}

//------------------------------------------------------------------------
// SendSpeed sends the speed information on the uart at 19200 baud
//------------------------------------------------------------------------
void SendSpeed()
{
unsigned int k;
unsigned char c;
	
	k = SPEEDMULT/timer3avg;
	c = k/10000;
	if (c > 0)
		k = k - c*10000;
	OutData[OFFSET] = (c + 0x30);
	c = k/1000;
	if (c > 0)
		k = k - c*1000;
	OutData[OFFSET+1] = (c + 0x30);
	c = k/100;
	if (c > 0)
		k = k - c*100;
	OutData[OFFSET+2] = (c + 0x30);
	c = k/10;
	if (c > 0)
		k = k - c*10;
	OutData[OFFSET+3] = (c + 0x30);
	OutData[OFFSET+4] = (char)(k + 0x30);
	TXPtr = &OutData[0];
	SendMsg();
	Flags.SndSpeed = 0;
}

void SendMsg(void)
{
while (*TXPtr)
	{
	while (U1STAbits.UTXBF);
	U1TXREG = *TXPtr++;
	}
}




//---------------------------------------------------------------------
// This is a generic 1ms delay routine to give a 1mS to 65.5 Seconds delay
// For N = 1 the delay is 1 mS, for N = 65535 the delay is 65,535 mS. 
// Note that FCY is used in the computation.  Please make the necessary
// Changes(PLLx4 or PLLx8 etc) to compute the right FCY as in the define
// statement above.

void DelayNmSec(unsigned int N)
{
unsigned int j;
while(N--)
 	for(j=0;j < MILLISEC;j++);
}
