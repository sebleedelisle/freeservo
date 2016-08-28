/**********************************************************************
 *                                                                     *
 *                        Software License Agreement                   *
 *                                                                     *
 *    The software supplied herewith by Microchip Technology           *
 *    Incorporated (the "Company") for its dsPIC controller            *
 *    is intended and supplied to you, the Company's customer,         *
 *    for use solely and exclusively on Microchip dsPIC                *
 *    products. The software is owned by the Company and/or its        *
 *    supplier, and is protected under applicable copyright laws. All  *
 *    rights are reserved. Any use in violation of the foregoing       *
 *    restrictions may subject the user to criminal sanctions under    *
 *    applicable laws, as well as to civil liability for the breach of *
 *    the terms and conditions of this license.                        *
 *                                                                     *
 *    THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION.  NO           *
 *    WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING,    *
 *    BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND    *
 *    FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE     *
 *    COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,  *
 *    INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.  *
 *                                                                     *
  **********************************************************************/

 /**********************************************************************
 *                                                                     * 
 *    Author: Smart Power Soutions, LLP                                * 
 *                                                                     *
 *    Filename:       setup.c         	                                *
 *    Date:           4/21/04                                         *
 *    File Version:   4.00                                             *
 *    Project:        53                                               *
 *    Drawing:        2                                                *
 *                                                                     *
 *    Tools used:    MPLAB C30 Compiler v 1.20.01                      *
 *                                                                     *
 *    Linker File:    p30f6010.gld                                   *
 *                                                                     *
 *                                                                     *
 ***********************************************************************
 *	Code Description
 *  
 *  This file contains all of the peripheral setup routines for the
 *  dsPIC.  There is also a WriteConfig() function, which is used to
 *  make sure the device configuration bits at runtime.
 *  In particular, the WriteConfig() function inverts the polarity of
 *  The PWM output signals.  Normally, this would be done because
 *  the gate driver circuits require active low polarity.  However, in
 *  this application, the polarity is changed so that the active portion
 *  of the duty cycle will occur at the end of the PWM timer count
 *  period.  The PWM is configured to trigger an A/D conversion at
 *  the end of the PWM cycle.  This allows the A/D to always be 
 *  triggered at the end of the on time, regardless of the PWM duty
 *  cycle.  The user must write a complemented duty cycle value to 
 *  the PDC registers for this technique to work.  (Note:  This trick
 *  will only work when the PWM is operating in the independent mode
 *  where no dead time insertion is used.)
 *
 *
 *                                          ADC triggered here
 *                                                |
 *  PWM1    _____                  _______________         
 *               |________________|               |_______
 *
 *  PWM2    _____          _______________________         
 *               |________|                       |_______ 
 *
 *  PWM3    _____                        _________         
 *               |______________________|         |_______ 
 *
 *
 **********************************************************************/

#include "general.h"
#include "hardware.h"
#include "defs.h"
#include "extern_globals.h"

void setup_ports(void);
void setup_motor_pwms(void);
void setup_adc(void);
void setup_qei(void);
void setup_timers(void);

unsigned int ReadConfig(int);
void WriteConfig(int,int);

#ifdef DEVELOPMODE
void setup_uart(void);
#endif

void setup_ports(void)
{
	// Clear All Ports Prior to defining I/O

	PORTB=0;
	PORTC=0;
	PORTD=0;
	PORTE=0;

	
	// Ensure Firing is disabled
	DISABLE_FIRING;
	
	// Now set pin direction registers
	

							
	TRISB = 0xFFFF;	// RB15 is the Output Compare(B) Fault Input -> input
							// RB14-0 Left as inputs as these are the analogue channels.							
							
	TRISC = 0x6000;			// RC14 is S2 Start/Stop switch -> input
							// RC13 is S3 Rev/Fwd switch -> input
							
	TRISD = 0x0003;	// RD0 and 1 default -> inputs used for debug EMUC2/EMUD2
							
							
	TRISE = 0x0100;			// RE8 is the input from the Trip Button for the PWM FLTA -> input
							// RE5-0 are the PWMs -> outputs
							

	return;
}

// Note that the normal operation of the PWM module
// is that switches turn on at the start of the cycle
// when the PWM ramp resets to zero. Switches turn off when
// the ramp reaches the duty cycle value.
// This means that if you want to use the special event 
// trigger to the ADC module, the natural place to put 
// the trigger is just after the switches turn on.
// This is so you don't have to keep moving the sample
// point which is not recommended as the SFR for
// the sample point (SEVTCMP) is not double buffered.
// If SEVTCMP is changed on the fly then the next ADC trigger
// might occur too early in the PWM cycle or may not happen.
// In this instance we would like to trigger ADC just before
// the switches turn off to get best sample of ibus. 
// This avoids problems with discontinuous current waveforms.
// In order to do this without changing SEVTCMP, all firing
// has been inverted. The actual ACTIVE state of firing is 1
// but this has been changed to be 0 in the config bits
// This means that switches turn off when ramp resets and
// back on when duty cycle (PDCx) = ramp.
// All duty cycle values therefore need to be corrected to be
// 2*PTER-D where D is the required duty.
// Also all definitions of what happens during faults and 
// output overrides need to be changed to drive swicthes 
// INACTIVE to be on and ACTIVE to be off.
// All comments below related to bit bit defs etc are for 
// conventional (non-inverted) firing.

void setup_motor_pwms(void)
{
	unsigned int config_value,temp;
	// Setup Motor Control PWM Module
	
		// PTCON - Timebase Control
		// Bit 15 1=Timebase is ON, 0=OFF
		// Bit 14 - Not Implemented
		// Bit 13 1=Timebase Halts in CPU idle,0 = runs
		// Bits12-8 - Not Implemented
		// Bits7-4 Timebase Interrupt Postscaler Select
		// 1111 = 1:16 Postscale
		// ::::
		// 0001 = 1:2 Postscale
		// 0000 = 1:1 Postscle
		// Bits3-2 Timebase I/p Clk Prescale Select
		// 11 = 1:64 Prescale
		// 10 = 1:16 Prescale
		// 01 = 1:4 Prescale
		// 00 = 1:1 Prescale
		// Bits1-0 Timebase Mode Select
		// 11 = Continuous Up/Down with Double Updates
		// 10 = Continuous Up/Down
		// 01 = Single Event
		// 00 = Free-running
		
	PTCON = 0x8000;		//Timebase On, runs in idle, no post or prescaler, free-running
	
		// PTPER - Period Register
		// Bit 15 - Not Implemented
		// Bits14-0 - Value used for period comparison and therefore
		//			  reset or change in direct of PWM ramp
		// PWM period is given by (PTER+1) * Tcy * Prescaler if in Free-run or single event
		// 				   and by (PTER+1) * Tcy * Prescaler / 2 otherwise
		
	PTPER = FULL_DUTY/2;		// With 7.3728MHz Fcy and no prescaler 
									// When FULL_DUTY= 922 this gives 16kHz PWM
	
		// SEVTCMP - Special Event Trigger Control
		// Bit15 1=Trigger Occurs When Counting Down, 0 = Up
		// Bits14-0 = Special Event Compare Value
						
							
	SEVTCMP = FULL_DUTY/2;	// Trigger occurs just before switch turns off
									// as firing is inverted
		
		// PWMCON1 - PWM Control Register #1
		// Bits15-12 Not Implemented
		// Bits11-8 1=PWM Pin Pair Independent, 0=Complementary
		// Bit11 = PWM4 --- Bit8 = PWM1
		// Bits7-4 1=PWM High Side Pin is Enabled for PWM, 0 = I/O
		// Bit7 = PWM4 --- Bit4 = PWM1
		// Bits3-0 1=PWM Low Side Pin is Enabled for PWM, 0 = I/O
		// Bit3 = PWM4 --- Bit4 = PWM1

	PWMCON1 = 0x0700;		// Phase#4 Firing Signals Not Used, all others independent
	

		// PWMCON2 - PWM Control Register #2
		// Bits15-12 Not Implemented
		// Bits11-8 Special Event Trigger Postscale
		// 1111 = 1:16 Postscale
		// ::::
		// 0001 = 1:2 Postscale
		// 0000 = 1:1 Postscle
		// Bits7-2 Not Implemented
		// Bit1 - 1=Output Overrides Synch to PWM Timebase
		//		  0=Output Overrides Occur on next Tcy boundary
		// Bit0 - 1=Updates from Duty Cycle and Period Registers Disabled
		//		  0=Updates from Duty Cycle and Period Registers Enabled
							
	PWMCON2 = 0x0000;		// No Special Event Prescale, Fast Overrides, Updates enabled
	
		// DTCON1 & 2 - Deadtime control registers
		// See manual for details of bits
		
	DTCON1 = 0x0000;		// Deadtime disabled as only modulating diagonally		
		
	
		// FLTACON - FaultA Input control register
		// Bits15-8 1=PWMs Driven ACTIVE on Fault, 0 = INACTIVE
		// Bit15 = #4 High Side
		// Bit14 = #4 Low Side
		// Bit13 = #3 High Side
		// :::::
		// Bit8 = #1 Low Side
		// Bit7 - 1=Cycle-cycle limit, 0=latched fault
		// Bits6-4 Not Implemented
		// Bits3-0 1=Pin Pair Controlled by FLTA, 0 = not controlled
		// Bit3 = #4 Pair
		// ::::
		// Bit0 = #1 Pair
							
			
	FLTACON = 0xFF0F;	//All pins driven ACTIVE with latched FLTA
							//but this turns switches off due to inversion
							//of ACTIVE definition


							

							
	OVDCON = 0x3FFF;		// All outputs (except #4 which is not used) made PWMs.
								// All override states = ACTIVE due to inversion
	
	// PDC1-4 - PWM#1-4 Duty Cycle Register
	// Bits15-0 PWM Duty Cycle Value
		
	PDC1=FULL_DUTY;			// In this instance 0 duty cycle is full value
	PDC2=FULL_DUTY;			// in duty cycle registers due to inversion
	PDC3=FULL_DUTY;

	// Note that the Pin Polarity Definition (ie ACTIVE=1 or ACTIVE=0)
	// And the Reset state is determined by device configuaration bits
	// usually set during programming.
	// However in this instance as inverting this definition we want
	// ACTIVE to be set to zero which is not what user would expect.
	// Therefore we will read the config value and check to see if the
	// bits which determine PWM operation are set correctly

	// Call ReadConfig with 4 first to get the bottom 16 bits of F80004
	// config register.

	config_value=ReadConfig(4);
	temp=config_value;
	// Mask off the bits we are interested in and shift them down
	temp=(temp & 0x0700)>>8;
	// If the PWM polarity and reset definition bits are set 
	// correctly already then temp should be set to 4
	if (temp != 4)
	{
		// Form correct config_value
		config_value &= 0xF0FF;
		config_value |= 0x0400;
		// Write the new config value into F80004
		WriteConfig(4,config_value);
	}
	// Now clear off any pending PWM or FLT interrupts due to config
	IFS2bits.PWMIF = 0;
	IFS2bits.FLTAIF =0;						
	return;
}

void setup_adc(void)
{
	// ADCON1 - ADC Control Regsiter#1
	// Bit 15 1=ADC Is On,0 = Off NB This bit should be set to turn on
	//		  the module after all other configuration bits are set.
	// Bit 14 - Not Implemented
	// Bit 13 1=Discontinue in IDLE, 0 = continue
	// Bits12-10 - Not Implemented
	// Bits9-8 - Output Data Formatting
	// 11 = Signed Fraction, 10 = Fractional
	// 01 = Signed Integer, 00 = Integer
	// Bits7-5 - Conversion Trigger Source
	// 111 = Internal counter ends sample and starts conversion
	// 110
	// :::
	// 100 = Reserved
	// 011 = Motor Control PWM Trigger ends sample and starts convert
	// 010 = TIMER3 compare ends sampling and starts conversion
	// 001 = INT0 transition ends sampling and starts conversion
	// 000 = Clearing Bit1 ends sampling and starts conversion
	// Bit4 - Not Implemented
	// Bit3 - Simulataneous Sample Select
	// See manual for details
	// Bit2 - 1 = Sampling begins immediately after last conversion complete
	// 		  0 = Sampling begins when Bit 1 (SAMP) set
	// Bit1 - See Manual
	// Bit0 - 1 = AD conversion cycle in progress, 0 = finished(Read),abort (write)
	
	ADCON1 = 0x006F; // Use PWM Trigger, continuous simulatenous sampling
	
	// ADCON2 - ADC Control Regsiter#2
	// Bits15-13 Voltage Reference Config
	// See Manual for details 000=AVdd/AVss used
	// Bit12 - Reserved but need to write 0
	// Bit11 - Not Implemented
	// Bit10 - Scan input Selection for CH0 S/H input
	// 		   1=scan inputs, 0 = don't scan
	// Bits9-8 S/H Channels used per sample
	//		   1x = All 4, 01 = CH0&1, 00 = CH0
	// Bit7 - Buffer Fill Status (when buffer into 2x8 words)
	//		  1=ADC filling 8-F, 0= ADC filling 0-7
	// Bit6 - Not Implemented
	// Bits5-2 No Sample/Convert Sequences / interrupt
	// 1111 = 16 sequences
	// ::::
	// 0000 = every sequence
	// Bit1 - 1=Results Buffer = 2x8,0 = 1x16
	// Bit0 - 0=Use MUXA settings, 1=Alternate MUXA & B starting with A
	
	ADCON2 = 0x0200; // Use Avdd/Avss, 4 samples, don't scan, 1 sequence/interrupt
							//Use MuxA settings
	
	// ADCON3 - ADC Control Regsiter#3
	// Bits15-13 - Not Implemented
	// Bits12-8 - Auto Sample Time Bits
	// 11111 = 31 TAD
	// :::::
	// 00000 = 0 TAD
	// Bit7 - 0=ADC Clock Derived from System, 1=ADC RC clock
	// Bit6 - Not Implemented
	// Bit5-0 ADC Conversion Clock
	// 111111 = 32 Tcy
	// ::::::
	// 000001 = Tcy
	// 000000 = Tcy/2

	ADCON3 = 0x0003; // Not using auto sample, system clock of 2Tcy
						  // giving 271ns Tad with 7.3728Mhz X4 PLL
						  // This gives approx 3us conversion time
	
	// ADCHS - A/D Input Select Register
	// See Manual For Details
	// Bits15-8 Are for MUXB
	// Bits7-0 Are for MUXA
	// If just doing single sample using channel 0
	// then can write channel number corresponding
	// to AN pin directly to ADCHS.
	
	ADCHS = VPH_RED; //MUXB not used, MUXA-ve=Vss,+ve=AN0-2 for CH1-3
						  //CH0 -ve=Vss +ve=AN12
	
	// ADPCFG - ADC port configuration register
	// Bits15-0 1=Digital, port read enabled, ADC Mux input=AVss
	//			0=Analogue, port read disabled, ADC samples pin
	
	ADPCFG = 0x8000; //AN15 used as OCFB digital pin
						  //For the moment assume that AN0,1 are used for ICD
						  //All other analogue channels

	// ADCSSL - ADC Scan Select Regsiter
	//	Bits15-0 - 1=ANx is selected for scan
	//				- 0=ANx is not selected for scan

	ADCSSL = 0x7020; //AN5, AN12-14 selected for scan

	// Clear Off Any Interrupt Due To Configuration
	IFS0bits.ADIF = 0;
	// Turn on ADC Module
	ADCON1bits.ADON=1;
	return;	
}

void setup_qei(void)
{

	// QEICON - QEI Control Register
	// Bit15 - 1=Position Error Count Has Occurred
	// Bit14 - Unused
	// Bit13 - 0=QEI Continues in idle, 1=discontinue
	// Bit12 - 1=Index Pin is High, 0=Low (Read Only)
	// Bit11 - 1=Pos Counter Dir is +,0=-ve
	//	Bits10-8 - Interface Mode Select - see manual
	// Bit7 - 1=A&B swapped
	// Bit6 - 1=DIR Output Enable,0=I/O Pin
	// Bit5 - 1=Enable Timer gate accumulation
	// Bits4-3 - Timer input prescale
	// 11 = 1:256
	// 10 = 1:64
	// 01 = 1:8
	// 00 = 1:1
	// Bit2 - 1=Index Pulse resets counter,0=doesn't
	// Bit1 - Timer Clock Select 1=QEA (rising),0 = Tcy
	// Bit0 - Counter Dir - 1=QEB state determines, 0=Bit11 determines

	QEICON = 0x0910;	//+ve count, 16 bit timer mode,DIR pin = I/O
							//1:64 divide from Tcy as clock = 115.2kHz with 7.3728 Mhz Tcy
	// DFLTCON - Digital Filter Control Register
	// Bit15-8 - Not Used
	// Bit7 - 1=Filters enabled on QEA/QEB,0=disabled
	// Bit6-4 - QEA/QEB Filter Clock Divide Bits
	// 111=1:256
	// 110=1:128
	// :::
	// 000=1:1
	// Bit3 -1=Filter enabled on index, 0=disabled
	// Bits2-0 - INDEX Filter Clock Divide ratio as per Bits6-4


	DFLTCON = 0x0000;

	MAXCNT = 0xffff;	//Set maximum count to full scale

	IFS2bits.QEIIF=0; //Clear off any interrupt due to config

	return;
}

	
void setup_timers(void)
{
	// Timer 1 is used for a guard timer to catch missed zero 
	// crossing events that otherwise would cause severe over currents
	// Timer 2 is used to time intervals between commutation events
	// Timer 3 is used to time the on time for the Brake Chopper switch
	// Brake Chopper PWM

	// T1CON
	// Bit 15 - 1=Timer 1 is on,o is off
	// Bit 14 - Not Used
	// Bit 13 - 0=continues in idle,1=discontinues
	// Bits12-7 - 	Not Used
	// Bit6 - 1=timer in gate mode using T1CK
	// Bits5-4 = clk prescalar
	// 11 = 1:256
	// 10 = 1:64
	// 01 = 1:8
	// 00 = 1:1
	// Bit 3 - Not Used
	// Bit 2 - 1= Synch external clock input
	// Bit 1 - 1=clk is rising edge of T1CK,0 = Tcy
	// Bit0 - Not Used
	
	T1CON= 0x8020;	//Timer On, 1:64 Tcy clk
						//Prescalar chsoen to match T2 and QEI
						//so that units are the same
	PR1=0xFFFF;		//Set Period Match Regsiter to full scale
	IFS0bits.T1IF=0; //Clear off any pending interrupt

	// T2CON
	// Bit 15 - 1=Timer 2 is on,0 is off
	// Bit 14 - Not Used
	// Bit 13 - 0=continues in idle,1=discontinues
	// Bits12-7 - 	Not Used
	// Bit6 - 1=timer in gate mode using T2CK
	// Bits5-4 = clk prescalar
	// 11 = 1:256
	// 10 = 1:64
	// 01 = 1:8
	// 00 = 1:1
	// Bit3 1=TIMER2&3 in 32 bit mode, 0= separate 16 bit
	// Bit2 - Not Used
	// Bit1 - 1=clk is rising edge of T2CK,0 = Tcy
	// Bit0 - Not Used

	T2CON=0x8020;	//Timer On, 1:64 Tcy clk, 16 bit mode
						//1:64 division chosen to match QEI config
						//which is being used as "input capture"
	PR2=0xFFFF;		//Set Period Match Regsiter to full scale
	IFS0bits.T2IF=0; //Clear off any pending interrupt

	// T3CON
	// Bit 15 - 1=Timer 3 is on,0 is off
	// Bit 14 - Not Used
	// Bit 13 - 0=continues in idle,1=discontinues
	// Bits12-7 - 	Not Used
	// Bit6 - 1=timer in gate mode using T3CK
	// Bits5-4 = clk prescalar
	// 11 = 1:256
	// 10 = 1:64
	// 01 = 1:8
	// 00 = 1:1
	// Bit3 - Not Used
	// Bit2 - Not Used
	// Bit1 - 1=clk is rising edge of T3CK,0 = Tcy
	// Bit0 - Not Used

	T3CON=0x8020; //Timer On, 1:64 Tcy clk
	PR3=0xFFFF;
	IFS0bits.T3IF=0; //Clear off any pending interrupt
	return;
}
unsigned int ReadConfig(int address)
{
	// Set Table Page to point to config page
	asm(" mov.w	#0x00F8,w2");
	asm(" mov.w w2,TBLPAG");
	// Now read the values
	asm(" tblrdl [w0],w0");
	// Note that strictly speaking the return WREG0
	// is uneccesary as the correct result is already in W0.
	// It does keep the compiler from generating a warning.
	return WREG0;
}
void WriteConfig(int address, int value) 
{ 
	// Set Table Page to point to config page
	asm(" mov.w	#0x00F8,w2");
	asm(" mov.w w2,TBLPAG");
	// Set NVMCON with correct value for program of
	// config location
	asm(" mov.w	#0x4008,w2");
	asm(" mov.w w2,NVMCON");
	// Address will be in w0 and value in w1
	// load these into the programming registers  
	asm(" tblwtl	W1,[W0]");
	asm(" bset		NVMCON,#14"); 
	asm(" mov		#0x55,W2 ");
	asm(" mov		W2,NVMKEY ");
	asm(" mov		#0xaa,W2 ");
	asm(" mov		W2,NVMKEY ");
	asm(" bset		NVMCON,#15"); 
	 

	while(NVMCONbits.WR); 
}


#ifdef DEVELOPMODE
void setup_uart(void)
{

 U1MODE = 0x8000;
 U1STA = 0x0000;
 U1BRG = ((CLOCK_FREQ/16)/BAUD) - 1;	// set baudrate to BAUD rate
 IEC0bits.U1RXIE = 1;
 RXPtr = &InData[0];		// point to first char in receive string
 control_flags2.CheckRX = 0;
 control_flags2.SendTX = 0;
 U1STAbits.UTXEN = 1;           // Initiate transmission
} 
#endif
