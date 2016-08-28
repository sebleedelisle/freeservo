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
 *    Filename:       defs.h          	                                *
 *    Date:           12/10/03                                         *
 *    File Version:   3.01                                             *
 *    Project:        53                                               *
 *    Drawing:        2                                                *
 *                                                                     *
 *    Tools used:    MPLAB C30 Compiler v 1.10.02                      *
 *                                                                     *
 *    Linker File:    p30f4012.gld                                   *
 *                                                                     *
 *                                                                     *
 ***********************************************************************
 *	Code Description
 *  
 *  This file contains constant definitions used throughout the code.
 *
 **********************************************************************/

// Definitions of the run_state
#define INITIALIZING 0
#define STANDBY 1
#define STARTING 2
#define RUNNING 3
#define FAULT 4
#define RESETTING 5

// Definitions of the trip_state
#define NO_TRIP 0
#define FAILED_TO_START 1
#define OVER_CURRENT 2
#define OVER_VOLTAGE 3
#define HARDWARE_TRIP 4
#define OVER_SPEED 5
#define LOST 6
#define STALLED 7

#define TRUE 1
#define FALSE 0

// Definitions for use in PWM Override register
// OVDCON. PWM Strategy is that of hard switching
// of diagonal devices without complementary modulation.
// For this to work it assumes pins are in independent mode.
// Phase#4 firing signals are held permanently off
// Note that due to deliberate inversion of firing signals
// (see Setup.c setup_motor_pwms for details) that switches
// are made ACTIVE to disable them and INACTIVE to turn them on
#define SECTOR0_OVERRIDE	0x06FF
#define SECTOR1_OVERRIDE	0x12FF
#define SECTOR2_OVERRIDE	0x18FF
#define SECTOR3_OVERRIDE	0x09FF
#define SECTOR4_OVERRIDE	0x21FF
#define SECTOR5_OVERRIDE	0x24FF


#define FORWARDS	1
#define BACKWARDS 0

// Definitions of values to be loaded into ADCHS depending on which
// set of ADC inputs are to be sampled.
// In this case always using MUXA settings with CH1-3 sampling AN0-2
// All -ve inputs at -VREF (AVss) and CH0 changing between AN12,13&14

#define VPH_RED 0x0003
#define VPH_YELLOW 0x0004
#define VPH_BLUE 0x0005

// This value is used to decide whether going too fast to
// reacquire. The units are in 1/64ths of Tcy as a period measurement
// Set at value corresponding to 125 Hz output frequency as this value
// is know no allow correct re-acquistion at 16 KHz PWM
#define REACQUIRE_THRESHOLD 460

// This is Fcy / prescalar used for both TIMER2 and QEI
// It is important that the prescalar is the same for 
// TIMER2 and QEI as the code assumed this to be the case
// In order to ensure best high speed performance the
// counter frequency should always be > PWM Freq

#define COUNTER_RATE 115200UL

// Value of duty cycle which it is assumed gives
// zero applied phase voltage. This is nominally 
// = PTPER but will be affected by opto distortion
// and deadtime (if used)

#define ZERO_DUTY 461

// FULL_DUTY is = 2*PTPER. It is used to calculate duty cycle 
//	correctly given the inversion strategy used and for other scaling
// purposes. It is also used by setup_motor_pwms() for configuration.
// Table of values as follows:
// XTAL(MHz)	PLL	MIPS	PWM(KHz)	Value
//	FRC - 8		--		2		16			250
//	FRC - 8		--		2		20			200
// 7.3728		X4		7.4	16			922
//	7.3728		X4		7.4	20			737
// 7.3728		X8		14.8	16			1843
//	7.3728		X8		14.8	20			1475
//	7.3728		X16	29.5	16			3686
//	7.3728		X16	29.5	20			2949
#define	FULL_DUTY 922UL

// This is the ratio of PTPER to the prescalar used for TIMER2
// And is required to compensate for zero crosssing detection
// always being at least 1 PWM cycle late
// The compensation factor is required in time units of TIMER2
#define TIME_CORRECTION 7

// These two event rates are used to determine when the two
// event handlers are called. As the software counters used
// by the handlers are driven by the PWM ISR routine, 
// these values must be changed if PWM frequency is changed.
// The nominal rates are 10ms for the medium event and
// 100ms for the slow event.

#define MED_EVENT_RATE	159	//199 for 20 kHz
#define SLOW_EVENT_RATE	1599	//1999 for 20 kHz

// Values used to define operational mode of speed control
#define CLOSED_VOLTS 0
#define CLOSED_CURRENT 1
#define OPEN_VOLTS 2
#define OPEN_CURRENT 3

// This value is used by the dc bus voltage control loop
// to define the maximum on time for the brake chopper sw
#define POS_V_LIMIT	589824

// Number of passes of slow event when rate
// at which parameters change moves to the
// medium rate and fast rate as defined in the
// parameter_data structure
// e.g. if slow event rate = 0.1Hz then 50 = 5s
// Note these values should be < 255
#define MED_RATE_T	50
#define FAST_RATE_T	100

// Gain of the 10 bit adc assuming 4.96V Avdd
// i.e. 1023/4.96
#define ADC_GAIN	206

// Number of user parameters
#define NO_PARAMETERS	45

// Number of times acquision is retried before
// system goes to FAILED_TO_START trip
#define NO_RETRIES	4

// Value of PR2 which will correspond to the bottom
// speed of the decel ramp. 1920==1Hz Electrical
#define BRAKING_PR2_LIMIT	19200

// If DEBUG is defined it results in the generation
// of multiple test signals and data logging inside 
// acquire_position due to conditional compile 
// statements in various parts of the code.
//#define DEBUG

// Test signals are as follows:
// Note LK6 should be open as RG2 is driven
// LED1 is high when in ADC ISR
// LED2 is high when in medium event code
// LED3 is high when in T2 ISR
// RD6 is R Phase Synthesised Hall
// RD12 is Y Phase Synthesised Hall
// RD14 is B Phase Synthesised Hall
// RD15 Mode2 acqusition - falls when acquires
// RD15 Mode1 acquistion - cleared at start of acquistion
//								 - toggles every valid zero X
// RG1 used in Mode1 acqusition - rising edge system acquired
// RG2 is high when in speed loop code
// RG3 Mode 1 acquistion - toggles every zero X missed

// The assembly macros are used throughout the code
// and so are declared here.
// NOTE THEY MUST BE USED AS A PAIR OR STACK CORRUPTION
// WILL RESULT DUE TO PUSH AND POP OF SR REGISTER

// To disable interrupts:
// First push current value of SR (which contains IPL) onto the stack
// Then set IPL to be above that which all interrupts (except FLTA)
// can execute to lock them out. FLTA has priority 7, all other
// have the default of 4. Thus setting SR Bits 7 & 6 will set IPL to 6
// Note that should not use operations that affect SR bits to write
// to SR. This includes logical operations. Easiest thing to do is 
// use three bit set instruction as otherwise will need to stack
#define DISABLE_INTERRUPTS 	asm("push SR \n bset SR,#7 \n bset SR,#6 \n	bclr	SR,#5")

// To enable interrupts simply pop stack back into SR

#define ENABLE_INTERRUPTS	asm(" pop SR")

#define MAXFAULT 200

#define BAUD 19200
#define MAXINDEX 45

#define CR 0x0D
#define LF 0x0A 
#define HT 0x09
#define SPACE 0x20
#define MAXINDEX 45
#define CLOSED_VOLTS 0
#define MOTORPARA 0
#define BOARDPARA 1
#define STARTINGPARA 2
#define CONTROLPARA 3
#define LIMITPARA 4
#define MAXHELPMSG 5

#define DEVELOPMODE TRUE
