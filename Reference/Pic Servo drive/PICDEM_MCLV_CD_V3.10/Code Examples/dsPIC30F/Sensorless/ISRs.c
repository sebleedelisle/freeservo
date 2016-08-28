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
 *    Filename:       ISRs.c          	                                *
 *    Date:           4/21/04                                         *
 *    File Version:   4.00                                             *
 *    Project:        53                                               *
 *    Drawing:        2                                                *
 *                                                                     *
 *    Tools used:    MPLAB C30 Compiler v 1.20.01                      *
 *                                                                     *
 *    Linker File:    p30f4012.gld                                   *
 *                                                                     *
 *                                                                     *
 ***********************************************************************
 *	Code Description
 *  
 *  This file contains all the interrupt service routines (ISRs)
 *
 **********************************************************************/

#include "general.h"
#include "hardware.h"
#include "defs.h"
#include "extern_globals.h"

void check_zero_crossing(void);
static void current_control(void);
static void acquire_position(void);

void __attribute__((__interrupt__)) _DefaultInterrupt(void)
{
// This is the default interrupt handler.  If an interrupt
// vector is not defined, then we will go here and wait
// in an infinite loop!
DISABLE_FIRING;
while(1);	
}

void __attribute__((__interrupt__)) _AddressError(void)
{
	DISABLE_FIRING;
	
	// Clear WDT and drive LED4 at 100% Duty
	while(1)
	{
		ClrWdt();
	}
	return;
}

void __attribute__((__interrupt__)) _StackError(void)
{
	unsigned char i;

	DISABLE_FIRING;
	
	// Clear WDT and drive LED4 at 50% Duty
	while(1)
	{
		ClrWdt();
	}
	return;
}

void __attribute__((__interrupt__)) _MathError(void)
{
	unsigned char i;

	DISABLE_FIRING;

	// Clear WDT and drive LED4 at 33% Duty
	while(1)
	{
		ClrWdt();

	}
	return;
}

void __attribute__((__interrupt__)) _PWMInterrupt(void)
{
	slow_event_count++;
	medium_speed_event_count++;
	IFS2bits.PWMIF = 0;
	return;
}

void __attribute__((__interrupt__)) _FLTAInterrupt (void)
{
	if (run_state == STARTING)
		{IFS2bits.FLTAIF = 0; return;}
	if (fault_count++ < MAXFAULT)
		{IFS2bits.FLTAIF = 0; return;}
	DISABLE_FIRING;
	run_state=FAULT;
	if (!trip_state)	trip_state=HARDWARE_TRIP;
	IFS2bits.FLTAIF = 0;
	return;
}

void __attribute__((__interrupt__)) _ADCInterrupt (void)
{
	// Clear off interrupt flag
	IFS0bits.ADIF = 0;

	
	// Get Results from ADC Buffer and put them into relevant variables
	// When in normal running we get one phase voltage, dc bus voltage
	// dc bus current and VR2 as simultaneous samples every pwm
	// When acquiring we need to see all three phase voltages so 
	// we continuously cycle ADC CH0 between the three phase voltages
	if (control_flags.ACQUIRE2==TRUE)
	{
		switch (ADCHS)
		{
			case VPH_RED :		vph_red=ADCBUF0;
									ADCHS=VPH_YELLOW;
									break;

			case VPH_YELLOW : vph_yellow=ADCBUF0;
									ADCHS=VPH_BLUE;
									break;

			case VPH_BLUE	 : vph_blue=ADCBUF0;
									ADCHS=VPH_RED;
									break;

			default :			ADCHS=VPH_RED;
									break;
		}
	}
	else
	{
		vph=ADCBUF0;
	}
	vdc=ADCBUF1;  
	ibus=ADCBUF2;
	pot=ADCBUF3;
	
	// If a fault condition exists ensure run_state is set to FAULT
	// and return at this point.
	if (trip_state)	
	{
		run_state=FAULT;
		return;
	}


	// Do over voltage and over current trip checks if not already in fault
	if ((ibus>current_trip) && (run_state != INITIALIZING))
	{
		DISABLE_FIRING;
		run_state=FAULT;
		trip_state=OVER_CURRENT;
		return;
	}

	if ((vdc>voltage_trip) && (run_state != INITIALIZING))
	{
		DISABLE_FIRING;
		run_state=FAULT;
		trip_state=OVER_VOLTAGE;
		return;
	}
	
	// Now do current control with most up to date current sample
	// If current control is not active routine initialises some
	// variables and returns
	current_control();
	
	// Now reconfigure adc if required to sample different 
	// phase voltage feedback channel

	if (control_flags.ADCCONFIG==TRUE)
	{
		ADCHS = adc_channel_config;
	}

	if ((control_flags.ACQUIRE2==TRUE) && (run_state!=INITIALIZING))
	{
		acquire_position();
	}

	if ((control_flags.SENSORLESS) || (control_flags.ACQUIRE1))
	{
		check_zero_crossing();
	}
	return;
}

#ifdef DEVELOPMODE
//UART interrupts
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
		{control_flags2.CheckRX = 1;RXPtr = &InData[0];}
	else *RXPtr++;
}

#endif
  
// Timer 1 is used as a guard timer to catch missed zero
// crossing events due to excessive demag time/insufficient
// phase advance for instance. Every zero crossing detected,
// PR1 is written with double the latest time delta between
// zero crossings and TIMER1 is reset to zero. If the timer
// ever reaches PR1 causing an interrupt, then the system is
// assumed to have missed a zero crossing and enters the LOST
// trip state.

void __attribute__((__interrupt__)) _T1Interrupt(void)
{
	DISABLE_FIRING;
	run_state=FAULT;
	if (!trip_state)	trip_state=LOST;
	IEC0bits.T1IE = 0;	// Disable interrupt so don't get
								// unnecessary calls
	IFS0bits.T1IF = 0;	// Clear off interrupt flag
	return;
}

// Timer 2 used as an output compare but not associated with
// an I/O line. It indicates time for a commutation.
// The ISR therefore actually does the commutation and also
// reconfigures voltage sensing channel and zero_crossing

void __attribute__((__interrupt__)) _T2Interrupt(void)
{

	// If we are acquiring using method1
	if (control_flags.ACQUIRE1)
	{
		// Set ADCCONFIG flag so that zero_crossing routine is correctly 
		// initialized next call.
		control_flags.ADCCONFIG=TRUE;
		// If we did not detect a valid zero crossing in the last
		// sector then need to initialize acquire and check counters
		// and also update the sector as this has not been done
		// by zero_crossing routine.
		if (control_flags.ZERO_CROSS==FALSE)
		{
			acquire_counter=0;
			check_counter=8;
			if (control_flags.DIR==FORWARDS)
			{
				if (sector==5) sector=0;
				else 				sector++;
			}
			else
			{
				if (sector==0) sector=5;
				else 				sector--;
			}
		}
	}

	// If this is first interrupt after ACQUIRE1 mode to be used
	if (control_flags2.ACQUIRE1_REQUEST)
	{
		control_flags2.ACQUIRE1_REQUEST=FALSE;
		control_flags.ACQUIRE1=TRUE;
		// Set ADCCONFIG flag so that zero_crossing routine is correctly 
		// initialized next call.
		control_flags.ADCCONFIG=TRUE;
		check_counter=8;	//This is 8 so that 1 elec cycle (6)
								//of tolerance checks after transition
								//to sensorless operation are ignored.
		acquire_counter=0;
		#ifdef DEBUG
			asm("bclr        LATD,#15");
			asm("bclr		  LATG,#3");
			asm("bclr		  LATG,#1");
			indx=0;
		#endif
	}
	// If swapping from acqisision mode 2 to sensorless
	// set sensorless flag and load check_counter
	// so that sensorless tolerance checks are disabled
	// for first electrical cycle as first few zero crossings
	// may violate tolerance checks due to change over transient
	if (control_flags.SWAP==TRUE)
	{
		control_flags.SENSORLESS=TRUE;
		control_flags.SWAP=FALSE;
		check_counter=6;
	}

	// Decrement the check_counter if > 0
	if (check_counter)	check_counter--;

	// If not in normal running then need to reload PR2 with
	// a new value calculated elsewhere.
	// When running sensorless the detection of a zero crossing
	// reloads PR2 and enables interrupt
	if (control_flags.SENSORLESS==FALSE)
	{
		// Load New Value Into PR2
		PR2=new_PR2;
		// Clear off TMR2 to ensure don't have issue with occasional
		// miss of TMR2=PR2 event when PR2 is written to.
		TMR2=0;
	}

	#ifdef DEBUG
	// These diagnostic signals re-create the Halls
		switch (sector)
		{
			case 0: 	asm(" bclr	LATD,#14");
						break;

			case 1: 	asm(" bset	LATD,#12");
						break;

			case 2: 	asm(" bclr	LATD,#6");
						break;

			case 3: 	asm(" bset	LATD,#14");
						break;

			case 4: 	asm(" bclr	LATD,#12");
						break;

			case 5: 	asm(" bset	LATD,#6");
						break;

			default : sector=0;
		}
	#endif
	
	// This is the main switch statement that actually
	// does the commutation. It also sets up which is the
	// next vph channel to look at for zero crossing. This
	// is used to configure CH0 S+H input channel.
	switch (sector)
	{
		case 0: 	OVDCON=SECTOR0_OVERRIDE;
					adc_channel_config=VPH_BLUE;
					break;

		case 1: 	OVDCON=SECTOR1_OVERRIDE;
					adc_channel_config=VPH_YELLOW;
					break;

		case 2: 	OVDCON=SECTOR2_OVERRIDE;
					adc_channel_config=VPH_RED;
					break;

		case 3: 	OVDCON=SECTOR3_OVERRIDE;
					adc_channel_config=VPH_BLUE;
					break;

		case 4: 	OVDCON=SECTOR4_OVERRIDE;
					adc_channel_config=VPH_YELLOW;
					break;

		case 5: 	OVDCON=SECTOR5_OVERRIDE;
					adc_channel_config=VPH_RED;
					break;
	}
	// When running sensorless, the zero_crossing
	// routine actually moves on the sector number
	// in response to a zero crossing. Otherwise
	// need to increment sector in the correct direction
	// to ensure that the next time the routine is called
	// the system commutates.
	
	if (control_flags.SENSORLESS==FALSE)
	{
		if (control_flags.ACQUIRE1==FALSE)
		{
			if (control_flags.DIR==FORWARDS)
			{
				if (sector==5) sector=0;
				else 				sector++;
			}
			else
			{
				if (sector==0) sector=5;
				else 				sector--;
			}
		}
	}

	// When running sensorless need to reconfigure ADC to 
	// look at correct vph channel and initialize several 
	// variables in zero_crossing routine. 
	// This is triggered by ADCCONFIG flag being set.
	// Also need to disable T2 interrupt as zero crossing
	// routine to load up next commutation instant.

	else
	{
		control_flags.ADCCONFIG=TRUE;
		IEC0bits.T2IE = 0;
	}
	IFS0bits.T2IF = 0;
	return;
}

// Timer 3 used as to provide simple PWM of
// brake switch while OC module not functioning
// It is used to time the On time and turns 
// brake switch off when interrupt occurs

void __attribute__((__interrupt__)) _T3Interrupt(void)
{
	IFS0bits.T3IF = 0;
	IEC0bits.T3IE = 0;
	return;
}


// check_zero_crossing is used to detect and act upon phase voltage
// zero crossings (when BEMF crosses vdc/2)
// It is called during normal running and when starting using 
// acqusition method1.

void check_zero_crossing(void)
{
	static unsigned int previous_sample;
	static unsigned char level_counter;
	static unsigned char blanking_counter;

	static unsigned int half_vdc=0;
	
	unsigned int new_timestamp;
	
	// If a commutation just occured which reconfigured
	// ADC to look at next vph feedack, need to clear
	// previous sample and level counter as well as
	// returning without checking for crossing and clearing
	// the relevant flags.
	// Also load up blanking counter to ignore first 
	// few samples to reject false crossings due to 
	// diode action at the end of the demag period
	// Finally determine whether looking for RISING or FALLING X

	if (control_flags.ADCCONFIG==TRUE)
	{
		control_flags.ADCCONFIG=FALSE;
		previous_sample=0;
		level_counter=0;
		control_flags.LEVEL=FALSE;
		control_flags.ZERO_CROSS=FALSE;
		blanking_counter=(unsigned char)user_parameters[32];
		// Decide whether rising or falling edge detect
		if (control_flags.DIR==FORWARDS)
		{
			if ((sector % 2) == 0)	control_flags2.FALLING_EDGE=TRUE;
			else							control_flags2.FALLING_EDGE=FALSE;
		}
		else
		{
			if ((sector % 2) == 0)	control_flags2.FALLING_EDGE=FALSE;
			else							control_flags2.FALLING_EDGE=TRUE;
		}
		return;
	}
	
	// Blanking counter used to ignore samples just after
	// commutation so that diode action doesn't cause false
	// zero_crossing.

	if (blanking_counter > 0)
	{
		blanking_counter--;
		return;
	}
	// This locks out the possibility of a second zero crossing 
	// being detected in a particular sector.
	if (control_flags.ZERO_CROSS==TRUE)
		return;

	// Half_vdc is a simple filtered version of vdc/2
	half_vdc = ((half_vdc + (vdc>>1)) >> 1);

	// To add robustness to the zero crossing detection a programmable
	// number of samples of vph must be seen above (falling edge) 
	// or below (rising edge) before the actual crossing detection is enabled
	// The level_counter variable along with the LEVEL flag implement this

	if (level_counter==(unsigned char)user_parameters[33])
	{
		control_flags.LEVEL=TRUE;
	}

	if (control_flags2.FALLING_EDGE)
	{ 
		if ((level_counter < (unsigned char)user_parameters[33]) && (vph > half_vdc))
		{
			level_counter++;
		}
		if ((level_counter > 0) && (vph <= half_vdc))
		{
			level_counter--;
		}
	}
	else
	{
		if ((level_counter < (unsigned char)user_parameters[33]) && (vph < half_vdc))
		{
			level_counter++;
		}
		if ((level_counter > 0) && (vph >= half_vdc))
		{
			level_counter--;
		}
	}

	// Note that saved timestamp and timedelta data is stored in arrays with three
	// elements corresponding to the three phases. The conventions used are as follows:
	// Index 0 = Red
	// Index 1 = Yellow
	// Index 2 = Blue
	// Sector 0,3 = Blue Phase Crossing
	// Sector 1,4 = Yellow Phase Crossing
	// Sector 2,5 = Red Phase Crossing 
	if (control_flags.LEVEL==TRUE) // Only check for the crossings once LEVEL is true
	{
		// If we have seen sufficient valid samples above or below vdc/2 
		// and the current and previous samples are <= vdc/2 or >= vdc/2 (edge dependant)
		// then this is the valid crossing.
		if (((control_flags2.FALLING_EDGE) && (previous_sample <= half_vdc)\
				&& (vph <= half_vdc)) || \
			((control_flags2.FALLING_EDGE==FALSE) && (previous_sample >= half_vdc)\
				&& (vph >= half_vdc)))
		{
			// Grab latest value of QEI counter which is
			// being used in the 16 bit timer mode
			new_timestamp=POSCNT;

			// Calculate time between this zero crossing and the last one
			// The result of this calculation is correct irrespective
			// of counter rollover due to the elegance of 2's comp
			// arithmetic on unsigned variables.
			// Note that to ensure we use the correct previous zero crossing
			// timestamp the calculation is direction dependent
			if (control_flags.DIR == FORWARDS)
			{
				switch (sector)
				{
					case 0:
					case 3:	latest_delta=new_timestamp-previous_timestamps[0];
								break;
					case 1:
					case 4:	latest_delta=new_timestamp-previous_timestamps[2];
								break;
					case 2:
					case 5:	latest_delta=new_timestamp-previous_timestamps[1];
								break;
				}
			}
			else // going BACKWARDS
			{
				switch (sector)
				{
					case 0:
					case 3:	latest_delta=new_timestamp-previous_timestamps[1];
								break;
					case 1:
					case 4:	latest_delta=new_timestamp-previous_timestamps[0];
								break;
					case 2:
					case 5:	latest_delta=new_timestamp-previous_timestamps[2];
								break;
				}
			}

			// Calculate period measurement used for speed feedback
			// rather than just using 60 degree measurement use 180
			// degrees to get better resolution at high speeds
			// Only calculate period once per electrical cycle in sector 0
			// Note if you calculate period more often than once per electrical
			// cycle, you will get faster update rate but more ripple.
			if (sector==0)
			{
				period_measurement=new_timestamp-previous_timestamps[2];

				// Reset stall counter to denote a valid period measurement event has ocurred
				stall_counter=0;
			}

			// Calculate the number of counts of TIMER2 until next commutation
			// Note that TIMER2 and QEI are on same timebase
			// The divison by 2 is because the diffence between the zero
			// crossing of a phase and the previous one from another phase
			// is 60 electrical degrees. The natural offset between zero_crossing 
			// and the correct commutation is 30 electrical degress without any
			// phase advance. 
			commutation_time=((latest_delta)/2) - phase_advance;

			// Now check that phase advance didn't cause too small a value for
			// commutation time. This can occur because phase advance is only
			// calculated in the medium speed event routine based on the period
			// measurement for efficiency. The test is done at 1 because this is
			// the smallest value that can be loaded into PR2 for the TIMER2 interrupt
			// to function correctly as we reset TIMER2 to zero after writing to PR2
			if (commutation_time < 1) commutation_time=1;

			// Now check to see if commutation has failed. This is done
			// by comparing latest_delta with previous_delta and applying a
			// a tolerance check. 			
			check_value=(latest_delta*100)/previous_delta;
			
			if (((check_value > upper_tol) \
				|| (check_value < lower_tol))	\
				&& (check_counter==0))
			{
				// If sensorless failed then enter
				// acquistion mode2 again if auto re-acquire (user param 31) is enabled
				DISABLE_FIRING;
				IEC0bits.T2IE=FALSE;
				control_flags.SENSORLESS=FALSE;
				if (user_parameters[31])
				{
					control_flags.ACQUIRE2=TRUE;
					control_flags.ADCCONFIG=TRUE;
					control_flags2.RETRY_FLAG=FALSE;
				}
				else
				{
					run_state=FAULT;
					trip_state=LOST;
				}
				return;
			}
			else		// Valid zero crossing detected
			{
				// Update sector
				if (control_flags.DIR==FORWARDS)
				{
					if (sector==5) sector=0;
					else 				sector++;
				}
				else
				{
					if (sector==0) sector=5;
					else 				sector--;
				}
				
				// Commutate as sensorless working provided not acquiring
				// using method1
				if (control_flags.ACQUIRE1==FALSE)
				{
					// Load up period register
					PR2 = (unsigned int) commutation_time;
					TMR2=0;
					// Now clear off interrupt flag and enable interrupt
					IFS0bits.T2IF = 0;
					IEC0bits.T2IE = 1;
					
					// Load up guard timer implemented in Timer 1 with
					// double the latest time delta between zero crossings
					PR1=latest_delta<<1;
					TMR1=0;
					
					// Clear interrupt flag just in case writing to timer
					// generated an interrupt.
					IFS0bits.T1IF = 0;
					// Now enable interrupt.
					IEC0bits.T1IE = 1;
				}
				else
				{
					// Incrementing acquire counter indicates a valid zero x
					// event has just occurred.
					// acquire counter is reset to zero by code in T2ISR if
					// a zero X event is missed during acquistion
					acquire_counter++;
					#ifdef DEBUG
						asm("btg        LATD,#15");
					#endif
					if (acquire_counter > 1) 
					{	// i.e. if have had two consequtive zero X events
						// Disable T2 interrupts ready for transition to 
						// closed loop commutation
						IEC0bits.T2IE = 0;

						// Set flags to indicate now running sensorless
						control_flags.ACQUIRE1=FALSE;
						control_flags.SENSORLESS=TRUE;
						run_state=RUNNING;
						// load up check counter to disable zeor X tolerance
						// checks for 1 elec cycle
						check_counter=6;
						// set up T2 to generate interrupt at correct time
						PR2 = (unsigned int) commutation_time;
						TMR2=0;
						// Now clear off interrupt flag and enable interrupt
						IFS0bits.T2IF = 0;
						IEC0bits.T2IE = 1;
						#ifdef DEBUG
							asm("btg        LATG,#1");
						#endif
					}
				}
			}
			// End of else associated with a valid zero crossing
			// Note that if you fail tolerance check code returns before
			// this point.

			// Save off timestamp info ready for next zero crossing

			if (control_flags.DIR==FORWARDS)
			{
				switch (sector)
				{
					case 0:	
					case 3:	previous_timestamps[0] = new_timestamp;
								break;
					case 1:	
					case 4:	previous_timestamps[2] = new_timestamp;
								break;
					case 2:	
					case 5:	previous_timestamps[1] = new_timestamp;
								break;
				}
			}
			else
			{
				switch (sector)
				{
					case 0:	
					case 3:	previous_timestamps[1] = new_timestamp;
								break;
					case 1:	
					case 4:	previous_timestamps[0] = new_timestamp;
								break;
					case 2:	
					case 5:	previous_timestamps[2] = new_timestamp;
								break;
				}	
			}
			// Save the latest 60 degree time difference for next
			// zero crossing tolerance check
			previous_delta=latest_delta;
			// Set flag to indicate a valid zero Xing occurred
			control_flags.ZERO_CROSS=TRUE;
			level_counter=0;
			control_flags.LEVEL=FALSE;

		}	// End of the if statement detecting zero X
	}	// End of the if (control_flags.LEVEL=TRUE)
	// Save off current phase voltage sample ready for next call
	
	previous_sample=vph;

	return;
}

// This function implements current control using the bus current
// It is intended to be called after the latest ADC reading of ibus
// is availble.
// As the control effort calculated is directly loaded into the
// the PWM duty cycle register, 0 effort is equal to 50% duty cycle
// 50% duty cycle gives 0 average phase voltage applied if current
// is continuous.
static void current_control(void)
{

	static long current_integral;
	static int previous_current_error;
	int current_error;
	long proportional_term;
	long derivative_term;
	int control_output;
	long temp;

	// Check to see whether current control loop should be active
	// Whenever switches are turned off or using voltage control
	// the current control should be inactive
	// Strictly speaking should check for faults and initializing
	// and exit, but will always enter the standby before can run
	// or start so there is no need
	// If current control not active clear off integral term and
	// also disable D term for first pass by setting flag

	if ((run_state == STANDBY) || (control_flags.ACQUIRE2==TRUE) \
		|| (control_flags2.ROTATION_CHECK==TRUE))
	{
		current_integral=0;
		control_flags2.D_TERM_DISABLE=TRUE;
		return;
	}

	if ((run_state == STARTING) && ((user_parameters[40]==TRUE) \
		&& (control_flags2.WINDMILLING==FALSE)))
	{
		current_integral=0;
		control_flags2.D_TERM_DISABLE=TRUE;
		return;
	}

	if ((run_state == RUNNING) && ((user_parameters[1]==OPEN_VOLTS) \
		|| (user_parameters[1]==CLOSED_VOLTS)))
	{
		current_integral=0;
		control_flags2.D_TERM_DISABLE=TRUE;
		return;
	}

	// Form the current error by subtracting the ibus from the current_demand
	// Also account for the fact that zero ibus is biased by 
	// approx 2.5V so that can measure bipolar currents
	// ibus_offset is the ADC value of the ibus signal for zero current
	current_error=(current_demand+ibus_offset)-ibus;

	// Start off by assuming not in voltage limit
	control_flags.VOLTS_LIMIT=FALSE;

	// All calculated quantities will be shifted back down at the 
	// end by 512. PID gains are effectively scaled up by 512.
	// This is done to allow fractional gains without
	// severe rounding issues which causes quantization noise etc.
	
	proportional_term=((long)current_error*(long)iloop_p_gain);

	derivative_term=((long)(current_error-previous_current_error)\
							*(long)iloop_d_gain);

	if (control_flags2.D_TERM_DISABLE)
		derivative_term=0;

	// Now calculate P+D terms and put them in temp
	temp=proportional_term+derivative_term;

	if (temp > pos_duty_limit)
	{
		control_flags.VOLTS_LIMIT=TRUE;
		control_output=(pos_duty_limit/512)+ZERO_DUTY;
		// If in limit due to just P term then clear
		// off I term so that when come out of limit
		// I term can smoothly take over to eliminate 
		// steady state error.
		current_integral=0;
	}
	else
	{
		if (temp < neg_duty_limit)
		{
			control_flags.VOLTS_LIMIT=TRUE;
			control_output=(neg_duty_limit/512)+ZERO_DUTY;
			// If in limit due to just P term then clear
			// off I term so that when come out of limit
			// I term can smoothly take over to eliminate 
			// steady state error.
			current_integral=0;
		}
	}
	// Now do integral term if not in torque limit already due to P+D term
	if (control_flags.VOLTS_LIMIT==FALSE)
	{
		// Update Integral term here but this will be overwritten if
		// found to be in limit later on in code
		current_integral+=((long)current_error * (long)iloop_i_gain);
		// Add I term to P+D terms
		temp+=current_integral;

		// If sum of PID terms pushes you into positive limit
		if (temp > pos_duty_limit)
		{
			control_flags.VOLTS_LIMIT=TRUE;
			control_output=(pos_duty_limit/512)+ZERO_DUTY;
			// Now calculate integrator limit and clamp
			current_integral = pos_duty_limit - proportional_term - derivative_term;
		}
		else
		{
			// If sum of P+I terms pushes you into negative limit
			if (temp < neg_duty_limit)
			{
				control_flags.VOLTS_LIMIT=TRUE;
				control_output=(neg_duty_limit/512)+ZERO_DUTY;
				// Now calculate integrator limit and clamp
				current_integral=neg_duty_limit - proportional_term - derivative_term;
			}
			else
			// Calculate control output based on unclamped PID terms
			{
				// Temp still contains unclamped P+I term so no need to recalculate
				control_output=(temp/512)+ZERO_DUTY;
			}
		}
	}
	// Now do final check to ensure control_output is not < 0
	// as this would cause large positive value to be loaded
	// into duty cycle registers!
	if (control_output < 0) control_output=0;

	// Now load calculated value into duty cycle registers
	// No need to disable writes to duty regsiters here
	// as we are not going to allow this code to be interrupted
	// (except by a fault) and assuming next pwm cycle starts after
	// registers are written.
	// Compensate calculated output due to fact that firing is inverted
	control_output=FULL_DUTY-control_output;		
	PDC1=(unsigned int)control_output;
	PDC2=(unsigned int)control_output;
	PDC3=(unsigned int)control_output;
	// Save current_error for use by D term next pass
	previous_current_error=current_error;
	// Clear D_TERM_DISABLE as this is only used for the first pass after
	// loop becomes active as don't have a valid previous error to base
	// D term calculation on.
	control_flags2.D_TERM_DISABLE=FALSE;
	return;
}

// This function is designed to determine the position of the rotor
// while the motor is turning but there is no current flowing.
// It looks for the sequence and timing between the rising edges of
// all three phase voltages. The sequence determines direction.
// The timing is used to set up the sensorless energisation
// This function will be called during starting and if lost during
// running.
// A rising edge of the 3 vph occur at the start of the following sectors
// Red = sector 5
// Yellow = sector 1
// Blue = sector 3	

static void acquire_position(void)
{

	static unsigned int previous_vph_red;
	static unsigned int previous_vph_yellow;
	static unsigned int previous_vph_blue;

	static unsigned char level_count_red;
	static unsigned char level_count_yellow;
	static unsigned char level_count_blue;

	static unsigned int previous_edge_time;
	static unsigned char retry_counter;
	unsigned int new_edge_time;
	unsigned int one_twenty_deg;
	unsigned int thirty_deg;
	

	if (control_flags.ADCCONFIG==TRUE)
	{
		control_flags.ADCCONFIG=FALSE;
		control_flags.ACQUIRE2_RED=FALSE;
		control_flags.ACQUIRE2_YELLOW=FALSE;
		control_flags.ACQUIRE2_BLUE=FALSE;
		previous_vph_red=0;
		previous_vph_yellow=0;
		previous_vph_blue=0;
		level_count_red=0;
		level_count_yellow=0;
		level_count_blue=0;
		previous_edge_time=0;
		if (control_flags2.RETRY_FLAG==FALSE)
			retry_counter=NO_RETRIES;
		control_flags2.WINDMILLING=FALSE;
		
		#ifdef DEBUG
			LATDbits.LATD15=1;
			indx=0;
		#endif
		return;
	}
	// This capture of data can be useful for solving acquisition
	// problems by inspecting values using ICD or ICE4000
	#ifdef DEBUG
		//data_log[indx]=vph_red;
		//data_log2[indx]=vph_yellow;
		//data_log3[indx]=vph_blue;
		//data_log4[indx]=POSCNT;
		if (indx<255)
			indx++;
	#endif

	// Check for failure to acquire by monitoring retry counter
	// The retry counter is loaded first time acqusition starts
	// Every time same phase is detected as rising before another phase
	// the counter is decremented.
	// If the retry counter has reached zero and no other trip exists
	if ((retry_counter==0) && (trip_state==0))
	{
		run_state=FAULT;
		trip_state=FAILED_TO_START;
		control_flags.ACQUIRE2=FALSE;
		return;
	}
	// Note that all three sections of code below are similar in function but 
	// differ according to the particular sectors, directions etc
	// Red phase code is fully commented, Yellow and Blue are not

	/******************** START OF RED PHASE CODE **********************************/
	
	// To provide robust crossing detection, a similar scheme as used for zero
	// crossing is used except need a separate counter for each phase.
	// The level counter is only incremented if we see VPH below the acquire_threshold
	// and it has not reached the level_threshold. We are looking for a rising edge

	if ((vph_red < vph_red_threshold) \
		&& (level_count_red < (unsigned char)user_parameters[35]))
	{
			level_count_red++;
	}

	// If we have seen sufficient samples below the acquire_threshold
	if (level_count_red == (unsigned char)user_parameters[35])
	{
		// If previous sample and current sample are >= acquire_threshold
		// then we treat this as a valid rising edge
		if ((previous_vph_red >= vph_red_threshold) \
			&& (vph_red >= vph_red_threshold))
		{
			// Grab new timestamp from free-running counter (QEI in this case)
			new_edge_time=POSCNT;

			// If ACQUIRE2_RED is already set this means that have two edges
			// of RED before another phase which is not correct
			// If detected then reinitialize acquision and decrement retry_counter
			if (control_flags.ACQUIRE2_RED)
			{
				#ifdef DEBUG
					LATDbits.LATD15=0;
				#endif
				// The RETRY_FLAG is set so initialization code does not reload
				// the retry_counter
				control_flags2.RETRY_FLAG=TRUE;
				if (retry_counter)
					retry_counter--;
				// Setting this flag will force re-initialization next call
				control_flags.ADCCONFIG=TRUE;
				return;
			}
			// Reset level_counter so that VPH has to fall back beneath
			// acquire_threshold before rising check done again.
			level_count_red=0;

			// Now check to see if a previous phase's rising edge has already
			// detected. The sequence of the rising edges determines direction
			// and therefore also which sector has just been entered.
			if (control_flags.ACQUIRE2_BLUE==TRUE)
			{
				#ifdef DEBUG
					LATDbits.LATD15=0;
				#endif

				// Update the sector that rotor is in
				sector=5;
				// Update OVDCON with appropriate value
				OVDCON=SECTOR5_OVERRIDE;

				// Calculate times for 120 and 30 electrical degrees
				one_twenty_deg=new_edge_time-previous_edge_time;
				thirty_deg=one_twenty_deg/4;
				
				// Calculate period measurement so speed loop has valid data
				// and ensure don't try to re-acquire at too high a speed. 
				// The period measurement is 180 electrical degrees
				period_measurement=one_twenty_deg+(one_twenty_deg/2);
				// Reset stall counter to denote period has been calculated
				stall_counter=0;

				// Check to see if windmilling i.e. motor is turning in opposite
				// direction to demanded direction.
				// If it is, then will open loop energise motor with
				// current control and gradually reduce output frequency until
				// stationary. Will then start using normal routine.
				if (control_flags.DIR==BACKWARDS) // If windmilling
				{
					// Deliberately use braking energisation
					// This is done by using commutation that is 180 electrical
					// degrees offset from actual position.
					// Therefore sector 5 maps to sector 2
					sector=2;
					OVDCON=SECTOR2_OVERRIDE;
					// Set WINDMILLING flag so that starting code enters a different
					// mode of operation which gradually reduces commutation frequency
					// and then locks and ramps in the correct direction
					control_flags2.WINDMILLING=TRUE;
					// Clear ROTATION_CHECK flag as valid rotation is occuring
					control_flags2.ROTATION_CHECK=FALSE;
					// Clear ACQUIRE2 flag as system going to be energised open loop
					control_flags.ACQUIRE2=FALSE;
					// Initial commutation period = 60 degrees
					new_PR2=one_twenty_deg/2;
					// Calculate decel rate based on minimum period and the user parameter
					// which determines time period over which ramp down will occur
					windmilling_decel_rate=(BRAKING_PR2_LIMIT-new_PR2)/user_parameters[42];
					// Invert direction in which energisation will occur
					control_flags.DIR=FORWARDS;
					// Force a Timer 2 interrupt event to start commutation
					IFS0bits.T2IF=TRUE;
					IEC0bits.T2IE=TRUE;
					// Set current demand to the windmilling_demand which is derived from
					// the user parameters for demand and scaling
					current_demand=windmilling_demand;
					ENABLE_FIRING;
					return;
				}

				// To reach this section of code motor not windmilling and have detected a
				// second valid rising edge and so ought to be able to launch sensorless.
				// However, if going too fast, sensorless will get immediately lost due 
				// to enevitable error in threshold detection and because there may
				// not be time for sufficient samples of VPH to catch the zero crossing

				if (period_measurement > REACQUIRE_THRESHOLD) // If OK to run sensorless
				{	
					// To ensure sensorless can pick straight up need to
					// extrapolate backwards in time and set up previous timestamps
					previous_timestamps[0]=new_edge_time - one_twenty_deg - thirty_deg;
					previous_timestamps[1]=new_edge_time - thirty_deg;
					previous_timestamps[2]=new_edge_time - one_twenty_deg + thirty_deg;
					previous_delta=one_twenty_deg/2;
					control_flags.ACQUIRE2=FALSE;
					// Setting the SWAP flag triggers the transition to sensorless
					control_flags.SWAP=TRUE;
					// Set run state to RUNNING in case this was a flying start
					run_state=RUNNING;
					// Force a Timer 2 interrupt event to start commutation
					IFS0bits.T2IF=TRUE;
					IEC0bits.T2IE=TRUE;
					ENABLE_FIRING;
				}
				else // Going too fast to re-acquire
				{
					// Force re-start of acqusition
					control_flags.ADCCONFIG=TRUE;
					// Also set ROTATION_CHECK flag
					// so that rotation timer can't timeout before
					// speed has fallen below where can re-acquire
					control_flags2.ROTATION_CHECK=TRUE;
				}	
				return;
			}

			if (control_flags.ACQUIRE2_YELLOW==TRUE)
			{

				sector=2;
				OVDCON=SECTOR2_OVERRIDE;
				one_twenty_deg=new_edge_time-previous_edge_time;
				thirty_deg=one_twenty_deg/4;
				period_measurement=one_twenty_deg+(one_twenty_deg/2);
				stall_counter=0;
				if (control_flags.DIR==FORWARDS)	// If windmilling
				{
					sector=5;
					OVDCON=SECTOR5_OVERRIDE;
					control_flags2.WINDMILLING=TRUE;
					control_flags2.ROTATION_CHECK=FALSE;
					control_flags.ACQUIRE2=FALSE;
					new_PR2=one_twenty_deg/2;
					windmilling_decel_rate=(BRAKING_PR2_LIMIT-new_PR2)/user_parameters[42];
					control_flags.DIR=BACKWARDS;
					IFS0bits.T2IF=TRUE;
					IEC0bits.T2IE=TRUE;
					current_demand=windmilling_demand;
					ENABLE_FIRING;
					return;
				}

				if (period_measurement > REACQUIRE_THRESHOLD) // If OK to run sensorless
				{	
					previous_timestamps[0]=new_edge_time - one_twenty_deg - thirty_deg;
					previous_timestamps[1]=new_edge_time - one_twenty_deg + thirty_deg;
					previous_timestamps[2]=new_edge_time - thirty_deg;
					previous_delta=one_twenty_deg/2;
					control_flags.ACQUIRE2=FALSE;
					control_flags.SWAP=TRUE;
					run_state=RUNNING;
					IFS0bits.T2IF=TRUE;
					IEC0bits.T2IE=TRUE;
					ENABLE_FIRING;
				}
				else
				{
					control_flags.ADCCONFIG=TRUE;
					control_flags2.ROTATION_CHECK=TRUE;
				}
				return;
			}
			// Save timestamp just taken as it will be required when second rising
			// edge is detected to work out period etc.
			previous_edge_time=new_edge_time;
			// Set flag to indicate a valid rising edge has been detected on this phase
			control_flags.ACQUIRE2_RED=TRUE;
		}	
	}	
	/********************* END OF RED PHASE CODE *****************************************/

	/********************START OF YELLOW PHASE CODE **************************************/		
	if ((vph_yellow < vph_yellow_threshold) \
		&& (level_count_yellow < (unsigned char)user_parameters[35]))
	{
			level_count_yellow++;
	}

	if (level_count_yellow == (unsigned char)user_parameters[35])
	{
		if ((previous_vph_yellow >= vph_yellow_threshold) \
		&& (vph_yellow >= vph_yellow_threshold))
		{
			new_edge_time=POSCNT;
			
			if (control_flags.ACQUIRE2_YELLOW)
			{

				control_flags2.RETRY_FLAG=TRUE;
				if (retry_counter)
					retry_counter--;
				control_flags.ADCCONFIG=TRUE;
				return;
			}
			
			level_count_yellow=0;
			if (control_flags.ACQUIRE2_RED==TRUE)
			{

				sector=1;
				OVDCON=SECTOR1_OVERRIDE;
				one_twenty_deg=new_edge_time-previous_edge_time;
				thirty_deg=one_twenty_deg/4;
				period_measurement=one_twenty_deg+(one_twenty_deg/2);
				stall_counter=0;

				if (control_flags.DIR==BACKWARDS)
				{
					sector=4;
					OVDCON=SECTOR4_OVERRIDE;
					control_flags2.WINDMILLING=TRUE;
					control_flags2.ROTATION_CHECK=FALSE;
					control_flags.ACQUIRE2=FALSE;
					new_PR2=one_twenty_deg/2;
					windmilling_decel_rate=(BRAKING_PR2_LIMIT-new_PR2)/user_parameters[42];
					control_flags.DIR=FORWARDS;
					IFS0bits.T2IF=TRUE;
					IEC0bits.T2IE=TRUE;
					current_demand=windmilling_demand;
					ENABLE_FIRING;
					return;
				}

				if (period_measurement > REACQUIRE_THRESHOLD)
				{
					previous_timestamps[0]=new_edge_time - one_twenty_deg + thirty_deg;
					previous_timestamps[1]=new_edge_time - one_twenty_deg - thirty_deg;
					previous_timestamps[2]=new_edge_time - thirty_deg;
					previous_delta=one_twenty_deg/2;
					control_flags.ACQUIRE2=FALSE;
					control_flags.SWAP=TRUE;
					run_state=RUNNING;
					IFS0bits.T2IF=TRUE;
					IEC0bits.T2IE=TRUE;
					ENABLE_FIRING;
				}
				else
				{
					control_flags.ADCCONFIG=TRUE;
					control_flags2.ROTATION_CHECK=TRUE;
				}
				return;
			}

			if (control_flags.ACQUIRE2_BLUE==TRUE)
			{

				sector=4;
				OVDCON=SECTOR4_OVERRIDE;
				one_twenty_deg=new_edge_time-previous_edge_time;
				thirty_deg=one_twenty_deg/4;
				period_measurement=one_twenty_deg+(one_twenty_deg/2);
				stall_counter=0;

				if (control_flags.DIR==FORWARDS)
				{
					sector=1;
					OVDCON=SECTOR1_OVERRIDE;
					control_flags2.WINDMILLING=TRUE;
					control_flags2.ROTATION_CHECK=FALSE;
					control_flags.ACQUIRE2=FALSE;
					new_PR2=one_twenty_deg/2;
					windmilling_decel_rate=(BRAKING_PR2_LIMIT-new_PR2)/user_parameters[42];
					control_flags.DIR=BACKWARDS;
					IFS0bits.T2IF=TRUE;
					IEC0bits.T2IE=TRUE;
					current_demand=windmilling_demand;
					ENABLE_FIRING;
					return;
				}

				if (period_measurement > REACQUIRE_THRESHOLD)
				{
					previous_timestamps[0]=new_edge_time - thirty_deg;
					previous_timestamps[1]=new_edge_time - one_twenty_deg - thirty_deg;
					previous_timestamps[2]=new_edge_time - one_twenty_deg + thirty_deg;
					previous_delta=one_twenty_deg/2;
					control_flags.ACQUIRE2=FALSE;
					control_flags.SWAP=TRUE;
					run_state=RUNNING;
					IFS0bits.T2IF=TRUE;
					IEC0bits.T2IE=TRUE;
					ENABLE_FIRING;
				}
				else
				{
					control_flags.ADCCONFIG=TRUE;
					control_flags2.ROTATION_CHECK=TRUE;
				}
				return;
			}
			previous_edge_time=new_edge_time;
			control_flags.ACQUIRE2_YELLOW=TRUE;			
		}	
	}
	/********************* END OF YELLOW PHASE CODE **********************************/	
	
	/**********************START OF BLUE PHASE CODE **********************************/

	if ((vph_blue < vph_blue_threshold) \
		&& (level_count_blue < (unsigned char)user_parameters[35]))
	{
			level_count_blue++;
	}

	if (level_count_blue == (unsigned char)user_parameters[35])
	{
		if ((previous_vph_blue >= vph_blue_threshold) \
			&& (vph_blue >= vph_blue_threshold))
		{
			new_edge_time=POSCNT;
			
			if (control_flags.ACQUIRE2_BLUE)
			{

				control_flags2.RETRY_FLAG=TRUE;
				if (retry_counter)
					retry_counter--;
				control_flags.ADCCONFIG=TRUE;
				return;
			}

			level_count_blue=0;
			if (control_flags.ACQUIRE2_YELLOW==TRUE)
			{

				
				sector=3;
				OVDCON=SECTOR3_OVERRIDE;
				one_twenty_deg=new_edge_time-previous_edge_time;
				thirty_deg=one_twenty_deg/4;
				period_measurement=one_twenty_deg+(one_twenty_deg/2);
				stall_counter=0;

				if (control_flags.DIR==BACKWARDS)
				{
					sector=0;
					OVDCON=SECTOR0_OVERRIDE;
					control_flags2.WINDMILLING=TRUE;
					control_flags2.ROTATION_CHECK=FALSE;
					control_flags.ACQUIRE2=FALSE;
					new_PR2=one_twenty_deg/2;
					windmilling_decel_rate=(BRAKING_PR2_LIMIT-new_PR2)/user_parameters[42];
					control_flags.DIR=FORWARDS;
					IFS0bits.T2IF=TRUE;
					IEC0bits.T2IE=TRUE;
					current_demand=windmilling_demand;
					ENABLE_FIRING;
					return;
				}

				if (period_measurement > REACQUIRE_THRESHOLD)
				{
					previous_timestamps[0]=new_edge_time - thirty_deg;
					previous_timestamps[1]=new_edge_time - one_twenty_deg + thirty_deg;
					previous_timestamps[2]=new_edge_time - one_twenty_deg - thirty_deg;
					previous_delta=one_twenty_deg/2;
					control_flags.ACQUIRE2=FALSE;
					control_flags.SWAP=TRUE;
					run_state=RUNNING;		
					IFS0bits.T2IF=TRUE;
					IEC0bits.T2IE=TRUE;
					ENABLE_FIRING;
				}
				else
				{
					control_flags.ADCCONFIG=TRUE;
					control_flags2.ROTATION_CHECK=TRUE;
				}
				return;
			}

			if (control_flags.ACQUIRE2_RED==TRUE)
			{

				sector=0;
				OVDCON=SECTOR0_OVERRIDE;
				one_twenty_deg=new_edge_time-previous_edge_time;
				thirty_deg=one_twenty_deg/4;
				period_measurement=one_twenty_deg+(one_twenty_deg/2);
				stall_counter=0;

				if (control_flags.DIR==FORWARDS)
				{
					sector=3;
					OVDCON=SECTOR3_OVERRIDE;
					control_flags2.WINDMILLING=TRUE;
					control_flags2.ROTATION_CHECK=FALSE;
					control_flags.ACQUIRE2=FALSE;
					new_PR2=one_twenty_deg/2;
					windmilling_decel_rate=(BRAKING_PR2_LIMIT-new_PR2)/user_parameters[42];
					control_flags.DIR=BACKWARDS;
					IFS0bits.T2IF=TRUE;
					IEC0bits.T2IE=TRUE;
					current_demand=windmilling_demand;
					ENABLE_FIRING;
					return;
				}

				if (period_measurement > REACQUIRE_THRESHOLD)
				{
					previous_timestamps[0]=new_edge_time - one_twenty_deg + thirty_deg;
					previous_timestamps[1]=new_edge_time - thirty_deg;
					previous_timestamps[2]=new_edge_time - one_twenty_deg - thirty_deg;
					previous_delta=one_twenty_deg/2;
					control_flags.ACQUIRE2=FALSE;
					control_flags.SWAP=TRUE;
					run_state=RUNNING;
					IFS0bits.T2IF=TRUE;
					IEC0bits.T2IE=TRUE;
					ENABLE_FIRING;
				}
				else
				{
					control_flags.ADCCONFIG=TRUE;
					control_flags2.ROTATION_CHECK=TRUE;
				}
				return;
			}
			previous_edge_time=new_edge_time;
			control_flags.ACQUIRE2_BLUE=TRUE;
		}	
	}
	/********************** END OF BLUE PHASE CODE *********************************/

	// Save current VPH samples for use next call of routine
	// Note that code above only returns before this point if an action is being taken 
	// on a rising edge detect.

	previous_vph_red=vph_red;
	previous_vph_yellow=vph_yellow;
	previous_vph_blue=vph_blue;

	return;
}
