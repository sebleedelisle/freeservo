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
 *    Filename:       medium_event.c   	                             *
 *    Date:           4/21/04                                        *
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
 *  This file has the medium event handler and its associated  functions,
 *  which execute every 10 msec.  In particular, the voltage and speed
 *  control loops are executed here.
 *  
 **********************************************************************/


#include "general.h"
#include "hardware.h"
#include "defs.h"
#include "extern_globals.h"




void medium_event_handler(void);
void speed_loop(void);
void voltage_control(void);
void starting_code(void); 

// debounce_switches is within user_interface.c
extern void debounce_switches(void);

void medium_event_handler(void)
{
	long	ltemp;
	int		itemp;
	// MED_EVENT_RATE set to 10ms in defs.h
	if (medium_speed_event_count > MED_EVENT_RATE)
	{

		medium_speed_event_count=0;
		debounce_switches();
			
		// Check to see if in the lockout period after
		// transitioning from acquistion to sensorless. If in the
		// lockout period (check_counter>0) then assume speed is
		// the same as end of ramp. Without this it is possible to
		// get spurious overspeed trips or even overcurrent
		// trips if speed loop active due to initial oscillation
		// giving unreliable speed calculation
		if (check_counter)
			rpm=user_parameters[7];
		else
			// Calculate speed in RPM for display and control purposes
			// Period used for speed measurement is 180 electrical degrees
			// There are therefore 1*poles counter units per rev
			// Thus to get to Revs per sec do 
			// COUNTER RATE / (poles*period_measurement)
			// To get speed in RPM * this by 60
			
			// First Calculate period*number of rotor poles as this is
			// used more than once
			{				// added this brace S. Bowling
			asm("disi  #5");
			ltemp=((unsigned long)user_parameters[25] \
					*(unsigned long)period_measurement);
					
			// Make sure trap a zero period_measurement or any value
			// that would cause an overflow
			if (ltemp < (COUNTER_RATE*60/65535))
				rpm=65535;
				
			else
				rpm = COUNTER_RATE*60/ltemp;
			}				// added this brace S. Bowling
		// Now check that a stall has not ocurred which would mean
		// that period_measurement is not being updated leading to 
		// incorrect calculation of speed. This is done by having a counter
		// which increments every medium speed event and is reset in the 
		// sector 0 zero_crossing code where the period measurement is done
		// If the counter exceeds the user parameter stall time then it is
		// assumed we are in stall.
		// Stall detection is disabled unless actually running
 		if (run_state!=RUNNING) stall_counter=0;

		// Now check for overspeed and stall if not already
		// in a fault condition
		if (!trip_state)
		{
			if ((rpm>user_parameters[14]) && (control_flags.SENSORLESS))
			{
				DISABLE_FIRING;
				run_state=FAULT;
				trip_state=OVER_SPEED;
			}
		
			if (stall_counter>user_parameters[13])
				
			{
				DISABLE_FIRING;
				run_state=FAULT;
				trip_state=STALLED;
			}
			else
				stall_counter++;
		}
		// Now calculate phase advance
		// If before the start of phase advance then just load in the
		// value required to account for the fact that zero crossing is 
		// always always detected at least 1 PWM cycle late
		if (rpm < user_parameters[11])
		{
			phase_advance=TIME_CORRECTION;
		}
		// else within phase advance speed region
		else
		{
			// Calculate the amount of phase advance required in thousandths
			// of an electrical degree
			ltemp=(long)(rpm-user_parameters[11])*user_parameters[12];
			// Then use the current period_measurement (180 electrical degrees)
			// To calculate the appropriate phase advance time
			phase_advance=((ltemp*period_measurement)/180000)+TIME_CORRECTION;
		}
				
		// Call the speed loop if running sensorless
		if (control_flags.SENSORLESS==TRUE) speed_loop();

		voltage_control();

		if (run_state==STARTING)	starting_code();


	}
	return;
}

// This is the speed control loop which can be used in 4 ways
// When the speed control mode is selected to be open loop the PI
// control is bypassed and either the PWM duty is controlled directly
// from the pot (OPEN_VOLTS mode) or the demand for the current
// control loop is taken from the pot(OPEN_CURRENT mode).
// If the (CLOSED_VOLTS mode) is selected, then the output from the
// speed loop directly controls the PWM duty cycle.
// If the (CLOSED_CURRENT mode) is selected, then the output from the 
// speed loop forms the demand for the current control loop
// The control mode is selectable via the user menu
void speed_loop(void)
{

	static long speed_integral;
	int speed_error;
	long proportional_term, pos_wloop_limit, neg_wloop_limit;
	int control_output,scaled_demand;
	long temp;

	#ifdef DEBUG
		asm("bset	LATG,#2");
	#endif
	// If either not RUNNING or during acquistion to sensorless
	// transition then clear off the integral and return
	if ((run_state != RUNNING) || (check_counter))
	{
		speed_integral=0;

		return;
	}
	switch (user_parameters[1])
	{
		case OPEN_VOLTS:	speed_integral=0;
								// Disable updates of PWM duty registers to ensure
								// All 3 values get loaded together
								PWMCON2bits.UDIS=1;
								scaled_demand=filtered_pot/user_parameters[37];
								// Compensate for inverted firing as FULL_DUTY gives
								// zero duty cycle.
								// First make sure we don't underflow calculation of 
								// correct duty
								if (scaled_demand > FULL_DUTY) 	PDC1=0;
								else	PDC1=FULL_DUTY-scaled_demand;
								PDC2=PDC1;
								PDC3=PDC1;
								// Now enable updates
								PWMCON2bits.UDIS=0;
								return;
								break;

		case OPEN_CURRENT:speed_integral=0;
								scaled_demand=filtered_pot/user_parameters[38];
								if (scaled_demand >= (current_trip-ibus_offset))
									current_demand=current_trip-ibus_offset;
								else
									current_demand=scaled_demand;
								return;
								break;

		case CLOSED_VOLTS:// The *32 is because duty limits
								// also used by current loop where
								// scaling is *512, whereas here it is
								// *16384 hence the 32 factor.
								pos_wloop_limit=pos_duty_limit*32;
								neg_wloop_limit=neg_duty_limit*32;
								break;
		case CLOSED_CURRENT:
								pos_wloop_limit=pos_current_limit;
								neg_wloop_limit=neg_current_limit;
								break;
	}
	
	speed_demand=filtered_pot*user_parameters[39];
	control_flags.TORQUE_LIMIT=FALSE;

	// Form the speed error by subtracting the speed in rpm from the
	// speed demand read from VR2.
	speed_error=speed_demand-rpm;
	
	// All calculated quantities will be shifted back down at the 
	// end by 512. P & I gains are effectively scaled up by 512.
	// This is done to allow fractional values of P & I without
	// severe rounding issues which causes quantization noise etc.

	// Now calculate proportional term
	
	proportional_term=((long)speed_error*(long)wloop_p_gain);

	if (proportional_term > pos_wloop_limit)
	{
		control_flags.TORQUE_LIMIT=TRUE;
		control_output=pos_wloop_limit/16384;
		if (user_parameters[1]==CLOSED_VOLTS)
			control_output+=ZERO_DUTY;

		// If in limit due to just P term then clear
		// off I term so that when come out of limit
		// I term can smoothly take over to eliminate 
		// steady state error.
		speed_integral=0;
	}
	else
	{
		if (proportional_term < neg_wloop_limit)
		{
			control_flags.TORQUE_LIMIT=TRUE;
			control_output=neg_wloop_limit/16384;
			if (user_parameters[1]==CLOSED_VOLTS)
				control_output+=ZERO_DUTY;
			// If in limit due to just P term then clear
			// off I term so that when come out of limit
			// I term can smoothly take over to eliminate 
			// steady state error.
			speed_integral=0;
		}
	}
	// Now do integral term if not in torque limit already due to P term
	if (control_flags.TORQUE_LIMIT==FALSE)
	{
		// Update Intergral term here but this will be overwritten if
		// found to be in limit later on in code
		speed_integral+=((long)speed_error * (long)wloop_i_gain);
		temp=speed_integral+proportional_term;

		// If sum of P+I terms pushes you into positive limit
		if (temp > pos_wloop_limit)
		{
			control_flags.TORQUE_LIMIT=TRUE;
			control_output=pos_wloop_limit/16384;
			if (user_parameters[1]==CLOSED_VOLTS)
				control_output+=ZERO_DUTY;
			// Now calculate integrator limit and clamp
			speed_integral = pos_wloop_limit - proportional_term;
		}
		else
		{
			// If sum of P+I terms pushes you into negative limit
			if (temp < neg_wloop_limit)
			{
				control_flags.TORQUE_LIMIT=TRUE;
				control_output=(neg_wloop_limit/16384);
				if (user_parameters[1]==CLOSED_VOLTS)
					control_output+=ZERO_DUTY;
				// Now calculate integrator limit and clamp
				speed_integral=neg_wloop_limit - proportional_term;
			}
			else
			// Calculate control output based on both unclamped P and I terms
			{
				// Temp still contains unclamped P+I term so no need to recalculate
				control_output=(temp/16384);
				if (user_parameters[1]==CLOSED_VOLTS)
					control_output+=ZERO_DUTY;
			}
		}
	}
	if (user_parameters[1]==CLOSED_VOLTS)
	{
		// Do final check to ensure control_output is not < 0
		// as this would cause large positive value to be loaded
		// into duty cycle registers!
		if (control_output < 0) control_output=0;

		// Disable updates of PWM duty registers to ensure
		// All 3 values get loaded together
		PWMCON2bits.UDIS=1;
		// Now load calculated value into duty cycle registers
		// Compensate for inverted firing as FULL_DUTY gives
		// zero duty cycle.
		control_output=FULL_DUTY-control_output;		
		PDC1=(unsigned int)control_output;
		PDC2=(unsigned int)control_output;
		PDC3=(unsigned int)control_output;
		// Now enable updates
		PWMCON2bits.UDIS=0;
	}
	else
	{
		current_demand=control_output;
	}

	return;
}

// This routine does the voltage control of the DC bus
// As there is an issue with using OC modules, TIMER3
// is used to provide the PWM.
// This is done by directly setting the port pin here
// and loading PR3 with the on time. The TIMER3 ISR 
// sets the port pin low.
void voltage_control(void)
{
	static long voltage_integral;
	int voltage_error;
	long proportional_term;
	long temp;

	if ((run_state != RUNNING) && (run_state != STARTING))
	{
		voltage_integral=0;
		return;
	}

	// If previous on pulse still running then disable the 
	// interrupt used to time the on time and turn switch off
	if(IEC0bits.T3IE)	
	{
		IEC0bits.T3IE=0;
//		BRAKE_FIRE=0;
	}

	// Form the voltage error by subtracting the voltage demand 
	// in the user parameters from the filtered vdc feedback
	// Note that error is of the opposite polarity than usual as
	// if the volts are too high you want to fire the switch
	voltage_error=filtered_vdc-voltage_demand;
	
	// All calculated quantities will be shifted back down at the 
	// end by 512. P & I gains are effectively scaled up by 512.
	// This is done to allow fractional values of P & I without
	// severe rounding issues which causes quantization noise etc.

	// Now calculate proportional term
	
	proportional_term=((long)voltage_error*(long)vloop_p_gain);

	if (proportional_term > POS_V_LIMIT)
	{
		voltage_integral=0;
		return;
	}
	else
	{
		// Note that negative saturation is tested by looking at
		// voltage error with -10 or approx 1% of full scale.
		// (The actual limit is 0 as if volts too low just set
		// brake chopper switch on time to zero.) 
		// The use of -10 prevents chatter as otherwise if volts  
		// even 1 LSB too low, the integral term is cleared.
		if (voltage_error < -10)
		{
			voltage_integral=0;
			return;
		}
	}
	
	// Update Integral term here but this will be overwritten if
	// found to be in limit later on in code
	voltage_integral+=((long)voltage_error * (long)vloop_i_gain);
	temp=voltage_integral+proportional_term;

	// If sum of P+I terms pushes you into positive limit
	if (temp > POS_V_LIMIT)
	{
		voltage_integral = POS_V_LIMIT - proportional_term;
		return;
	}
	else
	{
		// If sum of P+I terms pushes you into negative limit
		// It is OK to use 0 as the negative limit here as
		// in steady state integral will be providing the
		// control effort and even if get a few LSBs of negative
		// error, voltage inegral is not cleared anyhow
		if (temp <= 0)
		{
			// Now calculate integrator limit and clamp
			voltage_integral = 0 - proportional_term;
			return;
		}
		else
		// Calculate control output based on both unclamped P and I terms
		{
			// Temp still contains unclamped P+I term so no need to recalculate
			// Load up PR3 with the On time of the switch
			PR3=(unsigned int)(temp/512);
			TMR3=0;
			// Set TIMER3 running so that when TMR3=PR3 switch turns off
			IFS0bits.T3IF=0;
			IEC0bits.T3IE=1;
		}
	}
	return;
}


void starting_code(void)
{
	unsigned int temp;
	unsigned long ltemp;
	static unsigned int rotation_timer=0;

	// First of all check to see whether motor is already turning
	// This is done by putting the system into acquire mode2
	// and by monitoring a countdown timer called rotation timer
	// If the system acquires it will automatically move to running
	// sensorless. If the timer reaches zero before the system has 
	// acquired then the normal lock and ramp routine is entered
	if (control_flags2.ROTATION_CHECK==TRUE)
	{
		DISABLE_INTERRUPTS;
		control_flags.ACQUIRE2=TRUE;
		control_flags.ADCCONFIG=TRUE;
		control_flags2.ROTATION_CHECK=FALSE;
		ENABLE_INTERRUPTS;
		rotation_timer=user_parameters[36];
		return;
	}
	
	if ((control_flags.ACQUIRE2==TRUE) && (rotation_timer))
	{
		rotation_timer--;
		return;
	}
	else
	{
		control_flags.ACQUIRE2=FALSE;
	}
	// Now check for windmilling (going opposite direction
	// to the demanded direction)
	// If windmilling then all we do is run open loop
	// decelerating motor with constant current demand
	// until near zero speed and then revert to normal
	// lock and ramp routine.
	if (control_flags2.WINDMILLING==TRUE)
	{
		if (new_PR2 < BRAKING_PR2_LIMIT)
		{
			new_PR2+=windmilling_decel_rate;
			return;
		}
		else
		{
			DISABLE_INTERRUPTS;
			control_flags2.WINDMILLING=FALSE;
			control_flags.DIR=user_parameters[0];
			IEC0bits.T2IE=0;
			control_flags.RAMP=FALSE;
			control_flags.LOCK1=FALSE;
			control_flags.LOCK2=FALSE;
			ENABLE_INTERRUPTS;
		}
	}
	// Are we in the ramping region of starting
	if (control_flags.RAMP==TRUE)
	{
		// The following section of code is done in assembly so that
		// can take advantage of the mul.us and div.sd instructions which
		// the compiler does not use due to ANSI C compliance
		// The calculations are:
		// new_speed= ramp_start_speed + (ramp_speed_delta*(starting_timer/ramp_time)^2)
		//	temp = ramp_start_demand + (ramp_demand_delta*starting_timer/ramp_time)
		// Note that the code is done within one asm instruction to ensure that
		// the compiler does not insert instructions when optimizing
		// First push w0-w5 onto the stack as the compiler may be using 
		// them to store intermediate results which are required later on
		// in the C code. This will be especially true if optimization is used

#define  V120	1	
		
#ifdef ORIG
		asm("	push.d	w0");
		asm("	push.d	w2");
		asm("	push.d	w4");
		asm("	mov.w	_starting_timer,w1");
		asm("	mov.w	_starting_timer,w0");
		asm("	mul.uu w0,w1,w4");
		asm("	mov.w _ramp_time,w2");
		asm("	repeat	#17");
		asm("	div.ud	w4,w2");
		asm("	mov.w _ramp_speed_delta,w1");
		asm("	mul.uu w0,w1,w4");
		asm("	repeat	#17");
		asm("	div.ud	w4,w2");
		asm("	mov.w	_ramp_start_speed,w1");
		asm("	add	w0,w1,w0");
		asm("	mov.w	w0,_new_speed");
		asm("	mov.w _ramp_demand_delta,w1");
		asm("	mov.w _starting_timer,w0");
		asm("	mul.us w0,w1,w4");
		asm("	mov.w _ramp_time,w2");
		asm("	repeat	#17");
		asm("	div.sd	w4,w2");
		asm("	mov.w	w0,_stemp");
		asm("	pop.d	w4");
		asm("	pop.d	w2");
		asm("	pop.d	w0");
#elif V120
                { unsigned long mul_result;
                  register unsigned int div_result asm("w0");
                  /* asm("instruction(s)" : outputs : inputs :
                                            clobbered registers ); */
 
                  asm volatile("mul.uu %1, %1, %0" : "=r"(mul_result) : 
                                            "r"(starting_timer));
                  asm volatile("repeat #17\n\t"
                      "div.ud %0, %1" : /* implied outputs */ : "r"(mul_result),
                                                                "r"(ramp_time) :
                                       "w0", "w1" );
                  asm volatile("mul.uu %1, %2, %0":"=r"(mul_result) : "r"(div_result),
                                                         "r"(ramp_speed_delta));
                  asm volatile("repeat #17\n\t"
                      "div.ud %0, %1" : /* implied outputs */ : "r"(mul_result),
                                                                "r"(ramp_time) :
                                       "w0", "w1" );
                  asm volatile("add %1, %2, %0" : "=r"(new_speed): "r"(ramp_start_speed),
                                                          "r"(div_result));
                  asm volatile("mul.us %1, %2, %0":"=r"(mul_result): "r"(starting_timer),
                                                        "r"(ramp_demand_delta));
                  asm volatile("repeat #17\n\t"
                      "div.ud %0, %1" : /* implied outputs */ : "r"(mul_result),
                                                                "r"(ramp_time) :
                                       "w0", "w1" );
                  stemp = div_result;
                }
#elif V130
                { unsigned long mul_result;
                  unsigned int div_result;

                  mul_result = __builtin_muluu(starting_timer, starting_timer);
                  div_result = __builtin_divud(mul_result, ramp_time);
                  mul_result = __builtin_muluu(div_result, ramp_speed_delta);
                  div_result = __builtin_divud(mul_result, ramp_time);
                  new_speed = div_result + ramp_start_speed;
                  mul_result =__builtin_mulus(starting_timer,ramp_demand_delta);
                  stemp = __builtin_divud(mul_result, ramp_time);
                }
#else
#error Which version
#endif
		temp=ramp_start_demand+stemp;
		
		// temp contains the new demand value for current ramp position
		// Now check to see if we are using voltage or current control
		if (user_parameters[40])
		{								
			// Compensate for inverted firing
			temp=FULL_DUTY-temp;
			// Disable updates of PWM duty registers to ensure
			// All 3 values get loaded together
			PWMCON2bits.UDIS=1;
			PDC1=temp;
			PDC2=temp;
			PDC3=temp;
			// Now enable updates
			PWMCON2bits.UDIS=0;
		}
		else
			current_demand=temp;
		
		// Calculate new period value based on new_speed value
		// number of motor poles and constants. Put it in ltemp
		ltemp=(COUNTER_RATE*20) / \
		((unsigned long)new_speed*(unsigned long)user_parameters[25]);

		// Check that a period overflow has not occurred before
		// actually writing the new value into new_PR2

		if (ltemp>65535)
			new_PR2=65535;
		else
			new_PR2=ltemp;
		// Now check to see which acquistion method is being used
		// If method 1 and the new step rate is < step rate where
		// zero crossing is enabled, then go ahead and set flag
		// so that zero crossings are checked during ramping.
		
		if ((user_parameters[43]==FALSE) && \
			(new_PR2<=acquire1_enable_rate) && \
			(control_flags.ACQUIRE1==FALSE))
		{
			control_flags2.ACQUIRE1_REQUEST=TRUE;
			acquire_counter=0;
		}

		// Now check to see if at end of ramp
		if (new_PR2 <=ramp_end_rate)
		{
			// Must ensure interrupt does not occur here
			// as otherwise can end up with ISR executing
			// with some flags correct and some not.
			// Also prefer to ensure all PWMs loaded up at
			// at same time to avoid glitches
			DISABLE_INTERRUPTS;
			control_flags.RAMP=FALSE;		
			DISABLE_FIRING;
			IEC0bits.T2IE = 0;

			// Now check to see which acquisition method being used
			// If using method 2 then set off acquire_position function
			// by setting appropriate flags
			if (user_parameters[43]==TRUE)
			{
				control_flags.ACQUIRE2=TRUE;
				control_flags.ADCCONFIG=TRUE;
				control_flags2.RETRY_FLAG=FALSE;
				run_state=RUNNING;
			}
			else 	// using Method1 should have already acquired before
			{		// end of ramp so enter fault condition
				run_state=FAULT;
				trip_state=FAILED_TO_START;
			}

			ENABLE_INTERRUPTS;
			return;
		}
	}	

	// If both LOCK flags are false then this is the first time we have entered
	// lock routines.

	if ((control_flags.LOCK1==FALSE) && (control_flags.LOCK2==FALSE))
	{
		if (control_flags.RAMP==FALSE)
		{
			starting_timer=0;
			control_flags.LOCK1=TRUE;
			// Now check to see if we are using voltage or current control
			if (user_parameters[40])
			{
				// Disable updates of PWM duty registers to ensure
				// All 3 values get loaded together
				PWMCON2bits.UDIS=1;
				// Compensate for inverted firing as FULL_DUTY gives
				// zero duty cycle.
				PDC1=(FULL_DUTY-hold1_demand);
				PDC2=PDC1;
				PDC3=PDC1;
				// Now enable updates
				PWMCON2bits.UDIS=0;
			}
			else
				current_demand=hold1_demand;

			// This write to OVDCON will only allow 2 of the 6 inverter
			// switches to actually fire so it does not matter that all
			// three duty cycle values were loaded with the demand
			OVDCON=SECTOR0_OVERRIDE;
			// Now enable firing
			ENABLE_FIRING;
		}
	}	
	
	// If at end of LOCK1 time
	if ((starting_timer >= user_parameters[2]) \
		&& (control_flags.LOCK1==TRUE))
	{
		starting_timer=0;
		control_flags.LOCK1=FALSE;
		control_flags.LOCK2=TRUE;

		// If using voltage control for starting
		if (user_parameters[40])
		{
			// Disable updates of PWM duty registers to ensure
			// All 3 values get loaded together
			PWMCON2bits.UDIS=1;

			// Compensate for inverted firing as FULL_DUTY gives
			// zero duty cycle.
			// Note that event though all three duty cycles are the 
			// same, the write to OVDCON below will only allow the 
			// correct switches to actually fire.
			PDC1=(FULL_DUTY-hold2_demand);
			PDC2=PDC1;
			PDC3=PDC1;
			// Now enable updates
			PWMCON2bits.UDIS=0;
		}
		else
			current_demand=hold2_demand;

		// Second energisation sector is direction dependent
		if (control_flags.DIR==FORWARDS)
			OVDCON=SECTOR1_OVERRIDE;
		else
			OVDCON=SECTOR5_OVERRIDE;
	}

	// If at end of LOCK2 time
	if ((starting_timer > user_parameters[3]) \
		&& (control_flags.LOCK2==TRUE))
	{
		control_flags.LOCK2=FALSE;
		control_flags.RAMP=TRUE;

		starting_timer=0;

		// Now energise motor with maximum
		// torque producive current which occurs
		// two sectors ahead. As have moved 1 sector
		// away from initial lock in correct direction
		// of rotation, this is the same sector whichever
		// direction we wish to rotate in.

		// If using voltage control for starting
		if (user_parameters[40])
		{
			// Disable updates of PWM duty registers to ensure
			// All 3 values get loaded together
			PWMCON2bits.UDIS=1;

			// Compensate for inverted firing as FULL_DUTY gives
			// zero duty cycle.
			PDC1=(FULL_DUTY-ramp_start_demand);
			PDC2=PDC1;
			PDC3=PDC1;
			// Now enable updates
			PWMCON2bits.UDIS=0;
		}
		else
			current_demand=ramp_start_demand;

		// Now check to see which acquistion method is being used
		// If method 1 and the initial ramp speed is > speed where
		// zero crossing is enabled, then go ahead and set flag
		// so that zero crossings are checked during ramping.
		if ((user_parameters[43]==FALSE) && \
			(ramp_start_rate<=acquire1_enable_rate))
		{
			control_flags2.ACQUIRE1_REQUEST=TRUE;
			acquire_counter=0;
		}
		
		// Write to the sector variable so that commutation ISR
		// will set up comutation correctly
		sector=3;
		// Load new_PR2 with initial step rate which will
		// be loaded into PR2 almost immediately as forcing
		// an interrupt of TMR2
		new_PR2=ramp_start_rate;
		IFS0bits.T2IF=TRUE;
		IEC0bits.T2IE=TRUE;
	}
	starting_timer++;
	return;
}
