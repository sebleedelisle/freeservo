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
 *    Filename:       user_interface.c	                                *
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
 *  This file contains all the code for implementing the user interface.
 *  The user interface functions are called from the slow event handler,
 *  which runs every 100 msec.
 *  Different screens are called depending on the operating state of the
 *  software.  The process_switches() function uses the debounced push
 *  buttons and determines the length of time the button has been 
 *  pressed for fast incrementing of the parameter. It also moves the 
 *  system between the various states according to the push buttons.
 *  The save_parameter() function reads an image of the Flash parameters
 *  into a RAM image, modifies the changed parameter, then writes the
 *  block back to Flash.
 *  The process_parameters() function does any scaling or units
 *  conversion that is required for the stored parameters.
 **********************************************************************/


#include "general.h"
#include "hardware.h"
#include "defs.h"
#include "extern_globals.h"


#ifdef DEVELOPMODE
/*********************************************************************/
// ASCII buffers for serial data to be transmitted
// This data is stored in Prog Mem and accessed as data memory 
// using the PSV feature of dsPIC
 struct parameter_data 
	{
    	char *line_msg;     //line 1 parameter screen message 
		char *quick_msg;	//abbriviation for message 
   	};
 struct fault_data
	{
		char *fault_msg;
	};

const struct fault_data fault_data [] =
{
	{"Press S2 to continue ..."},
	{"Failed to Start"},
	{"Over Current"},
	{"Over Voltage"},
	{"Hardware Trip"},
	{"Over Speed"},
	{"Sensorless Lost"},
	{"Motor Stalled"}
};

const unsigned char help_info[5][14]=
{
	{0,25,32,41,50,50,50,50,50,50,50,50,50,50},
	{26,27,28,29,33,34,35,36,37,38,39,42,50,50},
	{2,3,4,5,6,7,8,9,10,30,31,40,43,44},
	{1,11,12,17,18,19,20,21,22,23,24,50,50,50},
	{13,14,15,16,50,50,50,50,50,50,50,50,50,50},
};

 const struct parameter_data parameter_data [] =	
{
// Line msg, Quick msg 
{"DIRECTION       ","DD"},//0
{"CONTROL MODE    ","CM"},//1
{"Lock Pos.1 Time ","LP1T"},//2
{"Lock Pos.2  Time","LP2T"},//3
{"Lock Pos.1 Dem. ","LP1D"},//4
{"Lock Pos.2 Dem. ","LP2D"},//5
{"Ramp Start Speed","RSS"},//6
{"Ramp End Speed  ","RES"},//7
{"Ramp Start Dem. ","RSD"},//8
{"Ramp End Dem.   ","RED"},//9
{"Ramp Duration   ","RD"},//10
{"Phase Adv. Enable Spd ","PAES"},//11
{"Phase Adv. Slope","PAS"},//12
{"Stall Time Limit","STL"},//13
{"Over Speed Limit","OSL"},//14
{"Over Volts Limit","OVL"},//15
{"Over Current Lim","OCL"},//16
{"Current P Gain  ","CKP"},//17
{"Current I Gain  ","CKI"},//18
{"Current D Gain  ","CKD"},//19
{"Speed P Gain    ","SKP"},//20
{"Speed I Gain    ","SKI"},//21
{"Voltage Demand  ","VD"},//22
{"Volts P Gain    ","VKP"},//23
{"Volts I Gain    ","VKI"},//24
{"No. Motor Poles ","MP"},//25
{"Current Scale X ","CSX"},//26
{"Current Scale / ","CSD"},//27
{"Volts Scale X   ","VSX"},//28
{"Volts Scale /   ","VSD"},//29
{"Tolerance Check ","TC"},//30
{"Auto Re-acquire ","ARA"},//31
{"Blanking Count  ","BC"},//32
{"Zero X Level Thd","ZXL"},//33
{"Acquire Threshld","AT"},//34
{"Acquire Level Td","AL"},//35
{"Rotation Timeout","RT"},//36
{"Pot / for Duty  ","PDD"},//37
{"Pot / for Currnt","PDC"},//38
{"Pot X for Speed ","PXS"},//39
{"Starting Control","SC"},//40
{"Windmilling Dem.","WD"},//41
{"Braking Ramp T  ","BRT"},//42
{"Acquire Method  ","AM"},//43
{"ZeroX Enable Spd","ZXES"},//44
};

const unsigned char ParaHeader[] = 
			{"\r\nParameter Description\tParameter Abbreviation\tParameter Value\r\n"};

 const struct parameter_data HelpMsg_data [] =	
{	
	{"For Motor Parameters\t","Use '?M'"},
	{"For Starting Parameters\t", "Use '?S'"},
	{"For Control Parameters\t", "Use '?C'"},
	{"For Limit Parameters\t", "Use '?L'"},
	{"For Board Parameters\t", "Use '?B'"},
};

const unsigned char ErrorMsg[] = {"\r\nIncorrect Command! Use '??' For command set.\r\n"};
const unsigned char MotorParaHeader[] = {"Motor Parameters:\r\n"};
const unsigned char LimitParaHeader[] = {"Limit Parameters:\r\n"};
const unsigned char StartingParaHeader[] = {"Starting Parameters:\r\n"};
const unsigned char BoardParaHeader[] = {"Board Parameters:\r\n"};
const unsigned char ControlParaHeader[] = {"Control Parameters:\r\n"};
const unsigned char RunMessage[] = {"\r\n\r\n Speed = 2200 r.p.m.  DutyCycle = 72% Peak Current = 0245 mA \r\n"};
const unsigned char FaultMessage[] = {"\r\n\r\n Fault = Failed to Start \r\n"};

const unsigned char RunMsg1[] = {"Speed in rpm = "};
const unsigned char RunMsg2[] = {" % DutyCycle = "}; 
#endif

// User parameters are stored in program Flash as EEPROM is not currently functional.
// New values are to be stored using special routines written in assembler for erase
// and write. Read will be done via PSV thus allowing C code to access the values directly.
// The values must therefore be stored on a Flash memory row boundary as can only 
// erase 1 row (16 program locations) at a time. This is the purpose of the aligned attribute.
// Furthermore, can not use the normal const C declaration as otherwise have problems when 
// using compiler optimization. Therefore use the section attribute to instruct the linker
// to place the "variable" into the const section whilst compiler still treats it as a 
// variable which can change outside of it's control owing to volatile declaration
// The initialized values are the default values used when programming
// Note that even if all values in a block of 32 are not used, the array size must be declared
// in integar multiples of 32 as the erase can only be done on a single flash row of 32.
// If this is not done it is possible that linker may put other constants or even program code
// just after the array which are inadvertantly erased along with the parameters!
#ifndef DEVELOPMODE
volatile unsigned int user_parameters[64] __attribute__((aligned(64),far,section(".const,r")))=
#endif
#ifdef DEVELOPMODE
unsigned int user_parameters[45] = 
#endif
{	
	//BACKWARDS,
	FORWARDS,
	CLOSED_VOLTS,	// Speed Control Mode - See defs.h 
	50,				// First Lock Position Time in units of medium event (10ms) 
	50,				// Second Lock Position Time in units of medium event (10ms) 
	50,				// % Demand Used For Lock Position 1
	50,				// % Demand Used For Lock Position 2			
	100,			// Starting Speed for Ramp / RPM 
	2000,			// Finish Speed for Ramp / RPM	
	52,				// % Demand Used At Start of Ramp
	68,				// % Demand Used At End of Ramp	
	100,				// Duration of starting ramp in units of medium event (10ms)
	1500,			// Phase Advance Start Speed in RPM (1500 default) 
	25,				// Phase Advance Slope in 1/1000th elec. degree / RPM	
	100,			// Stall Time Limit in units of medium event (10ms) 
	3500,			// Over Speed Trip in RPM (3500 default)	
	500,			//	Over Voltage Trip in 1/10ths V
	100,			// Over Current Limit in 1/10ths A 
	900,			// Current Loop P Gain 
	100,			// Current Loop I Gain 
	0,				// Current Loop D Gain 
	2500,			// Speed Loop P Gain 
	40,				// Speed Loop I Gain 
	490,			// Voltage Demand for Brake Chopper in 1/10ths V 
	10000,			// Voltage Loop P Gain 
	5000,			// Voltage Loop I Gain 
	8,				// Number of Motor Poles 
	100,			// Current Scaling X - see below
	539,			// Current Scaling / - see below
	//239,
	100,			// Voltage Scaling X - see below
	//10,
	1305,			// Voltage Scaling / - see below
	//910,
	90,				// % Tolerance used for Lost Check - see below
	1,				// Auto-reaquire if lost ON/OFF
	1,				// Blanking Length used for zero X in no of ADC samples
	2,				// No. of samples required > or < VDC/2 before zero X checked for
	12,  			// ADC value used for rising edge detect for acquisition routine
	6,				// No of samples of VPH before rising edge detect done in acquisition
	5,				// Rotation Timeout in units of medium event (10ms)
	1,				// Divisor used to scale pot ADC reading to Duty Cycle
	8,				// Divisor used to scale pot ADC reading to I Dem in ADC
	3,				// X Used to Scale pot ADC reading to wdemand in rpm
	1,				// 1=Use Voltage Control for Starting, 0=Use Current Control
	20,				// % Demand Used For Windmilling Braking
	1,				// Duration of braking ramp in units of medium event (10ms)
	0,				// Use Method 1 (=0) or Method 2 (=1) for acquisition of position
	400};			// Speed at Which ZeroX detection enabled when using method1 acqusition

// Current and Voltage Scaling X and / Parameters
// These are used to scale ADC readings into Amps or Volts as follows: 
// If you get 12.87 A/V for ibus or 12.87 V/V for vdc then
// X = 100 and / = 1287 is one possible combination

// For LV Power Module Use The Following values: 
// VX=100 V/=1305 i.e scaling is 13.05 V/V		 
// If LK11&12 Open 	IX=100 I/=539 i.e scaling is 5.39A/V 
// If LK11&12 Closed	IX=10  I/=119 i.e.scaling is 11.9A/V 

// For HV Power Module Use The Following values: 
// VX=10  V/910 i.e. scaling is 91.0 V/V 
// If LK11&12 Open	IX=100 I/=108 i.e scaling is 1.08A/V 
// if LK11&12 Closed IX=100 I/=239 i.e scaling is 2.39A/V 

// Tolerance for Lost Check Parameter 
// Every 60 electrical degrees a new zero crossing event should
// be detected. In order to determine if the system is lost, 
// a check is carried out comparing the time elapsed since the 
// last zero crossing and the one before that. 
// If the tolerance parameter is set to 25% then up to 25%
// variation between the two times is considered to be acceptable.
// If the system fails the check, then the system is lost.
// Some natural variation in the times is to be expected even at
// a constant speed due to timer and ADC resolution as well as 
// motor asymmetry. Speed changes will also result in variation.
// It is suggested that the parameter is not set to less than 25%
// as otherwise the system may be determined to lost just due to
// normal variations.
/*********************************************************************/

#ifdef DEVELOPMODE
void GetMsgIndex(void);
void SendCRLF(void);
void SendErrorMsg(void);
void CheckHelp(void);
void SendHelpMsg(void);
void SendMsg(void);
void SendValue(unsigned int k);
void SaveValue(void);
void InitUart(void);
void SendHelpInfo(unsigned char Tab);
void SendMotorPara(void);
void SendLimitPara(void);
void SendControlPara(void);
void SendBoardPara(void);
void SendStartingPara(void);
void SendHeader(void);
void SendTab(void);
void SendRunMsg(void);
void SendFaultMsg(void);
void send_run(void);
void send_fault(void);
void serial_handler(void);
#endif

void process_switches(void);
void save_parameter(void);
void process_parameters(void);
void debounce_switches(void);
void uint_to_string(unsigned int, unsigned char *);

extern void erase_flash_row(unsigned int);
extern void program_flash(unsigned int, unsigned int);

struct interface_flags {
				unsigned EDIT_PARAM : 1;
				unsigned EDIT_MENU :	1;
				unsigned PARAM_CHANGING	: 1;
				unsigned S4_RISING :1;
				unsigned S5_RISING :1;
				unsigned S6_RISING :1;
				unsigned S7_RISING :1;
				unsigned RUN_FRAME :1;
				unsigned UNUSED : 8;
};

struct interface_flags interface_flags;




void process_switches(void)
{
	static unsigned char initializing_timer=20;
	static unsigned char previous_valid_switch_states=0xFF;
	static unsigned char key_hold_timer=0;
	static unsigned int param_increment=1;

	if (initializing_timer==1)	run_state=STANDBY;
	if (initializing_timer) initializing_timer--;



	if (((previous_valid_switch_states & 0x08)==FALSE) && (valid_switch_states & 0x08))
		interface_flags.S7_RISING=TRUE;
	else
		interface_flags.S7_RISING=FALSE;

	previous_valid_switch_states=valid_switch_states;


	switch(run_state)
	{
		case INITIALIZING: break;

		case STANDBY:	if (interface_flags.S7_RISING)
							{
								DISABLE_INTERRUPTS;
								control_flags2.ROTATION_CHECK=TRUE;
								control_flags2.WINDMILLING=FALSE;
								control_flags2.RETRY_FLAG=FALSE;
								control_flags.LOCK1=FALSE;
								control_flags.LOCK2=FALSE;
								control_flags.RAMP=FALSE;
								control_flags.SENSORLESS=FALSE;
								control_flags.ACQUIRE2=FALSE;
								control_flags.ACQUIRE1=FALSE;
								control_flags.DIR=user_parameters[0];
								ENABLE_INTERRUPTS;
								run_state=STARTING;
								interface_flags.EDIT_MENU=FALSE;
								interface_flags.RUN_FRAME=0;
								fault_count = 0;
							}
							if (interface_flags.S4_RISING)
								interface_flags.EDIT_MENU=TRUE;
							break;

		case STARTING:	if (interface_flags.S7_RISING)
							{
								DISABLE_FIRING;
								control_flags.SENSORLESS=FALSE;
								control_flags.ACQUIRE2=FALSE;
								IEC0bits.T1IE=FALSE;
								IEC0bits.T2IE=FALSE;
								run_state=STANDBY;
							}
							break;

		case RUNNING: 	if (interface_flags.S7_RISING)
							{
								DISABLE_FIRING;
								control_flags.SENSORLESS=FALSE;
								control_flags.ACQUIRE2=FALSE;
								IEC0bits.T1IE=FALSE;
								IEC0bits.T2IE=FALSE;
								run_state=STANDBY;
							}
							if (interface_flags.S4_RISING)
								interface_flags.RUN_FRAME= !interface_flags.RUN_FRAME;
							break;
		case FAULT:		if (interface_flags.S7_RISING)
							{
								trip_state=NO_TRIP;
								DISABLE_INTERRUPTS;
								control_flags.LOCK1=FALSE;
								control_flags.LOCK2=FALSE;
								control_flags.RAMP=FALSE;
								control_flags.SENSORLESS=FALSE;
								control_flags.ACQUIRE1=FALSE;
								control_flags.ACQUIRE2=FALSE;
								IEC0bits.T1IE=FALSE;
								IEC0bits.T2IE=FALSE;
								ENABLE_INTERRUPTS;
								period_measurement=1000;
								//FAULT_RESET=TRUE;
								//FAULT_RESET=FALSE;
								run_state=STANDBY;
							}
							if (interface_flags.S4_RISING)
								interface_flags.EDIT_MENU=TRUE;
							break;
		default:			break;
	}

	return;
}



// This function does the necessary calculations when a parameter
// value changes. Some parameters values are used directly, others
// form the basis for other variables but these need to be calculated.
void process_parameters(void)
{
	unsigned long ltemp,ltemp2;

	// If a value is missing from this switch statement this implies the
	// user parameter is used directly.
	// Note that parameters that affect other variables should also be in
	// this list e.g.if Voltage scaling changes, voltage demand and trips
	// need to be recalculated.
	switch (param)
	{
		case 0:	control_flags.DIR=user_parameters[0];
					break;
		case 4:	// If using voltage control for starting
					if (user_parameters[40])
						hold1_demand=(unsigned int)((unsigned long)user_parameters[4])*FULL_DUTY/100;
					else //Using current control assume scaling is in % of trip
					{
						ltemp=((unsigned long)(current_trip-ibus_offset));
						ltemp*=((unsigned long)user_parameters[4]);
						hold1_demand=(unsigned int)(ltemp/100);
					}	
					break;
		case 5:	// If using voltage control for starting
					if (user_parameters[40])
						hold2_demand=(unsigned int)((unsigned long)user_parameters[5])*FULL_DUTY/100;
					else //Using current control assume scaling is in % of trip
					{
						ltemp=(unsigned long)(current_trip-ibus_offset);
						ltemp*=((unsigned long)user_parameters[5]);
						hold2_demand=(unsigned int)(ltemp/100);
					}	
					break;
		case 8:	// If using voltage control for starting
					if (user_parameters[40])
						ramp_start_demand=(unsigned int)((unsigned long)user_parameters[8])*FULL_DUTY/100;
					else //Using current control assume scaling is in % of trip
					{
						ltemp=(unsigned long)(current_trip-ibus_offset);
						ltemp*=((unsigned long)user_parameters[8]);
						ramp_start_demand=(unsigned int)(ltemp/100);
					}
					ramp_demand_delta=(signed int)(ramp_end_demand-ramp_start_demand);
					break;

		case 9:	// If using voltage control for starting
					if (user_parameters[40])
						ramp_end_demand=(unsigned int)((unsigned long)user_parameters[9])*FULL_DUTY/100;
					else //Using current control assume scaling is in % of trip
					{
						ltemp=(unsigned long)(current_trip-ibus_offset);
						ltemp*=((unsigned long)user_parameters[9]);
						ramp_end_demand=(unsigned int)(ltemp/100);
					}
					ramp_demand_delta=(signed int)(ramp_end_demand-ramp_start_demand);
					break;

					// ramp time is used to hold user_parameters[10] so that
					// it can be directly referenced in some inline assembly 
		case 10: ramp_time=user_parameters[10];
					break;
		case 17:	iloop_p_gain=(int)user_parameters[17];
					break;
		case 18: iloop_i_gain=(int)user_parameters[18];
					break;
		case 19: iloop_d_gain=(int)user_parameters[19];
					break;
		case 20:	wloop_p_gain=(int)user_parameters[20];
					break;
		case 21:	wloop_i_gain=(int)user_parameters[21];
					break;
		case 23:	vloop_p_gain=(int)user_parameters[23];
					break;
		case 24:	vloop_i_gain=(int)user_parameters[24];
					break;
		case 6:	
		case 7:
		case 25:
		case 44: // Calculate step rates in units of TIMER2 from 
					// user parameters in RPM ensuring no overflows occur
					ltemp=((unsigned long)user_parameters[6]*(unsigned long)user_parameters[25])/20;
					// This check ensures that ramp_start_rate is calculated with no overflow
					if ((COUNTER_RATE/ltemp) > 65535)
					{	
						ramp_start_rate=65535;
						ramp_start_speed=COUNTER_RATE*20/(65535*(unsigned long)user_parameters[25]);
					}
					else
					{
						ramp_start_rate=(unsigned int)(COUNTER_RATE/ltemp);
						ramp_start_speed=user_parameters[6];
					}
					ltemp=((unsigned long)user_parameters[7]*(unsigned long)user_parameters[25])/20;
					// This check ensures that ramp_end_rate is calculated with no overflow
					if ((COUNTER_RATE/ltemp) > 65535)	
						ramp_end_rate=65535;
					else
						ramp_end_rate=(unsigned int)(COUNTER_RATE/ltemp);

					ramp_speed_delta=(int)(user_parameters[7]-user_parameters[6]);
					// Also calculate step rate at which zero X detection enabled when using
					// acquistion method 1
					ltemp=((unsigned long)user_parameters[44]*(unsigned long)user_parameters[25])/20;
					if ((COUNTER_RATE/ltemp) > 65535)
						acquire1_enable_rate=65535;
					else
						acquire1_enable_rate=(unsigned int)(COUNTER_RATE/ltemp);
					break;
		case 16:
		case 26:	
		case 27:	// Get value reported for ibus
					// At this stage assuming ibus=0 and have first valid sample
					// and so use this one for offset
					ibus_offset=ibus;
					ltemp=((unsigned long)user_parameters[16])*ADC_GAIN;
					ltemp*=((unsigned long)user_parameters[26]);
					current_trip=(ltemp/(user_parameters[27]*10))+ibus_offset;
					// Now calculate the current limits used when in CLOSED_CURRENT
					// speed control as 95% of the current trip levels
					pos_current_limit=(long)(current_trip-ibus_offset)*95/100;
					pos_current_limit*=16384L;
					neg_current_limit=-1*pos_current_limit;
					// Now recalculate starting parameters if using current control
					// for starting.
					if (user_parameters[40]==FALSE)
					{
						ltemp2=((unsigned long)(current_trip-ibus_offset));
						ltemp=((unsigned long)user_parameters[4])*ltemp2;
						hold1_demand=(unsigned int)(ltemp/100);
						ltemp=((unsigned long)user_parameters[5])*ltemp2;
						hold2_demand=(unsigned int)(ltemp/100);
						ltemp=((unsigned long)user_parameters[8])*ltemp2;
						ramp_start_demand=(unsigned int)(ltemp/100);
						ltemp=((unsigned long)user_parameters[9])*ltemp2;
						ramp_end_demand=(unsigned int)(ltemp/100);
						ramp_demand_delta=(signed int)(ramp_end_demand-ramp_start_demand);
					}
					ltemp2=((unsigned long)(current_trip-ibus_offset));
					ltemp=((unsigned long)user_parameters[41])*ltemp2;
					windmilling_demand=(ltemp/100);	
					break;
		case 15:
		case 22:
		case 28:
		case 29:	ltemp=((unsigned long)user_parameters[15])*ADC_GAIN;
					ltemp*=((unsigned long)user_parameters[28]);
					voltage_trip=ltemp/((unsigned long)user_parameters[29]*10);
					ltemp=((unsigned long)user_parameters[22])*ADC_GAIN;
					ltemp*=((unsigned long)user_parameters[28]);
					voltage_demand=ltemp/((unsigned long)user_parameters[29]*10);
					break;
		case 30: upper_tol=100+user_parameters[30];
					lower_tol=100-user_parameters[30];
					break;

		case 34: // Calculate acquision (method2) threshold values based
					// on user parameter and ADC value read during initialization.
					// This compensates for offset voltages due to ADC and power module.
					vph_red_threshold=vph_red+user_parameters[34];
					vph_yellow_threshold=vph_yellow+user_parameters[34];
					vph_blue_threshold=vph_blue+user_parameters[34];
					break;
		case 40:	if (user_parameters[40])
					{
						hold1_demand=(unsigned int)((unsigned long)user_parameters[4])*FULL_DUTY/100;
						hold2_demand=(unsigned int)((unsigned long)user_parameters[5])*FULL_DUTY/100;
						ramp_start_demand=(unsigned int)((unsigned long)user_parameters[8])*FULL_DUTY/100;
						ramp_end_demand=(unsigned int)((unsigned long)user_parameters[9])*FULL_DUTY/100;
					}
					else
					{
						ltemp2=((unsigned long)(current_trip-ibus_offset));
						ltemp=((unsigned long)user_parameters[4])*ltemp2;
						hold1_demand=(unsigned int)(ltemp/100);
						ltemp=((unsigned long)user_parameters[5])*ltemp2;
						hold2_demand=(unsigned int)(ltemp/100);
						ltemp=((unsigned long)user_parameters[8])*ltemp2;
						ramp_start_demand=(unsigned int)(ltemp/100);
						ltemp=((unsigned long)user_parameters[9])*ltemp2;
						ramp_end_demand=(unsigned int)(ltemp/100);
					}
					ramp_demand_delta=(int)(ramp_end_demand-ramp_start_demand);
					break;
		case 41: ltemp2=((unsigned long)(current_trip-ibus_offset));
					ltemp=((unsigned long)user_parameters[41])*ltemp2;
					windmilling_demand=(unsigned int)(ltemp/100);
					break;
		default:	break;
	}
	return;
} 

// This function does a simple switch debounce by not
// updating the global variable valid_switch_states
// unless all 4 push buttons have been in the same
// state for 3 calls of the function
void debounce_switches(void)
{
	static unsigned char oldest_switch_states=0;
	static unsigned char previous_switch_states=0;
	unsigned char switch_states;

	// The four push buttons are on PORTG6-9 but have pull
	// up resistors making a logic 0 equal to a button press
	// So we complement and shift them down to be aligned to 
	//	the bottom which will also effectively mask off all other bits.

	//switch_states=(unsigned char)((~PORTG)>>6);
	if (!PORTCbits.RC14)
		switch_states = 0x8;
	else switch_states = 0x0;
	if (switch_states!=previous_switch_states)
	{
		oldest_switch_states=previous_switch_states;
		previous_switch_states=switch_states;
		return;
	}

	if (previous_switch_states != oldest_switch_states)
	{
		oldest_switch_states=previous_switch_states;
		previous_switch_states=switch_states;
	}
	else
	{
		valid_switch_states=switch_states;
		oldest_switch_states=previous_switch_states;
		previous_switch_states=switch_states;
	}
	return;
}

// ******************************************************************
// The code below is only compiled when in DEVELOPMODE is TRUE
//*******************************************************************
#ifdef DEVELOPMODE

void serial_handler(void)
{

	if (control_flags2.Fault)
		control_flags2.Fault = 0;
	if (control_flags2.CheckRX)
		{
		CheckHelp();
		if (!control_flags2.HelpMsg)
			GetMsgIndex();
		if (control_flags2.SendTX)
			{
			SendCRLF();
			TXPtr = &parameter_data[Index].line_msg[0];
			SendMsg();
			SendTab();
			SendValue(user_parameters[Index]);
			} 	
		// initialize pointer to first char
		if (control_flags2.ErrorMsg)
			SendErrorMsg();
		RXPtr = &InData[0];
		SendCRLF();
		control_flags2.SendTX = 0;
		control_flags2.CheckRX = 0;
		control_flags2.ErrorMsg = 0;
		control_flags2.HelpMsg = 0;	
		}
}

void CheckHelp(void)
{
	if (InData[0] == '?')
	{
		switch (InData[1])
		{
		case '?': 	SendHelpMsg();
					break;
		case 'M': 	SendMotorPara();
					break;
		case 'L':	SendLimitPara();
					break;
		case 'C':	SendControlPara();
					break;
		case 'S':	SendStartingPara();
					break;
		case 'B':	SendBoardPara();
					break;
		case 'R':	SendRunMsg();
					break;
		case 'F': 	SendFaultMsg();
					break;
		default:	SendErrorMsg();
		}
	}
}

void GetMsgIndex(void)
{
	Index = 0;
	control_flags2.SendTX = 0;
	
	while (Index < MAXINDEX)
		{
		RXPtr = &InData[0];
		TXPtr = &parameter_data[Index].quick_msg[0];
		while(1)
			if (*RXPtr++ == *TXPtr++)
				if (*RXPtr == 0x20 || *RXPtr == 0x0D)
					break;
				else ;
			else
				{control_flags2.NextIndex = 1; break;}
		if (!control_flags2.NextIndex)
			if (*RXPtr == 0x20) 
				{SaveValue();break;}
			else 
				{control_flags2.SendTX = 1; break;}
		else {control_flags2.NextIndex = 0;Index++;}
		}
	if (Index == MAXINDEX) control_flags2.ErrorMsg = 1;
	else control_flags2.SendTX = 1;
}

void SaveValue()
{
unsigned char Temp[6]={"000000"};
unsigned char i, l;
 
	while (*RXPtr++ != CR);
	*RXPtr--; l = 5;
	while (*RXPtr != 0x20)
		Temp[l--] = *RXPtr--;
	user_parameters[Index] = atoi(Temp);
	// Now force any variables which depend on user_parameters
	// to be calculated by calling process_parameters for every
	// parameter. 
	
	for (i=0;i<NO_PARAMETERS;i++)
	{
		param=i;
		process_parameters();
	}
	// Set param back to start of menu
	param=0;
}

void SendTab(void)
{

		while (U1STAbits.UTXBF);
		U1TXREG = HT;
}


void SendCRLF(void)
{
	while (U1STAbits.UTXBF);
	U1TXREG = CR;	
	while (U1STAbits.UTXBF);
	U1TXREG = LF;
}

void SendErrorMsg(void)
{
TXPtr = (unsigned char*)&ErrorMsg[0];
SendMsg();
control_flags2.ErrorMsg = 0;
}

void SendMsg(void)
{
while (*TXPtr)
	{
	while (U1STAbits.UTXBF);
	U1TXREG = *TXPtr++;
	}
}

void SendHelpMsg(void)
{

Index = 0;
do
		{
		TXPtr = &HelpMsg_data[Index].line_msg[0];
		SendMsg();
		TXPtr = &HelpMsg_data[Index].quick_msg[0];
		SendMsg();	
		SendCRLF();
		} 
	while (++Index < MAXHELPMSG);
control_flags2.HelpMsg = 1;
}
void SendHelpInfo(unsigned char Tab)
{

unsigned char i;
i=0;
Index = 0;
do
	if (Index == help_info[Tab][i])
		{
		TXPtr = &parameter_data[Index].line_msg[0];
		SendMsg();
		SendTab();
		SendTab();
		TXPtr = &parameter_data[Index].quick_msg[0];
		SendMsg();
		SendTab();
		SendTab();
		SendValue(user_parameters[Index]);	
		SendCRLF();
		i++;
		} 
	while (++Index < MAXINDEX);
control_flags2.HelpMsg = 1;
}


		
void SendValue(unsigned int k)
{
//unsigned int k;
unsigned char c;


	c = k/10000;
	if (c > 0)
		k = k - c*10000;
	while (U1STAbits.UTXBF);
	U1TXREG = (c + 0x30);
	c = k/1000;
	if (c > 0)
		k = k - c*1000;
	while (U1STAbits.UTXBF);
	U1TXREG = (c + 0x30);
	c = k/100;
	if (c > 0)
		k = k - c*100;
	while (U1STAbits.UTXBF);
	U1TXREG = (c + 0x30);
	c = k/10;
	if (c > 0)
		k = k - c*10;
	while (U1STAbits.UTXBF);
	U1TXREG = (c + 0x30);
	while (U1STAbits.UTXBF);
	U1TXREG = (char)(k + 0x30);
}



void SendFaultMsg(void)
{

	TXPtr = (unsigned char *)&FaultMessage[0];
	SendMsg();
	control_flags2.HelpMsg = 1;
}



void SendRunMsg(void)
{
	TXPtr = (unsigned char *)&RunMessage[0];
	SendMsg();
	control_flags2.HelpMsg = 1;

}

void SendMotorPara(void)
{
	TXPtr =  (unsigned char *)&MotorParaHeader[0];
	SendMsg();
	SendHeader();
	SendHelpInfo(MOTORPARA);
}

void SendLimitPara(void)
{
	TXPtr = (unsigned char *)&LimitParaHeader[0];
	SendMsg();	
	SendHeader();
	SendHelpInfo(LIMITPARA);	
}

void SendControlPara(void)
{
	TXPtr = (unsigned char *)&ControlParaHeader[0];
	SendMsg();
	SendHeader();
	SendHelpInfo(CONTROLPARA);
}

void SendStartingPara(void)
{
	TXPtr = (unsigned char *)&StartingParaHeader[0];
	SendMsg();
	SendHeader();
	SendHelpInfo(STARTINGPARA);
}

void SendBoardPara(void)
{
	TXPtr = (unsigned char *)&BoardParaHeader[0];
	SendMsg();
	SendHeader();
	SendHelpInfo(BOARDPARA);
}

void SendHeader(void)
{

	TXPtr = (unsigned char *)&ParaHeader[0];
	SendMsg();
}
	
void send_run(void)
{

unsigned int k;
	
	k = PDC1 >> 1;
	k = PTPER - k;
	k = k*100;
	k = k/PTPER; 
	TXPtr = (unsigned char *)&RunMsg1[0];
	SendMsg();
	SendValue(filtered_rpm);
	SendTab();
	TXPtr = (unsigned char *)&RunMsg2[0];
	SendMsg();
	SendValue(k);
	while (U1STAbits.UTXBF);
	U1TXREG = CR;
}


void send_fault(void)
{
	if (!control_flags2.Fault)
		{
		control_flags2.Fault = 1;
		SendCRLF();
		TXPtr = (unsigned char *)&fault_data[trip_state].fault_msg[0];
		SendMsg();
		SendTab();
		TXPtr = (unsigned char *)&fault_data[0].fault_msg[0];
		SendMsg();
		SendCRLF();
		}		

}


#endif
