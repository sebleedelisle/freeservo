#include "defs.h"

/*********************************************************************/
/*                        Variable Declaration                       */
/*********************************************************************/

volatile unsigned char run_state=0;
volatile unsigned char trip_state=0;

unsigned char sector=0;

unsigned int hold1_demand;			//Demand for first lock
unsigned int hold2_demand;			//Demand for second lock
unsigned int ramp_start_speed;	//Ramp start speed in RPM
unsigned int ramp_speed_delta;	//Ramp end speed - ramp start speed in RPM
unsigned int ramp_start_demand;	//Demand for start of ramp
unsigned int ramp_end_demand;		//Demand for end of ramp
unsigned int ramp_time;				//Ramp time in 10's of ms
unsigned int windmilling_demand;	//Current demand used for windmilling
unsigned int windmilling_decel_rate;
											//Holds increment for commutation period
											//used to decelerate the motor if windmilling
unsigned int acquire1_enable_rate;
											//Holds counter rate at which zeroX checking
											//enabled when using acquire method1

unsigned int current_trip;			// Trip Level in ADC counts
unsigned int voltage_trip;			// Trip level in ADC counts
unsigned int speed_trip;			// Speed Trip in RPM

int voltage_demand;					// Voltage Demand in ADC counts

int wloop_p_gain;						// Control gains for the three
int wloop_i_gain;						// different control loops.
int iloop_p_gain;						
int iloop_i_gain;						// These are used directly within
int iloop_d_gain;						// the control loops so don't have
int vloop_p_gain;						// to do unsigned -> signed conversions
int vloop_i_gain;						// every call of the control loops as 
											// user parameters are stored unsigned.
											// The vars are updated by process_parameters()

// The pos limit should be set to just
// beneath the over current limit in adc counts
// * 16384
long pos_current_limit=3200000L;

// As no braking is provided in normal running
// this value is used to detect when it is 
// appropriate to clear off speed integral
long neg_current_limit=-3200000L;

// For full scale these would be
// initialised to +/- PTPER*512 respectively
// The neg duty limit is deliberately not made 
// the same magnitude as pos to help prevent 
// the system getting lost.
// If the system is allowed to demand zero duty
// cycle then it will definitely get lost!
// FULL_DUTY is declared in defs.h and is 2*PTPER

long pos_duty_limit = FULL_DUTY*256L;
long neg_duty_limit = FULL_DUTY*-200L;

unsigned int upper_tol;
unsigned int lower_tol;

int current_demand=0;				// Current Demand to the Speed Loop

// This variable is used when erasing a block of 32 parameters
unsigned int parameter_mirror[32];

// Software counters used to schedule slow and medium speed events
// As these are incremented in PWM ISR they are declared as volatile
volatile unsigned int slow_event_count=0;
volatile unsigned char medium_speed_event_count=0;

// These vars are used to hold the feedback values
unsigned int vdc=0;
unsigned int ibus=0;
unsigned int vph=0;
unsigned int pot=0;
unsigned int vph_red=0;
unsigned int vph_yellow=0;
unsigned int vph_blue=0;
unsigned int vph_red_threshold=0;
unsigned int vph_yellow_threshold=0;
unsigned int vph_blue_threshold=0;

struct control_flags {		
				unsigned TORQUE_LIMIT : 1;
				unsigned VOLTS_LIMIT	: 1;
				unsigned DIR		: 1;
				unsigned ADCCONFIG: 1;
				unsigned ZERO_CROSS : 1;
				unsigned LEVEL		: 1;
				unsigned ACQUIRE1	: 1;
				unsigned SWAP	: 1;
				unsigned SENSORLESS : 1;
				unsigned	ACQUIRE2	: 1;
				unsigned ACQUIRE2_RED : 1;
				unsigned ACQUIRE2_YELLOW : 1;
				unsigned ACQUIRE2_BLUE : 1;
				unsigned LOCK1 : 1;
				unsigned LOCK2	: 1;
				unsigned RAMP	: 1;
};
// control flags declared as volatile to ensure 
// images of them are not used as this causes
// issues with ISRs modifying flags which are
// then overwritten by image in background code
volatile struct control_flags control_flags;

struct control_flags2	{
				unsigned AUTO_REACQUIRE	:	1;
				unsigned ROTATION_CHECK	:	1;
				unsigned WINDMILLING		:	1;
				unsigned RETRY_FLAG		:	1;
				unsigned D_TERM_DISABLE	:	1;
				unsigned ACQUIRE1_REQUEST :1;
				unsigned FALLING_EDGE	:	1;
				unsigned	CheckRX:1;
        		unsigned   	SendTX:1;
				unsigned 	ErrorMsg:1;
				unsigned	HelpMsg:1;
				unsigned 	NextIndex:1;
				unsigned	Fault:1;
				unsigned UNUSED			:	3;
								};

volatile struct control_flags2 control_flags2;

unsigned int previous_timestamps[] = {0,0,0};
//unsigned int edge_deltas[]= {0,0,0};

unsigned int adc_channel_config=0;

// new_PR2 needs to be a volatile as it is updated
// inside inline assembly
volatile unsigned int new_PR2=0x1800;

// Var used to hold ADC of ibus f/b when i=0
unsigned int ibus_offset=512;

// Set period measurement to such a value that will
// not cause false over speed trip on initialization
unsigned int period_measurement=1000;
unsigned int rpm=0;

// Demand to speed loop
unsigned int speed_demand=500;

unsigned int stall_counter=0;

unsigned int phase_advance=0;

// This var is used to contain the latest time different
// between two zero crossings. It is a global so that 
// inline assembler code can pick up reference to it
unsigned int latest_delta=1;
unsigned int previous_delta=1;

// This var is used to contain the time in TMR2 counts
// before next commutation should occur
int commutation_time=1;

// check_value needs to be volatile as it is updated
// within inline assembly
// It is used for the sensorless tolerance check
volatile unsigned int check_value=0;

// check counter needs to be volatile as it is updated
// in ISR code but used in background code
volatile unsigned char check_counter=0;

// acquire_counter is used when acquiring using "method1"
// It is incremented every valid zero crossing detected
// and reset if a commutation occurs before a valid
// crossing is detected.

volatile unsigned char acquire_counter=0;

unsigned int ramp_start_rate;
unsigned int ramp_end_rate;

unsigned int starting_timer;
int ramp_rate_delta;
int ramp_demand_delta;

// stemp needs to be volatile as it is updated
// within inline assembly
volatile int stemp;

// new_speed needs to be volatile as it is updated
// within inline assembly
volatile unsigned int new_speed;

unsigned int filtered_rpm;
unsigned int filtered_vdc;
unsigned int filtered_ibus;
unsigned int filtered_pot;

// Initialize valid swicth state with a value
// it can never get in practice to force a set
// of the flag signifying swicthes have changed
unsigned char valid_switch_states=0xff;

unsigned char param=0;

unsigned int new_param_value=0;
unsigned int param_increment=1;
unsigned int max_param_value=1;
unsigned int min_param_value=1;

#ifdef DEVELOPMODE
unsigned char *RXPtr;
unsigned char InData[8];
unsigned char Index;
unsigned char *TXPtr;
unsigned char Tab;
#endif

unsigned int fault_count=0;

#ifdef DEBUG
	unsigned char indx=0;
#endif
