;This file is used for defining the variables and compile time options
;This file should be included in all .asm/.c files
;Author : Padmaraja Yedamale
;Version: V1.1

	include		"P18F2431.inc"
	

;User defined variables
;---------------------------------------------------------------------------------------
;Oscillator frequency
#define OSCILLATOR	d'20000000'
;---------------------------------------------------------------------------------------
;Enter the PPR of the Optical Encoder on the motor
#define ENCODER_PPR d'1024'
;---------------------------------------------------------------------------------------
;Claculating RPM based on the encoder type. The QEI should be in velocity measurement mode
;Timer5 prescaler
#define	TIMER5_PRESCALE	d'1'
;---------------------------------------------------------------------------------------
;Define the QEI mode of operation. 
;If the velocity conter is updated only on QEA transition, then enable 2x mode
;If the velocity conter is updated every QEA and QEB transition, then enable 4x mode
#define QEI_X_UPDATE d'2'
;#define QEI_X_UPDATE d'4'
;---------------------------------------------------------------------------------------
;Define Velocity pulse decimation ratio(Only one line should be selected)
#define VELOCITY_PULSE_DECIMATION d'1' 
;#define VELOCITY_PULSE_DECIMATION d'4' 
;#define VELOCITY_PULSE_DECIMATION d'16' 
;#define VELOCITY_PULSE_DECIMATION d'64' 
;---------------------------------------------------------------------------------------
;Calculate the constant to be divided by the Velocity counter value will give RPS
INSTRUCTION_CYCLE = (OSCILLATOR)/d'4'
RPM_CONSTANT_QEI = 8*((INSTRUCTION_CYCLE)/(ENCODER_PPR*QEI_X_UPDATE*VELOCITY_PULSE_DECIMATION*TIMER5_PRESCALE))
;The value is multiplied by 8 to increase the accuracy of division. After devision, the result is devided by 8
RPM_CONSTANT_HALL = INSTRUCTION_CYCLE/(d'12'*d'8')
;60 to convert RPM to RPS, 12= # of Hall pulses/revolution
;---------------------------------------------------------------------------------------
;PWM frequency definition
#define PWM_TIMER_PRESCALE	d'01'
#define	PWM_FREQUENCY	d'16000'
PTPER_VALUE = (OSCILLATOR/(4*PWM_FREQUENCY*PWM_TIMER_PRESCALE))-1
;---------------------------------------------------------------------------------------
;Defining the PWM duty cycle constant based on the Motor voltage, DC bus voltage and PWM period register value
#define MOTOR_VOLTAGE d'24'
#define DC_INPUT_VOLTAGE	d'24'
#define MOTOR_RATED_SPEED d'2500'

MAIN_PWM_CONSTANT =(MOTOR_VOLTAGE*PTPER_VALUE*4*d'16')/(DC_INPUT_VOLTAGE*d'256')
;1.414*256 = 360
;Multiplication factor = 16
ERROR_PWM_CONSTANT = (MAIN_PWM_CONSTANT * d'256')/MOTOR_RATED_SPEED
MAX_PWM_VALUE = (MOTOR_VOLTAGE*PTPER_VALUE*4*d'100')/(DC_INPUT_VOLTAGE* d'141')
;---------------------------------------------------------------------------------------
ESTIMATE_TIMER_CONSTANT = ((d'60'* INSTRUCTION_CYCLE)/(MOTOR_RATED_SPEED*d'12'))+1

;******************************************************************
#define	CYCLE_COUNT_MAXH	0x4E
#define	CYCLE_COUNT_MAXL	0x20
#define	MAX_FLTA_COUNT	0x80

;----------------------------------------------
;FLAGS bits
#define	HALL_FLAG		0
#define	FLAG_FAULT		1
#define	PARAM_DISPLAY 	2
#define	POSITION_BIT 	3
#define	VELOCITY_READY 	4
#define	NEGATIVE_ERROR 	5
#define CALC_PWM		6
#define	VALID_HALL		7


;FLAGS1 bits
#define	DEBOUNCE	0
#define	KEY_RS		1
#define	KEY_FR		2
#define	KEY_PRESSED 3
#define	RUN_STOP 4
#define	FWD_REV	5

;FLT_FLAGS bits
#define	OCUR	0
#define	OVOLT	1
#define	OTEMP	2

;Delay parameters
#define	DELAY_COUNT1	0xFF
#define	DELAY_COUNT2	0xFF
;----------------------------------------------------------
;#define BLDC_MOTOR_CONTROL_BOARD	;uncomment if older version of the board is used
#define PICDEM_MC_LV				;uncomment if the release version of PICDEM LV is used.
;----------------------------------------------------------

#ifdef BLDC_MOTOR_CONTROL_BOARD
;Keys parameters
#define KEY_PORT PORTC
#define RUN_STOP_KEY 4
#define FWD_REV_KEY 5
#define DEBOUNCE_COUNT 0x8F

;LED parameters
#define LED_PORT PORTC
#define RUN_STOP_LED 0
#define FWD_REV_LED 2

#define	LED1	PORTC,0
#define	LED2	PORTC,2
#define	LED3	PORTC,3

;PORTC<0> : LED1, Output
;PORTC<1> : FLTA, Input
;PORTC<2> : LED2, Output
;PORTC<3> : LED3, Output
;PORTC<4> : Switch S3, Input
;PORTC<5> : Switch S2, Input
;PORTC<6> : TX, Output
;PORTC<7> : RX, Input
#define TRISC_VALUE b'00110010'
#endif



#ifdef PICDEM_MC_LV
;Keys parameters
#define KEY_PORT PORTC
#define RUN_STOP_KEY 0
#define FWD_REV_KEY 2
#define DEBOUNCE_COUNT 0x8F

;LED parameters
#define LED_PORT PORTC
#define RUN_STOP_LED 5
#define FWD_REV_LED 4

#define	LED1	PORTC,5
#define	LED2	PORTC,4
#define	LED3	PORTC,3

;PORTC<0> : Switch S3, Input 
;PORTC<1> : FLTA, Input
;PORTC<2> : Switch S2, Input
;PORTC<3> : LED3, Output
;PORTC<4> : LED2, Output
;PORTC<5> : LED1, Output
;PORTC<6> : TX, Output
;PORTC<7> : RX, Input

#define TRISC_VALUE b'00000111'

#endif


;ADC initialization values

#define ADCON0_VALUE b'00010001'	;Multi channel, Sequenctial mode conversion of Group A&B
#define ADCON1_VALUE b'00010000'	;FIFO enabled, Vref+=AVdd, Vref-= AVss
#define ADCON2_VALUE b'00110010'	;Left justified result, 64Tad, Fosc/32 clock
#define ADCON3_VALUE b'01000000'	;Interrupt every 2nd and 4th written to FIFO, no ext trigger 
#define ADCHS_VALUE b'00000000'		;AN0 and AN1 selected from group A&B
#define ANSEL0_VALUE b'00000011'	;;AN0 and AN1 selected as analog inputs
#define ANSEL1_VALUE b'00000000'	




;******************************************************************

;Configurartin bits

	LIST p=18f2431,f=INHX32

;Old way of defining the configuration registers
;	__CONFIG _CONFIG1H, 0x06 ;_OSC_HS_1H &_FCMEN_OFF_1H&_IESO_OFF_1H
;	__CONFIG _CONFIG2L, 0x0E ;_PWRTEN_ON_2L & _BOREN_ON_2L & _BORV_20_2L  
;	__CONFIG _CONFIG2H, 0x1E ;_WDTEN_OFF_2H
;	__CONFIG _CONFIG3L, 0x3C ;0x24 ;_PWMPIN_OFF_3L & _LPOL_LOW_3L & _HPOL_LOW_3L & _GPTREN_ON_3L
;	__CONFIG _CONFIG3H, 0x9D ;_FLTAMX_RC1_3H & _PWM4MX_RB5_3H
;	__CONFIG _CONFIG4L, 0x80 
;	__CONFIG _CONFIG5L, 0x0F 
;	__CONFIG _CONFIG5H, 0xC0  
;	__CONFIG _CONFIG6L, 0x0F 
;	__CONFIG _CONFIG6H, 0xE0 
;	__CONFIG _CONFIG7L, 0x0F 
;	__CONFIG _CONFIG7H, 0x40    

;New way of defining the configuration registers
	CONFIG OSC=HSPLL, FCMEN=OFF, IESO=OFF
	CONFIG PWRTEN=ON, BOREN=ON, BORV=27
	CONFIG WDTEN=OFF, WINEN=OFF
	CONFIG T1OSCMX=OFF, HPOL=HIGH, LPOL=HIGH, PWMPIN=OFF
	CONFIG MCLRE=ON
	CONFIG STVREN=OFF, LVP=OFF
	CONFIG CP0=OFF, CP1=OFF, CP2=OFF, CP3=OFF
	CONFIG CPB=OFF, CPD=OFF
	CONFIG WRT0=OFF, WRT1=OFF
	CONFIG WRTB=OFF, WRTC=OFF, WRTD=OFF
	CONFIG EBTR0=OFF, EBTR1=OFF 
	CONFIG EBTRB=OFF



;--------------------------------------------------
;Write the commutation sequence from the winding energizing table given below
;-------------------------------------------------------------
; |	Hall A | Hall B | Hall C | Phase A | Phase B | Phase C |
;-------------------------------------------------------------
; |	   0   |	0	|	0	 |	---	   |	---	 |	---	   |
; |	   1   |	0	|	0	 |	DC+	   |	---	 |	DC-	   |
; |	   0   |	1	|	0	 |	---	   |	DC-	 |	DC+    |
; |	   1   |	1	|	0	 |	DC+	   |	DC-	 |	---    |
; |	   0   |	0	|	1	 |	DC-	   |	DC+	 |	---    |
; |	   1   |	0	|	1	 |	---	   |	DC+	 |	DC-    |
; |	   0   |	1	|	1	 |	DC-	   |    ---	 |	DC+    |
; |	   1   |	1	|	1	 |	---	   |	---	 |	---    |
;-------------------------------------------------------------

;Define the following bytes based on the table above(corresponding to the Hall inputs)
;These will be loaded to POSITION_TABLE_FWD<0> to POSITION_TABLE_FWD<7> and 
;POSITION_TABLE_REV<0> to POSITION_TABLE_REV<7> upon initialization 
;Be sure not to enable more than two PWMs for a position
;and the PWMs enabled should not control the MOSFETs on the same half bridge. 
;Refer to the circuit schematic to findout which PWM is controlling which MOSFET.  
;Note: 
;For the given HURST motor, the reverse energizing sequence can be obtained from the forward sequence just by complementing the Hall states  
;Some motors may have different sequence for forward and reverse rotation. That is the reason for giving two different tables for the sequence
;----------------------------------------------------
#define	POSITION0	b'00000000'	;This will be loaded to OVDCOND register when HallA=0,HallB=0,HallC=0
#define	POSITION1	b'00010010'	;This will be loaded to OVDCOND register when HallA=1,HallB=0,HallC=0
#define	POSITION2	b'00100100'	;This will be loaded to OVDCOND register when HallA=0,HallB=1,HallC=0
#define	POSITION3	b'00000110' ;This will be loaded to OVDCOND register when HallA=1,HallB=1,HallC=0
#define	POSITION4	b'00001001' ;This will be loaded to OVDCOND register when HallA=0,HallB=0,HallC=1
#define	POSITION5	b'00011000' ;This will be loaded to OVDCOND register when HallA=1,HallB=0,HallC=1
#define	POSITION6	b'00100001' ;This will be loaded to OVDCOND register when HallA=0,HallB=1,HallC=1
#define	POSITION7	b'00000000' ;This will be loaded to OVDCOND register when HallA=1,HallB=1,HallC=1
;****************************************************************
;****************************************************************
;****************************************************************
;****************************************************************
;----------------------------------------------------
;Hurst Motor power connection(White square connector with 4 wires)
;Red : connect to M1 on J9 on the board 
;Black : connect to M2 on J9 on the board 
;White : connect to M3 on J9 on the board 
;Green : connect to G on J9 on the board 
;----------------------------------------------------
;HALL A connected to IC1 on pin RA2
;HALL B connected to IC2 on pin RA3
;HALL C connected to IC3 on pin RA4
;Motor Hall sensor connection(Black inline connector with 5 wires)
;Red : Connect to +5V on J9 on the board 
;Black : connect to COM(Earth symbol) on J9 on the board 
;White : connect to HA on J9 on the board 
;Brown : connect to HB on J9 on the board 
;Green : connect to HC on J9 on the board 

;----------------------------------------------------------
#ifndef ANSEL1
ANSEL1           EQU  H'0FB9'
#endif

;--------------------------------------------------
