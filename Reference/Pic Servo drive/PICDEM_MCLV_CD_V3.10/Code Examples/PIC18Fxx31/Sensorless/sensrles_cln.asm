;******************************************************************************
;
; Software License Agreement                                         
;                                                                    
; The software supplied herewith by Microchip Technology             
; Incorporated (the "Company") is intended and supplied to you, the  
; Company’s customer, for use solely and exclusively on Microchip    
; products. The software is owned by the Company and/or its supplier,
; and is protected under applicable copyright laws. All rights are   
; reserved. Any use in violation of the foregoing restrictions may   
; subject the user to criminal sanctions under applicable laws, as   
; well as to civil liability for the breach of the terms and         
; conditions of this license.                                        
;                                                                     
; THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,  
; WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED  
; TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A       
; PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,  
; IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR         
; CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.       
;
;*********************************************************************************
;-----------------------------------------------------------------
;This program is used for sensorless control of BLDC motor. 
;Theory and implementation is expalined in application note AN970
;Motor connections to the board is as follows:
;Motor power connection(White square connector with 4 wires)
;Red : connect to M1 on J9 on the board 
;White : connect to M2 on J9 on the board 
;Black : connect to M3 on J9 on the board 
;Green : connect to G on J9 on the board 
;----------------------------------------------------
;-----------------------------------------------------------------
;	Author	: Padmaraja Yedamle
;			: Home Appliance Solutions Group
;			: Microchip Technology Inc
;	Date	: June-25-2004
;
;******************************************************************
	include 	"snsrles_DATA.inc"

;--------------------------------------------------
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




;******************************************************************
#define		HURST_MOTOR	

#define	CYCLE_COUNT_MAXH	0x4E
#define	CYCLE_COUNT_MAXL	0x20
#define	MAX_FLTA_COUNT	0x20
#define	MAX_FLTB_COUNT	.20
#define	MAX_HEATSINKTEMP	.20

#define CURRENT_FAULT_INPUT	1
;----------------------------------------------
;FLAGS bits
#define	HALL_FLAG		0
#define	FLAG_FAULT		1
#define	PARAM_DISPLAY 	2
#define	POSITION_BIT 	3
#define	VELOCITY_READY 	4
#define	NEGATIVE_ERROR 	5
#define CALC_PWM		6


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

;FLAGS_SRLS bits
#define	IC_ALOWED	0
#define	SWITCH_SENSORLESS	1
#define PHASE_SHIFT_READY	2
#define	OPEN_SPEED_STEP2	3

#define	PC_COM	7

;Delay parameters
#define	DELAY_COUNT1	0x01
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



;******************************************************************
;	extern		INITIALIZE_SERIAL_PORT
;	extern		WELCOME_MESSAGE
;	extern		DISPLAY_PARAMETERS
;----------------------------------------------------------------
;	extern	DISPLAY_SPEED_REF	
;	extern	DISPLAY_SPEED_ACTH	
;	extern	DISPLAY_SPEED_ACTL	
;	extern	DISPLAY_CURRENT_Iu	
;	extern	DISPLAY_CURRENT_Iv	
;	extern	DISPLAY_CURRENT_Iw	
;;	extern	DISPLAY_MISC_PARAH
;	extern	DISPLAY_MISC_PARAL
;	extern	DISPLAY_MISC_PARA2H
;	extern	DISPLAY_MISC_PARA2L
;******************************************************************
BLDC_MOTOR_CONTROL	UDATA_ACS

TEMP			res	1
TEMP1			res	1
HALL_SENSOR_COUNT	res	1
SPEED_REFH			res	1
SPEED_REFL			res	1
FLAGS				res	1
FLAGS1				res	1
FLT_FLAGS			res	1
FLAGS_SRLS			res	1

DEBOUNCE_COUNTER	res	1
COUNTER			res	1
COUNTER1			res	1
COUNTER_SP			res	1
COUNTER_SP1			res	1
RPM_COUNTER			res	1
VELOCITY_READH		res	1
VELOCITY_READL		res	1

SPEED_REF_RPMH		res	1
SPEED_REF_RPML		res	1
SPEED_FEEDBACKH		res	1
SPEED_FEEDBACKL		res	1
SPEED_ERRORH		res	1
SPEED_ERRORL		res	1
ERROR_PWMH			res	1
ERROR_PWML			res	1

POSITION_TABLE_FWD	res	8
POSITION_TABLE_REV	res	8

CURRENT_UH			res	1
CURRENT_UL			res	1

PDC_TEMPH			res	1
PDC_TEMPL			res	1

ARG1H				res	1
ARG1L				res	1
ARG2H				res	1
ARG2L				res	1
RESH				res	1
RESL				res	1

CYCLE_COUNTH		res	1
CYCLE_COUNTL		res	1

FAULTA_COUNT		res	1
FAULTB_COUNT		res	1
PWM_CYCLE_COUNT		res	1
OPEN_HALL			res	1
TABLE_OFFSET		res	1
COUNT_OPEN_LOOPL	res	1
COUNT_OPEN_LOOPH	res	1

DISPLAY_TEMP1		res	1
DISPLAY_TEMP2		res	1

OVDCOND_TEMP		res	1
BEMF_ZC				res	1
LOCK_ROTOR_COUNT	res	1

;----------------------------------------------------------------
STARTUP	code 0x00
 	goto	Start		;Reset Vector address 
	
	CODE	0x08
	goto	ISR_HIGH	;Higher priority ISR at 0x0008

PRG_LOW	CODE	0x018
	goto	ISR_LOW		;Lower priority ISR at 0x0018
	
;****************************************************************
PROG	code
Start
;****************************************************************
	clrf	PDC_TEMPH	
	clrf	PDC_TEMPL		
	clrf	SPEED_REFH		
	clrf	FLAGS
	clrf	FLAGS1
	
	call	FIRST_ADC_INIT

WAIT_HERE
	call	LED_BLINK
	call	KEY_CHECK
	btfss	FLAGS1,KEY_PRESSED
	bra		WAIT_HERE
	call	INIT_PERPHERALS

	
	bcf		LED1
	bcf		LED2
	bcf		LED3

	movlw	0xA0
	movwf	COUNTER_SP1

	
	clrf	FLAGS
	clrf	FLAGS1
	clrf	FLT_FLAGS
	clrf	FLAGS_SRLS
	movlw	0xF0
	movwf	COUNT_OPEN_LOOPH
	movwf	COUNT_OPEN_LOOPL
		
	bsf		INTCON,PEIE	;Port interrupts enable
	bsf		INTCON,GIE	;Global interrupt enable

	bsf		FLAGS1,FWD_REV
	clrf	OVDCOND_TEMP
;*******************************************************************
MAIN_LOOP
	btfss	FLAGS_SRLS,SWITCH_SENSORLESS		
	bra		KEEP_SAME_PWM	
	btfss	FLAGS,CALC_PWM
	bra		KEEP_SAME_PWM	
	call	UPDATE_PWM
	bcf		FLAGS,CALC_PWM

KEEP_SAME_PWM	
	call	KEY_CHECK
	call	PROCESS_KEY_PRESSED

	btfsc	FLT_FLAGS,OCUR	
	call	FAULTA_PROCESS

	btfsc	FLAGS,FLAG_FAULT
	call	TOGGLE_LEDS
	
	btfss	ADCON0,GO	
	bsf		ADCON0,GO	;Set GO bit for ADC conversion start	

	goto	MAIN_LOOP
;--------------------------------------------------------------
ISR_HIGH
	btfsc	FLAGS_SRLS,IC_ALOWED
	bra		BYPASS_IC_INT
	btfsc	PIR3,IC1IF
	bra		HALL_A_HIGH
;	btfsc	PIR3,IC2QEIF
;	bra		HALL_B_HIGH
;	btfsc	PIR3,IC3DRIF
;	bra		HALL_C_HIGH
BYPASS_IC_INT
	btfsc	PIR1,TMR1IF
	bra		TIMER1_INT

	btfsc	PIR1,ADIF
	bra		AD_CONV_COMPLETE	

	btfsc	PIR3,PTIF
	bra		PWM_INTERRUPT
	
	RETFIE	FAST

;******************************************************************
AD_CONV_COMPLETE			;ADC interrupt
	
	movff	ADRESL,CURRENT_UL	;Sample A = Iu
	movff	ADRESH,CURRENT_UH

	movff	ADRESL,SPEED_REFL	;Sample B = speed ref
	movff	ADRESH,SPEED_REFH	
	
	bsf		FLAGS,CALC_PWM
	bcf		PIR1,ADIF		;ADIF flag is cleared for next interrupt
	RETFIE	FAST		

;******************************************************************
HALL_A_HIGH
	bcf		PIR3,IC1IF
	clrf	LOCK_ROTOR_COUNT
;	movf	CAP1BUFL,W
;	addwf	VELOCITY_READL,F	;Capture Timer5 count and avarage the count 
;	movf	CAP1BUFH,W
;	addwfc	VELOCITY_READH,F
;	bcf		STATUS,C
;	rrcf	VELOCITY_READH,F
;	rrcf	VELOCITY_READL,F
	call	CHECK_SEQUENCE		;UPDATE_SEQUENCE		;Update the commutation sequence 
	RETFIE	FAST

HALL_B_HIGH
	bcf		PIR3,IC2QEIF
	movf	CAP2BUFL,W
	addwf	VELOCITY_READL,F
	movf	CAP2BUFH,W
	addwfc	VELOCITY_READH,F
	bcf		STATUS,C
	rrcf	VELOCITY_READH,F
	rrcf	VELOCITY_READL,F
	call	CHECK_SEQUENCE		;UPDATE_SEQUENCE
	RETFIE	FAST


HALL_C_HIGH
	bcf		PIR3,IC3DRIF	
	movf	CAP3BUFL,W
	addwf	VELOCITY_READL,F
	movf	CAP3BUFH,W
	addwfc	VELOCITY_READH,F
	bcf		STATUS,C
	rrcf	VELOCITY_READH,F
	rrcf	VELOCITY_READL,F
	call	CHECK_SEQUENCE		;UPDATE_SEQUENCE
	RETFIE	FAST

;-----------------------------------------
CHECK_SEQUENCE
	bsf		FLAGS_SRLS,PC_COM

	btfss	FLAGS_SRLS,OPEN_SPEED_STEP2
	return
	btfsc	FLAGS_SRLS,SWITCH_SENSORLESS		
	bra		SENSERLESS_ON
	incfsz	COUNT_OPEN_LOOPH,F
	return
	bsf		FLAGS_SRLS,SWITCH_SENSORLESS
	movlw	b'00111010'
	movwf	DFLTCON
	return
SENSERLESS_ON
	bsf		FLAGS_SRLS,IC_ALOWED
	bcf		PIE3,IC1IE		;Cap1 interrupt


;Timer1 value = FFFF-Velocity reg/2 for 30 degrees 
;	bcf		STATUS,C
;	rrcf	VELOCITY_READH,F
;	rrcf	VELOCITY_READL,F	
;	bcf		STATUS,C
;	rrcf	VELOCITY_READH,F
;	rrcf	VELOCITY_READL,F	

;	bsf		STATUS,C
;	movlw	0xFF
;	subfwb	VELOCITY_READL,F
;	subfwb	VELOCITY_READH,F			;The Timer1 reload value stored in 
	movlw	0xEA
	cpfsgt	VELOCITY_READH,W
	bra		LOWER_SPEED_LEVEL1
	movlw	b'00111001'
	movwf	DFLTCON
	bsf		LED2
	bra		CONTINUE_SPEED
LOWER_SPEED_LEVEL1
	movlw	0xE4
	cpfsgt	VELOCITY_READH,W
	bra		LOWER_SPEED_LEVEL2
	movlw	b'00111101'
	movwf	DFLTCON
	bcf		LED2
	bra		CONTINUE_SPEED
LOWER_SPEED_LEVEL2
	movlw	b'00111110'
	movwf	DFLTCON
	btg		LED2
CONTINUE_SPEED
	movff	VELOCITY_READH,TMR1H
	movff	VELOCITY_READL,TMR1L

;	movlw	0xE2	;F1			;0xEC77 = 1mSec@20MHz
;	movwf	TMR1H			;0xF1F0 = 720usec@20MHz
;	movlw	0x50	;F0
;	movwf	TMR1L	

	bsf	PIE1,TMR1IE


	btg		LED3


	return

;--------
UPDATE_SEQUENCE
;	btfss	FLAGS_SRLS,PHASE_SHIFT_READY
;	return
;	bcf		FLAGS_SRLS,PHASE_SHIFT_READY

;init table again
	btfss	FLAGS1,FWD_REV
	bra		ITS_REVERSE
	lfsr	0,POSITION_TABLE_FWD
	bra		PICK_FROM_TABLE
ITS_REVERSE
	lfsr	0,POSITION_TABLE_REV
;--
PICK_FROM_TABLE
	movf	PORTA,W
	comf	WREG,W		;In forward direction, the BEMF ZC is inverted
	andlw	0x1C		;IC1/IC2/IC3
	rrncf	WREG,W
	rrncf	WREG,W
	movwf	BEMF_ZC
	movf	PLUSW0,W
	movwf	OVDCOND_TEMP	
;	bsf		LED3
	return

;-------------------------------------------------------------
TIMER1_INT
	bcf		PIR1,TMR1IF
	btfsc	FLAGS_SRLS,SWITCH_SENSORLESS		
	bra		SENSERLESS_T0INT
OPEN_LOOP_ACTIVE

	bcf		FLAGS_SRLS,IC_ALOWED	
	btfsc	FLAGS_SRLS,OPEN_SPEED_STEP2
	bra		OPEN_SPEED_STEP_2
	movlw	LOW_OL_SPEEDH		;0xD8F0 = 1.92mSec@20MHz/1000RPM
	movwf	TMR1H
	movlw	LOW_OL_SPEEDL
	movwf	TMR1L	
	call	SET_SEQUENCE_COUNT
	infsnz	COUNT_OPEN_LOOPL,F
	bsf		FLAGS_SRLS,OPEN_SPEED_STEP2
	retfie	FAST
OPEN_SPEED_STEP_2
	movlw	HIGH_OL_SPEEDH	;B0		;0xD8F0 = 1.92mSec@20MHz/1000RPM
	movwf	TMR1H
	movlw	HIGH_OL_SPEEDL	;F0
	movwf	TMR1L	
	call	SET_SEQUENCE_COUNT
	retfie	FAST


SENSERLESS_T0INT
;	bsf		FLTCONFIG,0
;	bsf		FLTCONFIG,1
;	bcf		LED3

;	bcf		LED3
	bcf		FLAGS_SRLS,IC_ALOWED	

	bsf		FLAGS_SRLS,SWITCH_SENSORLESS		
	call	UPDATE_SEQUENCE
	movff	OVDCOND_TEMP,OVDCOND

	movff	VELOCITY_READH,TMR1H
	movff	VELOCITY_READL,TMR1L

;	btfsc	FLAGS_SRLS,PHASE_SHIFT_READY
;	bra		BLOCK_IC_OVER
;	bsf		FLAGS_SRLS,PHASE_SHIFT_READY
;	call	UPDATE_SEQUENCE
;	retfie	FAST
	
BLOCK_IC_OVER
	bsf		PIE3,IC1IE		;Cap1 interrupt
	bsf		PIE1,TMR1IE

;	bsf		PIE3,IC2QEIE	;Cap2 interrupt
;	bsf		PIE3,IC3DRIE	;Cap3 interrupt
;	bcf		INTCON,TMR0IE
;	bcf		PIE1,TMR1IE
;	bcf		FLAGS_SRLS,IC_ALOWED
;	bcf		FLAGS_SRLS,PHASE_SHIFT_READY


	retfie	FAST

;******************************************************************
ISR_LOW
;	btfsc		PIR3,PTIF
;	bra		PWM_INTERRUPT
	RETFIE	FAST		

;******************************************************************
PWM_INTERRUPT
	bcf		PIR3,PTIF
	infsnz	LOCK_ROTOR_COUNT,F
	clrf	OVDCOND

CHECK_OTHER_FAULTS
	incfsz	PWM_CYCLE_COUNT,F
	bra		CHECK_FOR_FAULTS
	clrf	FAULTA_COUNT
	bra		CHECK_PARAMETER_DISPLAY	
	
CHECK_FOR_FAULTS
;	btfss	FLTCONFIG,FLTAS
	btfsc	PORTC,CURRENT_FAULT_INPUT
	bra		CHECK_PARAMETER_DISPLAY	
	incf	FAULTA_COUNT,F
	movlw	MAX_FLTA_COUNT
	cpfsgt	FAULTA_COUNT
	bra		CHECK_PARAMETER_DISPLAY	
	bcf		FLTCONFIG,FLTAMOD
	bsf		FLTCONFIG,FLTAEN
	bsf		FLT_FLAGS,OCUR
;----------------------------------Old code
CHECK_PARAMETER_DISPLAY
	retfie	FAST
	movlw	CYCLE_COUNT_MAXH
	cpfseq	CYCLE_COUNTH
	bra		NOT_YET_THERE
	movlw	CYCLE_COUNT_MAXL
	cpfsgt	CYCLE_COUNTL
	bra		NOT_YET_THERE
	bsf		FLAGS,PARAM_DISPLAY
	clrf	CYCLE_COUNTH
	clrf	CYCLE_COUNTL
	btfsc	FLT_FLAGS,OCUR
	btg		LED1
	bcf		PIR3,PTIF
	retfie	FAST
NOT_YET_THERE
	incfsz	CYCLE_COUNTL,F
	retfie	FAST
	incf	CYCLE_COUNTH,F
	bcf		PIR3,PTIF
	retfie	FAST

;******************************************************************
UPDATE_PWM
	movlw	0x40	;20
	cpfsgt	SPEED_REFH	
	bra		RESET_DUTY_CYCLE

	movlw	0x40	;0x80	;66	;40
	addwf	SPEED_REFH,F
	btfss	STATUS,C
	bra		SPEED_REF_OK
	movlw	0xFF
	movwf	SPEED_REFH
SPEED_REF_OK	
	movlw	0xFD
	cpfslt	SPEED_REFH	
	movwf	SPEED_REFH

	call	CALCULATE_TIME_60DEG

;PWM = [(MotorVoltage/DCbus voltage)*(PTPER*4)]*[SpeedRef/255] *16
;16 is the multiplication factor
	movf	SPEED_REFH,W	
	mullw	(MAIN_PWM_CONSTANT)
	swapf	PRODL,W
	andlw	0x0F
	movwf	PDC_TEMPL
	swapf	PRODH,W
	andlw	0xF0
	iorwf	PDC_TEMPL,F
	swapf	PRODH,W
	andlw	0x0F
	movwf	PDC_TEMPH
 
	bsf		PWMCON1,UDIS	;Disable the PWM buffer update

	movf	PDC_TEMPH,W
	movwf	PDC0H
	movwf	PDC1H
	movwf	PDC2H	
;	movwf	PDC3H	

	movf	PDC_TEMPL,W
	movwf	PDC0L	
	movwf	PDC1L
	movwf	PDC2L
;	movwf	PDC3L

	bcf		PWMCON1,UDIS	;Disable the PWM buffer update

	RETURN



;---------------------------------------------
RESET_DUTY_CYCLE
	clrf	PDC0H
	clrf	PDC1H
	clrf	PDC2H
;	clrf	PDC3H	
	clrf	PDC0L
	clrf	PDC1L
	clrf	PDC2L
;	clrf	PDC3L

;	call	UPDATE_SEQUENCE
	RETURN

;---------------------------------------------
CALCULATE_TIME_60DEG
	movlw	0x63
	subwf	SPEED_REFH,W	;offset
	mullw	0x9D	;39.2*4
	bcf		STATUS,C
	rrcf	PRODH,F		;/4
	rrcf	PRODL,F				
	bcf		STATUS,C
	rrcf	PRODH,F
	rrcf	PRODL,F				
;FFFF - (10,000 - 39.2 X (ADC-64h))
	bcf		STATUS,C
	movlw	0xEF
	addwfc	PRODL,F
	movlw	0xD8
	addwfc	PRODH,F
	movff	PRODH,VELOCITY_READH
	movff	PRODL,VELOCITY_READL
	return
;******************************************************************
FAULTA_PROCESS
	bsf		LED1
	bsf		LED2
	bsf		LED3

	bsf		FLAGS,FLAG_FAULT
	call	STOP_MOTOR
	bcf		FLTCONFIG,FLTAS
	bcf		FLT_FLAGS,OCUR
	bcf		FLAGS1,KEY_PRESSED
	bcf		FLAGS1,KEY_RS
	bcf		FLAGS1,RUN_STOP	
	return	

;******************************************************************
INIT_PERPHERALS

;Init ADC 

	movlw	b'00010001'
	movwf	ADCON0
	movlw	b'00010000'
	movwf	ADCON1
	movlw	b'00110010'
	movwf	ADCON2
	movlw	b'10000000'
	movwf	ADCON3
	movlw	b'00000000'  
	movwf	ADCHS
	movlw	b'00000011'	
	movwf	ANSEL0
	movlw	b'00000000'
	movwf	ANSEL1

;-----------------------------------------------------------------
;Init PWM
	
	movlw	b'00000000'
	movwf	PTCON0
	
	movlw	0x37	; 20KHz = 0xFA		;20KHz of PWM frequency			
	movwf	PTPERL	;16KHz = 0x137
					;12KHz = 0x1A0	
	movlw	0x01
	movwf	PTPERH
	
	movlw	b'01001111'	;PWM0-5 enabled in independent mode
	movwf	PWMCON0
	
	movlw	b'00000001'	;Output overides synched wrt PWM timebase
	movwf	PWMCON1
	
	movlw	b'00000000'	;
	movwf	DTCON
	
	movlw	b'00000000'	;PWM0-5 PWM Duty cycle on overide 
	movwf	OVDCOND
	
	movlw	b'00000000'	; All PWMs = 0 on init
	movwf	OVDCONS
	
	movlw	b'10110011'	;Faults on 
	movlw	b'10000000'	;Faults on 
	movwf	FLTCONFIG
	
	movlw	0x00
	movwf	SEVTCMPL
	movlw	0x00
	movwf	SEVTCMPH

	clrf	PDC0L	
	clrf	PDC1L	
	clrf	PDC2L	
;	clrf	PDC3L	
	clrf	PDC0H	
	clrf	PDC1H	
	clrf	PDC2H	
;	clrf	PDC3H	

	movlw	b'10000000'		;PWM timer ON
	movwf	PTCON1
;-----------------------------------------------------------------
;init Hall @ IC1/IC2/IC3, Timer5
	bsf		TRISA,2
	bsf		TRISA,3
	bsf		TRISA,4
	movlw	b'00000001'	;1:1
	movwf	T5CON
;	movlw	b'01001000'	;Cap1/2/3-capture every input state change
	movlw	b'00000010'
	movwf	CAP1CON
;	movlw	b'01001000'
	movlw	b'00000000'
	movwf	CAP2CON
;	movlw	b'01001000'
	movlw	b'00000000'
	movwf	CAP3CON
	movlw	b'00111010'
	movwf	DFLTCON
	movlw	b'00000000'	;Disable QEI
	movwf	QEICON

;-----------------------------------------------------------------
;init PORTC
	movlw	TRISC_VALUE
	movwf	TRISC

;-----------------------------------------------------------------
;init Timer1
	movlw	b'10000001'
	movwf	T1CON
	movlw	0xC3		;0xC350 = 10mSec@20MHz
	movwf	TMR1H
	movlw	0xFF
	movwf	TMR1L
;-----------------------------------------------------------------
LOAD_SEQUENCE_TABLE
 
;Forward sequence
	movlw	POSITION01
	movwf	POSITION_TABLE_FWD
	movlw	POSITION61
	movwf	POSITION_TABLE_FWD+1
	movlw	POSITION51
	movwf	POSITION_TABLE_FWD+2
	movlw	POSITION41
	movwf	POSITION_TABLE_FWD+3
	movlw	POSITION31
	movwf	POSITION_TABLE_FWD+4
	movlw	POSITION21
	movwf	POSITION_TABLE_FWD+5
	movlw	POSITION11
	movwf	POSITION_TABLE_FWD+6
	movlw	POSITION71
	movwf	POSITION_TABLE_FWD+7
;Reverse sequence
	movlw	POSITION71
	movwf	POSITION_TABLE_REV
	movlw	POSITION21
	movwf	POSITION_TABLE_REV+1
	movlw	POSITION41
	movwf	POSITION_TABLE_REV+2
	movlw	POSITION61
	movwf	POSITION_TABLE_REV+3
	movlw	POSITION11
	movwf	POSITION_TABLE_REV+4
	movlw	POSITION31
	movwf	POSITION_TABLE_REV+5
	movlw	POSITION51
	movwf	POSITION_TABLE_REV+6
	movlw	POSITION01
	movwf	POSITION_TABLE_REV+7
;-----------------------------------------------------------------
	clrf	SPEED_REFH
	clrf	CURRENT_UH	
	clrf	CURRENT_UL	
	clrf	FLAGS_SRLS
;-----------------------------------------------------------------
INITIALIZE_SERIAL_PORT

	movlw	0x81		;Baudrate = 9600
	movwf	SPBRG
	
	movlw	0x24		;8-bit transmission;Enable Transmission;	
	movwf	TXSTA		;Asynchronous mode with High speed transmission
	
	movlw	0x90		;Enable the serial port
	movwf	RCSTA		;with 8-bit continuous reception

	bcf	TRISC,6
	bsf	TRISC,7

;------------------------------------------------------------------------

;init interrupts
	bsf	PIE1,ADIE	;AD Converter over Interrupt enable

;	bsf	PIE3,IC1IE		;Cap1 interrupt
;	bsf	PIE3,IC2QEIE	;Cap2 interrupt
;	bsf	PIE3,IC3DRIE	;Cap3 interrupt
	bsf	PIE3,PTIE		;PWM interrupt
;	bsf	INTCON,TMR0IE
	bsf	PIE1,TMR1IE

;	bsf	INTCON,RBIE 

	movlw	0x093		;Power ON reset status bit/Brownout reset status bit
	movwf	RCON		;and Instruction flag bits are set
				;Priority level on Interrupots enabled

;	bsf	INTCON,PEIE	;Port interrupts enable
;	bsf	INTCON,GIE	;Global interrupt enable


	RETURN
	

;*******************************************************************************
;This routine checks for the keys status. 2 keys are checked, Run/Stop and 
;Forward(FWD)/Reverse(REV)  
;*******************************************************************************
KEY_CHECK
	btfsc	KEY_PORT,RUN_STOP_KEY			;Is key pressed "RUN/STOP"?
	goto	CHECK_FWD_REV_KEY
	btfsc	FLAGS1,DEBOUNCE
	return
	call	KEY_DEBOUNCE
	btfss	FLAGS1,DEBOUNCE
	return
	bsf		FLAGS1,KEY_RS
	return
	
CHECK_FWD_REV_KEY
	btfsc	KEY_PORT,FWD_REV_KEY			;Is key pressed "RUN/STOP"?
	goto	SET_KEYS
	btfsc	FLAGS1,DEBOUNCE
	return
	call	KEY_DEBOUNCE
	btfss	FLAGS1,DEBOUNCE
	return
	bsf		FLAGS1,KEY_FR
	return

SET_KEYS
	btfss	FLAGS1,DEBOUNCE
	return
	bcf		FLAGS1,DEBOUNCE
	bsf		FLAGS1,KEY_PRESSED	
	btfss	FLAGS1,KEY_RS
	bra		ITS_FWD_REV	
	btg		FLAGS1,RUN_STOP
	return
ITS_FWD_REV	
	btg		FLAGS1,FWD_REV
	return


;*******************************************************************************
KEY_DEBOUNCE
	decfsz	DEBOUNCE_COUNTER,F
	return
	bsf		FLAGS1,DEBOUNCE
	movlw	DEBOUNCE_COUNT
	movwf	DEBOUNCE_COUNTER
	return
;*******************************************************************************
PROCESS_KEY_PRESSED
	btfss	FLAGS1,KEY_PRESSED
	return
	btfss	FLAGS1,KEY_RS
	goto	CHECK_FWD_REV
	btfss	FLAGS1,RUN_STOP	
	goto	STOP_MOTOR_NOW
	call	RUN_MOTOR_AGAIN
	bcf		FLAGS1,KEY_PRESSED
	bcf		FLAGS1,KEY_RS

	bsf		LED_PORT,RUN_STOP_LED	
	return

STOP_MOTOR_NOW
	call	STOP_MOTOR		;STOP motor
	bcf		FLAGS1,KEY_PRESSED
	bcf		FLAGS1,KEY_RS
	bcf		LED_PORT,RUN_STOP_LED	
	return

CHECK_FWD_REV
	btfss	FLAGS1,KEY_FR
	return

	btg		LED_PORT,FWD_REV_LED	

	bcf		LED_PORT,RUN_STOP_LED	
	call	STOP_MOTOR

	call	DELAY

	call	RUN_MOTOR_AGAIN
	bcf		FLAGS1,KEY_PRESSED
	bcf		FLAGS1,KEY_FR
	bsf		LED_PORT,RUN_STOP_LED	
	return

;*******************************************************************************
;This routine stops the motor by driving the PWMs to 0% duty cycle.
;*******************************************************************************
STOP_MOTOR
	bcf		PIE1,ADIE
	bcf		PIE3,IC1IE		;Velocity capture
	bcf		PIE3,IC3DRIE	;Diection change
	bcf		PIE3,IC2QEIE	;QEI interrupt
	bcf		PIE3,PTIE		;PWM interrupt
	bcf		PIE1,TMR1IE

	clrf	OVDCOND		;STOP motor
	clrf	PDC0H
	clrf	PDC1H
	clrf	PDC2H	
;	clrf	PDC3H	
	clrf	PDC0L	
	clrf	PDC1L
	clrf	PDC2L
;	clrf	PDC3L
	bcf		FLAGS,CALC_PWM
	clrf	SPEED_REFH
	return
;*******************************************************************************
;This routine starts motor from previous stop with motor parameters initialized
;*******************************************************************************
RUN_MOTOR_AGAIN
	bsf		FLAGS1,RUN_STOP	
;	bcf		FLAGS,FLAG_FAULT
;-----------------------------------------------------------------
	clrf	SPEED_REFH
	clrf	CURRENT_UH	
	clrf	CURRENT_UL	

;-----------------------------------------------------------------


	bsf		PIE1,ADIE
	bsf		PIE3,IC1IE		;Velocity capture
;	bsf		PIE3,IC3DRIE	;Diection change
;	bsf		PIE3,IC2QEIE	;QEI interrupt
	bsf		PIE1,TMR1IE		;enable T1 interrupt
	bsf		PIE3,PTIE		;PWM interrupt
;	bcf		IPR3,PTIP		;PWM int low priority
	
;	bcf		FLAGS,POSITION_BIT
	clrf	FLAGS

	movlw	b'10000000'	;Faults on 
	movwf	FLTCONFIG
	bcf		FLT_FLAGS,OCUR
	bcf		FLT_FLAGS,OVOLT
	bcf		FLT_FLAGS,OTEMP

	clrf	FLAGS_SRLS
;	clrf	COUNT_OPEN_LOOPL
;	clrf	COUNT_OPEN_LOOPH
;	call	UPDATE_SEQUENCE

	clrf	OVDCOND_TEMP

	movlw	0xF0
	movwf	COUNT_OPEN_LOOPH
	movwf	COUNT_OPEN_LOOPL
	return


;---------------------------------
TOGGLE_LEDS

	incfsz	COUNTER_SP,F
	return
	incfsz	COUNTER_SP1,F
	return
	btg		LED1
	btg		LED2
	btg		LED3

	return
;------------------------------------------
FIRST_ADC_INIT
	movlw	b'00100111'
	movwf	ADCON0
	movlw	b'00000000'
	movwf	ADCON1
	movlw	b'00110010'
	movwf	ADCON2
	movlw	b'00000000'
	movwf	ADCON3
	movlw	b'00000000' 
	movwf	ADCHS
	movlw	b'00000010'	
	movwf	ANSEL0
	movlw	b'00000000'
	movwf	ANSEL1


	movlw	TRISC_VALUE
	movwf	TRISC

	return
	
LED_BLINK
	bcf		LED1
	bcf		LED2
	bcf		LED3

	movlw	0X40
	cpfsgt	ADRESH	
	return
	bsf		LED1
	movlw	0x80
	cpfsgt	ADRESH
	return
	bsf		LED2
	movlw	0XD0
	cpfsgt	ADRESH	
	return
	bsf		LED3
	return

;*******************************************************************************
SET_SEQUENCE_COUNT
	call	OPEN_LOOP_PWM

	btfss	FLAGS1,FWD_REV
	bra		RUN_REVERSE

	movlw	0x5					;Check for the last value on the table
	cpfslt	TABLE_OFFSET
	bra		CLEAR_OFFSET
	incf	TABLE_OFFSET,F		;Increment offset1
	bra		LOAD_SEQUENCE_COUNT
CLEAR_OFFSET
	clrf	TABLE_OFFSET
	bra		LOAD_SEQUENCE_COUNT
RUN_REVERSE
	decf	TABLE_OFFSET,F		;Increment offset1
	btfsc	STATUS,C
	bra		LOAD_SEQUENCE_COUNT
LOAD_OFFSET
	movlw	0x5	
	movwf	TABLE_OFFSET

LOAD_SEQUENCE_COUNT
	movf	TABLE_OFFSET,W
	btfss	STATUS,Z
	bra		NEXT_TEST1
;	btfss	FLAGS1,FWD_REV
;	bra		OPEN_STP_0_REV
	movlw	POSITION1
	movwf	OVDCOND
	movwf	OVDCOND_TEMP
	btg		LED1
	return
;OPEN_STP_0_REV
;	movlw	POSITION5
;	movwf	OVDCOND
;	movwf	OVDCOND_TEMP
;	btg		LED2
;	return
;---
NEXT_TEST1
	movf	TABLE_OFFSET,W
	sublw	0x1
	btfss	STATUS,Z
	bra		NEXT_TEST2
;	btfss	FLAGS1,FWD_REV
;	bra		OPEN_STP_1_REV
	movlw	POSITION3
	movwf	OVDCOND
	movwf	OVDCOND_TEMP
	btg		LED1
	return
;OPEN_STP_1_REV
;	movlw	POSITION4
;	movwf	OVDCOND
;	movwf	OVDCOND_TEMP
;	btg		LED2
;	return
;---
NEXT_TEST2
	movf	TABLE_OFFSET,W
	sublw	0x2
	btfss	STATUS,Z
	bra		NEXT_TEST3
;	btfss	FLAGS1,FWD_REV
;	bra		OPEN_STP_2_REV
	movlw	POSITION2
	movwf	OVDCOND
	movwf	OVDCOND_TEMP
	btg		LED1
	return
;OPEN_STP_2_REV
;	movlw	POSITION6
;	movwf	OVDCOND
;	movwf	OVDCOND_TEMP
;	btg		LED2
;	return
;---
NEXT_TEST3
	movf	TABLE_OFFSET,W
	sublw	0x3
	btfss	STATUS,Z
	bra		NEXT_TEST4
;	btfss	FLAGS1,FWD_REV
;	bra		OPEN_STP_3_REV
	movlw	POSITION6
	movwf	OVDCOND
	movwf	OVDCOND_TEMP
	btg		LED1
	return
;OPEN_STP_3_REV
;	movlw	POSITION2
;	movwf	OVDCOND
;	movwf	OVDCOND_TEMP
;	btg		LED2
;	return
;---
NEXT_TEST4
	movf	TABLE_OFFSET,W
	sublw	0x4
	btfss	STATUS,Z
	bra		NEXT_TEST5
;	btfss	FLAGS1,FWD_REV
;	bra		OPEN_STP_4_REV
	movlw	POSITION4
	movwf	OVDCOND
	movwf	OVDCOND_TEMP
	btg		LED1
	return
;OPEN_STP_4_REV
;	movlw	POSITION3
;	movwf	OVDCOND
;	movwf	OVDCOND_TEMP
;	btg		LED2
;	return
;---
NEXT_TEST5
	movf	TABLE_OFFSET,W
	sublw	0x5
	btfss	STATUS,Z
	return
;	btfss	FLAGS1,FWD_REV
;	bra		OPEN_STP_5_REV
	movlw	POSITION5
	movwf	OVDCOND
	movwf	OVDCOND_TEMP
	btg		LED1
	return
;OPEN_STP_5_REV
;	movlw	POSITION1
;	movwf	OVDCOND
;	movwf	OVDCOND_TEMP
;	btg		LED2
;	return
	

;*******************************************************************************
OPEN_LOOP_PWM

	movlw	0x64
	cpfslt	SPEED_REFH	
	bra		AAAAA
	clrf	PDC_TEMPL	
	clrf	PDC_TEMPH
	bra		BBBBB	
AAAAA
	movlw	0xA5	
	movwf	PDC_TEMPL
	movlw	0x03	
	movwf	PDC_TEMPH
BBBBB
	bsf		PWMCON1,UDIS	;Disable the PWM buffer update

	movf	PDC_TEMPH,W
	movwf	PDC0H
	movwf	PDC1H
	movwf	PDC2H	
;	movwf	PDC3H	

	movf	PDC_TEMPL,W
	movwf	PDC0L	
	movwf	PDC1L
	movwf	PDC2L
;	movwf	PDC3L

	bcf		PWMCON1,UDIS	;Disable the PWM buffer update

	RETURN


;*******************************************************************************

SEND_DATA_TO_PC


	bcf		FLAGS_SRLS,PC_COM
	movlw	0xA	;new line
	call	SEND_BYTE_FROM_WREG
	movlw	0xD	;carriage return
	call	SEND_BYTE_FROM_WREG

	movf	OVDCOND_TEMP,W	
	addlw	0x41
	call	SEND_BYTE_FROM_WREG
	movf	BEMF_ZC,W	
	addlw	0x41
	call	SEND_BYTE_FROM_WREG

	return


;	movff	VELOCITY_READH,DISPLAY_TEMP1
	movff	OVDCOND_TEMP,DISPLAY_TEMP1
	call	DISPLAY_DIGITS
;	movff	VELOCITY_READL,DISPLAY_TEMP1
	movff	TABLE_OFFSET,DISPLAY_TEMP1
	call	DISPLAY_DIGITS

	return

;---------------------------------------------------------
DISPLAY_DIGITS
	movf	DISPLAY_TEMP1,W
	andlw	0xF0
	swapf	WREG,W
	addlw	0x30	
	call	CHECK_39
	call	SEND_BYTE_FROM_WREG
	movf	DISPLAY_TEMP1,W
	andlw	0x0F
	addlw	0x30	
	call	CHECK_39
	call	SEND_BYTE_FROM_WREG
	RETURN
;------------------------------
CHECK_39
	movwf	DISPLAY_TEMP2
	movlw	0x39
	cpfsgt	DISPLAY_TEMP2
	bra		LESS_39
	movf	DISPLAY_TEMP2,W
	addlw	0x7
	return
LESS_39
	movf	DISPLAY_TEMP2,W
	return
;------------------------------
SEND_BYTE_FROM_WREG
	btfss	PIR1,TXIF
	goto	SEND_BYTE_FROM_WREG
	movwf	TXREG
	return

;*******************************************************************************
;Delay routine.
;*******************************************************************************
DELAY
	movlw	DELAY_COUNT1
	movwf	COUNTER
dec_count	
	movlw	DELAY_COUNT2
	movwf	COUNTER1
dec_count1
	decfsz	COUNTER1,F
	goto	dec_count1
	decfsz	COUNTER,F
	goto	dec_count
	clrf	COUNTER
	clrf	COUNTER1
	return		





	END

