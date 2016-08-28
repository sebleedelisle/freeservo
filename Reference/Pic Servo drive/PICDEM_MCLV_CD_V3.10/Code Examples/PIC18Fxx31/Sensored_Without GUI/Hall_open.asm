;-----------------------------------------------------------------
;This program is used for Motor control training using PIC18Fxx31
;On BLDC motor control board - Derive the commutation sequence and run the motor!!!!
;Make sure that jumpers and potentiometers are configured as follows.
;	J2(2 pin)  - shorted. 
;	J15(3 pin) - shorted between pin 2 & 3(short link towards the crystal oscillator)
;	J7, J11 and J13 - shorted between pin 1 & 2(short link towards the ICD2 connector)
;	J8, J12, J14, J16 and J17 open
;	Keep Potentiometer REF(R14) completely turned counter clockwise
;	Keep Potentiometer R60 completely turned clockwise
;
;-----------------------------------------------------------------
;Padmaraja Yedamale:Home Appliance Solutions Group: Microchip Technology Inc
;V1.0
;******************************************************************
;******************************************************************
	include 	"BLDC_DATA.inc"
;--------------------------------------------------
;******************************************************************
	extern	INITIALIZE_SERIAL_PORT
	extern	WELCOME_MESSAGE
	extern	DISPLAY_SPEED_REF
	extern	DISPLAY_CURRENT_Iu
	extern	DISPLAY_PARAMETERS
	extern	DISPALY_CAP1_H
	extern	DISPALY_CAP1_L
	extern	DISPALY_CAP2_H
	extern	DISPALY_CAP2_L
	extern	DISPALY_CAP3_H
	extern	DISPALY_CAP3_L
	
;******************************************************************
BLDC_MOTOR_CONTROL	UDATA_ACS

SPEED_REFH			res	1
SPEED_REFL			res	1
FLAGS				res	1
FLAGS1				res	1
DEBOUNCE_COUNTER	res	1
COUNTER				res	1
COUNTER1			res	1

POSITION_TABLE_FWD	res	8
POSITION_TABLE_REV	res	8

MOTOR_CURRENTH		res	1
MOTOR_CURRENTL		res	1

PDC_TEMPH			res	1
PDC_TEMPL			res	1


CYCLE_COUNTH		res	1
CYCLE_COUNTL		res	1

FAULTA_COUNT		res	1
PWM_CYCLE_COUNT		res	1
FLT_FLAGS			res	1
TIMER0_COUNT		res	1
DISPLAY_COUNT		res	1

CAP1H		res	1
CAP1L		res	1
CAP2H		res	1
CAP2L		res	1
CAP3H		res	1
CAP3L		res	1



;----------------------------------------------------------------
STARTUP	code 0x00
	goto	Start		;Reset Vector address 
	
	CODE	0x08
	goto	ISR_HIGH	;Higher priority ISR at 0x0008

PRG_LOW	CODE	0x018
	goto	ISR_LOW		;Lower priority ISR at 0x0018
	
;****************************************************************
PROG	code

;******************************************************************
;****************************************************************
;****************************************************************
;****************************************************************
;****************************************************************
;****************************************************************
Start
;****************************************************************
	clrf	SPEED_REFH		
	clrf	FLAGS
	clrf	FLAGS1
;-----------------------------------------------------------------
	movlw	TRISC_VALUE		; Initialize PortC
	movwf	TRISC
;-----------------------------------------------------------------
	call	INITIALIZE_ADC	;Initialize ADC 
	bsf		INTCON,PEIE		;Port interrupts enable
	bsf		INTCON,GIE		;Global interrupt enable

WAIT_HERE					;Control waits here until a key is pressed	
	call	LED_ON_OFF		;Potentiometer level is deispalyed on LED1,2 and 3
	call	KEY_CHECK
	bsf		ADCON0,GO
	btfss	FLAGS1,KEY_PRESSED
	bra		WAIT_HERE
;---------------------
	call	INITIALIZE_INPUT_CAPTURE	;Input cature module initialized here
	call	INITIALIZE_PCPWM			;PCPWM module initialized here		
	call	LOAD_SEQUENCE_TABLE			;Energizing Sequence table is initialized
	call	INITIALIZE_SERIAL_PORT		;Serial port is initialized to display few key parameters on Hyper terminal
	call	WELCOME_MESSAGE				;Send a test message to PC

;*******************************************************************************************
;Control keeps revolving in the main loop
;Tasks done in MAIN_LOOP
;PWM duty cycle updated after every ADC conversion
;Key activity is checked
;If a key is pressed action is taken based on the key pressed
;----------------------------------------------------------------
MAIN_LOOP
	btfss	FLAGS,CALC_PWM			;Is ADc result ready?
	bra		KEEP_SAME_PWM	
	call	UPDATE_PWM				;Update PWM duty cycle
	bcf		FLAGS,CALC_PWM
	call	DISPLAY_ROUTINE			;Dump parameters on to Hyper terminal
KEEP_SAME_PWM	
	call	KEY_CHECK				;Check for Key activity, take care of debounce
	call	PROCESS_KEY_PRESSED		;If a key is pressed, take action accordingly

	btfss	ADCON0,GO	
	bsf		ADCON0,GO				;Set GO bit for ADC conversion start	

	goto	MAIN_LOOP

;*******************************************************************************************
;High priority Intterrupt service routines
;ADC, Input captures(hall sensors) and PWM interrupts are serviced here 
;--------------------------------------------------------------
ISR_HIGH
	btfsc	PIR3,IC1IF			;IC1/IC2 or IC3 interrupt??
	bra		HALL_A_HIGH
	btfsc	PIR3,IC2QEIF
	bra		HALL_B_HIGH
	btfsc	PIR3,IC3DRIF
	bra		HALL_C_HIGH

	btfsc	PIR1,ADIF			;ADC conversion ready?
	bra		ADC_INT	

	btfsc	PIR3,PTIF			;PWM interrupt?
	bra		PWM_INTERRUPT

	RETFIE	FAST
;******************************************************************
ADC_INT	;ADC interrupt
	
	movff	ADRESL,MOTOR_CURRENTL	;Sample A = Iu
	movff	ADRESH,MOTOR_CURRENTH

	movff	ADRESL,SPEED_REFL		;Sample B = speed ref
	movff	ADRESH,SPEED_REFH	
	
	bcf		PIR1,ADIF				;ADIF flag is cleared for next interrupt
	
	bsf		FLAGS,CALC_PWM			;Set flag to indicate ADC conversion complete	

	RETFIE	FAST		
;******************************************************************
;******************************************************************
;Hall sensor interrupts on IC1,IC2 and IC3
;******************************************************************
HALL_A_HIGH
	call	UPDATE_SEQUENCE			;Update the winding energizing sequence based on IC state
	bcf		LED1					;Display the Hall sensor 1 status on LED1
	btfsc	PORTA,2
	bsf		LED1	
	bcf		PIR3,IC1IF

	movff	CAP1BUFL,CAP1L
	movff	CAP1BUFH,CAP1H

	RETFIE	FAST

HALL_B_HIGH
	call	UPDATE_SEQUENCE			;Update the winding energizing sequence based on IC state
	bcf		LED2					;Display the Hall sensor 2 status on LED2
	btfsc	PORTA,3
	bsf		LED2	
	bcf		PIR3,IC2QEIF

	movff	CAP2BUFL,CAP2L
	movff	CAP2BUFH,CAP2H

	RETFIE	FAST


HALL_C_HIGH
	call	UPDATE_SEQUENCE			;Update the winding energizing sequence based on IC state
	bcf		LED3					;Display the Hall sensor 3 status on LED3
	btfsc	PORTA,4
	bsf		LED3	
	bcf		PIR3,IC3DRIF	

	movff	CAP3BUFL,CAP3L
	movff	CAP3BUFH,CAP3H

	RETFIE	FAST
;******************************************************************
;Lower ISR  not used in this code example
ISR_LOW

	RETFIE	FAST		
	
;******************************************************************
;Update the winding energizing sequence
UPDATE_SEQUENCE
	btfss	FLAGS1,FWD_REV			;Check the direction command
	bra		ITS_REVERSE				
	lfsr	0,POSITION_TABLE_FWD	;If it is forward, point FSR0 to the begining of Forward table 
	bra		PICK_FROM_TABLE
ITS_REVERSE
	lfsr	0,POSITION_TABLE_REV	;If it is reverse, point FSR0 to the begining of Reverse table 

PICK_FROM_TABLE
	call	PICK_ENERGIZING_SEQUENCE	;Pick up the energizing value according to IC1/IC2/IC3 offset

	return
;****************************************************************
;This routine picks up the correct value from the table according to he direction and offset
;----------------------------------------------------
PICK_ENERGIZING_SEQUENCE
	movf	PORTA,W					;Load porta value to Wreg
	andlw	0x1C					;Mask other bits, IC1/IC2/IC3 are on PORTA pin 2,3,4
	rrncf	WREG,W					
	rrncf	WREG,W					;Rotate twice to move the bits to LSB
	movf	PLUSW0,W				;Pick the value from table with the offset
	movwf	OVDCOND					;Load it to Override register. 
	return							;Override register ecides which PWM to allow and which to block according to the energizing sequence.

;******************************************************************
;This routine updates the PWM duty cycle according to the input speed referance, Motor voltage and DC bus voltage
;PWM duty cycle = [(MotorVoltage/DCbus voltage)*(PTPER*4)]*[SpeedRef/255] *16
;16 is the multiplication factor and 255 is max value of speed referance(8 MSBs are taken from ADC conversion)

UPDATE_PWM
	movlw	0x20					;Lower side cut off for the speed referance
	cpfsgt	SPEED_REFH	
	bra		RESET_DUTY_CYCLE
	
	movlw	0x40					;Additional offset given to the speed ref
	addwf	SPEED_REFH,F
	btfss	STATUS,C
	bra		SPEED_REF_OK
	movlw	0xFF					;Limit speed ref to 8 bits
	movwf	SPEED_REFH
SPEED_REF_OK	
	movlw	0xFD
	cpfslt	SPEED_REFH	
	movwf	SPEED_REFH

;PWM = [(MotorVoltage/DCbus voltage)*(PTPER*4)]*[SpeedRef/255] *16
;16 is the multiplication factor
	movf	SPEED_REFH,W	
	mullw	(MAIN_PWM_CONSTANT)		;MAIN_PWM_CONSTANT=(MOTOR_VOLTAGE*PTPER_VALUE*4*d'16')/(DC_INPUT_VOLTAGE*d'256')
	swapf	PRODL,W					;
	andlw	0x0F
	movwf	PDC_TEMPL
	swapf	PRODH,W
	andlw	0xF0
	iorwf	PDC_TEMPL,F
	swapf	PRODH,W
	andlw	0x0F
	movwf	PDC_TEMPH				;Take 12 MS bits from the product and load to duty cycle registers
 
	bsf		PWMCON1,UDIS			;Disable the PWM buffer update

	movf	PDC_TEMPH,W				;Same duty cycle values are loaded to all PDC registers
	movwf	PDC0H
	movwf	PDC1H
	movwf	PDC2H	
;	movwf	PDC3H	

	movf	PDC_TEMPL,W
	movwf	PDC0L	
	movwf	PDC1L
	movwf	PDC2L
;	movwf	PDC3L

	bcf		PWMCON1,UDIS			;Enable the PWM buffer update
	RETURN
;---------------------------------------------
;Output 0% duty cycle at starting and while reversing the direction

RESET_DUTY_CYCLE
	clrf	PDC0H
	clrf	PDC1H
	clrf	PDC2H
;	clrf	PDC3H	
	clrf	PDC0L
	clrf	PDC1L
	clrf	PDC2L
;	clrf	PDC3L

	call	UPDATE_SEQUENCE		;Update the energizing sequence at startup
	return
;******************************************************************
;This routine checks for the Faults. Also sets the flag for parameter display
;Over current (Fault A) is initialized in cycle-by-cycle mode. 
;If these faults occur very frequently, the mode is changed to catestriphic mode and PWMs are shut down
;The occurence of these faults are checked every PWM interrupt with a limit(MAX_FLTA_COUNT) and if it exeeds the limit
;in 256 PWM cycles, the mode is changed to catestrophic.
;LED1 is blinked at fixed rate, if the Overcurrent fault is detected in catestrophic mode 

PWM_INTERRUPT
	incfsz	PWM_CYCLE_COUNT,F
	bra		CHECK_FOR_FAULTS
	clrf	FAULTA_COUNT
	bra		CHECK_PARAMETER_DISPLAY	
	
CHECK_FOR_FAULTS
	btfss	FLTCONFIG,FLTAS
	bra		CHECK_PARAMETER_DISPLAY	
	incf	FAULTA_COUNT,F
	movlw	MAX_FLTA_COUNT
	cpfsgt	FAULTA_COUNT
	bra		CHECK_PARAMETER_DISPLAY	
	bcf		FLTCONFIG,FLTAMOD
	bsf		FLT_FLAGS,OCUR
CHECK_PARAMETER_DISPLAY
	movlw	CYCLE_COUNT_MAXH
	cpfseq	CYCLE_COUNTH
	bra		NOT_YET_THERE
	movlw	CYCLE_COUNT_MAXL
	cpfsgt	CYCLE_COUNTL
	bra		NOT_YET_THERE
;	bsf		FLAGS,PARAM_DISPLAY
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

;******************************************************************
;******************************************************************
;ADC is initialized here
INITIALIZE_ADC
	movlw	ADCON0_VALUE			;ADC 
	movwf	ADCON0
	movlw	ADCON1_VALUE
	movwf	ADCON1
	movlw	ADCON2_VALUE
	movwf	ADCON2
	movlw	ADCON3_VALUE
	movwf	ADCON3
	movlw	ADCHS_VALUE
	movwf	ADCHS
	movlw	ANSEL0_VALUE
	movwf	ANSEL0
	movlw	ANSEL1_VALUE
	movwf	ANSEL1
	bsf		ADCON0,GO
	bsf	PIE1,ADIE	;AD Converter Interrupt enable
	RETURN
	

;******************************************************************
;******************************************************************
;Input capture initialization
INITIALIZE_INPUT_CAPTURE
;init Hall @ IC1/IC2/IC3
	bsf		TRISA,2				;TrisA 2,3,4 configured as inputs
	bsf		TRISA,3
	bsf		TRISA,4
	movlw	b'00011001'
	movwf	T5CON
	movlw	b'01001000'	;Cap1/2/3-capture every input state change
	movwf	CAP1CON
	movlw	b'01001000'
	movwf	CAP2CON
	movlw	b'01001000'
	movwf	CAP3CON
	movlw	b'00000000'
	movwf	DFLTCON
	movlw	b'00000000'	;Disable QEI
	movwf	QEICON	

;init interrupts
	bsf	PIE3,IC1IE		;Cap1 interrupt
	bsf	PIE3,IC2QEIE	;Cap2 interrupt
	bsf	PIE3,IC3DRIE	;Cap3 interrupt


	RETURN
	
;******************************************************************
;******************************************************************
;PWM initialization
;-----------------------------------------------------------------
INITIALIZE_PCPWM

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
	
	movlw	b'00000000'	;No dead time
	movwf	DTCON
	
	movlw	b'00000000'	;PWM0-5 PWM Duty cycle on overide 
	movwf	OVDCOND
	
	movlw	b'00000000'	; All PWMs = 0 on init
	movwf	OVDCONS
	
	movlw	b'10000011'	;FaultA ON in cycle-by-cycle mode
	movwf	FLTCONFIG
	
	movlw	0x00			;No special event trigger
	movwf	SEVTCMPL
	movlw	0x00
	movwf	SEVTCMPH

	clrf	PDC0L			;All PWM duty cycles cleared
	clrf	PDC1L	
	clrf	PDC2L	
;	clrf	PDC3L	
	clrf	PDC0H	
	clrf	PDC1H	
	clrf	PDC2H	
;	clrf	PDC3H	

	movlw	b'10000000'		;PWM timer ON
	movwf	PTCON1


	return

;******************************************************************
;******************************************************************
;This routine loads the switching sequence to the RAM locations
;-----------------------------------------------------------------
LOAD_SEQUENCE_TABLE
;Forward sequence
	movlw	POSITION0
	movwf	POSITION_TABLE_FWD
	movlw	POSITION1
	movwf	POSITION_TABLE_FWD+1
	movlw	POSITION2
	movwf	POSITION_TABLE_FWD+2
	movlw	POSITION3
	movwf	POSITION_TABLE_FWD+3
	movlw	POSITION4
	movwf	POSITION_TABLE_FWD+4
	movlw	POSITION5
	movwf	POSITION_TABLE_FWD+5
	movlw	POSITION6
	movwf	POSITION_TABLE_FWD+6
	movlw	POSITION7
	movwf	POSITION_TABLE_FWD+7
;Reverse sequence
	movlw	POSITION7
	movwf	POSITION_TABLE_REV
	movlw	POSITION6
	movwf	POSITION_TABLE_REV+1
	movlw	POSITION5
	movwf	POSITION_TABLE_REV+2
	movlw	POSITION4
	movwf	POSITION_TABLE_REV+3
	movlw	POSITION3
	movwf	POSITION_TABLE_REV+4
	movlw	POSITION2
	movwf	POSITION_TABLE_REV+5
	movlw	POSITION1
	movwf	POSITION_TABLE_REV+6
	movlw	POSITION0
	movwf	POSITION_TABLE_REV+7	

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
	call	KEY_DEBOUNCE					;Debounce delay
	btfss	FLAGS1,DEBOUNCE
	return
	bsf		FLAGS1,KEY_RS					;Set a flag indicating Run/Stop key is pressed
	return
	
CHECK_FWD_REV_KEY
	btfsc	KEY_PORT,FWD_REV_KEY			;Is key pressed "RUN/STOP"?
	goto	SET_KEYS
	btfsc	FLAGS1,DEBOUNCE
	return
	call	KEY_DEBOUNCE					;Debounce delay
	btfss	FLAGS1,DEBOUNCE
	return
	bsf		FLAGS1,KEY_FR					;Set a flag indicating FWD/REV key is pressed
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
;Key debounce routine; checks keys being pressed for at least for a minimum time(DEBOUNCE_COUNT)
KEY_DEBOUNCE
	decfsz	DEBOUNCE_COUNTER,F
	return
	bsf		FLAGS1,DEBOUNCE
	movlw	DEBOUNCE_COUNT
	movwf	DEBOUNCE_COUNTER
	return

;*******************************************************************************
;This routine takes action for the key pressed. 
;*******************************************************************************
PROCESS_KEY_PRESSED
	btfss	FLAGS1,KEY_PRESSED			;Is there a key pressed waiting or service
	return								;No
	btfss	FLAGS1,KEY_RS				;Is it RUN/STOP
	goto	CHECK_FWD_REV
	btfss	FLAGS1,RUN_STOP					
	goto	STOP_MOTOR_NOW
	call	RUN_MOTOR_AGAIN				;If it is RUN,re initialize the states to run the motor
	bcf		FLAGS1,KEY_PRESSED
	bcf		FLAGS1,KEY_RS

	bsf		LED_PORT,RUN_STOP_LED		;Turn the RUN LED ON
	return

STOP_MOTOR_NOW
	call	STOP_MOTOR					;STOP motor
	bcf		FLAGS1,KEY_PRESSED
	bcf		FLAGS1,KEY_RS
	bcf		LED_PORT,RUN_STOP_LED		;Turn the RUN LED OFF
	return

CHECK_FWD_REV							;If the key pressed is FWD/REV
	btfss	FLAGS1,KEY_FR
	return

	btg		LED_PORT,FWD_REV_LED		;Toggle the LED

	bcf		LED_PORT,RUN_STOP_LED		
	call	STOP_MOTOR					;Stop the motor before reversing the state

	call	DELAY						;Call a long delay to make sure the motor reversed.
	call	DELAY
	call	DELAY
	call	DELAY

	call	RUN_MOTOR_AGAIN				;Re-initialize the states to run the motor in opposite direction
	bcf		FLAGS1,KEY_PRESSED
	bcf		FLAGS1,KEY_FR
	bsf		LED_PORT,RUN_STOP_LED		;Turn ON RUN LED	
	return

;*******************************************************************************
;This routine stops the motor by driving the PWMs to 0% duty cycle.
;*******************************************************************************
STOP_MOTOR
	bcf		PIE1,ADIE		;	
	bcf		PIE3,IC1IE		;IC1 interrupt
	bcf		PIE3,IC3DRIE	;IC3 interrupt
	bcf		PIE3,IC2QEIE	;IC2 interrupt
	bcf		PIE3,PTIE		;PWM interrupt
	clrf	OVDCOND			;STOP motor
	clrf	PDC0H			;Clear PWM duty cycles
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
;-----------------------------------------------------------------
	clrf	SPEED_REFH
	clrf	MOTOR_CURRENTH
	clrf	MOTOR_CURRENTL

;-----------------------------------------------------------------
	bsf		PIE1,ADIE
	bsf		PIE3,IC1IE			;IC1 interrupt
	bsf		PIE3,IC3DRIE		;IC3 interrupt
	bsf		PIE3,IC2QEIE		;IC2 interrupt
	bsf		PIE3,PTIE			;PWM interrupt
	clrf	FLAGS

	movlw	b'10000011'			;FaultA is enabled in cycle-by-cycle mode, HMIN mode is enabled 
	movwf	FLTCONFIG

	bcf		FLT_FLAGS,OCUR		;clear over current fault flag		
	bcf		FLT_FLAGS,OTEMP		;Clear over temp flag

	call	UPDATE_SEQUENCE		;Re initialize the energizing sequence
	return

;*******************************************************************************
;On POR, turn on LEDs according to the potentiomer level >25% LED1,>50%LED2 and >80% LED3
LED_ON_OFF
	bcf		LED1
	bcf		LED2
	bcf		LED3

	movlw	0X40
	cpfsgt	SPEED_REFH	
	return
	bsf		LED1
	movlw	0x80
	cpfsgt	SPEED_REFH
	return
	bsf		LED2
	movlw	0XD0
	cpfsgt	SPEED_REFH	
	return
	bsf		LED3
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

;*******************************************************************************
;Display routine displays parameters on Hyper terminal
DISPLAY_ROUTINE
	incfsz	DISPLAY_COUNT,F
	return
	movff	SPEED_REFH,DISPLAY_SPEED_REF		;Speed ref,potetiometer(R14) in hex
	movff	MOTOR_CURRENTH,DISPLAY_CURRENT_Iu	;Motor current read 8 bit MSB, in hex
	movff	CAP1H,DISPALY_CAP1_H				;Hall sensorA, pulse width wrt previous Hall transition, Timer5 value in hex
	movff	CAP1L,DISPALY_CAP1_L				;Speed in RPM = 60*(5,000,000/(Timer5Count*8*6*polepairs))
	movff	CAP2H,DISPALY_CAP2_H				;Hall sensorB, pulse width wrt previous Hall transition, Timer5 value in hex
	movff	CAP2L,DISPALY_CAP2_L				;;Speed in RPM = 60*(5,000,000/(Timer5Count*8*6*polepairs))
	movff	CAP3H,DISPALY_CAP3_H				;Hall sensorB, pulse width wrt previous Hall transition, Timer5 value in hex
	movff	CAP3L,DISPALY_CAP3_L				;Speed in RPM = 60*(5,000,000/(Timer5Count*8*6*polepairs))
	call	DISPLAY_PARAMETERS
	return
	

;*******************************************************************************
	
	END

