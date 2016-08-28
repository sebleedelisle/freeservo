;**************************************************************************************
;PROGRAM		:	BLDC OPEN LOOP SPEED CONTROL USING HALL SENSORS ON PICDEM MC LV BOARD
;MICROCONTROLLER	:	PIC18FXX31
;CRYSTAL FREQUENCY	:	5MHZ (X4 PLL)
;**************************************************************************************
;AUTHOR			:	PADMARAJA YEDAMALE
;			:	RAKESH PAREKH
;DATE			:	01-APR-2005
;VERSION		:	V1.0
;**************************************************************************************
;*
;* Software License Agreement
;*
;* The software supplied herewith by Microchip Technology Incorporated
;* (the �Company�) for its PICmicro� Microcontroller is intended and
;* supplied to you, the Company�s customer, for use solely and
;* exclusively on Microchip PICmicro Microcontroller products. The
;* software is owned by the Company and/or its supplier, and is
;* protected under applicable copyright laws. All rights are reserved.
;* Any use in violation of the foregoing restrictions may subject the
;* user to criminal sanctions under applicable laws, as well as to
;* civil liability for the breach of the terms and conditions of this
;* license.
;*
;* THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
;* WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
;* TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
;* PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
;* IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
;* CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
;**************************************************************************************

#DEFINE	INTERFACEBLDC

;	LIST P = P18F2431
	INCLUDE <P18F2431.INC>
	INCLUDE <INTERFACEBLDCCONSTANT.INC>
	INCLUDE <INTERFACEBLDCVAR.INC>
	
	EXTERN	FAULT_CHECK,STOP_MOTOR,CONFIGURE_PWM,PROCESS_KEY_PRESSED,UPDATE_PWM,INIT_PWM
	EXTERN	SET_DIRECTION_AT_POR,INIT_MOTION_FEEDBACK_MODULE
	EXTERN	INIT_TMR1,INIT_ADC,READ_SYSTEM_PARAMETERS,INIT_PORTS
	EXTERN	ISR_HIGH,ISR_LOW,CLEAR_ALL_RAM,INIT_INTERRUPT,LOAD_SEQUENCE_TABLE,CLEAR_ALL_LED

#IFDEF	RAMP
	EXTERN	RAMP_SPEED
#ENDIF

#IF ((ACTIVE_MODE == 0X01) || (ACTIVE_MODE == 0X03))
	EXTERN	KEY_CHECK,CONVERT_MANUAL_COUNT_TO_RPS
#ENDIF

#IF ((ACTIVE_MODE == 0X01) || (ACTIVE_MODE == 0X02))
	EXTERN	INIT_EUSART,ACKNOWLEDGE_HOST,PARSE_HOSTCMD_FSM,PARSE_HOSTCMD_RESET,SEND_HOSTDATA
	EXTERN	HOSTBUF_GETCH,RESPONSEBUF_PUTCH,UARTINT_GETCH,PROCESS_SEND_PARAMETERS_CMD
	EXTERN	PROCESS_STOP_CMD,REINITIALIZE_BUFFERS,PROCESS_DOWNLOAD_PARAMETERS_CMD
	EXTERN	PROCESS_SET_DIRECTION_CMD,PROCESS_RUN_CMD
#ENDIF

;Old way of defining the configuration registers
;	__CONFIG	_CONFIG1H, _OSC_HSPLL_1H & _FCMEN_OFF_1H & _IESO_OFF_1H
;	__CONFIG	_CONFIG2L, _PWRTEN_ON_2L & _BOREN_ON_2L & _BORV_27_2L
;	__CONFIG	_CONFIG2H, _WDTEN_OFF_2H & _WINEN_OFF_2H
;	__CONFIG	_CONFIG3L, _T1OSCMX_OFF_3L & _HPOL_HIGH_3L & _LPOL_HIGH_3L & _PWMPIN_OFF_3L
;	__CONFIG	_CONFIG3H, _MCLRE_ON_3H
;	__CONFIG	_CONFIG4L, _STVREN_OFF_4L & _LVP_OFF_4L & _DEBUG_OFF_4L
;	__CONFIG	_CONFIG5L, _CP0_OFF_5L & _CP1_OFF_5L & _CP2_OFF_5L & _CP3_OFF_5L
;	__CONFIG	_CONFIG5H, _CPB_OFF_5H & _CPD_OFF_5H 
;	__CONFIG	_CONFIG6L, _WRT0_OFF_6L & _WRT1_OFF_6L & _WRT2_OFF_6L & _WRT3_OFF_6L
;	__CONFIG	_CONFIG6H, _WRTB_OFF_6H & _WRTC_OFF_6H & _WRTD_OFF_6H
;	__CONFIG	_CONFIG7L, _EBTR0_OFF_7L & _EBTR1_OFF_7L & _EBTR2_OFF_7L & _EBTR3_OFF_7L 
;	__CONFIG	_CONFIG7H, _EBTRB_OFF_7H


;Newer way of defining the configuration registers
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






STARTUP	CODE	0X00
	GOTO	START
H_INT	CODE	0X08
	GOTO	ISR_HIGH
L_INT	CODE	0X18
	GOTO	ISR_LOW
	
MAIN	CODE
START	CLRF	WDTCON				;DISABLE WATCHDOG TIMER
	CLRF	INTCON				;DISABLE ALL INTERRUPTS
	MOVLB	0X01				;SELECT BANK ONE
	BCF	T1CON,TMR1ON			;TURN OFF TIMER 1
	CALL	CLEAR_ALL_RAM			;CLEAR BANK 0, 1  AND 2 RAM
	CALL	READ_SYSTEM_PARAMETERS		;READ LAST SAVED SYSTEM PARAMETERS FROM DATA EEPROM
	CALL	INIT_PORTS			;INITIALIZE PORTS	
#IF ((ACTIVE_MODE == 0X01) || (ACTIVE_MODE == 0X02))
	CALL	INIT_EUSART			;INITIALIZE EUSART
#ENDIF

	CALL	INIT_ADC			;INITIALIZE ADC
	CALL	INIT_TMR1			;INITIALIZE TIMER 1
	CALL	INIT_MOTION_FEEDBACK_MODULE	;INITIALIZE MOTION FEEDBACK MODULE
	CALL	LOAD_SEQUENCE_TABLE
	CALL	INIT_PWM
	BSF	BLDC_FLAGS_1,HW_CONTOL_1ST_TIME
	BSF	GUI_FLAGS,GUI_CONTROL_1ST_TIME
	CALL	INIT_INTERRUPT
	BSF	ADCON0,GO			;SET GO BIT FOR ADC CONVERSION START

#IF (ACTIVE_MODE == 0X01)
CHECK_GUI_PRESENCE
	BTFSC	BAUDCTL,ABDEN			;IS AUTO BAUD DETECT DONE?
	BRA	CHECK_FOR_MANUAL_MODE		;NO - CHECK FOR MANUAL MODE OPERATION
	BRA	IDENTIFY_COMMUNICATOR		;YES - IDENTIFY COMMUNICATOR
CHECK_FOR_MANUAL_MODE
	BTFSS	GUI_FLAGS,GUI_CONTROL_1ST_TIME	;WAS GUI ON EARLIER?	
	BRA	CHECK_GUI_PRESENCE		;YES - WAIT FOR GUI TO COME BACK
	BTFSS	PORTC,SW3			;IS SW1 PRESSED?
	BRA	HW_MOTOR_CONTROL		;YES - JUMP TO MANUAL MODE MOTOR CONTROL
	BTFSS	PORTC,SW2			;NO - SW2 PRESSED
	BRA	HW_MOTOR_CONTROL		;YES - JUMP TO MANUAL MODE MOTOR CONTROL
	BRA	CHECK_GUI_PRESENCE		;NO - JUMP TO CHECK GUI PRESENCE
#ENDIF

#IF (ACTIVE_MODE == 0X02)
CHECK_GUI_PRESENCE
	BTFSC	BAUDCTL,ABDEN			;IS AUTO BAUD DETECT DONE?
	BRA	$-2				;NO - CHECK AGAIN
	BRA	IDENTIFY_COMMUNICATOR		;YES - IDENTIFY COMMUNICATOR
#ENDIF

#IF (ACTIVE_MODE == 0X03)
CHECK_FOR_MANUAL_MODE
	BTFSS	PORTC,SW3			;IS SW3 PRESSED?
	BRA	HW_MOTOR_CONTROL		;YES - JUMP TO MANUAL MODE MOTOR CONTROL
	BTFSS	PORTC,SW2			;NO - SW2 PRESSED
	BRA	HW_MOTOR_CONTROL
	BRA	CHECK_FOR_MANUAL_MODE		;JUMP TO MANUAL CONTROL
#ENDIF
;**************************************************************************************
;GUI IDENTIFICATION IS DONE HERE. ON SUCCESSFUL IDENTIFICATION LED BLINK WILL STOP
;ALSO GUI WILL SHOW INDICATION FOR THE SAME
;**************************************************************************************
#IF ((ACTIVE_MODE == 0X01) || (ACTIVE_MODE == 0X02))
IDENTIFY_COMMUNICATOR
	BCF	INTCON,PEIE			;DISABLE ALL PERIPHERAL INTS
	CALL	UARTINT_GETCH			;READ RX BUFFER - NO ACTION ON DATA AS IT IS 1ST READ AFTER ABD
	BSF	INTCON,PEIE			;REENABLE ALL PERIPHERAL INTS
WAIT_FOR_MORE_DATA
	MOVF	UARTINT_RX_BUF_DATACNT,W	;CHECK IS THERE ANY UNREAD RECEIVED DATA?
	BZ	WAIT_FOR_MORE_DATA		;IF NO - WAIT FOR MORE DATA
	BTFSS	GUI_FLAGS,COM_IN_PROGRESS	;IS COMMUNICATION IN PROGRESS?
	CALL	PARSE_HOSTCMD_RESET		;NO - RESET FSM AND ITS FLAG
	CALL	PARSE_HOSTCMD_FSM		;ANALYSE HOST COMMAND AND DATA
	BTFSC	GUI_FLAGS,COMMUNICATION_ERROR	;CHECK FOR COMMUNICATION ERROR
	BRA	GUI_COMMUNICATION_ERROR	
	BTFSC	GUI_FLAGS,COM_IN_PROGRESS	;IS COMMUNICATION IN PROGRESS?
	BRA	WAIT_FOR_MORE_DATA
	MOVLW	0X05				;CHECK FOR HOST DATA LENGTH = 05
	MOVFF	HOSTDATALEN,TEMP4
	CPFSEQ	TEMP4
	BRA	GUI_NOT_PRESENT
	CALL	HOSTBUF_GETCH
	MOVLW	'I'				;CHECK FOR HOST IDENTIFICATION COMMAND 'I'
	CPFSEQ	TEMP4
	BRA	GUI_NOT_PRESENT
	CALL	HOSTBUF_GETCH
	MOVLW	'M'				;CHECK FOR HOST IDENTIFICATION DATA 'MCGUI'
	CPFSEQ	TEMP4
	BRA	GUI_NOT_PRESENT
	CALL	HOSTBUF_GETCH
	MOVLW	'C'
	CPFSEQ	TEMP4
	BRA	GUI_NOT_PRESENT
	CALL	HOSTBUF_GETCH
	MOVLW	'G'
	CPFSEQ	TEMP4
	BRA	GUI_NOT_PRESENT
	CALL	HOSTBUF_GETCH
	MOVLW	'U'
	CPFSEQ	TEMP4
	BRA	GUI_NOT_PRESENT
	CALL	HOSTBUF_GETCH
	MOVLW	'I'
	CPFSEQ	TEMP4
	BRA	GUI_NOT_PRESENT
	BSF	GUI_FLAGS,GUI_ON		;SET FLAG FOR SUCCESSFUL GUI IDENTIFICATION OF MC GUI
	CALL	ACKNOWLEDGE_HOST		;SEND ACKNOWLEDGEMENT TO  GUI
	MOVLW	0X0A				;SET RESPONSE DATA LENGTH 'A'
	MOVWF	RESPONSEDATALEN			
	MOVLW	'I'			
	MOVWF	RESPONSE			;SET RESPONSE COMMAND 'I'
	MOVLW	MOTOR_TYPE_CONTROL		;SEND IDENTIFICATION BYTE FOR MOTOR CONTROL TYPE
	CALL	RESPONSEBUF_PUTCH
	MOVLW	PIC_TYPE			;SEND IDENTIFICATION BYTE FOR USED PIC TYPE
	CALL	RESPONSEBUF_PUTCH
	MOVLW	FM_VER				;SEND IDENTIFICATION BYTE FOR FIRMWARE VERSION
	CALL	RESPONSEBUF_PUTCH
	MOVLW	OSC_VALUE_IN_MHZ		;SEND BOARD OSCILLATOR FREQUENCY (DEFINED IN INTERFACEBLDCCONSTANT.INC)
	CALL	RESPONSEBUF_PUTCH
	MOVFF	PARAMETER_BUFFER+LOC_BUFFER_CHKSUM_1,WREG
	CALL	RESPONSEBUF_PUTCH		;SEND CHECKSUM FOR GUI SETUP WINDOW
	MOVFF	PARAMETER_BUFFER+LOC_BUFFER_CHKSUM_2,WREG
	CALL	RESPONSEBUF_PUTCH
	MOVLW	0X00				;FOUR DUMMY BYTES FOR FUTURE USE
	CALL	RESPONSEBUF_PUTCH
	MOVLW	0X00
	CALL	RESPONSEBUF_PUTCH
	MOVLW	0X00
	CALL	RESPONSEBUF_PUTCH
	MOVLW	0X00
	CALL	RESPONSEBUF_PUTCH
	CALL	SEND_HOSTDATA			;SEND RESPONSE DATA BYTES TO MCGUI
	CALL	CLEAR_ALL_LED			;TURN OFF ALL LED
	CALL	SET_DIRECTION_AT_POR		;SET DEFAULT ROTATION DIRECTION FOR MOTOR (DEFINED IN INTERFACEBLDCCONSTANT.INC)
	CALL	STOP_MOTOR			;ROUTINE CALLED TO SET ALL VARIABLES IN DEFAULT STATES
	BRA	GUI_MOTOR_CONTROL		;JUMP TO GUI BASED MOTOR CONTROL
;**************************************************************************************
;GUI IDENTIFICATION ROUTINE ENDS
;**************************************************************************************
GUI_MOTOR_CONTROL
	BTFSC	GUI_FLAGS,GUI_TIME_OUT
	BRA	GUI_NOT_PRESENT
	MOVF	UARTINT_RX_BUF_DATACNT,W
	BZ	ACT_ON_CMD			;IF RX BUFFER EMPTY, THEN JUMP TO ACT_ON_CMD
        BTFSS	GUI_FLAGS,COM_IN_PROGRESS
	CALL	PARSE_HOSTCMD_RESET		;RESET FSM AND ITS FLAG
	CALL	PARSE_HOSTCMD_FSM		;ANALYSE HOST COMMAND AND DATA
	BTFSC	GUI_FLAGS,COMMUNICATION_ERROR
	BRA	GUI_COMMUNICATION_ERROR
	BTFSC	GUI_FLAGS,COM_IN_PROGRESS	;IS COMMUNICATION IN PROGRESS?
	BRA	ACT_ON_CMD
	CALL	HOSTBUF_GETCH			;READ COMMAND ID
	XORLW	COMMAND_ASK_DATA		;IS RECEIVED COMMAND 'ASK DATA'
	BZ	GUI_SEND_MOTOR_PARAMETERS	;YES
	MOVF	TEMP4,W				;RELOAD COMMAND ID
	XORLW	COMMAND_RUN			;IS RECEIVED COMMAND 'RUN MOTOR'
	BZ	GUI_RUN_MOTOR			;YES
	MOVF	TEMP4,W				;RELOAD COMMAND ID
	XORLW	COMMAND_STOP			;IS RECEIVED COMMAND 'STOP MOTOR'
	BZ	GUI_STOP_MOTOR			;YES
	MOVF	TEMP4,W				;RELOAD COMMAND ID
	XORLW	COMMAND_DIRECTION		;IS RECEIVED COMMAND 'CHANGE DIRECTION'
	BZ	GUI_SET_MOTOR_DIRECTION		;YES
	MOVF	TEMP4,W				;RELOAD COMMAND ID
	XORLW	COMMAND_SYSTEM_PARAMETERS	;IS RECEIVED COMMAND 'SYSTEM PARAMETERS'
	BZ	GUI_SET_SYSTEM_PARAMETERS	;YES
	BRA	GUI_COMMUNICATION_ERROR		;ILLEGAL COMMAND ID - EXECUTE COMMUNICATION ERROR

GUI_STOP_MOTOR
	CALL	ACKNOWLEDGE_HOST		;SEND ACKNOWLEDGEMENT TO  GUI
	CALL	PROCESS_STOP_CMD		;STOP THE MOTOR
	BRA	GUI_MOTOR_CONTROL
GUI_RUN_MOTOR
	CALL	ACKNOWLEDGE_HOST		;SEND ACKNOWLEDGEMENT TO  GUI
	CALL	PROCESS_RUN_CMD			;RUN THE MOTOR
	BTFSS	GUI_FLAGS,GUI_CONTROL_1ST_TIME	;IS IT 1ST GUI CONTROL AFTER DOWNLOAD?
	BRA	GUI_MOTOR_CONTROL		;NO
	CALL	CONFIGURE_PWM			;YES - SET NEW PWM FREQUENCY
	BCF	GUI_FLAGS,GUI_CONTROL_1ST_TIME	;CLEAR 1ST GUI CONTROL FLAG
	BRA	GUI_MOTOR_CONTROL
GUI_SET_MOTOR_DIRECTION
	CALL	ACKNOWLEDGE_HOST		;SEND ACKNOWLEDGEMENT TO  GUI
	CALL	PROCESS_SET_DIRECTION_CMD	;CHANGE MOTOR ROTATION DIRECTION
	BRA	GUI_MOTOR_CONTROL
GUI_SEND_MOTOR_PARAMETERS
	CALL	ACKNOWLEDGE_HOST		;SEND ACKNOWLEDGEMENT TO  GUI
	CALL	PROCESS_SEND_PARAMETERS_CMD	;SEND RUNTIME MOTOR PARAMETERS TO GUI FOR DISPLAY
	BRA	GUI_MOTOR_CONTROL
GUI_SET_SYSTEM_PARAMETERS
	CALL	PROCESS_DOWNLOAD_PARAMETERS_CMD	;CONFIGURE GUI SETUP PARAMETERS - STORE THEM IN RAM AS WELL AS DATA EEPROM
	CALL	ACKNOWLEDGE_HOST		;SEND ACKNOWLEDGEMENT TO  GUI
	BSF	GUI_FLAGS,GUI_CONTROL_1ST_TIME	;SET FLAG TO INDICATE 1ST GUI CONTROL AFTER DOWNLOAD
	BRA	GUI_MOTOR_CONTROL
#ENDIF
;*************************************************************************************
;ROUTINE FOR MANUAL MOTOR CONTROL
;*************************************************************************************
#IF ((ACTIVE_MODE == 0X01) || (ACTIVE_MODE == 0X03))
HW_MOTOR_CONTROL
	BTFSS	BLDC_FLAGS_1,HW_CONTOL_1ST_TIME	;IS IT 1ST MANUAL MODE CONTROL AFTER POR?
	BRA	ACT_ON_CMD			;NO
	CALL	CONFIGURE_PWM			;YES - SET NEW PWM FREQUENCY
	CALL	CLEAR_ALL_LED			;CLEAR ALL LED BEFRE START
	CALL	SET_DIRECTION_AT_POR		;SET POR ROTATION DIRECTION
	BCF	BLDC_FLAGS_1,HW_CONTOL_1ST_TIME	;CLEAR 1ST MANUAL MODE CONTROL FLAG
#ENDIF
;*************************************************************************************
;ROUTINE FOR MOTOR CONTROL - SAME FOR GUI AND MANUAL CONTROL
;*************************************************************************************
ACT_ON_CMD
#IF ((ACTIVE_MODE == 0X01) || (ACTIVE_MODE == 0X03))
	BTFSS	GUI_FLAGS,GUI_ON		
	CALL	KEY_CHECK			;CHECK FOR ANY PRESSED KEY FOR MANUAL MODE CONTROL
#ENDIF
	CALL	PROCESS_KEY_PRESSED		;ACT ON LAST RECEIVED COMMAND - EITHER FROM GUI OR FROM MANUAL MODE
	BTFSC	BLDC_FLAGS_1,TMR1_OF
	CALL	FAULT_CHECK			;CHECK FOR FAULT AT 5msec INTERVAL

#IF ((ACTIVE_MODE == 0X01) || (ACTIVE_MODE == 0X03))
	BTFSS	GUI_FLAGS,GUI_ON
	CALL	CONVERT_MANUAL_COUNT_TO_RPS	;FOR MANUAL MODE CONTROL, CONVERT INPUT SPEED COMMAND TO RPS SPEED COMMAND
#ENDIF

	BTFSC	BLDC_FLAGS,CALC_PWM
	CALL	UPDATE_PWM			;CALCULATE NEW PWM DUTY CYCLE AFTER EVERY ADC CONVERSION
	
#IFDEF	RAMP					;THIS IS ACCELERATION/DECELERATION ROUTINE AND IS EXECUTED AT EVERY 1SEC
	MOVLW	0XC7
	CPFSGT	RAMP_COUNT
	BRA	CHECK_CONTROL_TYPE
	BTFSS	BLDC_FLAGS_1,RUN_STOP
	BRA	CHECK_CONTROL_TYPE
	CALL	RAMP_SPEED			;INCREASE/DECREASE SPEED TO GET REQUIRED SPEED AT EVERY 1SEC
#ELSE
	MOVFF	REQ_SPEED_RPS,ACTUAL_SPEED_RPS	;FOR ZERO ACCELERATION/DECELERATION, SET ACTUAL SPEED TO REQUIRED SPEED
#ENDIF

CHECK_CONTROL_TYPE
;*************************************************************************************
;USER CAN ENTER HIS CODE AT THIS POINT - WHILE ENTERING USER CODE
;TAKE CARE TO SET CORRECT BSR BITS AND RETRIEVE PREVIOUS BSR ON RETURN
;THIS FW USES PORTION OF ACCESS AND BANK1 RAM
;IF USER DEFINES HIS VARIABLES IN THESE AREA, NO NEED TO SAVE AND RETRIEVE BSR
;*************************************************************************************
#IF (ACTIVE_MODE == 0X01)
	BTFSS	GUI_FLAGS,GUI_ON		;JUMP BACK TO APPROPRIATE CONTROL
	BRA	HW_MOTOR_CONTROL
	BRA	GUI_MOTOR_CONTROL
#ENDIF

#IF (ACTIVE_MODE == 0X02)
	BRA	GUI_MOTOR_CONTROL
#ENDIF

#IF (ACTIVE_MODE == 0X03)
	BRA	HW_MOTOR_CONTROL
#ENDIF
;*************************************************************************************
;ROUTINE FOR HANDLING COMMUNICATION ERROR
;*************************************************************************************
#IF ((ACTIVE_MODE == 0X01) || (ACTIVE_MODE == 0X02))
GUI_COMMUNICATION_ERROR
	BTFSC	GUI_FLAGS,GUI_TIME_OUT		;IF COMMUNICATION ERROR DUE TO TIME OUT OR 1ST TIME
	BRA	GUI_NOT_PRESENT			;RESET CONTROL FOR CONTROL CHECKING
	BTFSS	GUI_FLAGS,GUI_ON
	BRA	GUI_NOT_PRESENT
	BCF	INTCON,GIE			;FOR ERRORNEOUS COMMUNICATION
	BCF	INTCON,GIE			;FLSUH ALL PREVIOUS COMMUNICATION DATAS
	CALL	REINITIALIZE_BUFFERS
	BSF	INTCON,GIE
	BRA	GUI_MOTOR_CONTROL
;*************************************************************************************
;ROUTINE FOR HANDLING COMMUNICATION TIME OUT AND IMPROPER GUI IDENTIFICATION
;*************************************************************************************
GUI_NOT_PRESENT
	BCF	INTCON,GIE			;DISABLE ALL INTERRUPTS
	BCF	INTCON,GIE
	CLRF	SPBRG
	CLRF	SPBRGH
	CALL	PROCESS_STOP_CMD		;STOP THE MOTOR
	CALL	PROCESS_KEY_PRESSED
	CALL	REINITIALIZE_BUFFERS		;RESET ALL VARIABLES
	BTFSC	GUI_FLAGS,GUI_ON
	BCF	GUI_FLAGS,GUI_CONTROL_1ST_TIME
	BCF	GUI_FLAGS,GUI_ON
	BSF	BAUDCTL,ABDEN			;REENABLE AUTO BAUD DETECT
	CALL	CLEAR_ALL_LED
	CLRF	FLT_FLAGS
	CLRF	FLT_FLAGS_1
	BSF	INTCON,GIE
	BRA	CHECK_GUI_PRESENCE		;GO BACK TO CHECK GUI PRESENCE	
#ENDIF
;*************************************************************************************
;EEDATA STORAGE FOR SYSTEM PARAMETER (DEFAULT CONFIGURATION)
;*************************************************************************************
EEDATA	CODE	0XF00000
	DW	0X0000
	DW	0X3282
	DW	0XFC3C
	DW	0X0408
	DW	0X000C	;0x0004 for 60 deg
	DW	0XC800
	DW	0X7A3F
	DW	0X000D
	DW	0X0000
	DW	0X0000
	DW	0X0A00
	DW	0X730A
	DW	0X2620
	DW	0X03E5
	DW	0X0138
	DW	0X001A
	DW	0X003E
	DW	0X0000
	DW	0X0000
	DW	0X0000
	DW	0X0A00
	DW	0X0000
;*************************************************************************************
;END OF FILE
;*************************************************************************************
	END