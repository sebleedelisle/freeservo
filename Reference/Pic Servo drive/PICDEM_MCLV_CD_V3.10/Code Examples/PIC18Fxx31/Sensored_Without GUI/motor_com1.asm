;This routine dispalys few key parameters on Hyper terminal. 
;Baud rate set to 9600
;Parameters displayed can be changed in the main motor control program
;After debugging, this program can be removed from the motor control project

; Author : Padmaraja Yedamale, Microchip Technology Inc
;Version : 1.0
;----------------------------------------------------------------


	include		"P18F2431.inc"

;----------------------------------------------------------------
	UDATA_ACS

DISPLAY_TEMP1		res	1		
DISPLAY_TEMP2		res	1	
DISPLAY_SPEED_REF	res	1
DISPLAY_SPEED_ACTH	res	1
DISPLAY_SPEED_ACTL	res	1
DISPLAY_CURRENT_Iu	res	1
DISPLAY_CURRENT_Iv	res	1
DISPLAY_CURRENT_Iw	res	1
DISPLAY_MISC_PARAH	res	1
DISPLAY_MISC_PARAL	res	1
DISPLAY_MISC_PARA2H	res	1
DISPLAY_MISC_PARA2L	res	1
DISPLAY_SFH	res	1
DISPLAY_SFL	res	1
DISPLAY_SRH	res	1
DISPLAY_SRL	res	1
DISPLAY_EIH	res	1
DISPLAY_EIL	res	1
DISPLAY_EOH	res	1
DISPLAY_EOL	res	1
DISPLAY_EOLL	res	1


DISPALY_CAP1_H	res	1
DISPALY_CAP1_L	res	1
DISPALY_CAP2_H	res	1
DISPALY_CAP2_L	res	1
DISPALY_CAP3_H	res	1
DISPALY_CAP3_L	res	1

;----------------------------------------------------------------
	GLOBAL	DISPLAY_SPEED_REF	
	GLOBAL	DISPLAY_SPEED_ACTH	
	GLOBAL	DISPLAY_SPEED_ACTL
	GLOBAL	DISPLAY_CURRENT_Iu	
	GLOBAL	DISPLAY_CURRENT_Iv	
	GLOBAL	DISPLAY_CURRENT_Iw	
	GLOBAL	DISPLAY_MISC_PARAH
	GLOBAL	DISPLAY_MISC_PARAL
	GLOBAL	DISPLAY_MISC_PARA2H
	GLOBAL	DISPLAY_MISC_PARA2L
	
	GLOBAL	DISPLAY_SFH
	GLOBAL	DISPLAY_SFL
	GLOBAL	DISPLAY_SRH
	GLOBAL	DISPLAY_SRL
	GLOBAL	DISPLAY_EIH
	GLOBAL	DISPLAY_EIL
	GLOBAL	DISPLAY_EOH
	GLOBAL	DISPLAY_EOL
	GLOBAL	DISPLAY_EOLL

	GLOBAL	DISPALY_CAP1_H
	GLOBAL	DISPALY_CAP1_L
	GLOBAL	DISPALY_CAP2_H
	GLOBAL	DISPALY_CAP2_L
	GLOBAL	DISPALY_CAP3_H
	GLOBAL	DISPALY_CAP3_L

;----------------------------------------------------------------
	GLOBAL	WELCOME_MESSAGE	
	GLOBAL	SEND_BYTE_FROM_WREG
	GLOBAL	INITIALIZE_SERIAL_PORT
	GLOBAL	DISPLAY_PARAMETERS
;*******************************************************************************
PRG1 code
;*******************************************************************
;This routine initializes USART parameters 
;******************************************************************
INITIALIZE_SERIAL_PORT

	movlw	0x81		;Baudrate = 9600
	movwf	SPBRG
	
	movlw	0x24		;8-bit transmission;Enable Transmission;	
	movwf	TXSTA		;Asynchronous mode with High speed transmission
	
	movlw	0x90		;Enable the serial port
	movwf	RCSTA		;with 8-bit continuous reception

	bcf	TRISC,6
	bsf	TRISC,7

	return


;*******************************************************************************
;This routine loads the data in Wreg to Transmission register(TXREG) after checking
;of completion of previously loaded byte transmission
;*******************************************************************************
SEND_BYTE_FROM_WREG
	btfss	PIR1,TXIF
	goto	SEND_BYTE_FROM_WREG
	movwf	TXREG
	return

;*******************************************************************************
;This routine intializes the USART module to communicate with host PC and displays 
;a welcome message on the screen
;*******************************************************************************
WELCOME_MESSAGE
	movlw	UPPER WELCOME_TABLE	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH WELCOME_TABLE
	movwf	TBLPTRH
	movlw	LOW WELCOME_TABLE
	movwf	TBLPTRL
SEND_NEXT_W1
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_W1	

WELCOME_TABLE ;BLDC Motor Control
	db	0xA,0xD,0x42,0x4C,0x44,0x43,0x20,0x4D,0x6F,0x74,0x6F,0x72,0x20,0x43,0x6F,0x6E,0x74,0x72,0x6F,0x6C,0x0A,0x0D,0x00
;*******************************************************************************
DISPLAY_PARAMETERS
	call	PARAMETER_ZERO
	movff	DISPLAY_SPEED_REF,DISPLAY_TEMP1
	call	DISPLAY_DIGITS
	call	PARAMETER_ONE
	movff	DISPLAY_CURRENT_Iu,DISPLAY_TEMP1
	call	DISPLAY_DIGITS
	call	PARAMETER_TWO
	movff	DISPALY_CAP1_H,DISPLAY_TEMP1
	call	DISPLAY_DIGITS
	movff	DISPALY_CAP1_L,DISPLAY_TEMP1
	call	DISPLAY_DIGITS
	call	PARAMETER_THREE
	movff	DISPALY_CAP2_H,DISPLAY_TEMP1
	call	DISPLAY_DIGITS
	movff	DISPALY_CAP2_L,DISPLAY_TEMP1
	call	DISPLAY_DIGITS
	call	PARAMETER_FOUR
	movff	DISPALY_CAP3_H,DISPLAY_TEMP1
	call	DISPLAY_DIGITS
	movff	DISPALY_CAP3_L,DISPLAY_TEMP1
	call	DISPLAY_DIGITS
	return

	call	PARAMETER_FIVE
	movff	DISPLAY_CURRENT_Iv,DISPLAY_TEMP1
	call	DISPLAY_DIGITS  
	call	PARAMETER_SIX
	movff	DISPLAY_CURRENT_Iw,DISPLAY_TEMP1
	call	DISPLAY_DIGITS  
;	call	PARAMETER_SEVEN
;	movff	DISPLAY_MISC_PARAH,DISPLAY_TEMP1
;	call	DISPLAY_DIGITS	
;	movff	DISPLAY_MISC_PARAL,DISPLAY_TEMP1
;	call	DISPLAY_DIGITS	
;	call	PARAMETER_EIGHT
;	movff	DISPLAY_MISC_PARA2H,DISPLAY_TEMP1
;	call	DISPLAY_DIGITS	
;	movff	DISPLAY_MISC_PARA2L,DISPLAY_TEMP1
;	call	DISPLAY_DIGITS	

	call	PARAMETER_SEVEN
	movff	DISPLAY_SRH,DISPLAY_TEMP1
	call	DISPLAY_DIGITS	
	movff	DISPLAY_SRL,DISPLAY_TEMP1
	call	DISPLAY_DIGITS
		
	call	PARAMETER_EIGHT
	movff	DISPLAY_SFH,DISPLAY_TEMP1
	call	DISPLAY_DIGITS	
	movff	DISPLAY_SFL,DISPLAY_TEMP1
	call	DISPLAY_DIGITS	
	
	call	PARAMETER_NINE
	movff	DISPLAY_EIH,DISPLAY_TEMP1
	call	DISPLAY_DIGITS	
	movff	DISPLAY_EIL,DISPLAY_TEMP1
	call	DISPLAY_DIGITS	
	
	call	PARAMETER_TEN
	movff	DISPLAY_EOH,DISPLAY_TEMP1
	call	DISPLAY_DIGITS	
	movff	DISPLAY_EOL,DISPLAY_TEMP1
	call	DISPLAY_DIGITS	
	movff	DISPLAY_EOLL,DISPLAY_TEMP1
	call	DISPLAY_DIGITS	
	return

;******************************************************************************
;Speed referance display : SpeedRef= 
PARAMETER_ZERO
	movlw	UPPER PARAMETER_0	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH PARAMETER_0
	movwf	TBLPTRH
	movlw	LOW PARAMETER_0
	movwf	TBLPTRL
SEND_NEXT_P0
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_P0	

PARAMETER_0	;	SpeedRef= 
	db	0xA,0xA,0xD,0x53,0x70,0x65,0x65,0x64,0x52,0x65,0x66,0x3D,0x20,0x00
;*******************************************************************************
;Actual Speed display : Motor Current= 
PARAMETER_ONE
	movlw	UPPER PARAMETER_1	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH PARAMETER_1
	movwf	TBLPTRH
	movlw	LOW PARAMETER_1
	movwf	TBLPTRL
SEND_NEXT_P1
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_P1

PARAMETER_1	;	MotorI = 
	db	0xA,0xD,0x4D,0x6F,0x74,0x6F,0x72,0x20,0x49,0x3D,0x20,0x00
	
;*******************************************************************************
;Ref motor direction : Direction Command :  
PARAMETER_TWO
	movlw	UPPER PARAMETER_2	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH PARAMETER_2
	movwf	TBLPTRH
	movlw	LOW PARAMETER_2
	movwf	TBLPTRL
SEND_NEXT_P2
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_P2

PARAMETER_2	;	IC1 :  
	db	0xA,0xD,0x49,0x43,0x31,0x3a,0x00
		
;*******************************************************************************
;Actual motor direction : Direction Act :  
PARAMETER_THREE
	movlw	UPPER PARAMETER_3	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH PARAMETER_3
	movwf	TBLPTRH
	movlw	LOW PARAMETER_3
	movwf	TBLPTRL
SEND_NEXT_P3
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_P3

PARAMETER_3	;	IC2 :  
	db	0xA,0xD,0x49,0x43,0x32,0x3a,0x00
;*******************************************************************************
;Motor Current Phase U : Current U =   
PARAMETER_FOUR
	movlw	UPPER PARAMETER_4	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH PARAMETER_4
	movwf	TBLPTRH
	movlw	LOW PARAMETER_4
	movwf	TBLPTRL
SEND_NEXT_P4
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_P4

PARAMETER_4	;		IC3 :  
	db	0xA,0xD,0x49,0x43,0x33,0x3a,0x00
;*******************************************************************************
;Motor Current Phase V : Current V =   
PARAMETER_FIVE
	movlw	UPPER PARAMETER_5	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH PARAMETER_5
	movwf	TBLPTRH
	movlw	LOW PARAMETER_5
	movwf	TBLPTRL
SEND_NEXT_P5
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_P5

PARAMETER_5	;	Current V =   
	db	0xA,0xD,0x43,0x75,0x72,0x72,0x65,0x6e,0x74,0x20,0x56,0x20,0x3d,0x20,0x00
;*******************************************************************************
;Motor Current Phase W : Current W =   
PARAMETER_SIX
	movlw	UPPER PARAMETER_6	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH PARAMETER_6
	movwf	TBLPTRH
	movlw	LOW PARAMETER_6
	movwf	TBLPTRL
SEND_NEXT_P6
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_P6

PARAMETER_6	;	Current W =   
	db	0xA,0xD,0x43,0x75,0x72,0x72,0x65,0x6e,0x74,0x20,0x57,0x20,0x3d,0x20,0x00

;*******************************************************************************
;Misc display: ParaX = (SR)  
PARAMETER_SEVEN
	movlw	UPPER PARAMETER_7	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH PARAMETER_7
	movwf	TBLPTRH
	movlw	LOW PARAMETER_7
	movwf	TBLPTRL
SEND_NEXT_P7
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_P7

PARAMETER_7	;	ParaX = 
;	db	0xA,0xD,0x50,0x61,0x72,0x61,0x58,0x20,0x3d,0x20,0x00
	db	0xA,0xD,0x53,0x52,0x00

;*******************************************************************************
;Misc display: ParaX2 =  (SF) 
PARAMETER_EIGHT
	movlw	UPPER PARAMETER_8	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH PARAMETER_8
	movwf	TBLPTRH
	movlw	LOW PARAMETER_8
	movwf	TBLPTRL
SEND_NEXT_P8
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_P8

PARAMETER_8	;	ParaX2= 
;	db	0xA,0xD,0x50,0x61,0x72,0x61,0x58,0x32,0x20,0x3d,0x20,0x00
	db	0xA,0xD,0x53,0x46,0x00

;*******************************************************************************
;Misc display: ParaX3 =  (EI) 
PARAMETER_NINE
	movlw	UPPER PARAMETER_9	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH PARAMETER_9
	movwf	TBLPTRH
	movlw	LOW PARAMETER_9
	movwf	TBLPTRL
SEND_NEXT_P9
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_P9

PARAMETER_9	;	ParaX2= 
;	db	0xA,0xD,0x50,0x61,0x72,0x61,0x58,0x32,0x20,0x3d,0x20,0x00
	db	0xA,0xD,0x45,0x49,0x00
	
;*******************************************************************************
;Misc display: ParaX2 =  (EO) 
PARAMETER_TEN
	movlw	UPPER PARAMETER_10	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH PARAMETER_10
	movwf	TBLPTRH
	movlw	LOW PARAMETER_10
	movwf	TBLPTRL
SEND_NEXT_P10
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_P10

PARAMETER_10	;	ParaX2= 
;	db	0xA,0xD,0x50,0x61,0x72,0x61,0x58,0x32,0x20,0x3d,0x20,0x00
	db	0xA,0xD,0x45,0x4F,0x00	
;*******************************************************************************
;Display: FORWARD   
DISPLAY_FORWARD
	movlw	UPPER DISP_FORWARD	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH DISP_FORWARD
	movwf	TBLPTRH
	movlw	LOW DISP_FORWARD
	movwf	TBLPTRL
SEND_NEXT_FWD
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_FWD

DISP_FORWARD	;	FORWARD 
	db	0x46,0x4F,0x52,0x57,0x41,0x52,0x44,0x00					
;*******************************************************************************
;Display: REVERSE   
DISPLAY_REVERSE   
	movlw	UPPER DISP_REVERSE   	;Initialize Table pointer to the first  
	movwf	TBLPTRU				;location of the table
	movlw	HIGH DISP_REVERSE   
	movwf	TBLPTRH
	movlw	LOW DISP_REVERSE   
	movwf	TBLPTRL
SEND_NEXT_REVERSE   
	TBLRD*+
	movf	TABLAT,W
	btfsc	STATUS,Z
	return
	call	SEND_BYTE_FROM_WREG
	bra		SEND_NEXT_REVERSE   

DISP_REVERSE   	 
	db	0x52,0x45,0x56,0x45,0x52,0x53,0x45,0x00		
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
		
	end
