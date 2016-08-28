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
;THIS FILE CONTAINS ROUTINES FOR 4 FIFO. ROUTINES ARE AS GIVEN BELOW
;WRITE TO TX_BUFFER, READ FROM RX_BUFFER
;READ FROM HOST_BUFFER AND WRITE TO HOST_BUFFER
;READ FROM RESPOSNE_BUFFER AND WRITE TO RESPONSE_BUFFER
;FOR ADDITIONAL INFORMATION ON CREATING AND USING FIFO, VISIT MpAM (MAESTRO MODULE) SECTION
;AT WWW.MICROCHIP.COM
;**************************************************************************************

	INCLUDE	<P18F2431.INC>
	INCLUDE <INTERFACEBLDCCONSTANT.INC>
	INCLUDE <INTERFACEBLDCVAR.INC>

#IF ((ACTIVE_MODE == 0X01) || (ACTIVE_MODE == 0X02))	
	GLOBAL	HOSTBUF_GETCH,HOSTBUF_PUTCH,RESPONSEBUF_GETCH,RESPONSEBUF_PUTCH
	GLOBAL  UARTINT_PUTCH,UARTINT_GETCH

BUFFER	CODE
;*************************************************************************************
;READ HOST BUFFER ROUTINE
;*************************************************************************************
HOSTBUF_GETCH:
        BANKSEL HOST_BUF_DATACNT
        MOVF    HOST_BUF_DATACNT,W
        BNZ     TRANSFERRXDATA
        BSF     HOSTRESPONSE_BUF_STATUS,HOST_BUF_EMPTY   ;SET HOST BUFFER EMPTY LAG
        RETLW   0
TRANSFERRXDATA
        LFSR    FSR2,HOST_BUFFER
        MOVF    HOST_BUF_RDPTR,W
        MOVFF   PLUSW2,TEMP4
        INCF    HOST_BUF_RDPTR,F			;INCREMENT READ POINTER
        MOVLW   HOST_BUFFER_SIZE  			;IF READ POINTER HAS REACHED THE MAXIMUM
        XORWF   HOST_BUF_RDPTR,W    			;VALUE THEN RESET IT FOR ROLL-OVER
        BTFSC   STATUS,Z
        CLRF    HOST_BUF_RDPTR
        DECF    HOST_BUF_DATACNT,F  			;DECREMENT HOST_BUFFER DATA SIZE
        BCF     HOSTRESPONSE_BUF_STATUS,HOST_BUF_FUL	;IS READ SO BUFFER HAS SPACE
        BCF     HOSTRESPONSE_BUF_STATUS,HOST_BUF_OF  	;FOR THE NEW DATA
        MOVF	TEMP4,W		              		;READ THE FIFO BUFFER DATA
        RETURN
;*************************************************************************************
;WRITE HOST BUFFER ROUTINE
;*************************************************************************************
HOSTBUF_PUTCH:
        BANKSEL HOSTRESPONSE_BUF_STATUS
	MOVWF   TEMP4
        BTFSC   HOSTRESPONSE_BUF_STATUS,HOST_BUF_OF  	;CHECK FOR HOST_BUF_OF BIT
	RETURN
        BTFSS   HOSTRESPONSE_BUF_STATUS,HOST_BUF_FUL  	;CHECK FOR HOST_BUF_FUL BIT
        BRA     APPENDHOSTBUFFER   			;IF BUFFER FULL THEN SET HOST BUFFER
        BSF     HOSTRESPONSE_BUF_STATUS,HOST_BUF_OF  	;OVER FLOW FLAG TO INDICATE THAT DATA IS MISSED.
	RETURN
APPENDHOSTBUFFER
        LFSR    FSR2, HOST_BUFFER
        BCF     HOSTRESPONSE_BUF_STATUS,HOST_BUF_EMPTY 	;INDICATE HOST BUF NOT EMPTY
        MOVF    HOST_BUF_WRPTR,W 	   		;POINT TO PREVIOUS LOCATION
        MOVFF   TEMP4,PLUSW2				;COPY THE DATA INTO FIFO BUFFER
        INCF    HOST_BUF_WRPTR,F    			;INCREMENT WRITE POINTER
        MOVLW   HOST_BUFFER_SIZE			;IF WRITE POINTER HAS REACHED THE MAXIMUM
        XORWF   HOST_BUF_WRPTR,W    			;VALUE THEN RESET IT FOR ROLL-OVER
        BTFSC   STATUS,Z
        CLRF    HOST_BUF_WRPTR
	INCF    HOST_BUF_DATACNT,F  			;DECREMENT UARTINT_RX_BUFFER DATA SIZE
        MOVLW   HOST_BUFFER_SIZE  			;IF BUFFER HAS REACHED THE MAXIMUM
        XORWF   HOST_BUF_DATACNT,W  			;VALUE THEN SET THE FLAG FOR FULL
        BZ      FLAG_HOST_BUF_FULL
	RETURN
FLAG_HOST_BUF_FULL
        BSF     HOSTRESPONSE_BUF_STATUS,HOST_BUF_FUL   	;SET FLAG TO INIDCATE HOST BUFFER FULL
	RETURN
;*************************************************************************************
;READ RESPONSE BUFFER ROUTINE
;*************************************************************************************
RESPONSEBUF_GETCH:
        BANKSEL RESPONSE_BUF_DATACNT
        MOVF    RESPONSE_BUF_DATACNT,W
        BNZ     TRANSFERRESPONSEDATA
        BSF     HOSTRESPONSE_BUF_STATUS,RESPONSE_BUF_EMPTY   ;SET RESPONSE BUFFER EMPTY LAG
        RETLW   0
TRANSFERRESPONSEDATA
        LFSR    FSR2,RESPONSE_BUFFER
        MOVF    RESPONSE_BUF_RDPTR,W
        MOVFF   PLUSW2,TEMP5
        INCF    RESPONSE_BUF_RDPTR,F				;INCREMENT READ POINTER
        MOVLW   RESPONSE_BUFFER_SIZE				;IF READ POINTER HAS REACHED THE MAXIMUM
        XORWF   RESPONSE_BUF_RDPTR,W				;VALUE THEN RESET IT FOR ROLL-OVER
        BTFSC   STATUS,Z
        CLRF    RESPONSE_BUF_RDPTR
        DECF    RESPONSE_BUF_DATACNT,F  			;DECREMENT RESPOSNE_BUFFER DATA SIZE
        BCF     HOSTRESPONSE_BUF_STATUS,RESPONSE_BUF_FUL	;IS READ SO BUFFER HAS SPACE
        BCF     HOSTRESPONSE_BUF_STATUS,RESPONSE_BUF_OF  	;FOR THE NEW DATA
        MOVF	TEMP5,W			              		;READ THE FIFO BUFFER DATA
        RETURN
;*************************************************************************************
;WRITE RESPONSE BUFFER ROUTINE
;*************************************************************************************
RESPONSEBUF_PUTCH:
        BANKSEL HOSTRESPONSE_BUF_STATUS
	MOVWF   TEMP5
        BTFSC   HOSTRESPONSE_BUF_STATUS,RESPONSE_BUF_OF  	;CHECK FOR RESPOSNE_BUF_OF BIT
	RETURN
        BTFSS   HOSTRESPONSE_BUF_STATUS,RESPONSE_BUF_FUL  	;CHECK FOR RESPOSNE_BUF_FUL BIT
        BRA     APPENDRESPONSEBUFFER				;IF BUFFER FULL THEN SET RESPOSNE BUFFER
        BSF     HOSTRESPONSE_BUF_STATUS,RESPONSE_BUF_OF  	;OVER FLOW FLAG TO INDICATE THAT
	RETURN
APPENDRESPONSEBUFFER
        LFSR    FSR2, RESPONSE_BUFFER
        BCF     HOSTRESPONSE_BUF_STATUS,RESPONSE_BUF_EMPTY 	;INDICATE RESPOSNE BUF NOT EMPTY
        MOVF    RESPONSE_BUF_WRPTR,W				;POINT TO PREVIOUS LOCATION
        MOVFF   TEMP5,PLUSW2					;COPY THE DATA INTO FIFO BUFFER
        INCF    RESPONSE_BUF_WRPTR,F    			;INCREMENT WRITE POINTER
        MOVLW   RESPONSE_BUFFER_SIZE				;IF WRITE POINTER HAS REACHED THE MAXIMUM
        XORWF   RESPONSE_BUF_WRPTR,W    			;VALUE THEN RESET IT FOR ROLL-OVER
        BTFSC   STATUS,Z
        CLRF    RESPONSE_BUF_WRPTR
	INCF    RESPONSE_BUF_DATACNT,F  			;DECREMENT UARTINT_RX_BUFFER DATA SIZE
        MOVLW   RESPONSE_BUFFER_SIZE  				;IF BUFFER HAS REACHED THE MAXIMUM
        XORWF   RESPONSE_BUF_DATACNT,W  			;VALUE THEN SET THE FLAG FOR FULL
        BZ      FLAG_RESPONSE_BUF_FULL
	RETURN
FLAG_RESPONSE_BUF_FULL
        BSF     HOSTRESPONSE_BUF_STATUS,RESPONSE_BUF_FUL    	;SET FLAG TO INIDCATE RESPOSNE BUFFER FULL
	RETURN
;****************************************************************************
;WRITE TX BUFFER ROUTINE
;****************************************************************************
UARTINT_PUTCH:
	BCF	PIE1,TXIE
        BANKSEL TEMP3
        MOVWF   TEMP3						;STORE VALUE IN TEMP. LOCATION
        BTFSC   UARTINT_STATUS,UARTINT_TX_BUF_FUL		;CHECK FOR UARTINT_TX_BUF_FUL BIT
        RETURN                  				;IF BUFFER FULL THEN RETURN
        MOVF    UARTINT_TX_BUF_DATACNT ,W 			;IF UARTINT_TX_BUFFER IS EMPTY THEN TRANSFER
	BNZ     APPENDTXBUFFER
        MOVFF   TEMP3,TXREG
        INCF    UARTINT_TX_BUF_RDPTR,F    			;INCREMENT READ POINTER
        MOVLW   UARTINT_TX_BUFFER_SIZE  			;IF READ POINTER HAS REACHED THE MAXIMUM
        XORWF   UARTINT_TX_BUF_RDPTR,W    			;VALUE THEN RESET IT FOR ROLL-OVER
        BTFSC   STATUS,Z
        CLRF    UARTINT_TX_BUF_RDPTR
APPENDTXBUFFER
        LFSR    FSR2, UARTINT_TX_BUFFER
        MOVF    UARTINT_TX_BUF_WRPTR,W
        MOVFF   TEMP3,PLUSW2
        INCF    UARTINT_TX_BUF_WRPTR,F    			;INCREMENT WRITE POINTER
        MOVLW   UARTINT_TX_BUFFER_SIZE  			;IF WRITE POINTER HAS REACHED THE MAXIMUM
        XORWF   UARTINT_TX_BUF_WRPTR,W				;VALUE THEN RESET IT FOR ROLL-OVER
        BTFSC   STATUS,Z
        CLRF    UARTINT_TX_BUF_WRPTR
        INCF    UARTINT_TX_BUF_DATACNT,F			;DECREMENT UARTINT_TX_BUFFER DATA SIZE
        MOVLW   UARTINT_TX_BUFFER_SIZE				;IF WRITE POINTER HAS REACHED THE MAXIMUM
        XORWF   UARTINT_TX_BUF_DATACNT,W			;VALUE THEN RESET IT FOR ROLL-OVER
        BTFSS   STATUS,Z
        BRA     TXBUFNOTFULL
        BSF     UARTINT_STATUS,UARTINT_TX_BUF_FUL		;IS BUFFER S FULL
TXBUFNOTFULL	
        BSF     PIE1,TXIE					;ENABLE TX INTERRUPT
        RETURN
;****************************************************************************
;READ RX BUFFER ROUTINE
;****************************************************************************
UARTINT_GETCH:
        BANKSEL UARTINT_RX_BUF_DATACNT
        LFSR    FSR2, UARTINT_RX_BUFFER
        MOVF    UARTINT_RX_BUF_RDPTR,W
        MOVFF   PLUSW2,TEMP3
        INCF    UARTINT_RX_BUF_RDPTR,F				;INCREMENT READ POINTER
        MOVLW   UARTINT_RX_BUFFER_SIZE				;IF READ POINTER HAS REACHED THE MAXIMUM
        XORWF   UARTINT_RX_BUF_RDPTR,W				;VALUE THEN RESET IT FOR ROLL-OVER
        BTFSC   STATUS,Z
        CLRF    UARTINT_RX_BUF_RDPTR
        DCFSNZ	UARTINT_RX_BUF_DATACNT,F			;DECREMENT UARTINT_RX_BUFFER DATA SIZE
        BSF	UARTINT_STATUS,UARTINT_RX_BUF_EMPTY		;SET UARTINT_RX_BUF_EMPTY FLAG
        BCF     UARTINT_STATUS,UARTINT_RX_BUF_FUL		;IS READ SO BUFFER HAS SPACE
        BCF     UARTINT_STATUS,UARTINT_RX_BUF_OF		;FOR THE NEW DATA
        MOVF	TEMP3,W						;READ THE FIFO BUFFER DATA
	RETURN
#ENDIF
;*************************************************************************************
;END OF FILE
;*************************************************************************************
	END