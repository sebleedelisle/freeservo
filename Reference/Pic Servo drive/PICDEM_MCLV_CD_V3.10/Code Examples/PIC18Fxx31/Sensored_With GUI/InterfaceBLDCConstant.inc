;GUI INTERFACE CODE CONSTANT AND BYTE DEFINITION

#DEFINE RAMP                	;THIS DIRECTIVE ADDS ACCELERATION/DECELERATION ROUTINE IN FIRMWARE
                    		;COMMENT THIS DIRECTIVE IF ACCELERATION/DECELERATION NOT REQUIRED
#DEFINE ACTIVE_MODE     0x01    ;DEFINES ACTIVE CONTROL MODE
                    		;0X01 - GUI OR MANUAL MODE
                    		;0X02 - GUI MODE ONLY
                    		;0X03 - MANUAL MODE ONLY
;****************************************************************************
;MOTOR TYPE, CONTROL ALGORITHM, PIC SELECTION AND FIRMWARE VERSION CONTROL
;****************************************************************************
#DEFINE MOTOR_TYPE_CONTROL  0X20	;DEFINE MOTOR TYPE AND CONTROL ALOGRITHM TYPE
;[Control Type]
;0x00=Information Not Available
;0x10=Brushless DC Motor - Open Loop Control on PICDEM MC
;0x11=Brushless DC Motor - Close Loop Control on PICDEM MC
;0x12=Brushless DC Motor - Open Loop Control on PICDEM MC with 3Ph Current Measurement
;0x13=Brushless DC Motor - Close Loop Control on PICDEM MC with 3Ph Current Measurement
;0x20=Brushless DC Motor - Open Loop Control on PICDEM MC LV
;0x21=Brushless DC Motor - Close Loop Control on PICDEM MC LV
;0x22=Brushless DC Motor - Sensorless Control on PICDEM MC LV



#DEFINE PIC_TYPE        0X03    	;DEFINES CONTROLLER TYPE
                    			;0X02 - PIC18F2331
                 			;0X03 - PIC18F2431
				

#DEFINE FM_VER          0X11		;DEFINES FIRMWARE VERSION
					;PRESENT FIRMWARE VER 1.1 = 0X11
#DEFINE OSC_VALUE_IN_MHZ    0X014	;5	;DEFINES OSCILLATOR FREQUENCY IN MHZ
		                 	;DEFAULT 20MHz - 0X14

;#DEFINE SHUNT_MEASUREMENT               ;Define for DC Bus Current measurement
                                        ;Comment for 3-Phase Current Measurement
;****************************************************************************
;BUFFER SIZE DEFINITION (DO NOT CHANGE)
;****************************************************************************
#DEFINE UARTINT_RX_BUFFER_SIZE  0X20    ;RX BUFFER SIZE
#DEFINE UARTINT_TX_BUFFER_SIZE  0X20    ;TX BUFFER SIZE
#DEFINE HOST_BUFFER_SIZE    0X30    	;HOST BUFFER SIZE
#DEFINE RESPONSE_BUFFER_SIZE    0X20    ;RESPONSE BUFFER SIZE
#DEFINE PARAMETER_BUFFER_SIZE   0X35    ;SYSTEM PARAMETER BUFFER SIZE

;****************************************************************************
;COMMUNICATION CONTROL CHARACTERS - BYTE SIZE (DO NOT CHANGE)
;****************************************************************************
#DEFINE STX                 0X02
#DEFINE DLE                 0X04
#DEFINE ETX                 0X03
#DEFINE HOST_ACK            0X06

;****************************************************************************
;SMHOSTFLAGS BIT DEFINITION
;****************************************************************************
#DEFINE SKIP_DLE            1
;****************************************************************************
;OTHER CONSTANTS
;****************************************************************************
;----------------------------------------------------------
;#define BLDC_MOTOR_CONTROL_BOARD	;uncomment if older version of the board is used
#define PICDEM_MC_LV				;uncomment if the release version of PICDEM LV is used.
;----------------------------------------------------------

#ifdef	PICDEM_MC_LV
#DEFINE LED1		0X05    ;RC0
#DEFINE LED2		0X04    ;RC2
#DEFINE LED3		0X03    ;RC3
#DEFINE SW3		0X00    ;RC4 - START/STOP SWITCH
#DEFINE SW2		0X02    ;RC5 - DIRECTION SWITCH
#endif

#ifdef	BLDC_MOTOR_CONTROL_BOARD
#DEFINE LED1		0X00    ;RC0
#DEFINE LED2		0X02    ;RC2
#DEFINE LED3		0X03    ;RC3
#DEFINE SW3		0X04    ;RC4 - START/STOP SWITCH
#DEFINE SW2		0X05    ;RC5 - DIRECTION SWITCH
#endif

#DEFINE TMR1L_COUNT	0XCB    ;COUNT IN TMR1 FOR 5msec INT
#DEFINE TMR1H_COUNT	0XF3    ;COUNT IN TMR1 FOR 5msec INT
#DEFINE MASKBYTE	0X07    ;FOR LED BLINK RATE
;****************************************************************************
;KEY (SWITCH) PIN DEFINITION (DO NOT CHANGE)
;****************************************************************************
#ifdef	PICDEM_MC_LV
#DEFINE KEY_PORT	PORTC
#DEFINE RUN_STOP_KEY	0	;NAME ON PICDEM LV MC BOARD - SW3
#DEFINE FWD_REV_KEY	2	;NAME ON PICDEM LV MC BOARD - SW2
#define TRISC_VALUE	b'11000111'
#endif
#ifdef	BLDC_MOTOR_CONTROL_BOARD
#DEFINE KEY_PORT	PORTC
#DEFINE RUN_STOP_KEY	4	;NAME ON PICDEM LV MC BOARD - SW3
#DEFINE FWD_REV_KEY	5	;NAME ON PICDEM LV MC BOARD - SW2
#define TRISC_VALUE	b'11110010'
#endif
#DEFINE DEBOUNCE_COUNT	0XFF
;****************************************************************************
;GUI_FLAGS BIT DEFINITION (DO NOT CHANGE)
;****************************************************************************
#DEFINE GUI_ON          	0   ;0 = GUI NOT ON, 1 = GUI ON
#DEFINE GUI_TIME_OUT            1   ;0 = NO TIME OUT YET, 1 = GUI TIME OUT
#DEFINE COM_IN_PROGRESS         2   ;0 = COMMUNICATION NOT IN PROGRESS, 1 = COMMUNICATION IN PROGRESS
#DEFINE COMMUNICATION_ERROR     3   ;0 = NO COMMUNICATION ERROR, 1 = COMMUNICATION ERROR
#DEFINE GUI_CONTROL_1ST_TIME    4   ;0 = NOT 1ST TIME, 1 = 1ST TIME

;****************************************************************************
;VUARTINTSTATUS BIT DEFINITION (DO NOT CHANGE)
;****************************************************************************
#DEFINE UARTINT_TX_BUF_FUL  0   ;TX BUFFER FULL FLAG, 0 = NOT FULL, 1 = FULL
#DEFINE UARTINT_RX_BUF_FUL  1   ;RX BUFFER FULL FLAG, 0 = NOT FULL, 1 = FULL
#DEFINE UARTINT_RX_BUF_EMPTY    2   ;RX BUFFER EMPTY FLAG, 0 = NOT EMPTY, 1 = EMPTY
#DEFINE UARTINT_RX_ERROR    3   ;RX ERROR FLAG, 0 = NO ERROR, 1 = ERROR
#DEFINE UARTINT_RX_BUF_OF   4   ;RX BUFFER OVER FLOW FLAG, 0 = NO OVER FLOW, 1 = OVER FLOW

;****************************************************************************
;VHOSTRESPONSEBUFSTATUS BIT DEFINITION (DO NOT CHANGE)
;****************************************************************************
#DEFINE HOST_BUF_FUL        0   ;HOST BUFFER FULL FLAG, 0 = NOT FULL, 1 = FULL
#DEFINE HOST_BUF_EMPTY      1   ;HOST BUFFER EMPTY FLAG, 0 = NOT EMPTY, 1 = EMPTY
#DEFINE HOST_BUF_OF     2   ;HOST BUFFER OVER FLOW FLAG, 0 = NO OVER FLOW, 1 = OVER FLOW
#DEFINE RESPONSE_BUF_FUL    3   ;HOST BUFFER FULL FLAG, 0 = NOT FULL, 1 = FULL
#DEFINE RESPONSE_BUF_EMPTY  4   ;HOST BUFFER EMPTY FLAG, 0 = NOT EMPTY, 1 = EMPTY
#DEFINE RESPONSE_BUF_OF     5   ;HOST BUFFER OVER FLOW FLAG, 0 = NO OVER FLOW, 1 = OVER FLOW

;****************************************************************************
;GPR DELAY PARAMETERS
;****************************************************************************
#DEFINE DELAY_COUNT1        0XFF
#DEFINE DELAY_COUNT2        0XFF
#DEFINE CHANGE_OVER_DELAY   0X30    ;TOTAL DELAY ~ CHANGE_OVER_DELAY * 39MS

;****************************************************************************
;CONSTANTS FOR FAULT CHECK AND DISPLAY
;****************************************************************************
#DEFINE CYCLE_COUNT_MAX     0X13    ;SETS FAULT DISPLAY BLINK RATE
#DEFINE MAX_FAULT_CHECK_COUNT   0X09    ;SETS TIME WINDOW FOR FAULT CHECK
#DEFINE MAX_FLTA_COUNT      0X01    ;LIMIT COUNT FOR FLTA (OC) IN 1 TIME WINDOW
#DEFINE MAX_FLTB_COUNT      0X05    ;LIMIT COUNT FOR FLTB (OV) IN 1 TIME WINDOW
#DEFINE MAX_JUNCTION_TEMP   0X80    ;LIMIT COUNT FOR OT
#define MAX_TEMP_FILT_COUNT     D'50'    ;Filter Count for Temperature

;****************************************************************************
;BLDC_FLAGS DEFINITION
;****************************************************************************
#DEFINE HALL_FLAG       0
#DEFINE PARAM_DISPLAY       1
#DEFINE POSITION_BIT        2
#DEFINE VELOCITY_READY      3
#DEFINE NEGATIVE_ERROR      4
#DEFINE CALC_PWM        5
#DEFINE VALID_HALL      6

;****************************************************************************
;BLDC_FLAGS_1 DEFINITION (DO NOT CHANGE)
;****************************************************************************
#DEFINE FWD_REV         0
#DEFINE DEBOUNCE        1
#DEFINE KEY_RS          2
#DEFINE KEY_FR          3
#DEFINE KEY_PRESSED     4
#DEFINE RUN_STOP        5
#DEFINE TMR1_OF         6
#DEFINE HW_CONTOL_1ST_TIME  7   ;0 = NOT 1ST TIME, 1 = 1ST TIME

;****************************************************************************
;DEFINE POR DIRECTION
;****************************************************************************
;#DEFINE    DIRECTION_AT_POR    0   ;POR DIRECTION = REVERSE
#DEFINE DIRECTION_AT_POR    1   ;POR DIRECTION = FORWARD

;****************************************************************************
;FLT_FLAGS DEFINITION (DO NOT CHANGE)
;****************************************************************************
#DEFINE OCUR            0
#DEFINE OVOLT           1
#DEFINE OTEMP           2

;****************************************************************************
;FLT_FLAGS_1 DEFINITION
;****************************************************************************
#DEFINE FAULT		0
#DEFINE CLEAR_LED_ONCE	1

;****************************************************************************
;BIT DEFINITION FOR SYSTEM PARAMETERS (DO NOT CHANGE)
;****************************************************************************
#DEFINE HALL_ANGLE_BIT      3
#DEFINE PWM_ALIGNMENT       0

;****************************************************************************
;LOCATION DETAILS FOR STORING SYSTEM PARAMETERS FROM GUI (DO NOT CHANGE)
;****************************************************************************
#DEFINE LOC_BUFFER_CHKSUM_1         0X00
#DEFINE LOC_BUFFER_CHKSUM_2         0X01
#DEFINE LOC_MOTOR_VOLTAGE_LSB           0X02
#DEFINE LOC_MOTOR_CURRENT           0X03
#DEFINE LOC_MOTOR_FREQUENCY         0X04
#DEFINE LOC_MOTOR_SPEED_RPM_LSB         0X05
#DEFINE LOC_MOTOR_SPEED_RPM_MSB         0X06
#DEFINE LOC_MOTOR_POLE              0X07
#DEFINE LOC_FEEDBACK_DEVICE         0X08
#DEFINE LOC_ENCODER_PPR_LSB         0X09
#DEFINE LOC_ENCODER_PPR_MSB         0X0A
#DEFINE LOC_VOLTAGE_LIMIT_LSB           0X0B
#DEFINE LOC_LIMIT_CURRENT           0X0C
#DEFINE LOC_SPEED_LIMIT_LSB         0X0D
#DEFINE LOC_SPEED_LIMIT_MSB         0X0E
#DEFINE LOC_SPEED_LOOP_KP           0X0F
#DEFINE LOC_SPEED_LOOP_KI           0X10
#DEFINE LOC_SPEED_LOOP_KD           0X11
#DEFINE LOC_CURRENT_LOOP_KP         0X12
#DEFINE LOC_CURRENT_LOOP_KI         0X13
#DEFINE LOC_CURRENT_LOOP_KD         0X14
#DEFINE LOC_ACCELERATION_RATE           0X15
#DEFINE LOC_DECELERATION_RATE           0X16
#DEFINE LOC_INPUT_VOLTAGE_LSB           0X17
#DEFINE LOC_PWM_TYPE_FREQ           0X18
#DEFINE LOC_MOTOR_SPEED_RPS         0X19
#DEFINE LOC_PWM_DC_MAX_LSB          0X1A
#DEFINE LOC_PWM_DC_MAX_MSB          0X1B
#DEFINE LOC_PTPER_LSB               0X1C
#DEFINE LOC_PTPER_MSB               0X1D
#DEFINE LOC_RPS_PWM_CONSTANT_LSB        0X1E
#DEFINE LOC_RPS_PWM_CONSTANT_MSB        0X1F
#DEFINE LOC_RPM_PWM_CONSTANT            0X20
#DEFINE LOC_ERROR_PWM_CONSTANT_LSB      0X21
#DEFINE LOC_ERROR_PWM_CONSTANT_MSB      0X22
#DEFINE LOC_ESTIMATE_TIMER_CONSTANT_LSB     0X23
#DEFINE LOC_ESTIMATE_TIMER_CONSTANT_MSB     0X24
#DEFINE LOC_RPM_CONSTANT_QEI_LSB        0X25
#DEFINE LOC_RPM_CONSTANT_QEI_MSB        0X26
#DEFINE LOC_RPM_CONSTANT_HALL_LSB       0X27
#DEFINE LOC_RPM_CONSTANT_HALL_MSB       0X28
#DEFINE LOC_V_MSB_TYPE              0X29
#DEFINE LOC_RESERVED_FOR_FUTURE_0       0X2A
#DEFINE LOC_RESERVED_FOR_FUTURE_1       0X2B
;****************************************************************************
;LOCATION DETAILS FOR STORING RUN TIME PARAMETERS (DO NOT CHANGE)
;****************************************************************************
#DEFINE LOC_REQD_SPEED_RPM_LSB          0X2C
#DEFINE LOC_REQD_SPEED_RPM_MSB          0X2D
#DEFINE LOC_REQD_SPEED_RPS          0X2E
#DEFINE LOC_CAPBUF_CNT_LSB          0X2F
#DEFINE LOC_CAPBUF_CNT_MSB          0X30
#DEFINE LOC_FUTURE_USE_0            0X31
#DEFINE LOC_FUTURE_USE_1            0X32
#DEFINE LOC_FUTURE_USE_2            0X33
#DEFINE LOC_FUTURE_USE_3            0X34

;****************************************************************************
;TOTAL SYSTEM PARAMETER COUNT (DO NOT CHANGE)
;****************************************************************************
#DEFINE SYSTEM_PARAMETER_COUNT          0X2C

;****************************************************************************
;GUI COMMAND DEFINITION (DO NOT CHANGE)
;****************************************************************************
#DEFINE COMMAND_STOP                0X11
#DEFINE COMMAND_RUN             0X12
#DEFINE COMMAND_FREQ                0X13
#DEFINE COMMAND_DIRECTION           0X14
#DEFINE COMMAND_ASK_DATA            0X15
#DEFINE COMMAND_SYSTEM_PARAMETERS       0X16

;****************************************************************************
;COMMUTATION TABLE. LOADED TO OVDCOND
;****************************************************************************
; THE HALL SENSOR MAKES A TRANSITION EVERY 60 DEGREES
; FOR OTHER TYPE OF MOTORS WITH DIFFERENT HALL SENSOR COMBINATION, THE TABLE HAS TO BE MODIFIED.
;****************************************************************************
#DEFINE	DUMMY_POSITION	B'00000000'
#DEFINE	POSITION1	B'00010010'
#DEFINE	POSITION2	B'00100100'
#DEFINE	POSITION3	B'00000110'
#DEFINE	POSITION4	B'00001001'
#DEFINE	POSITION5	B'00011000'
#DEFINE	POSITION6	B'00100001'

