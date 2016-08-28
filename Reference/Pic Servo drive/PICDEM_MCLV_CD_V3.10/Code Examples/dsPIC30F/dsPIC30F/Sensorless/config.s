;*******************************************************************
; CONFIG settings
;*******************************************************************
          .include "general.inc"

; Clock switching off, Fail Safe off, XT clock
; if the clock source is 7.3728Mhz then use XT_PLL4 as below

  ;config __FOSC, CSW_FSCM_OFF & XT_PLL4

;if the clock source is 5.00Mhz then use XT_PLL8 as below

	config __FOSC, CSW_FSCM_OFF & XT_PLL8 

          .end

