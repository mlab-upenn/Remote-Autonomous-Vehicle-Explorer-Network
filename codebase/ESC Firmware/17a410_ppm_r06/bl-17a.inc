;*********************************************************** 
;* BL-17A                                                  *
;* Dez.2006
;* http://home.versanet.de/~b-konze/                       *
;*********************************************************** 

#define Typ "bl-17a"

; activate Motor brake 

.equ MOT_BRAKE      = 0

;Select Brake Power
;	1 = min Brake
;	2 = medium Brake
;  	3 = hard Brake

.equ BrakeSelect    = 2

.equ CALIBRATE			= 1      
.equ OSCAL_VALUE		= 0x01ff 	; eeprom position

.equ FORCE_LIPO 	= 1			; 1=Lipo und 0=NC wenn keine Buecke
.equ ActiveFreeRun	= 0

;**** **** **** **** ****
; PORT  definitions
;**** **** **** **** ****

;*********************
; PORT D 
;*********************  
.equ	BpFET		= 7	;o
.equ	c_comp		= 6	;i common comparator input (AIN0)
.equ	ApFET		= 5	;o
.equ	CpFET		= 4	;o
.equ	CnFET		= 3	;o
.equ	rcp_in		= 2	;i r/c pulse input
.equ	BnFET		= 1	;o
.equ	AnFET		= 0	;o
.equ	INIT_PD		= 0

.equ	DIR_PD		= (1<<ApFET)+(1<<BpFET)+(1<<CpFET)+(1<<AnFET)+(1<<BnFET)+(1<<CnFET)
.equ	BRAKE_PD	= (1<<AnFET)+(1<<BnFET)+(1<<CnFET)

#define ApFET_on	sbi	PORTD,5
#define ApFET_off	cbi	PORTD,5
#define BpFET_on	sbi	PORTD,7
#define BpFET_off	cbi	PORTD,7
#define CpFET_on	sbi	PORTD,4
#define CpFET_off	cbi	PORTD,4
#define AnFET_on	sbi	PORTD,0
#define AnFET_off	cbi	PORTD,0
#define BnFET_on	sbi	PORTD,1
#define BnFET_off	cbi	PORTD,1
#define CnFET_on	sbi	PORTD,3
#define CnFET_off	cbi	PORTD,3


;*********************
; PORT C definitions *
;*********************
;.equ			= 0	; ADC0
.;equ    	  	= 1	; ADC1
.equ	mux_a		= 2	; ADC2 phase input
.equ    mux_b   	= 3     ; ADC3 phase input 
.equ	mux_c		= 4	; ADC4 phase input
;.equ    	  	= 5	; ADC5

;*********************
; ADC only
;*********************
;.equ    	  	= 6	; ADC6
.equ 	ACCU_MUX	= 7 	; ADC7 voltage control input 

.equ	INIT_PC		= 0
.equ	DIR_PC		= 0
.equ	BRAKE_PC	= 0


;*********************
; PORT B definitions *
;*********************
;.equ		= 7	
;.equ		= 6	
;.equ		= 5    	(sck stk200 interface) 
;.equ		= 4	(miso stk200 interface)
;.equ		= 3	(mosi stk200 interface) 
;.equ		= 2	
;.equ		= 1	
;.equ		= 0	; connected with pb4 ???	

.equ	INIT_PB		= 0
.equ	DIR_PB		= 0
.equ	BRAKE_PB	= 0

