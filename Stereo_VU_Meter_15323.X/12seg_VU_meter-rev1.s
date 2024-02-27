;=========================================================================================
;	TITLE	12 segment Stereo VU Meter rev.1C/1CTH
;
;	Copyright 2023 Akimitsu Sadoi - The LED Artist - http://theLEDart.com
;	The use and distribution of this code without explicit permission is prohibited.
;-----------------------------------------------------------------------------------------
;	[Version history]
;	0.0	converted from the PIC16F1823 version 0.34
;	0.1 reversible LED order (LED1 becomes LED12) and L/R channel flip
;	0.1A use all 10 bits of ADC results and sum in 16 bits
;	0.1B ADC input offset compensation (fixed value)
;	0.1C offset voltage calibration
;	0.1D ADC reference voltage changed to 2.048V from 1.024V
;	1.0	Production version
;
;	[To Do]
;	- needs better way to enter offset calibration
;	- visual feedback when config change happens
;
;	[Issues]
;	
;---------------------------------------------------------------------------------------------------
;	Use additional options in pic-as: -Wa,-a -Wl,-DCODE=2
;=========================================================================================
	PROCESSOR 16F15323
	RADIX	DEC

;===================================================================================================
; PIC16F15323 Configuration Bit Settings

; Assembly source line config statements

; CONFIG1
  CONFIG  FEXTOSC = OFF         ; External Oscillator mode selection bits (Oscillator not enabled)
  CONFIG  RSTOSC = HFINTPLL     ; Power-up default value for COSC bits (HFINTOSC with 2x PLL, with OSCFRQ = 16 MHz and CDIV = 1:1 (FOSC = 32 MHz))
  CONFIG  CLKOUTEN = OFF        ; Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
  CONFIG  CSWEN = ON            ; Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)

; CONFIG2
  CONFIG  MCLRE = OFF           ; Master Clear Enable bit (MCLR pin function is port defined function)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  LPBOREN = OFF         ; Low-Power BOR enable bit (ULPBOR disabled)
  CONFIG  BOREN = NSLEEP        ; Brown-out reset enable bits (Brown-out Reset enabled while running, disabled in sleep; SBOREN is ignored)
  CONFIG  BORV = HI             ; Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) is set to 2.7V)
  CONFIG  ZCD = OFF             ; Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
  CONFIG  PPS1WAY = ON          ; Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
  CONFIG  STVREN = ON           ; Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

; CONFIG3
  CONFIG  WDTCPS = WDTCPS_31    ; WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
  CONFIG  WDTE = SWDTEN         ; WDT operating mode (WDT enabled/disabled by SWDTEN bit in WDTCON0)
  CONFIG  WDTCWS = WDTCWS_7     ; WDT Window Select bits (window always open (100%); software control; keyed access not required)
  CONFIG  WDTCCS = LFINTOSC     ; WDT input clock selector (WDT reference clock is the 31.0kHz LFINTOSC output)

; CONFIG4
  CONFIG  BBSIZE = BB512        ; Boot Block Size Selection bits (512 words boot block size)
  CONFIG  BBEN = OFF            ; Boot Block Enable bit (Boot Block disabled)
  CONFIG  SAFEN = ON            ; SAF Enable bit (SAF enabled)
  CONFIG  WRTAPP = OFF          ; Application Block Write Protection bit (Application Block not write protected)
  CONFIG  WRTB = OFF            ; Boot Block Write Protection bit (Boot Block not write protected)
  CONFIG  WRTC = ON             ; Configuration Register Write Protection bit (Configuration Register write protected)
  CONFIG  WRTSAF = OFF          ; Storage Area Flash Write Protection bit (SAF not write protected)
  CONFIG  LVP = OFF             ; Low Voltage Programming Enable bit (High Voltage on MCLR/Vpp must be used for programming)

; CONFIG5
  CONFIG  CP = OFF              ; UserNVM Program memory code protection bit (UserNVM code protection disabled)

// config statements should precede project file includes.
#include <xc.inc>

;=========================================================================================
;	constants

;--- debugging -------------------------------------------------------
;#define		DEBUG		DEBUG		; uncomment for debug

DEBUG_OUT_H		MACRO
				banksel	DEBUG_TRIS
				bsf		DEBUG_TRIS,DEBUG_PIN
ENDM

DEBUG_OUT_L		MACRO
				banksel	DEBUG_TRIS
				bcf		DEBUG_TRIS,DEBUG_PIN
ENDM

DEBUG_PORT		equ		PORTA
DEBUG_LAT		equ		LATA
DEBUG_PIN		equ		1
DEBUG_TRIS		equ		TRISA
DEBUG_WPU		equ		WPUA
DEBUG_ODCON		equ		ODCONA

;--- NVM config data save --------------------------------------------
; config data is saved in Storage Area Flash (the last 128 words of the flash)
												; gflags bits that needs saved
CONFIG_SAVE_FLAGS	equ	(1<<DOT_DS_POS)|(1<<PEAK_DISP_POS)|(1<<LED_REV_L_POS)|(1<<LED_REV_R_POS)|(1<<CH_FLIP_POS)
												; default setting
CONFIG_DEFAULT		equ	(0<<DOT_DS_POS)|(1<<PEAK_DISP_POS)|(0<<LED_REV_L_POS)|(0<<LED_REV_R_POS)|(0<<CH_FLIP_POS)

OFFSET_L		equ		10						; default ADC offset compensation value
OFFSET_R		equ		10						; default ADC offset compensation value

;--- timebase --------------------------------------------------------
SYSTICK_INC		equ		2						; 256 / (systick_us / timer0_pr)
SYSTICK_MS		equ		8						; main loop duration in milliseconds
DECAY_SPEED		equ		64						; peak hold decay speed
SCAN_STEPS		equ		NUM_OF_LED+2			; extra above NUM_OF_LED will display peak levels
												; ADC samples will accumulate (value x SCAN_STEPS/2)
CONFIG_SAVE_DLY	equ		(2040/SYSTICK_MS)		; config save delay in system ticks (MAX:2040, zero disables config save)

;--- VU level steps/scales -------------------------------------------
;* uncomment one of the following VU step scales *

;#define		VU_SCALE30DB			; 30 db default VU scale
;#define		VU_SCALE32DB			; 32 db range
;#define		VU_SCALE33DB			; 33 db range (LM3916 extended)
#define		VU_SCALE36DB			; 36 db range (least error)
;#define		VU_SCALE11DB			; 11 db - 1 db step for calibration

;--- VU meter voltage inputs -----------------------------------------
VU_IN_L_PORT	equ		PORTC
VU_IN_L_PIN		equ		0
VU_IN_L_ANS		equ		ANSELC
VU_IN_L_AIN		equ		010000B

VU_IN_R_PORT	equ		PORTC
VU_IN_R_PIN		equ		1
VU_IN_R_ANS		equ		ANSELC
VU_IN_R_AIN		equ		010001B

;--- Button SW -------------------------------------------------------
BTN1_PORT		equ		PORTA
BTN1_ANS		equ		ANSELA
BTN1_WPU		equ		WPUA					; needs pull-up
BTN1_PIN		equ		3

BTN2_PORT		equ		PORTA
BTN2_ANS		equ		ANSELA
BTN2_WPU		equ		WPUA					; needs pull-up
BTN2_PIN		equ		1

BTN_DEB_CNT		equ		(56/(SYSTICK_MS))		; button debounce count
BTN_HOLD_CNT	equ		(2000/(SYSTICK_MS))		; button switch hold time

;--- LEDs ------------------------------------------------------------
LED1_LAT		equ		LATA
LED1_PIN		equ		2
LED2_LAT		equ		LATA
LED2_PIN		equ		0
LED3_LAT		equ		LATA
LED3_PIN		equ		5
LED4_LAT		equ		LATA
LED4_PIN		equ		4

LED5_LAT		equ		LATC
LED5_PIN		equ		5 ; 4
LED6_LAT		equ		LATC
LED6_PIN		equ		4 ; 3
LED7_LAT		equ		LATC
LED7_PIN		equ		3 ; 2
LED8_LAT		equ		LATC
LED8_PIN		equ		2 ; 1

;-- LED level indicator matrix ---
NUM_OF_LED		equ		12

; TRISx/LATx set/mask bits to clear all LEDs
LED_CLR_TRIA	equ		(1<<LED1_PIN)|(1<<LED2_PIN)|(1<<LED3_PIN)|(1<<LED4_PIN)
LED_CLR_LATA	equ		~(LED_CLR_TRIA)
LED_CLR_TRIC	equ		(1<<LED5_PIN)|(1<<LED6_PIN)|(1<<LED7_PIN)|(1<<LED8_PIN)
LED_CLR_LATC	equ		~(LED_CLR_TRIC)

; LATx/TRISx set/mask bits for each LED
;--- left ch ---
LED1_LATA		equ		(1<<LED1_PIN)				; LATA set bits
LED1_TRIA		equ		~((1<<LED1_PIN)|(1<<LED2_PIN))	; TRISA mask bits
LED2_LATA		equ		(1<<LED1_PIN)				; LATA set bits
LED2_TRIA		equ		~((1<<LED1_PIN)|(1<<LED3_PIN))	; TRISA mask bits
LED3_LATA		equ		(1<<LED1_PIN)				; LATA set bits
LED3_TRIA		equ		~((1<<LED1_PIN)|(1<<LED4_PIN))	; TRISA mask bits

LED4_LATA		equ		(1<<LED2_PIN)				; LATA set bits
LED4_TRIA		equ		~((1<<LED2_PIN)|(1<<LED1_PIN))	; TRISA mask bits
LED5_LATA		equ		(1<<LED2_PIN)				; LATA set bits
LED5_TRIA		equ		~((1<<LED2_PIN)|(1<<LED3_PIN))	; TRISA mask bits
LED6_LATA		equ		(1<<LED2_PIN)				; LATA set bits
LED6_TRIA		equ		~((1<<LED2_PIN)|(1<<LED4_PIN))	; TRISA mask bits

LED7_LATA		equ		(1<<LED3_PIN)				; LATA set bits
LED7_TRIA		equ		~((1<<LED3_PIN)|(1<<LED1_PIN))	; TRISA mask bits
LED8_LATA		equ		(1<<LED3_PIN)				; LATA set bits
LED8_TRIA		equ		~((1<<LED3_PIN)|(1<<LED2_PIN))	; TRISA mask bits
LED9_LATA		equ		(1<<LED3_PIN)				; LATA set bits
LED9_TRIA		equ		~((1<<LED3_PIN)|(1<<LED4_PIN))	; TRISA mask bits

LED10_LATA		equ		(1<<LED4_PIN)				; LATA set bits
LED10_TRIA		equ		~((1<<LED4_PIN)|(1<<LED1_PIN))	; TRISA mask bits
LED11_LATA		equ		(1<<LED4_PIN)				; LATA set bits
LED11_TRIA		equ		~((1<<LED4_PIN)|(1<<LED2_PIN))	; TRISA mask bits
LED12_LATA		equ		(1<<LED4_PIN)				; LATA set bits
LED12_TRIA		equ		~((1<<LED4_PIN)|(1<<LED3_PIN))	; TRISA mask bits

;--- right ch ---
LED13_LATC		equ		(1<<LED5_PIN)				; LATC set bits
LED13_TRIC		equ		~((1<<LED5_PIN)|(1<<LED6_PIN))	; TRISC mask bits
LED14_LATC		equ		(1<<LED5_PIN)				; LATC set bits
LED14_TRIC		equ		~((1<<LED5_PIN)|(1<<LED7_PIN))	; TRISC mask bits
LED15_LATC		equ		(1<<LED5_PIN)				; LATC set bits
LED15_TRIC		equ		~((1<<LED5_PIN)|(1<<LED8_PIN))	; TRISC mask bits

LED16_LATC		equ		(1<<LED6_PIN)				; LATC set bits
LED16_TRIC		equ		~((1<<LED6_PIN)|(1<<LED5_PIN))	; TRISC mask bits
LED17_LATC		equ		(1<<LED6_PIN)				; LATC set bits
LED17_TRIC		equ		~((1<<LED6_PIN)|(1<<LED7_PIN))	; TRISC mask bits
LED18_LATC		equ		(1<<LED6_PIN)				; LATC set bits
LED18_TRIC		equ		~((1<<LED6_PIN)|(1<<LED8_PIN))	; TRISC mask bits

LED19_LATC		equ		(1<<LED7_PIN)				; LATC set bits
LED19_TRIC		equ		~((1<<LED7_PIN)|(1<<LED5_PIN))	; TRISC mask bits
LED20_LATC		equ		(1<<LED7_PIN)				; LATC set bits
LED20_TRIC		equ		~((1<<LED7_PIN)|(1<<LED6_PIN))	; TRISC mask bits
LED21_LATC		equ		(1<<LED7_PIN)				; LATC set bits
LED21_TRIC		equ		~((1<<LED7_PIN)|(1<<LED8_PIN))	; TRISC mask bits

LED22_LATC		equ		(1<<LED8_PIN)				; LATC set bits
LED22_TRIC		equ		~((1<<LED8_PIN)|(1<<LED5_PIN))	; TRISC mask bits
LED23_LATC		equ		(1<<LED8_PIN)				; LATC set bits
LED23_TRIC		equ		~((1<<LED8_PIN)|(1<<LED6_PIN))	; TRISC mask bits
LED24_LATC		equ		(1<<LED8_PIN)				; LATC set bits
LED24_TRIC		equ		~((1<<LED8_PIN)|(1<<LED7_PIN))	; TRISC mask bits


;--- global flags ----------------------------------------------------

DOT_DS_POS		equ		0						; place config bits at LSb
PEAK_DISP_POS	equ		1
LED_REV_L_POS	equ		2
LED_REV_R_POS	equ		3
CH_FLIP_POS		equ		4
SYSTICK_POS		equ		5
ADC_CH_POS		equ		6
LED_NO_UPD_POS	equ		7

#define	SYSTICK			gflags,SYSTICK_POS		; systick flag
#define	ADC_CH			gflags,ADC_CH_POS		; VU channel being digitized (0:Left, 1:Right)
#define	DOT_DS			gflags,DOT_DS_POS		; display dot style
#define	PEAK_DISP		gflags,PEAK_DISP_POS	; peak level display on
#define LED_NO_UPD		gflags,LED_NO_UPD_POS	; stop LED bar update
#define	LED_REV_L		gflags,LED_REV_L_POS	; reverse Left channel LED order
#define	LED_REV_R		gflags,LED_REV_R_POS	; reverse Right channel LED order
#define	CH_FLIP			gflags,CH_FLIP_POS		; L/R channel flip


OFFSET_CAL_POS	equ		0

#define	OFFSET_CAL		mflags,OFFSET_CAL_POS	; input offset calibration


;--- button event flags ----------------------------------------------

BTN1_EVENT_POS	equ		0						; button event happened
BTN1_PUSH_POS	equ		1						; button short pushed
BTN1_HELD_POS	equ		2						; button held long enough
BTN1_LPUSH_POS	equ		3						; button long pushed
BTN2_EVENT_POS	equ		4						; button event happened
BTN2_PUSH_POS	equ		5						; button short pushed
BTN2_HELD_POS	equ		6						; button held long enough
BTN2_LPUSH_POS	equ		7						; button long pushed

#define	BTN1_EVENT		bflags,BTN1_EVENT_POS	; button event flag
#define	BTN1_PUSH		bflags,BTN1_PUSH_POS	; button pushed flag
#define	BTN1_HELD		bflags,BTN1_HELD_POS	; button held flag
#define	BTN1_LPUSH		bflags,BTN1_LPUSH_POS	; button long pushed flag
#define	BTN2_EVENT		bflags,BTN2_EVENT_POS	; button event flag
#define	BTN2_PUSH		bflags,BTN2_PUSH_POS	; button pushed flag
#define	BTN2_HELD		bflags,BTN2_HELD_POS	; button held flag
#define	BTN2_LPUSH		bflags,BTN2_LPUSH_POS	; button long pushed flag


;=========================================================================================
;	variables
;
PSECT	udata_bank0
	;--- bank 0 (80 bytes) -----------------------------------------------------
	_ram_start_:						; - clear variables from here -

	decay_speed_acc:	DS	1			; peak value decay speed accumulator
	decay_timer_L:		DS	1			; peak value decay timer
	decay_timer_R:		DS	1			; peak value decay timer

	adc_result_LH:		DS	1			; vu value L high byte
	adc_result_RH:		DS	1			; vu value R high byte
	adc_offset_L:		DS	1			; analog input offset value L
	adc_offset_R:		DS	1			; analog input offset value R

	bflags:				DS	1			; button flags
	btn1_deb_cnt:		DS	1			; button debounce counter
	btn1_hold_cnt:		DS	1			; button hold counter
	btn2_deb_cnt:		DS	1			; button debounce counter
	btn2_hold_cnt:		DS	1			; button hold counter

	config_save_timer:	DS	1			; timer to delay config write to NVM

	swap_temp:			DS	1			; swap space
	bar_level_temp:		DS	1			; working variable
	loop_cnt:			DS	1			; general purpose loop counter
	sample_cnt:			DS	1

PSECT	udata_shr
	;--- common memory (16 bytes) ----------------------------------------------

	gflags:				DS	1			; global flags
	mflags:				DS	1			; mode/option flags
	systick:			DS	1			; system tick accumulator

	adc_result_L:		DS	1			; vu value L low byte
	adc_result_R:		DS	1			; vu value R low byte
	adc_peak_L:			DS	1			; peak value L
	adc_peak_R:			DS	1			; peak value R

	bar_level1:			DS	1			; level indicator bar level (0 - NUM_OF_LED)
	bar_level2:			DS	1			; level indicator bar level (0 - NUM_OF_LED)

	analog_val:			DS	1			; analog value to be converted to step levels
	vu_level:			DS	1			; VU level (0 - NUM_OF_LED) converted

	vu_level_L:			DS	1			; VU level (0 - NUM_OF_LED) used by LED driver
	vu_level_R:			DS	1			; VU level (0 - NUM_OF_LED) used by LED driver
	peak_level_L:		DS	1			; Peak level (0 - NUM_OF_LED) used by LED driver
	peak_level_R:		DS	1			; Peak level (0 - NUM_OF_LED) used by LED driver
	scan_step:			DS	1			; used by LED driver

	_ram_end_:

PSECT	udata_bank1
	; --- bank 1 (32 bytes) ----------------------------------------------------


;=========================================================================================
; Reset Vector
;
PSECT   ResetVec,class=CODE,space=SPACE_CODE,delta=2,abs
ORG	0
GLOBAL  ResetVector
ResetVector:
				nop
				movlw   HIGH main
				movwf   PCLATH
				goto	main
;
;=========================================================================================
;   Interrupt vector and handler
ORG	4
IsrVec:
;				----------------------------------------------------------------
;				STATUS and other registers automatically saved by hardware
;				----------------------------------------------------------------
#ifdef	DEBUG
				DEBUG_OUT_L			; set debug pin low
#endif
				;===============================================================
				; Timer 0 service - called every 64 us

				btfsc	LED_NO_UPD				; if (LED_NO_UPD == 1)
				bra		LED_scan_skip			;   no LED bar update

				;--- display peak level ------------------------------
				btfss	PEAK_DISP				; if (PEAK_DISP == 1)
				bra		peak_disp_skip
				movf	scan_step,W				; and if (scan_step > NUM_LED)
				sublw	NUM_OF_LED
				btfsc	CARRY
				bra		peak_disp_skip			; {
				movf	peak_level_L,W			;   bar_level1 = peak_level_L
				movwf	bar_level1
				movf	peak_level_R,W			;   bar_level2 = peak_level_R
				movwf	bar_level2
				bra		LED_scan_cont
peak_disp_skip:									; } else

				;--- multiplex LED bar -------------------------------

				btfss	DOT_DS					; if (DOT_DS == 1)
				bra		LED_bargraph			; {
LED_dot:
				; // dot style display //
				movf	scan_step,W				;   if (vu_level_L != scan_step)
				subwf	vu_level_L,W
				movlw	0						;     bar_level1 = 0
				btfsc	ZERO					;   else
				movf	scan_step,W				;     bar_level1 = scan_step
				movwf	bar_level1

				movf	scan_step,W				;   if (vu_level_R != scan_step)
				subwf	vu_level_R,W
				movlw	0						;     bar_level2 = 0
				btfsc	ZERO					;   else
				movf	scan_step,W				;     bar_level2 = scan_step
				movwf	bar_level2
				bra		LED_scan_cont
LED_bargraph:									; } else {
				; // bar-graph style display //
				movf	scan_step,W				;   if (vu_level_L >= scan_step)
				subwf	vu_level_L,W
				movf	scan_step,W				;     bar_level1 = scan_step
				btfss	CARRY					;   else
				clrw							;     bar_level1 = 0
				movwf	bar_level1

				movf	scan_step,W				;   if (vu_level_R >= scan_step)
				subwf	vu_level_R,W			; 
				movf	scan_step,W				;     bar_level2 = scan_step
				btfss	CARRY					;   else
				clrw							;     bar_level2 = 0
				movwf	bar_level2
LED_scan_cont:									; }
				call	drive_LED_bar
LED_scan_skip:
				;--- read & start ADC --------------------------------
				banksel	ADCON0					; if ADC done
				btfsc	GOnDONE
				bra		read_adc_skip			; {
#ifdef	DEBUG
				DEBUG_OUT_H			; set debug pin high
				banksel	ADCON0
#endif
												;   --- sum up ADC result ---
				btfsc	ADC_CH					;   if (ADC_CH == 0) // Left ch data
				bra		read_adc_right			;   {
												;     set ADC input -> VU_IN_R_AIN, ADC on
				movlw	(VU_IN_R_AIN<<ADCON0_CHS0_POSN)|ADCON0_ADON_MASK
				movwf	ADCON0
				bsf		ADC_CH					;     set ADC_CH

				movf	ADRESL,W				;     adc_result_L += ADC Low Byte
				addwf	adc_result_L,F
				movf	ADRESH,W				;     adc_result_LH += ADC High bits + Carry
				banksel	adc_result_LH
				addwfc	adc_result_LH,F

				bra		read_adc_cont			;   } else			// Right ch data
read_adc_right:									;   {
												;     set ADC input -> VU_IN_L_AIN, ADC on
				movlw	(VU_IN_L_AIN<<ADCON0_CHS0_POSN)|ADCON0_ADON_MASK
				movwf	ADCON0
				bcf		ADC_CH					;     clear ADC_CH

				movf	ADRESL,W				;     adc_result_R += ADC Low Byte
				addwf	adc_result_R,F
				movf	ADRESH,W				;     adc_result_RH += ADC High bits + Carry
				banksel	adc_result_RH
				addwfc	adc_result_RH,F
read_adc_cont:									;   }
				banksel	ADCON0
				bsf		GOnDONE					;   start the next ADC conversion
#ifdef	DEBUG
				DEBUG_OUT_L			; set debug pin low
#endif
read_adc_skip:									; }
				;-----------------------------------------------------
				; offset calibration

				btfss	OFFSET_CAL				; if OFFSET_CAL flag is set
				bra		no_calibration
				movlw	SCAN_STEPS				; and if (scan_step == SCAN_STEPS)
				subwf	scan_step,W
				btfss	ZERO
				bra		calib_running			; {
				clrf	scan_step				;   scan_step = 0

				banksel	sample_cnt
				movf	sample_cnt,F			;   if (sample_cnt == 0)
				btfss	ZERO
				bra		calib_cont				;   {
				;--- start calibration ---
				movlw	0xFF					;     set sample counter (use 0xFF since 2^8 is not possible)
				movwf	sample_cnt				
				clrf	adc_result_L			;     clear adc_result_L
				clrf	adc_result_LH
				clrf	adc_result_R			;     clear adc_result_R
				clrf	adc_result_RH
				clrf	peak_level_L			;     clear peak_level
				clrf	peak_level_R
				movlw	1						;     set vu_level_L = 1 to lit the segment 1
				movwf	vu_level_L
				movwf	vu_level_R
				bra		calib_running
calib_cont:										;   } else {
				decfsz	sample_cnt,F			;     if (--sample_cnt == 0)
				bra		calib_running			;     { // finished sampling - calculate offset from accumulated samples
												;       // samples summed 2^8 times (almost),
												;       // so that upper byte of adc_result can be used as the avarage offset
				;--- update offset values ---
				banksel	adc_offset_L
				movf	adc_result_LH,W			;       adc_result_LH -> adc_offset_L
				movwf	adc_offset_L

				movf	adc_result_RH,W			;       adc_result_RH -> adc_offset_R
				movwf	adc_offset_R

				bcf		OFFSET_CAL				;       clear OFFSET_CAL flag to finish calibration
												;     }
												;   }
calib_running:									; }
				bra		LED_scan_cont4
no_calibration:

				;-----------------------------------------------------

				movlw	SCAN_STEPS-1			; if (scan_step == SCAN_STEPS-1)
				subwf	scan_step,W
				btfss	ZERO
				bra		LED_scan_cont2			; {

				;--- update VU level: L ---
				banksel	adc_offset_L
				movf	adc_offset_L,W			;   subtract offset compensation value from adc_result_L
				subwf	adc_result_L,F
				clrw
				subwfb	adc_result_LH,F
				btfsc	CARRY					;   if (adc_result_LH < 0)
				bra		adc_not_neg_L			;   {
				clrw							;     analog_val = adc_result_L:H = 0
				bra		adc_neg_L
adc_not_neg_L:									;   }
				lsrf	adc_result_LH,F			;   adc_result_L:H>>2
				rrf		adc_result_L,F
				lsrf	adc_result_LH,F			;   Zero flag set/cleared here
				rrf		adc_result_L,W
				btfss	ZERO					;   if (adc_result_LH != 0)
				movlw	0xFF					;     analog_val = 0xFF (adc_result overflow)
adc_neg_L:
				movwf	analog_val
				clrf	adc_result_L			;   clear adc_result_L
				clrf	adc_result_LH

				call	VU_convert_steps		;   convert to steps
				movf	vu_level,W				;   vu_level_L = vu_level
				movwf	vu_level_L
				;--- update peak level: L ---
				movf	analog_val,W
				subwf	adc_peak_L,W			;   compare analog_val : adc_peak
				btfsc	CARRY					;   if (adc_peak < analog_val)
				bra		peak_skip_L				;   {
				movf	analog_val,W			;     adc_peak_L = analog_val
				movwf	adc_peak_L
				banksel	decay_timer_L
				clrf	decay_timer_L			;     decay_timer_L = 0
peak_skip_L:									;   }
				;--- update VU level: R ---
				banksel	adc_offset_R
				movf	adc_offset_R,W			;   subtract offset compensation value from adc_result_L
				subwf	adc_result_R,F
				clrw
				subwfb	adc_result_RH,F
				btfsc	CARRY					;   if (adc_result_RH < 0)
				bra		adc_not_neg_R			;   {
				clrw							;     analog_val = adc_result_R:H = 0
				bra		adc_neg_R
adc_not_neg_R:									;   }
				lsrf	adc_result_RH,F			;   adc_result_R:H>>2
				rrf		adc_result_R,F
				lsrf	adc_result_RH,F			;   Zero flag set/cleared here
				rrf		adc_result_R,W
				btfss	ZERO					;   if (adc_result_RH != 0)
				movlw	0xFF					;     analog_val = 0xFF (adc_result overflow)
adc_neg_R:
				movwf	analog_val
				clrf	adc_result_R			;   clear adc_result_R
				clrf	adc_result_RH

				call	VU_convert_steps		;   convert to steps
				movf	vu_level,W				;   vu_level_R = vu_level
				movwf	vu_level_R
				;--- update peak level: R ---
				movf	analog_val,W
				subwf	adc_peak_R,W			;   compare analog_val : adc_peak
				btfsc	CARRY					;   if (adc_peak < analog_val)
				bra		peak_skip_R				;   {
				movf	analog_val,W			;     adc_peak_R = analog_val
				movwf	adc_peak_R
				banksel	decay_timer_R
				clrf	decay_timer_R			;     decay_timer_R = 0
peak_skip_R:									;   }
				bra		LED_scan_cont4

LED_scan_cont2:									; } else

				;-----------------------------------------------------

				movlw	SCAN_STEPS				; if (scan_step == SCAN_STEPS)
				subwf	scan_step,W
				btfss	ZERO
				bra		LED_scan_cont3			; {
				clrf	scan_step				;   scan_step = 0

				;--- convert peak level: L ---
				movf	adc_peak_L,W
				movwf	analog_val
				call	VU_convert_steps		;   convert to steps
				movf	vu_level,W				;   peak_level_L = vu_level
				movwf	peak_level_L

				;--- convert peak level: R ---
				movf	adc_peak_R,W
				movwf	analog_val
				call	VU_convert_steps		;   convert to steps
				movf	vu_level,W				;   peak_level_R = vu_level
				movwf	peak_level_R

				bra		LED_scan_cont4

LED_scan_cont3:									; } else

				;-----------------------------------------------------

				movlw	NUM_OF_LED				; if (scan_step == NUM_OF_LED)
				subwf	scan_step,W
				btfss	ZERO
				bra		peak_decay_skip

				movlw	DECAY_SPEED				; and if ((decay_speed_acc += DECAY_SPEED) > 255)
				banksel	decay_speed_acc
				addwf	decay_speed_acc,F
				btfss	CARRY
				bra		peak_decay_skip			; {

				;--- slow decay peak values ---
				decfsz	decay_timer_L,F			;   if (--decay_timer_L == 0)
				bra		peak_L_zero				;   {
				movlw	1						;     decay_timer_L = 1
				movwf	decay_timer_L

				movf	adc_peak_L,F			;     if (adc_peak_L != 0)
				btfsc	ZERO
				bra		peak_L_zero				;     {
				decf	adc_peak_L,F			;       adc_peak_L--
												;     }
peak_L_zero:									;   }
				decfsz	decay_timer_R,F			;   if (--decay_timer_R == 0)
				bra		peak_R_zero				;   {
				movlw	1						;     decay_timer_R = 1
				movwf	decay_timer_R

				movf	adc_peak_R,F			;     if (adc_peak_R != 0)
				btfsc	ZERO
				bra		peak_R_zero				;     {
				decf	adc_peak_R,F			;       adc_peak_R--
												;     }
peak_R_zero:									;   }
peak_decay_skip:								; }

				;-----------------------------------------------------
LED_scan_cont4:
				incf	scan_step,F				; scan_step++

				;--- take care of systick ----------------------------
				movlw	SYSTICK_INC
				addwf	systick,F				; if ((systick += SYSTICK_INC) > 255)
				btfsc	CARRY					; 
				bsf		SYSTICK					;   set tick overflow flag (every 8.192 ms)

#ifdef	DEBUG
				DEBUG_OUT_H			; set debug pin high
#endif

				;--- clear interrupt flags and exit ------------------
				banksel	PIR0
				bcf		TMR0IF					; clear timer0 int flag
				retfie


;=========================================================================================
;	Main program
;
main:
				;-----------------------------------------------------
				; clear RAM

				movlw	_ram_start_				; starting RAM addr
				movwf	FSR0L
				clrf	FSR0H
next_ram:		clrf	INDF0					; clear memory using indirect addressing
				incf	FSR0L,F
				movlw	_ram_end_				; done if (FSR > last memory address)
				subwf	FSR0L,W
				btfss	CARRY
				goto	next_ram

				;-----------------------------------------------------
				; initialize variables


				;-----------------------------------------------------
				; configure WDT

				movlw	(00100B<<WDTCON0_WDTPS_POSN)|WDTCON0_SWDTEN_MASK	; 16 ms interval
				banksel	WDTCON0
				movwf	WDTCON0


				;-----------------------------------------------------
				; restore saved config from NVM

				call	read_config


				;-----------------------------------------------------
				; configure GPIO

				;--- disable individual pull-ups ---
				banksel	WPUA
				clrf	WPUA
				clrf	WPUC
				bsf		BTN1_WPU,BTN1_PIN				; enable pull-up on button SW
				bsf		BTN2_WPU,BTN2_PIN				; enable pull-up on button SW

				;--- initialize LATs ---
				banksel	LATA
				clrf	LATA
				clrf	LATC

				;--- analog/digital mode ---
				banksel	ANSELA
				bcf		BTN1_ANS,BTN1_PIN				; configure button SW input to digital
				bcf		BTN2_ANS,BTN2_PIN

#ifdef	DEBUG
				banksel	ODCONA
				bsf		DEBUG_ODCON,DEBUG_PIN			; configure debug pin open-drain output
#endif

				;-----------------------------------------------------
				; configure ADC
														; FVR voltage = 1.024 V (FVRCON.ADFVR = 01), (FVRCON.FVREN = 1)
;				movlw	(01B<<FVRCON_ADFVR0_POSN)|(1<<FVRCON_FVREN_POSN)
														; FVR voltage = 2.048 V (FVRCON.ADFVR = 10), (FVRCON.FVREN = 1)
				movlw	(10B<<FVRCON_ADFVR0_POSN)|(1<<FVRCON_FVREN_POSN)
				banksel	FVRCON
				movwf	FVRCON
														; set ADC input -> VU_IN_L_AIN, turn on ADC
				movlw	(VU_IN_L_AIN<<ADCON0_CHS0_POSN)|ADCON0_ADON_MASK
				banksel	ADCON0
				movwf	ADCON0
														; ADC reference voltage = FVR (ADCON1.ADPREF = 11)
														; ADC clock = Fosc/64 (ADCON1.ADCS = 110)
														; right justify ADC result (ADCON1.ADFM = 1)
				movlw	(11B<<ADCON1_ADPREF0_POSN)|(110B<<ADCON1_ADCS0_POSN)|(1<<ADCON1_ADFM_POSN)
				movwf	ADCON1

				clrf	ADRESH							; clear ADC result for clean start
				clrf	ADRESL


				;-----------------------------------------------------
				; configure Timer0 for 64 us timebase
														; Fosc/4 as clock, 2:1 prescaler
				movlw	(010B<<T0CON1_T0CS_POSN)|(0001B<<T0CON1_T0CKPS_POSN)
				banksel	T0CON1
				movwf	T0CON1

				movlw	0xFF							; period = 0xFF
				movwf	TMR0H

				bsf		T0EN							; enable Timer0

				banksel	PIE0							; enable interrupt
				bsf		TMR0IE
				bsf		GIE


				;-----------------------------------------------------
				; startup animation

				bsf		LED_NO_UPD						; inhibit LED driver from driving LEDs
startup_loop:
				banksel	bar_level_temp
				movf	bar_level_temp,W
				movwf	bar_level1
				movwf	bar_level2
				call	drive_LED_bar

				banksel	bar_level_temp
				movlw	NUM_OF_LED						; if (bar_level_temp < NUM_OF_LED)
				subwf	bar_level_temp,W
				btfsc	CARRY
				bra		startup_done					; {
				incf	bar_level_temp,F				;   increment bar_level_temp
														; }
				movlw	10								; set loop_cnt (loop counter)
				movwf	loop_cnt						; 
startup_wait:
				btfss	SYSTICK							; while (SYSTICK == 0)
				bra		startup_wait					; {}
				bcf		SYSTICK							; SYSTICK = 0
				clrwdt
				decfsz	loop_cnt,F						; if (--loop_cnt != 0)
				bra		startup_wait					;    continue loop
				bra		startup_loop
startup_done:
				movlw	24
				movwf	loop_cnt
startup_wait2:
				btfss	SYSTICK							; while (SYSTICK == 0)
				bra		startup_wait2					; {}
				bcf		SYSTICK							; SYSTICK = 0
				clrwdt
				decfsz	loop_cnt,F						; if (--loop_cnt != 0)
				bra		startup_wait2					;    continue loop
startup_end:
				bcf		LED_NO_UPD						; resume LED driver


				;===============================================================
				;--- mainloop --------------------------------------------------
mainloop:
				btfss	SYSTICK							; while (SYSTICK == 0)
				goto	mainloop						; { }
				bcf		SYSTICK							; SYSTICK = 0
				clrwdt

				;-----------------------------------------------------
				; handle button switch
				
				call	check_buttons

				banksel	bflags
				btfss	BTN1_PUSH						; if button1 pushed
				bra		btn1_no_push					; {
				movlw	1<<DOT_DS_POS					;   toggle display style
				xorwf	gflags,F
				bcf		BTN1_PUSH						;   clear push flag
				call	config_save_set					;   set config save timer
btn1_no_push:											; }
				banksel	bflags
				btfss	BTN1_HELD						; if button1 held
				bra		btn1_no_held					; {
				bsf		LED_NO_UPD						;   pause LED update
				movlw	1								;   turn on LEDs at segment 1
				movwf	bar_level1
				movwf	bar_level2
				call	drive_LED_bar
btn1_no_held:											; }
				banksel	bflags
				btfss	BTN1_LPUSH						; if button1 long pushed
				bra		btn1_no_lpush					; {
				bcf		LED_NO_UPD						;   resume LED update
														;   toggle reverse display
				movlw	((1<<LED_REV_L_POS)|(1<<LED_REV_R_POS))
				xorwf	gflags,F
				bcf		BTN1_LPUSH						;   clear long push flag
				call	config_save_set					;   set config save timer
btn1_no_lpush:											; }

				banksel	bflags
				btfss	BTN2_PUSH						; if button2 pushed
				bra		btn2_no_push					; {
				movlw	1<<PEAK_DISP_POS				;   toggle peak display
				xorwf	gflags,F
				bcf		BTN2_PUSH						;   clear push flag
				call	config_save_set					;   set config save timer
btn2_no_push:											; }
				banksel	bflags
				btfss	BTN2_HELD						; if button2 held
				bra		btn2_no_held					; {
				bsf		LED_NO_UPD						;   pause LED update
				movlw	12								;   turn on LEDs at segment 12
				movwf	bar_level1
				movwf	bar_level2
				call	drive_LED_bar
btn2_no_held:											; }
				banksel	bflags
				btfss	BTN2_LPUSH						; if button2 long pushed
				bra		btn2_no_lpush					; {
				bcf		LED_NO_UPD						;   resume LED update
				movlw	(1<<CH_FLIP_POS)				;   toggle L/R flip display
				xorwf	gflags,F
;				bsf		OFFSET_CAL						;   start offset calibration
				bcf		BTN2_LPUSH						;   clear long push flag
				call	config_save_set					;   set config save timer
btn2_no_lpush:											; }
				

				;-----------------------------------------------------
				; save mode/config in NVM

				banksel	config_save_timer
				movf	config_save_timer,F				; if (config_save_timer != 0)
				btfsc	ZERO
				bra		config_save_skip
				decfsz	config_save_timer,F				; and if (--config_save_timer == 0)
				bra		config_save_skip				; {

				;--- write config data to NVM ---
				call	write_config

config_save_skip:										; }

				;-----------------------------------------------------

				goto	mainloop

				;--- end of mainloop -------------------------------------------
				;===============================================================

				;-----------------------------------------------------
				; check button switches
check_buttons:
				banksel	BTN1_PORT						; if (button1 down)
				btfsc	BTN1_PORT,BTN1_PIN
				bra		button1_up						; {
				banksel	bflags
				btfsc	BTN1_EVENT						;   if button event flag hasn't been set
				bra		button1_down
				decfsz	btn1_deb_cnt,F					;     decrement button debounce counter
				bra		button1_done
button1_debounced:										;   --- counter cleared ---
				bsf		BTN1_EVENT						;     set button event flag
button1_down:											;   
				movf	btn1_hold_cnt,F					;   if (btn1_hold_cnt != 0)
				btfsc	ZERO							;   {
				bra		button1_held
				decf	btn1_hold_cnt,F					;     btn1_hold_cnt--
				bra		button1_done					;   }
button1_held:											;   else {
				bsf		BTN1_HELD						;     set button held flag
				bra		button1_done					;   }
button1_up:												; } else {
				banksel	bflags
				btfss	BTN1_EVENT						;   if button was just released
				bra		button1_up1						;   {
				bcf		BTN1_EVENT						;     clear event flag

				btfss	BTN1_HELD						;     if (BTN1_HELD == 0) // quick push
				bsf		BTN1_PUSH						;       set button pushed flag
				btfsc	BTN1_HELD						;     if (BTN1_HELD == 1) // long push
				bsf		BTN1_LPUSH						;       set button long pushed flag

				bcf		BTN1_HELD						;     clear button held flag
button1_up1:											;   }
				banksel	btn1_deb_cnt
				movlw	BTN_DEB_CNT						;   reset debounce counter
				movwf	btn1_deb_cnt

				movlw	BTN_HOLD_CNT					;   reset hold counter
				movwf	btn1_hold_cnt
button1_done:											; }

				banksel	BTN2_PORT						; if (button2 down)
				btfsc	BTN2_PORT,BTN2_PIN
				bra		button2_up						; {
				banksel	bflags
				btfsc	BTN2_EVENT						;   if button event flag hasn't been set
				bra		button2_down
				decfsz	btn2_deb_cnt,F					;     decrement button debounce counter
				bra		button2_done
button2_debounced:										;   --- counter cleared ---
				bsf		BTN2_EVENT						;     set button event flag
button2_down:											;   
				movf	btn2_hold_cnt,F					;   if (btn2_hold_cnt != 0)
				btfsc	ZERO							;   {
				bra		button2_held
				decf	btn2_hold_cnt,F					;     btn2_hold_cnt--
				bra		button2_done					;   }
button2_held:											;   else {
				bsf		BTN2_HELD						;     set button held flag
				bra		button2_done					;   }
button2_up:												; } else {
				banksel	bflags
				btfss	BTN2_EVENT						;   if button was just released
				bra		button2_up1						;   {
				bcf		BTN2_EVENT						;     clear event flag

				btfss	BTN2_HELD						;     if (BTN2_HELD == 0) // quick push
				bsf		BTN2_PUSH						;       set button pushed flag
				btfsc	BTN2_HELD						;     if (BTN2_HELD == 1) // long push
				bsf		BTN2_LPUSH						;       set button long pushed flag

				bcf		BTN2_HELD						;     clear button held flag
button2_up1:											;   }
				banksel	btn2_deb_cnt
				movlw	BTN_DEB_CNT						;   reset debounce counter
				movwf	btn2_deb_cnt

				movlw	BTN_HOLD_CNT					;   reset hold counter
				movwf	btn2_hold_cnt
button2_done:											; }

				return


				;-----------------------------------------------------
				; set config save timer when setting changes occur
config_save_set:
				movlw	CONFIG_SAVE_DLY
				banksel	config_save_timer
				movwf	config_save_timer
				return


				;-----------------------------------------------------
				; voltage -> display step conversion
				;   call;   analog_val: value to convert
				;   return; vu_level: LED index
VU_convert_steps:
				movlw	HIGH(VU_step_lookup)|0x80	; setup FSR0 for lookup
													; set the 7th bit for program memory access
				movwf	FSR0H
				movlw	LOW(VU_step_lookup)
				movwf	FSR0L

				movlw	NUM_OF_LED					; vu_level = NUM_OF_LED
				movwf	vu_level
VU_convert_loop:									; do {
				moviw	FSR0++						;   load lookup data -> W
				subwf	analog_val,W				;   if (analog_val >= threshold)
				btfsc	CARRY
				bra		VU_convert_done				;     break
				decfsz	vu_level,F					; 
				bra		VU_convert_loop				; } while (--vu_level != 0)
VU_convert_done:
				return

VU_step_lookup:
#ifdef	VU_SCALE11DB
				;--- special VU scale 11 db range ---
				DB		254	;  0 db
				DB		226	; -1
				DB		202	; -2
				DB		180	; -3
				DB		160	; -4
				DB		143	; -5
				DB		127	; -6
				DB		113	; -7
				DB		101	; -8
				DB		90	; -9
				DB		80	; -10
				DB		72	; -11
#else
#ifdef	VU_SCALE32DB
				;--- VU scale 32 db range ---
				DB		238	;   0 db
				DB		212	;  -1
				DB		189	;  -2
				DB		150	;  -4
				DB		119	;  -6
				DB		84	;  -9
				DB		60	; -12
				DB		42	; -15
				DB		30	; -18
				DB		19	; -22
				DB		12	; -26
				DB		6	; -32
#else
#ifdef	VU_SCALE33DB
				;--- VU scale 33 db range (expanded from LM3916) ---
				DB		226	;  +3 db
				DB		201	;  +2
				DB		180	;  +1
				DB		160	;   0
				DB		143	;  -1
				DB		113	;  -3
				DB		90	;  -5
				DB		71	;  -7
				DB		51	; -10
				DB		32	; -14
				DB		16	; -20
				DB		5	; -30
#else
#ifdef	VU_SCALE36DB
				;--- VU scale 36 db range (least error) ---
				DB		254	;  0 db
				DB		226	; -1
				DB		202	; -2
				DB		180	; -3
				DB		143	; -5
				DB		113	; -7
				DB		80	; -10
				DB		51	; -14
				DB		32	; -18
				DB		16	; -24
				DB		8	; -30
				DB		4	; -36
#else
				;--- default VU scale 30 db range ---
				DB		255	;  0 db
				DB		227	; -1
				DB		202	; -2
				DB		180	; -3
				DB		160	; -4
				DB		127	; -6
				DB		90	; -9
				DB		64	; -12
				DB		45	; -15
				DB		32	; -18
				DB		16	; -24
				DB		8	; -30
#endif
#endif
#endif
#endif

				;-----------------------------------------------------
				; LED bar driver
				; call: bar_level1, bar_level2 = index of the LED to turn on
				; notes: contents of bar_level1 and bar_level2 might be altered
drive_LED_bar:
				;--- flip Left & Right ---
				btfss	CH_FLIP					; if (Channel flip flag on)
				bra		no_ch_flip				; {
				movf	bar_level2,W
				banksel	swap_temp
				movwf	swap_temp
				movf	bar_level1,W
				movwf	bar_level2
				movf	swap_temp,W
				movwf	bar_level1
no_ch_flip:										; }

				;--- verify bar_level1 ---
				movf	bar_level1,W			; if (bar_level1 > NUM_OF_LED)
				sublw	NUM_OF_LED
				btfsc	CARRY
				bra		bar_level1_ok			; {
				movlw	NUM_OF_LED				;   bar_level1 = NUM_OF_LED
				movwf	bar_level1
bar_level1_ok:									; }
				btfss	LED_REV_L				; if (LED reverse flag on)
				bra		bar_level1_done			; {
				movf	bar_level1,W			;   bar_level = (NUM_OF_LED+1) - bar_level
				sublw	NUM_OF_LED+1
				movwf	bar_level1				; }
bar_level1_done:

				;--- verify bar_level2 ---
				movf	bar_level2,W			; if (bar_level2 > NUM_OF_LED)
				sublw	NUM_OF_LED
				btfsc	CARRY
				bra		bar_level2_ok			; {
				movlw	NUM_OF_LED				;   bar_level2 = NUM_OF_LED
				movwf	bar_level2
bar_level2_ok:									; }
				btfss	LED_REV_R				; if (LED reverse flag on)
				bra		bar_level2_done			; {
				movf	bar_level2,W			;   bar_level = (NUM_OF_LED+1) - bar_level
				sublw	NUM_OF_LED+1
				movwf	bar_level2				; }
bar_level2_done:

				;--- turn off all LEDs ---
				banksel	TRISA					; tristate LED pins
				movlw	LED_CLR_TRIA
				iorwf	TRISA,F
				movlw	LED_CLR_TRIC
				iorwf	TRISC,F

				banksel	LATA
				movlw	LED_CLR_LATA			; clear LED pin state
				andwf	LATA,F
				movlw	LED_CLR_LATC			; clear LED pin state
				andwf	LATC,F

				;--- modify the I/O ports ---
				movf	bar_level1,W			; LED_bar_LATA_lookup[bar_level1] -> W
				call	LED_bar_LATA_lookup
				iorwf	LATA,F					; set bits

				movf	bar_level2,W			; LED_bar_LATC_lookup[bar_level2] -> W
				call	LED_bar_LATC_lookup
				iorwf	LATC,F					; set bits

				movf	bar_level1,W			; LED_bar_TRIA_lookup[bar_level1] -> W
				call	LED_bar_TRIA_lookup
				banksel	TRISA
				andwf	TRISA,F					; clear bits

				movf	bar_level2,W			; LED_bar_TRIC_lookup[bar_level2] -> W
				call	LED_bar_TRIC_lookup
				andwf	TRISC,F					; clear bits

				return

LED_bar_LATA_lookup:
				brw
				retlw	0x00			; 0 (off)
				retlw	LED1_LATA		; 1
				retlw	LED2_LATA		; 2
				retlw	LED3_LATA		; 3
				retlw	LED4_LATA		; 4
				retlw	LED5_LATA		; 5
				retlw	LED6_LATA		; 6
				retlw	LED7_LATA		; 7
				retlw	LED8_LATA		; 8
				retlw	LED9_LATA		; 9
				retlw	LED10_LATA		; 10
				retlw	LED11_LATA		; 11
				retlw	LED12_LATA		; 12
				retlw	0x00			; 13 (off)

LED_bar_LATC_lookup:
				brw
				retlw	0x00			; 0 (off)
				retlw	LED13_LATC		; 1
				retlw	LED14_LATC		; 2
				retlw	LED15_LATC		; 3
				retlw	LED16_LATC		; 4
				retlw	LED17_LATC		; 5
				retlw	LED18_LATC		; 6
				retlw	LED19_LATC		; 7
				retlw	LED20_LATC		; 8
				retlw	LED21_LATC		; 9
				retlw	LED22_LATC		; 10
				retlw	LED23_LATC		; 11
				retlw	LED24_LATC		; 12
				retlw	0x00			; 13 (off)

LED_bar_TRIA_lookup:
				brw
				retlw	0xFF			; 0 (off)
				retlw	LED1_TRIA		; 1
				retlw	LED2_TRIA		; 2
				retlw	LED3_TRIA		; 3
				retlw	LED4_TRIA		; 4
				retlw	LED5_TRIA		; 5
				retlw	LED6_TRIA		; 6
				retlw	LED7_TRIA		; 7
				retlw	LED8_TRIA		; 8
				retlw	LED9_TRIA		; 9
				retlw	LED10_TRIA		; 10
				retlw	LED11_TRIA		; 11
				retlw	LED12_TRIA		; 12
				retlw	0xFF			; 13 (off)

LED_bar_TRIC_lookup:
				brw
				retlw	0xFF			; 0 (off)
				retlw	LED13_TRIC		; 1
				retlw	LED14_TRIC		; 2
				retlw	LED15_TRIC		; 3
				retlw	LED16_TRIC		; 4
				retlw	LED17_TRIC		; 5
				retlw	LED18_TRIC		; 6
				retlw	LED19_TRIC		; 7
				retlw	LED20_TRIC		; 8
				retlw	LED21_TRIC		; 9
				retlw	LED22_TRIC		; 10
				retlw	LED23_TRIC		; 11
				retlw	LED24_TRIC		; 12
				retlw	0xFF			; 13 (off)


				;-----------------------------------------------------
				; read user config data from NVM
read_config:
				banksel	NVMADRL
				movlw	LOW(config_data)		;
				movwf	NVMADRL					; set LSB of address
				movlw	HIGH(config_data)		;
				movwf	NVMADRH					; set MSB of address
				bcf		NVMREGS					; select PFM
				bsf		RD						; initiate read
				movf	NVMDATL,W				; read low byte
				andlw	CONFIG_SAVE_FLAGS		; mask/clear unused bits
				movwf	gflags

				incf	NVMADRL,F				; increment data address
				bsf		RD						; initiate read
				movf	NVMDATL,W				; read low byte
				banksel	adc_offset_L
				movwf	adc_offset_L			; -> ADC offset value L

				banksel	NVMADRL
				incf	NVMADRL,F				; increment data address
				bsf		RD						; initiate read
				movf	NVMDATL,W				; read low byte
				banksel	adc_offset_R
				movwf	adc_offset_R			; -> ADC offset value R

				return

				;-----------------------------------------------------
				; write user config onto NVM
write_config:

				;--- erase flash memory ---
				banksel	NVMADRL
				movlw	HIGH(config_data)
				movwf	NVMADRH					; load upper 6 bits of erase address boundary
				movlw	LOW(config_data)
				movwf	NVMADRL					; load lower 8 bits of erase address boundary
				bcf		NVMREGS					; select PFM
				bsf		FREE					; specify an erase operation
				bsf		WREN					; enable writes
				call	unlock_seq				; unlock flash write
				bcf		WREN					; disable writes

				;--- write config data ---
				bsf		WREN					; enable writes
				bsf		LWLO					; load write latches

				movf	gflags,W				; config data to write
				movwf	NVMDATL					; load data low byte
				call	unlock_seq				; unlock write enable

				incf	NVMADRL,F				; increment data address
				banksel	adc_offset_L
				movf	adc_offset_L,W			; ADC offset value L -> data low byte
				banksel	NVMDATL
				movwf	NVMDATL					; load data low byte
				call	unlock_seq				; unlock write enable

				incf	NVMADRL,F				; increment data address
				bcf		LWLO					; latch writes complete, only the last location left

				banksel	adc_offset_R
				movf	adc_offset_R,W			; ADC offset value R -> data low byte
				banksel	NVMDATL
				movwf	NVMDATL					; load data low byte
				call	unlock_seq				; unlock write enable

				bcf		WREN					; disable writes

				return

unlock_seq:
				;--- REQUIRED UNLOCK SEQUENCE ---
				bcf		GIE						; disable interrupts during unlock sequence
				btfsc	GIE						; loop until GIE clears
				goto	$-2
				movlw	0x55
				movwf	NVMCON2
				movlw	0xAA
				movwf	NVMCON2
				bsf		WR
				bsf		GIE						; re-enable interrupts
				return
				;---------------------------------

;=========================================================================================

PSECT	data

config_data:	DB	(CONFIG_DEFAULT)
_offset_L:		DB	(OFFSET_L)
_offset_R:		DB	(OFFSET_R)

;=========================================================================================
END		ResetVector