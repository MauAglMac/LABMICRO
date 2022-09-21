PROCESSOR   18F45K50
#include    <xc.inc>
PSECT udata_acs
cnt1:	DS 1
cnt2:	DS 1    
    
PSECT code
delay_500us:				; La llamada "call" aporta 2 ciclos máquina.
	nop				; Aporta 1 ciclo máquina.
	movlw	164			; Aporta 1 ciclo máquina. Este es el valor de "K".
	goto	delay_us		; Aporta 2 ciclos máquina.
delay_200us:				; La llamada "call" aporta 2 ciclos máquina.
	nop				; Aporta 1 ciclo máquina.
	movlw	64			; Aporta 1 ciclo máquina. Este es el valor de "K".
	goto	delay_us		; Aporta 2 ciclos máquina.
delay_100us:				; La llamada "call" aporta 2 ciclos máquina.
	movlw	31			; Aporta 1 ciclo máquina. Este es el valor de "K".
	goto	delay_us		; Aporta 2 ciclos máquina.
delay_50us:				; La llamada "call" aporta 2 ciclos máquina.
	nop				; Aporta 1 ciclo máquina.
	movlw	14			; Aporta 1 ciclo máquina. Este es el valor de "K".
	goto	delay_us		; Aporta 2 ciclos máquina.
delay_20us:				; La llamada "call" aporta 2 ciclos máquina.
	movlw	5
	
delay_us:
	movwf   BANKMASK(cnt1),0
delay_loop:
	decfsz  BANKMASK(cnt1),1,0
	goto    delay_loop
	return  
	
; - Retardo_500micros:	2 + 1 + 1 + 2 + (2 + 3K) = 500 cm = 500 µs. (para K=164 y 4 MHz).
; - Retardo_200micros:	2 + 1 + 1 + 2 + (2 + 3K) = 200 cm = 200 µs. (para K= 64 y 4 MHz).
; - Retardo_100micros:	2     + 1 + 2 + (2 + 3K) = 100 cm = 100 µs. (para K= 31 y 4 MHz).
; - Retardo_50micros :	2 + 1 + 1 + 2 + (2 + 3K) =  50 cm =  50 µs. (para K= 14 y 4 MHz).
; - Retardo_20micros :	2     + 1     + (2 + 3K) =  20 cm =  20 µs. (para K=  5 y 4 MHz).
 
delay_200ms:				; La llamada "call" aporta 2 ciclos máquina.
	movlw	200			; Aporta 1 ciclo máquina. Este es el valor de "M".
	goto	delay_ms		; Aporta 2 ciclos máquina.
delay_100ms:				; La llamada "call" aporta 2 ciclos máquina.
	movlw	100			; Aporta 1 ciclo máquina. Este es el valor de "M".
	goto	delay_ms		; Aporta 2 ciclos máquina.
delay_50ms:				; La llamada "call" aporta 2 ciclos máquina.
	movlw	50			; Aporta 1 ciclo máquina. Este es el valor de "M".
	goto	delay_ms		; Aporta 2 ciclos máquina.
delay_20ms:				; La llamada "call" aporta 2 ciclos máquina.
	movlw	20			; Aporta 1 ciclo máquina. Este es el valor de "M".
	goto	delay_ms		; Aporta 2 ciclos máquina.
delay_10ms:				; La llamada "call" aporta 2 ciclos máquina.
	movlw	10			; Aporta 1 ciclo máquina. Este es el valor de "M".
	goto	delay_ms		; Aporta 2 ciclos máquina.
delay_5ms:				; La llamada "call" aporta 2 ciclos máquina.
	movlw	5			; Aporta 1 ciclo máquina. Este es el valor de "M".
	goto	delay_ms		; Aporta 2 ciclos máquina.
delay_2ms:				; La llamada "call" aporta 2 ciclos máquina.
	movlw	2			; Aporta 1 ciclo máquina. Este es el valor de "M".
	goto	delay_ms		; Aporta 2 ciclos máquina.
delay_1ms:				; La llamada "call" aporta 2 ciclos máquina.
	movlw	1			; Aporta 1 ciclo máquina. Este es el valor de "M".
	
delay_ms:
	movwf	BANKMASK(cnt2),0	; Aporta 1 ciclo máquina.
ext_loop:
	movlw	249			; Aporta Mx1 ciclos máquina. Este es el valor de "K".
	movwf	BANKMASK(cnt1),0	; Aporta Mx1 ciclos máquina.
int_loop:
	nop				; Aporta KxMx1 ciclos máquina.
	decfsz	BANKMASK(cnt1),1,0	; (K-1)xMx1 cm (cuando no salta) + Mx2 cm (al saltar).
	goto	int_loop		; Aporta (K-1)xMx2 ciclos máquina.
	decfsz	BANKMASK(cnt2),1,0	; (M-1)x1 cm (cuando no salta) + 2 cm (al saltar).
	goto	ext_loop		; Aporta (M-1)x2 ciclos máquina.
	return
; PIC18F45K50 Configuration Bit Settings

; Assembly source line config statements

; CONFIG1L
  CONFIG  PLLSEL = PLL4X        ; PLL Selection (4x clock multiplier)
  CONFIG  CFGPLLEN = OFF        ; PLL Enable Configuration bit (PLL Disabled (firmware controlled))
  CONFIG  CPUDIV = NOCLKDIV     ; CPU System Clock Postscaler (CPU uses system clock (no divide))
  CONFIG  LS48MHZ = SYS24X4     ; Low Speed USB mode with 48 MHz system clock (System clock at 24 MHz, USB clock divider is set to 4)

; CONFIG1H
  CONFIG  FOSC = INTOSCIO       ; Oscillator Selection (Internal oscillator)
  CONFIG  PCLKEN = ON           ; Primary Oscillator Shutdown (Primary oscillator enabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor (Fail-Safe Clock Monitor disabled)
  CONFIG  IESO = OFF            ; Internal/External Oscillator Switchover (Oscillator Switchover mode disabled)

; CONFIG2L
  CONFIG  nPWRTEN = OFF         ; Power-up Timer Enable (Power up timer disabled)
  CONFIG  BOREN = SBORDIS       ; Brown-out Reset Enable (BOR enabled in hardware (SBOREN is ignored))
  CONFIG  BORV = 190            ; Brown-out Reset Voltage (BOR set to 1.9V nominal)
  CONFIG  nLPBOR = OFF          ; Low-Power Brown-out Reset (Low-Power Brown-out Reset disabled)

; CONFIG2H
  CONFIG  WDTEN = OFF           ; Watchdog Timer Enable bits (WDT disabled in hardware (SWDTEN ignored))
  CONFIG  WDTPS = 32768         ; Watchdog Timer Postscaler (1:32768)

; CONFIG3H
  CONFIG  CCP2MX = RC1          ; CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
  CONFIG  PBADEN = ON           ; PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
  CONFIG  T3CMX = RC0           ; Timer3 Clock Input MUX bit (T3CKI function is on RC0)
  CONFIG  SDOMX = RB3           ; SDO Output MUX bit (SDO function is on RB3)
  CONFIG  MCLRE = ON            ; Master Clear Reset Pin Enable (MCLR pin enabled; RE3 input disabled)

; CONFIG4L
  CONFIG  STVREN = ON           ; Stack Full/Underflow Reset (Stack full/underflow will cause Reset)
  CONFIG  LVP = ON              ; Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
  CONFIG  ICPRT = OFF           ; Dedicated In-Circuit Debug/Programming Port Enable (ICPORT disabled)
  CONFIG  XINST = OFF           ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled)

; CONFIG5L
  CONFIG  CP0 = OFF             ; Block 0 Code Protect (Block 0 is not code-protected)
  CONFIG  CP1 = OFF             ; Block 1 Code Protect (Block 1 is not code-protected)
  CONFIG  CP2 = OFF             ; Block 2 Code Protect (Block 2 is not code-protected)
  CONFIG  CP3 = OFF             ; Block 3 Code Protect (Block 3 is not code-protected)

; CONFIG5H
  CONFIG  CPB = OFF             ; Boot Block Code Protect (Boot block is not code-protected)
  CONFIG  CPD = OFF             ; Data EEPROM Code Protect (Data EEPROM is not code-protected)

; CONFIG6L
  CONFIG  WRT0 = OFF            ; Block 0 Write Protect (Block 0 (0800-1FFFh) is not write-protected)
  CONFIG  WRT1 = OFF            ; Block 1 Write Protect (Block 1 (2000-3FFFh) is not write-protected)
  CONFIG  WRT2 = OFF            ; Block 2 Write Protect (Block 2 (04000-5FFFh) is not write-protected)
  CONFIG  WRT3 = OFF            ; Block 3 Write Protect (Block 3 (06000-7FFFh) is not write-protected)

; CONFIG6H
  CONFIG  WRTC = OFF            ; Configuration Registers Write Protect (Configuration registers (300000-3000FFh) are not write-protected)
  CONFIG  WRTB = OFF            ; Boot Block Write Protect (Boot block (0000-7FFh) is not write-protected)
  CONFIG  WRTD = OFF            ; Data EEPROM Write Protect (Data EEPROM is not write-protected)

; CONFIG7L
  CONFIG  EBTR0 = OFF           ; Block 0 Table Read Protect (Block 0 is not protected from table reads executed in other blocks)
  CONFIG  EBTR1 = OFF           ; Block 1 Table Read Protect (Block 1 is not protected from table reads executed in other blocks)
  CONFIG  EBTR2 = OFF           ; Block 2 Table Read Protect (Block 2 is not protected from table reads executed in other blocks)
  CONFIG  EBTR3 = OFF           ; Block 3 Table Read Protect (Block 3 is not protected from table reads executed in other blocks)

; CONFIG7H
  CONFIG  EBTRB = OFF           ; Boot Block Table Read Protect (Boot block is not protected from table reads executed in other blocks)

// config statements should precede project file includes.

PSECT udata_acs
COUNT: DS 1
; DECLARE HERE YOUR VARIABLES WITH FORMAT: "VAR_NAME: DS 1"

PSECT	resetVec, class=CODE, reloc=2

PSECT	absdata, abs, ovrld
absdata:
    org	    0x1000

resetVec:
    goto    MAIN

PSECT code
MAIN:
    ; CLOCK CONFIGURATION
    BANKSEL OSCCON	;ACCESS TO OSCCON REGISTER 
    MOVLW   0x5E	   ;4MHZ FREQUENCY OF INTERNAL OSCILLATOR
    MOVWF   OSCCON,1	;LOAD DATA THROUGH BSR

    ; GPIO CONFIGURATION
    CLRF    TRISB,0   ;CONFIGURE PORT B AS OUTPUT
    SETF    LATB,0    ;TURN OFF LEDS CONNECTED TO PORT B

;    LEDSONOFF:
;   MOVLW 00000000B
;   MOVWF LATB,0
;   CALL DELAY_200MS
;   MOVLW 11111111B
;   MOVWF LATB,0
;   CALL DELAY_200MS
   
;  GOTO LEDSONOFF

; KNIGHTRIDER:
;MOVLW 7
;MOVWF COUNTF,0
;MOVLW 01111111B
;MOVWF LATB,0
;CALL DELAY_50MS
    
; LOOP1:
;RRNCF LATB
;CALL DELAY_50MS
;DECFSZ COUNT,1,0
;GOTO LOOP1
;MOVLW 6
;MOVWF COUNT,0
    
; LOOP2:
;RLNCF LATB
;CALL DELAY_50MS
;DECFSZ COUNT,1,0
;GOTO LOOP2

 ;   GOTO KNIGHTRIDER

MUSTANG:
    MOVLW 11111111B
    MOVWF LATB,0
    CALL delay_200ms
    MOVLW 11100111B
    MOVWF LATB,0
    CALL delay_200ms
    MOVLW 11000011B
    MOVWF LATB,0
    CALL delay_200ms
    MOVLW 10000001B
    MOVWF LATB,0
    CALL delay_200ms
    MOVLW 00000000B
    MOVWF LATB,0
    CALL delay_200ms
    GOTO MUSTANG
 
  


END resetVec