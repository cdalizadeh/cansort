#include <p18f4620.inc>
#include <lcd18.inc>
#include <rtc_macros.inc>
	list P=18F4620, F=INHX32, C=160, N=80, ST=OFF, MM=OFF, R=DEC

; ****************************************************************************
; Configuration Bits
; ****************************************************************************

		CONFIG OSC=HS, FCMEN=OFF, IESO=OFF
		CONFIG PWRT = OFF, BOREN = SBORDIS, BORV = 3
		CONFIG WDT = OFF, WDTPS = 32768
		CONFIG MCLRE = ON, LPT1OSC = OFF, PBADEN = OFF
		CONFIG STVREN = ON, LVP = OFF, XINST = OFF
		CONFIG DEBUG = OFF
		CONFIG CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
		CONFIG CPB = OFF, CPD = OFF
		CONFIG WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
		CONFIG WRTB = OFF, WRTC = OFF, WRTD = OFF
		CONFIG EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
		CONFIG EBTRB = OFF

; ****************************************************************************
; CONSTANT DEFINES
; ****************************************************************************

#define	    TimerConstantH	0x9E
#define	    TimerConstantL	0x58

; Operation Controls

key1		    equ		d'0'
key2		    equ		d'1'
key3		    equ		d'2'
keyA		    equ		d'3'
key4		    equ		d'4'
key5		    equ		d'5'
key6		    equ		d'6'
keyB		    equ		d'7'
key7		    equ		d'8'
key8		    equ		d'9'
key9		    equ		d'10'
keyC		    equ		d'11'
keyStar		    equ		d'12'
key0		    equ		d'13'
keyHash		    equ		d'14'
keyD		    equ		d'15'

STATUS_TEMP	    equ		0x00
W_TEMP		    equ		0x01
Temp		    equ		0x04

; 0x20 to 0x27 for LCD

reg200us	    equ		0x31
reg5us              equ	        0x32
reg50ms		    equ		0x33

KEY		    equ		0x50
TempKEY		    equ		0x51
KEY_ISR		    equ		0x52

InOperation	    equ		0x60
operationDelay	    equ		0x64
operationEnded	    equ		0x65

Op_Seconds	    equ		0x90
Op_Interrupts	    equ		0x91

; 0xB0 and onwards for RTC

    extern tens_digit, ones_digit

; ****************************************************************************
; MACROS
; ****************************************************************************

; Save/Restore Context in Subroutines
saveContext macro
    movff   STATUS, STATUS_TEMP     ; save STATUS first
    movwf   W_TEMP                  ; save W
			endm
restContext macro
    swapf   W_TEMP, W               ; restore W first
    movff   STATUS_TEMP, STATUS     ; restore STATUS last without affecting W
			endm

; Increment file as 2 digit BCD
incf_BCD	macro		BCD
	local	justones, end_incf_BCD
	movff	BCD, Temp
	; Process 1's digit BCD
	movlw	0x0F
	andwf	Temp					; Temp = lower nibble of BCD
	movlw	d'9'
	; If 1's is 9:
	cpfseq	Temp					; Skip if 1's digit is 9
	goto	justones				; Just increment lower nibble if 1's is less than 9
	movlw	b'00010000'				; Increment 10's digit of BCD
	addwf	BCD
	movlw	0xF0					; Clear 1's digit of BCD
	andwf	BCD
	; Check 100:
	movlw	0xA0
	cpfslt	BCD						; If BCD's 10's digit is less than 100, skip
	clrf	BCD						; If BCD's 10's digit is 100, clear everything
	goto	end_incf_BCD
	; If 1's is less than 9:
justones
	incf	BCD
	goto	end_incf_BCD
end_incf_BCD
	nop
			endm

; Delay 50xN milliseconds
Delay50xNms macro	countReg, N
    local   Again
    movlw   N
    movwf	countReg
Again
	call	Delay50ms
    decfsz	countReg
    goto	Again
			endm

; Write RTC Data
WriteRTC	macro
		movff		tens_digit, WREG
		call		WR_DATA
		movff		ones_digit, WREG
		call		WR_DATA
			endm

; Display Table Data on LCD
DispTable	macro	TableVar
		local	Again
		movlw	upper TableVar	; Move Table<20:16> into TBLPTRU
		movwf	TBLPTRU
		movlw	high TableVar	; Move Table<15:8> into TBLPTRH
		movwf	TBLPTRH
		movlw	low TableVar	; Move Table<7:0> into TBLPTRL
		movwf	TBLPTRL
		tblrd*					; Read byte at TBLPTR and copy to TABLAT
		movf	TABLAT, W		; Move byte into W
Again
		call	WR_DATA			; Write byte to LCD
		tblrd+*					; Increment pointer
		movf	TABLAT, W		; Move new byte into W
		bnz		Again			; Keep going until the end (0 byte)
				endm


; Change FSM State
ChangeState	macro	KeyCode, NextState
		local		Next, NotNext		; Need this to bypass 'out of range' error
		movlw		KeyCode			; If 'KeyCode' was pressed
		cpfseq		KEY
		goto		NotNext
Next								; Before going to the next state,
		clrf		LATA			; Clear all Pins
        clrf		LATB
        clrf		LATC
        clrf		LATD
		goto		NextState
NotNext
		nop
			endm

; ****************************************************************************
; VECTORS
; ****************************************************************************
	org		0x0000
	goto	Init

	org		0x08				;high priority ISR
	goto	ISR_HIGH

	org		0x18				;low priority ISR
	goto	ISR_LOW

; ****************************************************************************
; INTERRUPT SERVICE ROUTINE
; ****************************************************************************

ISR_HIGH
	saveContext						; 3
	; Reset Timer
	movlw		TimerConstantH				; 1
	movwf		TMR0H						; 1
	movlw		TimerConstantL - 9			; 1
	movwf		TMR0L						; 1
	; TIMER INTERRUPT
	btfss		INTCON, TMR0IF
	goto		END_ISR_HIGH
	; Increment as BCD interrupts and seconds
	incf_BCD	Op_Interrupts
	movlw		d'0'
	cpfseq		Op_Interrupts				; Skip to seconds++ if interrupts is 0 after adding (100)
	goto		END_ISR_HIGH				; Skip seconds++ if interrupts > 0 after adding (!= 100)
	incf_BCD	Op_Seconds					
END_ISR_HIGH
	bcf			INTCON, TMR0IF
	restContext
	retfie

ISR_LOW
    saveContext
	; KEYPAD INTERRUPT
    btfss		INTCON3, INT1IF			; If KEYPAD interrupt, skip return
    goto		END_ISR_LOW
	; Check operation status
    movlw		0xFF					; If in operation, skip return
    cpfseq		InOperation
    goto		END_ISR_LOW
	; Process KEY
    swapf		PORTB, W				; Read PORTB<7:4> into W<3:0>
    andlw		0x0F
    movwf		KEY_ISR					; Put W into KEY_ISR
    movlw		keyStar					; Put keyStar into W to compare to KEY_ISR
    cpfseq		KEY_ISR					; If key was '*', skip return
    goto		END_ISR_LOW
	; Reset program counter
    clrf		TOSU
    clrf		TOSH
    clrf		TOSL
END_ISR_LOW
    bcf			INTCON3, INT1IF         ; Clear flag for next interrupt
    restContext
	retfie

; ****************************************************************************
; TABLES
; ****************************************************************************

Operation_L1	db	"In Operation.   ", 0
Operation_L2	db	"          STOP~D", 0
Emergency_L1	db	"Emerg  STANDBY~*", 0
Emergency_L2	db	"Stop. CONTINUE~A", 0
TerminationA_L1	db	"Complete. NEXT~A", 0
TerminationA_L2	db	"       STANDBY~*", 0
TerminationB_L1	db	"Duration:  00:19", 0
TerminationB_L2	db	"Total Cans:   12", 0
TerminationC_L1	db	"Al+T: 4  Al-T: 4", 0
TerminationC_L2	db	"Sn+L: 4  Sn-L: 4", 0
LogMenu_L1 	db	"NEXTLOG~A BACK~*", 0
LogMenu_L2 	db	"PREVLOG~B       ", 0

LogA_L1		db	"Log #1  17/02/07", 0
LogA_L2		db	"TotalCans:12 Alu", 0
LogB_L1		db	"Log #2  17/02/05", 0
LogB_L2		db	"TotalCans:10 Alu", 0
LogC_L1		db	"Log #3  17/01/29", 0
LogC_L2		db	"TotalCans:11 Alu", 0
LogD_L1		db	"Log #4  16/12/25", 0
LogD_L2		db	"TotalCans:09 Alu", 0

Test		db	"Test", 0

; ****************************************************************************
; MAIN PROGRAM
; ****************************************************************************
        CODE
Init
; Setup I/O
        movlw   b'00000000'
        movwf   TRISA
        movlw	b'11111111'		; Set required keypad inputs (RB1 is interrupt)
        movwf	TRISB
	movlw	b'10111111'		; RC7: USART RC, RC6: USART TX
	movwf	TRISC			; RC3, RC4: for RTC
	movlw	b'00000000'
	movwf	TRISD
        movlw   b'00000111'
        movwf   TRISE
        movlw	b'00001111'     ; Set all AN pins to Digital
        movwf	ADCON1
; Clear Ports
        clrf	LATA
        clrf	LATB
        clrf	LATC
        clrf	LATD
	clrf	LATE
; Setup Peripherals
	call	InitLCD
	call	i2c_common_setup
	movlw	b'00001000'			; No 16-bit, internal, no prescaler
	movwf	T0CON
; Setup Interrupts
	clrf	RCON
	clrf	INTCON
	clrf	INTCON2
	clrf	INTCON3
	bsf	RCON, IPEN          ; Priority mode interrupts
        bsf	INTCON, GIEH        ; Allow global interrupt
	bsf	INTCON, GIEL
        bsf	INTCON2, INTEDG1    ; set INTEDG1 to detect rising edge
	bsf	INTCON, TMR0IE
	bsf	INTCON2, TMR0IP		; set TRM0IP to high priority
        bsf	INTCON3, INT1IE     ; enable INT1 int flag bit on RB1
	bcf	INTCON3, INT1IP		; set INT1IP to low priority
; Clear FSR
	clrf	InOperation
	clrf	Op_Seconds
	clrf	Op_Interrupts
	clrf	tens_digit
	clrf	ones_digit
; Set operationDelay
        movlw   b'11111111'
        movwf   operationDelay
        movlw   b'00000000'
        movwf   operationEnded

; SET UP RTC            11:30PM, 02/19/2014
;		rtc_resetAll
;		rtc_set 0x00, b'00000000'		; Set seconds to 0
;		rtc_set 0x01, b'00110101'		; Set minutes (30)
;		rtc_set	0x02, b'00100010'		; Set hours (11PM) (0, 12hour/24hour, PM/AM, 10hour)
;		rtc_set 0x04, b'00000111'		; Set day (19)
;		rtc_set	0x05, b'00000010'		; Set month (2)
;		rtc_set 0x06, b'00010111'		; Set year (14)


; STANDBY STATE
Standby
		call	    ClrLCD
		movlw   b'11111111'
		movwf   operationDelay
Stay_Standby
		call		Read_KEY_RTC			; Wait for key inputs
		ChangeState	keyA, Operation			; A for Operation
		ChangeState	keyB, Log			; B for Log
		bra		Stay_Standby

; OPERATION STATE
Operation
		call		ClrLCD
		call		LCD_L1
		DispTable	Operation_L1
		call		LCD_L2
		DispTable	Operation_L2
Stay_Operation	
		call		ReadKEY_delay
		tstfsz		operationEnded
		bra		Termination
		ChangeState	keyD, Emergency
		bra		Stay_Operation

; EMERGENCY STATE
Emergency
		call		ClrLCD
		call		LCD_L1
		DispTable	Emergency_L1
		call		LCD_L2
		DispTable	Emergency_L2
Stay_Emergency	
		call		ReadKEY
		ChangeState	keyStar, Standby
		ChangeState	keyA, Operation
		bra		Stay_Emergency

; TERMINATION STATE
Termination
		movlw		0x00
		movwf		operationEnded
TerminationA
		call		ClrLCD
		call		LCD_L1
		DispTable	TerminationA_L1
		call		LCD_L2
		DispTable	TerminationA_L2
Stay_TerminationA	
		call		ReadKEY
		ChangeState	keyStar, Standby
		ChangeState	keyA, TerminationB
		bra		Stay_TerminationA
TerminationB
		call		ClrLCD
		call		LCD_L1
		DispTable	TerminationB_L1
		call		LCD_L2
		DispTable	TerminationB_L2
Stay_TerminationB	
		call		ReadKEY
		ChangeState	keyStar, Standby
		ChangeState	keyA, TerminationC
		bra		Stay_TerminationB
TerminationC
		call		ClrLCD
		call		LCD_L1
		DispTable	TerminationC_L1
		call		LCD_L2
		DispTable	TerminationC_L2
Stay_TerminationC	
		call		ReadKEY
		ChangeState	keyStar, Standby
		ChangeState	keyA, Standby
		ChangeState	keyB, TerminationB
		bra		Stay_TerminationC

; LOG STATE
Log
LogMenu
		call		ClrLCD
		call		LCD_L1
		DispTable	LogMenu_L1
		call		LCD_L2
		DispTable	LogMenu_L2
Stay_LogMenu
		call		ReadKEY
		ChangeState keyStar, Standby
		ChangeState keyA, LogA
		ChangeState keyB, LogA
		bra			Stay_LogMenu
LogA
		call		ClrLCD
		call		LCD_L1
		DispTable	LogA_L1
		call		LCD_L2
		DispTable	LogA_L2
Stay_LogA
		call		ReadKEY
		ChangeState keyStar, Standby
		ChangeState keyA, LogB
		ChangeState keyB, LogD
		bra			Stay_LogA
LogB
		call		ClrLCD
		call		LCD_L1
		DispTable	LogB_L1
		call		LCD_L2
		DispTable	LogB_L2
Stay_LogB
		call		ReadKEY
		ChangeState keyStar, Standby
		ChangeState keyA, LogC
		ChangeState keyB, LogA
		bra			Stay_LogB
LogC
		call		ClrLCD
		call		LCD_L1
		DispTable	LogC_L1
		call		LCD_L2
		DispTable	LogC_L2
Stay_LogC
		call		ReadKEY
		ChangeState keyStar, Standby
		ChangeState keyA, LogD
		ChangeState keyB, LogB
		bra			Stay_LogC
LogD
		call		ClrLCD
		call		LCD_L1
		DispTable	LogD_L1
		call		LCD_L2
		DispTable	LogD_L2
Stay_LogD
		call		ReadKEY
		ChangeState keyStar, Standby
		ChangeState keyA, LogA
		ChangeState keyB, LogC
		bra			Stay_LogD


; ****************************************************************************
; SUBROUTINES
; ****************************************************************************

; DELAYS
Delay200us							; delay 497 cycles (10MHz Clock)
		movlw		0xA4			; 1 cycle
		movwf		reg200us		; 1 cycle
loop200us
		decfsz		reg200us		; (3 * 164) - 1 = 491 cycles
		goto		loop200us
		nop							; 1 cycle
		nop							; 1 cycle
		return						; 2 cycles

Delay50ms							; delay 125,000 cycles (10MHz Clock)
		movlw		0xF9			; 1 cycle
		movwf		reg50ms			; 1 cycle
loop50ms
		 call		Delay200us
		 decfsz		reg50ms			; (497 + 3) * 249 - 1 = 124,499 Cycles
		 goto		loop50ms
		 call		Delay200us		; 497 cycles
		 return						; 2 cycles

; DISPLAY SUBROUTINES

; Read Keypad Input
ReadKEY
WaitKey
	btfss		PORTB,1     ;Wait until data is available from the keypad
        goto		WaitKey		;Once a key is pressed,
        swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
	andlw		0x0F		;Mask 00001111
	movwf		KEY			;Save result in KEY
	btfsc		PORTB,1     ;Wait until key is released
        goto		$-2			;Back 1 instruction
	return

ReadKEY_delay
WaitKey_delay
	call		Delay50ms
	decfsz		operationDelay
	goto		delaywait
	movlw		0x01
	movwf		operationEnded
	return
	
delaywait
	btfss		PORTB,1     ;Wait until data is available from the keypad
        goto		WaitKey_delay		;Once a key is pressed,
        swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
	andlw		0x0F		;Mask 00001111
	movwf		KEY			;Save result in KEY
	btfsc		PORTB,1     ;Wait until key is released
        goto		$-2			;Back 1 instruction
	return

Read_KEY_RTC
WaitKeyRTC
; Reset to first line
	call	    LCD_L1
; Display year
	rtc_read    0x06
	WriteRTC
	movlw	    0x2F		; ASCII '/'
	call	    WR_DATA
; Display month
	rtc_read    0x05
	WriteRTC
	movlw	    0x2F		; ASCII '/'
	call	    WR_DATA
	rtc_read    0x04
	WriteRTC	
	movlw	    " "
	call	    WR_DATA
	movlw	    "S"
	call	    WR_DATA
	movlw	    "T"
	call	    WR_DATA
	movlw	    "A"
	call	    WR_DATA
	movlw	    "R"
	call	    WR_DATA
	movlw	    "T"
	call	    WR_DATA
	movlw	    "~"			; ASCII '>'
	call	    WR_DATA
	movlw	    "A"
	call	    WR_DATA
; Reset to second line
	call	    LCD_L2
; Display hours
	rtc_read    0x02
	movf        tens_digit, W
        andlw       b'00000001'
        addlw       0x30
        call        WR_DATA
        movf        ones_digit, W
        call        WR_DATA
	movlw	    ":"
	call	    WR_DATA
; Dispay minutes
	rtc_read    0x01
	WriteRTC
	movlw	    ":"
	call	    WR_DATA
; Display seconds
	rtc_read    0x00
	WriteRTC
; Display 'LOGS>B'
	movlw	    " "
        call        WR_DATA
	movlw	    " "
	call	    WR_DATA
	movlw	    "L"
	call	    WR_DATA
	movlw	    "O"
	call	    WR_DATA
	movlw	    "G"
	call	    WR_DATA
	movlw	    "S"
	call	    WR_DATA
	movlw	    "~"			; ASCII '>'
	call	    WR_DATA
	movlw	    "B"
	call	    WR_DATA
;Process KEY
	btfss	    PORTB,1	    ;Wait until data is available from the keypad
        goto	    WaitKeyRTC	    ;Once a key is pressed,
        swapf	    PORTB,W	    ;Read PortB<7:4> into W<3:0>
	andlw	    0x0F	    ;Mask 00001111
	movwf	    KEY		    ;Save result in KEY
	btfsc	    PORTB,1	    ;Wait until key is released
        goto	    $-2		    ;Back 1 instruction
	return

	END