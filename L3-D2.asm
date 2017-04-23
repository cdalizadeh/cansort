#include <p18f4620.inc>
#include <lcd18.inc>
#include <rtc_macros.inc>
	list	P=18F4620, F=INHX32, C=160, N=80, ST=OFF, MM=OFF, R=DEC

; ****************************************************************************
; Configuration Bits
; ****************************************************************************

	CONFIG OSC=HS, FCMEN=OFF, IESO=OFF
	CONFIG PWRT = OFF, BOREN = SBORDIS, BORV = 2
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

; CONSTANTS

; Keys
key1		equ	d'0'
key2		equ	d'1'
key3		equ	d'2'
keyA		equ	d'3'
key4		equ	d'4'
key5		equ	d'5'
key6		equ	d'6'
keyB		equ	d'7'
key7		equ	d'8'
key8		equ	d'9'
key9		equ	d'10'
keyC		equ	d'11'
keyStar		equ	d'12'
key0		equ	d'13'
keyHash		equ	d'14'
keyD		equ	d'15'

; Operation Constants
maxCans		equ	d'12'
TimerConstantH	equ	d'163'
TimerConstantL	equ	d'64'

; Bins
AlTab		equ	d'0'
AlNtb		equ	d'1'
SnLabel		equ	d'2'
SnNlbl		equ	d'3'

; MEMORY ADRESSES

; Macro memory
statusTemp	equ	0x00			; save/restore status
wTemp		equ	0x01			; save/restore W
key		equ	0x02			; stores last key pressed
keyISR		equ	0x03			; stores key in ISR
reg200us	equ	0x04			; 200us delay
reg1ms		equ	0x05			; 1ms delay
reg10ms		equ	0x06			; 10ms delay
reg100ms	equ	0x07			; 100ms delay
reg1s		equ	0x08			; 1s delay
DRN		equ	0x09			; N in delays
conversionReg	equ	0x0A
testreg		equ	0x0B

; 0x20 to 0x27 for LCD

; Operation memory
inOperation	equ	0x28
targetBin	equ	0x29
stepperCounter	equ	0x2A
inLog		equ	0x2B
logOffset	equ	0x2C
tickerPrescale	equ	0x2D
curLog		equ	0x2E

; Data
numCans		equ	0x40
cansAlTab	equ	0x41
cansAlNtb	equ	0x42
cansSnLabel	equ	0x43
cansSnNlbl	equ	0x44
startYear0	equ	0x45
startYear1	equ	0x46
startMonth0	equ	0x47
startMonth1	equ	0x48
startDay0	equ	0x49
startDay1	equ	0x4A
digit1		equ	0x4B
digit0		equ	0x4C

; EEPROM
EEPROM_H	equ	0x50
EEPROM_L	equ	0x51
clearCounter	equ	0x52
EEReader	equ	0x53

; Tabling
ATCounter	equ	0x54
DTCounter	equ	0x55

; TESTING
COUNTER		equ	0x60
BUFFER		equ	0x61
BUFFER2		equ	0x8F
BUFFER3		equ	0x90
BUFFER4		equ	0x91
BUFFER5		equ	0x92
BUFFER6		equ	0x93

; 0xB0 and onwards for RTC

; SFR
opTime0		equ	0x100
opTime1		equ	0x101
opTime2		equ	0x102
opTime3		equ	0x103
opTime4		equ	0x104

	extern	tens_digit, ones_digit

; ****************************************************************************
; MACROS
; ****************************************************************************

; Save/Restore Context in Subroutines
SaveContext	macro
	movff		STATUS, statusTemp     ; save STATUS first
	movwf		wTemp                  ; save W
		endm
RestContext	macro
	swapf		wTemp, W               ; restore W first
	movff		statusTemp, STATUS     ; restore STATUS last without affecting W
		endm

Ascii12		macro	reg, file1, file0
	local		GT9, LT9, finally
	movlw		d'9'
	cpfsgt		reg
	goto		LT9
GT9	movlw		d'1'
	call		AsciiConvert
	movwf		file1
	movlw		d'10'
	subwf		reg, 0
	call		AsciiConvert
	movwf		file0
	goto		finally
LT9	movlw		d'0'
	call		AsciiConvert
	movwf		file1
	movf		reg, 0
	call		AsciiConvert
	movwf		file0
finally
		endm

DispNumLT12	macro	reg
	local		GT9, LT9, end_Disp
	movlw		d'9'
	cpfsgt		reg
	goto		LT9
GT9	movlw		d'1'
	call		AsciiConvert
	call		WR_DATA
	movlw		d'10'
	subwf		reg, 0
	call		AsciiConvert
	call		WR_DATA
	goto		end_Disp
LT9	movlw		d'0'
	call		AsciiConvert
	call		WR_DATA
	movf		reg, 0
	call		AsciiConvert
	call		WR_DATA
end_Disp
		endm

; Delay N milliseconds
DelayNms	macro	countReg, N
	local		Again
	movlw		N
	movwf		countReg
Again
	call		Delay1ms
	decfsz		countReg
	goto		Again
		endm

; Delay N millisecondsF
DelayNmsF	macro	countReg, N
	local		Again
	movff		N, countReg
Again
	call		Delay1ms
	decfsz		countReg
	goto		Again
		endm

; Delay 10xN milliseconds
Delay10xNms	macro	countReg, N
	local		Again
	movlw		N
	movwf		countReg
Again
	call		Delay10ms
	decfsz		countReg
	goto		Again
		endm

Delay100xNms	macro	countReg, N
	local		Again
	movlw		N
	movwf		countReg
Again
	call		Delay100ms
	decfsz		countReg
	goto		Again
		endm

DelayNs		macro	countReg, N
	local		Again
	movlw		N
	movwf		countReg
Again
	call		Delay1s
	decfsz		countReg
	goto		Again
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
	local		Again, finally
	movlw		d'16'
	movwf		DTCounter
	movlw		upper TableVar		; Move Table<20:16> into TBLPTRU
	movwf		TBLPTRU
	movlw		high TableVar		; Move Table<15:8> into TBLPTRH
	movwf		TBLPTRH
	movlw		low TableVar		; Move Table<7:0> into TBLPTRL
	movwf		TBLPTRL
	tblrd*					; Read byte at TBLPTR and copy to TABLAT
	movf		TABLAT, W		; Move byte into W
Again
	call		WR_DATA			; Write byte to LCD
	tblrd+*					; Increment pointer
	movf		TABLAT, W		; Move new byte into W
	dcfsnz		DTCounter
	bra		finally
	bnz		Again			; Keep going until the end (0 byte)
finally
		endm

DispTableEE	macro	TableVar
	local		Again, Continue
	movlw		upper TableVar		; Move Table<20:16> into TBLPTRU
	movwf		TBLPTRU
	movlw		high TableVar		; Move Table<15:8> into TBLPTRH
	movwf		TBLPTRH
	movlw		low TableVar		; Move Table<7:0> into TBLPTRL
	movwf		TBLPTRL
	tblrd*					; Read byte at TBLPTR and copy to TABLAT
	movf		TABLAT, W		; Move byte into W
Again
	WriteEEPROM	WREG, EEPROM_H, EEPROM_L
	incf		EEPROM_L
	bnz		Continue
	incf		EEPROM_H
Continue
	tblrd+*					; Increment pointer
	movf		TABLAT, W		; Move new byte into W
	bnz		Again			; Keep going until the end (0 byte)
		endm

DispTableOffset	macro	TableVar, offset
	local		Again, finally, offsetter
	movlw		upper TableVar		; Move Table<20:16> into TBLPTRU
	movwf		TBLPTRU
	movlw		high TableVar		; Move Table<15:8> into TBLPTRH
	movwf		TBLPTRH
	movlw		low TableVar		; Move Table<7:0> into TBLPTRL
	movwf		TBLPTRL
	decf		TBLPTRL
	incf		offset
	movff		offset, DTCounter
offsetter
	incf		TBLPTRL
	decfsz		DTCounter
	bra		offsetter
	movlw		d'16'
	movwf		DTCounter
	tblrd*					; Read byte at TBLPTR and copy to TABLAT
	movf		TABLAT, W		; Move byte into W
Again
	call		WR_DATA			; Write byte to LCD
	tblrd+*					; Increment pointer
	movf		TABLAT, W		; Move new byte into W
	dcfsnz		DTCounter
	bra		finally
	bnz		Again			; Keep going until the end (0 byte)
finally
		endm

; Change FSM State
ChangeState	macro	KeyCode, NextState
	local		Next, NotNext		; Need this to bypass 'out of range' error
	movlw		KeyCode			; If 'KeyCode' was pressed
	cpfseq		key
	goto		NotNext
Next						; Before going to the next state,
	;clrf		LATA			; Clear all Pins
        ;clrf		LATB
        ;clrf		LATC
        ;clrf		LATD
	goto		NextState
NotNext
	nop
		endm

; EEPROM

; Write word to EEPROM
WriteEEPROM	macro	word, addrH, addrL
	movff		addrH, EEADRH		; Set high address
	movff		addrL, EEADR		; Set low address
	movff		word,  EEDATA		; Set word data
	btfsc		EECON1, WR		; Check if WR = 0
	bra		$-2
	bcf		EECON1, EEPGD		; Point to DATA memory
	bcf		EECON1, CFGS		; Access EEPROM
	bsf		EECON1, WREN		; Enable writes
	bcf		INTCON, GIE		; Disable interrupts
	bcf		PIR2, EEIF
	movlw		0x55
	movwf		EECON2			; Write 55h
	movlw		0xAA
	movwf		EECON2			; Write 0xAA
	bsf		EECON1, WR		; Set WR bit to begin write
	btfsc		EECON1, WR
	bra		$-2
	bsf		INTCON, GIE		; Enable interrupts
	bcf		EECON1, WREN		; Disable writes on write complete (EEIF set)
		endm

; Read EEPROM into file
ReadEEPROM	macro	file, addrH, addrL
	movff		addrH, EEADRH		; Set high address
	movff		addrL, EEADR		; Set low address
	bcf		EECON1, EEPGD		; Point to DATA memory
	bcf		EECON1, CFGS		; Access EEPROM
	bsf		EECON1, RD		; EEPROM Read
	movff		EEDATA, file		; file <- EEDATA
		endm

; ****************************************************************************
; VECTORS
; ****************************************************************************
	org		0x0000
	goto		Init

	org		0x08			;high priority ISR
	goto		ISR_HIGH

	org		0x18			;low priority ISR
	goto		ISR_LOW

; ****************************************************************************
; INTERRUPT SERVICE ROUTINE
; ****************************************************************************

ISR_HIGH
	btfss		inOperation, 0
	goto		ISR_LOG
; Reset Timer
ISR_OP
	movlw		TimerConstantH
	movwf		TMR0H
	movlw		TimerConstantL - 9
	movwf		TMR0L
; TIMER INTERRUPT
	btfss		INTCON, TMR0IF
	goto		END_ISR_HIGH
; Increment as BCD seconds
	call		IncFile0
	goto		END_ISR_HIGH
ISR_LOG
	btfss		inLog, 0
	goto		END_ISR_HIGH
	decfsz		tickerPrescale
	goto		END_ISR_HIGH
	movlw		0x1F
	movwf		tickerPrescale
	call		LCD_L2
	movlw		d'0'
	cpfseq		curLog
	goto		ISR_LOGB
	DispTableOffset	LogA_L2, logOffset
	incf		logOffset
	movlw		d'48'
	cpfsgt		logOffset
	goto		END_ISR_HIGH
	bcf		T0CON, TMR0ON		; Turn off timer
	goto		END_ISR_HIGH
ISR_LOGB
	movlw		d'1'
	cpfseq		curLog
	goto		ISR_LOGC
	DispTableOffset	LogB_L2, logOffset
	incf		logOffset
	movlw		d'48'
	cpfsgt		logOffset
	goto		END_ISR_HIGH
	bcf		T0CON, TMR0ON		; Turn off timer
	goto		END_ISR_HIGH
ISR_LOGC
	movlw		d'2'
	cpfseq		curLog
	goto		ISR_LOGD
	DispTableOffset	LogC_L2, logOffset
	incf		logOffset
	movlw		d'48'
	cpfsgt		logOffset
	goto		END_ISR_HIGH
	bcf		T0CON, TMR0ON		; Turn off timer
	goto		END_ISR_HIGH
ISR_LOGD
	DispTableOffset	LogD_L2, logOffset
	incf		logOffset
	movlw		d'48'
	cpfsgt		logOffset
	goto		END_ISR_HIGH
	bcf		T0CON, TMR0ON		; Turn off timer
END_ISR_HIGH
	bcf		INTCON, TMR0IF
	RestContext
	retfie

ISR_LOW
	SaveContext
; keyPAD INTERRUPT
	;btfss		INTCON3, INT1IF		; If keyPAD interrupt, skip return
	goto		END_ISR_LOW
; Process key
	btfss		inOperation, 0
	goto		END_ISR_LOW
	movlw		0xFF
	swapf		PORTB, W		; Read PORTB<7:4> into W<3:0>
	andlw		0x0F
	movwf		keyISR			; Put W into keyISR
	movlw		keyD			; Put keyStar into W to compare to keyISR
	cpfseq		keyISR			; If key was 'D', skip return
	goto		END_ISR_LOW
	RestContext
	movlw		low Emergency
	movwf		TOSL
	movlw		high Emergency
	movwf		TOSH
	movlw		upper Emergency
	movwf		TOSU
END_ISR_LOW
	bcf		INTCON3, INT1IF         ; Clear flag for next interrupt
	RestContext
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
TerminationB_L1	db	"Duration: ", 0
TerminationB_L2	db	"Total Cans:   ", 0
TerminationC_L1	db	"Al+T: 4  Al-T: 4", 0
TerminationC_L2	db	"Sn+L: 4  Sn-L: 4", 0
LogMenu_L1 	db	"NEXTLOG~A BACK~*", 0
LogMenu_L2 	db	"PREVLOG~B       ", 0

LogA_L1		db	"Log #1  17/04/12", 0
bufferA		db	"Log #1  17/04/12                                                ", 0
LogA_L2		db	"Time: XXXs TotalCans:XX Al+Tab:XX Al-Tab:XX Sn+Lbl:XX Sn-Lbl:03 ", 0
LogB_L1		db	"Log #2  17/04/12", 0
bufferB		db	"Log #1  17/04/12                                                ", 0
LogB_L2		db	"Time: XXXs TotalCans:XX Al+Tab:XX Al-Tab:XX Sn+Lbl:XX Sn-Lbl:03 ", 0
LogC_L1		db	"Log #3  17/04/12", 0
bufferC		db	"Log #1  17/04/12                                                ", 0
LogC_L2		db	"Time: XXXs TotalCans:XX Al+Tab:XX Al-Tab:XX Sn+Lbl:XX Sn-Lbl:03 ", 0
LogD_L1		db	"Log #4  17/04/12", 0
bufferD		db	"Log #1  17/04/12                                                ", 0
LogD_L2		db	"Time: XXXs TotalCans:XX Al+Tab:XX Al-Tab:XX Sn+Lbl:XX Sn-Lbl:03 ", 0

Err1		db	"Error 1         ", 0
Err2		db	"Error 2         ", 0
Test		db	"Test            ", 0
Test2		db	"Test2           ", 0
Test3		db	"Test3           ", 0
Test4		db	"Test4           ", 0
Test5		db	"Test5           ", 0
Test6		db	"Test6           ", 0
LongTable	db	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz1234567890AB", 0
LongTable2	db	"ABCDEFGHIJKLMNOPQRSTUVWXYZ1234", 0
LongTable3	db	"567890abcdefghijklmnopqrstuvwx", 0
LongTable4	db	"DDDDDDDDDDDDDDDD", 0

; EEPROM Tables

EETable0	db	"****************  PC INTERFACE  ****************                ", 0
EETableA	db	"LOG #1           DATE   XX/XX/XX OPTIME     XXXs #CANS-           AL+TAB      XX  AL-TAB      XX  SN+LBL      XX  SN-LBL      XX  TOTAL       XX                ", 0
EETableB	db	"LOG #2           DATE   XX/XX/XX OPTIME     XXXs #CANS-           AL+TAB      XX  AL-TAB      XX  SN+LBL      XX  SN-LBL      XX  TOTAL       XX                ", 0
EETableC	db	"LOG #3           DATE   XX/XX/XX OPTIME     XXXs #CANS-           AL+TAB      XX  AL-TAB      XX  SN+LBL      XX  SN-LBL      XX  TOTAL       XX                ", 0
EETableD	db	"LOG #4           DATE   XX/XX/XX OPTIME     XXXs #CANS-           AL+TAB      XX  AL-TAB      XX  SN+LBL      XX  SN-LBL      XX  TOTAL       XX                ", 0
EETable1	db	"****************LAST UPDATED:  4", 0

;Locations_LogA	db	h'58', h'00', h'59', h'00', h'5B', h'00', h'5C', h'00', h'5E', h'00', h'5F', h'00', h'6C', h'00', h'6D', h'00', h'6E', h'00', h'8E', h'00', h'8F', h'00', h'9E', h'00', h'9F', h'00', h'AE', h'00', h'AF', h'00', h'BE', h'00', h'BF', h'00', h'CE', h'00', h'CF', h'00' 
;Locations_LogB	db	0xF8, 0x00, 0xF9, 0x00, 0xFB, 0x00, 0xFC, 0x00, 0xFE, 0x00, 0xFF, 0x00, 0x6C, 0x00, 0x6D, 0x00, 0x6E, 0x00, 0x8E, 0x00, 0x8F, 0x00, 0x9E, 0x00, 0x9F, 0x00, 0xAE, 0x00, 0xAF, 0x00, 0xBE, 0x00, 0xBF, 0x00, 0xCE, 0x00, 0xCF, 0x00
;Locations_LogC	db	0x58, 0x00, 0x59, 0x00, 0x5B, 0x00, 0x5C, 0x00, 0x5E, 0x00, 0x5F, 0x00, 0x6C, 0x00, 0x6D, 0x00, 0x6E, 0x00, 0x8E, 0x00, 0x8F, 0x00, 0x9E, 0x00, 0x9F, 0x00, 0xAE, 0x00, 0xAF, 0x00, 0xBE, 0x00, 0xBF, 0x00, 0xCE, 0x00, 0xCF, 0x00
;Locations_LogD	db	0x58, 0x00, 0x59, 0x00, 0x5B, 0x00, 0x5C, 0x00, 0x5E, 0x00, 0x5F, 0x00, 0x6C, 0x00, 0x6D, 0x00, 0x6E, 0x00, 0x8E, 0x00, 0x8F, 0x00, 0x9E, 0x00, 0x9F, 0x00, 0xAE, 0x00, 0xAF, 0x00, 0xBE, 0x00, 0xBF, 0x00, 0xCE, 0x00, 0xCF, 0x00

; ****************************************************************************
; MAIN PROGRAM
; ****************************************************************************
        CODE
Init
; Setup I/O
	movlw		b'00001111'
	movwf		ADCON1
        movlw		b'10000000'		; 0: Stepper1A | 1: Stepper1B | 2: Stepper2A | 3: Stepper2A | 4: Stepper3A | 5: Stepper3B | 6: Osc out | 7: Osc in
        movwf		TRISA
        movlw		b'11111010'		; 0: XXXX | 1: Key interrupt | 2: Solenoid1 | 3: XXXX | 4: Key input | 5: Key input | 6: Key input | 7: Key input
        movwf		TRISB
	movlw		b'10111101'		; 0: DC motor direction | 1: DC motor run | 2: Label Test | 3: RTC | 4: RTC | 5: Magnetic sensor | 6: USART TX | 7: USART RC
	movwf		TRISC			
	movlw		b'00000011'		; 0: Breakbeam | 1: Tab Test | 2: LCD RS | 3: LCD E | 4: LCD | 5: LCD | 6: LCD | 7: LCD
	movwf		TRISD
	movlw		b'00001000'		; 0: XXXX | 1: XXXX | 2: XXXX | 3: MCLR | 4: - | 5: - | 6: - | 7: -
	movwf		TRISE
; Clear Ports
	clrf		LATA
	clrf		LATB
	clrf		LATC
	clrf		LATD
	clrf		LATE
; Setup Peripherals
	call		InitLCD
	;call		i2c_common_setup
	movlw		b'00001000'		; No 16-bit, internal, no prescaler
	movwf		T0CON
; Setup Interrupts
	clrf		RCON
	clrf		INTCON
	clrf		INTCON2
	clrf		INTCON3
	bsf		RCON, IPEN		; Priority mode interrupts
	bsf		INTCON, GIEH		; Allow global interrupt
	bsf		INTCON, GIEL
	bsf		INTCON2, INTEDG1	; set INTEDG1 to detect rising edge
	bsf		INTCON, TMR0IE
	bsf		INTCON2, TMR0IP		; set TRM0IP to high priority
	bsf		INTCON3, INT1IE		; enable INT1 int flag bit on RB1
	bcf		INTCON3, INT1IP		; set INT1IP to low priority
; Disable pull-up
	bsf		INTCON2, RBPU
; Setup EEPROM
	bcf		EECON1, 3
	call		InitializeEEPROM
; Clear other
	clrf		inOperation
	clrf		inLog
	clrf		tens_digit
	clrf		ones_digit
	movlw		0x1F
	movwf		tickerPrescale

;SET UP RTC            11:30PM, 02/19/2014
;rtc_resetAll
;	rtc_set 0x00, b'00000000'		; Set seconds to 0
;	rtc_set 0x01, b'00001000'		; Set minutes (30)
;	rtc_set	0x02, b'01100011'		; Set hours (11PM) (0, 12hour/24hour, PM/AM, 10hour)
;	rtc_set 0x04, b'00010110'		; Set day (19)
;	rtc_set	0x05, b'00000011'		; Set month (2)
;	rtc_set 0x06, b'00010100'		; Set year (14)	
	

; STANDBY STATE
Standby
	bcf		T0CON, TMR0ON		; Turn off timer
	bcf		inOperation, 0		; Used in ISR
	bcf		inLog, 0
	call		ClrLCD
stay_Standby
	call		ReadKey		; Wait for key inputs
	ChangeState	keyA, Operation		; A for Operation
	;ChangeState	keyA, TestState		; A for Test
	ChangeState	keyB, Log		; B for Log
	bra		stay_Standby

; EMERGENCY STATE
Emergency
	bcf		inOperation, 0		; Used in ISR
	call		ClrLCD
	call		LCD_L1
	DispTable	Emergency_L1
	call		LCD_L2
	DispTable	Emergency_L2
stay_Emergency
	call		ReadKey
	ChangeState	keyStar, Standby
	ChangeState	keyA, Operation
	bra		stay_Emergency

; LOG STATE
Log
LogMenu
	call		ClrLCD
	call		LCD_L1
	DispTable	LogMenu_L1
	call		LCD_L2
	DispTable	LogMenu_L2
stay_LogMenu
	call		ReadKey
	ChangeState	keyStar, Standby
	ChangeState	keyA, LogA
	ChangeState	keyB, LogA
	bra		stay_LogMenu
LogA
	movlw		d'0'
	movwf		curLog
	clrf		logOffset
	bsf		inLog, 0
	movlw		TimerConstantH		; 1
	movwf		TMR0H
	movlw		TimerConstantL		; 1
	movwf		TMR0L			; 1
	bsf		T0CON, TMR0ON		; Turn on timer
	call		ClrLCD
	call		LCD_L1
	DispTable	LogA_L1
	call		LCD_L2
	DispTable	LogA_L2
stay_LogA
	call		ReadKey
	ChangeState	keyStar, Standby
	ChangeState	keyA, LogB
	ChangeState	keyB, LogD
	bra		stay_LogA
LogB
	movlw		d'1'
	movwf		curLog
	bsf		T0CON, TMR0ON		; Turn on timer
	clrf		logOffset
	call		ClrLCD
	call		LCD_L1
	DispTable	LogB_L1
	call		LCD_L2
	DispTable	LogB_L2
stay_LogB
	call		ReadKey
	ChangeState	keyStar, Standby
	ChangeState	keyA, LogC
	ChangeState	keyB, LogA
	bra		stay_LogB
LogC
	movlw		d'2'
	movwf		curLog
	bsf		T0CON, TMR0ON		; Turn on timer
	clrf		logOffset
	call		ClrLCD
	call		LCD_L1
	DispTable	LogC_L1
	call		LCD_L2
	DispTable	LogC_L2
stay_LogC
	call		ReadKey
	ChangeState	keyStar, Standby
	ChangeState	keyA, LogD
	ChangeState	keyB, LogB
	bra		stay_LogC
LogD
	movlw		d'3'
	movwf		curLog
	bsf		T0CON, TMR0ON		; Turn on timer
	clrf		logOffset
	call		ClrLCD
	call		LCD_L1
	DispTable	LogD_L1
	call		LCD_L2
	DispTable	LogD_L2
stay_LogD
	call		ReadKey
	ChangeState	keyStar, Standby
	ChangeState	keyA, LogA
	ChangeState	keyB, LogC
	bra		stay_LogD

; TERMINATION STATE
Termination
	bcf		inOperation, 0		; used in ISR
	bcf		T0CON, TMR0ON		; Turn off timer
TerminationA
	call		ClrLCD
	call		LCD_L1
	DispTable	TerminationA_L1
	call		LCD_L2
	DispTable	TerminationA_L2
	call		UpdateLogs
stay_TerminationA	
	call		ReadKey
	ChangeState	keyStar, Standby
	ChangeState	keyA, TerminationB
	bra		stay_TerminationA
TerminationB
	call		ClrLCD
	call		LCD_L1
	DispTable	TerminationB_L1
	lfsr		0, opTime4
	movf		POSTDEC0, 0
	call		AsciiConvert
	call		WR_DATA
	movf		POSTDEC0, 0
	call		AsciiConvert
	call		WR_DATA
	movf		POSTDEC0, 0
	call		AsciiConvert
	call		WR_DATA
	movlw		d'46'
	call		WR_DATA
	movf		POSTDEC0, 0
	call		AsciiConvert
	call		WR_DATA
	movlw		d'115'
	call		WR_DATA
	call		LCD_L2
	DispTable	TerminationB_L2
	DispNumLT12	numCans
stay_TerminationB	
	call		ReadKey
	ChangeState	keyStar, Standby
	ChangeState	keyA, TerminationC
	bra		stay_TerminationB
TerminationC
	call		ClrLCD
	call		LCD_L1
	movlw		"A"
	call		WR_DATA
	movlw		"l"
	call		WR_DATA
	movlw		"+"
	call		WR_DATA
	movlw		"T"
	call		WR_DATA
	movlw		":"
	call		WR_DATA
	DispNumLT12	cansAlTab
	movlw		" "
	call		WR_DATA
	movlw		" "
	call		WR_DATA
	movlw		"A"
	call		WR_DATA
	movlw		"l"
	call		WR_DATA
	movlw		"-"
	call		WR_DATA
	movlw		"T"
	call		WR_DATA
	movlw		":"
	call		WR_DATA
	DispNumLT12	cansAlNtb
	    call		LCD_L2
	movlw		"S"
	call		WR_DATA
	movlw		"n"
	call		WR_DATA
	movlw		"+"
	call		WR_DATA
	movlw		"L"
	call		WR_DATA
	movlw		":"
	call		WR_DATA
	DispNumLT12	cansSnLabel
	movlw		" "
	call		WR_DATA
	movlw		" "
	call		WR_DATA
	movlw		"S"
	call		WR_DATA
	movlw		"n"
	call		WR_DATA
	movlw		"-"
	call		WR_DATA
	movlw		"L"
	call		WR_DATA
	movlw		":"
	call		WR_DATA
	DispNumLT12	cansSnNlbl
stay_TerminationC	
	call		ReadKey
	ChangeState	keyStar, Standby
	ChangeState	keyA, Standby
	ChangeState	keyB, TerminationB
	bra		stay_TerminationC

; OPERATION STATE
Operation
; prepare operation timer
	movlb		d'1'
	clrf		opTime0
	clrf		opTime1
	clrf		opTime2
	clrf		opTime3
	clrf		opTime4
	movlb		d'0'
	lfsr		0, opTime0
	movlw		TimerConstantH		; 1
	movwf		TMR0H
	movlw		TimerConstantL		; 1
	movwf		TMR0L			; 1
	bsf		T0CON, TMR0ON		; Turn on timer
; reset number of cans
	movlw		maxCans
	movwf		numCans
; clear can counts
	clrf		cansAlTab
	clrf		cansAlNtb
	clrf		cansSnLabel
	clrf		cansSnNlbl
; set operation state (for ISR)
	bsf		inOperation, 0
; save start time
	;rtc_read	0x06
	;movff		tens_digit, startYear1
	;movff		ones_digit, startYear0
	;rtc_read	0x05
	;movff		tens_digit, startMonth1
	;movff		ones_digit, startMonth0
	;rtc_read	0x04
	;movff		tens_digit, startDay1
	;movff		ones_digit, startDay0
; display
	call		ClrLCD
	call		LCD_L1
	DispTable	Operation_L1
	call		LCD_L2
	DispTable	Operation_L2
stay_Op_Detect
	bsf		LATC, 1			; activate drum motor
	bsf		LATC, 0			; activate drum motor
	btfsc		PORTD, 0		; wait for breakbeam
	bra		stay_Op_Detect
	bcf		LATC, 1			; deactivate drum motor
	DelayNs		DRN, 5
Op_Turn
	call		StepAp
	Delay100xNms	DRN, 1
	call		StepAn
	DelayNs		DRN, 1
Op_Test
	bsf		LATB, 2			; activate solenoid
	btfsc		PORTC, 5		; test magswitch
	bra		Op_TestSn
Op_TestAl
	DelayNs		DRN, 3
	btfsc		PORTD, 1		; test current1
	bra		Op_IsTab
Op_IsNtb
	bcf		LATB, 2			; deactivate solenoid
	movlw		AlNtb
	movwf		targetBin
	incf		cansAlNtb
	bra		Op_MoveBin_AlTab
Op_IsTab
	bcf		LATB, 2			; deactivate solenoid
	movlw		AlTab
	movwf		targetBin
	incf		cansAlTab
	bra		Op_MoveBin_AlTab
Op_TestSn
	bcf		LATB, 2			; deactivate solenoid
	btfsc		PORTC, 0		; test current2
	bra		Op_IsNlbl
Op_IsLabel
	bcf		LATB, 2			; deactivate solenoid
	movlw		SnLabel
	movwf		targetBin
	incf		cansSnLabel
	bra		Op_MoveBin_AlTab
Op_IsNlbl
	movlw		SnNlbl
	movwf		targetBin
	incf		cansSnNlbl
	bra		Op_MoveBin_AlTab

Op_MoveBin_AlTab
	movlw		AlTab
	cpfseq		targetBin
	bra		Op_MoveBin_AlNtb
	call		StepBp
	Delay100xNms	DRN, 1
	call		StepBn
	bra		Op_Finish
Op_MoveBin_AlNtb
	movlw		AlNtb
	cpfseq		targetBin
	bra		Op_MoveBin_SnLabel
	call		StepC90p
	call		StepBp
	Delay100xNms	DRN, 1
	call		StepBn
	call		StepC90n
	bra		Op_Finish
Op_MoveBin_SnLabel
	movlw		SnLabel
	cpfseq		targetBin
	bra		Op_MoveBin_SnNlbl
	call		StepC180p
	call		StepBp
	Delay100xNms	DRN, 1
	call		StepBn
	call		StepC180n
	bra		Op_Finish
Op_MoveBin_SnNlbl
	movlw		SnNlbl
	cpfseq		targetBin
	bra		Error1
	call		StepC90n
	call		StepBp
	Delay100xNms	DRN, 1
	call		StepBn
	call		StepC90p
	bra		Op_Finish
Op_Finish
	dcfsnz		numCans
	bra		Termination
	bra		stay_Op_Detect

Error1
	call		ClrLCD
	call		LCD_L1
	DispTable	Err1
	bra		endlop2
	
; ****************************************************************************
; TEST STATES
; ****************************************************************************

TestState
	bra		endlop
	call		ClrLCD
	call		LCD_L1
	DispTable	Test2
	call		LCD_L2
	;bra		TableTest
	;bra		StepATest
	;bra		StepBTest
	;bra		StepCTest
	;bra		StepOGTest
	bra		EEPROMTest

endlop2
	nop
	bra		endlop2

EEPROMTest
	clrf		EEPROM_H
	clrf		EEPROM_L
	DispTableEE	EETable0
	DispTableEE	EETableA
	DispTableEE	EETableB
	DispTableEE	EETableC
	DispTableEE	EETableD
	DispTableEE	EETable1
	call		UpdateLogs
	movlw		"X"
	call		WR_DATA
	bra		endlop

TableTest
;	MOVLW     D'64'                      ; number of bytes in erase block
;	MOVWF     COUNTER
;	MOVLW     high BUFFER	             ; point to buffer
;	MOVWF     FSR0H
;	MOVLW     low BUFFER
;	MOVWF     FSR0L
;	MOVLW     upper LongTable           ; Load TBLPTR with the base
;	MOVWF     TBLPTRU                   ; address of the memory block
;	MOVLW     high LongTable
;	MOVWF     TBLPTRH
;	MOVLW     low LongTable
;	MOVWF     TBLPTRL 
;READ_BLOCK
;	TBLRD*+                             ; read into TABLAT, and inc
;	MOVF      TABLAT, W                 ; get data
;	MOVWF     POSTINC0                  ; store data
;	DECFSZ    COUNTER                   ;                   done?
;	BRA       READ_BLOCK                ; repeat
;MODIFY_WORD
;	MOVLW     high BUFFER            ; point to buffer
;	MOVWF     FSR0H
;	MOVLW     low BUFFER
;	MOVWF     FSR0L
;	; modify buffer

;ERASE_BLOCK
;	MOVLW     upper LongTable           ; load TBLPTR with the base
;	MOVWF     TBLPTRU                   ; address of the memory block
;	MOVLW     high LongTable
;	MOVWF     TBLPTRH 
;	MOVLW     low LongTable
;	MOVWF     TBLPTRL
;	BSF       EECON1, EEPGD             ; point to Flash program memory
;	BCF       EECON1, CFGS              ; access Flash program memory
;	BSF       EECON1, WREN              ; enable write to memory
;	BSF       EECON1, FREE              ; enable Row Erase operation
;	BCF       INTCON, GIE               ; disable interrupts
;	MOVLW     55h
;	MOVWF     EECON2                    ;                    write                    55h
;	MOVLW     0AAh
;	MOVWF     EECON2                    ;                    write                    0AAh
;	BSF       EECON1, WR                ; start erase (CPU stall)
;	BSF       INTCON, GIE               ; re-enable interrupts
;	TBLRD*-                             ; dummy read decrement
;	MOVLW     high BUFFER          ; point to buffer
;	MOVWF     FSR0H
;	MOVLW     low BUFFER
;	MOVWF     FSR0L
;WRITE_BUFFER_BACK
;	MOVLW     D'64'                      ; number of bytes in holding register
;	MOVWF     COUNTER
;WRITE_BYTE_TO_HREGS
;	;MOVFF     POSTINC0, WREG            ; get low byte of buffer data
;	movlw	    0x61
;	MOVWF     TABLAT                   ; present data to table latch
;	TBLWT+*                             ; write data, perform a short write 
;	; to internal TBLWT holding register.
;	DECFSZ    COUNTER                   ; loop until buffers are full
;	BRA       WRITE_BYTE_TO_HREGS
;PROGRAM_MEMORY
;	MOVLW     upper LongTable           ; load TBLPTR with the base
;	MOVWF     TBLPTRU                   ; address of the memory block
;	MOVLW     high LongTable
;	MOVWF     TBLPTRH 
;	MOVLW     low LongTable
;	MOVWF     TBLPTRL 
;	BSF     EECON1, EEPGD         ; point to Flash program memory
;	BCF     EECON1, CFGS          ; access Flash program memory
;	BSF     EECON1, WREN          ; enable write to memory
;	BCF     INTCON, GIE           ; disable interrupts
;	MOVLW   55h
;	MOVWF   EECON2                ; write 55h
;	MOVLW   0AAh
;	MOVWF   EECON2                ; write 0AAh
;	BSF     EECON1, WR            ; start program (CPU stall)
;	BSF     INTCON, GIE           ; re-enable interrupts
;	BCF     EECON1, WREN          ; disable write to memory
	
	call		ClrLCD
	call		LCD_L1
	DispTable	LongTable
	
	bra		endlop
	
StepATest
	call		StepAp
	call		StepAn
	DelayNs		DRN, 3
	bra		StepATest
	call		StepAp
	call		StepAn
	call		StepAp
	call		StepAn
	bra		endlop

StepBTest
	call		StepBp
	call		StepBn
	call		StepBp
	call		StepBn
	call		StepBp
	call		StepBn
	bra		endlop

StepCTest
	call		StepC90p
	DelayNs		DRN, 1
	call		StepC90n
	DelayNs		DRN, 1
	call		StepC90p
	DelayNs		DRN, 1
	call		StepC90n
	DelayNs		DRN, 1
	call		StepC90p
	DelayNs		DRN, 1
	call		StepC90n
	bra		endlop

StepOGTest
	call		StepOG
	bra		endlop
	
endlop	nop
	bra		endlop
; ****************************************************************************
; SUBROUTINES
; ****************************************************************************
; OUPUT SUBROUTINES

StepOG
; Description:	orignal non-decoded stepper functionality
; Input:	-
; Output:	-
; Regs Used:	stepperCounter
	movlw		d'13'
	movwf		stepperCounter
loop_StepOG
	dcfsnz		stepperCounter
	return
	bsf		LATA, 0
	bsf		LATA, 1
	bcf		LATA, 2
	bcf		LATA, 3
	DelayNms	DRN, 3
	bsf		LATA, 0
	bcf		LATA, 1
	bcf		LATA, 2
	bsf		LATA, 3
	DelayNms	DRN, 3
	bcf		LATA, 0
	bcf		LATA, 1
	bsf		LATA, 2
	bsf		LATA, 3
	DelayNms	DRN, 3
	bcf		LATA, 0
	bsf		LATA, 1
	bsf		LATA, 2
	bcf		LATA, 3
	DelayNms	DRN, 3
	bra		loop_StepOG

StepAp
; Description:	Turns stepper A 45 degrees positive
; Input:	-
; Output:	-
; Regs Used:	stepperCounter
	movlw		d'50'
	movwf		stepperCounter
loop_StepAp
	dcfsnz		stepperCounter
	return
	; 1100
	bcf		LATA, 4
	bcf		LATA, 2
	DelayNms	DRN, 5
	; 1001
	bcf		LATA, 4
	bsf		LATA, 2
	DelayNms	DRN, 5
	; 0011
	bsf		LATA, 4
	bsf		LATA, 2
	DelayNms	DRN, 5
	; 0110
	bsf		LATA, 4
	bcf		LATA, 2
	DelayNms	DRN, 5
	bra		loop_StepAp

StepAn
; Description:	Turns stepper A 45 degrees negative
; Input:	-
; Output:	-
; Regs Used:	stepperCounter
	movlw		d'50'
	movwf		stepperCounter
loop_StepAn
	dcfsnz		stepperCounter
	return
	; 1100
	bcf		LATA, 4
	bcf		LATA, 2
	DelayNms	DRN, 5
	; 1001
	bsf		LATA, 4
	bcf		LATA, 2
	DelayNms	DRN, 5
	; 0011
	bsf		LATA, 4
	bsf		LATA, 2
	DelayNms	DRN, 5
	; 0110
	bcf		LATA, 4
	bsf		LATA, 2
	DelayNms	DRN, 5
	bra		loop_StepAn

StepBp
; Description:	Turns stepper B 45 degrees positive
; Input:	-
; Output:	-
; Regs Used:	stepperCounter
	movlw		d'35'
	movwf		stepperCounter
loop_StepBp
	dcfsnz		stepperCounter
	return
	; 1100
	bcf		LATA, 0
	bcf		LATA, 5
	DelayNms	DRN, 5
	; 1001
	bcf		LATA, 0
	bsf		LATA, 5
	DelayNms	DRN, 5
	; 0011
	bsf		LATA, 0
	bsf		LATA, 5
	DelayNms	DRN, 5
	; 0110
	bsf		LATA, 0
	bcf		LATA, 5
	DelayNms	DRN, 5
	bra		loop_StepBp

StepBn
; Description:	Turns stepper B 45 degrees negative
; Input:	-
; Output:	-
; Regs Used:	stepperCounter
	movlw		d'35'
	movwf		stepperCounter
loop_StepBn
	dcfsnz		stepperCounter
	return
	; 1100
	bcf		LATA, 0
	bcf		LATA, 5
	DelayNms	DRN, 5
	; 1001
	bsf		LATA, 0
	bcf		LATA, 5
	DelayNms	DRN, 5
	; 0011
	bsf		LATA, 0
	bsf		LATA, 5
	DelayNms	DRN, 5
	; 0110
	bcf		LATA, 0
	bsf		LATA, 5
	DelayNms	DRN, 5
	bra		loop_StepBn

StepC90p
; Description:	Turns stepper C 90 degrees positive
; Input:	-
; Output:	-
; Regs Used:	stepperCounter
	movlw		d'64'
	movwf		stepperCounter
loop_StepC90p
	dcfsnz		stepperCounter
	return
	; 1100
	bcf		LATA, 3
	bcf		LATA, 1
	DelayNms	DRN, 10
	; 1001
	bcf		LATA, 3
	bsf		LATA, 1
	DelayNms	DRN, 10
	; 0011
	bsf		LATA, 3
	bsf		LATA, 1
	DelayNms	DRN, 10
	; 0110
	bsf		LATA, 3
	bcf		LATA, 1
	DelayNms	DRN, 10
	bra		loop_StepC90p
	
StepC90n
; Description:	Turns stepper C 90 degrees negative
; Input:	-
; Output:	-
; Regs Used:	stepperCounter
	movlw		d'64'
	movwf		stepperCounter
loop_StepC90n
	dcfsnz		stepperCounter
	return
	; 1100
	bcf		LATA, 3
	bcf		LATA, 1
	DelayNms	DRN, 10
	; 1001
	bsf		LATA, 3
	bcf		LATA, 1
	DelayNms	DRN, 10
	; 0011
	bsf		LATA, 3
	bsf		LATA, 1
	DelayNms	DRN, 10
	; 0110
	bcf		LATA, 3
	bsf		LATA, 1
	DelayNms	DRN, 10
	bra		loop_StepC90n

StepC180p
; Description:	Turns stepper C 90 degrees positive
; Input:	-
; Output:	-
; Regs Used:	stepperCounter
	movlw		d'100'
	movwf		stepperCounter
loop_StepC180p
	dcfsnz		stepperCounter
	return
	; 1100
	bcf		LATA, 3
	bcf		LATA, 1
	DelayNms	DRN, 10
	; 1001
	bcf		LATA, 3
	bsf		LATA, 1
	DelayNms	DRN, 10
	; 0011
	bsf		LATA, 3
	bsf		LATA, 1
	DelayNms	DRN, 10
	; 0110
	bsf		LATA, 3
	bcf		LATA, 1
	DelayNms	DRN, 10
	bra		loop_StepC180p

StepC180n
; Description:	Turns stepper C 90 degrees positive
; Input:	-
; Output:	-
; Regs Used:	stepperCounter
	movlw		d'100'
	movwf		stepperCounter
loop_StepC180n
	dcfsnz		stepperCounter
	return
	; 1100
	bcf		LATA, 3
	bcf		LATA, 1
	DelayNms	DRN, 10
	; 0110
	bsf		LATA, 3
	bcf		LATA, 1
	DelayNms	DRN, 10
	; 0011
	bsf		LATA, 3
	bsf		LATA, 1
	DelayNms	DRN, 10
	; 1001
	bcf		LATA, 3
	bsf		LATA, 1
	DelayNms	DRN, 10
	bra		loop_StepC180n

; DATA SUBROUTINES

AsciiConvert
; Description:	converts BCD digit to Ascii
; Input (W):	binary number 0-9 inclusive
; Output (W):	ascii code for number, ? if unknown
; Regs Used:	conversionReg	
	movwf		conversionReg
	movlw		0
conv0
	cpfseq		conversionReg
	bra		conv1
	movlw		d'48'
	return
conv1
	decfsz		conversionReg
	bra		conv2
	movlw		d'49'
	return
conv2
	decfsz		conversionReg
	bra		conv3
	movlw		d'50'
	return
conv3
	decfsz		conversionReg
	bra		conv4
	movlw		d'51'
	return
conv4
	decfsz		conversionReg
	bra		conv5
	movlw		d'52'
	return
conv5
	decfsz		conversionReg
	bra		conv6
	movlw		d'53'
	return
conv6
	decfsz		conversionReg
	bra		conv7
	movlw		d'54'
	return
conv7
	decfsz		conversionReg
	bra		conv8
	movlw		d'55'
	return
conv8
	decfsz		conversionReg
	bra		conv9
	movlw		d'56'
	return
conv9
	decfsz		conversionReg
	bra		convX
	movlw		d'57'
	return
convX
	movlw		d'63'			; = ?
	return

IncFile0
; Description:	increments file associated with Timer0. Always ensure lfsr is opTime0
; Input:	none
; Output:	none
; Regs Used:	opTime0:opTime4
	incf		INDF0
	movlw		d'10'
	cpfseq		INDF0
	goto		end_IncFile0
	clrf		INDF0
	incf		FSR0L
	call		IncFile0
	decf		FSR0L
end_IncFile0
	return

clear

; DELAY SUBROUTINES

Delay200us					; delay 497 cycles (10MHz Clock)
	movlw		0xA4			; 1 cycle
	movwf		reg200us		; 1 cycle
loop200us
	decfsz		reg200us		; (3 * 164) - 1 = 491 cycles
	goto		loop200us
	nop					; 1 cycle
	nop					; 1 cycle
	return					; 2 cycles
	
Delay1ms					; delay 2,500 cycles (10MHz Clock)
	movlw		0x04			; 1 cycle
	movwf		reg1ms			; 1 cycle
loop1ms
	call		Delay200us
	decfsz		reg1ms			; (497 + 3) * 4 - 1 = 1,999 Cycles
	goto		loop1ms
	call		Delay200us		; 497 cycles
	return					; 2 cycles

Delay10ms					; delay 25,000 cycles (10MHz Clock)
	movlw		0x31			; 1 cycle
	movwf		reg10ms			; 1 cycle
loop10ms
	call		Delay200us
	decfsz		reg10ms			; (497 + 3) * 49 - 1 = 24,499 Cycles
	goto		loop10ms
	call		Delay200us		; 497 cycles
	return					; 2 cycles

Delay100ms					; delay 250,000 cycles (10MHz Clock)
	movlw		0x0A
	movwf		reg100ms
loop100ms
	call		Delay10ms
	decfsz		reg100ms
	goto		loop100ms
	nop
	return

Delay1s						; delay 2,500,000 cycles (10MHz Clock)
	movlw		0x64
	movwf		reg1s
loop1s
	call		Delay10ms
	decfsz		reg1s
	goto		loop1s
	nop
	return

; DISPLAY SUBROUTINES

; Read Keypad Input
ReadKey
	btfss		PORTB, 1		; Wait until data is available from the keypad
	goto		ReadKey			; Once a key is pressed,
	swapf		PORTB, W		; Read PortB<7:4> into W<3:0>
	andlw		0x0F			; Mask 00001111
	movwf		key			; Save result in key
	btfsc		PORTB, 1		; Wait until key is released
	goto		$-2			; Back 1 instruction
	return

ReadKeyStandby
; Reset to first line
	call		LCD_L1
; Display year
	rtc_read	0x06
	WriteRTC
	movlw		0x2F			; ASCII '/'
	call		WR_DATA
; Display month
	rtc_read	0x05
	WriteRTC
	movlw		0x2F			; ASCII '/'
	call		WR_DATA
	rtc_read	0x04
	WriteRTC
; Display "START>A"
	movlw		" "
	call		WR_DATA
	movlw		"S"
	call		WR_DATA
	movlw		"T"
	call		WR_DATA
	movlw		"A"
	call		WR_DATA
	movlw		"R"
	call		WR_DATA
	movlw		"T"
	call		WR_DATA
	movlw		"~"			; ASCII '>'
	call		WR_DATA
	movlw		"A"
	call		WR_DATA
; Reset to second line
	call		LCD_L2
; Display hours
	rtc_read	0x02
	movf		tens_digit, W
	andlw		b'00000001'
	addlw		0x30
	call		WR_DATA
	movf		ones_digit, W
	call		WR_DATA
	movlw		":"
	call		WR_DATA
; Dispay minutes
	rtc_read	0x01
	WriteRTC
	movlw		":"
	call		WR_DATA
; Display seconds
	rtc_read	0x00
	WriteRTC
; Display 'LOGS>B'
	movlw		" "
	call		WR_DATA
	movlw		" "
	call		WR_DATA
	movlw		"L"
	call		WR_DATA
	movlw		"O"
	call		WR_DATA
	movlw		"G"
	call		WR_DATA
	movlw		"S"
	call		WR_DATA
	movlw		"~"			; ASCII '>'
	call		WR_DATA
	movlw		"B"
	call		WR_DATA
;Process key
	btfss		PORTB, 1		; Wait until data is available from the keypad
	goto		ReadKeyStandby		; Once a key is pressed,
	swapf		PORTB, W		; Read PortB<7:4> into W<3:0>
	andlw		0x0F			; Mask 00001111
	movwf		key			; Save result in key
	btfsc		PORTB, 1		; Wait until key is released
	goto		$-2			; Back 1 instruction
	return

; TABLING SUBROUTINES
	
; Description:	Accesses element of table at offset W
; Input:	W (table offset)
; Output:	W (value of  table)
; Regs Used:	TBLPTRU, TBLPTRH, TBLPTRL
AccessTable
	movwf		ATCounter
	incf		ATCounter
AT_loop
	incf		TBLPTRL
	
	decfsz		ATCounter
	bra		AT_loop
	tblrd*
	movf		TABLAT, W
	return


; EEPROM SUBROUTINES

ClearEEPROM_63
	clrf		EEPROM_H
	clrf		EEPROM_L
	clrf		clearCounter
ClearNext
	WriteEEPROM	d'0', EEPROM_H, EEPROM_L
	incf		clearCounter
	incf		EEPROM_L
	movlw		d'63'
	cpfseq		clearCounter
	goto		ClearNext
	clrf		EEPROM_H
	clrf		EEPROM_L
	return

InitializeEEPROM
	clrf		EEPROM_H
	clrf		EEPROM_L
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movlw		"*"
	cpfseq		EEReader
	bra		InitializeEEPROM_next
	return
InitializeEEPROM_next
	DispTableEE	EETable0
	DispTableEE	EETableA
	DispTableEE	EETableB
	DispTableEE	EETableC
	DispTableEE	EETableD
	DispTableEE	EETable1
	return

UpdateLogs
	movlw		0xDF
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
UpdateLogs1
	movlw		"4"
	cpfseq		EEReader
	bra		UpdateLogs2
	movlw		0x58
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	startYear1, EEPROM_H, EEPROM_L
	movlw		0x59
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	startYear0, EEPROM_H, EEPROM_L
	movlw		0x5B
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	startMonth1, EEPROM_H, EEPROM_L
	movlw		0x5C
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	startMonth0, EEPROM_H, EEPROM_L
	movlw		0x5E
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	startDay1, EEPROM_H, EEPROM_L
	movlw		0x5F
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	startDay0, EEPROM_H, EEPROM_L
	lfsr		0, opTime4
	movf		POSTDEC0, 0
	call		AsciiConvert
	movwf		EEReader
	movlw		0x6C
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		POSTDEC0, 0
	call		AsciiConvert
	movwf		EEReader
	movlw		0x6D
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		POSTDEC0, 0
	call		AsciiConvert
	movwf		EEReader
	movlw		0x6E
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	Ascii12		cansAlTab, digit1, digit0
	movlw		0x8E
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0x8F
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	Ascii12		cansAlNtb, digit1, digit0
	movlw		0x9E
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0x9F
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	Ascii12		cansSnLabel, digit1, digit0
	movlw		0xAE
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0xAF
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	Ascii12		cansSnNlbl, digit1, digit0
	movlw		0xBE
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0xBF
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	bsf		STATUS, 0
	movlw		maxCans
	subfwb		numCans
	Ascii12		numCans, digit1, digit0
	movlw		0xCE
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0xCF
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	movlw		0xDF
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	movlw		"1"
	movwf		EEReader
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	call		UpdateTables1
	return
UpdateLogs2
	movlw		"1"
	cpfseq		EEReader
	bra		UpdateLogs3
	movlw		0xF8
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	startYear1, EEPROM_H, EEPROM_L
	movlw		0xF9
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	startYear0, EEPROM_H, EEPROM_L
	movlw		0xFB
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	startMonth1, EEPROM_H, EEPROM_L
	movlw		0xFC
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	startMonth0, EEPROM_H, EEPROM_L
	movlw		0xFE
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	startDay1, EEPROM_H, EEPROM_L
	movlw		0xFF
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	WriteEEPROM	startDay0, EEPROM_H, EEPROM_L
	lfsr		0, opTime4
	movf		POSTDEC0, 0
	call		AsciiConvert
	movwf		EEReader
	movlw		0x0C
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		POSTDEC0, 0
	call		AsciiConvert
	movwf		EEReader
	movlw		0x0D
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		POSTDEC0, 0
	call		AsciiConvert
	movwf		EEReader
	movlw		0x0E
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	Ascii12		cansAlTab, digit1, digit0
	movlw		0x2E
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0x2F
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	Ascii12		cansAlNtb, digit1, digit0
	movlw		0x3E
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0x3F
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	Ascii12		cansSnLabel, digit1, digit0
	movlw		0x4E
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0x4F
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	Ascii12		cansSnNlbl, digit1, digit0
	movlw		0x5E
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0x5F
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	bsf		STATUS, 0
	movlw		maxCans
	subfwb		numCans
	Ascii12		numCans, digit1, digit0
	movlw		0x6E
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0x6F
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	movlw		0xDF
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	movlw		"2"
	movwf		EEReader
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	call		UpdateTables2
	return
UpdateLogs3
	movlw		"2"
	cpfseq		EEReader
	bra		UpdateLogs4
	movlw		0x98
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	startYear1, EEPROM_H, EEPROM_L
	movlw		0x99
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	startYear0, EEPROM_H, EEPROM_L
	movlw		0x9B
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	startMonth1, EEPROM_H, EEPROM_L
	movlw		0x9C
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	startMonth0, EEPROM_H, EEPROM_L
	movlw		0x9E
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	startDay1, EEPROM_H, EEPROM_L
	movlw		0x9F
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	startDay0, EEPROM_H, EEPROM_L
	lfsr		0, opTime4
	movf		POSTDEC0, 0
	call		AsciiConvert
	movwf		EEReader
	movlw		0xAC
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		POSTDEC0, 0
	call		AsciiConvert
	movwf		EEReader
	movlw		0xAD
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		POSTDEC0, 0
	call		AsciiConvert
	movwf		EEReader
	movlw		0xAE
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	Ascii12		cansAlTab, digit1, digit0
	movlw		0xCE
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0xCF
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	Ascii12		cansAlNtb, digit1, digit0
	movlw		0xDE
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0xDF
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	Ascii12		cansSnLabel, digit1, digit0
	movlw		0xEE
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0xEF
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	Ascii12		cansSnNlbl, digit1, digit0
	movlw		0xFE
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0xFF
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	bsf		STATUS, 0
	movlw		maxCans
	subfwb		numCans
	Ascii12		numCans, digit1, digit0
	movlw		0x0E
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0x0F
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	movlw		0xDF
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	movlw		"3"
	movwf		EEReader
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	call		UpdateTables3
	return
UpdateLogs4
	movlw		"3"
	cpfseq		EEReader
	bra		Error2
	movlw		0x38
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	startYear1, EEPROM_H, EEPROM_L
	movlw		0x39
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	startYear0, EEPROM_H, EEPROM_L
	movlw		0x3B
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	startMonth1, EEPROM_H, EEPROM_L
	movlw		0x3C
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	startMonth0, EEPROM_H, EEPROM_L
	movlw		0x3E
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	startDay1, EEPROM_H, EEPROM_L
	movlw		0x3F
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	startDay0, EEPROM_H, EEPROM_L
	lfsr		0, opTime4
	movf		POSTDEC0, 0
	call		AsciiConvert
	movwf		EEReader
	movlw		0x4C
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		POSTDEC0, 0
	call		AsciiConvert
	movwf		EEReader
	movlw		0x4D
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		POSTDEC0, 0
	call		AsciiConvert
	movwf		EEReader
	movlw		0x4E
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	Ascii12		cansAlTab, digit1, digit0
	movlw		0x6E
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0x6F
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	Ascii12		cansAlNtb, digit1, digit0
	movlw		0x7E
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0x7F
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	Ascii12		cansSnLabel, digit1, digit0
	movlw		0x8E
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0x8F
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	Ascii12		cansSnNlbl, digit1, digit0
	movlw		0x9E
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0x9F
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	bsf		STATUS, 0
	movlw		maxCans
	subfwb		numCans
	Ascii12		numCans, digit1, digit0
	movlw		0xAE
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	digit1, EEPROM_H, EEPROM_L
	movlw		0xAF
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	WriteEEPROM	digit0, EEPROM_H, EEPROM_L
	movlw		0xDF
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	movlw		"4"
	movwf		EEReader
	WriteEEPROM	EEReader, EEPROM_H, EEPROM_L
	call		UpdateTables4
	return

endlop3
	nop
	bra		endlop3

Error2
	call		ClrLCD
	call		LCD_L1
	DispTable	Err2
	bra		endlop3

UpdateTables1
	MOVLW     D'64'                      ; number of bytes in erase block
	MOVWF     COUNTER
	MOVLW     high BUFFER	             ; point to buffer
	MOVWF     FSR0H
	MOVLW     low BUFFER
	MOVWF     FSR0L
	MOVLW     upper LogA_L2           ; Load TBLPTR with the base
	MOVWF     TBLPTRU                   ; address of the memory block
	MOVLW     high LogA_L2
	MOVWF     TBLPTRH
	MOVLW     low LogA_L2
	MOVWF     TBLPTRL 
READ_BLOCK
	TBLRD*+                             ; read into TABLAT, and inc
	MOVF      TABLAT, W                 ; get data
	MOVWF     POSTINC0                  ; store data
	DECFSZ    COUNTER                   ;                   done?
	BRA       READ_BLOCK                ; repeat
MODIFY_WORD
	MOVLW     high BUFFER            ; point to buffer
	MOVWF     FSR0H
	MOVLW     low BUFFER
	MOVWF     FSR0L
	; modify buffer
	;"Time: XX0s TotalCans:XX Al+Tab:XX Al-Tab:XX Sn+Lbl:XX Sn-Lbl:XX ", 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x6C
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x6D
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x6E
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0xCE
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0xCF
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x8E
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x8F
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x9E
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x9F
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0xAE
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0xAF
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0xBE
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0xBF
	movwf		EEPROM_L
	movlw		0x00
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0

ERASE_BLOCK
	MOVLW     upper LogA_L2           ; load TBLPTR with the base
	MOVWF     TBLPTRU                   ; address of the memory block
	MOVLW     high LogA_L2
	MOVWF     TBLPTRH 
	MOVLW     low LogA_L2
	MOVWF     TBLPTRL
	BSF       EECON1, EEPGD             ; point to Flash program memory
	BCF       EECON1, CFGS              ; access Flash program memory
	BSF       EECON1, WREN              ; enable write to memory
	BSF       EECON1, FREE              ; enable Row Erase operation
	BCF       INTCON, GIE               ; disable interrupts
	MOVLW     55h
	MOVWF     EECON2                    ;                    write                    55h
	MOVLW     0AAh
	MOVWF     EECON2                    ;                    write                    0AAh
	BSF       EECON1, WR                ; start erase (CPU stall)
	BSF       INTCON, GIE               ; re-enable interrupts
	TBLRD*-                             ; dummy read decrement
	MOVLW     high BUFFER          ; point to buffer
	MOVWF     FSR0H
	MOVLW     low BUFFER
	MOVWF     FSR0L
WRITE_BUFFER_BACK
	MOVLW     D'64'                      ; number of bytes in holding register
	MOVWF     COUNTER
WRITE_BYTE_TO_HREGS
	MOVFF     POSTINC0, WREG            ; get low byte of buffer data
	MOVWF     TABLAT                   ; present data to table latch
	TBLWT+*                             ; write data, perform a short write 
	; to internal TBLWT holding register.
	DECFSZ    COUNTER                   ; loop until buffers are full
	BRA       WRITE_BYTE_TO_HREGS
PROGRAM_MEMORY
	MOVLW     upper LogA_L2           ; load TBLPTR with the base
	MOVWF     TBLPTRU                   ; address of the memory block
	MOVLW     high LogA_L2
	MOVWF     TBLPTRH 
	MOVLW     low LogA_L2
	MOVWF     TBLPTRL 
	BSF     EECON1, EEPGD         ; point to Flash program memory
	BCF     EECON1, CFGS          ; access Flash program memory
	BSF     EECON1, WREN          ; enable write to memory
	BCF     INTCON, GIE           ; disable interrupts
	MOVLW   55h
	MOVWF   EECON2                ; write 55h
	MOVLW   0AAh
	MOVWF   EECON2                ; write 0AAh
	BSF     EECON1, WR            ; start program (CPU stall)
	BSF     INTCON, GIE           ; re-enable interrupts
	BCF     EECON1, WREN          ; disable write to memory
	return

UpdateTables2
	MOVLW     D'64'                      ; number of bytes in erase block
	MOVWF     COUNTER
	MOVLW     high BUFFER	             ; point to buffer
	MOVWF     FSR0H
	MOVLW     low BUFFER
	MOVWF     FSR0L
	MOVLW     upper LogB_L2           ; Load TBLPTR with the base
	MOVWF     TBLPTRU                   ; address of the memory block
	MOVLW     high LogB_L2
	MOVWF     TBLPTRH
	MOVLW     low LogB_L2
	MOVWF     TBLPTRL 
READ_BLOCK2
	TBLRD*+                             ; read into TABLAT, and inc
	MOVF      TABLAT, W                 ; get data
	MOVWF     POSTINC0                  ; store data
	DECFSZ    COUNTER                   ;                   done?
	BRA       READ_BLOCK2                ; repeat
MODIFY_WORD2
	MOVLW     high BUFFER            ; point to buffer
	MOVWF     FSR0H
	MOVLW     low BUFFER
	MOVWF     FSR0L
	; modify buffer
	;"Time: XX0s TotalCans:XX Al+Tab:XX Al-Tab:XX Sn+Lbl:XX Sn-Lbl:XX ", 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x0C
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x0D
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x0E
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x6E
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x6F
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x2E
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x2F
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x3E
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x3F
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x4E
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x4F
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x5E
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x5F
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0

ERASE_BLOCK2
	MOVLW     upper LogB_L2           ; load TBLPTR with the base
	MOVWF     TBLPTRU                   ; address of the memory block
	MOVLW     high LogB_L2
	MOVWF     TBLPTRH 
	MOVLW     low LogB_L2
	MOVWF     TBLPTRL
	BSF       EECON1, EEPGD             ; point to Flash program memory
	BCF       EECON1, CFGS              ; access Flash program memory
	BSF       EECON1, WREN              ; enable write to memory
	BSF       EECON1, FREE              ; enable Row Erase operation
	BCF       INTCON, GIE               ; disable interrupts
	MOVLW     55h
	MOVWF     EECON2                    ;                    write                    55h
	MOVLW     0AAh
	MOVWF     EECON2                    ;                    write                    0AAh
	BSF       EECON1, WR                ; start erase (CPU stall)
	BSF       INTCON, GIE               ; re-enable interrupts
	TBLRD*-                             ; dummy read decrement
	MOVLW     high BUFFER          ; point to buffer
	MOVWF     FSR0H
	MOVLW     low BUFFER
	MOVWF     FSR0L
WRITE_BUFFER_BACK2
	MOVLW     D'64'                      ; number of bytes in holding register
	MOVWF     COUNTER
WRITE_BYTE_TO_HREGS2
	MOVFF     POSTINC0, WREG            ; get low byte of buffer data
	MOVWF     TABLAT                   ; present data to table latch
	TBLWT+*                             ; write data, perform a short write 
	; to internal TBLWT holding register.
	DECFSZ    COUNTER                   ; loop until buffers are full
	BRA       WRITE_BYTE_TO_HREGS2
PROGRAM_MEMORY2
	MOVLW     upper LogB_L2           ; load TBLPTR with the base
	MOVWF     TBLPTRU                   ; address of the memory block
	MOVLW     high LogB_L2
	MOVWF     TBLPTRH 
	MOVLW     low LogB_L2
	MOVWF     TBLPTRL 
	BSF     EECON1, EEPGD         ; point to Flash program memory
	BCF     EECON1, CFGS          ; access Flash program memory
	BSF     EECON1, WREN          ; enable write to memory
	BCF     INTCON, GIE           ; disable interrupts
	MOVLW   55h
	MOVWF   EECON2                ; write 55h
	MOVLW   0AAh
	MOVWF   EECON2                ; write 0AAh
	BSF     EECON1, WR            ; start program (CPU stall)
	BSF     INTCON, GIE           ; re-enable interrupts
	BCF     EECON1, WREN          ; disable write to memory
	return

UpdateTables3
	MOVLW     D'64'                      ; number of bytes in erase block
	MOVWF     COUNTER
	MOVLW     high BUFFER	             ; point to buffer
	MOVWF     FSR0H
	MOVLW     low BUFFER
	MOVWF     FSR0L
	MOVLW     upper LogC_L2           ; Load TBLPTR with the base
	MOVWF     TBLPTRU                   ; address of the memory block
	MOVLW     high LogC_L2
	MOVWF     TBLPTRH
	MOVLW     low LogC_L2
	MOVWF     TBLPTRL 
READ_BLOCK3
	TBLRD*+                             ; read into TABLAT, and inc
	MOVF      TABLAT, W                 ; get data
	MOVWF     POSTINC0                  ; store data
	DECFSZ    COUNTER                   ;                   done?
	BRA       READ_BLOCK3                ; repeat
MODIFY_WORD3
	MOVLW     high BUFFER            ; point to buffer
	MOVWF     FSR0H
	MOVLW     low BUFFER
	MOVWF     FSR0L
	; modify buffer
	;"Time: XX0s TotalCans:XX Al+Tab:XX Al-Tab:XX Sn+Lbl:XX Sn-Lbl:XX ", 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0xAC
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0xAD
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0xAE
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x0E
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x0F
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0xCE
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0xCF
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0xDE
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0xDF
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0xEE
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0xEF
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0xFE
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0xFF
	movwf		EEPROM_L
	movlw		0x01
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0

ERASE_BLOCK3
	MOVLW     upper LogC_L2           ; load TBLPTR with the base
	MOVWF     TBLPTRU                   ; address of the memory block
	MOVLW     high LogC_L2
	MOVWF     TBLPTRH 
	MOVLW     low LogC_L2
	MOVWF     TBLPTRL
	BSF       EECON1, EEPGD             ; point to Flash program memory
	BCF       EECON1, CFGS              ; access Flash program memory
	BSF       EECON1, WREN              ; enable write to memory
	BSF       EECON1, FREE              ; enable Row Erase operation
	BCF       INTCON, GIE               ; disable interrupts
	MOVLW     55h
	MOVWF     EECON2                    ;                    write                    55h
	MOVLW     0AAh
	MOVWF     EECON2                    ;                    write                    0AAh
	BSF       EECON1, WR                ; start erase (CPU stall)
	BSF       INTCON, GIE               ; re-enable interrupts
	TBLRD*-                             ; dummy read decrement
	MOVLW     high BUFFER          ; point to buffer
	MOVWF     FSR0H
	MOVLW     low BUFFER
	MOVWF     FSR0L
WRITE_BUFFER_BACK3
	MOVLW     D'64'                      ; number of bytes in holding register
	MOVWF     COUNTER
WRITE_BYTE_TO_HREGS3
	MOVFF     POSTINC0, WREG            ; get low byte of buffer data
	MOVWF     TABLAT                   ; present data to table latch
	TBLWT+*                             ; write data, perform a short write 
	; to internal TBLWT holding register.
	DECFSZ    COUNTER                   ; loop until buffers are full
	BRA       WRITE_BYTE_TO_HREGS3
PROGRAM_MEMORY3
	MOVLW     upper LogC_L2           ; load TBLPTR with the base
	MOVWF     TBLPTRU                   ; address of the memory block
	MOVLW     high LogC_L2
	MOVWF     TBLPTRH 
	MOVLW     low LogC_L2
	MOVWF     TBLPTRL 
	BSF     EECON1, EEPGD         ; point to Flash program memory
	BCF     EECON1, CFGS          ; access Flash program memory
	BSF     EECON1, WREN          ; enable write to memory
	BCF     INTCON, GIE           ; disable interrupts
	MOVLW   55h
	MOVWF   EECON2                ; write 55h
	MOVLW   0AAh
	MOVWF   EECON2                ; write 0AAh
	BSF     EECON1, WR            ; start program (CPU stall)
	BSF     INTCON, GIE           ; re-enable interrupts
	BCF     EECON1, WREN          ; disable write to memory
	return

UpdateTables4
	MOVLW     D'64'                      ; number of bytes in erase block
	MOVWF     COUNTER
	MOVLW     high BUFFER	             ; point to buffer
	MOVWF     FSR0H
	MOVLW     low BUFFER
	MOVWF     FSR0L
	MOVLW     upper LogD_L2           ; Load TBLPTR with the base
	MOVWF     TBLPTRU                   ; address of the memory block
	MOVLW     high LogD_L2
	MOVWF     TBLPTRH
	MOVLW     low LogD_L2
	MOVWF     TBLPTRL 
READ_BLOCK4
	TBLRD*+                             ; read into TABLAT, and inc
	MOVF      TABLAT, W                 ; get data
	MOVWF     POSTINC0                  ; store data
	DECFSZ    COUNTER                   ;                   done?
	BRA       READ_BLOCK4                ; repeat
MODIFY_WORD4
	MOVLW     high BUFFER            ; point to buffer
	MOVWF     FSR0H
	MOVLW     low BUFFER
	MOVWF     FSR0L
	; modify buffer
	;"Time: XX0s TotalCans:XX Al+Tab:XX Al-Tab:XX Sn+Lbl:XX Sn-Lbl:XX ", 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x4C
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x4D
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x4E
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0xAE
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0xAF
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x6E
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x6F
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x7E
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x7F
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x8E
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x8F
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movf	    POSTINC0, 0
	movlw		0x9E
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0
	movlw		0x9F
	movwf		EEPROM_L
	movlw		0x02
	movwf		EEPROM_H
	ReadEEPROM	EEReader, EEPROM_H, EEPROM_L
	movf		EEReader, 0
	movwf	    POSTINC0, 0

ERASE_BLOCK4
	MOVLW     upper LogD_L2           ; load TBLPTR with the base
	MOVWF     TBLPTRU                   ; address of the memory block
	MOVLW     high LogD_L2
	MOVWF     TBLPTRH 
	MOVLW     low LogD_L2
	MOVWF     TBLPTRL
	BSF       EECON1, EEPGD             ; point to Flash program memory
	BCF       EECON1, CFGS              ; access Flash program memory
	BSF       EECON1, WREN              ; enable write to memory
	BSF       EECON1, FREE              ; enable Row Erase operation
	BCF       INTCON, GIE               ; disable interrupts
	MOVLW     55h
	MOVWF     EECON2                    ;                    write                    55h
	MOVLW     0AAh
	MOVWF     EECON2                    ;                    write                    0AAh
	BSF       EECON1, WR                ; start erase (CPU stall)
	BSF       INTCON, GIE               ; re-enable interrupts
	TBLRD*-                             ; dummy read decrement
	MOVLW     high BUFFER          ; point to buffer
	MOVWF     FSR0H
	MOVLW     low BUFFER
	MOVWF     FSR0L
WRITE_BUFFER_BACK4
	MOVLW     D'64'                      ; number of bytes in holding register
	MOVWF     COUNTER
WRITE_BYTE_TO_HREGS4
	MOVFF     POSTINC0, WREG            ; get low byte of buffer data
	MOVWF     TABLAT                   ; present data to table latch
	TBLWT+*                             ; write data, perform a short write 
	; to internal TBLWT holding register.
	DECFSZ    COUNTER                   ; loop until buffers are full
	BRA       WRITE_BYTE_TO_HREGS4
PROGRAM_MEMORY4
	MOVLW     upper LogD_L2           ; load TBLPTR with the base
	MOVWF     TBLPTRU                   ; address of the memory block
	MOVLW     high LogD_L2
	MOVWF     TBLPTRH 
	MOVLW     low LogD_L2
	MOVWF     TBLPTRL 
	BSF     EECON1, EEPGD         ; point to Flash program memory
	BCF     EECON1, CFGS          ; access Flash program memory
	BSF     EECON1, WREN          ; enable write to memory
	BCF     INTCON, GIE           ; disable interrupts
	MOVLW   55h
	MOVWF   EECON2                ; write 55h
	MOVLW   0AAh
	MOVWF   EECON2                ; write 0AAh
	BSF     EECON1, WR            ; start program (CPU stall)
	BSF     INTCON, GIE           ; re-enable interrupts
	BCF     EECON1, WREN          ; disable write to memory
	return

	END
	
	
