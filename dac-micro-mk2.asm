; dac-micro-mk2 - main.asm
; Version: 1.0
; Author:  Matthew J. Wolf
; Date:    26 FEB 2017
;
; A microprocessor based Controller for Audio DAC
;
; This file is part of the Audio-DAC-Controler-MK3.
; By Matthew J. Wolf <matthew.wolf.hpsdr@speciosus.net>
; Copyright 2017 Matthew J. Wolf
;
; The Audio-DAC-Controler-MK3 is distributed in the hope that
; it will be useful, but WITHOUT ANY WARRANTY; without even the implied
; warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
; the GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with the Audio-DAC-Controler-MK3.
; If not, see <http://www.gnu.org/licenses/>.
;
;
; Serial Commands
; ---------------
; p - Power Cycle ON and Off
; O - Power ON
; o - Power OFF
; b - CS8416 Bypass
; i - Cycle thought inputs
; s - Status
; 0 - Input 0
; 1 - Input 1
; 2 - Input 2
; 3 - Input 3
; 4 - Input 4

    processor 16F628A
    radix     dec
    include p16f628a.inc

    __config _CP_OFF&_LVP_OFF&_BODEN_OFF&_MCLRE_ON&_PWRTE_ON&_WDT_OFF&_FOSC_INTOSCIO

; ****************************************************************************
; *                    Port and EEPROM Constants                             *
; ****************************************************************************
;
PortA   equ     0x05
PortB   equ     0x06
TRISA   equ     0x85
TRISB   equ     0x86

; ****************************************************************************
; * RAM page independent file registers:                                     *
; ****************************************************************************
STATUS  EQU     0x03

; *****************************************************************************
; * Bit numbers for the STATUS file register:                                 *
; *****************************************************************************
;
B_RP0   equ     5

; Manifest Constants -------------------------------------------------
PB_IS       equ		H'04'		; PORTA PIN for PB1
PB_BS	    equ		H'03'		; PORTA pin for PB2
PB_PW       equ         H'02'		; PORTA pin for PB3

LED_5	    equ		H'07'		; PORTB pin for Input 5
LED_I0      equ		H'06'		; PORTB pin for LED1
LED_I1      equ         H'05'		; PORTB pin for LED2
LED_BS	    equ		H'04'		; PORTB pin for LED3
LED_PW      equ         H'03'		; PORTB pin for LED4

INPORTNUM   equ		H'05'		; Number of Input Ports

; File Registers ------------------------------------------------------
    udata 0x0020
Buttons		res			1
PB_State	res			1
Input_Num	res			1
Out_State	res			1
RXData		res			1
Test		res		        1

Flags		res			1	; byte to store indicator flags
RFlags		res			1
CommandFlags	res			1
; EEPROM Initialization -------------------------------------------------------
DATAEE    	org  		0x2100
    data		H'00',H'00',H'00'
			
			
;-------------------------------------------------------------------------------
STARTUP 	org     	0x0000      ; processor reset vector
    clrf    	PCLATH
    goto		Start

Start

    clrf    INTCON            ; No interrupts for now

    banksel VRCON
    movlw   0x00              ; Disable Volage Ref
    movwf   VRCON

    banksel CMCON
    movlw   0x07              ; Code to turn off the analog comparitors
    movwf   CMCON             ; Turn off comparators

    bsf     STATUS,B_RP0      ; Switch to bank 1
    movlw   0xFF              ; Tristate PortA
    movwf   TRISA             ;
    clrf    TRISB             ; Set port B to all outputs

    bsf     TRISB,PB_PW       ; set PW Button input

    bcf     STATUS,B_RP0      ; Switch back to bank 0

;Init -------------------------------------------------------------------
    clrf	PORTA			  ; Clear Outputs
    clrf	Buttons
    clrf	PB_State
    clrf	Out_State		  ; Clear Outputs
    clrf	Input_Num

    call	SerialSetup

    ;Restore Input
    bsf         STATUS,RP0         ; Bank 1
    movlw       H'00'
    movwf       EEADR              ; Address to read
    bsf         EECON1,RD          ; EE Read
    movf        EEDATA,W
    bcf         STATUS,RP0         ; Bank 0

    movwf       Input_Num

    movf        Input_Num,W
    xorlw       0x00
    btfsc       STATUS,Z
    goto        IS_Reset_Rollover

    decf        Input_Num          ; Counter Fuge

Skip_Dec
    call	Restore_InPut

    ;Restore Bypass
    bsf         STATUS,RP0         ; Bank 1
    movlw       H'01'
    movwf       EEADR              ; Address to read
    bsf         EECON1,RD          ; EE Read
    movf        EEDATA,W
    bcf         STATUS,RP0         ; Bank 0

    movwf       Out_State

    call        Restore_Bypass

    movlw       "H"
    call        SerialSendChar
    movlw       "I"
    call        SerialSendChar

    call        SerialSendChar

    call        SerNewLine


Loop
    btfss	Out_State,LED_PW     ; Skip if PW ON; go if off
    goto	Power_is_OFF

    call	Input_Sel
    call	Bypass_Sel

Power_is_OFF
    call	ReceiveSerial
    call	Power

    goto 	Loop

; Subroutines ------------------------------------------------------------
Input_Sel
    movf		PORTA,W			; Get inputs
    movwf		Buttons

			; Check button state
    btfss		PB_State,PB_IS		; Was button down?
    goto		PB_IS_wasDown		; Yes

PB_IS_wasUp
    btfsc		Buttons,PB_IS		; Is button still up?
    return					; Was up and still up, do nothing
    bcf			PB_State,PB_IS		; Was up, remember now down
    return

PB_IS_wasDown
    btfss		Buttons,PB_IS		; If it is still down
    return					; Was down and still down, do nothing
    bsf			PB_State,PB_IS		; remember released

    incf		Input_Num,F		; Button was down and now it's up,
						; we need to increment the counter
    bcf			STATUS,Z
    movf		Input_Num,W		; Move Input Port Count to W
    xorlw		INPORTNUM		; Is the Port Count equal the
    btfsc		STATUS,Z		; to the max port number
    clrf		Input_Num		; if Yes clear port count

    call 		Change_InPut

    return

Change_InPut
    movf		Input_Num,W
    xorlw		0x00
    btfsc		STATUS,Z
    goto		Input0

    movf		Input_Num,W
    xorlw		0x01
    btfsc		STATUS,Z
    goto		Input1

    movf		Input_Num,W
    xorlw		0x02
    btfsc		STATUS,Z
    goto		Input2

    movf		Input_Num,W
    xorlw		0x03
    btfsc		STATUS,Z
    goto		Input3

    movf		Input_Num,W
    xorlw		0x04
    btfsc		STATUS,Z
    goto		Input4

    return

Input0
    banksel		0
    bcf			PortB,LED_I0  		; off
    bcf			PortB,LED_I1  		; off
    bcf			PortB,LED_5		; off

    goto		EEPROM_IS_Write

    return
Input1
    banksel		0
    bsf			PortB,LED_I0  		; on
    bcf			PortB,LED_I1  		; off
    bcf			PortB,LED_5		; off

    goto		EEPROM_IS_Write

    return

Input2
    banksel		0
    bcf			PortB,LED_I0  		; off
    bsf			PortB,LED_I1  		; on
    bcf			PortB,LED_5		; off

    goto		EEPROM_IS_Write

    return

Input3
    banksel		0
    bsf			PortB,LED_I0	  	; on
    bsf			PortB,LED_I1  		; on
    bcf			PortB,LED_5		; off

    goto		EEPROM_IS_Write

    return

Input4
    banksel		0
    bcf			PortB,LED_I0	  	; off
    bcf			PortB,LED_I1  		; off
    bsf			PortB,LED_5		; on

    goto		EEPROM_IS_Write

    return

Bypass_Sel

    movf		PORTA,W			; Get inputs
    movwf		Buttons

			; Check button state
    btfss		PB_State,PB_BS		; Was button down?
    goto		PB_BS_wasDown		; Yes

PB_BS_wasUp
    btfsc		Buttons,PB_BS		; Is button still up?
    return					; Was up and still up, do nothing
    bcf			PB_State,PB_BS		; Was up, remember now down
    return

PB_BS_wasDown
    btfss		Buttons,PB_BS		; If it is still down
    return					; Was down and still down, do nothing
    bsf			PB_State,PB_BS		; remember released

    btfss		Out_State,LED_BS	; Skip if BS ON; go if off
    goto		BS_On

    btfsc		Out_State,LED_BS	; Skip if BS OFF; go if on
    goto		BS_Off

    return


BS_On
    banksel		PortB

    bsf			PortB,LED_BS		; LED on
    bsf			Out_State,LED_BS

    goto		EEPROM_BS_Write

    return
BS_Off
    banksel		PortB

    bcf			PortB,LED_BS		; LED off
    bcf			Out_State,LED_BS

    goto		EEPROM_BS_Write

    return
;--------------
Power

    movf		PORTA,W			; Get inputs
    movwf		Buttons

			; Check button state
    btfss		PB_State,PB_PW		; Was button down?
    goto		PB_PW_wasDown		; Yes

PB_PW_wasUp
    btfsc		Buttons,PB_PW		; Is button still up?
    return					; Was up and still up, do nothing
    bcf			PB_State,PB_PW		; Was up, remember now down
    return

PB_PW_wasDown
    btfss		Buttons,PB_PW		; If it is still down
    return					; Was down and still down, do nothing
    bsf			PB_State,PB_PW		; remember released

    btfss		Out_State,LED_PW	; Skip if BS ON; go if off
    goto		PW_On

    btfsc		Out_State,LED_PW	; Skip if BS OFF; go if on
    goto		PW_Off

    return


PW_On
    banksel		PortB

    bsf			PortB,LED_PW		; LED on
    bsf			Out_State,LED_PW

    call		Restore_InPut
    call		Restore_Bypass

    return
PW_Off
    banksel		PortB

    bcf			PortB,LED_PW		; LED off
    bcf			Out_State,LED_PW

    bcf			PortB,LED_I0  		; off
    bcf			PortB,LED_I1  		; off

    bcf			PortB,LED_BS		; LED off

    bcf			PortB,LED_5		; LED off

    return

EEPROM_IS_Write
    banksel		Input_Num
    movf		Input_Num,w

    banksel		EEDATA

    movwf		EEDATA			; Byte to write

    movlw		H'00'			; Set the EEPROM address
    movwf		EEADR

    bsf			STATUS,RP0		; Bank 1
    bsf			EECON1,WREN		; Enable Write
    bcf			INTCON,GIE		; Disable Int
    btfsc		INTCON,GIE
    goto		$-2
    movlw		H'55'			; -Start of Required Sequence
    movwf		EECON2			; Write 0x55
    movlw		H'AA'
    movwf		EECON2			; Write 0XAA
    bsf			EECON1,WR		; Set WR bit to write
						; -End of Required Sequence
    bsf			INTCON,GIE		; Enable Interutps

    banksel		0			; return to main bank
    return

EEPROM_BS_Write
    banksel		Out_State
    movf		Out_State,w

    banksel		EEDATA

    movwf		EEDATA			; Byte to write

    movlw		H'01'			; Set the EEPROM address
    movwf		EEADR

    bsf			STATUS,RP0		; Bank 1
    bsf			EECON1,WREN		; Enable Write
    bcf			INTCON,GIE		; Disable Int
    btfsc		INTCON,GIE
    goto		$-2
    movlw		H'55'			; -Start of Required Sequence
    movwf		EECON2			; Write 0x55
    movlw		H'AA'
    movwf		EECON2			; Write 0XAA
    bsf			EECON1,WR		; Set WR bit to write
						; -End of Required Sequence
    bsf			INTCON,GIE		; Enable Interutps

    banksel		0			; return to main bank

    return

EEPROM_IS_Read

    BSF			STATUS,RP0		; Bank 1
    movlw		H'00'
    movwf		EEADR			; Address to read
    bsf			EECON1,RD		; EE Read
    movf		EEDATA,W
    bcf			STATUS,RP0	        ; Bank 0

    movwf		Input_Num

    return

Restore_InPut
    movf		Input_Num,W
    xorlw		0x00
    btfsc		STATUS,Z
    goto		Restore_0

    movf		Input_Num,W
    xorlw		0x01
    btfsc		STATUS,Z
    goto		Restore_1

    movf		Input_Num,W
    xorlw		0x02
    btfsc		STATUS,Z
    goto		Restore_2

    movf		Input_Num,W
    xorlw		0x03
    btfsc		STATUS,Z
    goto		Restore_3

    movf		Input_Num,W
    xorlw		0x04
    btfsc		STATUS,Z
    goto		Restore_4

    return

Restore_0
    banksel		PortB
    nop
    bcf			PortB,LED_I0  		; off
    nop
    bcf			PortB,LED_I1  		; off
    nop
    bcf			PortB,LED_5		; off
    return
Restore_1
    banksel		PortB
    nop
    bsf			PortB,LED_I0  		; on
    nop
    bcf			PortB,LED_I1  		; off
    nop
    bcf			PortB,LED_5		; off
    return
Restore_2
    banksel		PortB
    nop
    bcf			PortB,LED_I0  		; off
    nop
    bsf			PortB,LED_I1  		; on
    nop
    bcf			PortB,LED_5		; off
    return
Restore_3
    banksel		PortB
    nop
    bsf			PortB,LED_I0	  	; on
    nop
    bsf			PortB,LED_I1  		; on
    nop
    bcf			PortB,LED_5		; off
    return
Restore_4
    banksel		PortB
    nop
    bcf			PortB,LED_I0	  	; off
    nop
    bcf			PortB,LED_I1  		; off
    nop
    bsf			PortB,LED_5		; on
    return

IS_Reset_Rollover
    movlw		H'04'
    movwf		Input_Num
    goto		Skip_Dec
    return

Restore_Bypass
    btfss		Out_State,LED_BS	; Skip if BS ON; go if off
    goto		Restore_BS_Off

    btfsc		Out_State,LED_BS	; Skip if BS OFF; go if on
    goto		Restore_BS_On
    return

Restore_BS_On
    bsf			PortB,LED_BS		; LED on
    return

Restore_BS_Off
    bcf			PortB,LED_BS		; LED off
    return

SerialSetup
    banksel		TRISB
    bcf 		TRISB,2			; TX pin
    bsf			TRISB,1			; RX

    movlw		D'25'			; 9600 at 4MHZ
    movwf		SPBRG			; w/ brgh high

    bsf 		TXSTA, TXEN		; enable TX
    bcf 		TXSTA, SYNC		; disable synchronous
    bsf			TXSTA, BRGH		; Low speed

    banksel		RCSTA
    bsf 		RCSTA, SPEN		; select TX and RX pins
    bcf			RCSTA, RX9		; 8 bits reception
    bsf 		RCSTA, CREN		; enable RX

    clrf		Flags			; clear all flags
    clrf		RFlags
    clrf		CommandFlags


    banksel		0
    return

ReceiveSerial
    banksel		PIR1
    btfss		PIR1,RCIF		; check if data
    return					; return if no data

    banksel		RCSTA
    btfsc		RCSTA,OERR		; if overrun error occurred
    goto		ErrSerialOverr		; then go handle error

    banksel		RCSTA
    btfsc		RCSTA,FERR		; if framing error occurred
    goto		ErrSerialFrame		; then go handle error

ReceiveSer1
    banksel		RCREG

    movf		RCREG,W			; get received data
    movwf		RXData

    btfss		Out_State,LED_PW	; Skip if PW ON; go if off
    goto		Serial_Off_is_Off

    bcf			STATUS,Z
    movf		RXData,W
    xorlw		0x30			; compare with 0
    btfsc		STATUS,Z
    goto		Serial_Input0

    movf		RXData,W
    xorlw		0x31			; compare with 1
    btfsc		STATUS,Z
    goto		Serial_Input1

    movf		RXData,W
    xorlw		0x32			; compare with 2
    btfsc		STATUS,Z
    goto		Serial_Input2

    movf		RXData,W
    xorlw		0x33			; compare with 3
    btfsc		STATUS,Z
    goto		Serial_Input3

    movf		RXData,W
    xorlw		0x34			; compare with 4
    btfsc		STATUS,Z
    goto		Serial_Input4

    movf		RXData,W
    xorlw		0x62			; compare with b
    btfsc		STATUS,Z
    goto		Serial_Bypass

Serial_Off_is_Off
    movf		RXData,W
    xorlw		0x70			; compare with p
    btfsc		STATUS,Z
    goto		Serial_Power

    movf		RXData,W
    xorlw		0x73			; compare with s
    btfsc		STATUS,Z
    goto		Serial_Status

    movf		RXData,W
    xorlw		0x69			; compare with i
    btfsc		STATUS,Z
    goto		Serial_Input_Cycle

    movf		RXData,W
    xorlw		0x4F			; compare with O
    btfsc		STATUS,Z
    goto		Serial_PW_On

    movf		RXData,W
    xorlw		0x6f			; compare with o
    btfsc		STATUS,Z
    goto		Serial_PW_Off

    return

Serial_Input0
    banksel		0
    bcf			PortB,LED_I0  		; off
    bcf			PortB,LED_I1  		; off
    bcf			PortB,LED_5		; off

    movlw		H'00'
    movwf		Input_Num

    goto		EEPROM_IS_Write

    return

Serial_Input1
    banksel		0
    bsf			PortB,LED_I0  		; on
    bcf			PortB,LED_I1  		; off
    bcf			PortB,LED_5		; off

    movlw		H'01'
    movwf		Input_Num

    goto		EEPROM_IS_Write

    return

Serial_Input2
    banksel		0
    bcf			PortB,LED_I0  		; off
    bsf			PortB,LED_I1  		; on
    bcf			PortB,LED_5		; off

    movlw		H'02'
    movwf		Input_Num

    goto		EEPROM_IS_Write

    return

Serial_Input3
    banksel		0
    bsf			PortB,LED_I0	  	; on
    bsf			PortB,LED_I1  		; on
    bcf			PortB,LED_5		; off

    movlw		H'03'
    movwf		Input_Num

    goto		EEPROM_IS_Write

    return

Serial_Input4
    banksel		0
    bcf			PortB,LED_I0	  	; off
    bcf			PortB,LED_I1  		; off
    bsf			PortB,LED_5		; on

    movlw		H'04'
    movwf		Input_Num

    goto		EEPROM_IS_Write

    return

Serial_Bypass
    btfss		Out_State,LED_BS	; Skip if BS ON; go if off
    goto		BS_On

    btfsc		Out_State,LED_BS	; Skip if BS OFF; go if on
    goto		BS_Off

    return

Serial_Power
    btfss		Out_State,LED_PW	; Skip if PW ON; go if off
    goto		PW_On

    btfsc		Out_State,LED_PW	; Skip if PW OFF; go if on
    goto		PW_Off

    return

Serial_PW_On
    goto		PW_On
    return

Serial_PW_Off
    goto		PW_Off
    return

Serial_Input_Cycle
    incf		Input_Num,F

    bcf			STATUS,Z
    movf		Input_Num,W		; Move Input Port Count to W
    xorlw		INPORTNUM		; Is the Port Count equal the
    btfsc		STATUS,Z		; to the max port number
    clrf		Input_Num		; if Yes clear port count

    call 		Change_InPut

    return


Serial_Status

    movlw		"p"
    call		SerialSendChar
    movlw		","
    call		SerialSendChar

    btfss		Out_State,LED_PW	; Skip if PW ON; go if off
    movlw		"0"

    btfsc		Out_State,LED_PW	; Skip if PW OFF; go if on
    movlw		"1"

    call		SerialSendChar

    movlw		","
    call		SerialSendChar

    movlw		"i"
    call		SerialSendChar
    movlw		","
    call		SerialSendChar

    movf		Input_Num,W
    xorlw		0x00
    btfsc		STATUS,Z
    goto		Status_0

    movf		Input_Num,W
    xorlw		0x01
    btfsc		STATUS,Z
    goto		Status_1

    movf		Input_Num,W
    xorlw		0x02
    btfsc		STATUS,Z
    goto		Status_2

    movf		Input_Num,W
    xorlw		0x03
    btfsc		STATUS,Z
    goto		Status_3

    movf		Input_Num,W
    xorlw		0x04
    btfsc		STATUS,Z
    goto		Status_4

Status_0
    movlw		H'30'
    call		SerialSendChar
    goto		Status_Bypass
Status_1
    movlw		H'31'
    call		SerialSendChar
    goto		Status_Bypass
Status_2
    movlw		H'32'
    call		SerialSendChar
    goto		Status_Bypass
Status_3
    movlw		H'33'
    call		SerialSendChar
    goto		Status_Bypass
Status_4
    movlw		H'34'
    call		SerialSendChar
    goto		Status_Bypass

Status_Bypass
    movlw		","
    call		SerialSendChar

    movlw		"b"
    call		SerialSendChar
    movlw		","
    call		SerialSendChar

    btfss		Out_State,LED_BS	; Skip if BS ON; go if off
    movlw		"0"

    btfsc		Out_State,LED_BS	; Skip if BS OFF; go if on
    movlw		"1"

    call		SerialSendChar

    call		SerNewLine


    return

; Error because OERR overrun error bit is set
; Can do special error handling here - this code simply clears and continues
ErrSerialOverr
    banksel		RCSTA
    bcf			RCSTA,CREN		; reset the receiver logic
    bsf			RCSTA,CREN		; enable reception again
    return

; Error because FERR framing error bit is set
; Can do special error handling here - this code simply clears and continues
ErrSerialFrame
    banksel		RCREG
    movf		RCREG,W			; discard received data that has error
    return

SerialSendChar
    banksel		PIR1
    btfss 		PIR1, TXIF
    goto		$-1
    banksel		TXREG
    movwf 		TXREG

    banksel		0
    return
SerNewLine
    movlw		0x0A			; Line Feed
    call		SerialSendChar

    movlw		0x0D			; Carriage Return
    call		SerialSendChar

    banksel		0
    return

CopyRight
    dt			"Copyright 2017 Matthew J. Wolf ",0
    dt			"WITHOUT ANY WARRANTY; without even the implied ",0
    dt			"warranty of MERCHANTABILITY or FITNESS FOR A ",0
    dt			"PARTICULAR PURPOSE. See GNU General Public ",0
    dt			"License for more details.",0
    
    end