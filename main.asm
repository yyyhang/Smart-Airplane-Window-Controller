; Author: Yuhang Yang
;zNumber: z5194094

.include "m2560def.inc"
	
.def row    =r22		; current row number
.def col    =r17		; current column number
.def rmask  =r18		; mask for current row
.def cmask	=r19		; mask for current column
.def temp1	=r20		
.def temp2  =r21
.def temp3  =r23
.def mark   =r9			; record the execute time for the timer in a press time
.def press  =r8			; record the press time for the timer in a press time

.equ INITCOLMASK = 0b11101111		; scan from the leftmost column, the value to mask output
.equ INITROWMASK = 0b00000001		; scan from the bottom row
.equ ROWMASK  =0b00001111			; low four bits are output from the keypad. This value mask the high 4 bits.

.equ LCD_RS = 7    
.equ LCD_E = 6       
.equ LCD_RW = 5  
.equ LCD_BE = 4

.macro do_lcd_command
	ldi r16, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro
.macro do_lcd_command_reg
	mov r16, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro
.macro do_lcd_data
	ldi r16, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro
.macro do_lcd_data_register
	mov r16, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro
.macro lcd_set
	sbi PORTA, @0
.endmacro
.macro lcd_clr
	cbi PORTA, @0
.endmacro
.macro Clear
    ldi YH,high(@0)
	ldi YL,low(@0)
	clr temp1
	st Y+,temp1
	st Y,temp1
.endmacro
.macro WinClear
    ldi YH,high(@0)
	ldi YL,low(@0)
	clr temp1
	st Y+,temp1
	st Y+,temp1
	st Y+,temp1
	st Y,temp1
.endmacro
.macro WinSet
    ldi YH,high(@0)
	ldi YL,low(@0)
	ldi temp1,3
	st Y+,temp1
	st Y+,temp1
	st Y+,temp1
	st Y,temp1
.endmacro

.dseg
.org 0x200				; start form 0x200 address
Counter:
    .byte 2				; counter for timer0
Counter1:
	.byte 2				; counter for timer1
Holder:
	.byte 0x30			; the value of pressed rows in sequence during a delay time
Pointer:
	.byte 0x30			; the sequence value need to be compared for Counter of timer0
Wins:
	.byte 4				; to store the four windows' states
;_____________________ ITERUPT ________________________
.cseg
     jmp RESET
.org INT0addr
     jmp EXT_INT0
.org INT1addr
     jmp EXT_INT1
.org INT2addr
     jmp EXT_INT2
.org OVF2addr
    jmp Timer2OVF
.org OVF0addr
    jmp Timer0OVF

EXT_INT0:			               ; the emergency state
	cli
    push temp1
    in temp1,SREG
    push temp1

	ldi temp1,0<<TOIE0			   ; disable all timers
	sts TIMSK0,temp1
	ldi temp1,0<<TOIE2
	sts TIMSK2,temp1

	do_lcd_command 0b10000001
	do_lcd_data '!'
	do_lcd_data '!'
	do_lcd_data ' '

	do_lcd_command 0b11000100
	do_lcd_data '0'
	do_lcd_command 0b11000111
	do_lcd_data '0'
	do_lcd_command 0b11001010
	do_lcd_data '0'
	do_lcd_command 0b11001101
	do_lcd_data '0'

	clr temp1					   ; set all PWM(compare register) to 0
	sts OCR1AL,temp1
	sts OCR1BL,temp1
	sts OCR3AL,temp1
	sts OCR3BL,temp1
	sts OCR4AL,temp1
	sts OCR4BL,temp1
	sts OCR5AL,temp1
	sts OCR5BL,temp1
	WinClear Wins				   ; set all windows' state to 0
	Clear Counter1				   ; reset the counter of timers
	Clear Counter

    pop temp1
    out SREG,temp1
    pop temp1
	sei
    jmp initial

EXT_INT1:				; central control: dark
    push temp1
    in temp1,SREG
    push temp1

	ldi temp1,0<<TOIE0
	sts TIMSK0,temp1
	ldi temp1,1<<TOIE2
	sts TIMSK2,temp1

	ldi temp1,0xff
	out PORTD,temp1				; poll up the interupt pin

	ldi temp2, '3'

	do_lcd_command 0b10000001
	do_lcd_data 'C'
	do_lcd_data ':'

	do_lcd_command 0b11000100
	do_lcd_data_register temp2
	do_lcd_command 0b11000111
	do_lcd_data_register temp2
	do_lcd_command 0b11001010
	do_lcd_data_register temp2
	do_lcd_command 0b11001101
	do_lcd_data_register temp2

    pop temp1
    out SREG,temp1
    pop temp1
    reti

EXT_INT2:						; central control: clear
    push temp1
    in temp1,SREG
    push temp1

	ldi temp1,0<<TOIE0			; disable timer0
	sts TIMSK0,temp1
	ldi temp1,1<<TOIE2			; enable timer2
	sts TIMSK2,temp1

	ldi temp1,0xff				; pull up the interupt2 pin
	sts PORTD,temp1
	ldi temp2, '0'				; in order to display '0' on LCD

	do_lcd_command 0b10000001	; set the address counter to DD RAM
	do_lcd_data 'C'
	do_lcd_data ':'

	do_lcd_command 0b11000100
	do_lcd_data_register temp2	; display '0'
	do_lcd_command 0b11000111
	do_lcd_data_register temp2
	do_lcd_command 0b11001010
	do_lcd_data_register temp2
	do_lcd_command 0b11001101
	do_lcd_data_register temp2

    pop temp1
    out SREG,temp1
    pop temp1
    reti

Timer2OVF:					; timer2, which will be used during central control
	push temp1
    in temp1, SREG
	push temp1
	push temp2
	push YH
	push YL
	push r17

	ldi YL,low(Counter1)	; load the address of Counter1
	ldi Yh,high(Counter1)
	ld r17,Y				; load the value of the address
	subi r17,-1
	cpi r17,62				; 62 = 1,000,000/(256*1024/16)
	brne Not1

	Clear Counter1			; reset counter

	cpi temp2, '3'			; if it is central control:set to dark, go to set_dark
	breq set_dark
	clr temp1				; else set all PWM duty cycle to 0, which can set all LED to dark 
	sts OCR1AL,temp1
	sts OCR1BL,temp1
	sts OCR3AL,temp1
	sts OCR3BL,temp1
	sts OCR4AL,temp1
	sts OCR4BL,temp1
	sts OCR5AL,temp1
	sts OCR5BL,temp1
	WinClear Wins			; set all windows states to 0
	rjmp done1

Not1:
	rjmp NotSecond1

set_dark:					; set all windows to dark
	ldi temp1,255			; set all PWM duty cycle to max value, which can set all LED to the highest light level
	sts OCR1AL,temp1
	sts OCR1BL,temp1
	sts OCR3AL,temp1
	sts OCR3BL,temp1
	sts OCR4AL,temp1
	sts OCR4BL,temp1
	sts OCR5AL,temp1
	sts OCR5BL,temp1
	WinSet Wins				; set all windows states to 0
	
done1:
	ldi temp1,0<<TOIE2		; after central control, disable timer2
	sts TIMSK2,temp1
	pop r17
	pop YL
	pop YH
	pop temp2
	pop temp1
	out SREG, temp1
	pop temp1
	jmp initial

NotSecond1:
	st Y,r17				; if not a second, store the value.
Endif1:
    pop r17
	pop YL
	pop YH
	pop temp2
	pop temp1
	out SREG, temp1
	pop temp1
reti

Timer0OVF:					; timer0, which will be used during local control
	push temp1
    in temp1, SREG
	push temp1
	push YH
	push YL
	push ZH
	push ZL
	push r17
	push r23
	push temp2

	ldi YL,low(Counter)		; load the address of Counter
	ldi Yh,high(Counter)
	ld r17,Y
	subi r17,-1

	ldi ZL,low(Pointer)		; load the address of Pointer
	ldi Zh,high(Pointer)
	add ZL,mark				; add the mark to Z to get relavant address
	ld temp1,Z				; load the value to temp1, which indicate the next execute time
	cp r17,temp1			; compare counter with the value to dicide whether execute. [62 = 1,000,000/(256*1024/16) as one second]
	brne Not

	ldi ZL,low(Holder)		; load the address of Holder
	ldi Zh,high(Holder)
	add ZL,mark
	ld r23, Z				; load the value to r23, which is the relavant row had been pressed at that time

	ldi ZL,low(Wins)		; load the address of Wins, which holds the state of current window
	ldi Zh,high(Wins)
	add ZL,r23
	ld temp2, Z				; load the value to temp2

	ldi temp1,85			; the duty cycle equal to 85 * the state level of this window
	mul temp2,temp1
	mov temp1,r0
	; to check whick window by comparing the row had been pressed at that time
	cpi r23,1
	breq win1
	cpi r23,2
	breq win2
	cpi r23,3
	breq win3
	rjmp win0

Not:
	rjmp NotSecond

win0:						; if it is window 1
	sts OCR1AL,temp1
	sts OCR1BL,temp1
	rjmp check
win1:
	sts OCR3AL,temp1
	sts OCR3BL,temp1
	rjmp check
win2:
	sts OCR4AL,temp1
	sts OCR4BL,temp1
	rjmp check
win3:
	sts OCR5AL,temp1
	sts OCR5BL,temp1

check:
	inc mark				; increase the how many times that the timer0 has executed in a press time

	ldi ZL,low(Pointer)
	ldi Zh,high(Pointer)
	add ZL,mark				; to load the next execute time
	ld temp1,Z
	cpi temp1,0				; if it is 0, means that it is the end of a press time
	breq endtimer			; otherwise, keep increasing Counter to match the next value (execute time)
	rjmp NotSecond

endtimer:
	Clear Counter
	Clear Pointer
	Clear Holder
	clr mark
	clr press
	ldi temp1,0<<TOIE0		;timer interupt mask register
	sts TIMSK0,temp1
	rjmp done

	ldi temp1,0<<TOIE0		;timer interupt mask register
	sts TIMSK0,temp1
done:
	rjmp Endif

NotSecond:
	st Y,r17

Endif:
	pop temp2
	pop r23
    pop r17
	pop ZL
	pop ZH
	pop YL
	pop YH
	pop temp1
	out SREG, temp1
	pop temp1
reti

;____________________ RESET ________________________
RESET:
    ldi temp1,(2<<ISC00)	; set interrupt as falling edge triggered interrupt
    sts EICRA,temp1
    in temp1,EIMSK
    ori temp1,(1<<INT0)
    out EIMSK,temp1
    ldi temp1,(2<<ISC10)
    sts EICRA,temp1
	ldi temp1,(2<<ISC20)
    sts EICRA,temp1

	Clear Counter			;set for the timer0
	ldi temp1,0x00			;just keep the timer count countrol register as initial
	out TCCR0A,temp1
	ldi temp1,0x05			;prescaler is 1024
	out TCCR0B,temp1

	Clear Counter1			;set for the timer2
	ldi temp1,0x00
	sts TCCR2A,temp1
	ldi temp1,0x07			;prescaler is 1024
	sts TCCR2B,temp1
	; initialize LCD
	ldi r16, low(RAMEND)
	out SPL, r16
	ldi r16, high(RAMEND)
	out SPH, r16

	ser r16
	out DDRF, r16
	out DDRA, r16
	clr r16
	out PORTF, r16
	out PORTA, r16

	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001100 ; Cursor off

	do_lcd_command 0b10000001 ; set DD RAM address
	do_lcd_data 'S'			  ; initial state
	do_lcd_data ':'
	do_lcd_data ' '
	do_lcd_data 'W'
	do_lcd_data '1'
	do_lcd_data ' '
	do_lcd_data 'W'
	do_lcd_data '2'
	do_lcd_data ' '
	do_lcd_data 'W'
	do_lcd_data '3'
	do_lcd_data ' '
	do_lcd_data 'W'
	do_lcd_data '4'

	do_lcd_command 0b11000100
	do_lcd_data '0'
	do_lcd_command 0b11000111
	do_lcd_data '0'
	do_lcd_command 0b11001010
	do_lcd_data '0'
	do_lcd_command 0b11001101
	do_lcd_data '0'

	WinClear Wins		   ; set all states of windows to 0

;_______________OCR5____________________LED 6,7
	ldi temp1,0b00011000	; pin PL3,PL4
	sts DDRL,temp1

	clr temp1				; initialize PWM duty cycle
	sts OCR5AH,temp1
	sts OCR5BH,temp1
	sts OCR5AL,temp1
	sts OCR5BL,temp1

	ldi temp1,(1<<CS50)								; set timer clock frequency (it is just same as timer setting)
	sts TCCR5B,temp1
	ldi temp1,(1<<WGM50)|(1<<COM5A1)|(1<<COM5B1)	; set timer5 to correct phase correct PWM mode and set compare output mode
	sts TCCR5A,temp1

;_______________OCR4____________________LED 4,5
	ldi temp1,0b00011000		;OC4 real pins are PE7,PE6, but setting are PORTH ph3,ph4???
	sts DDRH,temp1

	clr temp1
	sts OCR4AH,temp1
	sts OCR4BH,temp1
	sts OCR4AL,temp1
	sts OCR4BL,temp1

	ldi temp1,(1<<CS50)
	sts TCCR4B,temp1
	ldi temp1,(1<<WGM40)|(1<<COM4A1)|(1<<COM4B1)
	sts TCCR4A,temp1

;________________OCR3___________________LED 2,3
	ldi temp1,0b00011000	;OC3 real pins are PE5,PE2, but setting are PORTB PE3,PE4???
	out DDRE,temp1

	clr temp1
	sts OCR3AH,temp1
	sts OCR3AL,temp1
	sts OCR3BH,temp1
	sts OCR3BL,temp1

	ldi temp1,(1<<CS50)
	sts TCCR3B,temp1
	ldi temp1,(1<<WGM30)|(1<<COM3A1)|(1<<COM3B1)
	sts TCCR3A,temp1

;________________OCR1___________________LED 0,1
	ldi temp1,0b01100000	;OC1 real pins are PH11,PH12, but setting are PORTB PB5,PB6???
	out DDRB,temp1

	clr temp1
	sts OCR1AH,temp1
	sts OCR1AL,temp1
	sts OCR1BH,temp1
	sts OCR1BL,temp1

	ldi temp1,(1<<CS50)
	sts TCCR1B,temp1
	ldi temp1,(1<<WGM10)|(1<<COM1A1)|(1<<COM1B1)
	sts TCCR1A,temp1

initial:
	ldi temp1, 0xF0				; columns(4-7 K-pins) are outputs, rows(0-3pins) are inputs
	sts	DDRK, temp1
	ldi temp1,0x06				; pull up interupt pins
	out DDRD,temp1
	Clear Pointer				; reset some register and memory
	Clear Holder
	clr mark
	clr press
	sei							; enable interupt

;____________________ KEYPAD ________________________
main:
	ldi cmask, INITCOLMASK		; initial column mask (from fisrt column)
	clr	col						; initial column
colloop:
	cpi col, 3					; check if already all the columns, but without the last column
	breq main
	sts	PORTK, cmask			; scan from the leftmost column (set pin-4 as 0)

	ldi temp1, 0xFF				; delay 
delay:
	dec temp1
	brne delay

	lds	temp1, PINK				; read PORTK
	andi temp1, ROWMASK			; only need row bits, if don't do this, the other column pins have been set to 1 will be read

	ldi rmask, INITROWMASK		; initialise row check
	clr	row						; initial row
rowloop:
	cpi row, 4
	breq nextcol
	mov temp2, temp1
	and temp2, rmask			; check masked bit
	breq convert 				; if bit is clear, convert the bitcode
	inc row						; else move to the next row
	lsl rmask					; shift the mask to the next bit
	jmp rowloop

nextcol:
	lsl cmask					; else get new mask by shifting and 
	ori cmask,0xf
	inc col						; increment column value
	jmp colloop					; and check the next column

convert:
	cpi col, 0					; if column is 2 we have central control
	breq central
	cpi col,2
	breq down

	mov temp1, row				; I personally set this column as 0,3,6,9
	lsl temp1					; temp1 = row * 2
	add temp1, row				; temp1 = row * 3
	jmp convert_end

down:
	mov temp1, row				; I personally set this column as 15,18,21,24
	lsl temp1					; temp1 = row * 2
	add temp1, row				; temp1 = row * 3
	subi temp1, -15				; temp1 = row * 3 + 15
	jmp convert_end

central:						; if press the first collumn
	cpi row, 2					
	breq clearwin
	cpi row, 3					
	breq dark
	jmp main					
clearwin:						; central control: set all windows to clear
	in temp1,EIMSK
	ori temp1,(1<<INT2)			; enable interupt2
    out EIMSK,temp1
	ldi temp1,0xFF				; poll up interupt2 pin to 1
	out PORTD,temp1
	ldi temp1,0xFB				; then set the pin to 0 to trigger interupt 2
	out PORTD,temp1
	jmp waiting
dark:							; central control: set all windows to dark
	in temp1,EIMSK
	ori temp1,(1<<INT1)			; enable interupt1
    out EIMSK,temp1
	ldi temp1,0xff
	out PORTD,temp1
	ldi temp1,0xFD				; trigger falling edge at interupt1 pin
	out PORTD,temp1
	jmp waiting

convert_end:
	do_lcd_command 0b10000001	; set DD RAM address
	do_lcd_data 'L'
	do_lcd_data ':'

	ldi XL,low(Wins)			; get the address of Wins
	ldi Xh,high(Wins)
	add XL,row					; read the status of current window by adding row to Wins address
	ld temp2,X					; load the value to temp2

	; check the boundary status of current window
	cpi temp1, 10				
	brsh decreasing				; check the command (increasing or decreasing, less than 10 is increasing)
	cpi temp2, 3
	breq complete				; if it has been 3, keep it as 3. do nothing.
	inc temp2					; otherwise, incease the status of the window
	st X,temp2					; and store the current status back to Wins in a relative address
	; convert to display
	ldi temp3, 0xC4				; to let the dispaly can get properate DD RAM address
	rjmp display

decreasing:
	cpi temp2, 0				; if it has been 0, keep it as 0. do nothing.
	breq complete
	dec temp2					; else decrease the states level
	st X,temp2
	ldi temp3, 0xB5				; to let the dispaly can get properate DD RAM address

display:
	add temp3,temp1				; set DD RAM address
	do_lcd_command_reg temp3
	subi temp2,-'0'				; display the current status of window
	do_lcd_data_register temp2

	; prepare for timer to execute
	; store a sequence value (the pressed row in sequence during the delay time) in Holder for timer0 to read and execute LED
	ldi YL,low(Holder)
	ldi Yh,high(Holder)
	add YL,press				; to get the sequence memory address
	st Y,row					; store the value of row

	; get the address to store data in sequence
	ldi ZL,low(Pointer)
	ldi Zh,high(Pointer)
	add ZL,press				; to get the sequence memory address

	; what is the next(sequence) time for timer0 to compare
	ldi YL,low(Counter)
	ldi Yh,high(Counter)
	ld	temp1,Y
	subi temp1, -62				; as the prescaler is 1024, to delay 1s, every time needs to add  
	st Z+,temp1					; 62 to current Counter to get relavant value
	ldi temp1,0					; and store 0 to next address to indicate the end
	st Z,temp1

	; record the press time (detect key press whthin 1s after last key press)
	inc press

	; enable timer0
	ldi temp1,1<<TOIE0			;timer interupt mask register
	sts TIMSK0,temp1			;enable the overflow interupt

complete:
	rcall sleep_150ms			; delay 1.5ms to detect the keypress
	lds	temp1, PINK				; load the value from row pin again
	inc row						; else move to the next row
	lsl rmask					; shift the mask to the next bit
	jmp rowloop					; scan next row

waiting:
	sei							; wait for timer2
	rjmp waiting
;____________________ LCD SETTINGS ________________________
lcd_command:
	out PORTF, r16
	nop
	lcd_set LCD_E
	nop
	nop
	nop
	lcd_clr LCD_E
	nop
	nop
	nop
	ret

lcd_data:
	out PORTF, r16
	lcd_set LCD_RS
	nop
	nop
	nop
	lcd_set LCD_E
	nop
	nop
	nop
	lcd_clr LCD_E
	nop
	nop
	nop
	lcd_clr LCD_RS
	ret

lcd_wait:
	push r16
	clr r16
	out DDRF, r16
	out PORTF, r16
	lcd_set LCD_RW
lcd_wait_loop:
	nop
	lcd_set LCD_E
	nop
	nop
    nop
	in r16, PINF
	lcd_clr LCD_E
	sbrc r16, 7
	rjmp lcd_wait_loop
	lcd_clr LCD_RW
	ser r16
	out DDRF, r16
	pop r16
	ret

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

sleep_1ms:
	push r24
	push r25
	ldi r25, high(DELAY_1MS)
	ldi r24, low(DELAY_1MS)
delayloop_1ms:
	sbiw r25:r24, 1
	brne delayloop_1ms
	pop r25
	pop r24
	ret

sleep_5ms:
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
ret

sleep_30ms:
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
ret

sleep_150ms:
	rcall sleep_30ms
	rcall sleep_30ms
	rcall sleep_30ms
	rcall sleep_30ms
	rcall sleep_30ms
ret