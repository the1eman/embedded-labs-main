; Lab
;4.asm
; Created: 3/13/2022 2:50:07 PM
; Author : Emmanuel A

;Vector Table-------------------------------------
.ORG 0 ;location for reset
JMP start
.ORG 0x06 ;loc. for PCINT0
JMP PCINT0_ISR
.ORG 0x08 ;loc. for PCINT1
JMP PCINT1_ISR


;.org 0x000E ; Timer/COunter 2 compare match 2
;.org 0x0020 ; Timer/COunter 0 Overflow

;pin 1 to gnd, pin 14 to vcc
;r16: free use r17: r18: r19: r20:timer temp r21:timer temp r22:timer count r23:5mil loop r24: r25: r26:rpg direction r27: r28: r29: r30: r31

start:
.cseg
.include "m328Pdef.inc"

;prepare for interrupts
ldi r24,HIGH(RAMEND)
out SPH,r24
ldi r24,LOW(RAMEND)
out SPL,r24
sbi DDRB,2
cbi PORTB,2

;enable PB4 PCINTs
LDI r24,0b00010000
STS PCMSK0,r24
;enable PORTB change interrupt
LDI r24,(1<<PCIE0)
STS PCICR,r24

;enable PB0PCINTs
LDI r24,0b00000001
STS PCMSK1,r24

;enable PB1 PCINTs
LDI r24,0b00000010
STS PCMSK2,r24


;LCD specific outputs
sbi DDRB,5
sbi DDRB,3
.equ rs=PORTB5;r/w goes to ground
.equ e=PORTB3 ;enable

sbi DDRB,2
cbi PORTB,2

;LCD Screen Outputs
sbi DDRC,1
sbi DDRC,2
sbi DDRC,3
sbi DDRC,0

cbi DDRB,4 ;input, push button
cbi DDRB,0 ;rpg channel b
cbi DDRB,1 ;rpg channel a

.def free_use = r16 ;a register used for misc tasks
.def looper = r17
.def mode=R18

ldi R26,0

.set LED_status=0 ;variable to compare LED status, 0 if off, 1 if on
.set RPG_direction=0 ;2 is cw, 1 is ccw
.set state_count=0
.set DC_value=50

rcall LCD_init

start_up:
sbi PORTB,2
rcall sendMessage1
rcall lineSwitch
rcall sendMessage2
cbi PORTB,2

rcall delayS
rcall delayS
rcall LCD_init

rcall lineRestart
rcall send_DC_Message
rcall lineSwitch
rcall send_LED_Message_off

ldi r16,0
ldi r27,0



sei

main:

rjmp main

;ISRs---------------------------------------------------
PCINT0_ISR:
sbis portb,2 ;skip if the led is on
rjmp LED_is_off

sbic portb,2; skip if the led is off
rjmp LED_is_on

LED_is_off:
sbi portb,2
rcall send_LED_Message_on
RETI

LED_is_on:
cbi portb,2
rcall send_LED_Message_off
RETI

PCINT1_ISR: ;channel a
.set DC_value = DC_value+1
;ldi R30,DC_value
RETI

PCINT2_ISR: ;channel b
.set DC_value = DC_value-1
;R30,DC_value
RETI


delayCS:
rcall wait0_1s
ret
delayS:
rcall delayCS
rcall delayCS
rcall delayCS
rcall delayCS
rcall delayCS
rcall delayCS
rcall delayCS
rcall delayCS
rcall delayCS
rcall delayCS
ret

LCD_init:
cbi PORTB,5 ;put the RS in command mode
//8-bit mode
;rcall RS_low
rcall wait0_1s ;1.) wait 100ms

ldi r25,0x03
rcall write_command ;2.) write D7-4=3, w\ RS=0
rcall delay5Mil ;3.) wait 5ms

ldi r25,0x03
rcall write_command ;4.) write D7-4=3, w\ RS=0
rcall delay200u ;5.) wait 200us

ldi r25,0x03
rcall write_command ;6.) write D7-4=3, w\ RS=0
rcall delay200u ;7.) wait 200us
ldi r25,0x02
rcall write_command ;8.) write D7-4=2, w\ RS=0

rcall delay5Mil ;9.) wait 5ms

//4-bit mode:
rcall write_SetInt ;10.) write command "set interface", write 28 hex(4-bits, 2 lines)
rcall delay5Mil
rcall write_EnDisp ;11.) write command "enable display/cursor", write 08 hex
rcall delay5Mil
rcall write_ClrHm ;12.) write command "clear and home", write 01 hex
rcall delay5Mil
rcall write_SetCurMvDir ;13.) write command "set cursor move direction", write 06 hex
rcall delay5Mil
rcall turn_on ;14.) write 0C hex to turn on display
rcall delay5Mil

sbi PORTB,5

;display is ready to receive data.
ret


;LCDstr:.db 0x33,0x32,0x28,0x01,0x0c,0x06

//sending characters to LCD in 4-bit
; Send the character 'E' to the LCD. The ASCII
; character 'E' is 0x45
;message information
msg1: .db "Ready to begin" ;14 chars
.dw 0

msg2: .db "Really" ;6 chars
.dw 0

DC_msg: .db "DC=50%" ;6 chars
.dw 0
LED_msg_off: .db "LED: off" ;8 chars
.dw 0

LED_msg_on: .db "LED: on " ;8 chars
.dw 0

sendMessage1:
ldi r30,LOW(2*msg1) ; Load Z register low
ldi r31,HIGH(2*msg1) ; Load Z register high
ldi r28,14
rcall displayCString
ret

sendMessage2:
ldi r30,LOW(2*msg2) ; Load Z register low
ldi r31,HIGH(2*msg2) ; Load Z register high
ldi r28,6
rcall displayCString
ret

send_DC_Message:
ldi r30,LOW(2*DC_msg) ; Load Z register low
ldi r31,HIGH(2*DC_msg) ; Load Z register high
ldi r28,6
rcall displayCString
ret

send_LED_Message_off:
rcall lineSwitch
ldi r30,LOW(2*LED_msg_off) ; Load Z register low
ldi r31,HIGH(2*LED_msg_off) ; Load Z register high
ldi r28,8
rcall displayCString
ret

send_LED_Message_on:
rcall lineSwitch
ldi r30,LOW(2*LED_msg_on) ; Load Z register low
ldi r31,HIGH(2*LED_msg_on) ; Load Z register high
ldi r28,8
rcall displayCString
ret

write_command:
;rcall RS_low
out   PORTC,r25
rcall LCDStrobe ;pulse
ret

write3_D7D4:
ldi r25,0x03 ;0x03
out   PORTC,r25
rcall LCDStrobe ;pulse
rcall _delay_100u
ret

;4 bit commands
write_SetInt:
ldi r25,0x28
swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u

swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u
ret

write_EnDisp:
ldi r25,0x08
swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u

swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u
ret

write_ClrHm:
ldi r25,0x01
swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u

swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u
ret

write_SetCurMvDir:
ldi r25,0x06
swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u

swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u
ret

turn_on:
ldi r25,0x0C
swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u

swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u
ret

sendChar:
; Send the character 'E' to the LCD. The ASCII
; character 'E' is 0x45
ldi r25,0x45
swap r25 ; Swap nibbles
out PORTC,r25 ; Send upper nibble
rcall LCDStrobe ; Strobe Enable line
rcall _delay_100u ; Wait


//Create static strings in program memory
; Displays a constant null-terminated string stored in program
; on the LCD.
displayCString:
;ldi r28,14
displayLoop:
lpm ; r0 <-- first byte
swap r0 ; Upper nibble in place
out PORTC,r0 ; Send upper nibble out
rcall LCDStrobe ; Latch nibble
rcall _delay_100u ; Wait
swap r0 ; Lower nibble in place
out PORTC,r0 ; Send lower nibble out
rcall LCDStrobe ; Latch nibble
rcall _delay_100u ; Wait
adiw zh:zl,1 ; Increment Z pointer
dec r28 ; Repeat until
brne displayLoop ; all characters are out
ret

lineSwitch:
cbi PORTB,5 ;rs low

ldi r25,0b11000000
swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u

swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u

sbi PORTB,5 ;rs high

ret
lineRestart:
cbi PORTB,5 ;rs low

ldi r25,0b10000000
swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u

swap r25
out   PORTC,r25
rcall LCDStrobe
rcall _delay_100u

sbi PORTB,5 ;rs high

ret

LCDStrobe:
sbi PORTB,3 ;rcall _delay_100u
nop
nop
nop
nop
nop
cbi PORTB,3
ret

timer:
.def tmp1 = r20    ; Use r20 for temporary variables
.def tmp2 = r21    ; Use r21 for temporary values
.def count = r22;.set count = 10

;ldi tmp1,0b0100
;ldi count,201
;100 us -> count=201 (C9), TCCR0B=2
;200 us -> count=51, TCCR0B=3
;5 ms   -> count=79, TCCR0B=7
;

timer_100us:
_delay_100u: ;slightly over 100micro
ldi count,0xC9 ;201
ldi  tmp2,0x02
out   TCCR0B,tmp2
rcall delay
ret

delay5Mil: ;was r25
ldi r23,0
timer_5ms_loop:
inc r23
rcall timer_100us
cpi r23,50
brlt timer_5ms_loop
ret

delay200u:
rcall timer_100us
rcall timer_100us
ret

wait0_1s://100ms
ldi looper,0
wait0_1_loop:
rcall delay5Mil
inc looper
cpi looper,20
brlt wait0_1_loop
ret


; Wait for TIMER0 to roll over.
delay:
ldi count,0x4F ;79
; Stop timer 0.
in    tmp1,TCCR0B;.set t1=TCCR0B    ; Save configuration
ldi  tmp2,0x00      ; Stop timer 0
out   TCCR0B,tmp2

; Clear overflow flag.    
in    tmp2,TIFR0      ; tmp <-- TIFR0
sbr   tmp2,1<<TOV0      ; Clear TOV0, write logic 1
out   TIFR0,tmp2

; Start timer with new initial count
out   TCNT0,count    ; Load counter
out   TCCR0B,tmp1    ; Restart timer
wait:
in    tmp2,TIFR0      ; tmp <-- TIFR0
sbrs  tmp2,TOV0      ; Check overflow flag
rjmp  wait
ret

tim0_ovf:
push r25
in r25,sreg
push r25
sbi PINC,5 ; Toggle PORTC,5
ldi r25,201 ; Reload counter
out TCNT0,r25
pop r25
out sreg,r25
pop r25
reti
