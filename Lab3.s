;****************** Lab3.s ***************
; Program written by: Saaketh Vangati, Sachin Nair
; Date Created: 2/4/2017
; Last Modified: 1/10/2022
; Brief description of the program
;   The LED toggles at 2 Hz and a varying duty-cycle
; Hardware connections (External: Two buttons and one LED)
;  Change is Button input  (1 means pressed, 0 means not pressed)
;  Breathe is Button input  (1 means pressed, 0 means not pressed)
;  LED is an output (1 activates external LED)
; Overall functionality of this system is to operate like this
;   1) Make LED an output and make Change and Breathe inputs.
;   2) The system starts with the the LED toggling at 2Hz,
;      which is 2 times per second with a duty-cycle of 30%.
;      Therefore, the LED is ON for 150ms and off for 350 ms.
;   3) When the Change button is pressed-and-released increase
;      the duty cycle by 20% (modulo 100%). Therefore for each
;      press-and-release the duty cycle changes from 30% to 70% to 70%
;      to 90% to 10% to 30% so on
;   4) Implement a "breathing LED" when Breathe Switch is pressed:
; PortE device registers
GPIO_PORTE_DATA_R  EQU 0x400243FC
GPIO_PORTE_DIR_R   EQU 0x40024400
GPIO_PORTE_DEN_R   EQU 0x4002451C
SYSCTL_RCGCGPIO_R  EQU 0x400FE608

        IMPORT  TExaS_Init
        THUMB
        AREA    DATA, ALIGN=2
;global variables go here

       AREA    |.text|, CODE, READONLY, ALIGN=2
       THUMB
       EXPORT EID1
EID1   DCB "srv629",0  ;replace ABC123 with your EID
       EXPORT EID2
EID2   DCB "svn343",0  ;replace ABC123 with your EID
       ALIGN 4

     EXPORT  Start

Start
; TExaS_Init sets bus clock at 80 MHz, interrupts, ADC1, TIMER3, TIMER5, and UART0
	MOV R0,#2  ;0 for TExaS oscilloscope, 1 for PORTE logic analyzer, 2 for Lab3 grader, 3 for none
	BL  TExaS_Init ;enables interrupts, prints the pin selections based on EID1 EID2
; Your Initialization goes here
	LDR R0,=SYSCTL_RCGCGPIO_R	;R0 points to SYSCTL_RCGCGPIO_R
	LDR R1,[R0]					;Read SYSCTL_RCGCGPIO_R into R1
	ORR R1,#0x10				;Turn on clock Bin: 00010000
	STR R1,[R0]					;Write back to SYSCTL_RCGCGPIO_R
	NOP
	NOP

	LDR R0,=GPIO_PORTE_DIR_R
	MOV R1,#0x10				;00010000
	STR R1,[R0]
	
	LDR	R0,=GPIO_PORTE_DEN_R
	MOV R1,#0x15				;Enable PE4, PE2, PE0
	STR R1,[R0]
	
	MOV R3,#0x86A0				;R3 = 100000 (multiply by duty cycle percentage)
	MOVT R3,#0x1
	
	MOV R4,#30					;R4 = duty cycle (default 30)
	
	MOV R6,#0					;Toggle (keeps track of is button is released)
	
	MOV R7,#20
	
loop
; main engine goes here
	LDR R0,=GPIO_PORTE_DATA_R
	
	LDR R1,[R0]
	ANDS R1,#1
	BNE SkipChange
	ADDS R6,#0
	BEQ SkipChange
	BL IncrementDutyCycle
SkipChange
	LDR R6,[R0]
	AND R6,#1
	
	LDR R1,[R0]
	ANDS R1,#4
	BEQ SkipBreathe
	BL IncrementDutyCycleBreathe
SkipBreathe

	BL LEDOn
	MUL R2,R3,R4
	BL Delay
	BL LEDOff
	RSB R5,R4,#100
	MUL R2,R3,R5
	BL Delay
	
	B loop
	
Delay
	SUBS R2,#1
	BNE Delay
	BX LR
	
LEDOn
	MOV R1,#0x10
	STR R1,[R0]
	BX LR
	
LEDOff
	MOV R1,#0
	STR R1,[R0]
	BX LR
	
IncrementDutyCycle
	ADD R4,#20
	MOV R5,#110
	CMP R4,R5
	BNE Leave1
	SUB R4,#100
Leave1
	BX LR
	
IncrementDutyCycleBreathe
	ADD R4,R7
	MOV R5,#90
	CMP R4,R5
	BEQ Leave2
	MOV R5,#10
	CMP R4,R5
	BEQ Leave3
	B Leave4
Leave2
	SUB R7,#40
	B Leave4
Leave3
	ADD R7,#40
Leave4
	BX LR
	
	ALIGN      ; make sure the end of this section is aligned
	END        ; end of file

