;****************** Lab1.s ***************
; Program initially written by: Yerraballi and Valvano
; Author: Sachin Nair
; Date Created: 1/15/2018 
; Last Modified: 2/2/2022
; Brief description of the program: Solution to Lab1
; The objective of this system is to implement a parity system
; Hardware connections: 
;  One output is positive logic, 1 turns on the LED, 0 turns off the LED
;  Three inputs are positive logic, meaning switch not pressed is 0, pressed is 1
GPIO_PORTD_DATA_R  EQU 0x400073FC
GPIO_PORTD_DIR_R   EQU 0x40007400
GPIO_PORTD_DEN_R   EQU 0x4000751C
GPIO_PORTE_DATA_R  EQU 0x400243FC
GPIO_PORTE_DIR_R   EQU 0x40024400
GPIO_PORTE_DEN_R   EQU 0x4002451C
SYSCTL_RCGCGPIO_R  EQU 0x400FE608
       PRESERVE8 
       AREA   Data, ALIGN=4
; No global variables needed

       ALIGN 4
       AREA    |.text|, CODE, READONLY, ALIGN=2
       THUMB
       EXPORT EID
EID    DCB "SVN343",0  ;replace abc123 with your EID
       EXPORT RunGrader
	   ALIGN 4
RunGrader DCD 1 ; change to nonzero when ready for grading
           
      EXPORT  Lab1
Lab1 
;Initializations
;Turn clock on for port E
	LDR R0, =SYSCTL_RCGCGPIO_R
	LDR R1, [R0]
	ORR R1, #0x10						;FLIP PORT E BIT
	STR R1, [R0]

	NOP
	NOP

;ENABLE PINS 0,1,2 ON PORT E
;SET DIRECTION BITS ON PIN 0,1,2 ON PORT E
	LDR R0, =GPIO_PORTE_DIR_R
	LDR R1, [R0]
	AND R1, #0x20						;SET PE0,1,2 TO 0 (INPUT)						
	ORR R1, #0x20						;SET PE5 TO 1 (OUTPUT)
	STR R1, [R0]

;SET DIGITAL ENABLE ON PORT E
	LDR R0, =GPIO_PORTE_DEN_R		
	LDR R1, [R0]					
	ORR R1, #0x27						;SET BITS USING OR with 0010 0111
	STR R1, [R0]					
loop
;input, calculate, output
	LDR R0, =GPIO_PORTE_DATA_R
	LDR R1, [R0]

	AND R2, R1, #0x01 					;PUT PE0 BIT INTO R2
	AND R3, R1, #0x02 					;PUT PE1 BIT INTO R3
	AND R4, R1, #0x04					;PUT PE2 BIT INTO R4
	
	
	;sSHIFT BITS TO COMPARE
	LSL R2, R2, #1 												
	LSR R4, R4, #1 						
	
	
	
	EOR R2, R2, R3						;EOR R2 AND R3 THEN STORE RESULT IN R2
	EOR R2, R2, R4						;EOR THE RESULT OF R2 AND R3 WITH R4 AND STORE RESULT IN R2 
	
	LSL R2, #4							;NEED TO SHIFT THIS OVER SO THAT WE FLIP PE5 FOR OUR OUTPUT
	STR R2, [R0] 						;STORE RESULT IN PORT E RESULT
	
	B    loop
	

    
    ALIGN        ; make sure the end of this section is aligned
    END          ; end of file
               