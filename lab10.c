// Lab8.c
// Runs on TM4C123
// Student names: change this to your names or look very silly
// Last modification date: change this to the last modification date or look very silly
// Last Modified: 1/12/2021 

// Specifications:
// Measure distance using slide pot, sample at 10 Hz
// maximum distance can be any value from 1.5 to 2cm
// minimum distance is 0 cm
// Calculate distance in fixed point, 0.001cm
// Analog Input connected to PD2=ADC0 channel 5
// displays distance on Sitronox ST7735
// PF3, PF2, PF1 are heartbeats (use them in creative ways)
// 

#include <stdint.h>

#include "ST7735.h"
#include "TExaS.h"
#include "ADC.h"
#include "Input.h"
#include "Graphics.h"
#include "Game.h"
#include "DAC.h"
#include "Timer0.h"
#include "print.h"
#include "../inc/tm4c123gh6pm.h"

//*****the first four main programs are for debugging *****
// main1 tests just the ADC and slide pot, use debugger to see data
//       used to test ADC and slide pot
// main2 adds the LCD to the ADC and slide pot, ADC data is on ST7735
//       used to measure execution time of ADC and LCD
// main3 adds your convert function, plot position on ST7735
//       used to test Nyquist Theorem
// main4 used to test Central Limit Theorem
//       observe noise versus ADC0_SAC_R

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
// Initialize Port F so PF1, PF2 and PF3 are heartbeats
void PortF_Init(void){
  volatile int delay;
  SYSCTL_RCGCGPIO_R |= 0x20;
  delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTF_DIR_R |= 0x0E;
  GPIO_PORTF_DEN_R |= 0x0E;
}

typedef enum
{
	ENGLISH, SPANISH
} language;

const language theLanguage = ENGLISH;

char *THINKING_WORD[] = {"Thinking.", "Pensamiento.."};
char *YOUR_MOVE_WORD[] = {"Your Move", "Su Movimiento"};
char *GAME_OVER_WORD[] = {"Game Over", "Juego Terminado"};

int usersTurn = 1;
board b;
playerColor userColor = BLACK_PLAYER;

int lockInput = 0;
uint32_t buttonA = 0;
uint32_t buttonB = 0;

void SysTick_Init(uint32_t period)
{
	NVIC_ST_CTRL_R = 0;
	NVIC_ST_RELOAD_R = period - 1;
	NVIC_ST_CURRENT_R = 0;
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x20000000;
	NVIC_ST_CTRL_R = 0x0007; 
}

int gameOverFlag = 0;

void SysTick_Handler(void)
{
	PF2 ^= 0x04;
	
	uint32_t gridX = getX();
	uint32_t gridY = getY();
	
	if(!hasMove(&b, userColor))
	{
		gameOverFlag = 1;
	}
	
	drawBoard(&b, getScore(&b, userColor), getScore(&b, WHITE_PLAYER - userColor), gridX, gridY, usersTurn);
	
	if(isGameOver(&b))
	{
		ST7735_DrawString(0, 15, GAME_OVER_WORD[theLanguage], 2016);
		return;
	}
	
	if(!usersTurn)
	{
		ST7735_DrawString(0, 15, THINKING_WORD[theLanguage], 2016);
		return;
	}
	else
	{
		ST7735_DrawString(0, 15, YOUR_MOVE_WORD[theLanguage], 2016);
	}
	
	//TEMP
	/*
	uint64_t aiMoves = findMoves(b.pieces[BLACK_PLAYER], b.pieces[WHITE_PLAYER]);
	uint32_t i = 0;
	for(i = 0; i < 64; ++i)
	{
		if(aiMoves & (1ULL << i))
		{
			doMove(&b, BLACK_PLAYER, i % 8, i / 8);
			break;
		}
	}
	usersTurn = 0;
	return;
	*/
	//TEMP
	
	lockInput = 1;
	
	if(usersTurn)
	{
		if(buttonA)
		{
			if(isValidMove(&b, userColor, gridX, gridY))
			{
				doMove(&b, userColor, gridX, gridY);
				usersTurn = 0;
			}
		}
	}
	
	lockInput = 0;
}

int main(void)
{
	DisableInterrupts();
	TExaS_Init(SCOPE);
	ST7735_InitR(INITR_REDTAB);
	newBoard(&b);
	SysTick_Init(8000000);
	PortF_Init();
	initializeInput();
	DAC_Init();
	Timer0_Init(10000, 5);
	while(1)
	{
		buttonB = getB();
		setMuted(buttonB);
		if(!lockInput)
		{
			buttonA = getA();
			
		}
		if(!usersTurn)
		{
			setMuted(getMuted() || 1);
			doAITurn(&b, WHITE_PLAYER - userColor);
			usersTurn = 1;
			setMuted(0);
		}
	};
}





#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "ADC.h"

void initializeInput(void)
{
	ADC_Init();
	GPIO_PORTE_DEN_R |= 0x3;
	GPIO_PORTE_DIR_R &= 0x3FC;
}

static uint32_t convert(uint32_t val){
	uint32_t raw = ((1618 * val / 4096) - 47) / 188;
	return raw;
}

uint32_t getX(void)
{
	return convert(ADCX_In());
}

uint32_t getY(void)
{
	return convert(ADCY_In());
}

uint32_t getA(void)
{
	uint32_t data = GPIO_PORTE_DATA_R;
	data &= 0x2;
	data >>= 1;
	return data;
}

uint32_t getB(void)
{
	uint32_t data = GPIO_PORTE_DATA_R;
	data &= 0x1;
	return data;
}


// Timer0.c
// Runs on LM4F120/TM4C123
// Use TIMER0 in 32-bit periodic mode to request interrupts at a periodic rate
// Daniel Valvano
// Last Modified: 11/15/2021  
// You can use this timer only if you learn how it works

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
  Program 7.5, example 7.6

 Copyright 2021 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
#include <stdint.h>

#include "DAC.h"
#include "../inc/tm4c123gh6pm.h"

void DisableInterrupts(void);
void EnableInterrupts(void);


// ***************** Timer0_Init ****************
// Activate TIMER0 interrupts to run user task periodically
// Inputs:  period in units (1/clockfreq)
//          priority is 0 (high) to 7 (low)
// Outputs: none
void Timer0_Init(uint32_t period, uint32_t priority){
  volatile int delay;
  SYSCTL_RCGCTIMER_R |= 0x01;   // 0) activate TIMER0
  delay = SYSCTL_RCGCTIMER_R;
  TIMER0_CTL_R = 0x00000000;    // 1) disable TIMER0A during setup
  TIMER0_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER0_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER0_TAILR_R = period-1;    // 4) reload value
  TIMER0_TAPR_R = 0;            // 5) bus clock resolution
  TIMER0_ICR_R = 0x00000001;    // 6) clear TIMER0A timeout flag
  TIMER0_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|(priority<<29); // priority is bits 31,30,29
// interrupts enabled in the main program after all devices initialized
// vector number 35, interrupt number 19
  NVIC_EN0_R = 1<<19;           // 9) enable IRQ 19 in NVIC
  TIMER0_CTL_R = 0x00000001;    // 10) enable TIMER0A
}

const uint32_t notes[16]= {0, 1, 7, 8, 15, 16, 17, 18, 31, 32, 33, 47, 48, 49, 62, 63};

uint32_t isMuted = 0;
void setMuted(uint32_t val)
{
	isMuted = val;
}

uint32_t getMuted(void)
{
	return isMuted;
}

void Timer0A_Handler(void){
	DisableInterrupts();
	TIMER0_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER0A timeout
	static int pos = 0;
	if(!isMuted)
	{
		DAC_Out(notes[pos]);
		pos = (pos + 1) % 16;
	}
	else
	{
		DAC_Out(0);
	}
	EnableInterrupts();
}

// Timer0.h
// Runs on LM4F120/TM4C123
// Use Timer0 in 32-bit periodic mode to request interrupts at a periodic rate
// Daniel Valvano
// 11/15/2021 

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
  Program 7.5, example 7.6

 Copyright 2021 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#ifndef __TIMER0INTS_H__ // do not include more than once
#define __TIMER0INTS_H__
#include <stdint.h>
// ***************** Timer0_Init ****************
// Activate TIMER0 interrupts to run user task periodically
// Inputs:  period in units (1/clockfreq)
//          priority is 0 (high) to 7 (low)
// Outputs: none
void Timer0_Init(uint32_t period, uint32_t priority);

void setMuted(uint32_t val);

uint32_t getMuted(void);


#endif // __TIMER2INTS_H__



; LCD.s
; Student names: change this to your names or look very silly
; Last modification date: change this to the last modification date or look very silly

; Runs on TM4C123
; Use SSI0 to send an 8-bit code to the ST7735 160x128 pixel LCD.

; As part of Lab 7, students need to implement these writecommand and writedata
; This driver assumes two low-level LCD functions

; Backlight (pin 10) connected to +3.3 V
; MISO (pin 9) unconnected
; SCK (pin 8) connected to PA2 (SSI0Clk)
; MOSI (pin 7) connected to PA5 (SSI0Tx)
; TFT_CS (pin 6) connected to PA3 (SSI0Fss)
; CARD_CS (pin 5) unconnected
; Data/Command (pin 4) connected to PA6 (GPIO)
; RESET (pin 3) connected to PA7 (GPIO)
; VCC (pin 2) connected to +3.3 V
; Gnd (pin 1) connected to ground

GPIO_PORTA_DATA_R       EQU   0x400043FC
SSI0_DR_R               EQU   0x40008008
SSI0_SR_R               EQU   0x4000800C

      EXPORT   writecommand
      EXPORT   writedata

      AREA    |.text|, CODE, READONLY, ALIGN=2
      THUMB
      ALIGN

; The Data/Command pin must be valid when the eighth bit is
; sent.  The SSI module has hardware input and output FIFOs
; that are 8 locations deep.  Based on the observation that
; the LCD interface tends to send a few commands and then a
; lot of data, the FIFOs are not used when writing
; commands, and they are used when writing data.  This
; ensures that the Data/Command pin status matches the byte
; that is actually being transmitted.
; The write command operation waits until all data has been
; sent, configures the Data/Command pin for commands, sends
; the command, and then waits for the transmission to
; finish.
; The write data operation waits until there is room in the
; transmit FIFO, configures the Data/Command pin for data,
; and then adds the data to the transmit FIFO.
; NOTE: These functions will crash or stall indefinitely if
; the SSI0 module is not initialized and enabled.

; This is a helper function that sends an 8-bit command to the LCD.
; Input: R0  8-bit command to transmit
; Output: none
; Assumes: SSI0 and port A have already been initialized and enabled
writecommand
;; --UUU-- Code to write a command to the LCD
;1) Read SSI0_SR_R and check bit 4, 
;2) If bit 4 is high, loop back to step 1 (wait for BUSY bit to be low)
;3) Clear D/C=PA6 to zero
;4) Write the command to SSI0_DR_R
;5) Read SSI0_SR_R and check bit 4, 
;6) If bit 4 is high, loop back to step 5 (wait for BUSY bit to be low)	
    LDR R1,=SSI0_SR_R
	
writecommand_loop1
	LDR R2,[R1]
	LSR R2,R2,#0x4
	AND R2,R2,#0x1
	ADDS R2,R2,#0
	BNE writecommand_loop1
	
	LDR R1,=GPIO_PORTA_DATA_R
	LDR R2,[R1]
	BIC R2,R2,#0x40
	STR R2,[R1]
	LDR R1,=SSI0_DR_R
	STRB R0,[R1]
	
	LDR R1,=SSI0_SR_R
	
writecommand_loop2
	LDR R2,[R1]
	LSR R2,R2,#0x4
	AND R2,R2,#0x1
	ADDS R2,R2,#0
	BNE writecommand_loop2
	
    BX  LR                          ;   return

; This is a helper function that sends an 8-bit data to the LCD.
; Input: R0  8-bit data to transmit
; Output: none
; Assumes: SSI0 and port A have already been initialized and enabled
writedata
;; --UUU-- Code to write data to the LCD
;1) Read SSI0_SR_R and check bit 1, 
;2) If bit 1 is low loop back to step 1 (wait for TNF bit to be high)
;3) Set D/C=PA6 to one
;4) Write the 8-bit data to SSI0_DR_R
	LDR R1,=SSI0_SR_R
	
writedata_loop1
	LDR R2,[R1]
	LSR R2,R2,#1
	AND R2,R2,#1
	ADDS R2,R2,#0
	BEQ writedata_loop1

	LDR R1,=GPIO_PORTA_DATA_R
	LDR R2,[R1]
	ORR R2,R2,#0x40
	STR R2,[R1]

	LDR R1,=SSI0_DR_R
	STRB R0,[R1]
	
    BX  LR                          ;   return


;***************************************************
; This is a library for the Adafruit 1.8" SPI display.
; This library works with the Adafruit 1.8" TFT Breakout w/SD card
; ----> http://www.adafruit.com/products/358
; as well as Adafruit raw 1.8" TFT display
; ----> http://www.adafruit.com/products/618
;
; Check out the links above for our tutorials and wiring diagrams
; These displays use SPI to communicate, 4 or 5 pins are required to
; interface (RST is optional)
; Adafruit invests time and resources providing this open source code,
; please support Adafruit and open-source hardware by purchasing
; products from Adafruit!
;
; Written by Limor Fried/Ladyada for Adafruit Industries.
; MIT license, all text above must be included in any redistribution
;****************************************************

    ALIGN                           ; make sure the end of this section is aligned
    END                             ; end of file


#include <stdint.h>

#include "Game.h"

#define PF1       (*((volatile uint32_t *)0x40025008)

uint32_t countPieces(uint64_t uncountedPieces)
{
	uint32_t count = 0;
	while(uncountedPieces)
	{
		uncountedPieces &= uncountedPieces - 1;
		++count;
	}
	return count;
}

pieceType getCell(board *b, uint32_t gridX, uint32_t gridY)
{
	uint64_t locMask = 1ULL << (gridX + gridY * 8);
	
	if(b->pieces[BLACK_PLAYER] & locMask)
	{
		return BLACK;
	}
	else if(b->pieces[WHITE_PLAYER] & locMask)
	{
		return WHITE;
	}
	return BLANK;
}

void setCell(board *b, uint32_t gridX, uint32_t gridY, pieceType p)
{
	uint64_t locMask = 1ULL << (gridX + gridY * 8);
	
	b->pieces[BLACK_PLAYER] &= ~locMask;
	b->pieces[WHITE_PLAYER] &= ~locMask;
	if(p == BLACK)
	{
		b->pieces[BLACK_PLAYER] |= locMask;
	}else if(p == WHITE)
	{
		b->pieces[WHITE_PLAYER] |= locMask;
	}
}

void newBoard(board *b)
{
	b->pieces[BLACK_PLAYER] = 0;
	b->pieces[WHITE_PLAYER] = 0;
	setCell(b, 4, 3, BLACK);
	setCell(b, 3, 4, BLACK);
	setCell(b, 3, 3, WHITE);
	setCell(b, 4, 4, WHITE);
}

uint32_t getScore(board *b, playerColor p)
{
	return countPieces(b->pieces[p]);
}

uint64_t moveBoard(uint64_t pieces, uint32_t direction)
{
	const uint64_t locMasks[] = {
		0x7F7F7F7F7F7F7F7FULL,
		0x007F7F7F7F7F7F7FULL,
		0xFFFFFFFFFFFFFFFFULL,
		0x00FEFEFEFEFEFEFEULL,
		0xFEFEFEFEFEFEFEFEULL,
		0xFEFEFEFEFEFEFE00ULL,
		0xFFFFFFFFFFFFFFFFULL,
		0x7F7F7F7F7F7F7F00ULL 
	};
	const uint64_t leftShifts[] = {0, 0, 0, 0, 1, 9, 8, 7};
	const uint64_t rightShifts[] = {1, 9, 8, 7, 0, 0, 0, 0};
	if(direction < 4)
	{
		return (pieces >> rightShifts[direction]) & locMasks[direction];
	}
	else
	{
		return (pieces << leftShifts[direction]) & locMasks[direction];
	}
}

uint64_t findMoves(uint64_t activePieces, uint64_t otherPieces)
{
	uint64_t temp, validMoves = 0, emptyPieces = ~(activePieces | otherPieces);
	
	uint32_t direction;
	for(direction = 0; direction < 8; ++direction)
	{
		temp = moveBoard(activePieces, direction) & otherPieces;
		temp |= moveBoard(temp, direction) & otherPieces;
		temp |= moveBoard(temp, direction) & otherPieces;
		temp |= moveBoard(temp, direction) & otherPieces;
		temp |= moveBoard(temp, direction) & otherPieces;
		temp |= moveBoard(temp, direction) & otherPieces;
		validMoves |= moveBoard(temp, direction) & emptyPieces;
	}
	return validMoves;
}

int hasMove(board *b, playerColor activePlayer)
{
	return findMoves(b->pieces[activePlayer], b->pieces[WHITE_PLAYER - activePlayer]) != 0;
}

int isValidMove(board *b, playerColor activePlayer, uint32_t gridX, uint32_t gridY)
{
	uint64_t locMask = 1ULL << (gridX + gridY * 8);
	return (findMoves(b->pieces[activePlayer], b->pieces[WHITE_PLAYER - activePlayer]) & locMask) != 0;
}

void doMove(board *b, playerColor activePlayer, uint32_t gridX, uint32_t gridY)
{
	uint64_t temp, surroundingPieces, newPiece = 1ULL << (gridX + gridY * 8), siezedPieces = 0;
	
	b->pieces[activePlayer] |= newPiece;
	
	uint32_t direction = 0;
	for(direction = 0; direction < 8; ++direction)
	{
		temp = moveBoard(newPiece, direction) & b->pieces[WHITE_PLAYER - activePlayer];
		temp |= moveBoard(temp, direction) & b->pieces[WHITE_PLAYER - activePlayer];
		temp |= moveBoard(temp, direction) & b->pieces[WHITE_PLAYER - activePlayer];
		temp |= moveBoard(temp, direction) & b->pieces[WHITE_PLAYER - activePlayer];
		temp |= moveBoard(temp, direction) & b->pieces[WHITE_PLAYER - activePlayer];
		temp |= moveBoard(temp, direction) & b->pieces[WHITE_PLAYER - activePlayer];
		surroundingPieces = moveBoard(temp, direction) & b->pieces[activePlayer];
		if(surroundingPieces)
		{
			siezedPieces |= temp;
		}
	}
	b->pieces[activePlayer] ^= siezedPieces;
	b->pieces[WHITE_PLAYER - activePlayer] ^= siezedPieces;
}

int32_t evaluateBoard(board *b, playerColor activePlayer)
{
	uint64_t bonus = 10000000;
	uint64_t corners = 0x8100000000000081ULL;
	uint64_t activePlayerMoves = findMoves(b->pieces[activePlayer], b->pieces[WHITE_PLAYER - activePlayer]);
	uint64_t otherPlayerMoves = findMoves(b->pieces[activePlayer], b->pieces[WHITE_PLAYER - activePlayer]);
	uint32_t activePlayerPieces, otherPlayerPieces, activePlayerCorners, otherPlayerCorners;
	int32_t evaluation = 0;
	if(activePlayerMoves == 0 && otherPlayerMoves == 0)
	{
		activePlayerPieces = countPieces(b->pieces[activePlayer]);
		otherPlayerPieces = countPieces(b->pieces[WHITE_PLAYER - activePlayer]);
		return (activePlayerPieces - otherPlayerPieces) * bonus;
	}
	activePlayerCorners = activePlayerPieces & corners;
	otherPlayerCorners = otherPlayerPieces & corners;
	evaluation += (countPieces(activePlayerCorners) - countPieces(otherPlayerCorners)) * 16;
	evaluation += (countPieces(activePlayerCorners) - countPieces(otherPlayerCorners)) * 2;
	evaluation -= (countPieces(activePlayerCorners) - countPieces(otherPlayerCorners));
	
	return evaluation;
}

int32_t negamax(board *b, playerColor activePlayer, uint32_t depth, int32_t alpha, int32_t beta, uint64_t *numEvaluations, uint64_t *bestMove)
{
	for(int i = 0; i < 5000000; ++i) {}
	#define light       (*((volatile uint32_t *)0x40025010))
	light ^= 4;
	uint64_t activePlayerMoves, otherPlayerMoves, activePlayerNewPieces, otherPlayerNewPieces;
	int32_t temp, best;
	
	activePlayerMoves = findMoves(b->pieces[activePlayer], b->pieces[WHITE_PLAYER - activePlayer]);
	otherPlayerMoves = findMoves(b->pieces[WHITE_PLAYER - activePlayer], b->pieces[activePlayer]);
	
	if(activePlayerMoves == 0 && otherPlayerMoves != 0)
	{
		return -negamax(b, WHITE_PLAYER - activePlayer, depth, -beta, -alpha, numEvaluations, bestMove);
	}
	
	if(depth == 0 || (activePlayerMoves == 0 && otherPlayerMoves == 0))
	{
		++*numEvaluations;
		return evaluateBoard(b, activePlayer);
	}
	
	best = -100000;
	int32_t pos = 0;
	for(pos = 0; pos < 64; ++pos)
	{
		if(!(activePlayerMoves & (1ULL << pos)))
		{
			continue;
		}
		
		activePlayerNewPieces = b->pieces[activePlayer];
		otherPlayerNewPieces = b->pieces[WHITE_PLAYER - activePlayer];
		board b2;
		if(activePlayer == WHITE_PLAYER)
		{
			b2.pieces[WHITE_PLAYER] = activePlayerNewPieces;
			b2.pieces[BLACK_PLAYER] = otherPlayerNewPieces;
		}
		else
		{
			b2.pieces[BLACK_PLAYER] = activePlayerNewPieces;
			b2.pieces[WHITE_PLAYER] = otherPlayerNewPieces;
		}
		doMove(&b2, activePlayer, pos % 8, pos / 8);
		
		temp = -negamax(&b2, WHITE_PLAYER - activePlayer, depth - 1, -beta, -alpha, numEvaluations, 0);
		
		if(temp > best)
		{
			best = temp;
			if(bestMove != 0)
			{
				*bestMove = pos;
			}
			
			if(temp > alpha)
			{
				alpha = temp;
			}
			
			if(alpha >= beta)
			{
				break;
			}
		}
	}
	
	return best;
}

int64_t gradualNegamax(board *b, playerColor activePlayer)
{
	const uint32_t maxDepth = 2;
	uint32_t depth;
	uint64_t numEvaluations = 0;
	uint64_t bestMove;
	for(depth = 0; depth < maxDepth; ++depth)
	{
		negamax(b, activePlayer, depth, -1000000, 1000000, &numEvaluations, &bestMove);
		if(numEvaluations > 6)
		{
			break;
		}
	}
	return bestMove;
}

void doAITurn(board *b, playerColor aiColor)
{
	uint64_t aiMove = gradualNegamax(b, aiColor);
	doMove(b, aiColor, aiMove % 8, aiMove / 8);
	return;
	uint64_t aiMoves = findMoves(b->pieces[aiColor], b->pieces[WHITE_PLAYER - aiColor]);
	uint32_t i = 0;
	for(i = 0; i < 64; ++i)
	{
		if(aiMoves & (1ULL << i))
		{
			doMove(b, aiColor, i % 8, i / 8);
			return;
		}
	}
}

int isGameOver(board *b)
{
	return !hasMove(b, WHITE_PLAYER) && !hasMove(b, BLACK_PLAYER);
}

