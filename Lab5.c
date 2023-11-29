// Lab5.c starter program EE319K Lab 5, Spring 2022
// Runs on TM4C123
// Saaketh Vangati, Sachin Nair
// Last Modified: 3/9/2022

/* Option A1, connect LEDs to PB5-PB0, switches to PA5-3, walk LED PF321
   Option A2, connect LEDs to PB5-PB0, switches to PA4-2, walk LED PF321
   Option A6, connect LEDs to PB5-PB0, switches to PE3-1, walk LED PF321
   Option A5, connect LEDs to PB5-PB0, switches to PE2-0, walk LED PF321
   Option B4, connect LEDs to PE5-PE0, switches to PC7-5, walk LED PF321
   Option B3, connect LEDs to PE5-PE0, switches to PC6-4, walk LED PF321
   Option B1, connect LEDs to PE5-PE0, switches to PA5-3, walk LED PF321
   Option B2, connect LEDs to PE5-PE0, switches to PA4-2, walk LED PF321
  */
// east/west red light connected to bit 5
// east/west yellow light connected to bit 4
// east/west green light connected to bit 3
// north/south red light connected to bit 2
// north/south yellow light connected to bit 1
// north/south green light connected to bit 0
// pedestrian detector connected to most significant bit (1=pedestrian present)
// north/south car detector connected to middle bit (1=car present)
// east/west car detector connected to least significant bit (1=car present)
// "walk" light connected to PF3-1 (built-in white LED)
// "don't walk" light connected to PF3-1 (built-in red LED)
#include <stdint.h>
#include "SysTick.h"
#include "Lab5grader.h"
#include "../inc/tm4c123gh6pm.h"
// put both EIDs in the next two lines
char EID1[] = "srv629"; //  ;replace abc123 with your EID
char EID2[] = "svn343"; //  ;replace abc123 with your EID

void DisableInterrupts(void);
void EnableInterrupts(void);

//SOUTH, WEST, WALK
//Wk S W ([2][1][0])
struct StateSetup
{
	uint8_t next[8];
	uint8_t EOut;
	uint8_t FOut;
	uint8_t delay;
};

typedef struct StateSetup state;

#define goSouth 0
#define slowSouth 1
#define stopSouth 2
#define goWest 3
#define slowWest 4
#define stopWest 5
#define goWalk 6
#define red1 7
#define off1 8
#define red2 9
#define off2 10
#define red3 11
#define off3 12
#define stopWalk 13

const state FSM[14] = {
	{{slowSouth, slowSouth, goSouth, slowSouth, slowSouth, slowSouth, slowSouth, slowSouth}, 		0x21, 2, 100},	//goSouth
	{{stopSouth, stopSouth, stopSouth, stopSouth, stopSouth, stopSouth, stopSouth, stopSouth}, 	0x22, 2, 30}, 	//slowSouth
	{{stopSouth, goWest, goSouth, goWest, goWalk, goWest, goWalk, goWest}, 											0x24, 2, 10}, 	//stopSouth
	{{slowWest, goWest, slowWest, slowWest, slowWest, slowWest, slowWest, slowWest}, 						0x0C, 2, 100}, //goWest
	{{stopWest, stopWest, stopWest, stopWest, stopWest, stopWest, stopWest, stopWest}, 					0x14, 2, 30}, 	//slowWest
	{{stopWest, goWest, goSouth, goSouth, goWalk, goWalk, goWalk, goWalk}, 											0x24, 2, 10}, 	//stopWest
	{{red1, red1, red1, red1, goWalk, red1, red1, red1}, 																				0x24, 14, 100}, //goWalk
	{{off1, off1, off1, off1, off1, off1, off1, off1}, 																					0x24, 2, 10}, 	//red1
	{{red2, red2, red2, red2, red2, red2, red2, red2}, 																					0x24, 0, 10}, 	//off1
	{{off2, off2, off2, off2, off2, off2, off2, off2}, 																					0x24, 2, 10}, 	//red2
	{{red3, red3, red3, red3, red3, red3, red3, red3}, 																					0x24, 0, 10}, 	//off2
	{{off3, off3, off3, off3, off3, off3, off3, off3}, 																					0x24, 2, 10}, 	//red3
	{{stopWalk, stopWalk, stopWalk, stopWalk, stopWalk, stopWalk, stopWalk, stopWalk}, 					0x24, 0, 10}, 	//off3
	{{stopWalk, goSouth, goSouth, goSouth, goWalk, goSouth, goSouth, goSouth}, 									0x24, 2, 10}, 	//stopWalk
};

#define GPIO_PORTF_OUT  (*((volatile uint32_t *)0x40025038))
int main(void){ 
  DisableInterrupts();
	SYSCTL_RCGC2_R |= 0x32;
  TExaS_Init(GRADER);
  SysTick_Init();   // Initialize SysTick for software waits
  // initialize system
	GPIO_PORTE_DEN_R |= 0x3F;
	GPIO_PORTA_DEN_R |= 0x38;
	GPIO_PORTF_DEN_R |= 0x0E;
	
	GPIO_PORTE_DIR_R |= 0x3F;
	GPIO_PORTA_DIR_R &= 0xC7;
	GPIO_PORTF_DIR_R |= 0x0E;
  EnableInterrupts(); 
	
	uint8_t currentState = goSouth;
  while(1)
	{
		GPIO_PORTE_DATA_R = FSM[currentState].EOut;
		GPIO_PORTF_DATA_R = FSM[currentState].FOut;
		SysTick_Wait10ms(FSM[currentState].delay);
		uint32_t input = GPIO_PORTA_DATA_R;
		input >>= 3;
		currentState = FSM[currentState].next[input];  
    // 1) output
    // 2) wait
    // 3) input
    // 4) next

  }
}





// SysTick.c
// Runs on TM4C123
// Put your names here
// Last Modified: 1/11/2022
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void){
    // EE319K students write this function
	NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
                                        // enable SysTick with core clock
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(uint32_t delay){
  // EE319K students write this function
	volatile uint32_t elapsedTime;
  uint32_t startTime = NVIC_ST_CURRENT_R;
  do{
    elapsedTime = (startTime-NVIC_ST_CURRENT_R)&0x00FFFFFF;
  }
  while(elapsedTime <= delay);
}
// 10000us equals 10ms
void SysTick_Wait10ms(uint32_t delay){
    // EE319K students write this function
	uint32_t i;
  for(i=0; i<delay; i++){
    SysTick_Wait(500000);  // wait 10ms (assumes 50 MHz clock)
		//SysTick_Wait(500);  // wait 10ms (assumes 50 MHz clock)
  }
}

