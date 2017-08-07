//
// This file is based on a file that is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
// Highly modified by Denry for cardatacollector use
//

// ----------------------------------------------------------------------------

//stm32f103c8t6

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "main.h"
//#include "BlinkLed.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_tim.h"


//#define EMB_BL_SP_ADDR 			(0x1FFFF000)	// Embedded bootloader address
//#define EMB_BL_ADDR_OFFSET		(0x00000004)	// Embedded bootloader reset vector offset (relative to address)

//required to put quotation marks around defines
//#define DEF2STR(x) #x
//#define STR(x) DEF2STR(x)


// ----------------------------------------------------------------------------
//
// Standalone STM32F1 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8 MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f10x.c
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
//#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 1 / 10)
//#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"



//#define FAKE_RESULTS //for debugging purposes, returns some random values
//#define USBUART_EVENPARITY //if commented out, even parity disabled (no parity used)

USART_TypeDef *USB_UART = USART1; //see comment on next line.
//USART_TypeDef *TIM_TACHO = TIM2; //unused cause RCC periph clock enable crap. Actually USART has that too. Would be neat to redefine that too?
//USART_TypeDef *TIM_VSS = TIM3;

carData_datastruct carData_latest; // most up-to-date info
carData_datastruct carData_previousPrinted; // to verify if anything has changed since last print.

bool init_uart_usb(){
	// this example seems to be useful:
	// http://pandafruits.com/stm32_primer/stm32_primer_uart.php

	USART_InitTypeDef USB_UART_Init;

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO |
	                           RCC_APB2Periph_GPIOA, ENABLE);

	//init UART clock
	USART_ClockInitTypeDef USB_UART_Clock;

	//USB_UART_Clock.USART_Clock = USART_Clock_Enable;
	USB_UART_Clock.USART_Clock = USART_Clock_Disable;
	USB_UART_Clock.USART_CPOL = USART_CPOL_Low;
	USB_UART_Clock.USART_CPHA = USART_CPHA_1Edge;
	USB_UART_Clock.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USB_UART, &USB_UART_Clock);

	USART_Cmd(USB_UART, ENABLE);

	//__GPIOA_CLK_ENABLE();
	//	__GPIOB_CLK_ENABLE();
	//	__USART1_CLK_ENABLE();

		//set up UART
//		USB_UART.Instance = USART1;
//		USB_UART.Init.BaudRate = 115200;
//		USB_UART.Init.WordLength = UART_WORDLENGTH_8B;
//		USB_UART.Init.StopBits = UART_STOPBITS_1;
//		USB_UART.Init.Parity = UART_PARITY_NONE;
//		USB_UART.Init.Mode = UART_MODE_TX_RX;
//		USB_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;

		USB_UART_Init.USART_BaudRate = 115200;

#ifdef USBUART_EVENPARITY
		USB_UART_Init.USART_WordLength = USART_WordLength_9b;
		USB_UART_Init.USART_Parity = USART_Parity_Even; // to be compatible with bootloader
#else
		USB_UART_Init.USART_WordLength = USART_WordLength_8b;
		USB_UART_Init.USART_Parity = USART_Parity_No;
#endif

		USB_UART_Init.USART_StopBits = USART_StopBits_1;
		USB_UART_Init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USB_UART_Init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USB_UART, &USB_UART_Init);

//		if(HAL_UART_Init(&USB_UART) != HAL_OK)
//		{
//			//Error_Handler();
//			while (1);
//		}

		//set up GPIOs to UART, PA10 as Rx


		GPIO_InitTypeDef GPIO_InitStructure;
		//GPIO_InitStructure.Pin = (1<<7);
//		GPIO_InitStructure.Pin = (GPIO_PIN_7);
//		GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
//		GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
//		GPIO_InitStructure.Pull = GPIO_PULLUP;
//		GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
//		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_10);
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		//GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE); //NB, this sets UART to PB6/PB7, not simply turn gpio to UART
		GPIO_PinRemapConfig(GPIO_Remap_USART1, DISABLE);
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		//and PA9 as Tx
		GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_9);
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		//GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE); //NB, this sets UART to PB6/PB7, not simply turn gpio to UART
		GPIO_PinRemapConfig(GPIO_Remap_USART1, DISABLE);
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		//and PA9 as Tx
//		GPIO_InitTypeDef GPIO_InitStructure2;
//		//GPIO_InitStructure.Pin = (1<<9);
//		GPIO_InitStructure2.Pin = (GPIO_PIN_9);
//		GPIO_InitStructure2.Mode = GPIO_MODE_AF_PP;
//		GPIO_InitStructure2.Speed = GPIO_SPEED_HIGH;
//		GPIO_InitStructure2.Pull = GPIO_PULLUP;
//		GPIO_InitStructure2.Alternate = GPIO_AF7_USART1;
//		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure2);
	return 0; //success
}

bool UART_sendstring(USART_TypeDef *UARTx, char *string){
	int i = 0;

	while(string[i] != 0){
		while(!(UARTx->SR & USART_SR_TXE));
		USART_SendData(UARTx, (uint16_t)(string[i]));
		i++;
	}
	return 0;
}

bool UART_sendint(USART_TypeDef *UARTx, uint32_t value){
	char string[15]; //allowing for 32bits nicely
	itoa(value, string, 10);
	UART_sendstring(UARTx, string);
	return 0;
}


void checkForUpdateRequest(){
	UART_sendstring(USB_UART, "\r\nPress b to enter emb. bootloader... ");
	const uint32_t timeToEnter = 1000;
	char lastUARTchar;
	uint32_t timestamp = HAL_GetTick();

	while((HAL_GetTick() - timestamp) <= timeToEnter){
		if(USB_UART->SR & USART_SR_RXNE){ //has data
			lastUARTchar = USB_UART->DR;
			if(lastUARTchar == 'b'){ //check if right character
				//enter embedded bootloader!

				//SystemCoreClock = HSI_VALUE;
				//SystemCoreClockUpdate();

				// if still no work, mby get more tips here https://stm32f4-discovery.net/2017/04/tutorial-jump-system-memory-software-stm32/
				// turns out that tutorial is useless for me, it required boot bits (not available for stm32f1)
				// Much better were instructions where they mention doing it with backup registers and reset

				UART_sendstring(USB_UART, "Entering in 3 sec ");
				timer_sleep(1000);
				UART_sendstring(USB_UART, ".");
				timer_sleep(1000);
				UART_sendstring(USB_UART, ".");
				timer_sleep(1000);
				UART_sendstring(USB_UART, ".\r\n");

				//start of alternative try
				/*void (*SysMemBootJump)(void);
				volatile uint32_t addr = EMB_BL_SP_ADDR;
				//	Step: Disable RCC, set it to default (after reset) settings
				//	Internal clock, no PLL, etc.
				RCC_DeInit();
				SysTick->CTRL = 0;
				SysTick->LOAD = 0;
				SysTick->VAL = 0;
				__disable_irq();
				//memory remap should be done here, not option for stm32f1?
				SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));
				__set_MSP(*(uint32_t *)addr);
				SysMemBootJump();*/

				// ok trying to enter @ boot
				RCC->APB1ENR |= RCC_APB1ENR_PWREN; //enable clock to power interface
				RCC->APB1ENR |= RCC_APB1ENR_BKPEN; //enable clock to BKP
				PWR->CR |= PWR_CR_DBP; //allow writing to RTC & backup registers
				BKP->DR1 = 0x0BAD;
				BKP->DR2 = 0xC0DE;
				NVIC_SystemReset();
				PWR->CR &= ~((uint32_t)(PWR_CR_DBP)); //forbid writing to RTC & backup registers

				//end of alternative try

				//		step: Disable RCC, set it to default (after reset) settings
				/* //Internal clock, no PLL, etc.
						RCC_DeInit();

				//unlock flash
				FLASH_Status flashHaxResult;
				FLASH_Unlock();
				//if(FLASH_GetReadOutProtectionStatus()){
				flashHaxResult = FLASH_ReadOutProtection(DISABLE);
				//}


				FLASH_ClearFlag(FLASH_FLAG_EOP);
				FLASH->CR &= ~(0x00000200); //clear OPTWRE
				FLASH->AR = 0; //set AR to 0
				// FLASH_SetLatency(FLASH_Latency_0); //this stops comm with flashloader, prolly cause mcu runs at 72
				FLASH_Lock();


				//#define EMB_BL_SP_ADDR 			(0x1FFFF000)	// Embedded bootloader address
				//#define EMB_BL_ADDR_OFFSET		(0x00000004)	// Embedded bootloader reset vector offset (relative to address)
				//required to put quotation marks around defines
				//#define DEF2STR(x) #x
				//#define STR(x) DEF2STR(x)

				//perform the jump
				//stack pointer initial value to stack pointer
				asm volatile("ldr r0, =" STR(EMB_BL_SP_ADDR));
				asm volatile("ldr r0, [r0]");  //not sure why, but this line is very necessary
				asm volatile("mov sp, r0");

				//reset vector address to program counter
				asm volatile("ldr r0, =" STR(EMB_BL_SP_ADDR + EMB_BL_ADDR_OFFSET));
				asm volatile("ldr r0, [r0]"); //not sure why, but this line is very necessary
				asm volatile("mov pc, r0");*/
			}
		}
	}

	UART_sendstring(USB_UART, "\r\nLoading app.\r\n");
	return;
}

bool init_pc13_led(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_13);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	return 0;
}



void eternalUSBUART_loopback(){
	uint32_t timestamp = HAL_GetTick();

	const uint32_t notifydelay = 1000;

	bool lastNotifyState = 0;

	//uint16_t uartchar16;
	//char uartchar;

	UART_sendstring(USB_UART, "Loopback\r\n");

//	USART_SendData(USB_UART, 'L');
//	while(!(USB_UART->SR & USART_SR_TXE));
//	USART_SendData(USB_UART, 'B');
//	while(!(USB_UART->SR & USART_SR_TXE));
//	USART_SendData(USB_UART, '\r');
//	while(!(USB_UART->SR & USART_SR_TXE));
//	USART_SendData(USB_UART, '\n');
//	while(!(USB_UART->SR & USART_SR_TXE));

//	USB_UART->DR = 'L';
//	while(!(USB_UART->SR & USART_SR_TXE));
//	USB_UART->DR = 'B';
//	while(!(USB_UART->SR & USART_SR_TXE));
//	USB_UART->DR = '\r';
//	while(!(USB_UART->SR & USART_SR_TXE));
//	USB_UART->DR = '\n';
//	while(!(USB_UART->SR & USART_SR_TXE));
	while(1){



		if((HAL_GetTick() - timestamp) >= notifydelay){
			// called every 1000 ms
			if (lastNotifyState){
				GPIO_WriteBit(GPIOC, GPIO_Pin_13, lastNotifyState);
				lastNotifyState = 0;
			} else {
				GPIO_WriteBit(GPIOC, GPIO_Pin_13, lastNotifyState);
				lastNotifyState = 1;
			}
			timestamp += notifydelay;

			//UART_sendint(USB_UART, HAL_GetTick());
			//UART_sendstring(USB_UART, "\r\n");
		}

		if(USB_UART->SR & USART_SR_RXNE){ //has data
			//uartchar16 = USB_UART->DR;
			//uartchar = uartchar16 & 0xFF;
			//USART_SendData(USB_UART, uartchar16); //send it back
			USART_SendData(USB_UART, USB_UART->DR); //send it back
		}
	}

}

bool init_uart_gps(){

	return 0;
}

bool init_uart_extension(){

	return 0;
}

bool init_speed(){
	/*
	// set up input capture on T3C1 (PB4)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_4);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// time to set up upcounting counter/timer
	// falling level on pin generates interrupt and resets counter
	TIM_ICInitTypeDef T3InitStruct;
	T3InitStruct.TIM_Channel = TIM_Channel_1;
	T3InitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;
	T3InitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	T3InitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV8; //CAPTURE DONE EVERY 8 EVENTS!?
	// invalid-> // div x from (72M / AHB (1) / APB1 (2) = 36 MHz)
	T3InitStruct.TIM_ICFilter = 0b0001; // idk, 2 samples I think?
	TIM_ICInit(TIM3, T3InitStruct);

	// enable interrupt
	 */



	// Run timer all the time and get timer value when GPIO interrupt fire

	// Set up GPIO and interrupt

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_4);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// time to set up upcounting counter/timer
	// falling level on pin generates interrupt and resets counter

	// If SysClk = 72 MHz
	// For AHBclk: Div SysClk by 1 (AHB Prescaler(HPRE)=0) to get 72 MHz
	// For APB1: Div PCLK by 2 (APB Prescaler(PPRE1)=0x4) to get 36 MHz (cannot exceed 36MHz)
	// For TIMxCLK: Multiply! APB1 by 2 (APB1 prescaler != 1) to get 72 MHz

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/*TIM_TimeBaseInitTypeDef TimeBase;
	TimeBase->TIM_ClockDivision = ;
	TimeBase->TIM_CounterMode = TIM_CounterMode_Up;
	TimeBase->TIM_Period = TIM_;
	TimeBase->TIM_Prescaler;
	TimeBase->TIM_RepetitionCounter;
	TIM_TimeBaseInit(TIM2, */
	TIM_InternalClockConfig(TIM3); // set CK_PSC to be from CK_INT
	TIM_PrescalerConfig(TIM3, (1080-1), TIM_PSCReloadMode_Update); // prescaler can go up to 65536. Set it so that 72 MHz turns to 66.667 kHz
	TIM_SetClockDivision(TIM3, TIM_CKD_DIV1); //nb, this exists too.
	TIM_SetAutoreload(TIM3, 65535); // overflow event shall set is just a little less, cause apparently our rpm/vss value is very low

	TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Global); //set urs bit to get from ARR register (under/overflow only)

	// and I do not care about update event (UEV) nor update interrupt flag (UIF) nor overflow
	TIM_CounterModeConfig(TIM3, TIM_CounterMode_Up);

	//but I do want to get interrupt when PB5 is falling
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line4;    // No idea what this is related to. But works only with pin number it seems
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //Interrupt mode, optional values for the interrupt EXTI_Mode_Interrupt and event EXTI_Mode_Event.
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //Trigger mode, can be a falling edge trigger EXTI_Trigger_Falling, the rising edge triggered EXTI_Trigger_Rising, or any level (rising edge and falling edge trigger EXTI_Trigger_Rising_Falling)
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn; // Related to pin number!
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; //Priority 2,
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02; //Sub priority 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //Enable external interrupt channel
	NVIC_Init(&NVIC_InitStructure);

	// and enable timer overflow to avoid rollover to show false high value
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM3_IRQn);

	TIM_Cmd(TIM3, ENABLE); //handles CR1 CREN bit.







	return 0;
}

bool init_tacho(){

	// Run timer all the time and get timer value when GPIO interrupt fire

	// Set up GPIO and interrupt

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_5);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// time to set up upcounting counter/timer
	// falling level on pin generates interrupt and resets counter

	// If SysClk = 72 MHz
	// For AHBclk: Div SysClk by 1 (AHB Prescaler(HPRE)=0) to get 72 MHz
	// For APB1: Div PCLK by 2 (APB Prescaler(PPRE1)=0x4) to get 36 MHz (cannot exceed 36MHz)
	// For TIMxCLK: Multiply! APB1 by 2 (APB1 prescaler != 1) to get 72 MHz

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/*TIM_TimeBaseInitTypeDef TimeBase;
	TimeBase->TIM_ClockDivision = ;
	TimeBase->TIM_CounterMode = TIM_CounterMode_Up;
	TimeBase->TIM_Period = TIM_;
	TimeBase->TIM_Prescaler;
	TimeBase->TIM_RepetitionCounter;
	TIM_TimeBaseInit(TIM2, */
	TIM_InternalClockConfig(TIM2); // set CK_PSC to be from CK_INT
	TIM_PrescalerConfig(TIM2, (1080-1), TIM_PSCReloadMode_Update); // prescaler can go up to 65536. Set it so that 72 MHz turns to 66.667 kHz
	TIM_SetClockDivision(TIM2, TIM_CKD_DIV1); //nb, this exists too.
	TIM_SetAutoreload(TIM2, 65535); // overflow event shall set is just a little less, cause apparently our rpm/vss value is very low

	TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Global); //set urs bit to get from ARR register (under/overflow only)

	// and I do not care about update event (UEV) nor update interrupt flag (UIF) nor overflow
	TIM_CounterModeConfig(TIM2, TIM_CounterMode_Up);

	//but I do want to get interrupt when PB5 is falling
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;    // No idea what this is related to. But works only with pin number it seems
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //Interrupt mode, optional values for the interrupt EXTI_Mode_Interrupt and event EXTI_Mode_Event.
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //Trigger mode, can be a falling edge trigger EXTI_Trigger_Falling, the rising edge triggered EXTI_Trigger_Rising, or any level (rising edge and falling edge trigger EXTI_Trigger_Rising_Falling)
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; // Related to pin number!
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; //Priority 2,
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02; //Sub priority 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //Enable external interrupt channel
	NVIC_Init(&NVIC_InitStructure);

	// and enable timer overflow to avoid rollover to show false high value
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM2_IRQn);


	TIM_Cmd(TIM2, ENABLE); //handles CR1 CREN bit.

	return 0;
}

/*
// called every time value is grabbed from timer
bool restart_tacho(){
	// set register value to 0
	TIM_SetCounter(TIM2, 0);
	return 0;
}*/

bool init_adc(){
	// NB, ADC clock must not exceed 14 MHz
	// If SysClk = 72 MHz
	// For AHBclk: Div SysClk by 1 (AHB Prescaler(HPRE)=0) to get 72 MHz
	// For APB2: Div PCLK by 1 (APB2 Prescaler(PPRE2)=0) to get 72 MHz
	// For ADC: Div APB2 by 8 (ADCPRE=0b11) to get 9 MHz
	RCC->CFGR |= (RCC_CFGR_ADCPRE_0 | RCC_CFGR_ADCPRE_1);

	//RCC->APB2ENR &= ~((uint32_t)(RCC_APB2ENR_ADC1EN)); //enable clock
	//RCC->APB2ENR &= ~((uint32_t)(RCC_APB2ENR_ADC2EN));
	//RCC->APB2RSTR &= ~((uint32_t)(RCC_APB2RSTR_ADC1RST)); //remove reset
	//RCC->APB2RSTR &= ~((uint32_t)(RCC_APB2RSTR_ADC2RST));
	//now same as before just shorter??
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_DeInit(ADC1);

	ADC_Cmd(ADC1, ENABLE);
	// wake up ADC1 (master)
	ADC1->CR2 |= (ADC_CR2_ADON); //set ADON bit

	// batteryvolt in
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_1);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //is this analog in or alternative in
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// set conversion time to max, 239.5 cycles
	ADC1->SMPR2 |= (ADC_SMPR2_SMP1_2 |ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_0);

	return 0;
}

/*bool get_speed(uint16_t *speed){ //returned in km/h //I prolly won't go over 255 km/h but nice to have spare

#ifdef FAKE_RESULTS
	speed = 123;
	return 0;
#endif


	return 0;
}*/
/*
bool get_tacho(uint16_t *rpm){ //returned in RPM

#ifdef FAKE_RESULTS
	rpm = 6666;
	return 0;
#endif


	return 0;
}*/

bool get_batteryvolt(uint16_t *volt){ //returned in mV

#ifdef FAKE_RESULTS
	volt = 1234;
	return 0;
#endif

	uint32_t voltCalc;

	//set adc channel to AD1_1

	//ADC1->CR1 |= ADC_CR1_DISCNUM_0; //perform just one measurement
	// set sequence
	ADC1->SQR1 &= ((uint32_t)(ADC_SQR1_L_3 | ADC_SQR1_L_2 | ADC_SQR1_L_1 | ADC_SQR1_L_0)); //L bits to be total seq length to be 1
	//ADC1->SQR2;
	ADC1->SQR3 &= ~((uint32_t)(0x3FFFFFFF)); // clear all sequence items
	ADC1->SQR3 |= (1 & ADC_SQR3_SQ1); //set ch 1 to be first conversion

	//read out if some crap in there already. It appears this is essential
	voltCalc = (ADC1->DR)&0xFFFF; // only lower half of 32bit register

	//perform conversion on ADC1 (master)
	ADC1->CR2 |= (ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0); //set trigger to be SW
	ADC1->CR2 |= (ADC_CR2_SWSTART); //set SWSTART bit
	ADC1->CR2 |= (ADC_CR2_ADON); //set ADON bit


	//wait for conversion to finish
	while(!(ADC1->SR & ADC_SR_EOC)); // todo: add timeout

	voltCalc = (ADC1->DR)&0xFFFF; // only lower half of 32bit register

	// convert bits to mV
	// adc vref is 3300mV
	// voltdiv has upper 10k, lower 2k2 (22 / 122 = 11 / 61)

	voltCalc = voltCalc * 61 * 3300; // voltCalc can be up to 21336 for OK calc. Above range of 12bit in with 32bit, all ok
	voltCalc = voltCalc / (11 * 0xFFF);

	*volt = voltCalc;

	return 0;
}

void eternalRPM_VSS_monitor(){
	// loop monitoring VSS and RPM


	uint32_t timestamp = HAL_GetTick();

	const uint32_t notifydelay = 1000;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	// VSS
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_4);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// RPM
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_5);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	bool vssReported=1; //1 cause they are high idle
	bool rpmReported=1; //1 cause they are high idle

	UART_sendstring(USB_UART, "VSS&RPM monitor loop start\r\n");

	while(1){
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)){
			//vss high
			if (vssReported == 0) {
				vssReported = 1;
				//send UART
				UART_sendint(USB_UART, HAL_GetTick());
				UART_sendstring(USB_UART, " VSS\r\n");
			}
		} else {
			//vss low
			if (vssReported) {
				vssReported = 0;
			}
		}

		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)){
			//rpm high
			if (rpmReported == 0) {
				rpmReported = 1;
				//send UART
				UART_sendint(USB_UART, HAL_GetTick());
				UART_sendstring(USB_UART, " RPM\r\n");
			}
		} else {
			//rpm low
			if (rpmReported) {
				rpmReported = 0;
			}
		}

		if((HAL_GetTick() - timestamp) >= notifydelay){
			// called every 1000 ms
			GPIO_WriteBit(GPIOC, GPIO_Pin_13, 0);
			timestamp += notifydelay;
		}

	}
}

int main(int argc, char* argv[])
{
	//these three functions are holy and may not be modified
	//must be first things in main.	clock needs to be set first ofc
	HAL_InitTick();
	init_uart_usb();
	checkForUpdateRequest();
	//holy part over, free for all now


	uint16_t batteryVoltage_mV;

	// Send a greeting to the trace device (skipped on Release).
	//trace_puts("Hello ARM World!");
	init_pc13_led();
	// At this stage the system clock should have already been configured
	// at high speed.
	//trace_printf("System clock: %u Hz\n", SystemCoreClock);



	init_uart_gps();
	init_uart_extension();
	init_speed();
	init_tacho();
	init_adc();

	GPIO_WriteBit(GPIOC, GPIO_Pin_13, 1); // LED off

	if(get_batteryvolt(&batteryVoltage_mV)){
		UART_sendstring(USB_UART, "Failed to get batteryvolt\r\n");
	} else {
		UART_sendstring(USB_UART, "Batteryvolt: ");
		UART_sendint(USB_UART, batteryVoltage_mV);
		UART_sendstring(USB_UART, " mV\r\n");
	}


	//eternalRPM_VSS_monitor();

  //eternalUSBUART_loopback();




  //timer_start();

  //blink_led_init();
  
  //uint32_t seconds = 0;

  // Infinite loop
  while (1)
    {
	  get_batteryvolt(&carData_latest.batteryValue);
	  if(carData_previousPrinted.RPMvalue != carData_latest.RPMvalue){
		  UART_sendint(USB_UART, HAL_GetTick());

		  UART_sendstring(USB_UART, ";RPM:");
	  UART_sendint(USB_UART, carData_latest.RPMvalue);

	  UART_sendstring(USB_UART, ";VSS:");
	  UART_sendint(USB_UART, carData_latest.VSSvalue);

	  UART_sendstring(USB_UART, ";RTO:");
	 	  UART_sendint(USB_UART, (carData_latest.RPMvalue * 1000) /carData_latest.VSSvalue);

	 	 UART_sendstring(USB_UART, ";BAT:");
	 	 	 	  UART_sendint(USB_UART, carData_latest.batteryValue);

	 	  UART_sendstring(USB_UART, "\r\n");
	 	 carData_previousPrinted = carData_latest;
	  }



      //blink_led_on();
      //timer_sleep(seconds == 0 ? TIMER_FREQUENCY_HZ : BLINK_ON_TICKS);

      //blink_led_off();
      //timer_sleep(BLINK_OFF_TICKS);

      //++seconds;

      // Count seconds on the trace device.
      //trace_printf("Second %u\n", seconds);
    }
  // Infinite loop, never return.
}






// Interrupts

void EXTI9_5_IRQHandler(void){
	const uint32_t pulses_per_rev = 2;
	const uint32_t pulse_duration_us = 15; // time per tick in us
	uint32_t calculationVar;
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, 0); // LED on
	if(EXTI_GetITStatus(EXTI_Line5)!=RESET) //Judge whether a line break
	{
		// tacho intterupt
		carData_latest.RPMcounter = TIM2->CNT;
		// set register value to 0 to measure for next pulse
		TIM_SetCounter(TIM2, 0);

		// timer counts at 66666.67 Hz. 20,000 ticks = 100 rpm; 500 ticks = 4000 rpm.
		calculationVar = carData_latest.RPMcounter * pulse_duration_us; // t (in us)
		calculationVar = calculationVar * pulses_per_rev; // t (in us) per rev
		calculationVar = calculationVar / 60; // ???
		calculationVar = 1000000 / calculationVar; // remove microsecond, get rpm value
		carData_latest.RPMvalue = calculationVar;

		// reset timer
		EXTI_ClearITPendingBit(EXTI_Line5);   //Remove LINE interrupt flag bit
	}


}

void EXTI4_IRQHandler(void){
	const uint32_t pulse_duration_us = 15; // time per tick in us

	// "multiply by (initial) 1.6" in two steps:
	const uint32_t multiplier_mul = 16;
	const uint32_t multiplier_div = 10;

	uint32_t calculationVar;
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, 0); // LED on
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET) //Judge whether a line break
	{
		// vss
		carData_latest.VSScounter = TIM3->CNT;
		// set register value to 0 to prepare for next pulse
		TIM_SetCounter(TIM3, 0);

		// timer counts at 66666.67 Hz. 20,000 ticks = 100 rpm; 500 ticks = 4000 rpm.
		calculationVar = carData_latest.VSScounter * pulse_duration_us; // t (in us)
		calculationVar = calculationVar * multiplier_div; // apply multiplier (step 1 of 2)
		calculationVar = calculationVar / multiplier_mul; // apply multiplier (step 2 of 2)
		calculationVar = 1000000 / calculationVar; // perform flip, remove microsecond, get vss
		carData_latest.VSSvalue = calculationVar;

		// reset timer
		EXTI_ClearITPendingBit(EXTI_Line4);   //Remove LINE interrupt flag bit
	}
}

void TIM2_IRQHandler(void){
	// rpm timer ovf
	TIM2->CNT = 65000;
	carData_latest.RPMvalue = 0;
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void TIM3_IRQHandler(void){
	// vss timer ovf
	TIM3->CNT = 65000;
	carData_latest.VSSvalue = 0;
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
