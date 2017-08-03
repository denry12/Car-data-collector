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
//#include "BlinkLed.h"
#include "stm32f10x_usart.h"

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
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 1 / 10)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"



#define FAKE_RESULTS 1 //for debugging purposes, returns some random values
#define USBUART_EVENPARITY //if commented out, even parity disabled (no parity used)

USART_TypeDef *USB_UART = USART1;

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
#endif
#ifndef USBUART_EVENPARITY
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

void checkForUpdateRequest(){

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

bool UART_sendstring(USART_TypeDef *UARTx, char *string){
	int i = 0;

	while(string[i] != 0){
		while(!(UARTx->SR & USART_SR_TXE));
		USART_SendData(UARTx, (uint16_t)(string[i]));
		i++;
	}

	//USART_SendData(UARTx, (uint16_t)(string));
	return 0;
}

bool UART_sendint(USART_TypeDef *UARTx, uint32_t value){
	char string[15]; //allowing for 32bits nicely
	itoa(value, string, 10);
	UART_sendstring(UARTx, string);
	return 0;
}

void eternalUSBUART_loopback(){
	uint32_t timestamp = HAL_GetTick();

	const uint32_t notifydelay = 1000;

	bool lastNotifyState = 0;

	//uint16_t uartchar16;
	//char uartchar;

	UART_sendstring(USB_UART, "\r\nLoopback\r\n");

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

	return 0;
}

bool init_tacho(){

	return 0;
}

bool init_adc(){

	return 0;
}

bool get_speed(uint16_t *speed){ //returned in km/h //I prolly won't go over 255 km/h but nice to have spare

	return 0;
}

bool get_tacho(uint16_t *rpm){ //returned in RPM

	return 0;
}

bool get_batteryvolt(uint16_t *volt){ //returned in mV

	return 0;
}

int main(int argc, char* argv[])
{
	//these two functions are holy and may not be modified
	//must be first things in main
  //clock needs to be set first ofc
	init_uart_usb();
	checkForUpdateRequest();
	//holy part over, free for all now

	// Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");
  HAL_InitTick();
  init_pc13_led();
  // At this stage the system clock should have already been configured
  // at high speed.
  trace_printf("System clock: %u Hz\n", SystemCoreClock);



	init_uart_gps();
	init_uart_extension();
	init_speed();
	init_tacho();
	init_adc();



  eternalUSBUART_loopback();




  //timer_start();

  //blink_led_init();
  
  //uint32_t seconds = 0;

  // Infinite loop
  while (1)
    {
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

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
