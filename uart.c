#include <stdio.h>
#include <stdint.h>
#include "uart.h"

#include "stm32f4xx.h"

#define GPIOAEN          (1U<<0)
#define UART2EN          (1U<<17)

#define SYS_FREQ         16000000
#define APB1_CLK         SYS_FREQ
#define UART_BAUDRATE	 115200

#define CR1_TE           (1U<<3)
#define CR1_UE           (1U<<13)

#define SR_TXE			 (1U<<7)



static uint32_t compute_uart_baud(uint32_t periph_clk, uint32_t baudrate);
static void uart_set_baudrate(uint32_t periph_clk, uint32_t baudrate);

static void uart_write(int ch);

void uart_tx_init(void)
{
	// enable clock to uart gpioa
	RCC->AHB1ENR |= GPIOAEN;
	// set pa2 mode to alternate function for uart tx
	GPIOA->MODER &= ~(1U<<4);
	GPIOA->MODER |= (1U<<5);
	// set af type to AF7 (UART2_TX)
	GPIOA->AFR[0] |= (1U<<8)|(1U<<9)|(1U<<10);
	GPIOA->AFR[0] &= ~(1<<11);
	// enable clock access to uart2
	RCC->APB1ENR |= UART2EN;
	// set baudrate
	uart_set_baudrate(APB1_CLK,UART_BAUDRATE);
	// set tx direction
	USART2->CR1 = CR1_TE;
	// enable uart
	USART2->CR1 |= CR1_UE;
}
int __io_putchar(int ch)
{
	uart_write(ch);
	return ch;
}


static void uart_write(int ch)
{
  // ensure transmit data register is empty
	while(!(USART2->SR & SR_TXE));
	// write to transmit data register
	USART2->DR = (ch & 0xff);
}

static void uart_set_baudrate(uint32_t periph_clk, uint32_t  baudrate)
{
	USART2->BRR = compute_uart_baud(periph_clk, baudrate);
}

static uint32_t compute_uart_baud(uint32_t periph_clk, uint32_t baudrate)
{
	return ((periph_clk + (baudrate/2U))/baudrate);
}
