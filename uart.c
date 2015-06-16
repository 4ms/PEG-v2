#include <stdarg.h>
#include <ctype.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#include "uart.h"

#define UARTBUFFSIZE 8
uint8_t uart_buff[UARTBUFFSIZE];
uint8_t uart_start_ptr=0;
uint8_t uart_end_ptr=0;

volatile uint8_t RX_error=0;

void init_uart(uint16_t ubbr){
	UBRR0 = ubbr;

	UCSR0A=0x00;
	UCSR0B=/*(1<<TXEN0) |*/ (1<<RXEN0) | (1<<RXCIE0); 	//enable RX and interrupt for RX

    //asynchronous 8N1  (8-bit character, no parity, 1 stop bit)
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);

//    DDRD |= (1<<PD1); //TXD output
	DDRD &= ~(1<<PD1);	//disable TX at first

    DDRD &= ~(1<<PD0); //RXD input

}

void usart_putc(unsigned char c) {
   // wait until UDR ready
        while(!(UCSR0A & (1 << UDRE0)));
        UDR0 = c;    // send character
}

void enable_usart_TX(void){	
	UCSR0B |= (1<<TXEN0);
	DDRD |= (1<<PD1);
}

void disable_usart_TX(void){
	while(!(UCSR0A & (1 << TXC0)));
	UCSR0B &= ~(1<<TXEN0);
	DDRD &= ~(1<<PD1); //tri-state: input and disable pullup
	PORTD &= ~(1<<PD1); 
}

ISR(USART_RX_vect){

	uint8_t udr=UDR0;
	uint8_t ucsra=UCSR0A;
	uint8_t uart_end_ptr_tmp=uart_end_ptr;

	//calculate the next pointer position
	if (++uart_end_ptr_tmp>UARTBUFFSIZE) uart_end_ptr_tmp=0;

	//return if the buffer is full
	if (uart_start_ptr==uart_end_ptr_tmp)
		return;

	//return if there's a frame error, data overrun, or parity error
	if(ucsra & ((1<<FE0)|(1<<DOR0)|(1<<UPE0)))	{
		RX_error=ucsra;
		return;
	}

	//store the incoming data in the buffer
	uart_buff[uart_end_ptr]=udr;

	//update the pointer to the next position
	uart_end_ptr=uart_end_ptr_tmp;

}

uint8_t get_next_uart_data(uint8_t *data){
	
	//make sure the buffer is not empty
	if (uart_start_ptr==uart_end_ptr)
		return (0);
	
	//fetch the next byte and store it
	*data = uart_buff[uart_start_ptr];
	
	//increment the uart buffer pointer
	if (++uart_start_ptr>UARTBUFFSIZE) uart_start_ptr=0;

	return(1);
}
