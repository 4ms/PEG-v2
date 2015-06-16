
#include <stdarg.h>
#include <ctype.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"
#include "dac_mcp4921.h"

volatile uint32_t tapouttmr;
volatile uint32_t tapintmr;
volatile uint32_t pingtmr;
volatile uint32_t divpingtmr;
volatile uint32_t eo1tmr;
volatile uint32_t eo2tmr;




extern volatile char timer_overflowed;

extern volatile uint16_t dacbuf[4];
extern volatile uint8_t dacbuf_rd;
extern volatile uint8_t dacbuf_wr;



//takes 1.56uS = 25 clocks
SIGNAL (TIMER0_OVF_vect){


	tapouttmr+=MINTIME;
	tapintmr+=MINTIME;
	pingtmr+=MINTIME;
	divpingtmr+=MINTIME;
	eo1tmr+=MINTIME;
	eo2tmr+=MINTIME;


	TCNT0=TMROFFSET; // 1 clock

	timer_overflowed++; //3 clocks


}
uint32_t get_tapouttmr(void){
	uint32_t result;
	cli();
	result = (tapouttmr << 8) | TCNT0;
	sei();
	return result;

}
uint32_t get_tapintmr(void){
	uint32_t result;
	cli();
	result = (tapintmr << 8) | TCNT0;
	sei();
	return result;

}
uint32_t get_pingtmr(void){
	uint32_t result;
	cli();
	result = (pingtmr << 8) | TCNT0;
	sei();
	return result;

}
uint32_t get_divpingtmr(void){
	uint32_t result;
	cli();
	result = (divpingtmr << 8) | TCNT0;
	sei();
	return result;

}
uint32_t get_eo1tmr(void){
	uint32_t result;
	cli();
	result = eo1tmr;
	sei();
	return result;

}
uint32_t get_eo2tmr(void){
	uint32_t result;
	cli();
	result = eo2tmr;
	sei();
	return result;

}

void reset_tapouttmr(void){
	cli();
	tapouttmr=0;
	sei();
}

void reset_tapintmr(void){
	cli();
	tapintmr=0;
	sei();
}
void reset_pingtmr(void){
	cli();
	pingtmr=0;
	sei();
}
void reset_divpingtmr(void){
	cli();
	divpingtmr=0;
	sei();
}
void reset_eo1tmr(void){
	cli();
	eo1tmr=0;
	sei();
}
void reset_eo2tmr(void){
	cli();
	eo2tmr=0;
	sei();
}


void inittimer(void){


	//Fast PWM , TOP at 0xFF, OC0 disconnected, Prescale @ FCK/8
	TCCR0A=(1<<WGM01) | (1<<WGM00) | (0<<COM0A0) | (0<<COM0A1);
	TCCR0B= (0<<WGM02) | (0<<CS00) | (1<<CS01) | (0<<CS02);
	TCNT0=TMROFFSET;
	TIMSK0|=(1<<TOIE0); 					// Enable timer overflow interrupt

	divpingtmr=0;
	pingtmr=0;
	tapouttmr=0;
	tapintmr=0;
	sei();

/*
prescale / 64 makes 800us steps (1.25kHz aliasing == audible, no good!)

prescale= /8 @ 20MHz => TCNT0/tmrms/now inc's +1 every 0.4us
with uint32_t tmrms, max time is about 1718s or 28.6 minutes
A glitch occurs when tmrms overflows, so this is not long enough (need at least 24 hours, preferrably 720 hours = a month)

*/
}
