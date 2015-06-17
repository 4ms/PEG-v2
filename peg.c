/*
Copyright (c) 2015 4ms Company

Author: Dan Green - danngreen1@gmail.com

LICENSE:

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

See http://creativecommons.org/licenses/MIT/ for more information.
*/

/* 
 For use with PEG PCB v2.0 
 (see peg_v1 project in github for PEG PCB 1.0.x)
*/

/*****************
 *  GLOBAL MODES *
 *****************/

//Enable to save code space by skipping the log lookup table. Useful for compiling with no optimization when using the debugger.
//#define OMITLOGCURVES

//Enables the UART, for communication with a PEG parameter controller
#define ENABLE_UART

#define F_CPU 16000000


/*****************
 *  INCLUDES     *
 *****************/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "uart.h"

#include "timer.h"
#include "dac_mcp4921.h"

#ifndef OMITLOGCURVES
	#include "log4096.h"
#endif

FUSES =
{
	.extended = 0x04, /*BOD 4.3V*/
	.high = 0xd9, /*d1 to preserve EEPROM*/
	.low = 0xd6  /*or 0xce*/
};


/*************************
 *  GLOBAL VARIABLES     *
 *************************/

//udiv32() is located in the div32.S file
extern uint32_t udiv32(uint32_t divisor);

extern volatile uint32_t tapintmr;
extern volatile uint32_t pingtmr;
extern volatile uint32_t divpingtmr;
extern volatile uint32_t eo1tmr;
extern volatile uint32_t eo2tmr;


volatile uint32_t ping_irq_timestamp=0, trigq_irq_timestamp=0;
volatile char reset_nextping_flag=0;
volatile char sync_to_ping_mode=1;
volatile uint8_t trigq_jack_down=0;
volatile char using_tap_clock=0;
volatile char timer_overflowed=0;

char eo1_high=0,eo2_high=0;


/***************
 *   SETTINGS  *
 ***************/

//3000000 is about 1.25s
#define HOLDTIMECLEAR 4800000

#define SKEW_ADC_DRIFT 1

#define USER_INPUT_POLL_TIME 400

//2400 is 1ms
//12000 is 5ms
#define LIMIT_SKEW_TIME 15000

//10 is about 100ms
#define NUM_ADC_CYCLES_BEFORE_TRANSITION 10

#define DIV_ADC_HYSTERESIS 1

//SYSTEM_MODE_HOLD_TIME: how long the ping button must be held down to enter System Mode
//with HOLDTIMECLEAR at 4800000:
	//200000 is about 5s
	//150000 is about 3.75s
	//103000 is about 2.5
#define SYSTEM_MODE_HOLD_TIME 130000
#define SYSTEM_MODE_EXIT_TIME 0x00080000

#define SYSTEM_MODE_CYCLE_FLASH_BRIGHT_ON 15000
#define SYSTEM_MODE_CYCLE_FLASH_BRIGHT_OFF 25000
#define SYSTEM_MODE_CYCLE_FLASH_DIM_ON 5000
#define SYSTEM_MODE_CYCLE_FLASH_DIM_OFF 50000


//119 is 13.15ms
//91 is 10.0ms
//46 is 5ms
//37 is 4ms
//9 is 1.1ms
//4 is 500us
//1 is 212us

#define EO_TRIG_TIME 91
#define EO_GATE_TIME 37


/*******************
 *   COMM          *
 *******************/
 //OR these with BLUE_DETECT
#define CMD_WRITE			0b11111100
#define CMD_ARE_YOU_HERE		0b11110000
#define CMD_ARE_YOU_HERE_PING		0b11111000
#define	CMD_HOW_ARE_YOU			0b11110100


#define REPLY_STATE			0b11000010
#define REPLY_HI_MY_NAME_IS		0b11001100
#define REPLY_HI_I_AM_PINGING		0b11000100


/*******************
 *   EEPROM        *
 *******************/

#define EEPROM_SET 0b10101010
#define EEPROM_CLEAR 0b00000000
#define EEPROM_UNPROGRAMMED 0b11111111

#define TAPCLK_EEPROMADDR 16
#define PING_EEPROMADDR 17
#define LIMIT_SKEW_EEPROMADDR 18
#define HALFRISE_EEPROMADDR 19
#define EO1_TRIG_EEPROMADDR 21
#define ASYNC_SUSTAIN_EEPROMADDR 20
#define EO2_TRIG_EEPROMADDR 22


char EOF_IS_TAPCLKOUT=0;
char NO_FREERUNNING_PING=0;
char EOR_IS_HALFRISE=0;
char EO1_IS_TRIG=0;
char EO2_IS_TRIG=0;
char LIMIT_SKEW=0;
char ROLLOFF_PING=1;
char ASYNC_CAN_SUSTAIN=1;

//The following are not user modifiable, change at your own risk!
#define QNT_REPHASES_WHEN_CYCLE_OFF 0
#define CYCLE_REPHASES_DIV_PING 1 


/************************
 * Mnemonics		*
 ************************/

#define WAIT 0
#define RISE 1
#define SUSTAIN 2
#define FALL 3
#define TRANSITION 4

#define EXP 0
#define EXP25 1
#define EXP50 2
#define EXP75 3
#define LIN 4
#define LIN25 5
#define LIN50 6
#define LIN75 7
#define LOG 8

/*******************
 * PIN DEFINITIONS *
 *******************/

#define DEBUG_pin PC4
#define DEBUG_init
//#define DEBUG_init DDRC |= (1<<DEBUG_pin)
#define DEBUGFLIP PORTC ^= (1<<DEBUG_pin)
#define DEBUGHIGH PORTC |= (1<<DEBUG_pin)
#define DEBUGLOW PORTC &= ~(1<<DEBUG_pin)

#define DEBUG2_pin PD0
#define DEBUG2_init
//#define DEBUG2_init DDRD |= (1<<DEBUG2_pin)
#define DEBUG2FLIP PORTD ^= (1<<DEBUG2_pin)
#define DEBUG2HIGH PORTD |= (1<<DEBUG2_pin)
#define DEBUG2LOW PORTD &= ~(1<<DEBUG2_pin)


#define PINGLED_pin PB1
#define PINGLED_init DDRB |= (1<<PINGLED_pin)
#define PINGLEDFLIP PORTB ^= (1<<PINGLED_pin)
#define PINGLEDHIGH PORTB |= (1<<PINGLED_pin)
#define PINGLEDLOW PORTB &= ~(1<<PINGLED_pin)

#define PING_pin PD2
#define PING_init DDRD &= ~(1<<PING_pin); PORTD &= ~(1<<PING_pin)
#define PING (PIND & (1<<PING_pin))

#define TRIGA_pin PD4
#define TRIGA_init DDRD &= ~(1<<TRIGA_pin); PORTD &= ~(1<<TRIGA_pin)
#define TRIGA (PIND & (1<<TRIGA_pin))

#define TRIGQ_pin PD3
#define TRIGQ_init DDRD &= ~(1<<TRIGQ_pin); PORTD &= ~(1<<TRIGQ_pin)
#define TRIGQ (PIND & (1<<TRIGQ_pin))

#define CYCLE_BUT_pin PC3
#define CYCLE_BUT_init DDRC &= ~(1<<CYCLE_BUT_pin); PORTC |= (1<<CYCLE_BUT_pin)
#define CYCLE_BUT_RAW (!(PINC & (1<<CYCLE_BUT_pin)))

#define CYCLE_LED_pin PC4
#define CYCLE_LED_init DDRC |= (1<<CYCLE_LED_pin)
#define CYCLE_LED_ON PORTC |= (1<<CYCLE_LED_pin)
#define CYCLE_LED_OFF PORTC &= ~(1<<CYCLE_LED_pin)

#define CYCLE_INVERT_pin PC5
#define CYCLE_INVERT_init DDRC &= ~(1<<CYCLE_INVERT_pin); PORTC &= ~(1<<CYCLE_INVERT_pin)
#define CYCLE_INVERT (PINC & (1<<CYCLE_INVERT_pin))

#define CYCLE_BUT ((CYCLE_BUT_RAW && !CYCLE_INVERT) || (!CYCLE_BUT_RAW && CYCLE_INVERT))

#define EO1_pin PD6
#define EO2_pin PD5

#define EO1_OFF PORTD |= (1<<EO1_pin)
#define EO1_ON PORTD &= ~(1<<EO1_pin)

#define EO2_OFF PORTD |= (1<<EO2_pin)
#define EO2_ON PORTD &= ~(1<<EO2_pin)


#define EO_init DDRD |= (1<<EO1_pin) | (1<<EO2_pin)


#define CHANNEL_ID_OUT_pin PC0
#define CHANNEL_ID_OUT_init DDRC |= (1 << CHANNEL_ID_OUT_pin)
#define CHANNEL_ID_OUT_ON PORTC |= (1 << CHANNEL_ID_OUT_pin)
#define CHANNEL_ID_OUT_OFF PORTC &= ~(1 << CHANNEL_ID_OUT_pin)

#define TAPIN_pin PD7
#define TAPIN_init DDRD &= ~(1<<TAPIN_pin); \
		PORTD |=(1<<TAPIN_pin)
#define TAPIN (!(PIND & (1<<TAPIN_pin)))


#define ADC_DDR DDRC
#define ADC_PORT PORTC
#define ADC_mask 0b00000111
#define NUM_ADC 3


#define CURVE_adc 6
#define CLKDIV_adc PC2
#define SKEW_adc PC1


#define SPI_PIN PINB
#define SPI_PORT PORTB
#define SPI_DDR DDRB
#define SPI_MOSI PB3
#define SPI_MISO PB4
#define SPI_SCLK PB5

#define BLUE_DETECT_PIN PB0
#define READ_BLUE_DETECT (PINB & (1<<BLUE_DETECT_PIN))
#define BLUE_DETECT_init DDRD &= ~(1<<BLUE_DETECT_PIN)
char BLUE_DETECT=0;


/**************
 *  FUNCTIONS *
 **************/

uint8_t diff(uint8_t a, uint8_t b);
inline uint8_t diff(uint8_t a, uint8_t b){
	if (a>b) return (a-b);
	else return (b-a);
}

uint32_t diff32(uint32_t a, uint32_t b);
inline uint32_t diff32(uint32_t a, uint32_t b){
	if (a>b) return (a-b);
	else return(b-a);
}

void init_spi(void){
	
	SPI_DDR |= (1<<SPI_MOSI) | (1<<SPI_SCLK); //SCK, MOSI are output
	DAC_CS_DDR |= (1<<DAC_CS); //CS for MCP4922 DAC
	DAC_CS_PORT |= (1<<DAC_CS); //pull CS high to intialize	
	
	SPI_DDR &= ~(1<<SPI_MISO); // MISO is input
				
	SPCR = (1<<SPE) | (1<<MSTR) | (0<<SPR1)| (0<<SPR0); //SPI enable, Master Mode, F_OSC/4, interrupt enabled
	SPSR = (1<<SPI2X); //SPI double speed = 2MHz

	output_dac(0);
}

void init_pins(void){
	init_spi();
	EO_init;
	PING_init;
	TRIGA_init;
	TRIGQ_init;
	CYCLE_BUT_init;
	CYCLE_LED_init;
	TAPIN_init;
	BLUE_DETECT_init ;
	CHANNEL_ID_OUT_init;
	PINGLED_init;
	CYCLE_INVERT_init;
	DEBUG_init;
	DEBUG2_init;
	
}

int8_t get_clk_div_nominal(uint8_t adc_val){
	if (adc_val<=1) 	  // /8 <=0..2 (3)
		return(8);
	else if (adc_val<=10) // /7 <= 3..15 (13)
		return(7);
	else if (adc_val<=26) // /6 <= 16..33 (18)
		return(6);
	else if (adc_val<=46) // /5 <= 34..53 (20)
		return(5);
	else if (adc_val<=65) // /4 <= 54..78 (27)
		return(4);
	else if (adc_val<=85) // /3 <= 79..97 (19)
		return(3);
	else if (adc_val<=100) // /2 <= 98..114 (19)
		return(2);
	else if (adc_val<=118) // =1 <= 117..134 (18)
		return(1);
	else if (adc_val<=128) // x2 <= 135..154 (20)
		return(-2);	
	else if (adc_val<=150) // x3 <= 155..177 (23)
		return(-3);	
	else if (adc_val<=170) // x4 was 196 <= 178..200 (23)
		return(-4);
	else if (adc_val<=188) // x5 <= 201..220 (20)
		return(-5);
	else if (adc_val<=207) // x6 was 246 <= 221..242 ()
		return(-6);
	else if (adc_val<=220) // x7 was 251 <= 243..253 ()
		return(-7);
	else /*if (adc_val<=255)*/ // x8 <=253..255 (3)
		return(-8);

//	else 
//		return(1);
}
uint32_t get_clk_div_time(int8_t clock_divide_amount, uint32_t clk_time){
	if (clock_divide_amount==8)  // /8
		return(clk_time<<3);
	else if (clock_divide_amount==7) // /7
		return(clk_time*7);
	else if (clock_divide_amount==6) // /6
		return(clk_time*6);
	else if (clock_divide_amount==5) // /5
		return(clk_time*5);
	else if (clock_divide_amount==4) // /4
		return(clk_time<<2);
	else if (clock_divide_amount==3) // /3
		return(clk_time*3);
	else if (clock_divide_amount==2) // /2
		return(clk_time<<1);
	else if (clock_divide_amount==1) // =1
		return(clk_time);
	else if (clock_divide_amount==-1) // =1
		return(clk_time);
	else if (clock_divide_amount==-2) // *2
		return(clk_time>>1);
	else if (clock_divide_amount==-3) // *3
		return(clk_time/3);
	else if (clock_divide_amount==-4) // *4
		return(clk_time>>2);
	else if (clock_divide_amount==-5) // *5
		return(clk_time/5);
	else if (clock_divide_amount==-6) // *6
		return(clk_time/6);
	else if (clock_divide_amount==-7) // *7
		return(clk_time/7);
	else if (clock_divide_amount==-8) // *8
		return(clk_time>>3);
else return(clk_time);
}

uint32_t get_fall_time(uint8_t skew_adc, uint32_t div_clk_time){
	uint32_t t,u;
	uint8_t rev_skew;

	if (!LIMIT_SKEW || (div_clk_time<(LIMIT_SKEW_TIME>>1)) ){
		if (skew_adc==0)
			return (768);

		else if (skew_adc==1)
			return(1024);

		else if (skew_adc==2)
			return(1280);

		else if (skew_adc<=25){
			t=(skew_adc) * (div_clk_time >> 8);
			u=(skew_adc*skew_adc*64)+960;
		
			if (t<1280) t=1280;
			if (t<u) return t;
			else 
			return u;

		}
		else if (skew_adc>=220) 
			return(div_clk_time-256);
		
		else if (skew_adc>200){
			t=(skew_adc) * (div_clk_time >> 8);
			if (t>(div_clk_time-256)) t= div_clk_time-256;

			rev_skew=255-skew_adc;
			u=rev_skew*rev_skew*64;

			if (u>(div_clk_time-256)){
					return t;
			} else {
				u=div_clk_time-u;
				if(t>u)	return t;
				else return u;
			}
		}
		else if ((skew_adc>101) && (skew_adc<=114))
			return(div_clk_time>>1);
		
		else 
			return ((skew_adc) * (div_clk_time >> 8));
	}

	else { //LIMIT_SKEW

		if ((skew_adc>101) && (skew_adc<=114)){
			return(div_clk_time>>1);
		} else {
			t=(skew_adc) * (div_clk_time >> 8);

			if (t<LIMIT_SKEW_TIME) t=LIMIT_SKEW_TIME;
			if (t>(div_clk_time-LIMIT_SKEW_TIME)) t=div_clk_time-LIMIT_SKEW_TIME;

			return(t);
		}
	}
}


int16_t calc_curve(int16_t t_dacout, char cur_curve){
	uint16_t t_loga, t_inv_loga;

	#ifndef OMITLOGCURVES
		t_loga=pgm_read_word_near(&(loga[t_dacout]));
		t_inv_loga=4095-pgm_read_word_near(&(loga[4095-t_dacout]));
	#else
		t_inv_loga=(((int32_t)t_dacout)*((int32_t)t_dacout))>>12;
		t_loga=4095-(
			(
				((int32_t)(4095-t_dacout)) * ((int32_t)(4095-t_dacout))
			)
		>>12
		);
	#endif

	if (cur_curve==LIN)
		return (t_dacout);
	
	else if (cur_curve==LOG)
		return(t_loga);

	else if (cur_curve==EXP)
		return(t_inv_loga);

	else if (cur_curve==EXP25){	//25% exp 75% lin
		return((t_dacout >> 1) + (t_dacout >> 2) + (t_inv_loga >> 2));
	}
	else if (cur_curve==EXP50)		//50% exp 50% lin
		return((t_inv_loga >> 1) + (t_dacout>>1));

	else if (cur_curve==EXP75){	//75% exp 25% lin
		return((t_inv_loga >> 1) + (t_inv_loga >> 2) + (t_dacout >> 2));
	}

	else if (cur_curve==LIN25){	//25% lin 75% log
		return((t_loga >> 1) + (t_loga >> 2) + (t_dacout >> 2));
	}
	else if (cur_curve==LIN50)	//50% lin 50% log
		return((t_loga >> 1) + (t_dacout>>1));

	else if (cur_curve==LIN75){	//75% lin 25% log
		return((t_dacout >> 1) + (t_dacout >> 2) + (t_loga >> 2));
	}
	else return(t_dacout);
}


void init_adc(void){
	//init the ADC:
	ADC_DDR &= ~(ADC_mask); //adc input
	ADC_PORT &= ~(ADC_mask); //disable pullup
	ADCSRA = (1<<ADEN);	//Enable ADC
	ADMUX = (1<<ADLAR);	//Left-Adjust
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //prescale = clk/128 = 125kHz
	ADCSRA |= (1<<ADSC);//set the Start Conversion Flag in the ADC Status Register
}

/*
TIMERS:
TCNT0 increments every 0.5us at 16MHz
overflows every 128us

So a timer value is expressed in 0.5us
e.g. if get_pingtmr() returns 8000, that's 4ms

*/

ISR (INT0_vect){
	if (PING){
		ping_irq_timestamp=(pingtmr << 8) | TCNT0;
		pingtmr=0;
		using_tap_clock=0;
	}
}

ISR (INT1_vect){
	if (TRIGQ){
		trigq_irq_timestamp=1;
		sync_to_ping_mode=1;
		reset_nextping_flag=1;
		trigq_jack_down=1;
	} else {
		trigq_jack_down=0;
	}
	
}


void pcint_init(void){
	//interrupt on any change on INT1 (PD3/triga), or rising-edge of INT0 (PD2/ping)
	EICRA = (1<<ISC10) | (0<<ISC11) | (1<<ISC00) | (0<<ISC01);
	EIMSK = (1<<INT1) | (1<<INT0);

	PCMSK2 = 0;
	PCMSK1 = 0;
}

void read_blue_detect(void){
	BLUE_DETECT=READ_BLUE_DETECT;
}

void eo1_on(void);
void eo2_on(void);
void eo1_off(void);
void eo2_off(void);

void eor_on(void);
void eor_off(void);
void eof_on(void);
void eof_off(void);
void hr_on(void);
void hr_off(void);
void tapclkout_off(void);
void tapclkout_on(void);

void eo1_on(void){
	if (!eo1_high){
		EO1_ON;
		eo1_high=1;
		reset_eo1tmr();
	}
}

void eo2_on(void){
	if (!eo2_high){
		EO2_ON;
		eo2_high=1;
		reset_eo2tmr();
	}
}

void eo1_off(void){
	if (!EO1_IS_TRIG && (eo1tmr>EO_GATE_TIME)) {
		EO1_OFF;
		reset_eo1tmr();
	}
	eo1_high=0;
}
void eo2_off(void){
	if (!EO2_IS_TRIG && (eo2tmr>EO_GATE_TIME)) {
		EO2_OFF;
		reset_eo2tmr();
	}
	eo2_high=0;
}

inline void eor_on(void){
	if (!EOR_IS_HALFRISE)
		eo1_on();
}
inline void eor_off(void){
	if (!EOR_IS_HALFRISE)
		eo1_off();
}

inline void eof_on(void){
	if (!EOF_IS_TAPCLKOUT)
		eo2_on();
}
inline void eof_off(void){
	if (!EOF_IS_TAPCLKOUT)
		eo2_off();
}

inline void hr_on(void){
	if (EOR_IS_HALFRISE)
		eo1_on();
}
inline void hr_off(void){
	if (EOR_IS_HALFRISE)
		eo1_off();
}

inline void tapclkout_on(void){
	if (EOF_IS_TAPCLKOUT) 
		eo2_on();
}
inline void tapclkout_off(void){
	if (EOF_IS_TAPCLKOUT) 
		eo2_off();
}

/***************************************************
 *             MAIN() FUNCTION                     *
 *                                                 *
 ***************************************************/


int main(void){

	uint16_t timer_overflowed_running_total=0;
	char use_timer_overflowed=0;

	uint32_t now=0;

	char env_state=WAIT;

	char reset_now_flag=0;

	char end_env_flag=0;
	char end_segment_flag=0;

	uint8_t triga_jack_down=0;
	uint8_t cycle_down=0;

	uint32_t div_clk_time=0;

	uint32_t async_phase_diff=0;
	uint8_t async_env_changed_shape=1;

	uint32_t elapsed_time=0;

	uint32_t clk_time=0;
	uint32_t rise_time=0;
	uint32_t fall_time=0;
	uint64_t time_t=0;

	uint32_t tapout_clk_time=0;

	uint8_t skew_adc=127; 
	uint8_t clock_div_adc=127;
	uint8_t curve_adc=127;

	int8_t clock_divider_amount=1;
	int8_t t_clock_divider_amount=1;
	int8_t c_d_a=1;
	int8_t new_clock_divider_amount=1;
	int8_t hys_clock_divider_amount=0;

	char curve_rise=LIN;
	char curve_fall=LIN;
	char cur_curve=LIN;

	char next_curve_rise=LIN, next_curve_fall=LIN;

	char current_adc_channel=CLKDIV_adc,last_adc_channel;

	unsigned char adch=0;
	uint16_t poll_user_input=0;

	char tapin_down=0;
	char tapin_up=0;
	uint32_t last_tapin_time=0;

	char envelope_running=0;

	int16_t t_dacout=0;
	//uint16_t t_loga, t_inv_loga;
	uint32_t rise_inc=0;
	uint32_t fall_inc=0;
	int32_t accum=0;

	uint8_t update_risefallincs=0;
	char divmult_changed=0;
	uint8_t didnt_change_divmult=0;
	char tracking_changedrisefalls=0;

	int32_t new_accum=0;
	int16_t new_dacout=0;
	int32_t transition_inc=0;
	char next_env_state=WAIT;
	int8_t transition_ctr=0;
	char outta_sync=0;
	uint32_t rise_per_clk=0;

	int8_t ping_div_ctr=0;
	char div_ping_led=0;

	char ready_to_start_async=0;
	char got_tap_clock=0;
	uint8_t temp_u8=0,d,data1,data2;
	uint16_t temp_u16=0;
	uint32_t temp_u32=0;
	uint8_t flash_cycle_led=0;

	uint32_t entering_system_mode=0;
	uint8_t system_mode_cur=0;
	uint8_t initial_cycle_button_state=0;
	char update_cycle_button_now=0;

	//uint8_t ping_link_mode_master=0;
	//uint8_t send_ping_link_now=0;

	uint8_t rx_data=0, rx_ptr=0,rx_checksum=0;
	uint8_t rx_buff[3]={0,0,0};


	/** Initialize **/


	accum=0;

	inittimer();
	init_pins();

	CHANNEL_ID_OUT_ON;

	init_adc();
	pcint_init();

	init_uart(64);
	
	eor_off();
	eof_on();

	if (CYCLE_BUT) CYCLE_LED_ON;
	else CYCLE_LED_OFF;

	_delay_ms(5);

	read_blue_detect();

	temp_u8=eeprom_read_byte((const uint8_t *)(TAPCLK_EEPROMADDR));
	if (temp_u8==EEPROM_SET)
		EOF_IS_TAPCLKOUT=1;
	else
		EOF_IS_TAPCLKOUT=0;

	temp_u8=eeprom_read_byte((const uint8_t *)(PING_EEPROMADDR));
	if (temp_u8==EEPROM_SET)
		NO_FREERUNNING_PING=1;
	else
		NO_FREERUNNING_PING=0;

	temp_u8=eeprom_read_byte((const uint8_t *)(HALFRISE_EEPROMADDR));
	if (temp_u8==EEPROM_SET)
		EOR_IS_HALFRISE=1;
	else
	if (temp_u8==EEPROM_UNPROGRAMMED && BLUE_DETECT) {//a new chip with the blue jumper should default to HALFRISE mode
		EOR_IS_HALFRISE=1; 
		eeprom_busy_wait();
		eeprom_write_byte ((uint8_t *)(HALFRISE_EEPROMADDR),EEPROM_SET);
		eeprom_busy_wait();
	}
	else EOR_IS_HALFRISE=0;

	temp_u8=eeprom_read_byte((const uint8_t *)(EO1_TRIG_EEPROMADDR));
	if (temp_u8==EEPROM_SET)
		EO1_IS_TRIG=1;
	else
		EO1_IS_TRIG=0;

	temp_u8=eeprom_read_byte((const uint8_t *)(EO2_TRIG_EEPROMADDR));
	if (temp_u8==EEPROM_SET)
		EO2_IS_TRIG=1;
	else
		EO2_IS_TRIG=0;

	temp_u8=eeprom_read_byte((const uint8_t *)(LIMIT_SKEW_EEPROMADDR));
	if (temp_u8==EEPROM_SET)
		LIMIT_SKEW=1;
	else
		LIMIT_SKEW=0;

	temp_u8=eeprom_read_byte((const uint8_t *)(ASYNC_SUSTAIN_EEPROMADDR));
	if (temp_u8==EEPROM_SET)
		ASYNC_CAN_SUSTAIN=0;
	else
		ASYNC_CAN_SUSTAIN=1;



#ifdef OMITLOGCURVES
	BLUE_DETECT=0;
	EOF_IS_TAPCLKOUT=1;
	NO_FREERUNNING_PING=1;
	LIMIT_SKEW=0;
	ASYNC_CAN_SUSTAIN=1;
	EO1_IS_TRIG=0;
	EO2_IS_TRIG=0;
	EOR_IS_HALFRISE=0;
	EOF_IS_TAPCLKOUT=0;
#endif

	_delay_ms(5);

	clk_time=0;
	div_clk_time=0;
	last_tapin_time=0;
	sync_to_ping_mode=1;

	//Read all analog inputs

	/** Main loop: about 19uS, but +5uS if some interrupts run **/
	while(1){


		/***************** TAP TEMPO ********************
		*						
		*	Read tap tempo buttons, and record	
		*	time between taps.			
		*					
		*	Clear the tap clock if button is held	
		*	down for more than HOLDTIMECLEAR	
		*						
		*************************************************/

		if (TAPIN){
			tapin_down=0;
			now=get_tapintmr();

			if (!(tapin_up)){
				tapin_up=1;
				using_tap_clock=1;

			#ifndef OMITLOGCURVES		
				if (last_tapin_time && (diff32(last_tapin_time,now)<(last_tapin_time>>1)) ) {
					clk_time=(now>>1) + (last_tapin_time>>1);
				} else {
					clk_time=now;
					last_tapin_time=now;
				}
			#else
					clk_time=now;
					last_tapin_time=now;
			#endif

				tapout_clk_time=clk_time;

				reset_tapintmr();
				reset_tapouttmr();

				div_clk_time=get_clk_div_time(clock_divider_amount,clk_time);
				fall_time=get_fall_time(skew_adc, div_clk_time);
				rise_time=div_clk_time-fall_time;
				rise_inc=udiv32(rise_time>>5);
				fall_inc=udiv32(fall_time>>5);

				didnt_change_divmult=NUM_ADC_CYCLES_BEFORE_TRANSITION; //force us to do a transition mode as if divmult or skew was changed
				poll_user_input=USER_INPUT_POLL_TIME; //force us to enter the ADC block

			} else {
				if (now > HOLDTIMECLEAR){ //button has been down for more than 2 seconds

					if (using_tap_clock){
						env_state=WAIT;
						envelope_running=0;

						reset_divpingtmr();
						clk_time=0; //added v4.2
						div_clk_time=0; //added v4.2

						accum=0;
						using_tap_clock=0;
						timer_overflowed=1;
					}
					tapout_clk_time=0;
					last_tapin_time=0;
					reset_tapouttmr();

					PINGLEDLOW;

				} else {
					PINGLEDHIGH;
				}

			}
		}
		else {
			tapin_up=0;
			if (!(tapin_down)){
				PINGLEDLOW;
				tapin_down=1;
			}
		}


/******************** READ TRIG JACKS  ****************
 *							
 *  Start and/or sync the envelope when receiveing a trigger 
 *	
 * 	-Quantized Trigger
 *  -Async Trigger		
 ******************************************************/


	// Read Trig jacks:
	// (Re)start the envelope if triggere
	
		if (TRIGA){
			if (!triga_jack_down){
			
				sync_to_ping_mode=0;
				triga_jack_down=1;

				reset_now_flag=1;
				ready_to_start_async=0;
				
				if (rise_time>0x1000)  //do an immediate fall if rise_time is fast
					outta_sync=1;		//otherwise set the outta_sync flag which works to force a slew limited transition to zero

				//start each envelope the same time after the divided clock
				async_phase_diff=get_divpingtmr();
				
			}
		} else {
			triga_jack_down=0;
		}


		if (trigq_irq_timestamp){ //this is set when the pinchange interrupt detects a TRIGQ jack going high
			trigq_irq_timestamp=0; //clear it so we only run this once per TRIGQ

			/*
			When Cycle is on, a TRIGQ should always re-phase (re-start) a divided-ping env.
			When Cycle is off, if and only if QNT_REPHASES_WHEN_CYCLE_OFF is set, then a TRIGQ should rephase 
			
			If the envelope is not running, a TRIGQ should always start the envelope.
			
			ping_div_ctr=clock_divider_amount makes sure that a divided-ping envelope will start from its beginning on the next ping
			*/
			if (QNT_REPHASES_WHEN_CYCLE_OFF || CYCLE_BUT || !envelope_running){
				ping_div_ctr=clock_divider_amount;
				if (rise_time>0x1000)  //do an immediate fall if rise_time is fast
					outta_sync=1;		//otherwise set the outta_sync flag which works to force a slew limited transition to zero
			}

			tracking_changedrisefalls=0;

			curve_rise=next_curve_rise;
			curve_fall=next_curve_fall;

		}




/********** READ CYCLE BUTTON **********
 *							
 *  Start the envelope mid-way in it's curve (must be calculated)
 *	
 * 	-Cycle Button						
 *  -Also if we change the divide amount while the envelope is running, we want to do the same calculation as
 *    if the CYCLE button was pressed (i.e. calculate where the envelope should be so that it "hits" zero at the
 *    proper divided/multed clock time)
 ***************************************/


		if (CYCLE_BUT){


			CYCLE_LED_ON;

			if (!cycle_down){
			

				if (clk_time>0) { //don't start the envelope if we don't have a PING time set yet

					if (using_tap_clock)
						elapsed_time=get_tapouttmr();
					else
						elapsed_time=get_pingtmr();


					div_clk_time=get_clk_div_time(clock_divider_amount,clk_time);
					fall_time=get_fall_time(skew_adc, div_clk_time);
					rise_time=div_clk_time-fall_time;
					rise_inc=udiv32(rise_time>>5);
					fall_inc=udiv32(fall_time>>5);


					//Start the envelope at whatever point it would be in if it were already running

					if (!envelope_running && sync_to_ping_mode){
						envelope_running=1;

						if (clock_divider_amount<=1){		//if we're multiplying, calculate the elapsed time
							while (elapsed_time > div_clk_time)	//since the last multiplied clock
								elapsed_time -= div_clk_time;		
						} else {							//otherwise, we're dividing the clock
							while (elapsed_time <= div_clk_time) //so we want to get as close to the end of the divided cycle
								elapsed_time += clk_time;
							elapsed_time-=clk_time;
						}


						if (elapsed_time <= rise_time) {  //Are we on the rise?
							time_t=((uint64_t)elapsed_time) * 4096;
							accum = time_t/rise_time;
							accum<<=16;
							env_state=RISE;
							curve_rise=next_curve_rise;
						
							if (t_dacout>=2048) hr_on();
							else hr_off();
							eor_off();
							eof_on();
						}
						else {
							elapsed_time=elapsed_time-rise_time;
							time_t=((uint64_t)elapsed_time) * 4096; //accum = 4096 * elapsed_time/curve_time
							accum = 4096 - (time_t/fall_time);
							accum<<=16;
							env_state=FALL;
							curve_fall=next_curve_fall;

							if (t_dacout<2048) hr_off();
							else hr_on();
							eor_on();
							eof_off();
						}

						ping_div_ctr=clock_divider_amount;	

					} else if (!envelope_running && !sync_to_ping_mode){
							envelope_running=1;
							reset_now_flag=1;
							ready_to_start_async=0;
							async_phase_diff=get_divpingtmr();

					} else if (envelope_running && CYCLE_REPHASES_DIV_PING) {
							ping_div_ctr=clock_divider_amount;
					}

					cycle_down=1;
 
				} //if (clk_time>0)
			} //if (!cycle_down)

		} else{
			CYCLE_LED_OFF;
			cycle_down=0;
		}



/******************** DIVIDED CLOCK (PING LED) ***********************
 *
 * Flash the PING LED
 * Start the envelope with the divided PING clock if we're multiplying
 * 
 *********************************************************************/

		now=get_divpingtmr();

		/*
		 Phase-lock the async'ed envelope 
		   if we're cycling in async mode
		...and have passed the phase offset point 
		...and the envelope hasn't changed shape
		...and are "ready", meaning we have completed an envelope or received the divided ping (this flag ensures we only reset once per divclk period)
		...and we're not holding a sustain
		 */

		if (async_phase_diff>div_clk_time) async_phase_diff=0; //fail-safe measure

		if (!sync_to_ping_mode && CYCLE_BUT && (now >= async_phase_diff) && !async_env_changed_shape && ready_to_start_async && !triga_jack_down)
		{ 	
			reset_now_flag=1;
			ready_to_start_async=0;
			async_env_changed_shape=0;
		}



		if (div_ping_led && (now>=(div_clk_time>>1))){
			PINGLEDLOW;
			div_ping_led=0;
		}

		if (div_clk_time){

			if ((!div_ping_led) && (now>div_clk_time)){

				now=(now-div_clk_time)>>8;
				cli();
				divpingtmr=now;
				sei();

				PINGLEDHIGH;
				div_ping_led=1;
			
				if (!sync_to_ping_mode)
					ready_to_start_async=1;

				/* Reset a multiplied envelope when the div_clk occurs
				   if we are in cycle mode or high-gate QNT jack, or have a request to reset/start
				*/
				if (sync_to_ping_mode){ 
					if (reset_nextping_flag || CYCLE_BUT || trigq_jack_down ){		
						if (!tracking_changedrisefalls) {
								reset_now_flag=1;
								reset_nextping_flag=0;
								if ((t_dacout>64) && !outta_sync)
									temp_u8=0;//do nothing
						}
					}  
					ping_div_ctr=clock_divider_amount; //make sure it restarts the next ping
				}
			}

		} else {
			PINGLEDLOW;
		}


		

/**************** HANDLE TAP CLOCK *********************
 *							
 ******************************************************/

		if (tapout_clk_time){
			now=get_tapouttmr();

			if (now>=(tapout_clk_time>>1)){
				tapclkout_off();
			}
			if (now>=tapout_clk_time){
				reset_tapouttmr();
				if (using_tap_clock) {
					if (clock_divider_amount<=1) reset_divpingtmr();
					got_tap_clock=1;
				}
				tapclkout_on();

			}
		}

/**************** READ PING CLOCK *********************
 *							
 *  If the ping interrupt was executed
 *  calculate the new clock time, divided clock time,
 *  and rise/fall times
 *
 *  Also address starting and sync'ing the envelope to pings
 ******************************************************/



		if (got_tap_clock || ping_irq_timestamp){
			
#ifdef ENABLE_UART
		/*	if (ping_link_mode_master)
				send_ping_link_now=1;
				*/

#endif

			if (ping_irq_timestamp && PING) { //added && PING because it was sometimes false triggering on falling edge of PING 
				clk_time=ping_irq_timestamp;
			}
			if (!using_tap_clock)
				last_tapin_time=0;

			ping_irq_timestamp=0;

			if (clock_divider_amount<=1) {//multiplying, reset div_clk on every ping
				if (!using_tap_clock) reset_divpingtmr();
				PINGLEDHIGH;
				div_ping_led=1;
			}

			got_tap_clock=0;
			
			//see if it changed more than 1.5%, if so force a slew limited transition... otherwise just jump-cut

			elapsed_time=div_clk_time; //temporary variable
			div_clk_time=get_clk_div_time(clock_divider_amount,clk_time);
			if (elapsed_time){
				if (div_clk_time>elapsed_time) now=div_clk_time-elapsed_time;
				else now=elapsed_time-div_clk_time;
				//if (now>(elapsed_time>>6)) //more than 1.5% change
				if (now>(elapsed_time>>3)) //smooth out more than 12.5% change (allow to chop a <12.5%)
				{
					didnt_change_divmult=NUM_ADC_CYCLES_BEFORE_TRANSITION; //force us to do a transition mode as if divmult or skew was changed
					poll_user_input=USER_INPUT_POLL_TIME; //force us to enter the ADC block
				}
			}

			fall_time=get_fall_time(skew_adc, div_clk_time);
			rise_time=div_clk_time-fall_time;
			rise_inc=udiv32(rise_time>>5);
			fall_inc=udiv32(fall_time>>5);


			if (CYCLE_BUT || trigq_jack_down || reset_nextping_flag || envelope_running ){

				if (clock_divider_amount > 1){ //we're dividing the clock, so resync on every N pings
					if (envelope_running && !tracking_changedrisefalls)
						ping_div_ctr++;
						c_d_a=clock_divider_amount;

					if (ping_div_ctr>=c_d_a){
						if (sync_to_ping_mode  && !tracking_changedrisefalls && (CYCLE_BUT || reset_nextping_flag || trigq_jack_down)){
							reset_now_flag=1;
							if ((t_dacout>64) && !outta_sync)
								temp_u8=0;//do nothing

						}
						reset_nextping_flag=0;

						ping_div_ctr=0;
						reset_divpingtmr();

						PINGLEDHIGH;
						div_ping_led=1;
					}

				} else {
					//re-sync on every ping, since we're mult or = to the clock
					if (sync_to_ping_mode  && !tracking_changedrisefalls && (reset_nextping_flag || CYCLE_BUT || trigq_jack_down)){
							reset_now_flag=1;								
							if ((t_dacout>64) && !outta_sync)
								temp_u8=0;//do nothing

					}
					reset_nextping_flag=0; //FYI: this goes low only right after an envelope starts (on the ping, of course)
					reset_divpingtmr();
				} 
			}





		} else { 
			/*	If we haven't received a ping within 2x expected clock time (that is, clock stopped or slowed to less than half speed)
					we should stop the ping clock. Or switch to the Tap clock if it's running and we have Tap Clock Output on EOF
			*/	

			if (clk_time && NO_FREERUNNING_PING && !using_tap_clock){
				now=get_pingtmr();
				if (now >= (clk_time<<1)) {
				
					reset_pingtmr();

					if (tapout_clk_time && EOF_IS_TAPCLKOUT){
						using_tap_clock=1;				
						clk_time=tapout_clk_time;
						div_clk_time=get_clk_div_time(clock_divider_amount,clk_time);
						fall_time=get_fall_time(skew_adc, div_clk_time);
						rise_time=div_clk_time-fall_time;
						rise_inc=udiv32(rise_time>>5);
						fall_inc=udiv32(fall_time>>5);

						reset_now_flag=1;
						timer_overflowed=1;

					} else {
						rise_time=0;
						fall_time=0;
						rise_inc=0;
						fall_inc=0;
						clk_time=0;
						div_clk_time=0;
						envelope_running=0;
						env_state=WAIT;
					}
					PINGLEDLOW;
				}
			}
		}


#ifdef ENABLE_UART

/****************************************
 *   UART
 *   
 ****************************************/


		if (get_next_uart_data(&temp_u8)){

			if (temp_u8 & 0b10000000) rx_ptr=0; //top nibble set represents a CMD

			switch (rx_ptr){
				case (0):
					if (temp_u8 & 0b1000000){
						rx_buff[0]=temp_u8;
						rx_ptr=1;
					}
				break;
		
				case (1):
					rx_buff[1]=temp_u8;
					rx_ptr=2;
				break;

				case (2):
					rx_buff[2]=temp_u8;
					rx_ptr=3;
				break;
			}

			if (rx_buff[0]==(CMD_ARE_YOU_HERE | BLUE_DETECT)){

				rx_ptr=0;
				enable_usart_TX();
				usart_putc(REPLY_HI_MY_NAME_IS | BLUE_DETECT);

				_delay_us(20);

				disable_usart_TX();

			}
			
			if (rx_buff[0]==(CMD_HOW_ARE_YOU | BLUE_DETECT)){
				rx_ptr=0;

				temp_u8=0;
				if (EOR_IS_HALFRISE) temp_u8+=(1<<3);
				if (EOF_IS_TAPCLKOUT) temp_u8+=(1<<5);
				if (ASYNC_CAN_SUSTAIN) temp_u8+=(1<<6);
				if (NO_FREERUNNING_PING) temp_u8+=(1<<7);
				if (LIMIT_SKEW) temp_u8+=(1<<2);	
				if (EO1_IS_TRIG) temp_u8+=(1<<4);
				if (EO2_IS_TRIG) temp_u8+=(1<<1);

				data1=temp_u8>>1; //0b0abcdefg 7 bits
				data2=((temp_u8 & 1)<<6); //0b0h00kkkk 1 bit and checksum

				//checksum
				if (temp_u8 & 0b00000001) data2++;
				if (temp_u8 & 0b00000010) data2++;
				if (temp_u8 & 0b00000100) data2++;
				if (temp_u8 & 0b00001000) data2++;
				if (temp_u8 & 0b00010000) data2++;
				if (temp_u8 & 0b00100000) data2++;
				if (temp_u8 & 0b01000000) data2++;
				if (temp_u8 & 0b10000000) data2++;

				enable_usart_TX();
			//	_delay_us(100);

				usart_putc(REPLY_STATE | BLUE_DETECT);
				usart_putc(data1);
				usart_putc(data2);

				_delay_us(100);

				disable_usart_TX();
			}


			if ((rx_ptr==3) && (rx_buff[0]==(CMD_WRITE | BLUE_DETECT))){

				rx_ptr=0;
				rx_data=(rx_buff[1] << 1) | (rx_buff[2] >> 6);
				rx_checksum=rx_buff[2] & 0b00001111;

				//check checksum
				temp_u8=0;
				if (rx_data & 0b00000001) temp_u8++;
				if (rx_data & 0b00000010) temp_u8++;
				if (rx_data & 0b00000100) temp_u8++;
				if (rx_data & 0b00001000) temp_u8++;
				if (rx_data & 0b00010000) temp_u8++;
				if (rx_data & 0b00100000) temp_u8++;
				if (rx_data & 0b01000000) temp_u8++;
				if (rx_data & 0b10000000) temp_u8++;

				if (temp_u8!=rx_checksum){ 
					//checksum error


				} else {
					if (rx_data & (1<<0))
						EOR_IS_HALFRISE=0;
					else 
						EOR_IS_HALFRISE=1;

					if (rx_data & (1<<3)) {
						EO1_IS_TRIG=0;
					} else {
						EO1_IS_TRIG=1; 
					}

					if (rx_data & (1<<4))
						EOF_IS_TAPCLKOUT=0;
					else
						EOF_IS_TAPCLKOUT=1;

					if (rx_data & (1<<5)){
						EO2_IS_TRIG=0;
					} else {
						EO2_IS_TRIG=1; 
					}

					if (rx_data & (1<<1)) 
						ASYNC_CAN_SUSTAIN=1;
					else
						ASYNC_CAN_SUSTAIN=0;

					if (rx_data & (1<<6))
						LIMIT_SKEW=1;
					else
						LIMIT_SKEW=0;

/*
					if (rx_data & (1<<2)){
						ping_link_mode_master=1;
					} else{
						ping_link_mode_master=0;
					}
*/
					if (rx_data & (1<<7))
						NO_FREERUNNING_PING=0; //off=no free run
					else
				    		NO_FREERUNNING_PING=1; //on=free run

				} //rx_checksum
			} //CMD_WRITE

		}
			
#endif



/****************** READ ADC *********************
 * Curve types
 * Skew
 * Clock division
 *************************************************/

		if (++poll_user_input>USER_INPUT_POLL_TIME){
			poll_user_input=0;


			/** Set the ADC to start a conversion on the next channel (takes <100uS from here to reading ADCH) **/

			if (current_adc_channel==SKEW_adc) current_adc_channel=CLKDIV_adc;
			else if (current_adc_channel==CLKDIV_adc) current_adc_channel=CURVE_adc;
			else if (current_adc_channel==CURVE_adc) current_adc_channel=SKEW_adc;

			ADMUX = (1<<ADLAR) | current_adc_channel;
			ADCSRA |= (1<<ADSC);		//Start Conversion

			while( !(ADCSRA & (1<<ADIF)) );
			ADCSRA |= (1<<ADIF);		// Clear the flag by sending a logical "1"
			adch=ADCH;
			last_adc_channel=current_adc_channel;



			/** Assign the adc data (adch), based on the channel read (last_adc_channel) **/
			update_risefallincs=0;

			if (last_adc_channel==SKEW_adc){

 				if (diff(adch, skew_adc)>SKEW_ADC_DRIFT){
					skew_adc=adch;
					update_risefallincs=1;
					new_clock_divider_amount=clock_divider_amount;
				}
			}

			else if (last_adc_channel==CURVE_adc){

				//don't need to check for drift or hysterias because shape is only updated at end of envelope curve
				curve_adc=adch;

				if (curve_adc<1) {next_curve_rise=EXP;next_curve_fall=LOG;}
				else if (curve_adc<=4) {next_curve_rise=EXP;next_curve_fall=LIN50;}

				else if (curve_adc<=21) {next_curve_rise=EXP;next_curve_fall=LIN;}
				else if (curve_adc<=35) {next_curve_rise=EXP;next_curve_fall=EXP50;}

				else if (curve_adc<=51) {next_curve_rise=EXP;next_curve_fall=EXP;}
				else if (curve_adc<=69) {next_curve_rise=EXP75;next_curve_fall=EXP75;}
				else if (curve_adc<=79) {next_curve_rise=EXP50;next_curve_fall=EXP50;}
				else if (curve_adc<=100) {next_curve_rise=EXP25;next_curve_fall=EXP25;}

				else if (curve_adc<=120) {next_curve_rise=LIN;next_curve_fall=LIN;}
				else if (curve_adc<=137) {next_curve_rise=LIN75;next_curve_fall=LIN75;}
				else if (curve_adc<=149) {next_curve_rise=LIN50;next_curve_fall=LIN50;}
				else if (curve_adc<=166) {next_curve_rise=LIN25;next_curve_fall=LIN25;}

				else if (curve_adc<=183) {next_curve_rise=LOG;next_curve_fall=LOG;}
				else if (curve_adc<=200) {next_curve_rise=LOG;next_curve_fall=LIN50;}
				else if (curve_adc<=211) {next_curve_rise=LOG;next_curve_fall=LIN;}
				else if (curve_adc<=223) {next_curve_rise=LOG;next_curve_fall=EXP50;}
				else if (curve_adc<=255) {next_curve_rise=LOG;next_curve_fall=EXP;}
			}

			else if (last_adc_channel==CLKDIV_adc){

				d=diff(clock_div_adc, adch);

				if ((env_state==TRANSITION) && envelope_running){
					temp_u8=0;  //do nothing
				}
				else if (d==0) {
					temp_u8=0; //do nothing

				} else if (d > DIV_ADC_HYSTERESIS){ //if the adc changed by more than the hysteresis, then we can just use the new adc value

					clock_div_adc=adch;
					new_clock_divider_amount=get_clk_div_nominal(clock_div_adc);

					if (clk_time){
						update_risefallincs=1;
						divmult_changed=1;
					}else{
						clock_divider_amount=new_clock_divider_amount;
					}

				} else { 
					/* CHECK FOR HYSTERESIS
					 If we moved into a faster divmult than the current divmult, add DIV_ADC_HYSTERESIS to the current adc (adding=slower)
				 	 If we moved slower, subtract DIV_ADC_HYSTERESIS from the current adc (subtracting=faster)
					 This will see if we moved far enough into the new divmult territory

					 If adc+/-HYSTER pushes us into a different divmult amount than the straigh measured adc, 
					 then we haven't moved far enough into this divmult range, so we should reject the change.
					 Otherwise, accept the new change and set update_risefallincs flag to 1
					*/

					clock_div_adc=adch;
					t_clock_divider_amount=get_clk_div_nominal(clock_div_adc); 

					if (t_clock_divider_amount > clock_divider_amount){

							if (clock_div_adc <= (0xFF - DIV_ADC_HYSTERESIS)) //Make sure we don't overflow past 0xFF
								temp_u8=clock_div_adc + DIV_ADC_HYSTERESIS;
							else temp_u8=0xFF;
						
							hys_clock_divider_amount=get_clk_div_nominal(temp_u8);
				
					} else if (t_clock_divider_amount < clock_divider_amount){
							if (clock_div_adc > DIV_ADC_HYSTERESIS)
								temp_u8=clock_div_adc - DIV_ADC_HYSTERESIS;
							else temp_u8=0;

							hys_clock_divider_amount=get_clk_div_nominal(temp_u8);

					} else {
						hys_clock_divider_amount=99; //clock_divider_amount has not changed, do nothing
					}
			
					if (hys_clock_divider_amount == t_clock_divider_amount){
						new_clock_divider_amount=t_clock_divider_amount;

						if (clk_time){
							update_risefallincs=1;
							divmult_changed=1;
						}else
							clock_divider_amount=new_clock_divider_amount; 
					}

				} //if (d...)

			} //if last_adc_channel ==...

		/********************
		 Update to the new envelope shape 
		*******************/
			if (didnt_change_divmult) //if it's 0 then that means we already did a TRANSITION for the update_risefallincs
				didnt_change_divmult++; //increment it so that we know how many ADC read cycles we've done since it was reset to 1
										//(which happens when update_risefallincs is true)

			if (update_risefallincs){
				didnt_change_divmult=1; //reset this to 1 which indicates we've made a change to risefallincs
				if (envelope_running && sync_to_ping_mode)
					tracking_changedrisefalls=1;
				
				async_env_changed_shape=1;

				if (divmult_changed){
					clock_divider_amount=new_clock_divider_amount;
					
					if (ping_div_ctr<0) ping_div_ctr=0;
					if (ping_div_ctr>clock_divider_amount) ping_div_ctr=clock_divider_amount;

					div_clk_time=get_clk_div_time(new_clock_divider_amount,clk_time);

				}

				fall_time=get_fall_time(skew_adc, div_clk_time);
				rise_time=div_clk_time-fall_time;
				rise_inc=udiv32(rise_time>>5);
				fall_inc=udiv32(fall_time>>5);

				update_risefallincs=0;


			} //update_risefallincs

				/************* SLEW LIMITED TRANSITION *************
					Only do a slew-limited transition if we're running locking to ping (which is always, in some versions)
					
					Calculate the new accum value if we were to switch instantly
					  This will be used to calculate the slope of the transistion segment
					  which is necessary so we don't chop instantly (but instead set the transistion time to 10ms)

					 Reset didnt_change_divmult to 0, so that it won't increment (above) and we will then be locked out of this 
					 TRANSITION block until update_risefallincs goes 1 again
				****************************************************/

			if (/*envelope_running && */ div_clk_time && sync_to_ping_mode && (didnt_change_divmult>=NUM_ADC_CYCLES_BEFORE_TRANSITION)){
				tracking_changedrisefalls=0;
				didnt_change_divmult=0;	
				divmult_changed=0;

				if (using_tap_clock)
					elapsed_time=get_tapouttmr();
				else
					elapsed_time=get_pingtmr();

				if (clock_divider_amount<=1){	
					while (elapsed_time > div_clk_time)
						elapsed_time -= div_clk_time;
				} else {
					
					if (envelope_running){

						if (env_state==RISE){
							ping_div_ctr=0;
							if (rise_time>clk_time) //otherwise rise_time<=clk_time, so leave ping_div_ctr=0
							{ 
								rise_per_clk=rise_inc * (clk_time>>8);
								if (rise_per_clk<32) rise_per_clk=32;
								temp_u32=0;
								temp_u8=0;
								while ((temp_u32<0x10000000) && (temp_u8<=clock_divider_amount)){
									temp_u32+=rise_per_clk;
									temp_u8++;
									if (accum>=temp_u32) ping_div_ctr++;
								}
							}
						} else if (env_state==FALL){
							ping_div_ctr=clock_divider_amount-1;
							if (fall_time>clk_time) //otherwise leave ping_div_ctr at the last step
							{
								rise_per_clk=fall_inc * (clk_time>>8);
								if (rise_per_clk<32) rise_per_clk=32;
								temp_u32=0;
								temp_u8=0;
								while ((temp_u32<0x10000000) && (temp_u8<=clock_divider_amount)){
									temp_u32+=rise_per_clk;
									temp_u8++;
									if (accum>=temp_u32) ping_div_ctr--;
								}
							}
						}

						while (elapsed_time >= clk_time)
							elapsed_time -= clk_time;
						elapsed_time+=(ping_div_ctr*clk_time);


					} else { //envelope not running

						ping_div_ctr=clock_divider_amount; //the next ping will be the one that the envelope starts on
						while (elapsed_time <= div_clk_time)
							elapsed_time += clk_time;
						elapsed_time-=clk_time;
					}
				}

				cli();
				divpingtmr=elapsed_time>>8;
				sei();
				
				if (envelope_running){
					elapsed_time=elapsed_time + ((uint32_t)0x00008000); //offset to account for transition period: 128 timer overflows
					if (elapsed_time>div_clk_time)
						elapsed_time-=div_clk_time;

					if (elapsed_time <= rise_time) {  //Are we on the rise?
						time_t=((uint64_t)elapsed_time) << 12;
						new_dacout = time_t/rise_time;
						new_accum = ((int32_t)new_dacout) <<16;
						new_dacout=calc_curve(new_dacout,next_curve_rise);
						next_env_state=RISE;
					} else {
						elapsed_time=elapsed_time-rise_time;
						time_t=((uint64_t)elapsed_time) << 12; 
						new_dacout = 4096 - (time_t/fall_time);
						new_accum = ((int32_t)new_dacout) <<16;
						new_dacout=calc_curve(new_dacout,next_curve_fall);
						next_env_state=FALL;
					}

					accum=((int32_t)t_dacout)<<16;
		
					if (new_dacout>t_dacout)
						transition_inc=((int32_t)(new_dacout - t_dacout)) << 9;
					else {
						transition_inc=((int32_t)(t_dacout - new_dacout)) << 9;
						transition_inc = -1 * transition_inc;
					}
					cur_curve=LIN;

					env_state=TRANSITION;
					transition_ctr=128;	

					outta_sync=1;
					ready_to_start_async=1;	
				}

				if ((clock_divider_amount>1) && envelope_running){
			
					if (next_env_state==RISE){
						ping_div_ctr=0;
						if (rise_time>clk_time) //otherwise leave ping_div_ctr at zero
						{ 
							rise_per_clk=rise_inc * (clk_time>>8);
							if (!rise_per_clk) rise_per_clk=1;
							temp_u32=0;
							temp_u8=0;
							while ((temp_u32<0x10000000) && (temp_u8<=clock_divider_amount)){
								temp_u32+=rise_per_clk;
								temp_u8++;
								if (new_accum>=temp_u32) ping_div_ctr++;
							}
						}
					} else if (next_env_state==FALL){
						ping_div_ctr=clock_divider_amount-1;
						if (fall_time>clk_time) //otherwise leave ping_div_ctr at the last step
						{
							rise_per_clk=fall_inc * (clk_time>>8);
							temp_u32=0;
							temp_u8=0;
							while ((temp_u32<0x10000000) && (temp_u8<=clock_divider_amount)){
								temp_u32+=rise_per_clk;
								temp_u8++;
								if (new_accum>=temp_u32) ping_div_ctr--;
							}

						}
					}
									
				}
					
			}

		
		}

		/********* EOR/F TRIGGERS **************
		 *
		 ***************************************/

		 if (EO1_IS_TRIG){
			now=get_eo1tmr();
			if (now>EO_TRIG_TIME){
				reset_eo1tmr();
				EO1_OFF;
				//eo1_high=1; //this is the trick! Keep this flag set so that future eo_on() won't send it high again, until we get a eo_off()
			}
		}

		if (EO2_IS_TRIG){
			now=get_eo2tmr();
			if (now>EO_TRIG_TIME){
				reset_eo2tmr();
				EO2_OFF;
				//eo2_high=1;
			}
		 }


		/*********UPDATE THE ENVELOPE************
		 *					
		 *  Update only when timer has overflowed
		 *  Timer overflows about once every 3-4 main loop cycles
		 * -restart if needed			
		 * -calculate new position		
		 * -change curve step (RISE/FALL)	
		 * -output EOR/EOF/HalfR 		
		 *					
		 ****************************************/

		if ((clk_time==0) || (div_clk_time==0)){
			envelope_running=0;
			outta_sync=0;
		}

		if (timer_overflowed){
			use_timer_overflowed=timer_overflowed;
			timer_overflowed=0;



			if (reset_now_flag){ 
				
				reset_now_flag=0;

				if (t_dacout<0x0010)
					outta_sync=0; // if we're practically at bottom, then consider us in sync and do an immediate transition

				if (/*(t_dacout<0x0020) &&*/ (!envelope_running ||  (outta_sync==0) || (div_clk_time<0x8000))){

					envelope_running=1;
					env_state=RISE;
					accum=0;
					eof_on();
					eor_off();

				} else {
					if (outta_sync==1) outta_sync=2;
					env_state=TRANSITION;
					//envelope_running=1; ///shouldn't we have this ??
					
	
					elapsed_time=0x8000; //0x8000 offset to account for transition period: 64/128 timer overflows (6/13ms)
					if (elapsed_time>div_clk_time)
						elapsed_time-=div_clk_time;

					if (elapsed_time <= rise_time) {  //Does our transition length exceed the rise time?
						time_t=((uint64_t)elapsed_time) << 12;
						new_dacout = time_t/rise_time;
						new_accum = ((int32_t)new_dacout) <<16;
						new_dacout=calc_curve(new_dacout,next_curve_rise);
						next_env_state=RISE;

					} else {
						elapsed_time=elapsed_time-rise_time;
						time_t=((uint64_t)elapsed_time) << 12; 
						new_dacout = 4096 - (time_t/fall_time);
						new_accum = ((int32_t)new_dacout) <<16;
						new_dacout=calc_curve(new_dacout,next_curve_fall);
						next_env_state=FALL;
					}

					if (new_dacout>t_dacout)
						transition_inc=((int32_t)(new_dacout - t_dacout)) << 9;//9
					else {
						transition_inc=((int32_t)(t_dacout - new_dacout)) << 9;//9
						transition_inc = -1 * transition_inc;
					}

					cur_curve=LIN;
					accum=((int32_t)t_dacout)<<16;

					transition_ctr=128;//128
				}

				curve_rise=next_curve_rise;
				curve_fall=next_curve_fall;

				reset_nextping_flag=0;
				timer_overflowed_running_total=0;
			}

			if (envelope_running){
			//this block takes about 15-18us and runs every 100us (10kHz sampling rate)
				t_dacout=0;
				switch (env_state){

					case(RISE):
						timer_overflowed_running_total+=use_timer_overflowed;
						accum+=rise_inc*use_timer_overflowed;
						t_dacout=accum>>16;

						if (accum>0x0FFF0000){
								accum=0x0FFF0000; 
								t_dacout=0x0FFF;
								if (triga_jack_down && ASYNC_CAN_SUSTAIN)
									end_segment_flag=SUSTAIN;
								else
									end_segment_flag=FALL;
						}
						
						cur_curve=curve_rise;

						if (t_dacout>=2048) hr_on();
						else hr_off();
						eor_off();
						eof_on();
					break;

					case(SUSTAIN):
						eor_off();
						eof_off();
						hr_on();

						t_dacout=0x0FFF;

						if (triga_jack_down && ASYNC_CAN_SUSTAIN){
							accum=0x0FFF0000;
							async_env_changed_shape=1;
						} else {
							end_segment_flag=FALL;
						}
					break;

					case(FALL):
						timer_overflowed_running_total+=use_timer_overflowed;
						accum-=fall_inc*use_timer_overflowed;
						t_dacout=accum>>16;


						if ((accum<0x00010000) || (accum>0x0FFF0000)){
								accum=0;
								t_dacout=0;
								end_env_flag=1;
						}

						eor_on();
						eof_off();
						if (t_dacout<2048)	hr_off();
						else hr_on();

						cur_curve=curve_fall;
					break;

					case(TRANSITION): 
						timer_overflowed_running_total+=use_timer_overflowed;
						accum+=transition_inc*use_timer_overflowed;
						if (accum<0 || (transition_inc==0)) { //trans_inc==0 would technically be an error, so this gives us an out
							accum=0;
							t_dacout=0;
							transition_ctr=use_timer_overflowed;
						} else if (accum>0x0FFF0000){
							accum=0x0FFF0000;
							t_dacout=0x0FFF;
							transition_ctr=use_timer_overflowed;
						} else{
							t_dacout=accum>>16;
						}
						if (transition_inc>0){
							eor_off();
							eof_on();
						} else {
							eor_on();
							eof_off();
						}
						transition_ctr-=use_timer_overflowed;
						if (transition_ctr<=0){
							end_segment_flag=next_env_state;
							accum=new_accum;
							if (outta_sync) { //2 means we got to transistion from reset_now_flag
								outta_sync=0;
							}
							else if (outta_sync==1) outta_sync=2;
							
							else outta_sync=0;
						}


					break;

				}
			

			/**** OUTPUT TO DAC *****/

				t_dacout=calc_curve(t_dacout,cur_curve);

				//Now we've calculated the DAC value, so output it
				output_dac(t_dacout);


		/*** HANDLE END OF SEGMENTS/ENV****/
				if (end_segment_flag){

					if (end_segment_flag==FALL) { //flagged to start fall segment
						curve_fall=next_curve_fall;
					}

					if (end_segment_flag==RISE) {//flagged to start (resume) rise segment
						curve_rise=next_curve_rise;
					}

					if (end_segment_flag==SUSTAIN) { //flagged to start sustain segment
						curve_fall=next_curve_fall;
					}
					env_state=end_segment_flag;
					end_segment_flag=0;
				}

				if (end_env_flag){

					eof_on();
					eor_off();	//should already be OFF, but make sure
					hr_off();

					end_env_flag=0;

					curve_rise=next_curve_rise;
					curve_fall=next_curve_fall;

					if (CYCLE_BUT || trigq_jack_down || reset_nextping_flag){

						if (sync_to_ping_mode){ 
							envelope_running=1;
							reset_nextping_flag=0; 
							env_state=RISE;


						} else { //!sync_to_ping_mode

							ready_to_start_async=1;

								
							envelope_running=1;
							env_state=RISE;
							if (async_env_changed_shape) //if we altered the waveshape, then re-calc the landing spot
								async_phase_diff=get_divpingtmr();
							async_env_changed_shape=0;
						}
						
					}else{
						envelope_running=0;
						env_state=WAIT;
						outta_sync=0;
					}
				}

			} 
			else{ //envelope_running!=1 
				eor_off();
				hr_off();
				eof_on();
				outta_sync=0;
				output_dac(0);
				timer_overflowed=0; //keep this at 0 when envelope is not running, or else when we actually start the env, 
									//accum+=rise_inc*timer_overflowed will cause us to start mid-way in the envelope
			}

		}



	/*************** SYSTEM MODE ********************
     * 												*
	 ************************************************/
		if (TAPIN)
			entering_system_mode++;
		else
			entering_system_mode=0;

		if (entering_system_mode>SYSTEM_MODE_HOLD_TIME){
			for(d=0;d<5;d++){
				EO1_ON;EO2_ON;
				CYCLE_LED_ON;PINGLEDHIGH;
				output_dac(0x0800);
				_delay_ms(100);
				if (d==4) _delay_ms(500);

				EO1_OFF;EO2_OFF;
				CYCLE_LED_OFF;PINGLEDLOW;
				output_dac(0);
				_delay_ms(100);
			}

			while(TAPIN){
			}

			_delay_ms(50);
			entering_system_mode=0;
			initial_cycle_button_state=CYCLE_BUT_RAW;

			/* Setup for the EOF mode*/
			system_mode_cur=0;
			EO2_ON;
			if (EOF_IS_TAPCLKOUT) 	CYCLE_LED_ON;
			else 					CYCLE_LED_OFF;
			
			update_cycle_button_now=1;

			/*Loop until we've held down TAPIN more than SYSTEM_MODE_EXIT_TIME cycles*/
			while(entering_system_mode<SYSTEM_MODE_EXIT_TIME){

				if (TAPIN || update_cycle_button_now) {
					entering_system_mode++;	
						
					if (!tapin_down){
						flash_cycle_led=0;
						tapin_down=1;

						initial_cycle_button_state=CYCLE_BUT_RAW;

						EO1_OFF;
						EO2_OFF;
						PINGLEDLOW;
						output_dac(0);
						
						if (!update_cycle_button_now) system_mode_cur++;
						else update_cycle_button_now=0;

						switch (system_mode_cur){
					
							case(5):
								system_mode_cur=0;
							case(0):
								EO2_ON;d=EO2_IS_TRIG;

								if (!EOF_IS_TAPCLKOUT && !d) 	CYCLE_LED_OFF;
								else if (!EOF_IS_TAPCLKOUT && d) flash_cycle_led=1;
								else if (EOF_IS_TAPCLKOUT && !d)	CYCLE_LED_ON;
								else flash_cycle_led=2;
							break;

							case(1):
								EO1_ON;d=EO1_IS_TRIG;

								if (!EOR_IS_HALFRISE && !d) 	CYCLE_LED_OFF;
								else if (!EOR_IS_HALFRISE && d) flash_cycle_led=1;
								else if (EOR_IS_HALFRISE && !d)	CYCLE_LED_ON;
								else flash_cycle_led=2;
							break;

							case(2):
								output_dac(0x0FFF);
								if (LIMIT_SKEW)	CYCLE_LED_ON;
								else					CYCLE_LED_OFF;
							break;
							
							case(3):
								PINGLEDHIGH;
								if (NO_FREERUNNING_PING) 	CYCLE_LED_ON;
								else 						CYCLE_LED_OFF;
							break;

							case(4):
								EO1_ON;EO2_ON;
								if (ASYNC_CAN_SUSTAIN) CYCLE_LED_OFF;
								else	CYCLE_LED_ON;
							break;


						}//switch system_mode_cur
					}//if tapin_down

				} else { //if TAPIN
					tapin_down=0;
					entering_system_mode=0;
				}		
				
				/* Flash the cycle button if necessary 
				*/
				if (flash_cycle_led==1){
					temp_u16++;
					if (temp_u16<SYSTEM_MODE_CYCLE_FLASH_DIM_ON) CYCLE_LED_ON;
					else if (temp_u16<SYSTEM_MODE_CYCLE_FLASH_DIM_OFF) CYCLE_LED_OFF;
					else temp_u16=0;
				}
				if (flash_cycle_led==2){
					temp_u16++;
					if (temp_u16<SYSTEM_MODE_CYCLE_FLASH_BRIGHT_ON) CYCLE_LED_ON;
					else if (temp_u16<SYSTEM_MODE_CYCLE_FLASH_BRIGHT_OFF) CYCLE_LED_OFF;
					else temp_u16=0;
				}

				/* Check to see if we've changed states with the Cycle button...*/
				temp_u8=CYCLE_BUT_RAW;

				if(initial_cycle_button_state!=temp_u8){
					_delay_ms(50); //to de-noise the cycle button

					flash_cycle_led=0;
					initial_cycle_button_state=temp_u8;

					switch(system_mode_cur){
						case(0):
							EO2_IS_TRIG=1-EO2_IS_TRIG;
							if (!EO2_IS_TRIG) EOF_IS_TAPCLKOUT=1-EOF_IS_TAPCLKOUT;
						break;

						case(1):
							EO1_IS_TRIG=1-EO1_IS_TRIG;
							if (!EO1_IS_TRIG) EOR_IS_HALFRISE=1-EOR_IS_HALFRISE;
						break;

						case(2):
							LIMIT_SKEW=1-LIMIT_SKEW;
						break;

						case(3):
							NO_FREERUNNING_PING=1-NO_FREERUNNING_PING;
						break;

						
						case(4):
							ASYNC_CAN_SUSTAIN=1-ASYNC_CAN_SUSTAIN;
						break;
					}
					update_cycle_button_now=1;							


				}
				
			} //while
			

/*			write_eeprom_setting(NO_FREERUNNING_PING, PING_EEPROMADDR);
			write_eeprom_setting(EOR_IS_HALFRISE, HALFRISE_EEPROMADDR);
			write_eeprom_setting(EOF_IS_TAPCLKOUT, TAPCLK_EEPROMADDR);
			write_eeprom_setting(EO2_IS_TRIG, EO2_TRIG_EEPROMADDR);
			write_eeprom_setting(EO1_IS_TRIG, EO1_TRIG_EEPROMADDR);
*/
#ifndef OMITLOGCURVES

				EO1_ON;EO2_ON;CYCLE_LED_ON;PINGLEDHIGH;output_dac(0x0800);
				_delay_ms(50);

			if (EOF_IS_TAPCLKOUT) temp_u8=EEPROM_SET;else temp_u8=EEPROM_CLEAR;
			eeprom_busy_wait();
			eeprom_write_byte ((uint8_t *)(TAPCLK_EEPROMADDR),temp_u8);
			eeprom_busy_wait();

				EO1_OFF;EO2_OFF;CYCLE_LED_OFF;PINGLEDLOW;output_dac(0);
				_delay_ms(50);
				EO1_ON;EO2_ON;CYCLE_LED_ON;PINGLEDHIGH;output_dac(0x0800);
				_delay_ms(50);

			if (NO_FREERUNNING_PING) temp_u8=EEPROM_SET;else temp_u8=EEPROM_CLEAR;
			eeprom_busy_wait();
			eeprom_write_byte ((uint8_t *)(PING_EEPROMADDR),temp_u8);
			eeprom_busy_wait();

				EO1_OFF;EO2_OFF;CYCLE_LED_OFF;PINGLEDLOW;output_dac(0);
				_delay_ms(50);
				EO1_ON;EO2_ON;CYCLE_LED_ON;PINGLEDHIGH;output_dac(0x0800);
				_delay_ms(50);

			if (EOR_IS_HALFRISE) temp_u8=EEPROM_SET;else temp_u8=EEPROM_CLEAR;
			eeprom_busy_wait();
			eeprom_write_byte ((uint8_t *)(HALFRISE_EEPROMADDR),temp_u8);
			eeprom_busy_wait();

				EO1_OFF;EO2_OFF;CYCLE_LED_OFF;PINGLEDLOW;output_dac(0);
				_delay_ms(50);
				EO1_ON;EO2_ON;CYCLE_LED_ON;PINGLEDHIGH;output_dac(0x0800);
				_delay_ms(50);


		 	if (LIMIT_SKEW) temp_u8=EEPROM_SET;else temp_u8=EEPROM_CLEAR;
			eeprom_busy_wait();
			eeprom_write_byte ((uint8_t *)(LIMIT_SKEW_EEPROMADDR),temp_u8);
			eeprom_busy_wait();

				EO1_OFF;EO2_OFF;CYCLE_LED_OFF;PINGLEDLOW;output_dac(0);
				_delay_ms(50);
				EO1_ON;EO2_ON;CYCLE_LED_ON;PINGLEDHIGH;output_dac(0x0800);
				_delay_ms(50);

		 	if (EO1_IS_TRIG) temp_u8=EEPROM_SET;else temp_u8=EEPROM_CLEAR;
			eeprom_busy_wait();
			eeprom_write_byte ((uint8_t *)(EO1_TRIG_EEPROMADDR),temp_u8);
			eeprom_busy_wait();

				EO1_OFF;EO2_OFF;CYCLE_LED_OFF;PINGLEDLOW;output_dac(0);
				_delay_ms(50);
				EO1_ON;EO2_ON;CYCLE_LED_ON;PINGLEDHIGH;output_dac(0x0800);
				_delay_ms(50);

		 	if (EO2_IS_TRIG) temp_u8=EEPROM_SET;else temp_u8=EEPROM_CLEAR;
			eeprom_busy_wait();
			eeprom_write_byte ((uint8_t *)(EO2_TRIG_EEPROMADDR),temp_u8);
			eeprom_busy_wait();

				EO1_OFF;EO2_OFF;CYCLE_LED_OFF;PINGLEDLOW;output_dac(0);
				_delay_ms(50);
				EO1_ON;EO2_ON;CYCLE_LED_ON;PINGLEDHIGH;output_dac(0x0800);
				_delay_ms(50);

		 	if (!ASYNC_CAN_SUSTAIN) temp_u8=EEPROM_SET;else temp_u8=EEPROM_CLEAR;
			eeprom_busy_wait();
			eeprom_write_byte ((uint8_t *)(ASYNC_SUSTAIN_EEPROMADDR),temp_u8);
			eeprom_busy_wait();

#endif


			eof_on();
			eor_off();
			hr_off();
			tapclkout_off();
			if (CYCLE_BUT) CYCLE_LED_ON;
			else CYCLE_LED_OFF;

			entering_system_mode=0;
			cycle_down=0;
		}


	} //main loop

} //void main()


