/*
 * timer.h
 *
 *  Created on: Feb 16, 2011
 *      Author: admin
 */

#ifndef TIMER_H_
#define TIMER_H_

//at prescale 8 (2MHz) and TMROFSET 256-125
//1 is 65us
//long is 2^32 which is 77 hours (actually a signed long is 38 hours)

//at prescale 64 (250kHz) and TMROFSET 256-125
//1 is 520uS
//long is 2^32 which is 25.8 days

#define MINTIME 1
#define TMROFFSET 0


void inittimer(void);
uint32_t gettmrms(void);

uint32_t get_tapouttmr(void);
uint32_t get_tapintmr(void);
uint32_t get_pingtmr(void);
uint32_t get_divpingtmr(void);
uint32_t get_eo1tmr(void);
uint32_t get_eo2tmr(void);

void reset_tapouttmr(void);
void reset_tapintmr(void);
void reset_pingtmr(void);
void reset_divpingtmr(void);
void reset_eo1tmr(void);
void reset_eo2tmr(void);

#endif /* TIMER_H_ */
