/*
 *  dac_mcp4921.c
 *
 *  Created by dann on 3/3/11.
 *
 */
#include <avr/io.h>
#include "dac_mcp4921.h"


void output_dac(uint16_t data){
	DAC_CS_PORT &= ~(1<<DAC_CS);	//pull CS low to enable DAC

//using a 4921, we only have one channel, so set _ABSEL=0
//with a 4922 our function is output_dac(char channel, uint16_t data){
//...and then do SPDR = (channel<<MCP4921_ABSEL)....
	SPDR = (0<<MCP4921_ABSEL) | (1<<MCP4921_BUF) | (1<<MCP4921_GAIN) | (1<<MCP4921_SHDN) | ((data>>8) & 0x0F);
	while (!(SPSR & (1<<SPIF)))
		;
	SPDR = data & 0x00FF;
	while (!(SPSR & (1<<SPIF)))
		;
	DAC_CS_PORT |= (1<<DAC_CS);		//pull CS high to latch data

}


