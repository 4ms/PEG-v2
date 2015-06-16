/*
 *  dac_mcp4921.h
 *
 *  Created by dann on 3/3/11.
 *
 */


#ifndef DAC_MCP4921_H_
#define DAC_MCP4921_H_

//#define DAC_ON_TIMER (PINC & (1<<PC4))
#define DAC_ON_TIMER (0)

#define DAC_CS_PORT PORTB
#define DAC_CS_DDR DDRB
#define DAC_CS PB2

#define MCP4921_ABSEL 7
#define MCP4921_BUF 6
#define MCP4921_GAIN 5
#define MCP4921_SHDN 4

void output_dac(uint16_t data);

#endif /* DAC_MCP4921_H_ */
