#ifndef _MCP2515_H_
#define _MCP2515_H_

#include <avr/io.h>

#include "spi.h"
/* this is the tricky part:
 * On the AVR, in master mode, the SS pin must either:
 * 1) be configured as input and held high.
 * 2) be configured as output only.
 *
 * As a consequence we use it as our slave select PIN for MCP2515, enabling it as output.
 * Any other pin could have been used, but it's just as simple like this.
 */ 

#define mcp2515_slave_setup() (SPI_DDR |= _BV(SPI_SS))
#define mcp2515_slave_select() (SPI_PORT &= ~_BV(SPI_SS))
#define mcp2515_slave_release()  (SPI_PORT |= _BV(SPI_SS))




#define mcp2515_interrupt_status() mcp2515_register_read(CANINTF)

uint8_t mcp2515_status(void);

uint8_t mcp2515_rx_status(void);

void mcp2515_register_write(uint8_t reg, uint8_t val);

void mcp2515_register_write_bit(uint8_t reg, uint8_t bit, uint8_t val);

uint8_t mcp2515_register_read(uint8_t reg);

int mcp2515_init(void);

int mcp2515_normal_mode(void);

#define MSG_WAIT_MORE 3

#define MCP_OK 0
#define MCP_FAIL -1

// MCP2515 registers using names in the datasheet
#define RXF0SIDH 0x00
#define RXF0SIDL 0x01
#define RXF0EID8 0x02
#define RXF0EID0 0x03
#define RXF1SIDH 0x04
#define RXF1SIDL 0x05
#define RXF1EID8 0x06
#define RXF1EID0 0x07
#define RXF2SIDH 0x08
#define RXF2SIDL 0x09
#define RXF2EID8 0x0A
#define RXF2EID0 0x0B
#define BFPCTRL 0x0C
#define TXRTSCTRL 0x0D
#define CANSTAT 0x0E
#define CANCTRL 0x0F
#define RXF3SIDH 0x10
#define RXF3SIDL 0x11
#define RXF3EID8 0x12
#define RXF3EID0 0x13
#define RXF4SIDH 0x14
#define RXF4SIDL 0x15
#define RXF4EID8 0x16
#define RXF4EID0 0x17
#define RXF5SIDH 0x18
#define RXF5SIDL 0x19
#define RXF5EID8 0x1A
#define RXF5EID0 0x1B
#define TEC 0x1C
#define REC 0x1D
#define RXM0SIDH 0x20
#define RXM0SIDL 0x21
#define RXM0EID8 0x22
#define RXM0EID0 0x23
#define RXM1SIDH 0x24
#define RXM1SIDL 0x25
#define RXM1EID8 0x26
#define RXM1EID0 0x27
#define CNF3 0x28
#define CNF2 0x29
#define CNF1 0x2A
#define CANINTE 0x2B
  #define MERRE 7
  #define WAKIE 6
  #define ERRIE 5
  #define TX2IE 4
  #define TX1IE 3
  #define TX0IE 2
  #define RX1IE 1
  #define RX0IE 0
#define EFLG 0x2D

//#define TXB0CTRL 0x30
  #define TXREQ 3
//#define TXB0SIDH 0x31
//#define TXB0SIDL 0x32
  #define EXIDE 3
//#define TXB0EID8 0x33
//#define TXB0EID0 0x34
//#define TXB0DLC 0x35
  #define TXRTR 6
//#define TXB0D0 0x36 

#define TXB0 0x30
#define TXB1 0x40
#define TXB2 0x50
#define RXB0 0x60
#define RXB1 0x70

#define xCTRL 0x0
  #define RXM1 6
  #define RXM0 5
  #define RXRTR 3
  #define BUKT 2
  //Bits 2:0 FILHIT2:0
#define xSIDH 0x1
#define xSIDL 0x2
#define xEID8 0x3
#define xEID0 0x4
#define xDLC 0x5
#define xD0 0x6


#define CANINTF 0x2C
  #define MERRF 7
  #define WAKIF 6
  #define ERRIF 5
  #define TX2IF 4
  #define TX1IF 3
  #define TX0IF 2
  #define RX1IF 1
  #define RX0IF 0

// see  mcp2515_status:
#define STATUS_CANINTF_RX0IF    0
#define STATUS_CANINTF_RX1IF    1
#define STATUS_TXB0CNTRL_TXREQ  2
#define STATUS_CANINTF_TX0IF    3
#define STATUS_TXB1CNTRL_TXREQ  4
#define STATUS_CANINTF_TX1IF    5
#define STATUS_TXB2CNTRL_TXREQ  6
#define STATUS_CANINTF_TX2IF    7

// see mcp_rx_status:
#define RX_STATUS_RXB0_READY    6
#define RX_STATUS_RXB1_READY    7


#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_READ_STATUS 0xA0
#define MCP_RX_STATUS   0xB0
#define MCP_BIT_MODIFY  0x05

#endif 
