#include "spi.h"
#include "mcp2515.h"
#include <util/delay.h>


void mcp2515_register_write(uint8_t reg, uint8_t val)
{
    mcp2515_slave_select();
    spi_transmit(MCP_WRITE);
    spi_transmit(reg);
    spi_transmit(val);
    mcp2515_slave_release();
}

void mcp2515_register_write_bit(uint8_t reg, uint8_t bit, uint8_t val)
{
    mcp2515_slave_select();
    spi_transmit(MCP_BIT_MODIFY);
    spi_transmit(reg);
    spi_transmit(1<<bit);
    spi_transmit(val?0xff:0x00);
    mcp2515_slave_release();
}

uint8_t mcp2515_register_read(uint8_t reg)
{
    mcp2515_slave_select();
    spi_transmit(MCP_READ);
    spi_transmit(reg);
    uint8_t retval = spi_transmit(0);
    mcp2515_slave_release();
    return retval;
}

uint8_t mcp2515_rx_status(void)
{
    mcp2515_slave_select();
    spi_transmit(MCP_RX_STATUS);
    uint8_t retval = spi_transmit(0);
    mcp2515_slave_release();
    return retval;
}

uint8_t mcp2515_status(void)
{
    mcp2515_slave_select();
    spi_transmit(MCP_READ_STATUS);
    uint16_t retval = spi_transmit(0);
    mcp2515_slave_release();
    return retval;
}


