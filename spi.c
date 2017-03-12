#include "spi.h"

void spi_master_init(void)
{
    SPI_DDR &= ~_BV(SPI_MISO); 
    SPI_DDR |= _BV(SPI_MOSI)|_BV(SPI_SCK); // set MOSI and SCK as output
    SPCR = _BV(SPE)|_BV(MSTR)|_BV(SPR0); /* enable spi as master, speed = fosc/16 */
}

uint8_t spi_transmit(uint8_t c)
{
    SPDR = c;
    while (!(SPSR & _BV(SPIF)));
    return SPDR;
}

