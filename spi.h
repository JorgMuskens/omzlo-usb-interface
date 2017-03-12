#ifndef _SPI_H_
#define _SPI_H_

#include <avr/io.h>

// NOTE: SS can be on any pin of SPI_PORT, and there could be more than one.
#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__)
    #define SPI_DDR     DDRB
    #define SPI_PORT    PORTB
    #define SPI_MOSI    PB3
    #define SPI_MISO    PB4
    #define SPI_SCK     PB5
    #define SPI_SS      PB2
#elif defined(__AVR_ATmega32U4__)
    #define SPI_DDR     DDRB
    #define SPI_PORT    PORTB
    #define SPI_MOSI    PB2
    #define SPI_MISO    PB3
    #define SPI_SCK     PB1
    #define SPI_SS      PB0     
#else
    #error "Missing cpu definition for SPI protocol."
#endif

//#define SPI_SLAVE_CONFIGURE() (SPI_DDR |= _BV(SPI_SS))

//#define spi_slave_select() (SPI_PORT &= ~_BV(SPI_SS))
//#define spi_slave_release() (SPI_PORT |= _BV(SPI_SS))

void spi_master_init(void);
uint8_t spi_transmit(uint8_t c);

#endif
