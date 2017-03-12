/* The Omzlo USB controller
 * http://omzlo.com/one
 * Copyright (c) 2017 Alain Pannetrat
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <util/delay.h>
#include "usb_serial.h"
#include "mcp2515.h"
#include "can_error.h"
#include <avr/wdt.h>

#define LED_CONFIG	(DDRD |= (1<<6))
#define LED_ON		(PORTD |= (1<<6))
#define LED_OFF		(PORTD &= ~(1<<6))
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))

#define R_OK 0
#define R_ERROR (-1)

#define CAN_EFF_FLAG 0x80 /* Extended frame*/
#define CAN_RTR_FLAG 0x40 /* remote transmission request */
#define CAN_ERR_FLAG 0x20 /* error message frame */
#define CAN_CTRL_FLAG 0xE0 /* control message frame */

#define CAN_P_H0        0
#define CAN_P_H1        1
#define CAN_P_H2        2
#define CAN_P_H3        3
#define CAN_P_DLC       4
#define CAN_P_D0        5
#define CAN_P_D1        6
#define CAN_P_D2        7
#define CAN_P_D3        8
#define CAN_P_D4        9
#define CAN_P_D5        10
#define CAN_P_D6        11
#define CAN_P_D7        12

#define FN_INT_CONFIG()         (DDRD &= ~(1<<PD0))
#define FN_INT_IS_SET()         (bit_is_clear(PIND,PD0))

#ifdef OLD_PINOUT
  #define FN_IN_CONFIG()          (DDRD |= (1<<3))
  #define FN_IN_SET()             (PORTD |= (1<<3))
  #define FN_IN_CLEAR()           (PORTD &= ~(1<<3))
#else
  #define FN_IN_CONFIG()          (DDRE |= (1<<6))
  #define FN_IN_SET()             (PORTE |= (1<<6))
  #define FN_IN_CLEAR()           (PORTE &= ~(1<<6))
#endif

#define FN_LED_ORANGE_CONFIG()  (DDRB |= (1<<7))
#define FN_LED_ORANGE_SET()     (PORTB |= (1<<7))
#define FN_LED_ORANGE_CLEAR()   (PORTB &= ~(1<<7))

#define FN_LED_RED_CONFIG()     (DDRB |= (1<<6))
#define FN_LED_RED_SET()        (PORTB |= (1<<6))
#define FN_LED_RED_CLEAR()      (PORTB &= ~(1<<6))

#define FN_DEN_CONFIG()         (DDRB |= (1<<5))
#define FN_DEN_SET()            (PORTB |= (1<<5))
#define FN_DEN_CLEAR()          (PORTB &= ~(1<<5))

#define FN_CAN_R_CONFIG()         (DDRB |= (1<<4))
#define FN_CAN_R_SET()            (PORTB |= (1<<4))
#define FN_CAN_R_CLEAR()          (PORTB &= ~(1<<4))



#define CHANNEL_SENSE   0x00
#define CHANNEL_VIN     0x01
#define CHANNEL_VREF    0x1E 


#define POWER_FLAGS_SUPPLY 0x01
#define POWER_FLAGS_SENSE  0x02
#define POWER_FLAGS_FAULT  0x04

uint8_t get_byte(uint8_t *dst)
{
	int16_t r = usb_serial_getchar();
    
    if (r == -1) 
        return R_ERROR;
    
    *dst = r&0xFF;
    return R_OK;
}

void put_byte(int8_t byte)
{
    usb_serial_putchar_nowait(byte);
}

void put_byte_n(const uint8_t *data, uint8_t len)
{
    //int i;
    //for (i=0;i<len;i++) usb_serial_putchar(data[i]);
    usb_serial_write(data,len);
}

int8_t can_recv(uint8_t *dest)
{
    uint8_t val = mcp2515_rx_status();
    uint8_t base,clear,i;
    uint8_t a0,a1,a2,a3,dlc;

    if (bit_is_set(val, RX_STATUS_RXB0_READY))
    {
        base = RXB0;
        clear = RX0IF;
    } 
    else if (bit_is_set(val, RX_STATUS_RXB1_READY))
    {
        base = RXB1;
        clear = RX1IF;
    }
    else 
    {
        return R_ERROR;
    }

    mcp2515_slave_select();
    spi_transmit(MCP_READ);
    spi_transmit(base+xSIDH);
    a0 = spi_transmit(0);
    a1 = spi_transmit(0);
    a2 = spi_transmit(0);
    a3 = spi_transmit(0);
    dlc= spi_transmit(0);
    for (i=5;i<13;i++)
        dest[i] = spi_transmit(0);
    mcp2515_slave_release();
 
    mcp2515_register_write_bit(CANINTF,clear,0);

    if ((a1&0x08)!=0)
    {   // extended frame
        dest[0] = (a0 >> 3) | CAN_EFF_FLAG;
        dest[1] = (a0 << 5) | ((a1&0xE0)>>3) | (a1&0x03);
        dest[2] = a2;
        dest[3] = a3;
    }
    else // standard frame
    {
        dest[0] = 0;
        dest[1] = 0;
        dest[2] = (a0>>5);
        dest[3] = (a0<<3) | (a1>>5);
    }
    // the RTR_FLAG is in the dlc byte, so we just mask it out and add it
    dest[0] |= (dlc & CAN_RTR_FLAG);
    dest[4] = dlc&0x0F;
    //if (dest[4]>8) dest[4]=8;

    return R_OK;
}

int8_t can_send(const uint8_t *src)
{
    uint8_t val,base,i,a0,a1,a2,a3,dlc;

    val = mcp2515_status();

    if (bit_is_clear(val,STATUS_TXB0CNTRL_TXREQ))
    {
        base = TXB0;
    } 
    else if (bit_is_clear(val, STATUS_TXB1CNTRL_TXREQ))
    {
        base = TXB1;
    }
    /*
       else if (bit_is_clear(val, STATUS_TXB2CNTRL_TXREQ))
       {
       base = TXB2;
       }
       */
    else
    {    
        return R_ERROR;
    }

    if ((*src&CAN_EFF_FLAG)!=0)
    {   // extended frame
        a0 = (src[0]<<3) | (src[1]>>5);
        a1 = ((src[1]&0x1C)<<3) | 0x08 | (src[1]&0x03);
        a2 = src[2];
        a3 = src[3];
    }
    else
    {   // standard frame
        a0 = (src[2]<<3) | (src[3]>>5);
        a1 = src[3] << 5;
        a2 = 0;
        a3 = 0;
    }
    dlc = (src[0]&CAN_RTR_FLAG) | (src[4]&0x0F);
    
    src += 5;

    mcp2515_slave_select();
    spi_transmit(MCP_WRITE);
    spi_transmit(base+xSIDH);
    spi_transmit(a0);
    spi_transmit(a1);
    spi_transmit(a2);
    spi_transmit(a3);
    spi_transmit(dlc);
    for (i=0;i<8;i++)
        spi_transmit(*src++);
    mcp2515_slave_release();

    mcp2515_register_write_bit(base+xCTRL,TXREQ,1);

    return R_OK;
}

int8_t can_init(void)
{
    spi_master_init();

    mcp2515_slave_setup();

    mcp2515_slave_select();
    spi_transmit(MCP_RESET);
    mcp2515_slave_release();

    _delay_ms(100);

    if ((mcp2515_register_read(CANSTAT)&0xE0)!=0x80) return R_ERROR;    // check if we are in config mode
   
    mcp2515_register_write(CNF1,0x03);
    mcp2515_register_write(CNF2,0xF0);
    mcp2515_register_write(CNF3,0x86);

    // set RXB0CTRL.BUKT to 1 for automatic overflow
    mcp2515_register_write(RXB0+xCTRL, (1<<RXM0) | (1<<RXM1) | (1<<BUKT)); // Buffer 0: receive everything + rollover to next buffer
    mcp2515_register_write(RXB1+xCTRL, (1<<RXM0) | (1<<RXM1));             // Buffer 1: receive everything

    // set CANINTE.RX0IE to 1, interrupt enable for RX0
    mcp2515_register_write_bit(CANINTE, RX0IE, 1);

    // set CANINTE.RX1IE to 1, interrupt enable for RX1
    mcp2515_register_write_bit(CANINTE, RX1IE, 1);

    mcp2515_register_write(CANCTRL,0x00);

    _delay_ms(10);

    if ((mcp2515_register_read(CANSTAT)&0xE0)!=0x00) return R_ERROR;

    return R_OK; 
}

uint16_t adc_read(uint8_t channel)
{
    uint8_t low;

    ADMUX = (1 << REFS0)|(channel&0x1F); // set channel in low 5 bits
    //ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1<<ADEN);                 // start reading
    ADCSRB = (1<<ADHSM) | (channel & 0x20);
    //if (channel==CHANNEL_VREF)
        _delay_ms(1);
    ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1<<ADEN) | (1<<ADSC);                 // start reading
    while (bit_is_set(ADCSRA,ADSC));          // wait until done
    low = ADCL;
    //result |= ((uint16_t)(ADCH))<<8;
    return (ADCH<<8)|low;             // return result
}

static int power_up(void)
{
    FN_LED_RED_SET();
    FN_IN_SET();    // Enable switch
        
    return 0;
}

static int power_down(void)
{
    FN_LED_RED_CLEAR();
    FN_IN_CLEAR();  // Disable switch
    
    return 0;
}

static int power_check(uint8_t* power_flags, uint16_t *supply_level, uint16_t *sense_level, uint16_t *vref)
{
    // We test the power supply of the board
    // First we look if it is connected to a power supply at all
    // Next we enable the switch and check the status of the switch

    *power_flags &= POWER_FLAGS_FAULT;

    *vref = adc_read(CHANNEL_VREF);

    FN_DEN_SET();
    *sense_level = adc_read(CHANNEL_SENSE);
    FN_DEN_CLEAR();

    if (*sense_level>1000)
    {
        *power_flags |= POWER_FLAGS_FAULT;
        power_down();
    } 
    
    if (*sense_level>0)
    {
        *power_flags |= POWER_FLAGS_SENSE;
    }


    *supply_level = adc_read(CHANNEL_VIN);
    if (*supply_level>=32)
    {
        *power_flags |= POWER_FLAGS_SUPPLY;
        FN_LED_ORANGE_SET();
    }
    else
    {
        FN_LED_ORANGE_CLEAR();
    }
    return 0;
}

#define NOCAN_HEADER_PACKET                 0x0F
#define NOCAN_HEADER_SUCCESS                0x10
#define NOCAN_HEADER_REQUEST_SOFT_RESET     0x20
#define NOCAN_HEADER_REQUEST_HARD_RESET     0x30
#define NOCAN_HEADER_REQUEST_POWER_STATUS   0x40
#define NOCAN_HEADER_SET_POWER              0x51
#define NOCAN_HEADER_SET_CAN_RES            0x61
#define NOCAN_HEADER_REQUEST_DEBUG          0x72
#define NOCAN_HEADER_VERSION                0x80
#define NOCAN_HEADER_FAIL                   0xE0
#define NOCAN_HEADER_COMMAND_UNKNOWN        0xF0

#include <avr/interrupt.h>
#define SLOTS 16
uint8_t circ_buffer[16*SLOTS];      // 00 TS TS F00 .. F12
volatile uint8_t slot_head = 0;
volatile uint8_t slot_tail = 0;
//volatile uint8_t slot_head_offset = 0;
uint16_t packet_id = 0;

ISR(INT0_vect) {
    uint8_t slot_next = (slot_tail+1)&(SLOTS-1);
    uint8_t *buf = circ_buffer+(slot_tail<<4);
    
    buf[0] = NOCAN_HEADER_PACKET;
    buf[1] = (packet_id>>8);
    buf[2] = (packet_id&0xFF);
    can_recv(buf+3);
    packet_id++;

    if (slot_next!=slot_head) {
        slot_tail = slot_next;
    } 
}

uint8_t *circ_buffer_pos(void)
{
    if (slot_head!=slot_tail) 
        return circ_buffer+slot_head*16;
    return 0;
}

void circ_buffer_advance(void)
{
    if (slot_head!=slot_tail) 
    {
        cli();
        slot_head = (slot_head+1)&(SLOTS-1);
        sei();
    }
}

void circ_buffer_clear(void)
{
    cli();
    slot_head = 0;
    slot_tail = 0;
    sei();
}

// Basic command interpreter for controlling port pins
int main(void)
{
    uint8_t i;
    uint8_t can_buffer[16];
    uint8_t can_info[16];
    uint8_t *packet;
    uint8_t vin_flags = 0;
	uint16_t vin_sense_level = 0;
    uint16_t vin_supply_level = 0;
    uint16_t vref_1_1 = 0;

    // in case that's what got us here
    MCUSR = 0;
    wdt_disable();

    // set for 16 MHz clock
	CPU_PRESCALE(0);

    // ADC config
    // Reference is AVCC with external capacitor on AREF pin
    ADMUX = (1 << REFS0);
    // prescaler = 128 -> 16Mhz/128 -> 125Khz
    // Freq must be between 50Khz and 200Khz
    ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1<<ADEN); 

    FN_INT_CONFIG();    // PD0 is input (IN PIN)
    FN_IN_CONFIG();     // this is the on/off switch for VIN (e.g. 12v)
    FN_DEN_CONFIG();    // Diagnosis enable switch
    FN_LED_ORANGE_CONFIG();
    FN_LED_RED_CONFIG();
    FN_CAN_R_CONFIG();
    FN_CAN_R_CLEAR();           // ENABLE CAN RES BY PUTTING MOSFET TO 0 WHICH IS THE DEFAULT
   
    // use a timer to clock checking the switch status
    TCCR0A = 0; // normal mode of operation timer0
    TCCR0B = (1<<CS00); // divide by 1024 (the timer will overflow in approx 16ms at 16Mhz)
 

    EIMSK |= (1 << INT0);
    // EICRA &= ~((1 << ISC01)|(1 << ISC00)); // this is the default, no need to set

	// initialize the USB, and then wait for the host
	// to set configuration.  If the device is powered
	// without a PC connected to the USB port, this 
	// will wait forever.

	usb_init();
	while (!usb_configured()) /* wait */ ;
    for (i=0;i<5;i++)
    {
        FN_LED_ORANGE_SET();
	    _delay_ms(100);
        FN_LED_ORANGE_CLEAR();
        FN_LED_RED_SET();
	    _delay_ms(100);
        FN_LED_RED_CLEAR();
    }

    
    // We now test the power supply of the board
    power_up();
    power_check(&vin_flags, &vin_supply_level, &vin_sense_level,&vref_1_1);

	while (1) {
		usb_serial_flush_input();

        can_init();

		while (1) {
            // If there are packets available, send them to the host
            while ((packet = circ_buffer_pos())!=0) 
            {
                put_byte_n(packet,16);
                circ_buffer_advance();
            }
            // TODO: we should flush here?

            // if the host wants to send something, get it
            if (usb_serial_available()>0) {
                if (get_byte(can_buffer)==R_ERROR) break; // panic!!!
                for (i=0;i<(can_buffer[0]&0xF);i++)
                {    
                    get_byte(can_buffer+i+1);
                } 

                switch (can_buffer[0]) {
                    case NOCAN_HEADER_PACKET:
                        if (can_send(can_buffer+3)!=R_OK)
                            put_byte(NOCAN_HEADER_FAIL);
                        else
                            put_byte(NOCAN_HEADER_SUCCESS);
                        break;
                    case NOCAN_HEADER_REQUEST_SOFT_RESET:
                        if (can_init()!=R_OK)
                            put_byte(NOCAN_HEADER_FAIL);
                        else
                            put_byte(NOCAN_HEADER_SUCCESS);
                        break;
                    case NOCAN_HEADER_REQUEST_HARD_RESET:
                        put_byte(NOCAN_HEADER_SUCCESS);
                        // TODO: flush output
                        wdt_enable(WDTO_15MS);
                        for (;;);
                    case NOCAN_HEADER_REQUEST_POWER_STATUS:
                        can_info[0] = NOCAN_HEADER_SUCCESS | 7;
                        can_info[1] = vin_flags;
                        can_info[2] = (vin_supply_level >> 8);
                        can_info[3] = (vin_supply_level & 0xFF);
                        can_info[4] = (vin_sense_level >> 8);
                        can_info[5] = (vin_sense_level & 0xFF);
                        can_info[6] = (vref_1_1 >> 8);
                        can_info[7] = (vref_1_1 & 0xFF);
                        put_byte_n(can_info,8);
                        break;
                    case NOCAN_HEADER_SET_POWER:
                        vin_flags &= ~POWER_FLAGS_FAULT; // reset fault line
                        if (can_buffer[1]!=0)
                        {
                            power_up();
                        }
                        else
                        {
                            power_down();
                        }
                        put_byte(NOCAN_HEADER_SUCCESS);
                        break;
                    case NOCAN_HEADER_SET_CAN_RES:
                        if (can_buffer[1]!=0)
                            FN_CAN_R_SET();             // DISABLE CAN RES. (inverted logic)
                        else
                            FN_CAN_R_CLEAR();           // ENABLE CAN RES.
                        put_byte(NOCAN_HEADER_SUCCESS);
                        break;
                    case NOCAN_HEADER_REQUEST_DEBUG:
                        can_buffer[2] &= 0xF;
                        can_info[0] = (NOCAN_HEADER_REQUEST_DEBUG&0xF0)|can_buffer[2];
                        for (i=0;i<can_buffer[2];i++)
                          can_info[1+i] = mcp2515_register_read(can_buffer[1]+i);
                        put_byte_n(can_info,can_buffer[2]+1);
                        break;
                    case NOCAN_HEADER_VERSION:
                        can_info[0] = NOCAN_HEADER_SUCCESS | 4;
                        can_info[2] = 'C';
                        can_info[2] = 'A';
                        can_info[2] = 'N';
                        can_info[2] = '0';
                        put_byte_n(can_info,5);
                        break;
                    default:
                        put_byte(NOCAN_HEADER_COMMAND_UNKNOWN);
                }
                continue;
            } //  if (usb_serial_available() ...
            
            if (bit_is_set(TIFR0,TOV0))
            {
                power_check(&vin_flags, &vin_supply_level, &vin_sense_level,&vref_1_1);
                TCNT0 = 0;
                TIFR0 |= (1<<TOV0); // clear interrupt flag by putting 1 in place
            }
        }
    }
}


