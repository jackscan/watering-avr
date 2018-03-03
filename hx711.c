#include "hx711.h"

#include <avr/io.h>

#include <util/delay.h>

#include <stdio.h>
#include <avr/sleep.h>

#define SERIAL_DDR  DDRD
#define SERIAL_PORT PORTD
#define SERIAL_PIN  PIND
#define PD_SCK      (1 << PD6)
#define DOUT        (1 << PD7)

void hx711_init(void) {
    SERIAL_DDR |= PD_SCK;
    SERIAL_DDR &= ~DOUT;

    // no pull up on DOUT
    SERIAL_PORT &= ~DOUT;

    hx711_powerdown();

    // enable digital input DOUT
    if (SERIAL_DDR == DDRD && (DOUT & ((1 << PD7) | (1 << PD6))) != 0) {
        DIDR1 &= ~((DOUT & ((1 << PD7) | (1 << PD6))) >> PD6);
    }
}

void hx711_powerdown(void)
{
    SERIAL_PORT |= PD_SCK;
    _delay_us(60);
}

uint16_t hx711_read(void) {
    // power up
    SERIAL_PORT &= ~PD_SCK;

    // wait for hx711 to become ready
    while ((SERIAL_PIN & DOUT) != 0)
        ;

    _delay_us(0.1);

    uint16_t ret = 0;
    uint8_t i;
    // we are only interested in the upper 14 bits
    for (i = 0; i < 14; ++i) {
        SERIAL_PORT |= PD_SCK;
        _delay_us(0.2);

        ret <<= 1;

        if ((SERIAL_PIN & DOUT) != 0)
            ret |=  1;

        // flip first bit for unsigned value
        if (i == 0)
            ret ^= 1;

        SERIAL_PORT &= ~PD_SCK;
        _delay_us(0.2);
    }

    // pulse 25 times for input A with gain 128
    for (; i < 25; ++i) {
        SERIAL_PORT |= PD_SCK;
        _delay_us(0.2);
        SERIAL_PORT &= ~PD_SCK;
        _delay_us(0.2);
    }

    return ret;
}
