#include "hx711.h"

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>

#include <util/delay.h>

#include <stdio.h>

#define SERIAL_DDR  DDRD
#define SERIAL_PORT PORTD
#define SERIAL_PIN  PIND
#define PD_SCK      (1 << PD6)
#define DOUT        (1 << PD7)

static uint8_t s_current = 0;
static uint8_t s_accuracy = 0;
static struct scale_calib s_calib_data[2];

static void set_channel_gain(enum hx711_chan_gain chan_gain) {
    switch (chan_gain) {
    case HX711_CHANNEL_A_128:
        s_accuracy = 0;
        s_current = 0;
        break;
    case HX711_CHANNEL_B_32:
        s_accuracy = 2;
        s_current = 1;
        break;
    case HX711_CHANNEL_A_64:
        s_accuracy = 1;
        s_current = 0;
        break;
    }
}

void hx711_init(const void *calib) {
    SERIAL_DDR |= PD_SCK;
    SERIAL_DDR &= ~DOUT;

    // no pull up on DOUT
    SERIAL_PORT &= ~DOUT;

    hx711_powerdown();

    // enable digital input DOUT
    if (SERIAL_DDR == DDRD && (DOUT & ((1 << PD7) | (1 << PD6))) != 0) {
        DIDR1 &= ~((DOUT & ((1 << PD7) | (1 << PD6))) >> PD6);
    }

    eeprom_read_block(s_calib_data, calib, sizeof s_calib_data);
}

void hx711_powerdown(void)
{
    SERIAL_PORT |= PD_SCK;
    _delay_us(60);
    set_channel_gain(HX711_CHANNEL_A_128);
}

void hx711_calib(uint8_t index, uint32_t offset, uint32_t scale) {
    s_calib_data[index].offset = offset;
    s_calib_data[index].scale = scale;
}

void hx711_write_calib(void *calib) {
    eeprom_write_block(s_calib_data, calib, sizeof s_calib_data);
}

uint32_t hx711_read32(void) {
    // power up
    SERIAL_PORT &= ~PD_SCK;

    // wait for hx711 to become ready
    while ((SERIAL_PIN & DOUT) != 0)
        ;

    _delay_us(0.1);

    uint32_t result = 0;
    uint8_t i;
    // we are only interested in the 17 upper bits
    for (i = 0; i < 17 + s_accuracy; ++i) {
        SERIAL_PORT |= PD_SCK;
        _delay_us(0.2);

        result <<= 1;

        if ((SERIAL_PIN & DOUT) != 0)
            result |=  1;

        // flip first bit for unsigned value
        if (i == 0)
            result ^= 1;

        SERIAL_PORT &= ~PD_SCK;
        _delay_us(0.2);
    }

    // pulse 25 times
    for (; i < 25; ++i) {
        SERIAL_PORT |= PD_SCK;
        _delay_us(0.2);
        SERIAL_PORT &= ~PD_SCK;
        _delay_us(0.2);
    }

    struct scale_calib *cal = s_calib_data + s_current;
    if (cal->offset != 0xFFFFFFFF && cal->scale != 0xFFFFFFFF) {
        result -= cal->offset << s_accuracy;
        result *= cal->scale;
        result /= 256UL;
    } else {
        result >>= 3 + s_accuracy;
    }

    return result;
}

uint16_t hx711_read(void) {
    return (uint16_t)hx711_read32();
}

void hx711_finish_read(enum hx711_chan_gain chan_gain) {
    // pulse for channel-gain selection
    for (uint8_t i = 0; i < (uint8_t)chan_gain; ++i) {
        SERIAL_PORT |= PD_SCK;
        _delay_us(0.2);
        SERIAL_PORT &= ~PD_SCK;
        _delay_us(0.2);
    }

    set_channel_gain(chan_gain);
}
