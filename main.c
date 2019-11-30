/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include "debug.h"
#include "timer.h"
#include "twi-slave.h"
#include "water.h"
#include "hx711.h"

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include <util/delay.h>

#include <stdlib.h>
#include <stdio.h>

#define CAPLOAD_DDR DDRD
#define CAPLOAD_PORT PORTD
#define CAPLOAD_BIT (1 << PD4)

#define CAPSENS_DDR DDRB
#define CAPSENS_PORT PORTB
#define CAPSENS_BIT (1 << PB0)

#define WATERING1_DDR DDRD
#define WATERING1_PORT PORTD
#define WATERING1_BIT (1 << PD3)

#define WATERING2_DDR DDRC
#define WATERING2_PORT PORTC
#define WATERING2_BIT (1 << PC3)

static struct {
    volatile uint8_t *ddr;
    volatile uint8_t *port;
    uint8_t bit;
} watering_pin[2] = {
    {.ddr = &WATERING1_DDR, .port = &WATERING1_PORT, .bit = WATERING1_BIT},
    {.ddr = &WATERING2_DDR, .port = &WATERING2_PORT, .bit = WATERING2_BIT},
};

static EEMEM struct scale_calib eemem_scale_calib[2];

static uint8_t early_init(void) {
    // save reset reason
    uint8_t mcusr = MCUSR;
    // clear reset flags for next reset
    MCUSR = 0;

    cli();

    wdt_disable();

    // disable AIN1 and AIN0
    DIDR1 = 0x03;

    // disable digital inputs on ADC pins
    // except for ADC5 and ADC4 (needed for TWI)
    DIDR0 = 0xFF & ~((1 << ADC4D) | (1 << ADC5D));

    // disable analog comparator
    ACSR |= (1 << ACD);

    // set pins to output
    DDRB = (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4) | (1 << DDB5);
    DDRC = (1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3);
    DDRD = (1 << DDD2) | (1 << DDD3) | (1 << DDD6) | (1 << DDD7);
    // drive pins low
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;

    hx711_init(eemem_scale_calib);

    // disable all peripheral clocks
    PRR = 0xFF;

    return mcusr;
}

static void measure_weight(void) {
    uint32_t t0 = get_time();

    while (true) {
        uint16_t w = hx711_read();
        if (twi_get_cmd() != CMD_CONS(CMD_GET_WEIGHT, 0) ||
            get_time() - t0 >= TIMER_MS(1000)) {
            hx711_finish_read(HX711_CHANNEL_B_32);
            break;
        } else {
            hx711_finish_read(HX711_CHANNEL_A_128);
        }
        twi_add_weight(0, w);
    }

    uint32_t t1 = get_time();
    uint32_t w1 = twi_get_weight(0);

    while (!twi_cmd_pending() && get_time() - t1 < TIMER_MS(1000))
        ;

    if (twi_cmd_pending() && twi_get_cmd() == CMD_CONS(CMD_GET_WEIGHT, 1)) {
        // TODO: fix possible race here, cmd could have changed and
        // twi_next_cmd() resets pending flag
        twi_next_cmd();
        while (twi_get_cmd() == CMD_CONS(CMD_GET_WEIGHT, 1) &&
                get_time() - t1 < TIMER_MS(1000)) {
            uint16_t w = hx711_read();
            hx711_finish_read(HX711_CHANNEL_B_32);
            twi_add_weight(1, w);
        }
    }

    uint32_t t2 = get_time();
    uint32_t w2 = twi_get_weight(1);

    hx711_powerdown();

    printf("weight measuring: %lu, %lu\n", t1 - t0, t2 - t1);

    uint32_t n1 = (w1 >> 16);
    uint32_t d1 = (w1 & 0xFFFF);
    uint32_t n2 = (w2 >> 16);
    uint32_t d2 = (w2 & 0xFFFF);
    printf("weight: %u / %u = %u, %u / %u = %u\n",
           (uint16_t)n1, (uint16_t)d1, (uint16_t)((n1 + d1 / 2) / d1),
           (uint16_t)n2, (uint16_t)d2, (uint16_t)((n2 + d2 / 2) / d2));
}

static void measure_weight_2(void) {
    printf("w1:\n");
    uint8_t i = 0;
    const uint8_t count = 16;
    uint32_t sum1 = 0;
    do {
        uint32_t w = hx711_read32();
        if (++i < count) {
            hx711_finish_read(HX711_CHANNEL_A_128);
        } else {
            hx711_finish_read(HX711_CHANNEL_B_32);
        }
        printf("%lu\n", w);
        sum1 += w;
    } while (i < count);
    printf("> %lu\n", (sum1 + count / 2) / count);

    printf("w2:\n");
    uint32_t sum2 = 0;
    for (uint8_t i = 0; i < count; ++i) {
        uint32_t w = hx711_read32();
        hx711_finish_read(HX711_CHANNEL_B_32);
        printf("%lu\n", w);
        sum2 += w;
    }
    hx711_powerdown();
    printf("> %lu\n", (sum2 + count / 2) / count);
}

static __attribute__ ((section (".noinit"))) struct {
    uint32_t balance;
    uint32_t last;
    uint32_t fract;
} g_account[2];

uint8_t water_limit(uint8_t index, uint8_t t) {
    uint32_t curr = get_time();

    uint8_t sreg = SREG;
    cli();
    uint32_t dt = (curr - g_account[index].last + g_account[index].fract);
    uint32_t da = dt / WATER_TIME_REFILL_INTERVAL;
    uint32_t na = da + g_account[index].balance;

    g_account[index].balance = na < MAX_WATER_TIME ? na : MAX_WATER_TIME;
    g_account[index].fract = dt - da * WATER_TIME_REFILL_INTERVAL;
    g_account[index].last = curr;
    SREG = sreg;

    if (t > g_account[index].balance)
        t = g_account[index].balance;

    return t;
}

static void update_water_account(uint8_t index, uint8_t t) {
    g_account[index].balance =
        (t < g_account[index].balance) ? g_account[index].balance - t : 0;
}

static uint8_t water(uint8_t index, uint8_t t) {
    t = water_limit(index, t);

    printf("watering %u %ums\n", index, (uint16_t)t * 250u);

    wdt_enable(WDTO_500MS);
    uint32_t t0 = get_time();
    *watering_pin[index].port |= watering_pin[index].bit;
    for (uint8_t i = 0; i < t; ++i) {
        _delay_ms(250);
        wdt_reset();
    }
    uint32_t t1 = get_time();
    *watering_pin[index].port &= ~watering_pin[index].bit;
    wdt_disable();

    uint32_t f = (F_CPU / 4UL);
    uint32_t dt = ((t1 - t0) * TIMER_PS + f/2) / f;
    printf("watered %lums\n", dt * 250);

    update_water_account(index, dt);

    return (uint8_t)dt;
}

static void measure_timer(void) {
    printf("measure timer\n");
    uint32_t t0 = get_time();
    _delay_ms(1000);
    uint32_t t1 = get_time();
    printf("%lu - %lu = %lu\n", t1, t0, t1 - t0);
    printf("expected: %lu\n", TIMER_MS(1000));
}

static void print_calib(void) {
    struct scale_calib *c = hx711_get_calib();
    printf("calib1: %lu %lu\n", c[0].offset, c[0].scale);
    printf("calib2: %lu %lu\n", c[1].offset, c[1].scale);
}

int main(void) {

    uint8_t mcusr = early_init();

    debug_init();

    if ((mcusr & (EXTRF | WDRF)) != 0) {
        debug_dump_trace();
    }
    debug_init_trace();

    timer_start();
    twi_slave_init(0x10);

    if ((mcusr & (PORF | BORF)) != 0) {
        for (uint8_t i = 0; i < sizeof(g_account) / sizeof(g_account[0]); ++i) {
            g_account[i].balance = MAX_WATER_TIME;
            g_account[i].fract = 0;
            g_account[i].last = get_time();
        }
    }

    sei();
    printf("\nboot: %#x\n", mcusr);

    char linebuf[64];
    uint8_t linelen = 0;
    uint8_t calib = 0;

    for (;;) {
        twi_dump_trace();
        if (twi_cmd_pending()) {
            CHECKPOINT;
            uint8_t cmd = twi_next_cmd();
            printf("command: %#x\n", cmd);
            switch (CMD_TYPE(cmd)) {
            case CMD_GET_WEIGHT:
                measure_weight();
                break;
            case CMD_WATERING:
                {
                    uint8_t index = CMD_INDEX(cmd);
                    twi_set_last_watering(index,
                        water(index, twi_get_watering(index)));
                }
                break;
            }
        }

        if (debug_char_pending()) {
            CHECKPOINT;
            char c = debug_getchar();
            if (calib) {
                if (c != '\n' && c != '\r') {
                    linebuf[linelen++] = c;
                } else {
                    linebuf[linelen] = '\0';
                    uint32_t offset, scale;
                    if (sscanf(linebuf, "%lu %lu", &offset, &scale) == 2) {
                        hx711_calib(calib - 1, offset, scale);
                    } else {
                        printf("failed to read offset and scale: '%s'\n",
                               linebuf);
                    }
                    linelen = 0;
                    calib = 0;
                }
            } else {
                switch (c) {
                case 't': measure_timer(); break;
                case 'w': measure_weight_2(); break;
                case '1': calib = 1; break;
                case '2': calib = 2; break;
                case 'a': water(0, 4); break;
                case 'b': water(1, 4); break;
                case 'p': print_calib(); break;
                case 'u': hx711_write_calib(eemem_scale_calib); break;
                }
            }
        }
    }

    return 0; /* never reached */
}
