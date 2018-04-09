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

#define WATERING_DDR DDRD
#define WATERING_PORT PORTD
#define WATERING_BIT (1 << PD3)


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


    hx711_init();

    // disable all peripheral clocks
    PRR = 0xFF;

    return mcusr;
}

static void measure_weight(void) {
    uint32_t t0 = get_time();

    while (twi_get_cmd() == CMD_GET_WEIGHT &&
           get_time() - t0 < TIMER_MS(1000)) {
        twi_add_weight(hx711_read());
    }

    uint32_t t1 = get_time();

    hx711_powerdown();

    printf("weight measuring: %lu - %lu = %lu\n", t1, t0, t1 - t0);

    uint32_t w = twi_get_weight();
    uint32_t n = (w >> 16);
    uint32_t d = (w & 0xFFFF);
    printf("weight: %u / %u = %u\n", (uint16_t)n, (uint16_t)d, (uint16_t)((n + d / 2) / d));
}

static void measure_weight_2(void) {
    for (uint8_t i = 0; i < 10; ++i) {
        printf("%u\n", hx711_read());
    }
    hx711_powerdown();
}

static __attribute__ ((section (".noinit"))) struct {
    uint32_t balance;
    uint32_t last;
    uint32_t fract;
} g_account;

uint8_t water_limit(uint8_t t) {
    uint32_t curr = get_time();

    uint8_t sreg = SREG;
    cli();
    uint32_t dt = (curr - g_account.last + g_account.fract);
    uint32_t da = dt / WATER_TIME_REFILL_INTERVAL;
    uint32_t na = da + g_account.balance;

    g_account.balance = na < MAX_WATER_TIME ? na : MAX_WATER_TIME;
    g_account.fract = dt - da * WATER_TIME_REFILL_INTERVAL;
    g_account.last = curr;
    SREG = sreg;

    if (t > g_account.balance) t = g_account.balance;

    return t;
}

static void update_water_account(uint8_t t) {
    g_account.balance = (t < g_account.balance) ? g_account.balance - t : 0;
}

static uint8_t water(uint8_t t) {
    t = water_limit(t);

    printf("watering %ums\n", (uint16_t)t * 250u);

    wdt_enable(WDTO_500MS);
    uint32_t t0 = get_time();
    WATERING_PORT |= WATERING_BIT;
    for (uint8_t i = 0; i < t; ++i) {
        _delay_ms(250);
        wdt_reset();
    }
    uint32_t t1 = get_time();
    WATERING_PORT &= ~WATERING_BIT;
    wdt_disable();

    uint32_t f = (F_CPU / 4UL);
    uint32_t dt = ((t1 - t0) * TIMER_PS + f/2) / f;
    printf("watered %lums\n", dt * 250);

    update_water_account(dt);

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
        g_account.balance = MAX_WATER_TIME;
        g_account.fract = 0;
        g_account.last = get_time();
    }

    sei();
    printf("\nboot: %#x\n", mcusr);

    for (;;) {
        twi_dump_trace();
        if (twi_cmd_pending()) {
            CHECKPOINT;
            uint8_t cmd = twi_next_cmd();
            printf("command: %#x\n", cmd);
            switch (cmd) {
            case CMD_GET_WEIGHT:
                measure_weight();
                break;
            case CMD_WATERING:
                twi_set_last_watering(water(twi_get_watering()));
                break;
            }
        }

        if (debug_char_pending()) {
            CHECKPOINT;
            char c = debug_getchar();
            switch (c) {
            case 't': measure_timer(); break;
            case 'w': measure_weight_2(); break;
            }
        }
    }

    return 0; /* never reached */
}
