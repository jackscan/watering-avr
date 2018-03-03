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

#define MOIST_DDR DDRC
// #define MOIST_PIN PINC
#define MOIST_PORT PORTC
#define MOIST_BIT (1 << PC0)
#define MOIST_MUX (0) // ADC0

#define MOISTVCC_DDR DDRB
#define MOISTVCC_PORT PORTB
#define MOISTVCC_BIT (1 << PB2)

#define CAPLOAD_DDR DDRD
#define CAPLOAD_PORT PORTD
#define CAPLOAD_BIT (1 << PD4)

#define CAPSENS_DDR DDRB
#define CAPSENS_PORT PORTB
#define CAPSENS_BIT (1 << PB0)

#define WATERING_DDR DDRD
#define WATERING_PORT PORTD
#define WATERING_BIT (1 << PD3)

// #define MOIST_DDR DDRC
// #define MOIST_PIN PINC
// #define MOIST_BIT (1 << PC3)
// #define MOIST_MUX ((1 << MUX1) | (1 << MUX0))

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
    DDRD = (1 << DDD2) | (1 << DDD3) | (1 << DDD4) | (1 << DDD5) | (1 << DDD6) | (1 << DDD7);
    // drive pins low
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;

    // set moisture signal pin to Hi-Z
    MOIST_PORT &= ~MOIST_BIT;
    MOIST_DDR &= ~MOIST_BIT;

    // power moisture sensor
    MOISTVCC_DDR |= MOISTVCC_BIT;
    MOISTVCC_PORT |= MOISTVCC_BIT;

    hx711_init();

    // disable all peripheral clocks
    PRR = 0xFF;

    return mcusr;
}

ISR(ADC_vect) {
    uint16_t val = 1024 - ADCW;

    switch (ADMUX & 0xF) {
    case MOIST_MUX:
        twi_add_moisture(val);
        break;
    }
}

static int uint16compare(const void *a, const void *b)
{
    return (int)(*(uint16_t*)a - *(uint16_t*)b);
}

static uint16_t measure_cap(void) {

#define CAP_MEASURE_COUNT 256
    // charge capacitor to ground
    CAPSENS_DDR |= CAPSENS_BIT;
    CAPSENS_PORT &= ~CAPSENS_BIT;
    CAPLOAD_DDR |= CAPLOAD_BIT;
    CAPLOAD_PORT &= ~CAPLOAD_BIT;

    uint16_t data[CAP_MEASURE_COUNT] = {};

    // disable power reduction for timer 1
    PRR &= ~(1 << PRTIM1);
    // no interrupts
    TIMSK1 = 0;
    uint8_t clp = CAPLOAD_PORT | CAPLOAD_BIT;
    // input capture noise canceler and input capture enabled and prescale 1
    uint8_t tccr1b = (1 << ICNC1) | (1 << ICES1) | (1 << CS10);

    uint32_t dt = 50 * CAP_MEASURE_COUNT;
    OCR1A = (F_CPU + dt / 2) / dt;

    // printf("measure %u * %u\n", CAP_MEASURE_COUNT, OCR1A);
    // uint32_t start = get_time();

    for (int i = 0; i < CAP_MEASURE_COUNT; ++i) {
        // set as input
        CAPSENS_DDR &= ~CAPSENS_BIT;

        // resetup timer 1

        // stop timer
        TCCR1B = 0;
        TCNT1 = 0;
        ICR1 = 0;
        // clear input capture flag and output compare flag
        TIFR1 = (1 << ICF1) | (1 << OCF1A);
        // start timer
        TCCR1B = tccr1b;
        // load capacitor to high
        CAPLOAD_PORT = clp;
        // wait for input capture
        while ((TIFR1 & (1 << ICF1)) == 0)
            ;

        uint16_t t = ICR1;

        // charge capacitor to ground
        CAPLOAD_PORT &= ~CAPLOAD_BIT;
        CAPSENS_PORT &= ~CAPSENS_BIT;
        CAPSENS_DDR |= CAPSENS_BIT;

        // restore SREG

        data[i] = t;

        // wait for output compare match
        while ((TIFR1 & (1 << OCF1A)) == 0)
            ;
    }

    // uint32_t end = get_time();
    // printf("duration: %lu\n", end - start);

    // reset pins
    CAPSENS_PORT &= ~CAPSENS_BIT;
    CAPSENS_DDR |= CAPSENS_BIT;
    CAPLOAD_PORT &= ~CAPLOAD_BIT;
    CAPLOAD_DDR |= CAPLOAD_BIT;

    // reenable power reduction for timer 1
    PRR |= (1 << PRTIM1);

    // printf("t:");
    // for (int i = 0; i < CAP_MEASURE_COUNT; ++i) {
    //     printf(" %u", data[i]);
    // }

    // qsort()

    uint32_t sum = 0;
    uint32_t m = CAP_MEASURE_COUNT/4;

    qsort(data, CAP_MEASURE_COUNT, sizeof(data[0]), uint16compare);
    // printf("t:");
    // for (int i = 0; i < CAP_MEASURE_COUNT; ++i) {
    //     printf(" %u", data[i]);
    // }

    for (int i = m; i < CAP_MEASURE_COUNT - m; ++i) {
        sum += data[i];
    }

    uint32_t gain = 2;
    uint32_t c = (CAP_MEASURE_COUNT - 2*m) / gain;
    uint16_t r = (uint32_t)((sum + c / 2) / c);

    // uint32_t calcEnd = get_time();
    // printf("calcDur: %lu\n", calcEnd - end);

    // printf("\n -> %u (%u)\n", r, sum);

    return r;
}

static void measure_water_level(void) {
    do {
        twi_add_water_level(measure_cap());
        printf(".");
    } while (twi_get_cmd() == CMD_GET_WATER_LEVEL);
}

static void start_moisture_measure(void) {
    // power sensor
    // MOISTVCC_PORT |= MOISTVCC_BIT;

    // disable power reduction for ADC
    PRR &= ~(1 << PRADC);

    // AVCC and MOISTURE ADC pin
    ADMUX = (1 << REFS0) | MOIST_MUX;

    // free running mode
    ADCSRB = 0;

    // enable ADC
    // auto trigger enabled
    // clear interrupt flag
    // enable interrupt
    // with prescaler set to 1/128 = 62.5kHz
    ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | 7;

    // _delay_ms(1000);
    // start ADC
    ADCSRA |= (1 << ADSC);
}

static void stop_adc(void) {
    // disable ADC
    ADCSRA &= ~(1 << ADEN);
    PRR |= (1 << PRADC);
    // MOISTVCC_PORT &= ~MOISTVCC_BIT;
}

static void measure_moisture(void) {
    // printf("s:\n");
    // debug_finish();

    start_moisture_measure();

    uint32_t t0 = get_time();

    uint8_t sreg = SREG;
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    while (twi_get_cmd() == CMD_GET_MOISTURE &&
           get_time() - t0 < TIMER_MS(1000)) {
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
    }

    stop_adc();
    SREG = sreg;

    uint32_t t1 = get_time();
    printf("measuring: %lu - %lu = %lu\n", t1, t0, t1 - t0);

    // printf("moisture: %u\n", v);

    uint32_t m = twi_get_moisture();
    uint16_t n = (uint16_t)(m >> 16);
    uint16_t d = (uint16_t)(m & 0xFFFF);
    printf("moisture: %u / %u = %u\n", n, d, (n + d / 2) / d);
}

static void measure_moisture_2(void) {
    // power sensor
    // MOISTVCC_PORT |= MOISTVCC_BIT;

    // disable power reduction for ADC
    PRR &= ~(1 << PRADC);

    // AVCC and MOISTURE ADC pin
    ADMUX = (1 << REFS0) | MOIST_MUX;

    // free running mode
    ADCSRB = 0;

    // enable ADC
    // with prescaler set to 1/128 = 62.5kHz
    ADCSRA = (1 << ADEN) | 7;

    // start ADC
    ADCSRA |= (1 << ADSC);

    while ((ADCSRA & (1 << ADSC)) != 0)
        ;

    printf("moisture: %u\n", 1024 - ADCW);

    // disable ADC
    ADCSRA &= ~(1 << ADEN);
    PRR |= (1 << PRADC);
    // MOISTVCC_PORT &= ~MOISTVCC_BIT;
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
            case CMD_GET_MOISTURE:
                measure_moisture();
                break;
            case CMD_GET_WATER_LEVEL:
                measure_water_level();
                break;
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
            case 'm': measure_moisture_2(); break;
            case 'c': measure_cap(); break;
            case 'w': measure_weight_2(); break;
            }
        }
    }

    return 0; /* never reached */
}
