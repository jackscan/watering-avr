#include "freqcounter.h"
#include "twi-slave.h"

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#include <stdio.h>

#define SD_DDR DDRD
#define SD_PORT PORTD
#define SD_BIT (1 << PD2)

#define CLOCK_T0 (1 << CS02) | (1 << CS01) | (1 << CS00)
#define CLOCK_T1 (1 << CS12) | (1 << CS11) | (1 << CS10)

static inline void stop_counter(void) {
    TCCR0B = 0;
    TCCR1B = 0;
}

static inline void start_counter(void) {
    // external clock on rising edge
    TCCR0B = CLOCK_T0;
    TCCR1B = CLOCK_T1;
}

void freqcounter_start(void) {
    PRR &= ~((1 << PRTIM0) | (1 << PRTIM1));

    // timer overflow interrupt disabled
    TIMSK0 = 0;
    TIMSK1 = 0;

    stop_counter();

    // power freqcounter
    SD_DDR |= SD_BIT;
    SD_PORT |= SD_BIT;

    TCNT0 = 0;
    TCNT1 = 0;

    start_counter();

    // wait for first cycle to finish
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    while ((TIFR0 & (1 << TOV0)) == 0) {
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
    }

    stop_counter();
    TCNT0 = 0;
    TCNT1 = 0;
    // unset overflow flag
    TIFR0 = (1 << TOV0);
    // timer overflow interrupt enabled
    TIMSK0 = (1 << TOIE0);
    TIMSK1 = (1 << TOIE1);
    start_counter();
}

void freqcounter_stop(void) {
    stop_counter();
    // powerdown freqcounter
    SD_DDR &= ~SD_BIT;
    SD_PORT &= ~SD_BIT;

    PRR |= (1 << PRTIM0) | (1 << PRTIM1);
}

ISR (TIMER0_OVF_vect) {
    twi_add_water_level(TCNT1);
    TCNT1 = 0;
}
