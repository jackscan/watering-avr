#include "timer.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

uint16_t s_count = 0;

void timer_start(void) {
    PRR &= ~(1 << PRTIM1);
    // timer overflow interrupt enabled
    TIMSK1 = (1 << TOIE1);
    // 1/1024
    TCCR1B = (1 << CS12) | (1 << CS10);
    // TCCR1B = (1 << CS12);
    TCNT1 = 0;
}

ISR (TIMER1_OVF_vect) {
    ++s_count;
}

uint32_t get_time(void) {
    uint16_t c = TCNT1;
    return ((uint32_t)s_count << 16) | (uint32_t)c;
}
