#include "timer.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

uint32_t s_count = 0;

void timer_start(void) {
    PRR &= ~(1 << PRTIM0);
    // timer overflow interrupt enabled
    TIMSK0 = (1 << TOIE0);
    // 1/1024
    TCCR0B = (1 << CS02) | (1 << CS00);
    // TCCR1B = (1 << CS12);
    TCNT0 = 0;
}

ISR (TIMER0_OVF_vect) {
    ++s_count;
}

uint32_t get_time(void) {
    uint8_t c = TCNT0;
    return (s_count << 8) | (uint32_t)c;
}
