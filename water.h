#pragma once

#include "timer.h"

#include <stdint.h>

#define MAX_WATER_TIME  80 // 20s
#define WATER_TIME_REFILL_INTERVAL TIMER_MIN(30)

uint8_t water_limit(uint8_t index, uint8_t t);
