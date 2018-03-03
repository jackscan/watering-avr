#pragma once

#include <stdint.h>

void hx711_init(void);
void hx711_powerdown(void);
uint16_t hx711_read(void);
