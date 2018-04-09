#pragma once

#include <stdint.h>

enum hx711_chan_gain {
    HX711_CHANNEL_A_128 = 0,
    HX711_CHANNEL_B_32 = 1,
    HX711_CHANNEL_A_64 = 2,
};

struct scale_calib {
    uint32_t offset;
    uint32_t scale;
};

void hx711_init(const void *calib);
void hx711_powerdown(void);
void hx711_calib(uint8_t index, uint32_t offset, uint32_t scale);
void hx711_write_calib(void *calib);
uint16_t hx711_read(void);
void hx711_finish_read(enum hx711_chan_gain);
