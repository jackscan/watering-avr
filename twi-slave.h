#pragma once

#include <stdint.h>
#include <stdbool.h>

#define CMD_GET_MOISTURE 0x10
#define CMD_GET_WATER_LEVEL 0x11
#define CMD_GET_LAST_WATERING 0x12
#define CMD_GET_WATER_LIMIT 0x13
#define CMD_WATERING 0x5A
#define CMD_ECHO 0x99

void twi_slave_init(uint8_t addr);
bool twi_busy(void);
bool twi_cmd_pending(void);
uint8_t twi_get_cmd(void);
uint8_t twi_next_cmd(void);
void twi_dump_trace(void);

uint8_t twi_get_watering(void);
void twi_set_last_watering(uint8_t);
void twi_add_moisture(uint16_t value);
uint32_t twi_get_moisture(void);
void twi_add_water_level(uint16_t value);
