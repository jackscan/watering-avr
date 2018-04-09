#pragma once

#include <stdint.h>
#include <stdbool.h>

#define CMD_SHIFT 1

#define CMD_GET_LAST_WATERING   0x10
#define CMD_GET_WATER_LIMIT     0x11
#define CMD_GET_WEIGHT          0x12
#define CMD_WATERING            0x1A
#define CMD_ECHO                0x29

#define CMD_MASK        (0xFF << CMD_SHIFT)
#define CMD_INDEX(cmd)  (cmd & ~CMD_MASK)
#define CMD_TYPE(cmd)   (cmd >> CMD_SHIFT)
#define CMD_CONS(cmd, index) ((cmd << CMD_SHIFT) | (index & ~CMD_MASK))

void twi_slave_init(uint8_t addr);
bool twi_busy(void);
bool twi_cmd_pending(void);
uint8_t twi_get_cmd(void);
uint8_t twi_next_cmd(void);
void twi_dump_trace(void);

uint8_t twi_get_watering(uint8_t index);
void twi_set_last_watering(uint8_t index, uint8_t);
void twi_add_weight(uint8_t index, uint16_t value);
uint32_t twi_get_weight(uint8_t index);
