#pragma once

#include "nrf.h"
#include "stdbool.h"

typedef enum {
    INPUT = 0,
    OUTPUT,
} gpio_direction_t;

typedef struct {
	uint32_t gpio_out;
	uint32_t gpio_outset;
	uint32_t gpio_outclr;
	uint32_t gpio_in;
	uint32_t gpio_dir;
	uint32_t gpio_dirset;
	uint32_t gpio_dirclr;
	uint32_t gpio_latch;
	uint32_t gpio_detectmode;
} gpio_vars_t;

typedef struct {
	uint32_t gpio_pin_cnf[32];
} gpio_reg_cnf_t;

// Inputs: 
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)
void gpio_config(uint8_t gpio_num, gpio_direction_t dir);

// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num);

// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num);

// Inputs: 
//  gpio_num - gpio number 0-31
// Returns:
//  current state of the specified gpio pin
bool gpio_read(uint8_t gpio_num);
