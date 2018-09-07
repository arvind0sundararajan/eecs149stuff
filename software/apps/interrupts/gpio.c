#include "gpio.h"
#include <stdbool.h>

gpio_vars_t* gpio_vars = 0x50000504;
gpio_reg_cnf_t* gpio_reg_cnf = 0x50000700;

// Inputs: 
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)
void gpio_config(uint8_t gpio_num, gpio_direction_t dir) {
	//get a bit mask
	uint32_t bit_mask = 1 << gpio_num;

	//find the direction
	if (dir == OUTPUT) {
		//modify the gpio_dir register at that bit
		gpio_vars->gpio_dir |= bit_mask; 
	}

	else {
		//modify gpio dir register to an input (0)
		//gpio_vars->gpio_dir &= ~bit_mask;
		//connect input buffer etc.
		gpio_reg_cnf->gpio_pin_cnf[gpio_num] &= 0xFFFFFFFC;
	}

}

// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num) {
	uint32_t bit_mask = 1 << gpio_num;
	gpio_vars->gpio_out |= bit_mask;
}

// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num) {
	uint32_t bit_mask = 1 << gpio_num;
	gpio_vars->gpio_out &= ~bit_mask;
}

// Inputs: 
//  gpio_num - gpio number 0-31
bool gpio_read(uint8_t gpio_num) {
    // should return pin state
    uint32_t pin_state = gpio_vars->gpio_in;
    uint32_t bit_mask = 1 << gpio_num;

    if ((pin_state & bit_mask) != 0) {
    	return true;
    }

    return false;
}