// Blink app
//
// Blinks the LEDs on Buckler

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"

#include "buckler.h"
#include "gpio.h"


int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  //blinking stuff for part 1
  //uint32_t* gpio_out_addr = 0x50000504;
  //uint32_t* gpio_dir_addr = 0x50000514;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  /* more blinking stuff for part 1
  uint32_t gpio_out_val = *gpio_out_addr;
  uint32_t gpio_dir_val = *gpio_dir_addr;

  printf("GPIO OUT addr, val: %x, %x\n", gpio_out_addr, gpio_out_val);
  printf("GPIO DIR addr, val: %x, %x\n", gpio_dir_addr, gpio_dir_val);

  //set gpio23 as outputs 
  *gpio_dir_addr = (1 << 23);

  // toggle those bits in gpio_out
  for (int j = 0; j < 100; j++) {
  	// xor the gpio_out value with 1 <<23
  	*gpio_out_addr ^= (1 << 23);

  	// pause 1 s
  	nrf_delay_ms(1000);
  }
  */

  /* part 2*/
  //set button0 (28) and switch0 (22) as inputs; connect input buffers
  //gpio_reg_cnf->gpio_pin_cnf_22 &= 0xFFFFFFFC;
  //gpio_reg_cnf->gpio_pin_cnf_28 &= 0xFFFFFFFC;

  uint8_t led0 = 25; 
  uint8_t led1 = 24;
  uint8_t button0 = 28;
  uint8_t switch0 = 22;

  // configure LED0, LED1 as outputs
  gpio_config(led0, OUTPUT);
  gpio_config(led1, OUTPUT);

  //configure BUTTON0, SWITCH0 as input
  gpio_config(switch0, INPUT);
  gpio_config(button0, INPUT);


  // loop forever
  while (1) {

  	/* task 3 stuff
  	uint32_t pin_state = gpio_vars->gpio_in;
  	printf("GPIO_IN: 0x%x\n", pin_state);

  	int button0_val = (pin_state & (1 << 28)) >> 28;
  	int switch0_val = (pin_state & (1 << 22)) >> 22;

  	printf("BUTTON0: %d; ", button0_val);
  	printf("SWITCH0: %d\n", switch0_val);

  	nrf_delay_ms(100);
  	*/
  	if (gpio_read(button0)){
  		gpio_set(led0);
  	} else {
  		gpio_clear(led0);
  	}

  	if (gpio_read(switch0)) {
  		gpio_set(led1);
  	} else {
  		gpio_clear(led1);
  	}
  }
}

