// Blink app
//
// Blinks the LEDs on Buckler

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "software_interrupt.h"
#include "gpio.h"

#include "buckler.h"

uint8_t led0 = 25; 
uint8_t switch0 = 22;
uint8_t button0 = 28;


void SWI1_EGU1_IRQHandler(void) {
    NRF_EGU1->EVENTS_TRIGGERED[0] = 0;
    printf("Branden is cool. SW interrupt beginning\n");
    nrf_delay_ms(2000);
    printf("SW interrupt complete.\n");
}

void GPIOTE_IRQHandler(void) {
    
    NRF_GPIOTE->EVENTS_IN[0] = 0;

    /*
    if (switch0_is_high) {
      printf("lowering switch\n");
      switch0_is_high = false;
    } else {
      printf("switch is high\n");
      switch0_is_high = true;
    }
    */

    //button0/led0 stuff for 5.2.2
    ///printf("setting led\n");
    // set/clear switched for some reason - maybe 0 is enable?
    printf("Shromona is cool. Button press ISR beginning\n");
    gpio_clear(led0);
  
    nrf_delay_ms(2000);

    //printf("clearing led\n");
    gpio_set(led0);
    printf("Button press ISR complete.\n");
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  software_interrupt_init();

  gpio_config(led0, OUTPUT);
  gpio_config(button0, INPUT);
  gpio_config(switch0, INPUT);

  //5.2.3; set switch 0 to be interrupt source
  // gpiote config[0]
  // set the 1s at 17 and 0, and 12, 11, and 10
  NRF_GPIOTE->CONFIG[0] |= ((1 << 17) | (1 << 12) | (1 << 11) | (1 << 10) | 1); 

  // set the 0s at 16 and 1
  NRF_GPIOTE->CONFIG[0] &= ~((1<< 16) | (1<<1) | (1 << 8) | (1 << 9));

  //enable interrupt for event[0]
  NRF_GPIOTE->INTENSET |= 1;

  // enable in NVIC
  NVIC_EnableIRQ(GPIOTE_IRQn);

  NVIC_SetPriority(GPIOTE_IRQn, 0);
  NVIC_SetPriority(SWI1_EGU1_IRQn, 1);



  // loop forever
  while (1) {
    if (gpio_read(switch0)) {
        software_interrupt_generate();
        //printf("looping\n");
        nrf_delay_ms(1000);
    } else {
      printf("switch is not set\n");
      __WFI();
    }

  }
}