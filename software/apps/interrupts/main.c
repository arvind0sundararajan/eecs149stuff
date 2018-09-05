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
uint8_t button0 = 28;
bool led_on = true;

void SWI1_EGU1_IRQHandler(void) {
    NRF_EGU1->EVENTS_TRIGGERED[0] = 0;
}

void GPIOTE_IRQHandler(void) {
    
    NRF_GPIOTE->EVENTS_IN[0] = 0;

    if (led_on){
      printf("clearing led\n");
      gpio_clear(led0);
      led_on = false;
    } else {
      printf("setting led\n");
      gpio_set(led0);
      led_on = true;
    }

}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  gpio_config(led0, OUTPUT);
  gpio_config(button0, INPUT);

  //5.2.2
  // gpiote config[0]
  // set the 1s at 17 and 0, and 12, 11, and 10
  NRF_GPIOTE->CONFIG[0] |= ((1 << 17) | (1 << 12) | (1 << 11) | (1 << 10) | 1); 

  // set the 0s at 16 and 1
  NRF_GPIOTE->CONFIG[0] &= ~((1 << 16) | (1<<1) | (1 << 8) | (1 << 9));

  // set psel
  NRF_GPIOTE->CONFIG[0] |= 

  //enable interrupt for event[0]
  NRF_GPIOTE->INTENSET |= 1;

  // enable in NVIC
  NVIC_EnableIRQ(GPIOTE_IRQn);



  // loop forever
  while (1) {
    printf("Looping\n");
    nrf_delay_ms(1000);
  }
}

