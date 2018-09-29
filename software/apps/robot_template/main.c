// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "mpu9250.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

typedef enum {
  OFF,
  DRIVING,
  TURN_CLOCKWISE,
  REDIRECT,
  CLIFF,
  REORIENT,
  FIND_UP,
} robot_state_t;

float distance = 0;

float x_accel = 0;
float y_accel = 0;
float z_accel = 0;

float theta;
float psi;

float theta_min = 0;

static float measure_distance (uint16_t current_encoder, uint16_t previous_encoder) {
	const float CONVERSION = 0.00008529;
	float retval = 0;
	float retval_edge_case = 0;
	retval = (current_encoder - previous_encoder)*CONVERSION; 

	if (current_encoder < previous_encoder) {
		retval_edge_case = (0xFFFF - previous_encoder + current_encoder)*CONVERSION;
		if (retval_edge_case > 1) {
			return 0;
		}
		return retval_edge_case;
	} 

	return retval;

}


static void read_tilt(void) {

	//your code here
	mpu9250_measurement_t tilt = mpu9250_read_accelerometer();


	float accel_x = tilt.x_axis;
	float accel_y = tilt.y_axis;
	float accel_z = tilt.z_axis;

	theta = atan(accel_x / sqrt(accel_y* accel_y + accel_z* accel_z));
	psi = atan(accel_y / sqrt(accel_x* accel_x + accel_z* accel_z));
	//convert to degrees
	psi = psi * (180 / M_PI);
	theta = theta * (180 / M_PI);


	printf("psi: %f, theta: %f\n", psi, theta);
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  mpu9250_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // configure initial state
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};

  float left_wheel_encoder_prev = 0;
  float right_wheel_encoder_prev = 0;
  float left_wheel_encoder_curr = 0;
  float right_wheel_encoder_curr = 0;
  float angle = 0;
  float newAngle;
  float reorient_angle;
  float cumulative_angle = 0;
  float orient_angle = 0;

  bool bump_right;
  bool bump_left;
  bool bump_center;

  bool cliff_left;
  bool cliff_right;
  bool cliff_center;

  bool redirect_integration = 1;
  int collision_direction;

  // loop forever, running state machine
  while (1) {
    // read sensors from robot
    kobukiSensorPoll(&sensors);
    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(100);

    //left_wheel_encoder_prev = left_wheel_encoder_curr;
    //right_wheel_encoder_prev = right_wheel_encoder_curr;
    read_tilt();

    left_wheel_encoder_curr = sensors.leftWheelEncoder;
    right_wheel_encoder_curr = sensors.rightWheelEncoder;
    bump_right = sensors.bumps_wheelDrops.bumpRight;
    bump_left = sensors.bumps_wheelDrops.bumpLeft;
    bump_center = sensors.bumps_wheelDrops.bumpCenter;

    cliff_left = sensors.cliffLeft;
    cliff_right = sensors.cliffRight;
    cliff_center = sensors.cliffCenter;

    // handle states
    switch(state) {

    	case OFF: {
    		if (is_button_pressed(&sensors)) {
    			// find which way is up.
    			state = FIND_UP;

    			//start rotating clockwise
    			kobukiDriveDirect(50, -50);
    		} else {
	          // perform state-specific actions here
	          display_write("OFF", DISPLAY_LINE_0);
	          state = OFF;
        	}
        	break;
    	}

    	case FIND_UP: {
    		display_write("FINDING UP", DISPLAY_LINE_0);
    		//keep driving until theta is at its min value
    		if (redirect_integration){ 
				mpu9250_start_gyro_integration();
				redirect_integration = 0;
			} else if(cumulative_angle < 360) {
				kobukiDriveDirect(50, -50);
				newAngle = abs(mpu9250_read_gyro_integration().z_axis);
				cumulative_angle += abs(newAngle - angle);
				orient_angle += abs(newAngle - angle);
				angle = newAngle;
				
				// display_write("TURNING", DISPLAY_LINE_0);
				char display_angle[16];
			    snprintf(display_angle, 16, "%f", theta);
				display_write(display_angle, DISPLAY_LINE_1); 

				if (theta < theta_min) {
					theta_min = theta;
					orient_angle = 0;
				}
			} else if(abs(orient_angle) > 5) {
				kobukiDriveDirect(-50, 50);
				newAngle = abs(mpu9250_read_gyro_integration().z_axis);
				orient_angle -= abs(newAngle - angle);
				angle = newAngle;
			} else {
				kobukiDriveDirect(0, 0);
				state = DRIVING;

			}
    		break;
    	}

    	case DRIVING: {
    		kobukiDriveDirect(200, 200);
	        // transition logic for part 1
	        /*if (is_button_pressed(&sensors) || distance >= 1) {
			  state = OFF;
			  kobukiDriveDirect(0, 0);
			  distance = 0;
			  //display_write("PRESSED_BUTTON_DRIVE", DISPLAY_LINE_0);*/
			if (is_button_pressed(&sensors)){
				state = OFF;
				kobukiDriveDirect(0, 0);
				distance = 0;
				angle = 0;
			} else if (cliff_left || cliff_right) {
				display_write("REDIRECT", DISPLAY_LINE_0);
				if (cliff_left) collision_direction = -1;
				if (cliff_center) collision_direction = 0;
				if (cliff_right) collision_direction = 1;
				state = REDIRECT;
				distance = 0;
				angle = 0;
				//mpu9250_start_gyro_integration();
				left_wheel_encoder_prev = sensors.leftWheelEncoder;
			  	right_wheel_encoder_prev = sensors.rightWheelEncoder;
				kobukiDriveDirect(-100, -100);
			} else if (bump_center || bump_left || bump_right) {
				//we hit an obstacle
				kobukiDriveDirect(0, 0);
				state = TURN_CLOCKWISE;


			} else  {
	          // perform state-specific actions here
	          state = DRIVING;
	          display_write("DRIVING", DISPLAY_LINE_0);
			  distance = measure_distance(right_wheel_encoder_curr, right_wheel_encoder_prev);
			  char display_str[16];
			  snprintf(display_str, 16, "%f", distance);
	          display_write(display_str, DISPLAY_LINE_1);
	   		}
	    	break; // each case needs to end with break!
    	}

    /*
      case OFF: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = DRIVING;
		  kobukiDriveDirect(100, 100);
		  left_wheel_encoder_prev = sensors.leftWheelEncoder;
		  right_wheel_encoder_prev = sensors.rightWheelEncoder;
		  //display_write("PRESSED_BUTTON_DRIVE", DISPLAY_LINE_0);
        } else {
          // perform state-specific actions here
          display_write("OFF", DISPLAY_LINE_0);
          state = OFF;
        }
        break; // each case needs to end with break!
      }

      case DRIVING: {
		kobukiDriveDirect(100, 100);
        // transition logic for part 1
        /*if (is_button_pressed(&sensors) || distance >= 1) {
		  state = OFF;
		  kobukiDriveDirect(0, 0);
		  distance = 0;
		  //display_write("PRESSED_BUTTON_DRIVE", DISPLAY_LINE_0);
		if (is_button_pressed(&sensors)){
			state = OFF;
			kobukiDriveDirect(0, 0);
			distance = 0;
			angle = 0;
		} else if (cliff_center || cliff_left || cliff_right) {
			// reached a cliff
			// stop driving.
			state = CLIFF;
			kobukiDriveDirect(0, 0);

			//reset the distance variable
			distance = 0;
		}

		else if (bump_left || bump_right || bump_center){
			display_write("REDIRECT", DISPLAY_LINE_0);
			if (bump_left) collision_direction = -1;
			if (bump_center) collision_direction = 0;
			if (bump_right) collision_direction = 1;
			state = REDIRECT;
			distance = 0;
			angle = 0;
			//mpu9250_start_gyro_integration();
			left_wheel_encoder_prev = sensors.leftWheelEncoder;
		  	right_wheel_encoder_prev = sensors.rightWheelEncoder;
			kobukiDriveDirect(-100, -100);
		} else if (distance >= 0.5) {
			//adjust my current orientation to cumulative angle
			state = REORIENT;
			kobukiDriveDirect(0, 0);
			distance = 0;
			//angle = 0;
			mpu9250_start_gyro_integration();
	     } else  {
	          // perform state-specific actions here
	          state = DRIVING;
	          display_write("DRIVING", DISPLAY_LINE_0);
			  distance = measure_distance(right_wheel_encoder_curr, right_wheel_encoder_prev);
			  char display_str[16];
			  snprintf(display_str, 16, "%f", distance);
	          display_write(display_str, DISPLAY_LINE_1);
	   	}
	    break; // each case needs to end with break!
      }
      */
      case TURN_CLOCKWISE: {
		kobukiDriveDirect(50, -50); 
		angle = abs(mpu9250_read_gyro_integration().z_axis);
		//transition logic
		if (is_button_pressed(&sensors)){
			state = OFF;
			kobukiDriveDirect(0, 0);
			angle = 0;
			distance = 0;
		 	mpu9250_stop_gyro_integration();
		} else if (abs(angle) >= 177) {	
			state = DRIVING;
		 	mpu9250_stop_gyro_integration();
			kobukiDriveDirect(0, 0);
			left_wheel_encoder_prev = sensors.leftWheelEncoder;
		  	right_wheel_encoder_prev = sensors.rightWheelEncoder;
			distance = 0;
			angle = 0;
		} else {
			state = TURN_CLOCKWISE;
			display_write("TURN BACK", DISPLAY_LINE_0);
			char display_angle[16];
		    snprintf(display_angle, 16, "%f", angle);
			display_write(display_angle, DISPLAY_LINE_1);	
		}
		break;
      }

      case REDIRECT: {
			if (is_button_pressed(&sensors)){
				state = OFF;
				kobukiDriveDirect(0, 0);
				angle = 0;
				distance = 0;
				redirect_integration = 1;
				mpu9250_stop_gyro_integration();
			}  else if (cliff_left || cliff_center || cliff_right) {
				//restart the redirect
				display_write("REDIRECT", DISPLAY_LINE_0);
				if (cliff_left) collision_direction = -1;
				if (cliff_center) collision_direction = 0;
				if (cliff_right) collision_direction = 1;
				state = REDIRECT;
				distance = 0;
				angle = 0;
				//mpu9250_start_gyro_integration();
				left_wheel_encoder_prev = sensors.leftWheelEncoder;
			  	right_wheel_encoder_prev = sensors.rightWheelEncoder;
				kobukiDriveDirect(-100, -100);
			}

			else if (distance < 0.1) {
				kobukiDriveDirect(-100, -100);
		 	     display_write("REDIRECT", DISPLAY_LINE_0);
				 distance = measure_distance(right_wheel_encoder_prev, right_wheel_encoder_curr);
			 	 char display_str[16];
			 	 snprintf(display_str, 16, "%f", distance);
		         display_write(display_str, DISPLAY_LINE_1);
			} else if ((collision_direction == -1 || collision_direction == 0) && angle < 45) {
				if (redirect_integration){ 
					mpu9250_start_gyro_integration();
					redirect_integration = 0;
				}
				kobukiDriveDirect(100, -100);
				newAngle = abs(mpu9250_read_gyro_integration().z_axis);
				cumulative_angle += abs(newAngle - angle);
				angle = newAngle;
				display_write("TURNING", DISPLAY_LINE_0);
				char display_angle[16];
			    snprintf(display_angle, 16, "%f", angle);
				display_write(display_angle, DISPLAY_LINE_1); 
			} else if (collision_direction == 1 && angle < 45) {
				if (redirect_integration){ 
					mpu9250_start_gyro_integration();
					redirect_integration = 0;
				}
				kobukiDriveDirect(-100, 100);
				newAngle = abs(mpu9250_read_gyro_integration().z_axis);
				cumulative_angle -= abs(newAngle - angle);
				angle = newAngle;
				display_write("TURNING", DISPLAY_LINE_0);
				char display_angle[16];
			    snprintf(display_angle, 16, "%f", angle);
				display_write(display_angle, DISPLAY_LINE_1); 
			} 
				else {
				//mpu9250_stop_gyro_integration();

				//return to original angle
				//mpu9250_start_gyro_integration();

				//drive forward a little bit to not hit the obstacle again
				// kobukiDriveDirect(100, 100);
				// nrf_delay_ms(1000);
				// kobukiDriveDirect(0, 0);

				//stop driving
				//mpu9250_stop_gyro_integration();
				kobukiDriveDirect(0, 0);
				//nrf_delay_ms(2000);
				state = DRIVING;
				redirect_integration = 1;
				reorient_angle = angle;
				// kobukiDriveDirect(100, 100);
				left_wheel_encoder_prev = sensors.leftWheelEncoder;
			  	right_wheel_encoder_prev = sensors.rightWheelEncoder;
				distance = 0;
				angle = 0;

			}
      	break;
      }

      // add other cases here

      case CLIFF: {
      	//make sure we stop driving
      	kobukiDriveDirect(0, 0);

      	//any other functionality here

      	//enter the OFF state
      	state = OFF;
      	break;
      }

      case REORIENT: {
      	// stop driving
      	printf("Cumulative angle %f\n", cumulative_angle);
      	if (is_button_pressed(&sensors)){
			state = OFF;
			kobukiDriveDirect(0, 0);
			angle = 0;
			distance = 0;
			redirect_integration = 1;
			mpu9250_stop_gyro_integration();
		} else if(abs(cumulative_angle) > 5.0f) {
			//printf("end2, %f\n", cumulative_angle);
			newAngle = abs(mpu9250_read_gyro_integration().z_axis);
			if (cumulative_angle < 0) {
				kobukiDriveDirect(100, -100);
				cumulative_angle += abs(newAngle - reorient_angle);
			} else {
				kobukiDriveDirect(-100, 100);
				cumulative_angle -= abs(newAngle - reorient_angle);
			}
			reorient_angle = newAngle;
			display_write("CUMUL. ANGLE", DISPLAY_LINE_0);
			char display_angle[16];
		    snprintf(display_angle, 16, "%f", cumulative_angle);
			display_write(display_angle, DISPLAY_LINE_1);
		} else {
			state = DRIVING;

			angle = 0;
			reorient_angle = 0;
			distance = 0;
			cumulative_angle = 0;
			redirect_integration = 1;
			
			left_wheel_encoder_prev = sensors.leftWheelEncoder;
		  	right_wheel_encoder_prev = sensors.rightWheelEncoder;

		  	mpu9250_stop_gyro_integration();
			kobukiDriveDirect(100, 100);
		}
		break;
      }

    }
  }
}

