#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define ROTATE_0 700 //Rotate to 0° position
#define ROTATE_180 2300 //Rotate to 180° position

typedef struct{
	pwm_config config;
	uint32_t clk;
    uint32_t div; 
    uint8_t gpio_pin;
    uint8_t slice_num;
} ServoMotor;

/**
 * @brief motor initialization function
 * @param gpio_pin gpio pin to servo pwm
 * @return servo-motor structure
*/
ServoMotor servo_init(const uint8_t gpio_pin){
    ServoMotor servomotor;
    servomotor.gpio_pin = gpio_pin;

    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    servomotor.slice_num = pwm_gpio_to_slice_num(gpio_pin);

    // Get clock speed and compute divider for 50 hz
	servomotor.clk = clock_get_hz(clk_sys);
	servomotor.div = servomotor.clk / (20000 * 50);

	// Check div is in range
	if ( servomotor.div < 1 ){
		servomotor.div = 1;
	}
	if ( servomotor.div > 255 ){
		servomotor.div = 255;
	}

    servomotor.config = pwm_get_default_config();
	pwm_config_set_clkdiv(&servomotor.config, (float)servomotor.div);

	// Set wrap so the period is 20 ms
	pwm_config_set_wrap(&servomotor.config, 20000);

	// Load the configuration
	pwm_init(servomotor.slice_num, &servomotor.config, false);

	pwm_set_enabled(servomotor.slice_num, true);

    return servomotor;
}

/**
 * @brief rotates the motor shaft to a desired angle
 * @param servomotor pointer to servomotor object
 * @param degree value in degrees by which to rotate
*/
void set_servo_degrees(ServoMotor *servomotor, uint8_t degree){
    if (degree > 180){
		return;
	}
	if (degree < 0){
		return;
	}

	uint32_t duty = (((float)(ROTATE_180 - ROTATE_0) / 180) * degree) + ROTATE_0;
	pwm_set_gpio_level(servomotor->gpio_pin, duty);
}