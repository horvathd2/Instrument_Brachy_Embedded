#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

#define CLKDIV 256.0f
#define PWM_WRAP 9804
#define MIN_POS_DELTA 10

struct PWM{
    uint pin;
    uint slice_num;
    uint channel;
};
struct PID{
    long int current_pos;
    long int pos_error;
    long int pos_setpoint;
    uint32_t duty_cycle;
    uint32_t critical_delta;
    uint32_t limit;
    uint32_t maxSpeed;
    uint8_t Kp;
};
struct encoder{
    int gear_ratio;
    int ticks_per_rev;
    int encoderAB;
};
typedef struct{
    struct PWM pwm;
    struct PID pid; 
    struct encoder encoder;
    uint8_t fwdPin;
    uint8_t bwdPin; 
} Motor;

/**
 * @brief motor initialization function
 * @param fwdPin forward pin to driver 
 * @param bwdPin backwards pin to driver
 * @param encoderAB first encoder pin (the second pin must be sequential)
 * @param gear_ratio gear ratio of the motor from the datasheet
 * @param tick_per_rev number of ticks activated after a single rotation of the motor shaft 
 * @return motor structure
*/
Motor init_motor(const int fwdPin,const int bwdPin,const int encoderAB, int gear_ratio, int tick_per_rev){
   
    // initialize motor components
    Motor motor;
    motor.fwdPin = fwdPin;
    motor.bwdPin = bwdPin;
    motor.encoder.encoderAB = encoderAB;

    // initialize pins
    gpio_init(fwdPin);
    gpio_init(bwdPin);
    gpio_set_dir(fwdPin, true);
    gpio_set_dir(bwdPin, true);
    motor.encoder.gear_ratio = gear_ratio;
    motor.encoder.ticks_per_rev = tick_per_rev;
    return motor;
}

/**
 * @brief initialize PID parameters for a defined motor
 * @param motor passed by pointer, modified motor parameters
 * @param critical_delta critical error, at which the PID will slope down
 * @param limit rotational limit of the motor (mechanical constraint)
*/
void init_PID(Motor *motor,const int critical_delta, const int limit){
    motor->pid.critical_delta = critical_delta;
    motor->pid.maxSpeed = 9804;
    //motor->pid.current_pos = 0;
    motor->pid.limit = (float) limit/360 * motor->encoder.gear_ratio * motor->encoder.ticks_per_rev;
}

/**
 * @brief initializes the motor pwm, sets the slices and channels, stores them into the motor structure, and sets the pwm frequency
 * @param motor pointer to the motor object
 * @param pin the pwm pin (must be in pairs) that leads to the driver due to slices
 * @param frequency the frequency of the pwm duty cycle [0 - 9804]
*/
void init_motor_pwm(Motor *motor,const int pin, const int frequency){

    motor->pwm.pin = pin;                                   
    gpio_set_function(motor->pwm.pin, GPIO_FUNC_PWM);                           // set pin function

    motor->pwm.slice_num = pwm_gpio_to_slice_num(pin);               // get pwm slice from gpio
    motor->pwm.channel = pwm_gpio_to_channel(pin);                   // get pwm channel of the pin (1/2)

    pwm_set_clkdiv(motor->pwm.slice_num, CLKDIV);                               // set clock div
    pwm_set_wrap(motor->pwm.slice_num, PWM_WRAP);                               // set pwm wrap

    pwm_set_chan_level(motor->pwm.slice_num, motor->pwm.channel, frequency);    // set channel level
    pwm_set_enabled(motor->pwm.slice_num, true);                                // enable slice
}

/**
 * @brief inline function used by a pinter function, to move the motors, at a set speed taken from the motors struct
 * @param motor the motor that will move
*/
inline static void forward(const Motor *motor){
    //printf("moving fwd\n");
    gpio_put(motor->fwdPin,1);
    gpio_put(motor->bwdPin,0);
    pwm_set_chan_level(motor->pwm.slice_num, motor->pwm.channel, motor->pid.duty_cycle); //AICI ERA BAIUL
}

/**
 * @brief inline static function used by a function pointer, that reverses the motors
 * @param motor the motor in question
*/
inline static void backward(const Motor *motor){
    //printf("movin bwd\n");
    gpio_put(motor->fwdPin,0);
    gpio_put(motor->bwdPin,1);
    pwm_set_chan_level(motor->pwm.slice_num, motor->pwm.channel, motor->pid.duty_cycle); //AICI ERA BAIUL
}

/**
 * @brief inline static function used by function pointer, that stops the motor
 * @param motor the motor inb question
*/
inline static void stop(const Motor *motor){
    gpio_put(motor->fwdPin,0);
    gpio_put(motor->bwdPin,0);
    pwm_set_chan_level(motor->pwm.slice_num,motor->pwm.channel,0);
    //printf("im on stop\n");
}

/**
 * @brief update function, needs to be called from main methon for each motor, it requires encoder ticks
 * @param motor motor in question
 * @param encoder the position of the motor given by encoder ticks
*/
void update_error(Motor *motor){
    motor->pid.pos_error = motor->pid.pos_setpoint - motor->pid.current_pos;
}

/**
 * @brief static inline function, that sets the setpoint in the motor structure
 * @param motor the motor who's setpoint will be modified
*/
static void set_motor_sp(Motor *motor,const long int SP){
    motor->pid.pos_setpoint = SP;
}

/**
 * @brief static function that computes the duty cycle for a given error, taking the critical_Delta into consideration
 * @param motor is the motor in question
*/
inline static void compute_duty(Motor *motor){
    if(motor->pid.pos_error > motor->pid.critical_delta) 
        motor->pid.duty_cycle = motor->pid.maxSpeed; //max speed in %
    else
        motor->pid.duty_cycle = ((abs(motor->pid.pos_error)/motor->pid.critical_delta)*(100));
    motor->pid.duty_cycle = motor->pid.duty_cycle/100 * 9804;    
}

/**
 * @brief function to detect motor stop conditions
 * @param motor the motor struct variable
 * @return true if motor is in stop condition 
*/
bool motor_stop_cond(const Motor motor){
    return (motor.pid.pos_error < MIN_POS_DELTA) || (abs(motor.pid.current_pos) >= motor.pid.limit);
}

/**
 * @brief function to move the motors by delta amount of ticks  
 * @param motor the motor struct 
 * @param posotion_delta position setpoint for the motor, passed by pointer
 * @param action function pointer (direction) 
 * @tparam forward()  - action function
 * @tparam backward()  - action function 
 * @tparam stop()  - action function
 * 
*/
void move_motor_inc(Motor *motor, uint32_t *position_delta,  void (*action)(Motor)){
    
    compute_duty(motor);
    (*action)(*motor);
    if(abs(motor->pid.pos_error)> *position_delta){ 
        stop(motor); 
        set_motor_sp(motor,motor->pid.current_pos);
        *position_delta = 0;
    }
    update_error(motor);    
}

/**
 * @brief converts axis degress into ticks, taking the gear ratio into account
 * @param deg the degrees of rotation
 * @param motor the motor struct in question
 * 
*/
int deg2ticks(const int deg, const Motor *motor){
    return deg/360* motor->encoder.gear_ratio * motor->encoder.ticks_per_rev;
}

/**
 * @brief converts imput percentage [0-255] into motor encoder ticks
 * @param input the input value in rage [0-255]
 * @param motor the motor struct in question
*/
static int input2ticks(const int input,const Motor *motor){
    //return 2*motor->pid.limit/255 * input - motor->pid.limit;
    return motor->pid.limit/255 * input;
}

/**
 * @brief absolute control for the motors
 * @param motor motor strcuture variable
 * @param abs_pos absolute position percentage [0-255]
 * @param enable variable that enables (>0) or disables the motor (0)
*/
void move_motor_abs(Motor *motor,const long int abs_pos, uint8_t enable){
    // CHANGE FUNCTION TO INT AND RETURN 0 WHILE MOVING (FWN OR BWD) AND RETURN 1 IF STOPPED (DESTINATION REACHED)?
    if(enable){
        set_motor_sp(motor,input2ticks(abs_pos,motor));
        //printf("ticks %d \n", input2ticks(abs_pos,motor));
        update_error(motor);
        //printf("err0r %d \n", motor->pid.pos_error);
        compute_duty(motor);
        //printf("duty %d \n", motor->pid.duty_cycle);
        if(motor->pid.pos_error > MIN_POS_DELTA){
            backward(motor);
            //printf("%d ticks, %d err, %d sp\n ", motor->pid.current_pos, motor->pid.pos_error, motor->pid.pos_setpoint);
        }
        else if (motor->pid.pos_error < -MIN_POS_DELTA) forward(motor);
        else{ 
            stop(motor); 
        }
    }
}


