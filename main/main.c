/*
    BRACHITHERAPY INSTRUMENT PI PICO W DRIVERS
*/

#include <string.h>
#include <time.h>
#include <math.h>
#include <inttypes.h>

#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "encoder.pio.h"
#include "motor_control.c"
#include "micro_servo.c"
#include "pin_def.h"

#define RX_BUFFER_SIZE      1024
#define TX_BUF_SIZE         1024

uint16_t values[] = {0,5000};
uint8_t stopall = 0;
uint8_t homing = 0;
uint8_t next_needle = 0;

Motor motor_insertion;
Motor motor_repository;

ServoMotor servo_gripper;
ServoMotor servo_funnel;

char rx_buffer[RX_BUFFER_SIZE];
char *com, *pos, *vel;

void core1_entry(){ 
    //INITIALIZE MICRO SERVO MOTORS
    servo_gripper = servo_init(SERVO_GRIPPER_NEEDLE);
    servo_funnel = servo_init(SERVO_GRIPPER_FUNNEL);

    set_servo_degrees(&servo_gripper,0);
    set_servo_degrees(&servo_funnel,0);

    while(true) {
        scanf("%1024s", rx_buffer);

        rx_buffer[strcspn(rx_buffer, "\n")] = 0;

        com = strtok(rx_buffer, "/");
        pos = strtok(NULL, "/");
        vel = strtok(NULL, "/");

        if(strcmp(com, "of") == 0) {
            set_servo_degrees(&servo_funnel,180);
        }else if(strcmp(com, "cf") == 0) {
            set_servo_degrees(&servo_funnel,0);
        }else if(strcmp(com, "og") == 0) {
            set_servo_degrees(&servo_gripper,180);
        }else if(strcmp(com, "cg") == 0) {
            set_servo_degrees(&servo_gripper,0);
        }else if(strcmp(com, "oa") == 0) {
            set_servo_degrees(&servo_funnel,180);
            set_servo_degrees(&servo_gripper,180);
        }else if(strcmp(com, "hi") == 0) {
            homing = 1; 
        }else if(strcmp(com, "hr") == 0) {
            //HOME 
        }else if(strcmp(com, "mp") == 0) {
            values[0] = atoi(pos);
            stopall = 0;
        }else if(strcmp(com, "nn") == 0){
            next_needle = 1;
        }else if(strcmp(com, "ba") == 0){
            
        }else if((strcmp(com, "st") == 0) | (strcmp(com, "di") == 0)){
            stopall = 1;
        }
        
        sleep_ms(10);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(250);
    if (cyw43_arch_init()) {
        return -1;
    }

    //LAUNCH CORE 1
    multicore_launch_core1(core1_entry);

    // Set encoder pins and state machines
    PIO  pio2 = pio1;
    const uint sm1 = 0;
    const uint sm2 = 1;

    uint offset2 = pio_add_program(pio2, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio2, sm1, offset2, PIN_AB1, 0);
    quadrature_encoder_program_init(pio2, sm2, offset2, PIN_AB2, 0);  

    motor_insertion = init_motor(PIN_B1_1,PIN_B2_1,PIN_AB1,300,12); //U3
    motor_repository = init_motor(PIN_A1_1,PIN_A2_1,PIN_AB2,300,12); //U4
 
    init_motor_pwm(&motor_insertion,PWM_B1,9804/2);
    init_motor_pwm(&motor_repository,PWM_A1,9804/2);
   
    init_PID(&motor_insertion,500,180);
    init_PID(&motor_repository,500,180);

    // INITIALIZE SENSORS
    gpio_init(SENSOR1);
    gpio_init(SENSOR2);
    gpio_set_dir(SENSOR1, GPIO_IN);
    gpio_set_dir(SENSOR2, GPIO_IN);
    gpio_pull_down(SENSOR1);
    gpio_pull_down(SENSOR2);

    while(true){
        motor_insertion.pid.current_pos = quadrature_encoder_get_count(pio2, sm1);
        motor_repository.pid.current_pos = quadrature_encoder_get_count(pio2, sm2);

        if(!stopall) {
            if(homing){
                if(!gpio_get(SENSOR1)) move_motor_abs(&motor_insertion, 1000, 1); // backward(&motor_insertion);
                else {
                    stop(&motor_insertion); // set homing to 0
                    printf("done\n");
                    homing = 0;
                }

                //if(!gpio_get(SENSOR2)) backward(&motor_repository);
                //else stop(&motor_repository);
            }   
            else{
                if(next_needle){
                    if(!gpio_get(SENSOR2)) move_motor_abs(&motor_repository, values[1], 1);
                    else{
                        stop(&motor_repository);
                        printf("done\n");
                        next_needle = 0;
                    } 
                }
                //if(!gpio_get(SENSOR1)) move_motor_abs(&motor_insertion, values[0], 1);
                //else stop(&motor_insertion);
            }
        }else{
            stop(&motor_insertion);
            stop(&motor_repository);
        } 
       
        //itoa(motor_insertion.pid.current_pos, snum, 10);

        sleep_ms(10);
    }

    return 0;
}