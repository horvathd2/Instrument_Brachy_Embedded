
#ifndef PIN_DEF
#define PIN_DEF

// This is where all the hardware pins are defined

// ENCODER PINS
// Base pin to connect the A phase of the encoder.
// The B phase must be connected to the next pin
#define PIN_AB1             16  //U13   B = 17
#define PIN_AB2             20  //U2    B = 21

//PWM PINS
#define PWM_B1              0   //U13 insertion motor
#define PWM_A1              5   //U2  repository motor

//MOTOR PINS
#define PIN_B1_1            1   //U13
#define PIN_B2_1            2   //U13

#define PIN_A1_1            3   //U2
#define PIN_A2_1            4   //U2

//SENSOR PINS
#define SENSOR1             18  //SENS1
#define SENSOR2             19  //SENS2

//PWM GRIPPERS
#define SERVO_GRIPPER_NEEDLE  12  //S_GR
#define SERVO_GRIPPER_FUNNEL  13  //S_FN

#endif