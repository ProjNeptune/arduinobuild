//Libraries needed:
#include <Wire.h>
#include <MPU6050.h>
#include <SpeedyStepper.h>

//pin assignments
//there's sensors and stepper motors (8 for 4 steppers, 2 for sensors)

const int MOTOR_1_STEP_PIN = 13;
const int MOTOR_1_DIRECTION_PIN = 12;
const int MOTOR_2_STEP_PIN = 11;
const int MOTOR_2_DIRECTION_PIN = 10;
const int MOTOR_3_STEP_PIN = 9;
const int MOTOR_3_DIRECTION_PIN = 8;
const int MOTOR_4_STEP_PIN = 7;
const int MOTOR_4_DIRECTION_PIN = 6;

MPU6050 mpu;