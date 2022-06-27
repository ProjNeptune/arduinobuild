#include <Wire.h>
#include <MPU6050.h>
#include <SpeedyStepper.h>
#include <Servo.h>
#include <IBusBM.h>

const int motor1_step_pin = 22;
const int motor1_direction_pin = 23;
const int motor2_step_pin = 24;
const int motor2_direction_pin = 25;
const int motor3_step_pin = 26;
const int motor3_direction_pin = 27;
const int motor4_step_pin = 28;
const int motor4_direction_pin = 29;

SpeedyStepper stepper1;
SpeedyStepper stepper2;
SpeedyStepper stepper3;
SpeedyStepper stepper4;


IBusBM ibus;


int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = Ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
 
// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void setup() {
    ibus.begin(serial1);
    Serial1.begin(115200);
    stepper1.connectToPins(motor1_step_pin, motor1_direction_pin);
    stepper2.connectToPins(motor2_step_pin, motor2_direction_pin);
    stepper3.connectToPins(motor3_step_pin, motor3_direction_pin);
    stepper4.connectToPins(motor4_step_pin, motor4_direction_pin);

}

void loop() {
     
}