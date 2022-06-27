#include <Servo.h>

#include <IBusBM.h>

IBusBM ibusRc;

Servo servo;

void setup() {
  servo.attach(22);
  IBus.begin(Serial);
}

void loop() {
  int val;
  val = IBus.readchannel(0);
  servo.write(val);
  delay(10);
}