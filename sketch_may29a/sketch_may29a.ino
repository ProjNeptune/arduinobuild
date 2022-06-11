#include <MPU6050.h>




#define PUL1_PIN 13
#define DIR1_PIN 12
#define PUL2_PIN 11
#define DIR2_PIN 10
#define PUL3_PIN 9
#define DIR3_PIN 8
#define PUL4_PIN 7
#define DIR4_PIN 6


int stepCount = 200;
 
void setup(){
    pinMode(PUL1_PIN, OUTPUT);
    pinMode(DIR1_PIN, OUTPUT);
    pinMode(PUL2_PIN, OUTPUT);
    pinMode(DIR2_PIN, OUTPUT);
    pinMode(PUL3_PIN, OUTPUT);
    pinMode(DIR3_PIN, OUTPUT);
    pinMode(PUL4_PIN, OUTPUT);
    pinMode(DIR4_PIN, OUTPUT);

 
    digitalWrite(DIR1_PIN, HIGH);
    digitalWrite(DIR2_PIN, HIGH);
    digitalWrite(DIR3_PIN, HIGH);
    digitalWrite(DIR4_PIN, HIGH);
}
 
void loop(){
    for(int i=0;i<stepCount;i++){
        digitalWrite(PUL1_PIN, HIGH);
        digitalWrite(PUL2_PIN, HIGH);
        digitalWrite(PUL3_PIN, HIGH);
        digitalWrite(PUL4_PIN, HIGH);
        delayMicroseconds(800);
        digitalWrite(PUL1_PIN, LOW);
        digitalWrite(PUL2_PIN, LOW);
        digitalWrite(PUL3_PIN, LOW);
        digitalWrite(PUL4_PIN, LOW);
        delayMicroseconds(800);
        
    }
    

    delay(1000);
}
 