#include <Servo.h>
#define PIN_SERVO 10
#define _SERVO_SPEED_A 3
#define _SERVO_SPEED_B 0.3

Servo myservo;
unsigned long MOVING_TIME = 3000;
unsigned long moveStartTime;
int startAngle = 0;
int stopAngle = 180;
int pos = 0;
int flag;
int angle;

void setup() {
  myservo.attach(PIN_SERVO); 
  myservo.write(0);
  moveStartTime = millis();
}

void loop() {
    unsigned long progress = millis() - moveStartTime;
    myservo.write(90);

    progress %= MOVING_TIME;
    long angle = map(progress, 0, MOVING_TIME, startAngle, stopAngle);
    
    for(pos=0; pos<=180; pos+=1){
      myservo.write(pos);
      delay(_SERVO_SPEED_A*110);
    }
      for(pos=180; pos>=0; pos-=1){
      myservo.write(pos);
      delay(_SERVO_SPEED_B*11100);
    }
}
