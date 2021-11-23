#include <Servo.h>

#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9
#define _DUTY_MIN 550
#define _DUTY_MAX 2400
const int duty_neu = 1475;


static long apt = 0;
int a;
int b;
Servo myservo;
#define interval 2
unsigned long past_mil;


int fc = 7; 
float dt = interval/1000.0;
float lambda = 2*PI*fc*dt;
float filter = 0.0, prev = 0.0;

void setup() {
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(10);
  myservo.writeMicroseconds(duty_neu);
  delay(1000);
  Serial.begin(115200);

  a = 75;
  b = 240;
}

float ir_distance(void){
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  unsigned long cur_mil = 0;
  unsigned long mil = millis();
  if (mil != past_mil) {
    cur_mil = mil-past_mil;
    past_mil = mil;
  }

  apt -= cur_mil;

  if (apt <= 0) {
    apt += interval;
    filter = lambda / (1 + lambda) * dist_cali + 1 / (1 + lambda) * prev; // filtering
    prev = filter;
    Serial.print("min:0,max:800,dist:");
    Serial.print(raw_dist);
    Serial.print(",dist_cali:");
    Serial.print(dist_cali);
    Serial.print(",fileterd:");
    Serial.println(filter);
  }
  
  if(raw_dist > 255 && raw_dist < 325) {
    digitalWrite(PIN_LED, 0);
  }
  else {
    digitalWrite(PIN_LED, 255);
  }
  if(raw_dist > 255 ){
    myservo.writeMicroseconds(1375);
  }
  else if(raw_dist < 355){
    myservo.writeMicroseconds(1675);
  }
  delay(20);
}
