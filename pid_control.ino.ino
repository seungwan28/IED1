#include <Servo.h>
// Arduino pin assignment
#define PIN_LED 9         
#define PIN_SERVO 10    
#define PIN_IR A0     

// Framework setting
#define _DIST_TARGET 250  
#define _DIST_MIN 100  
#define _DIST_MAX 410                

// Distance sensor
#define _DIST_ALPHA 0.5  

// Servo ran
#define _DUTY_MIN 1600  
#define _DUTY_NEU 1900  
#define _DUTY_MAX 2400                

// Servo speed control
#define _SERVO_ANGLE 30   
#define _SERVO_SPEED 300 

// Event periods
#define _INTERVAL_DIST 20   
#define _INTERVAL_SERVO 20  
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 2.0 
#define _KI 0.5   
#define _KD 45.0

//#define _INTERVAL_DIST 30  
#define DELAY_MICROS  1000
#define EMA_ALPHA 0.35     
float ema_dist=0;            
float filtered_dist;           
float samples_num = 3;   

// Servo instance
Servo myservo; 

// Distance sensor
float dist_target; 
float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo,
last_sampling_time_serial;


bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval;
int duty_target, duty_curr; 
// PID variables
float cur, prev, control, pterm, dterm, iterm; 


void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO); 
  
// initialize global variables
duty_target, duty_curr = _DUTY_MIN;
//duty_target, duty_curr = _DUTY_NEU;
last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial = millis();

prev, cur = _DUTY_NEU; 
dist_raw, dist_ema  = _DIST_MIN; 
pterm = dterm = iterm = 0; 

// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize serial port
  Serial.begin(57600); 

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN)/(float)(_SERVO_ANGLE) * (_SERVO_SPEED /1000.0)*_INTERVAL_SERVO; 
  

}
  

void loop() {
    
    unsigned long time_curr = millis(); 

    if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
    }
    if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
    }
    if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
    }

    if(event_dist) {
        event_dist = false; 
        // get a distance reading from the distance sensor
        dist_raw = filtered_ir_distance(); 
        //dist_ema = _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema; 
        //filtered_dist = filtered_ir_distance();

        // PID control logic
        cur = _DIST_TARGET - dist_raw; 
        pterm = _KP * cur; 
        dterm = _KD * (cur - prev);
        iterm += _KI * cur;
        //control = _KP * pterm + _KI * iterm +  _KD * dterm;
        if (abs(iterm) > 40) iterm = 0;
        control = pterm + dterm + iterm;
        //duty_target = f(duty_neutral, control)
        //duty_target = _DUTY_NEU * (1 + control);

        duty_target = _DUTY_NEU + control;
        
                
  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
        if(duty_target < _DUTY_MIN){
            duty_target = _DUTY_MIN;
        }
        if(duty_target > _DUTY_MAX){
            duty_target = _DUTY_MAX;
        }
       prev = cur;
  }
  
    if(event_servo) {
        event_servo = false; 

        if(duty_target > duty_curr) 
        { 
            duty_curr += duty_chg_per_interval;
        if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else
    {
        duty_curr -= duty_chg_per_interval;
        if(duty_curr < duty_target) duty_curr = duty_target;
    }

        // update servo position
        myservo.writeMicroseconds((int)duty_curr);
        event_servo = false; 
    }
  
    if(event_serial) {
        event_serial = false; 
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
        event_serial = false; 
    }
}

float ir_distance(void){ // return value unit: mm
    float val; 
    float volt = float(analogRead(PIN_IR)); 
    val = ((6762.0/(volt - 9.0)) - 4.0) * 10.0; 
    return val;
}
float under_noise_filter(void){ 
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++){
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}
