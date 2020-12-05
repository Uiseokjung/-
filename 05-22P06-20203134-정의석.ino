#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// configurable parameters
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_TARGET 255
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 410 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.5 // EMA weight if new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.

#define _DUTY_MIN 750 // 
#define _DUTY_NEU 1470// 
#define _DUTY_MAX 1750 // 

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 30
 
// global variables 
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev, dist_ema, alpha; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
float pterm;
int duty_curr, duty_target;
Servo myservo;

void setup() {
// initialize GPIO pins
  Serial.begin(57600);
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  dist_ema = 0;
  alpha = _DIST_ALPHA;
  
 // initialize last sampling time
  last_sampling_time = 0;
}

// volt to value(for dist)
float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  // wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

  float dist_raw = ir_distance();
  float dist_mod = 1.1 * (dist_raw) + 20.2;
// get a distance reading from the USS
  dist_ema = dist_ema*(1-alpha)+alpha*dist_mod;


  // 25.5cm를 기준으로 각도 전환
  if(dist_ema >= 255) {
    myservo.writeMicroseconds(750);
  }
  else {
    myservo.writeMicroseconds(1750);
  }
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    

  }
