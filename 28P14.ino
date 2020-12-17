#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9  
#define PIN_SERVO 10 
#define PIN_IR A0  

// Framework setting
#define _DIST_TARGET 255 
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.3  

// Servo range
#define _DUTY_MIN 1000
#define _DUTY_NEU 1585
#define _DUTY_MAX 2170

// Servo speed control
#define _SERVO_ANGLE 30        
#define _SERVO_SPEED 90

// Event periods
#define _INTERVAL_DIST 20         
#define _INTERVAL_SERVO 20     
#define _INTERVAL_SERIAL 100   


// PID parameters
#define _KP 1.2
#define _KD 15
#define _KI 0.5
// IR Filtering
#define _INTERVAL_DIST 30  
#define DELAY_MICROS  1500 
#define EMA_ALPHA 0.3     



//////////////////////
// global variables //
//////////////////////
float dist_min, dist_max, alpha; 
// Servo instance
Servo myservo; 

// Distance sensor
float dist_target; 
float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control_P, control_D, control_I, pterm, dterm, iterm;
float _ITERM_MAX = 40;
//IR Filtering
float ema_dist = 0;            
float filtered_dist;
float samples_num = 3;

void setup() {
// initialize GPIO pins for LED and attach servo 
 pinMode(PIN_LED,OUTPUT);
 digitalWrite(PIN_LED, 1);
 myservo.attach(PIN_SERVO); 

// move servo to starting position
 myservo.writeMicroseconds(_DUTY_NEU);

// initialize global variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_raw = 0.0;
  dist_ema = 0;
  alpha = _DIST_ALPHA;
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
  iterm = 0;


// initialize serial port
Serial.begin(57600); 

// convert angle speed into duty change per interval. ****How to?
  if(filtered_ir_distance() >= 255) {
    duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000); 
  }
  else {
    duty_chg_per_interval = 2*(_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000);
  }

}

void loop() {
/////////////////////
// Event generator //  
/////////////////////
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;

////////////////////
// Event handlers //
////////////////////
  if(event_dist) {
      event_dist = false; 
  // get a distance reading from the distance sensor
  

  // PID control logic
    error_curr = _DIST_TARGET - filtered_ir_distance(); 
    pterm = error_curr * _KP; 
    control_P = pterm;  
    dterm = _KD*(error_curr - error_prev);
    control_D = dterm;
    iterm += _KI * error_curr;
    control_I = iterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control_P + control_D;

  // [3133] keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;

  // update error_prev
    error_prev = error_curr;
  
    last_sampling_time_dist = millis();
  }
  
  if(event_servo) {
    event_servo = false; 
    // update servo position
    myservo.writeMicroseconds(duty_target);
    // adjust duty_curr toward duty_target by duty_chg_per_interval ...How to?
    

    
    last_sampling_time_servo = millis();

  }
  dist_raw = ir_distance();
  filtered_dist = filtered_ir_distance();
  
  duty_curr = myservo.read();
  
  if(abs(iterm) > _ITERM_MAX) iterm = 20; 
  if(iterm > _ITERM_MAX) iterm = _ITERM_MAX; 
  if(iterm < - _ITERM_MAX) iterm = - _ITERM_MAX;
  
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(filtered_dist);
    Serial.print(",T");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
    last_sampling_time_serial = millis(); 

  }
}

float ir_distance(void){ 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
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
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist + 45;
}
