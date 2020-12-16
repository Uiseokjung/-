#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9  //[1692] LED 9번핀에 연결
#define PIN_SERVO 10 // [3228] 서보10핀에 연결
#define PIN_IR A0  // [3133] 적외선 센서 signal -> A0핀

// Framework setting
#define _DIST_TARGET 255  //[0028] 목표 위치가 25.5cm임을 선언
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.3   // [1628] ema 필터의 측정 보정치

// Servo range
#define _DUTY_MIN 1000
#define _DUTY_NEU 1585
#define _DUTY_MAX 2170

// Servo speed control
#define _SERVO_ANGLE 30        // [3131] servo 각도 설정
#define _SERVO_SPEED 90        // [3141] servo 속도 설정

// Event periods
#define _INTERVAL_DIST 20         // [3123] distance 측정 이벤트 주기
#define _INTERVAL_SERVO 20     // [3123] servo 조정 이벤트 주기
#define _INTERVAL_SERIAL 100   // [3123] serial 출력 이벤트 주기


// PID parameters
#define _KP 1.2
#define _KD 15
#define _KI 0.5
// IR Filtering(referenced by 박우혁)
#define _INTERVAL_DIST 30  // DELAY_MICROS * samples_num^2 의 값이 최종 거리측정 인터벌임. 넉넉하게 30ms 잡음.
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
#define EMA_ALPHA 0.3     



//////////////////////
// global variables //
//////////////////////
float dist_min, dist_max, alpha; // [3228]
// Servo instance
Servo myservo; 

// Distance sensor
float dist_target; // location to send the ball
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
float ema_dist = 0;            // EMA 필터에 사용할 변수
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
Serial.begin(57600); //[3128] 시리얼 포트 초기화

// convert angle speed into duty change per interval. ****How to?
  if(filtered_ir_distance() >= 255) {
    duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000); //[3128]
  }
  else {
    duty_chg_per_interval = 2*(_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000);
  }

}

void loop() {
/////////////////////
// Event generator // [3133] 이벤트 실행 간격 구현 
/////////////////////
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;

////////////////////
// Event handlers //
////////////////////
  if(event_dist) {
      event_dist = false; // [3133]
  // get a distance reading from the distance sensor
  

  // PID control logic
    error_curr = _DIST_TARGET - filtered_ir_distance(); 
    pterm = error_curr * _KP; //[0028] kp * 오차
    control_P = pterm;  //[0028] 제어량 계산
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
  
    last_sampling_time_dist = millis(); // [3133] 마지막 dist event 처리 시각 기록
  }
  
  if(event_servo) {
    event_servo = false; // [3133]
    // update servo position
    myservo.writeMicroseconds(duty_target);
    // adjust duty_curr toward duty_target by duty_chg_per_interval ...How to?
    

    
    last_sampling_time_servo = millis(); // [3133] 마지막 servo event 처리 시각 기록

  }
  dist_raw = ir_distance();
  filtered_dist = filtered_ir_distance();
  
  duty_curr = myservo.read();
  
  if(abs(iterm) > _ITERM_MAX) iterm = 20; 
  if(iterm > _ITERM_MAX) iterm = _ITERM_MAX; 
  if(iterm < - _ITERM_MAX) iterm = - _ITERM_MAX;
  
  if(event_serial) {
    event_serial = false; // [3133]
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
    last_sampling_time_serial = millis(); // [3133] 마지막 serial event 처리 시각 기록

  }
}

float ir_distance(void){ // return value unit: mm 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

  float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
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

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
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
