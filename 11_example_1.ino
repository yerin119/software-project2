#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficient to convert duration to distance

#define _EMA_ALPHA 0.35    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// global variables
float dist_ema = 0;       // EMA filtered distance (mm)
float dist_prev = _DIST_MAX; // previous raw distance output from USS (mm)
unsigned long last_sampling_time;  // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 

  myservo.attach(PIN_SERVO);
  myservo.write(90); // 서보를 중립 위치에 설정

  // initialize USS related variables
  dist_ema = _DIST_MAX; // 초기 거리

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw;
  
  // wait until next sampling time
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  // 초음파 센서로 거리 측정
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // 범위 필터 적용: 거리가 지정된 범위를 벗어나면 이전 값을 유지
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
    dist_raw = dist_prev;        // 최대 범위 초과 시 이전 값 유지
    digitalWrite(PIN_LED, HIGH); // LED OFF
  } else if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;        // 최소 범위 미만 시 이전 값 유지
    digitalWrite(PIN_LED, HIGH); // LED OFF
  } else { 
    dist_prev = dist_raw;        // 유효한 범위 내에서 거리 갱신
    digitalWrite(PIN_LED, LOW);  // LED ON
  }

  // EMA 필터 적용
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_ema;

  // 거리 값에 따른 서보 각도 제어
  int servo_angle = calculate_servo_angle(dist_ema);
  myservo.write(servo_angle); // 서보 모터 각도 설정

  // Serial 모니터에 거리 값 출력
  Serial.print("Raw Distance: ");  Serial.print(dist_raw);
  Serial.print(", EMA Distance: "); Serial.print(dist_ema);
  Serial.print(", Servo Angle: "); Serial.println(servo_angle);

  // 마지막 샘플링 시간 갱신
  last_sampling_time += INTERVAL;
}

// 초음파 센서로 거리 측정
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  float duration = pulseIn(ECHO, HIGH, TIMEOUT);
  return duration * SCALE; // unit: mm
}

// 거리에 따라 서보 각도를 계산하는 함수
int calculate_servo_angle(float distance) {
  if (distance <= _DIST_MIN) {
    return 0;  // 18cm 이하에서는 0도
  } else if (distance >= _DIST_MAX) {
    return 180;  // 36cm 이상에서는 180도
  } else {
    // 18cm ~ 36cm 사이에서는 거리에 비례하여 각도 계산
    return map(distance, _DIST_MIN, _DIST_MAX, 0, 180);
  }
}
