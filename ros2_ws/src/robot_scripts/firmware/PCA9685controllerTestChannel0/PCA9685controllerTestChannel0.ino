#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  125
#define SERVOMAX  575
uint8_t servoIdx = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Servo Test:");
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz

  pwm.setPWM(servoIdx, 0, (SERVOMAX+SERVOMIN)/2);  // Channel 0, ~1.5ms pulse
}

void loop() {
  for (int ang = 10; ang <= 170; ang += 5) {
    pwm.setPWM(servoIdx, 0, angleToPulse(ang));
    delay(100);
  }
}

int angleToPulse(int ang){
   int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
   Serial.print("Angle: "); Serial.print(ang);
   Serial.print(" pulse: "); Serial.println(pulse);
   return pulse;
}
