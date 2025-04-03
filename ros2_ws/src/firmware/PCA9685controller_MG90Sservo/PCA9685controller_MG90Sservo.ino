#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Default I2C address

#define SERVOMIN  150  // Min pulse length (MG90S ~0°)
#define SERVOMAX  600  // Max pulse length (MG90S ~180°)

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // Set PWM frequency to 50Hz (standard for servos)
}

void loop() {
  // Move servo on channel 0 from 0° to 180°
  for (int pulselen = SERVOMIN; pulselen <= SERVOMAX; pulselen += 10) {
    pwm.setPWM(0, 0, pulselen);
    delay(20);
  }
  delay(1000);

  // Move back from 180° to 0°
  for (int pulselen = SERVOMAX; pulselen >= SERVOMIN; pulselen -= 10) {
    pwm.setPWM(0, 0, pulselen);
    delay(20);
  }
  delay(1000);
}
