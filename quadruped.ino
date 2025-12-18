#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// servo consts
constexpr uint16_t SERVOMIN = 110; // min pulse width microsec
constexpr uint16_t SERVOMAX = 510;   // max pulse width microsec
constexpr uint16_t FREQ = 50; // servo frequency in Hz

// leg consts
constexpr float LINK1_LENGTH = 7.7f;
constexpr float LINK2_LENGTH = 11.5f;

// leg state
float footX = 0;
float footY = 0;


// servo map
uint8_t hipChannel = 0; // driver channel
uint8_t kneeChannel = 1;


void setup() {

  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(25000000); // 25MHz internal clock
  pwm.setPWMFreq(FREQ);
  delay(10);

}

uint16_t angleToPulse(float angle) {
  float degrees = angle * 180.0 / M_PI;
  return map(degrees, 0, 180, SERVOMIN, SERVOMAX);
}




void loop() {

  if (Serial.available() > 0) {
    int read = Serial.parseInt();
    degrees = read;
    // if (read != 0) degrees = read;
  }

  uint16_t pulselen = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulselen);
  delay(10);
  
}
