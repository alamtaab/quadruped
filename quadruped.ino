#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 110 // min pulse width microsec
#define SERVOMAX 510   // max pulse width microsec

#define FREQ 50 // servo frequency in Hz

uint8_t channel = 0; // driver channel
int degrees = 90;




void setup() {

  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(25000000); // 25MHz internal clock
  pwm.setPWMFreq(FREQ);
  delay(10);

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
