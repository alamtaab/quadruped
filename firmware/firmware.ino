#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int degrees = 0;
// servo consts
constexpr uint16_t SERVOMIN = 110;  // min pulse width microsec
constexpr uint16_t SERVOMAX = 510;  // max pulse width microsec
constexpr uint8_t FREQ = 50;        // servo frequency in Hz

// servo map
uint8_t hipChannel = 0;  // driver channel
uint8_t kneeChannel = 1;

// leg consts
constexpr float LINK1_LENGTH = 7.7f;
constexpr float LINK2_LENGTH = 11.5f;

// leg state
float footX = 0;
float footY = 0;


void setup() {

  Serial.begin(115200);

  delay(1000);
  Serial.println("Initialized. \n");

  pwm.begin();
  pwm.setOscillatorFrequency(25000000);  // 25MHz internal clock
  pwm.setPWMFreq(FREQ);

  Serial.println("Initialized. \n");
  
  pwm.setPWM(hipChannel, 0, angleToPulse(90));
  pwm.setPWM(kneeChannel, 0, angleToPulse(90));
  setFootPosition(LINK1_LENGTH, LINK2_LENGTH);
  delay(10);
}

uint16_t angleToPulse(float angle) {
  float degrees = angle * 180.0 / M_PI;
  return map(degrees, 0, 180, SERVOMIN, SERVOMAX);
}

void setFootPosition(float x, float y) {
  y *= -1;
  float r2 = x * x + y * y;  //distance to end effector from origin
  float innerAngle = (r2 - LINK1_LENGTH * LINK1_LENGTH - LINK2_LENGTH * LINK2_LENGTH) / (2 * LINK1_LENGTH * LINK2_LENGTH);

  if (innerAngle > 1.0f) innerAngle = 1.0f;
  if (innerAngle < -1.0f) innerAngle = -1.0f;

  float jointAngle2 = -acos(innerAngle);                                                                                     // knee up link 2 angle from axis of link 1
  float jointAngle1 = atan2(y, x) - atan2(LINK2_LENGTH * sin(jointAngle2), LINK1_LENGTH + LINK2_LENGTH * cos(jointAngle2));  //link 1 angle from positive x

  pwm.setPWM(hipChannel, 0, angleToPulse(jointAngle1));
  pwm.setPWM(kneeChannel, 0, angleToPulse(jointAngle2));
}

void loop() {

  // if (Serial.available()) {
  //   String line = Serial.readStringUntil('\n');
  //   float x, y;
  //   if (sscanf(line.c_str(), "%f %f", & x, & y) == 2) {
  //     setFootPosition(x, y);
  //   }
  // }




  // uint16_t pulselen = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
  // pwm.setPWM(1, 0, pulselen);
  if (Serial.available() > 0) {
    float read = Serial.parseFloat();
    //  degrees = read;
    if (read != 0.0) setFootPosition(0, read);
  }

  // pwm.setPWM(hipChannel, 0, angleToPulse(1.57));
  // pwm.setPWM(kneeChannel, 0, angleToPulse(1.57) );
  // Serial.println("Test. \n");


  delay(10);
}
