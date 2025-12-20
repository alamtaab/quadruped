#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// int angle = 0;
// servo consts
constexpr uint16_t SERVOMIN = 140;  // min pulse width microsec
constexpr uint16_t SERVOMAX = 560;  // max pulse width microsec
constexpr uint8_t FREQ = 50;        // servo frequency in Hz

// servo map
uint8_t hipChannel = 0;  // driver channel
uint8_t kneeChannel = 1;

// leg consts
constexpr float LINK1_LENGTH = 8.0f;
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

  angle *= 180.0 / M_PI;
  angle = constrain(angle, 25, 160);

  Serial.println(angle);
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
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
  pwm.setPWM(kneeChannel, 0, angleToPulse(constrain(jointAngle2, 0.43633, 2.7925)));

  Serial.print("Angle1: ");
  Serial.print(jointAngle1);
  Serial.print(" Angle2: ");
  Serial.println(jointAngle2);
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.println(y);
}

void loop() {


  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int separator = input.indexOf(':');
    if (separator > 0) {
      String xStr = input.substring(0, separator);
      String yStr = input.substring(separator + 1);
      float x = xStr.toFloat();
      float y = yStr.toFloat();
      setFootPosition(x, y);
    }
  }


  delay(10);
}
