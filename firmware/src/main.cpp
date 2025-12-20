#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// servo consts
constexpr uint16_t SERVOMIN = 140; // min pulse width microsec
constexpr uint16_t SERVOMAX = 560; // max pulse width microsec
constexpr uint8_t FREQ = 50;	   // servo frequency in Hz

// servo map
uint8_t hipChannel = 0; // driver channel
uint8_t kneeChannel = 1;

// leg consts
constexpr float LINK1_LENGTH = 8.0f;
constexpr float LINK2_LENGTH = 11.5f;

// leg state
float footX = 0;
float footY = 0;

uint16_t degreesToPulse(float angle);
void setFootPosition(float x, float y);

void setup()
{

	Serial.begin(115200);

	delay(1000);
	Serial.println("Initialized. \n");

	pwm.begin();
	pwm.setOscillatorFrequency(27000000); // 25MHz internal clock but ~27MHz actual reccomended by manufacturer
	pwm.setPWMFreq(FREQ);

	Serial.println("Initialized. \n");

	pwm.setPWM(hipChannel, 0, degreesToPulse(90));
	pwm.setPWM(kneeChannel, 0, degreesToPulse(90));
	setFootPosition(LINK1_LENGTH, LINK2_LENGTH);
	delay(10);
}

void loop()
{

	if (Serial.available() > 0)
	{
		String input = Serial.readStringUntil('\n');
		input.trim();

		int separator = input.indexOf(':');
		if (separator > 0)
		{
			String xStr = input.substring(0, separator);
			String yStr = input.substring(separator + 1);
			float x = xStr.toFloat();
			float y = yStr.toFloat();
			setFootPosition(x, y);
		}
	}

	delay(10);
}

uint16_t degreesToPulse(float angle)
{

	angle = constrain(angle, 0, 180);

	// Serial.println(angle);
	return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

/*
  leg kinematic frame:
  origin: hip joint
  +x axis horizontal
  +y axis upward
  angles ccw from +x

  nb: servo coords don't match frame and joint specific dir./angle corrections are applied hwen mapping ik angles to servo angles
*/
void setFootPosition(float x, float y)
{
	float r2 = x * x + y * y; // distance to end effector from origin
	float innerAngle = (r2 - LINK1_LENGTH * LINK1_LENGTH - LINK2_LENGTH * LINK2_LENGTH) / (2 * LINK1_LENGTH * LINK2_LENGTH);

	innerAngle = constrain(innerAngle, -1.0f, 1.0f);

	// knee angle (IK)
	// 0 degs = leg fully extended, + = flexion
	float kneeRadians = acos(innerAngle);

	// hip angle (IK)
	// angle of femur ccw from +X axis
	float hipRadians = atan2(y, x) - atan2(LINK2_LENGTH * sin(kneeRadians), LINK1_LENGTH + LINK2_LENGTH * cos(kneeRadians));

	float kneeDegrees = kneeRadians * 180.0 / M_PI;

	// hip servo is mounted such that servo 90deg = leg pointing down
	// + rotation is CW
	// ik angle is sign inverted such that CCW -> CW

	float hipDegrees = -hipRadians * 180.0f / M_PI;

	kneeDegrees = constrain(kneeDegrees, 25, 160);
	pwm.setPWM(hipChannel, 0, degreesToPulse(hipDegrees));
	pwm.setPWM(kneeChannel, 0, degreesToPulse(kneeDegrees));

	Serial.print("Angle1: ");
	Serial.print(hipDegrees);
	Serial.print(" Angle2: ");
	Serial.println(kneeDegrees);
	Serial.print("X: ");
	Serial.print(x);
	Serial.print(" Y: ");
	Serial.println(y);
}
