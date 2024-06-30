#include <Servo.h>

const byte SERVO_PIN = 9; // PWM
const byte RGB_PIN[3] = { 3, 5, 6 }; // PWM
const unsigned int LOOP_DELAY = 10;

unsigned int servo_step_delay = 10;
unsigned int rgb_step_delay = 4;

byte rgb[3], rand_rgb[3], servo_position, servo_increment = 1;
unsigned int rgb_timeout, servo_timeout;

Servo servo;

void setup() {
	for (byte i = 0; i < 3; ++i) {
		pinMode(RGB_PIN[i], OUTPUT);
		rgb[i] = rand_rgb[i] = 0;
	} randomSeed(analogRead(0));

	servo.attach(SERVO_PIN);
	servo.write(0);

	servo_timeout = (servo_step_delay *= LOOP_DELAY);
	rgb_timeout = (rgb_step_delay *= LOOP_DELAY);
}

void loop() {
	delay(LOOP_DELAY);
	if ((rgb_timeout -= rgb_timeout ? LOOP_DELAY : 0) == 0) {
		for (byte i = 0; i < 3; ++i) {
			if (rgb[i] == rand_rgb[i]) rand_rgb[i] = random(256);
			if (rgb[i] != rand_rgb[i]) analogWrite(RGB_PIN[i], (rgb[i] += (rgb[i] < rand_rgb[i]) ? 1 : -1));
		} rgb_timeout = rgb_step_delay;
	}
	if ((servo_timeout -= servo_timeout ? LOOP_DELAY : 0) == 0) {
		servo.write((servo_position += servo_increment));
		if (servo_position % 180 == 0) servo_increment = -servo_increment;
		servo_timeout = servo_step_delay;
	}
}
