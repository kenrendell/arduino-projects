#include <Servo.h>

const float SOUND_SPEED = 0.0343; // centimeter per microsecond at 20 degrees celsius
const int LOOP_DELAY = 100; // millisecond

const int SERVO_PIN = 11; // Must be a PWM Pin
const int SERVO_TRIG_PIN = 8;
const int SERVO_ECHO_PIN = 9;

const int DISPENSER_PIN = 5;
const int DISPENSER_TRIG_PIN = 2;
const int DISPENSER_ECHO_PIN = 4;

// {{{{{ SETTINGS

/* Values: HIGH or LOW (depends on relay setup) */
byte dispenser_close_state = HIGH;

/* Unit: degree */
unsigned int servo_close_angle = 105; // Servo position where the dust-bin cap is closed.
unsigned int servo_open_angle = 0; // Servo position where the dust-bin cap is opened.

/* Unit: centimeter */
unsigned int servo_max_distance = 10;     // Maximum distance where the servo activates
unsigned int dispenser_max_distance = 10; // Maximum distance where the dispenser activates

/* Unit: millisecond (multiples of LOOP_DELAY) */
unsigned int servo_return_delay = 40;     // Delay before returning the dust-bin cap to closed position
unsigned int dispenser_spit_delay = 5;    // Delay before the dispenser spits a liquid
unsigned int dispenser_spit_duration = 5; // Duration where dispenser spits a liquid continuously after delay

// END }}}}}

bool dispenser_running = false;
unsigned int dispenser_timeout = 0;
unsigned int servo_timeout = 0;

byte dispenser_open_state;
unsigned long duration, time;
float servo_distance, dispenser_distance;

Servo servo;

float echo(int trig_pin, int echo_pin) {
	digitalWrite(trig_pin, LOW); delayMicroseconds(2);
	digitalWrite(trig_pin, HIGH); delayMicroseconds(10);
	digitalWrite(trig_pin, LOW);

	duration = pulseIn(echo_pin, HIGH, 10000);
	return duration ? (duration * SOUND_SPEED) / 2 : 0;
}

void setup() {
	pinMode(DISPENSER_PIN, OUTPUT);
	pinMode(DISPENSER_TRIG_PIN, OUTPUT);
	pinMode(DISPENSER_ECHO_PIN, INPUT);

	pinMode(SERVO_TRIG_PIN, OUTPUT);
	pinMode(SERVO_ECHO_PIN, INPUT);

	digitalWrite(DISPENSER_PIN, dispenser_close_state);
	
	servo.attach(SERVO_PIN);
	servo.write(servo_close_angle);

	servo_return_delay *= LOOP_DELAY;
	dispenser_spit_delay *= LOOP_DELAY;
	dispenser_spit_duration *= LOOP_DELAY;
	dispenser_open_state = (dispenser_close_state == LOW) ? HIGH : LOW;
}

void loop() {
	servo_distance = echo(SERVO_TRIG_PIN, SERVO_ECHO_PIN);
	dispenser_distance = echo(DISPENSER_TRIG_PIN, DISPENSER_ECHO_PIN);

	if (servo_distance && servo_distance <= servo_max_distance) {
		if (servo_timeout == 0) servo.write(servo_open_angle);
		servo_timeout = servo_return_delay;
	} else if (servo_timeout && (servo_timeout -= LOOP_DELAY) == 0) servo.write(servo_close_angle);

	if (dispenser_distance && dispenser_distance <= dispenser_max_distance) {
		if (! dispenser_running) {
			dispenser_running = true;
			dispenser_timeout = dispenser_spit_delay + dispenser_spit_duration;
		} else dispenser_timeout -= dispenser_timeout ? LOOP_DELAY : 0;
		digitalWrite(DISPENSER_PIN, (dispenser_timeout && dispenser_timeout <= dispenser_spit_duration) ? dispenser_open_state : dispenser_close_state);
	} else {
		dispenser_running = false;
		dispenser_timeout = 0;
		digitalWrite(DISPENSER_PIN, dispenser_close_state);
	}
	delay(LOOP_DELAY);
}
