#include <PinChangeInterrupt.h> /* https://github.com/NicoHood/PinChangeInterrupt */
#include <PID_v1.h>             /* https://github.com/br3ttb/Arduino-PID-Library */

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif

const unsigned long DEBOUNCE_DELAY = 50; // trigger debouncing delay (in milliseconds)

const byte TRIGGER_PIN = 3;
const byte MOT_A1_PIN = 5;
const byte MOT_A2_PIN = 6;
const byte MOT_B1_PIN = 9;
const byte MOT_B2_PIN = 11;
const byte MOT_ENABLE_PIN = 7;
const byte MOT_FAULT_PIN = 2;

// See https://github.com/NicoHood/PinChangeInterrupt/#pinchangeinterrupt-table
const byte MPU6050_INT_PIN = A1; // A1 pin in arduino nano

const double MOT_PWM_OFFSET = 105;
const double SETPOINT_OFFSET = 0;

// MPU control/status vars
bool dmp_ready = false;  // set true if DMP init was successful
uint8_t mpu_int_status;  // holds actual interrupt status byte from MPU
uint8_t dev_status;      // return status after each device operation (0 = success, !0 = error)
uint16_t packet_size;    // expected DMP packet size (default is 42 bytes)
uint16_t fifo_count;     // count of all bytes currently in FIFO
uint8_t fifo_buffer[64]; // FIFO storage buffer

volatile bool mpu_interrupt = false;     // indicates whether MPU interrupt pin has gone high

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool motor_enabled = false;

double input, output;

// Tune these 4 values
// See https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Manual_tuning
double setpoint = 1; // set the value when the robot is perpendicular to ground using serial monitor. 
double Kp = 15.0;    // (proportion)
double Ki = 100.0;   // (integral)
double Kd = 1;       // (derivative)

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

typedef struct {
	uint8_t state;
	uint8_t previous_state;
	uint8_t edge; // change state
	uint8_t edge_positive; // rising edge
	uint32_t previous_time;
} edge_t;

edge_t btn;

MPU6050 mpu;

// Trigger to stop/start (toggle) the motors with button debounce.
void trigger() {
	btn.previous_state = btn.state;
	btn.state = digitalRead(TRIGGER_PIN);
	btn.edge = btn.state ^ btn.previous_state;
	btn.edge_positive = btn.edge & btn.state;

	if (!btn.edge) return;
	if ((millis() - btn.previous_time) <= DEBOUNCE_DELAY) return;

	if (btn.edge_positive) {
		motor_enabled = !motor_enabled;
		digitalWrite(MOT_ENABLE_PIN, motor_enabled);
		Serial.println(motor_enabled);
	}

	btn.previous_time = millis();
}

// Set the current on a motor channel using PWM and directional logic.
void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN) {
	if (pwm < 0) {  // reverse speeds
		analogWrite(IN1_PIN, -pwm);
		digitalWrite(IN2_PIN, LOW);

	} else { // stop or forward
		digitalWrite(IN1_PIN, LOW);
		analogWrite(IN2_PIN, pwm);
	}
}

// Simple primitive for the motion sequence to set a speed and wait for an interval.
void spin(int pwm_A, int pwm_B) {
	if (!motor_enabled) return;
	set_motor_pwm(pwm_A, MOT_A1_PIN, MOT_A2_PIN);
	set_motor_pwm(pwm_B, MOT_B1_PIN, MOT_B2_PIN);
}

// MPU6050 interrupt detection routine
void dmpDataReady() {
	mpu_interrupt = true;
}

void setup() {
	// Join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	Serial.begin(115200);
	while (!Serial);

	// Initialize MPU6050 device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
	pinMode(MPU6050_INT_PIN, INPUT);

	// Verify MPU6050 device connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// Load and configure the MPU6050 DMP
	Serial.println(F("Initializing DMP..."));
	dev_status = mpu.dmpInitialize();

	// Supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788);

	// Make sure MPU6050 works (returns 0 if so)
	if (dev_status == 0) {
		// Turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// Enable pin change interrupt detection for MPU6050 interrupt pin
		attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(MPU6050_INT_PIN), dmpDataReady, RISING);
		mpu_int_status = mpu.getIntStatus();

		// Set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmp_ready = true;

		// Get expected DMP packet size for later comparison
		packet_size = mpu.dmpGetFIFOPacketSize();

		// Setup PID
		pid.SetMode(AUTOMATIC);
		pid.SetSampleTime(10);
		pid.SetOutputLimits(-255 + MOT_PWM_OFFSET, 255 - MOT_PWM_OFFSET); 
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(dev_status);
		Serial.println(F(")"));
	}

	// Initialise the motor input pins
	pinMode(MOT_A1_PIN, OUTPUT);
	pinMode(MOT_A2_PIN, OUTPUT);
	pinMode(MOT_B1_PIN, OUTPUT);
	pinMode(MOT_B2_PIN, OUTPUT);
	pinMode(MOT_ENABLE_PIN, OUTPUT);

	//,By default turn off both the motors
	digitalWrite(MOT_A1_PIN, LOW);
	digitalWrite(MOT_A2_PIN, LOW);
	digitalWrite(MOT_B1_PIN, LOW);
	digitalWrite(MOT_B2_PIN, LOW);
	digitalWrite(MOT_ENABLE_PIN, LOW);

	// Attach interrupt for enabling motor driver manually
	pinMode(TRIGGER_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), trigger, CHANGE);

	// Attach interrupt for detecting motor driver faults
	//pinMode(MOT_FAULT_PIN, INPUT);
	//attachInterrupt(digitalPinToInterrupt(MOT_FAULT_PIN), driver_fault_detected, FALLING);
}

void loop() {
	// If programming failed, don't try to do anything
	if (!dmp_ready) return;

	// Wait for MPU interrupt or extra packet(s) available
	while (!mpu_interrupt && fifo_count < packet_size) {
		// No mpu data - performing PID calculations and output to motors     
		if (pid.Compute()) {
			if (output > 0) spin(output + MOT_PWM_OFFSET, -(output + MOT_PWM_OFFSET));
			else if (output < 0) spin(output - MOT_PWM_OFFSET, -(output - MOT_PWM_OFFSET));
			else spin(0, 0);
		}
		
		// Print the value of Input and Output on serial monitor to check how it is working.
		Serial.print(input); Serial.print(" =>"); Serial.println(output);
	}

	// Reset interrupt flag and get INT_STATUS byte
	mpu_interrupt = false;
	mpu_int_status = mpu.getIntStatus();

	// Get current FIFO count
	fifo_count = mpu.getFIFOCount();

	// Check for overflow (this should never happen unless our code is too inefficient)
	if ((mpu_int_status & 0x10) || fifo_count == 1024) {
		// Reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));
	// Otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (mpu_int_status & 0x02) {
		// Wait for correct available data length, should be a VERY short wait
		while (fifo_count < packet_size) fifo_count = mpu.getFIFOCount();
		// Read a packet from FIFO
		mpu.getFIFOBytes(fifo_buffer, packet_size);

		// Track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifo_count -= packet_size;
		mpu.dmpGetQuaternion(&q, fifo_buffer); // get value for q
		mpu.dmpGetGravity(&gravity, &q); // get value for gravity
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // get value for ypr
		input = (ypr[2] * 180) / M_PI;
   }
}
