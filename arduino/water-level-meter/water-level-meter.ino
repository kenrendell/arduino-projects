// WATER LEVEL SENSOR

#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// Arduino pins
const byte ECHO_PIN = 4;
const byte TRIG_PIN = 5;
const byte POT_PIN = 0;
const byte BUTTON_PIN = 2;
const byte SPEAKER_PIN = 6;

const float SOUND_SPEED = 0.0343; // centimeter per microsecond at 20 degrees celsius
const float MAX_DISTANCE = 300; // distance in centimeters, maximum distance that sensor can detect.
const unsigned long MAX_TIME = (unsigned long) (2 * MAX_DISTANCE / SOUND_SPEED); // time in microseconds 

boolean update = false, tone_playing = false;
float container_height, full_height, distance, offset;
byte bar_level, level, prev_level = 1;
unsigned int loop_count = 0;

// Progress bar characters
byte bar_char[5][8];

// LCD I2C connections for arduino uno and nano
// SDA -> A4
// SCL -> A5
LiquidCrystal_I2C lcd(0x27, 16, 2);

void set_update() {
	digitalWrite(LED_BUILTIN, (update = true));
}

float echo(byte trig_pin, byte echo_pin, unsigned int n) {
	float distance = 0;
	unsigned long duration;
	unsigned int m = 0;

	for (unsigned int i = 0; i < n; ++i) {
		digitalWrite(trig_pin, LOW); delayMicroseconds(2);
		digitalWrite(trig_pin, HIGH); delayMicroseconds(10);
		digitalWrite(trig_pin, LOW);

		duration = pulseIn(echo_pin, HIGH, MAX_TIME);

		if (duration <= 0) m += 1;
		else distance += ((float) duration * SOUND_SPEED) / 2;
	}

	// returns distance in centimeters
	return distance > 0 ? distance / (n - m) : 0;
}

void setup() {
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);
	pinMode(BUTTON_PIN, INPUT);
	pinMode(SPEAKER_PIN, OUTPUT);

	attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), set_update, RISING);

	// Initialize LCD
	lcd.init(); lcd.clear(); lcd.backlight();

	// Create all progress bar characters.
	for (byte i = 0, b = 0b11111; i < 5; ++i, b <<= 1) {
		for (byte j = 0; j < 8; ++j) bar_char[i][j] = b;
		lcd.createChar(5 - i, bar_char[i]);
	}

	// Get the stored container height value.
	EEPROM.get(0, container_height);
}

void loop() {
	if (update) { // Calibrate the container height.
		container_height = echo(TRIG_PIN, ECHO_PIN, 10);
		EEPROM.put(0, container_height);
		delay(200); // ignore bounces after pressing the button.
		digitalWrite(LED_BUILTIN, (update = false));
	}

	// If the measurements are invalid, set the default container height value.
	if (container_height <= 0) container_height = 10;

	// Determine the current water level.
	distance = echo(TRIG_PIN, ECHO_PIN, 5);
	offset = (float) map(analogRead(POT_PIN), 0, 1023, 0, 100) * container_height / 100;
	full_height = container_height - offset;

	// Get the current water level percentage.
	if (distance && distance >= offset && (distance - offset) <= full_height)
		level = (byte) (100 - ((distance - offset) / full_height) * 100);
	else level = (distance && distance < offset) ? 100 : 0;

	if (level >= 85) { // Warning sound for critical water levels.
		if (!tone_playing && (loop_count %= 2) == 0) {
			tone(SPEAKER_PIN, 300);
			digitalWrite(LED_BUILTIN, (tone_playing = true));
		} else if (tone_playing && (loop_count %= 3) == 0) {
			noTone(SPEAKER_PIN);
			digitalWrite(LED_BUILTIN, (tone_playing = false));
		} loop_count += 1;
	} else {
		if (tone_playing) {
			noTone(SPEAKER_PIN);
			digitalWrite(LED_BUILTIN, (tone_playing = false));
		} loop_count = 0;
	}

	// Display measurements
	if (level != prev_level) {
		lcd.clear(); lcd.setCursor(0, 0);
		lcd.print("Level: "); lcd.print(level); lcd.print("%");

		lcd.setCursor(0, 1);
		bar_level = map(level, 0, 100, 0, 16 * 5);
		for (byte i = 0; i < bar_level / 5; ++i) lcd.write((byte) 5);
		if (bar_level % 5 > 0) lcd.write(bar_level % (byte) 5);
	} prev_level = level;

	delay(250);
}
