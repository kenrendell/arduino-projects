#define BLYNK_TEMPLATE_ID "TMPL6KAppJF8V"
#define BLYNK_TEMPLATE_NAME "Builtin LED"
#define BLYNK_AUTH_TOKEN "tc5Dvr46kqZginwk3s3OXRZIb2pL5hw0"

/*
 * Libraries:
 * - "Blynk" by Volodymyr Shymanskyy (IoT platform)
 * - "ADS1X15" by Rob Tillaart (ADC converter)
 * - "Adafruit SSD1306" by Adafruit (OLED display)
 * - "arduinoFFT" by Enrique Condes (fast fourier transform)
 *
 * For Blynk, see https://github.com/blynkkk/blynk-library/tree/master/examples
 */
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <ADS1X15.h>
#include <Adafruit_SSD1306.h>
#include <arduinoFFT.h>

const byte LED = 2;

// ADS1115 sampling configuration
const uint16_t ADS1115_SPS[] = {8, 16, 32, 64, 128, 250, 475, 860};
const uint16_t SAMPLES = 1024; // must be a power of 2
const uint8_t SPS_INDEX = 7;
const double SAMPLING_RATE = (double) ADS1115_SPS[SPS_INDEX];
const double SAMPLING_PERIOD = 1000000 / SAMPLING_RATE; // sampling period in microseconds

// Current sensor sensitivity.
// ACS712 (5A) = 185 mV/A
// ACS712 (20A) = 100 mV/A
// ACS712 (30A) = 66 mV/A
const double CURRENT_SENSITIVITY = 100; // in millivolts per ampere (mV/A).

// Voltage sensor sensitivity.
// sensitivity = (RMS output voltage) / (RMS input voltage)
// RMS input voltage can be measured using a true-RMS multimeter.
// RMS output voltage can be measured using a true-RMS multimeter.
const double VOLTAGE_SENSITIVITY = 3.35; // in millivolts per volt (mV/V).

// OLED configuration
const unsigned int SCREEN_WIDTH = 128; // OLED display width, in pixels
const unsigned int SCREEN_HEIGHT = 64; // OLED display height, in pixels
const unsigned int SCREEN_ADDRESS = 0x3C;

double voltage_samples[SAMPLES]; // real part of voltage samples
double im_voltage_samples[SAMPLES]; // imaginary part of voltage samples
double current_samples[SAMPLES];
double voltage_zero_point;
double current_zero_point;

// char ssid[] = "ZTE_2.4G_rjp36G_EXT";
// char pass[] = "Pi4c7Q2V";
// char ssid[] = "WiWater";
// char pass[] = "LnK|1029_3847_56-(0)";
char ssid[] = "...";
char pass[] = "3dotsdotdotdot";

BlynkTimer timer;

// Declaration for an SSD1306 OLED display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Initialize ADS1115 on I2C bus 1 with default address 0x48
ADS1115 ADS(0x48);

arduinoFFT FFT;

BLYNK_CONNECTED() {
	// Request Blynk server to re-send latest values for all pins.
	Blynk.syncAll();
}

double get_samples(double data[], uint16_t samples, bool verbose) {
	double sum = 0;
	uint16_t count = 0;
	uint32_t start_time, prev_time, present_time, total_time;

	start_time = prev_time = micros();

	// Get samples
	if (data != NULL) while (count < samples) {
		present_time = micros(); // overflows after approximately 70 minutes.
		if (present_time - prev_time >= SAMPLING_PERIOD) {
			prev_time = present_time;
			data[(count++)] = (double) ADS.getValue();
		}
	} else while (count < samples) {
		present_time = micros(); // overflows after approximately 70 minutes.
		if (present_time - prev_time >= SAMPLING_PERIOD) {
			prev_time = present_time;
			sum += (double) ADS.getValue();
			count++;
		}
	}

	// Can be used to check if the data is evenly sampled.
	// Total time must be approximately equal to the number of samples multiplied by the sampling period.
	// In other words, total_time must be approximately equal to (samples * SAMPLING_PERIOD).
	total_time = micros() - start_time;

	if (verbose) {
		Serial.print(F("Samples: "));
		Serial.print(count);
		Serial.print(F(" / "));
		Serial.println(samples);
		Serial.print(F("Sampling rate: "));
		Serial.print(SAMPLING_RATE, 0);
		Serial.println(F(" samples per second"));
		Serial.print(F("Total sampling time: "));
		Serial.print(total_time);
		Serial.println(F(" us")); // microseconds
		Serial.print(F("Ideal total sampling time: "));
		Serial.print((double) samples * SAMPLING_PERIOD, 0);
		Serial.println(F(" us")); // microseconds
	}

	// If no passed data, return the average reading.
	return ((data == NULL) ? floor(sum / (double) count) : 0.0);
}

void convert(double data[], uint16_t samples, double offset, double sensitivity) {
	// Remove DC offset and convert the results to
	// their proper unit based on the sensitivity value.
	for (uint16_t i = 0; i < samples; ++i) {
		uint16_t steps = (uint16_t) (data[i] - offset);
		data[i] = (ADS.toVoltage(steps) * 1000.0) / sensitivity;
	}
}

double get_frequency(bool verbose) {
	if (verbose) Serial.println(F("[FREQUENCY]"));

	// Switch to voltage sensor ADC channel.
	ADS.readADC_Differential_1_3();

	// Get voltage sensor readings.
	get_samples(voltage_samples, SAMPLES, verbose);

	// Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows.
	for (uint16_t i = 0; i < SAMPLES; ++i) im_voltage_samples[i] = 0.0;

	// Find the frequency from voltage samples.
	FFT = arduinoFFT(voltage_samples, im_voltage_samples, SAMPLES, SAMPLING_RATE);
	FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); // weigh data
	FFT.Compute(FFT_FORWARD); // compute FFT
	FFT.ComplexToMagnitude(); // compute magnitudes

	// Highest fundamental frequency
	return FFT.MajorPeak();
}

uint16_t get_voltage_current_samples(double frequency, bool verbose) {
	if (frequency < 1) return 0; // minimum frequency
	if (frequency > SAMPLING_RATE) return 0; // maximum frequency

	uint16_t count = 0, samples;
	uint32_t start_time, prev_time, present_time, total_time;
	double period = (1000000.0 / frequency); // in microseconds

	samples = (uint16_t) floor(SAMPLING_RATE / frequency); // samples in one period

	if (verbose) Serial.println(F("[VOLTAGE-CURRENT]"));

	start_time = prev_time = micros();

	// Switch to voltage sensor ADC channel.
	ADS.requestADC_Differential_1_3();

	// Prevent reading previous values from the other ADC channel.
	while (count < samples) {
		present_time = micros(); // overflows after approximately 70 minutes.
		if (present_time - prev_time >= SAMPLING_PERIOD) {
			prev_time = present_time;
			ADS.getValue(); count++;
		}
	} count = 0;

	// Get voltage samples in one period.
	while (count < samples) {
		present_time = micros(); // overflows after approximately 70 minutes.
		if (present_time - prev_time >= SAMPLING_PERIOD) {
			prev_time = present_time;
			voltage_samples[(count++)] = (double) ADS.getValue();
		}
	} count = 0;

	// Align next samples to the next period.
	while ((prev_time = micros()) - start_time < period * 2.0);

	// Switch to current sensor ADC channel.
	ADS.requestADC_Differential_0_3();

	// Prevent reading previous values from the other ADC channel.
	while (count < samples) {
		present_time = micros(); // overflows after approximately 70 minutes.
		if (present_time - prev_time >= SAMPLING_PERIOD) {
			prev_time = present_time;
			ADS.getValue(); count++;
		}
	} count = 0;

	// Get current samples in one period.
	while (count < samples) {
		present_time = micros(); // overflows after approximately 70 minutes.
		if (present_time - prev_time >= SAMPLING_PERIOD) {
			prev_time = present_time;
			current_samples[(count++)] = (double) ADS.getValue();
		}
	}

	// Can be used to check if the data is evenly sampled.
	// Total time must be approximately equal to the number of samples multiplied by the sampling period.
	// In other words, total_time must be approximately equal to (2 * samples * SAMPLING_PERIOD).
	total_time = micros() - start_time;

	if (verbose) {
		Serial.print(F("Sampling rate: "));
		Serial.print(SAMPLING_RATE, 0);
		Serial.println(F(" samples per second"));
		Serial.print(F("Total sampling time: "));
		Serial.print(total_time);
		Serial.println(F(" us")); // microseconds
		Serial.print(F("Ideal total sampling time: "));
		Serial.print((double) (4 * samples) * SAMPLING_PERIOD, 0);
		Serial.println(F(" us")); // microseconds
	}

	// Remove the DC values from the samples.
	convert(voltage_samples, samples, voltage_zero_point, VOLTAGE_SENSITIVITY);
	convert(current_samples, samples, current_zero_point, CURRENT_SENSITIVITY);

	return samples;
}

double get_rms_value(double data[], uint16_t samples) {
	double sum = 0;
	for (uint16_t i = 0; i < samples; ++i) sum += (data[i] * data[i]);
	return sqrt(sum / (double) samples);
}

double get_power(double voltage[], double current[], uint16_t samples) {
	double sum = 0;
	for (uint16_t i = 0; i < samples; ++i) sum += (voltage[i] * current[i]);
	return sum / (double) samples;
}

void blink(byte pin, unsigned int time_delay, unsigned int count) {
	for (unsigned int i = 0; i < count * 2; ++i) {
		digitalWrite(pin, (i % 2 == 0 ? LOW : HIGH));
		if (i + 1 < count * 2) delay(time_delay);
	}
}

void measure() {
	double frequency = get_frequency(false);
	double voltage = 0.0, current = 0.0, power = 0.0;
	uint8_t count = 3;

	for (uint8_t i = 0; i < count; ++i) {
		uint16_t samples = get_voltage_current_samples(frequency, false);
		voltage += get_rms_value(voltage_samples, samples);
		current += get_rms_value(current_samples, samples);
		power += get_power(voltage_samples, current_samples, samples);
	}

	voltage /= count;
	current /= count;
	power /= count;
	double apparent_power = voltage * current;
	double power_factor = power / apparent_power;
	
	OLED.clearDisplay();
	OLED.setCursor(0, 0);
	OLED.print(F("Frequency: "));
	OLED.print(frequency, 3);
	OLED.println(F(" Hz"));
	OLED.print(F("Voltage: "));
	OLED.print(voltage, 3);
	OLED.println(F(" V"));
	OLED.print(F("Current: "));
	OLED.print(current, 3);
	OLED.println(F(" A"));
	OLED.print(F("Power: "));
	OLED.print(power, 3);
	OLED.println(F(" W"));
	OLED.print(F("Power VA: "));
	OLED.print(apparent_power, 3);
	OLED.println(F(" VA"));
	OLED.print(F("Power Factor: "));
	OLED.println(power_factor, 3);
	OLED.display();

	Blynk.virtualWrite(V0, power);
	Blynk.virtualWrite(V1, power_factor);
	Blynk.virtualWrite(V2, apparent_power);
	Blynk.virtualWrite(V3, frequency);
	Blynk.virtualWrite(V4, voltage);
}

void setup() {
	Serial.begin(115200);
	while (!Serial) Serial.println(F("Serial is ready!"));

	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);

	// Initialize OLED display
	if (OLED.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
		delay(2000); // wait for initializing
		OLED.clearDisplay();
		OLED.setTextSize(1);
		OLED.setTextColor(WHITE);
		OLED.setCursor(0, 0);
		OLED.display();
	} else for (;;) { // error
		blink(LED, 300, 2); delay(2000);
		Serial.println(F("SSD1306 allocation failed!"));
	}

	ADS.begin();

	// Initialize ADC converter
	if (ADS.isConnected()) {
		ADS.setWireClock(400000);
		ADS.setGain(0); // 6.144 V
		ADS.setDataRate(SPS_INDEX);
		ADS.setMode(0); // continuous mode
	} else for (;;) { // error
		blink(LED, 300, 3); delay(2000);
		Serial.println(F("ADS1115 not connected!"));
	}

	// Find the zero point of voltage sensor.
	ADS.readADC_Differential_1_3();
	voltage_zero_point = get_samples(NULL, SAMPLES, false);

	// Find the zero point of current sensor.
	ADS.readADC_Differential_0_3();
	current_zero_point = get_samples(NULL, SAMPLES, false);

	// Initialize Blynk server
	Blynk.config(BLYNK_AUTH_TOKEN);
	Blynk.connectWiFi(ssid, pass);
	timer.setInterval(1000L, measure);
}

void loop() {
	Blynk.run();
	timer.run();
}
