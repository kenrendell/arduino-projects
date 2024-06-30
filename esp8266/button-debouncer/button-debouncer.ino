/* For more info about ESP8266 on Arduino, see
   https://arduino-esp8266.readthedocs.io/en/latest/
*/

const uint8_t BUTTON_PIN = D1;
const uint32_t DEBOUNCE_DELAY = 50;

uint32_t edge_positive_count = 0;
uint32_t edge_negative_count = 0;

typedef struct {
	uint8_t state;
	uint8_t previous_state;
	uint8_t edge; // change state
	uint8_t edge_positive; // rising edge
	uint8_t edge_negative; // falling edge
	uint32_t previous_time;
} edge_t;

edge_t btn;

/* See interrupts, https://arduino-esp8266.readthedocs.io/en/latest/reference.html#interrupts */
IRAM_ATTR void edge_detect() {
	btn.previous_state = btn.state;
	btn.state = digitalRead(BUTTON_PIN);
	btn.edge = btn.state ^ btn.previous_state;
	btn.edge_positive = btn.edge & btn.state;
	btn.edge_negative = btn.edge & btn.previous_state;

	if (!btn.edge) return;
	if ((millis() - btn.previous_time) <= DEBOUNCE_DELAY) return;

	if (btn.edge_positive) {
		Serial.print(F("Button pressed down! "));
		Serial.println(edge_positive_count++);
	} else if (btn.edge_negative) {
		Serial.print(F("Button released! "));
		Serial.println(edge_negative_count++);
	}

	btn.previous_time = millis();
}

void setup() {
	Serial.begin(115200);
	while (!Serial) Serial.println(F("Serial is ready!"));

	pinMode(BUTTON_PIN, INPUT);
	btn.state = digitalRead(BUTTON_PIN);
	btn.previous_time = millis();

	// Attach interrupt for edge detection
	attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), edge_detect, CHANGE);
}

void loop() {
	// Your code here
}
