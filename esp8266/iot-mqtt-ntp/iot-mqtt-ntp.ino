/* For more info about ESP8266 on Arduino, see
   https://arduino-esp8266.readthedocs.io/en/latest/
   https://github.com/esp8266/Arduino/tree/master/cores/esp8266

   For more info about MQTT setup on ESP8266, see
   https://docs.emqx.com/en/cloud/latest/connect_to_deployments/esp8266.html

   For NTP time synchronization, see
   https://github.com/esp8266/Arduino/blob/master/libraries/esp8266/examples/NTP-TZ-DST/NTP-TZ-DST.ino
   https://github.com/esp8266/Arduino/blob/master/cores/esp8266/TZ.h
   https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
   https://www.iana.org/time-zones
*/

///////////////////////////////

#include <FS.h>           /* https://arduino-esp8266.readthedocs.io/en/latest/filesystem.html */
#include <LittleFS.h>     /* https://arduino-esp8266.readthedocs.io/en/latest/filesystem.html */
#include <ArduinoJson.h>  /* https://github.com/bblanchon/ArduinoJson */

#include <ESP8266WiFi.h>  /* https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/readme.html */
#include <WiFiManager.h>  /* https://github.com/tzapu/WiFiManager */

/* See https://github.com/marvinroger/async-mqtt-client/blob/develop/docs/index.md */
#include <Ticker.h>
#include <AsyncMqttClient.h> /* https://github.com/marvinroger/async-mqtt-client/tree/develop */

#include <coredecls.h>  // settimeofday_cb()
#include <PolledTimeout.h>
#include <time.h>      // time() ctime()

///////////////////////////////

const char* EMPTY_STR = "";

const char* NTP_SERVER = "time.nist.gov"; // default NTP server
const char* CONFIG_FILE = "/config.json";
const uint16_t MQTT_PORT = 1883;

const byte TRIGGER_PIN = D1;

const unsigned int TIMEOUT = 180; // configuration portal timeout (in seconds)
const unsigned int TRIGGER_DELAY = 1000; // trigger delay (in milliseconds) before starting configuration portal
const unsigned long DEBOUNCE_DELAY = 50; // trigger debouncing delay (in milliseconds)

static time_t now;

// This uses the PolledTimeout library to allow an action to be performed every 60 seconds
static esp8266::polledTimeout::periodicMs show_time_now(60000);

unsigned int trigger_count = 0; // trigger count within a trigger delay, resets to zero after the trigger delay
unsigned long portal_start_time = millis();
unsigned long trigger_start_time = millis();
bool portal_running = false;
bool new_config = false;
bool triggered = false;

// Our configuration structure.
// For PEM format, see https://en.wikipedia.org/wiki/Privacy-Enhanced_Mail
struct Config {
  char wifi_ap_ssid[50]; // WiFiManager access point SSID
  char wifi_ap_password[50]; // WiFiManager access point PSK
  char posix_timezone[50]; // for possible time-zone values, see https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
  char mqtt_broker[50]; // MQTT broker (server)
  char mqtt_username[50]; // MQTT username for authentication
  char mqtt_password[50]; // MQTT password for authentication
};

typedef struct {
    uint8_t state;
    uint8_t previous_state;
    uint8_t edge; // change state
    uint8_t edge_positive; // rising edge
    uint32_t previous_time;
} edge_t;

edge_t btn;

Config config;

WiFiManager wm;

// id/name, placeholder/prompt, default, length
WiFiManagerParameter custom_wifi_ap_ssid("wifi_ap_ssid", "WiFi Access-Point SSID", "", sizeof(config.wifi_ap_ssid));
WiFiManagerParameter custom_wifi_ap_password("wifi_ap_password", "WiFi Access-Point password", "", sizeof(config.wifi_ap_password));
WiFiManagerParameter custom_posix_timezone("posix_timezone", "Posix Timezone", "", sizeof(config.posix_timezone));
WiFiManagerParameter custom_mqtt_broker("mqtt_broker", "MQTT broker (server)", "", sizeof(config.mqtt_broker));
WiFiManagerParameter custom_mqtt_username("mqtt_username", "MQTT username", "", sizeof(config.mqtt_username));
WiFiManagerParameter custom_mqtt_password("mqtt_password", "MQTT password", "", sizeof(config.mqtt_password));

// MQTT client initialization
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

// WiFi events initialization
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

void mqtt_set_credentials() {
  if (strcmp(config.mqtt_broker, EMPTY_STR) == 0) return;
  mqttClient.setServer(config.mqtt_broker, MQTT_PORT);

  if ((strcmp(config.mqtt_username, EMPTY_STR) != 0) && (strcmp(config.mqtt_password, EMPTY_STR) != 0)) {
    mqttClient.setCredentials(config.mqtt_username, config.mqtt_password);
  } else mqttClient.setCredentials(nullptr, nullptr);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  if (strcmp(config.mqtt_broker, EMPTY_STR) != 0) {
    Serial.print(F("MQTT broker: "));
    Serial.println(config.mqtt_broker);
    mqttClient.connect();
  }
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe("test/lol", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  mqttClient.publish("test/lol", 0, true, "test 1");
  Serial.println("Publishing at QoS 0");
  uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  Serial.print("Publishing at QoS 1, packetId: ");
  Serial.println(packetIdPub1);
  uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  Serial.print("Publishing at QoS 2, packetId: ");
  Serial.println(packetIdPub2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void time_is_set(bool from_sntp) {
  Serial.print(F("time was sent! from_sntp="));
  Serial.println(from_sntp);
  show_time();
}

void init_time() {
  if (strcmp(config.posix_timezone, EMPTY_STR) != 0) {
    Serial.print(F("Timezone: "));
    Serial.println(config.posix_timezone);
    configTime(config.posix_timezone, NTP_SERVER);
  }
}

void show_time() {   // This function is used to print stuff to the serial port, it's not mandatory
  now = time(nullptr);      // Updates the 'now' variable to the current time value

  // human readable
  Serial.print("ctime: ");
  Serial.print(ctime(&now));
  // Here is one example showing how you can get the current month
  Serial.print("current month: ");
  Serial.println(localtime(&now)->tm_mon);
  // Here is another example showing how you can get the current year
  Serial.print("current year: ");
  Serial.println(localtime(&now)->tm_year);
  // Look in the printTM method to see other data that is available
  Serial.println();
}

// Loads the configuration from a file
bool load_config(const char* filename, Config& config) {
  // Open file for reading
  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.println(F("Failed to open config file"));
    return false;
  }

  // Allocate a temporary JsonDocument
  JsonDocument doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
	  Serial.println(F("Failed to read file, using default configuration"));
	  return false;
  }

  // Copy values from the JsonDocument to the Config
  strlcpy(config.wifi_ap_ssid, doc[custom_wifi_ap_ssid.getID()] | "", sizeof(config.wifi_ap_ssid)); // WiFiManager access-point name
  strlcpy(config.wifi_ap_password, doc[custom_wifi_ap_password.getID()] | "", sizeof(config.wifi_ap_password)); // WiFiManager access-point PSK
  strlcpy(config.posix_timezone, doc[custom_posix_timezone.getID()] | "", sizeof(config.posix_timezone)); // NTP time zone
  strlcpy(config.mqtt_broker, doc[custom_mqtt_broker.getID()] | "", sizeof(config.mqtt_broker)); // MQTT broker (server)
  strlcpy(config.mqtt_username, doc[custom_mqtt_username.getID()] | "", sizeof(config.mqtt_username)); // MQTT username for authentication
  strlcpy(config.mqtt_password, doc[custom_mqtt_password.getID()] | "", sizeof(config.mqtt_password)); // MQTT password for authentication

  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
  return true;
}

// Saves the configuration to a file
bool save_config(const char* filename, const Config& config) {
  // Open file for writing
  File file = LittleFS.open(filename, "w");
  if (!file) {
	Serial.println(F("Failed to open config file for writing"));
    return false;
  }

  // Allocate a temporary JsonDocument
  JsonDocument doc;

  // Set the values in the document
  doc[custom_wifi_ap_ssid.getID()] = (strcmp(custom_wifi_ap_ssid.getValue(), EMPTY_STR) == 0) ? config.wifi_ap_ssid : custom_wifi_ap_ssid.getValue(); // WiFiManager access-point name
  doc[custom_wifi_ap_password.getID()] = (strcmp(custom_wifi_ap_password.getValue(), EMPTY_STR) == 0) ? config.wifi_ap_password : custom_wifi_ap_password.getValue(); // WiFiManager access-point PSK
  doc[custom_posix_timezone.getID()] = (strcmp(custom_posix_timezone.getValue(), EMPTY_STR) == 0) ? config.posix_timezone : custom_posix_timezone.getValue(); // NTP time zone
  doc[custom_mqtt_broker.getID()] = (strcmp(custom_mqtt_broker.getValue(), EMPTY_STR) == 0) ? config.mqtt_broker : custom_mqtt_broker.getValue(); // MQTT broker (server)
  doc[custom_mqtt_username.getID()] = (strcmp(custom_mqtt_username.getValue(), EMPTY_STR) == 0) ? config.mqtt_username : custom_mqtt_username.getValue(); // MQTT username for authentication
  doc[custom_mqtt_password.getID()] = (strcmp(custom_mqtt_password.getValue(), EMPTY_STR) == 0) ? config.mqtt_password : custom_mqtt_password.getValue(); // MQTT password for authentication

  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
	return false;
  }

  // Close the file
  file.close();
  return true;
}

/* See interrupts, https://arduino-esp8266.readthedocs.io/en/latest/reference.html#interrupts */
IRAM_ATTR void count_trigger() {
	if (triggered) return;

	btn.previous_state = btn.state;
	btn.state = digitalRead(TRIGGER_PIN);
	btn.edge = btn.state ^ btn.previous_state;
	btn.edge_positive = btn.edge & btn.state;

	if (!btn.edge) return;
	if ((millis() - btn.previous_time) <= DEBOUNCE_DELAY) return;

	if (btn.edge_positive) {
		trigger_count += 1;
		trigger_start_time = millis(); // reset trigger delay
        Serial.print(F("Trigger: "));
		Serial.println(trigger_count);
	}

	btn.previous_time = millis();
}

void stop_config_portal() {
	Serial.println(F("<<< Stopping Config Portal"));
	portal_running = false;
	wm.stopConfigPortal();
}

bool wifi_connect(bool auto_connect) {
    if (strcmp(config.wifi_ap_ssid, EMPTY_STR) != 0) {
      if (strlen(config.wifi_ap_password) < 8) {
        Serial.println(F("Access-Point set password must be greater than 7 characters!"));
        return auto_connect ? wm.autoConnect(config.wifi_ap_ssid) : wm.startConfigPortal(config.wifi_ap_ssid);
      }
      return auto_connect ? wm.autoConnect(config.wifi_ap_ssid, config.wifi_ap_password) : wm.startConfigPortal(config.wifi_ap_ssid, config.wifi_ap_password);
    }
    return auto_connect ? wm.autoConnect() : wm.startConfigPortal();
}

// NOTE: No proper termination of non-blocking ondemand config portal
void do_wifi_manager() {
	if (!triggered && trigger_count > 0) {
		if ((millis() - trigger_start_time) > TRIGGER_DELAY) triggered = true;
	}

	// Is auto timeout portal running?
	if (portal_running) {
		wm.process(); // do processing

		// Check for timeout
        if (!wm.getConfigPortalActive()) {
            Serial.println(F("<<< Stopping Config Portal"));
            portal_running = false;
            new_config = true;
        } else if ((millis() - portal_start_time) > (TIMEOUT * 1000)) {
			stop_config_portal();
            new_config = true;
		} else if (triggered && trigger_count == 2) { // force stop
			stop_config_portal();
            new_config = true;
			triggered = false;
			trigger_count = 0;
		}

	}

    if (new_config) {
      Serial.println(F("Saving configuration..."));
      if (save_config(CONFIG_FILE, config)) {
        Serial.println(F("Configuration saved"));
        Serial.println(F("Loading configuration..."));
        if (load_config(CONFIG_FILE, config)) Serial.println(F("Configuration loaded"));
        else Serial.println(F("Loading configuration failed!"));
      } else Serial.println(F("Saving configuration failed!"));

      mqtt_set_credentials();
      mqttClient.disconnect();

      new_config = false;
      init_time();
    }

	if (!triggered) return;

	if (trigger_count == 3) { // reset WiFi settings
		if (portal_running) stop_config_portal();
        memset(config.wifi_ap_ssid, 0, sizeof(config.wifi_ap_ssid));
        memset(config.wifi_ap_password, 0, sizeof(config.wifi_ap_password));

        Serial.println(F("Saving configuration..."));
        if (save_config(CONFIG_FILE, config)) Serial.println(F("Configuration saved"));
        else Serial.println(F("Saving configuration failed!"));

		wm.resetSettings();
	} else if ((trigger_count == 1) && (!portal_running)) { // Is configuration portal requested?
		Serial.println(F("Starting Config Portal >>>"));

		// Start configuration portal
        wifi_connect(false);

		triggered = false;
		portal_running = true;
		portal_start_time = millis();
	}

	triggered = false;
	trigger_count = 0;
}

void setup() {
	WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

	Serial.begin(115200);
	while (!Serial);
    Serial.println();

	if (!LittleFS.begin()) {
		Serial.println(F("Failed to mount file system"));
        ESP.restart();
	}

	// WiFi debug information
	wm.setDebugOutput(true); // WiFiManager verbose
	//WiFi.printDiag(Serial); // print WiFi credentials

	wm.addParameter(&custom_wifi_ap_ssid);
	wm.addParameter(&custom_wifi_ap_password);
	wm.addParameter(&custom_posix_timezone);
	wm.addParameter(&custom_mqtt_broker);
	wm.addParameter(&custom_mqtt_username);
	wm.addParameter(&custom_mqtt_password);

    Serial.println("Hello world");

	wm.setClass("invert"); // dark theme for captive portal
	wm.setScanDispPerc(true); // display percentages instead of graphs for RSSI
    wm.setConfigPortalBlocking(false); // non-blocking mode to avoid blocking
    wm.setBreakAfterConfig(true); // exit after config, even if connection is unsuccessful

    if (!LittleFS.exists(CONFIG_FILE)) save_config(CONFIG_FILE, config);

    Serial.println(F("Loading configuration..."));
    if (load_config(CONFIG_FILE, config)) {
      Serial.println(F("Configuration loaded"));

      custom_wifi_ap_ssid.setValue(config.wifi_ap_ssid, sizeof(config.wifi_ap_ssid)); // WiFiManager access-point name
      custom_wifi_ap_password.setValue(config.wifi_ap_password, sizeof(config.wifi_ap_password)); // WiFiManager access-point PSK
      custom_posix_timezone.setValue(config.posix_timezone, sizeof(config.posix_timezone)); // NTP time zone
      custom_mqtt_broker.setValue(config.mqtt_broker, sizeof(config.mqtt_broker)); // MQTT broker (server)
      custom_mqtt_username.setValue(config.mqtt_username, sizeof(config.mqtt_username)); // MQTT username for authentication
      custom_mqtt_password.setValue(config.mqtt_password, sizeof(config.mqtt_password)); // MQTT password for authentication
    }

    // install callback - called when settimeofday is called (by SNTP or user)
    // once enabled (by DHCP), SNTP is updated every hour by default
    // ** optional boolean in callback function is true when triggered by SNTP **
    settimeofday_cb(time_is_set);
    init_time(); // X.509 validation requires synchronization time

    wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
    wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onSubscribe(onMqttSubscribe);
    mqttClient.onUnsubscribe(onMqttUnsubscribe);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.onPublish(onMqttPublish);

    mqtt_set_credentials();

	// Automatically connect using saved credentials if they exist.
    // If connection fails it starts an access point with the specified name.
    if (!wifi_connect(true)) portal_running = true;

	btn.state = digitalRead(TRIGGER_PIN);
	btn.previous_time = millis();

	// Attach interrupt to configuration portal button
	pinMode(TRIGGER_PIN, INPUT); // Configuration portal button
	attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), count_trigger, CHANGE);
}

void loop() {
	do_wifi_manager();
    if (show_time_now) show_time();
	// if (WiFi.status() != WL_CONNECTED) Serial.println(F("Not Connected!"));
	// else Serial.println(F("Connected!"));
}
