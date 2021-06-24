#include "FS.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

#include <DHT.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DHTTYPE DHT11
#define DHTPin 4      // D2
#define SOIL_TEMPERATURE 5 // D1

#define MOISTURE A0  // ADC0

#define RELAY_PIN 14    // D5

#define LED_WIFI 16    // D0
#define LED_MQTT 0    // D3


#define wifi_ssid "wifi_name" // Wifi Name
#define wifi_password "12345678" // Wifi Password

#define mqtt_server "mqtt.eclipse.org" // Mqtt address
#define mqtt_port 8883 // Mqtt Port
#define mqtt_username "mqtt_user" // Mqtt User
#define mqtt_password "mqtt_password" // Mqtt Password

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

BearSSL::WiFiClientSecure espClient;
PubSubClient client(espClient);
DHT dht(DHTPin, DHTTYPE);

OneWire soilTempWire(SOIL_TEMPERATURE);
DallasTemperature soilTemp(&soilTempWire);

/*
 * Set NodeMCU Token
 */
const String mcu_token = "SAMPLE_TOKEN";

/*
 * MQTT subscribe topic
 * Do not change this
 */
const String humidity_topic = "sensor.dht.humidity/" + mcu_token;
const String temperature_topic = "sensor.dht.temperature/" + mcu_token;
const String soil_moisture_topic = "sensor.soil.moisture/" + mcu_token;
const String soil_temperature_topic = "sensor.soil.temperature/" + mcu_token;
const String action_topic = "esp.action/" + mcu_token;

/*
 * Current dht-11 data
 * Do not change this
 */
float Temperature;
float Humidity;

/*
 * Set a condition of soil-moisture
 */
const int AirValue = 850;
const int WaterValue = 250;

/*
 * Current soil-moisture data
 * Do not change this
 */
int soilMoistureValue = 0;
int soilmoisturepercent = 0;

/*
 * Current relay status
 * Do not change this
 */
boolean isRelayRunning = false;

/*
 * Current relay active
 * Do not change this
 */
int currentRelayTimer = 0;

/*
 * Set maximum attemp wifi or mqtt disconnect into restart devices
 */
const int maxAttemp = 3;

/*
 * Current maximum attemp wifi or mqtt disconnect
 * Do not change this
 */
int currentAttemp = 0;

/*
 * Set a unique NodeMCU connect ID in MQTT Broker
 */
String clientId = "ESP8266Client-" + String(random(0xffff), HEX);

// Looping check
long lastMsg = 0;

/*
 * Set duration pump active  
 */
const int RELAY_MAX_ACTIVE = 60 * 1000; // 60 seconds

/*
 * Set interval nodemcu collect a data sensor and send them into MQTT Broker
 */
const int MQTT_COLLECT_SEND = 30 * 1000; // 30 seconds

/*
 * Set looping Arduino function
 */
const int LOOP_DELAY = 100; // 0.1 second

/*
 * Storing a pumping action data
 * Do not change anything this
 */
int ACTION_DATA[2] = {0, 0};

void callback(char* topic, byte* payload, unsigned int length) {
  boolean isError = false;
  String fullPayloadJSON = String((char*)payload);

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  StaticJsonDocument<128> docJSON;

  DeserializationError error = deserializeJson(docJSON, fullPayloadJSON);
  if (error) {
    isError = true;
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
  }

  if (!isError) {
    int dataAction = docJSON["data"][0];
    int dataFrom = docJSON["data"][1];

    ACTION_DATA[0] = dataAction;
    ACTION_DATA[1] = dataFrom;

    if (dataAction == 1) {
      Serial.println("Relay: Trigger");
      if (!isRelayRunning) {
        isRelayRunning = true;
        digitalWrite(RELAY_PIN, HIGH);
        currentRelayTimer = 0;
      } else {
        Serial.println("Relay: Already trigger");
      }
    } else if (dataAction == 2) {
      if (isRelayRunning) {
        Serial.println("Relay: Forced turn off");
        digitalWrite(RELAY_PIN, LOW);
        isRelayRunning = false;
        currentRelayTimer = 0;

        int actionParam[2] = {0, ACTION_DATA[1]};
        client.publish(String(action_topic).c_str(), String(generateJSONPayload(action_topic, actionParam)).c_str());
      }
    }

  }

  Serial.print("Heap: "); Serial.println(ESP.getFreeHeap());
  Serial.println();
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_MQTT, OUTPUT);

  pinMode(RELAY_PIN, OUTPUT);

  Serial.begin(115200);
  //  Serial.setDebugOutput(true);
  dht.begin();
  soilTemp.begin();
  delay(700);
  pinMode(DHTPin, INPUT);
  pinMode(SOIL_TEMPERATURE, INPUT);
  pinMode(MOISTURE, INPUT);

  setupWifi();
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  digitalWrite(LED_BUILTIN, HIGH);

  delay(1000);
  if (!SPIFFS.begin()) {
    Serial.println("Failed to mount file system");
    return;
  }

  Serial.print("Heap: "); Serial.println(ESP.getFreeHeap());
  
  // Load CA file
  File ca = SPIFFS.open("/ca.der", "r"); //replace ca eith your uploaded file name
  if (!ca) {
    Serial.println("Failed to open ca ");
  } else {
    Serial.println("Success to open ca");
  }
  delay(1000);
  if (espClient.loadCACert(ca)) {
    Serial.println("ca loaded");
  } else {
    Serial.println("ca failed");
  }

  Serial.print("Heap: "); Serial.println(ESP.getFreeHeap());
  Serial.println();
}

void setupWifi() {
  //  WiFi.mode(WIFI_OFF);
  //  delay(1000);
  //  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  //  espClient.setBufferSizes(512, 512);

  WiFi.begin(wifi_ssid, wifi_password);

  Serial.print("Connecting wifi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    digitalWrite(LED_WIFI, LOW);
  }
  Serial.println();

  Serial.print("Connected to ");
  Serial.println(wifi_ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  digitalWrite(LED_WIFI, HIGH);


  timeClient.begin();
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }
  espClient.setX509Time(timeClient.getEpochTime());
}


void reconnect() {
  digitalWrite(LED_MQTT, LOW);
  // Loop until we're reconnected
  while (!client.connected()) {

    Serial.print("Attempting MQTT connection...");
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      //        if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      Serial.println();
      client.subscribe(String(action_topic).c_str());

      digitalWrite(LED_MQTT, HIGH);
    } else {
      currentAttemp++;
      if (currentAttemp >= maxAttemp) {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" restarting ESP");
        restartESP();
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");

        char buf[256];
        espClient.getLastSSLError(buf, 256);
        Serial.print("WiFiClientSecure SSL error: ");
        Serial.println(buf);
      }
      delay(5000);
    }
  }
}

void restartESP() {
  WiFi.forceSleepBegin();
  wdt_reset();
  ESP.restart();
  while (1) wdt_reset();
}

String generateJSONPayload(String topic, int value) {
  return "{\"pattern\": \"" + String(topic) + "\", \"data\": " + value + "}";
}

String generateJSONPayload(String topic, float value) {
  return "{\"pattern\": \"" + String(topic) + "\", \"data\": " + value + "}";
}

String generateJSONPayload(String topic, int value[2]) {
  return "{\"pattern\": \"" + String(topic) + "\", \"data\": [" + value[0] + "," + value[1] + "]}";
}

// the loop function runs over and over again forever
void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (isRelayRunning) {
    currentRelayTimer += LOOP_DELAY;

    if (currentRelayTimer >= RELAY_MAX_ACTIVE) {
      Serial.println("Relay: Turn off");
      digitalWrite(RELAY_PIN, LOW);
      isRelayRunning = false;

      int actionParam[2] = {0, ACTION_DATA[1]};
      client.publish(String(action_topic).c_str(), String(generateJSONPayload(action_topic, actionParam)).c_str());
      currentRelayTimer = 0;
    }
  }

  long now = millis();
  if (now - lastMsg > MQTT_COLLECT_SEND) {
    lastMsg = now;

    digitalWrite(LED_BUILTIN, LOW);

    float newTemp = dht.readTemperature();
    if (!isnan(newTemp)) {
      Serial.print("Nilai sensor dht temperature:");
      Serial.println(String(newTemp).c_str());
      client.publish(String(temperature_topic).c_str(), String(generateJSONPayload(temperature_topic, newTemp)).c_str());
    } else {
      Serial.print("Sensor dht temperature tidak valid:");
      Serial.println(String(newTemp).c_str());
    }

    float newHum = dht.readHumidity();
    if (!isnan(newHum)) {
      Serial.print("Nilai sensor dht humidity:");
      Serial.println(String(newHum).c_str());
      client.publish(String(humidity_topic).c_str(), String(generateJSONPayload(humidity_topic, newHum)).c_str());
    } else {
      Serial.print("Sensor read dht humidity tidak valid:");
      Serial.println(String(newHum).c_str());
    }

    soilMoistureValue = analogRead(MOISTURE);
    soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
    if (soilmoisturepercent >= 0 && soilmoisturepercent <= 100) {
      Serial.print("Nilai sensor soil moisture: ");
      Serial.print(String(soilMoistureValue).c_str());
      Serial.print(" [");
      Serial.print(String(soilmoisturepercent).c_str());
      Serial.println("]");
      client.publish(String(soil_moisture_topic).c_str(), String(generateJSONPayload(soil_moisture_topic, soilmoisturepercent)).c_str());
    } else {
      Serial.print("Sensor soil moisture tidak valid:");
      Serial.print(String(soilMoistureValue).c_str());
      Serial.print(" [");
      Serial.print(String(soilmoisturepercent).c_str());
      Serial.println("]");
    }

    soilTemp.requestTemperatures();
    float valueSoilTemp = soilTemp.getTempCByIndex(0);
    if (!isnan(valueSoilTemp) && valueSoilTemp > -127) {
      Serial.print("Nilai sensor soil temperature:");
      Serial.println(String(valueSoilTemp).c_str());
      client.publish(String(soil_temperature_topic).c_str(), String(generateJSONPayload(soil_temperature_topic, valueSoilTemp)).c_str());
    } else {
      Serial.print("Sensor soil temp tidak valid:");
      Serial.println(String(valueSoilTemp).c_str());
    }
    Serial.print("Heap: "); Serial.println(ESP.getFreeHeap());

    Serial.println();

    digitalWrite(LED_BUILTIN, HIGH);
  }

  delay(LOOP_DELAY);
}
