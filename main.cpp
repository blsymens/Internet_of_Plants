#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <driver/adc.h>
#include <time.h>
#include <sys/time.h>

#include <Wire.h>
#include <DFRobot_ENS160.h> // Include library for ENS160 sensor
#include <Adafruit_AHTX0.h> // Include Adafruit AHTX0 library

// Wi-Fi credentials
#define WIFI_SSID "***"
#define WIFI_PASSWORD "***"

// MQTT broker details
#define MQTT_BROKER_URI "***.emqxsl.com"
#define MQTT_PORT 8883
#define MQTT_TOPIC "sensor"

#define MQTT_USERNAME "***"
#define MQTT_PASSWORD "***"

// NTP Server details
const char* ntpServer = "pool.ntp.org";

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// Create instances of the ENS160 and AHT21 classes
DFRobot_ENS160_I2C ens160(&Wire, 0x53); // Use I2C address 0x53 for ENS160
Adafruit_AHTX0 aht; // Create an instance of the AHTX0 class

void initWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void mqttConnect() {
    espClient.setInsecure();
    mqttClient.setServer(MQTT_BROKER_URI, MQTT_PORT);
    
    Serial.println("Attempting MQTT connection...");
    String clientId = "esp-client";
    if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
        Serial.println("Connected to MQTT broker");
    } else {
        Serial.print("Failed to connect, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" Retrying in 5 seconds");
        delay(5000);
    }
}

unsigned long long getTimestampMs() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;
}

void setup() {
    Serial.begin(115200);
    
    initWiFi();
    
    // Configure time
    configTime(0, 0, ntpServer);
    
    mqttConnect();

    Wire.begin(21, 22); // Initialize I2C with custom SDA and SCL pins

    // Initialize the ENS160 sensor
    if (ens160.begin() != NO_ERR) {
        Serial.println("Failed to initialize ENS160 sensor!");
        while (1) yield();
    }

    // Initialize the AHT21 sensor using Adafruit library
    if (!aht.begin()) {
        Serial.println("Failed to initialize AHT21 sensor!");
        while (1) yield();
    }

    // Set power mode for ENS160
    ens160.setPWRMode(ENS160_STANDARD_MODE); // Set to standard operation mode

    Serial.println("Setup complete.");
}

void loop() {
    if (!mqttClient.connected()) {
        mqttConnect();
    }
    
    mqttClient.loop();

    // Read temperature and humidity from the AHT21 sensor
    sensors_event_t humidity, temperature;
    
    aht.getEvent(&humidity, &temperature); // Get temperature and humidity readings

    // Set temperature and humidity for ENS160 compensation
    ens160.setTempAndHum(temperature.temperature, humidity.relative_humidity);

    // Read CO2 from ENS160 sensor
    uint16_t eco2 = ens160.getECO2(); // Get eCO2 value

    if (eco2 >= 400 && eco2 <= 65000) { // Check for valid eCO2 range
        unsigned long long timestamp = getTimestampMs();
        
        Serial.printf("eCO2: %d ppm\n", eco2);
        Serial.printf("Temperature: %.1f °C\n", temperature.temperature);
        Serial.printf("Humidity: %.1f %%\n", humidity.relative_humidity);

        char message[200];
        snprintf(message, sizeof(message), "{\"timestamp\": %llu, \"co2_value\": %d, \"temperature\": %.1f, \"humidity\": %.1f}", timestamp, eco2, temperature.temperature, humidity.relative_humidity);

        if (mqttClient.publish(MQTT_TOPIC, message, true)) {
            Serial.printf("Published: %s\n", message);
        } else {
            Serial.println("Failed to publish message.");
        }
        
        delay(5000); // Wait for 5 seconds before next reading
    } else {
        Serial.println("Error reading from ENS160 or invalid values.");
        delay(5000); // Wait before retrying on error
    }
}