#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// WiFi + MQTT
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>

// I2C pins for BME280 (ESP32 default)
#define I2C_SDA 21
#define I2C_SCL 22

// ====== Network / MQTT configuration ======
// WiFi credentials (from build-time env vars or defaults)
static const char *getWifiSsid()
{
#ifdef WIFI_SSID
    if (strlen(WIFI_SSID) > 0)
    {
        return WIFI_SSID;
    }
#endif
    return ""; // must be set via env var
}

static const char *getWifiPassword()
{
#ifdef WIFI_PASSWORD
    return WIFI_PASSWORD;
#endif
    return ""; // empty password if not set
}

// MQTT broker
static const char *MQTT_HOST = "sandbox.rightech.io"; // change to your broker host/IP
static const uint16_t MQTT_PORT = 1883;               // change if needed (e.g. 8883 TLS not covered here)
static const char *MQTT_USER = "";                    // optional
static const char *MQTT_PASS = "";                    // optional
static const char *MQTT_TOPIC = "home/bme280";

// ====== Globals ======
Adafruit_BME280 bme;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastPublishMs = 0;
const unsigned long publishIntervalMs = 300000; // 5 minutes
unsigned long lastSuccessfulMs = 0;             // watchdog anchor

static void logNetworkInfo()
{
    Serial.print("[WiFi] SSID=");
    Serial.print(WiFi.SSID());
    Serial.print(", RSSI=");
    Serial.print(WiFi.RSSI());
    Serial.print(" dBm, IP=");
    Serial.println(WiFi.localIP());
}

static const char *getMqttClientId()
{
#ifdef MQTT_CLIENT_ID
    if (strlen(MQTT_CLIENT_ID) > 0)
    {
        return MQTT_CLIENT_ID;
    }
#endif
    return "xxx"; // safe default if env var not provided
}

static bool publishBme()
{
    float t = bme.readTemperature();
    float h = bme.readHumidity();
    float p = bme.readPressure() / 100.0f;

    char payload[160];
    snprintf(payload, sizeof(payload),
             "{\"t\": %.2f, \"h\": %.2f, \"p\": %.2f}",
             t, h, p);

    Serial.print("[MQTT] Publish ");
    Serial.print(MQTT_TOPIC);
    Serial.print(" -> ");
    Serial.println(payload);

    bool sent = false;
    if (mqtt.connected())
    {
        sent = mqtt.publish(MQTT_TOPIC, payload, true);
        if (sent)
        {
            lastSuccessfulMs = millis();
            Serial.println("[MQTT] Publish OK");
        }
        else
        {
            Serial.println("[MQTT] Publish FAILED");
        }
    }
    else
    {
        Serial.println("[MQTT] Not connected, skip publish");
    }
    return sent;
}

static bool ensureWifi()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        return true;
    }

    const char *ssid = getWifiSsid();
    const char *password = getWifiPassword();

    if (strlen(ssid) == 0)
    {
        Serial.println("[WiFi] ERROR: WIFI_SSID not set! Set it via env var: WIFI_SSID=your-ssid pio run");
        return false;
    }

    Serial.print("[WiFi] Connecting to ");
    Serial.print(ssid);
    Serial.print(" ... ");

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30)
    {
        delay(1000);
        Serial.print(".");
        attempts++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("[WiFi] Connected");
        logNetworkInfo();
        return true;
    }
    else
    {
        Serial.println("[WiFi] Connection failed");
        return false;
    }
}

static bool mqttReconnect()
{
    if (mqtt.connected())
    {
        return true;
    }
    Serial.print("[MQTT] Connecting to ");
    Serial.print(MQTT_HOST);
    Serial.print(":");
    Serial.print(MQTT_PORT);
    Serial.print(" as ");
    Serial.print(getMqttClientId());
    Serial.print(" ... ");
    bool ok;
    if (strlen(MQTT_USER) == 0)
    {
        ok = mqtt.connect(getMqttClientId());
    }
    else
    {
        ok = mqtt.connect(getMqttClientId(), MQTT_USER, MQTT_PASS);
    }
    if (ok)
    {
        Serial.println("OK");
        // Immediate publish on first successful connect
        publishBme();
        lastPublishMs = millis();
    }
    else
    {
        int st = mqtt.state();
        Serial.print("FAILED, state=");
        Serial.println(st);
        Serial.println("[MQTT] States: -4:CONN_TIMEOUT, -3:CONN_LOST, -2:CONNECT_FAILED, -1:DISCONNECTED, 1:BAD_PROTO, 2:BAD_CLIENT_ID, 3:UNAVAILABLE, 4:BAD_CREDENTIALS, 5:UNAUTHORIZED");
    }
    return ok;
}

void setup()
{
    Serial.begin(115200);

    // Bring up I2C for BME280 first
    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialize BME280
    bool status;
    for (;;)
    {
        status = bme.begin(0x76);
        if (!status)
        {
            Serial.println("Could not find a valid BME280 sensor, check wiring!");
            delay(1000);
        }
        else
        {
            break;
        }
    }

    // Connect to WiFi
    for (int i = 0; i < 3; ++i)
    {
        Serial.print("[WiFi] Try ");
        Serial.print(i + 1);
        Serial.println("/3");
        if (ensureWifi())
            break;
        delay(5000);
    }

    // Setup MQTT
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    lastSuccessfulMs = millis();

    // Hardware watchdog (hang protection): 60s timeout for loop task
    // Will reset ESP32 if loop stops feeding for >60s (e.g., deadlock)
    const int wdtTimeoutSec = 60;
    esp_task_wdt_init(wdtTimeoutSec, true);
    esp_task_wdt_add(NULL); // add current (loop) task
}

void loop()
{
    // Maintain WiFi and MQTT connection
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("[NET] WiFi disconnected, reconnecting...");
        ensureWifi();
    }

    if (!mqtt.connected())
    {
        if (!mqttReconnect())
        {
            // Light cooldown to avoid tight loops when broker is unavailable
            delay(2000);
        }
    }
    mqtt.loop();

    // Feed hardware watchdog each loop iteration
    esp_task_wdt_reset();

    unsigned long now = millis();
    if (now - lastPublishMs >= publishIntervalMs)
    {
        lastPublishMs = now;
        publishBme();
    }

    // Connectivity watchdog: reboot if there was no successful publish for 10 minutes
    if (millis() - lastSuccessfulMs > 600000UL)
    {
        Serial.println("[WDT] No successful publish for 10 minutes. Restarting...");
        ESP.restart();
        delay(1000);
    }
}