#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// WiFi + GSM + MQTT
#include <WiFi.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>

// I2C pins for BME280 (ESP32 default)
#define I2C_SDA 21
#define I2C_SCL 22

// TTGO T-Call (SIM800L) Pins
#define MODEM_TX 27
#define MODEM_RX 26
#define MODEM_PWRKEY 4
#define MODEM_RST 5
#define MODEM_POWER_ON 23

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
#ifdef WIFI_PASS
    return WIFI_PASS;
#endif
    return ""; // empty password if not set
}

// GSM/GPRS configuration
static const char *GPRS_APN = "internet";
static const char *GPRS_USER = "";
static const char *GPRS_PASS = "";

// MQTT broker
static const char *MQTT_HOST = "sandbox.rightech.io";
static const uint16_t MQTT_PORT = 1883;
static const char *MQTT_USER = "";
static const char *MQTT_PASS = "";
static const char *MQTT_TOPIC = "home/bme280";

// ====== Globals ======
Adafruit_BME280 bme;

// Network clients
WiFiClient wifiClient;
HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);

// MQTT client (will use either WiFi or GSM)
PubSubClient *mqtt = nullptr;

// Connection state
enum ConnectionType
{
    NONE,
    WIFI,
    GSM
};
ConnectionType activeConnection = NONE;

unsigned long lastPublishMs = 0;
const unsigned long publishIntervalMs = 300000; // 5 minutes
unsigned long lastSuccessfulMs = 0;             // watchdog anchor

static void logNetworkInfo()
{
    if (activeConnection == WIFI)
    {
        Serial.print("[WiFi] SSID=");
        Serial.print(WiFi.SSID());
        Serial.print(", RSSI=");
        Serial.print(WiFi.RSSI());
        Serial.print(" dBm, IP=");
        Serial.println(WiFi.localIP());
    }
    else if (activeConnection == GSM)
    {
        int16_t rssi = modem.getSignalQuality();
        String oper = modem.getOperator();
        Serial.print("[GSM] RSSI=");
        Serial.print(rssi);
        Serial.print(" dBm, Operator=");
        Serial.print(oper);
        Serial.print(", IP=");
        Serial.println(modem.localIP());
    }
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
    if (mqtt && mqtt->connected())
    {
        sent = mqtt->publish(MQTT_TOPIC, payload, true);
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

static void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("[MQTT] Message received on topic: ");
    Serial.println(topic);

    // Check if it's the read command topic
    if (strcmp(topic, "home/bme280/read") == 0)
    {
        Serial.println("[MQTT] Read command received, publishing current readings");
        publishBme();
    }
}

static void powerOnModem()
{
    pinMode(MODEM_POWER_ON, OUTPUT);
    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_RST, OUTPUT);

    // Turn on the modem power supply
    digitalWrite(MODEM_POWER_ON, HIGH);
    delay(100);

    // Ensure reset is high (inactive)
    digitalWrite(MODEM_RST, HIGH);
    delay(100);

    // Pulse PWRKEY to power the modem
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, HIGH);

    // Give the modem time to boot
    delay(3000);
}

static bool ensureGprs()
{
    if (modem.isGprsConnected())
    {
        return true;
    }

    Serial.println("[GSM] Waiting for network (60s)...");
    if (!modem.waitForNetwork(60000L))
    {
        Serial.println("[GSM] Network failed");
        return false;
    }
    if (!modem.isNetworkConnected())
    {
        Serial.println("[GSM] Not connected to network");
        return false;
    }

    Serial.print("[GPRS] Attaching with APN='");
    Serial.print(GPRS_APN);
    Serial.println("' ...");
    if (!modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASS))
    {
        Serial.println("[GPRS] GPRS attach failed");
        return false;
    }
    if (!modem.isGprsConnected())
    {
        Serial.println("[GPRS] Not connected after attach");
        return false;
    }
    Serial.println("[GPRS] Connected");
    logNetworkInfo();
    return true;
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
    if (!mqtt)
    {
        return false;
    }

    if (mqtt->connected())
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
        ok = mqtt->connect(getMqttClientId());
    }
    else
    {
        ok = mqtt->connect(getMqttClientId(), MQTT_USER, MQTT_PASS);
    }
    if (ok)
    {
        Serial.println("OK");
        mqtt->subscribe("home/bme280/read");
        // Immediate publish on first successful connect
        publishBme();
        lastPublishMs = millis();
    }
    else
    {
        int st = mqtt->state();
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

    // Try WiFi first
    bool wifiOk = false;
    for (int i = 0; i < 3; ++i)
    {
        Serial.print("[WiFi] Try ");
        Serial.print(i + 1);
        Serial.println("/3");
        if (ensureWifi())
        {
            wifiOk = true;
            activeConnection = WIFI;
            mqtt = new PubSubClient(wifiClient);
            break;
        }
        delay(5000);
    }

    // If WiFi failed, try GSM
    if (!wifiOk)
    {
        Serial.println("[NET] WiFi failed, trying GSM...");
        powerOnModem();
        Serial.println("[MODEM] Initializing SerialAT...");
        SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
        delay(1000);
        modem.restart();
        delay(2000);
        String modemInfo = modem.getModemInfo();
        Serial.print("[MODEM] ");
        Serial.println(modemInfo);

        for (int i = 0; i < 3; ++i)
        {
            Serial.print("[GSM] Try ");
            Serial.print(i + 1);
            Serial.println("/3");
            if (ensureGprs())
            {
                activeConnection = GSM;
                mqtt = new PubSubClient(gsmClient);
                break;
            }
            delay(5000);
        }
    }

    // Setup MQTT
    if (mqtt)
    {
        mqtt->setServer(MQTT_HOST, MQTT_PORT);
        mqtt->setCallback(mqttCallback);
    }
    lastSuccessfulMs = millis();

    // Hardware watchdog (hang protection): 60s timeout for loop task
    // Will reset ESP32 if loop stops feeding for >60s (e.g., deadlock)
    esp_task_wdt_config_t wdt_config;
    wdt_config.timeout_ms = 60000;
    wdt_config.idle_core_mask = 0;
    wdt_config.trigger_panic = true;
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL); // add current (loop) task
}

void loop()
{
    // Maintain network connection
    if (activeConnection == WIFI)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("[NET] WiFi disconnected, reconnecting...");
            if (!ensureWifi())
            {
                // WiFi failed, try GSM fallback
                Serial.println("[NET] WiFi failed, switching to GSM...");
                activeConnection = NONE;
                if (mqtt)
                {
                    delete mqtt;
                    mqtt = nullptr;
                }
            }
        }
    }
    else if (activeConnection == GSM)
    {
        if (!modem.isNetworkConnected() || !modem.isGprsConnected())
        {
            Serial.println("[NET] GSM/GPRS disconnected, reconnecting...");
            if (!ensureGprs())
            {
                activeConnection = NONE;
                if (mqtt)
                {
                    delete mqtt;
                    mqtt = nullptr;
                }
            }
        }
    }
    else
    {
        // No active connection, try WiFi first, then GSM
        if (ensureWifi())
        {
            activeConnection = WIFI;
            if (!mqtt)
            {
                mqtt = new PubSubClient(wifiClient);
                mqtt->setServer(MQTT_HOST, MQTT_PORT);
                mqtt->setCallback(mqttCallback);
            }
        }
        else
        {
            if (ensureGprs())
            {
                activeConnection = GSM;
                if (!mqtt)
                {
                    mqtt = new PubSubClient(gsmClient);
                    mqtt->setServer(MQTT_HOST, MQTT_PORT);
                    mqtt->setCallback(mqttCallback);
                }
            }
        }
    }

    if (mqtt)
    {
        if (!mqtt->connected())
        {
            if (!mqttReconnect())
            {
                // Light cooldown to avoid tight loops when broker is unavailable
                delay(2000);
            }
        }
        mqtt->loop();
    }

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