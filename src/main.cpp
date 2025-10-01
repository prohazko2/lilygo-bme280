#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Cellular + MQTT
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// ====== TTGO T-Call (SIM800L) Pins ======
// Reference values commonly used by TTGO T-Call boards
#define MODEM_TX 27
#define MODEM_RX 26
#define MODEM_PWRKEY 4
#define MODEM_RST 5
#define MODEM_POWER_ON 23

// I2C pins for BME280 (ESP32 default)
#define I2C_SDA 21
#define I2C_SCL 22

// ====== Network / MQTT configuration (fill with your values) ======
// SIM card APN credentials
static const char *GPRS_APN = "internet";        // e.g. "internet"
static const char *GPRS_USER = "";               // often empty
static const char *GPRS_PASS = "";               // often empty

// MQTT broker
static const char *MQTT_HOST = "sandbox.rightech.io";   // change to your broker host/IP
static const uint16_t MQTT_PORT = 1883;           // change if needed (e.g. 8883 TLS not covered here)
static const char *MQTT_CLIENT_ID = "mqtt-olegprohazko-ecqy8r";
static const char *MQTT_USER = "";               // optional
static const char *MQTT_PASS = "";               // optional
static const char *MQTT_TOPIC = "dht11/raw";

// ====== Globals ======
Adafruit_BME280 bme;

// Serial for modem
HardwareSerial SerialAT(1);

#define TINY_GSM_MODEM_SIM800
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
PubSubClient mqtt(gsmClient);

unsigned long lastPublishMs = 0;
const unsigned long publishIntervalMs = 300000; // 5 minutes


static void logNetworkInfo(const char *prefix)
{
    int16_t rssi = modem.getSignalQuality();
    String oper = modem.getOperator();
    //String ip = modem.localIP();
    Serial.print(prefix);
    Serial.print(" RSSI="); Serial.print(rssi);
    Serial.print(" dBm, OP="); Serial.print(oper);
    //Serial.print(", IP="); Serial.println(ip);
}

static void publishBme()
{
    float temperatureC = bme.readTemperature();
    //float humidity = bme.readHumidity();
    //float pressurePa = bme.readPressure();

    char payload[160];
    snprintf(payload, sizeof(payload),
             "{\"c\":%.2f}",
             temperatureC);

    Serial.print("[MQTT] Publish ");
    Serial.print(MQTT_TOPIC);
    Serial.print(" -> ");
    Serial.println(payload);

    if (mqtt.connected())
    {
        mqtt.publish(MQTT_TOPIC, payload, true);
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
    logNetworkInfo("[GSM]");

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
    logNetworkInfo("[GPRS]");
    return true;
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
    Serial.print(MQTT_CLIENT_ID);
    Serial.print(" ... ");
    bool ok;
    if (strlen(MQTT_USER) == 0)
    {
        ok = mqtt.connect(MQTT_CLIENT_ID);
    }
    else
    {
        ok = mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
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

    // Power on and initialize modem
    powerOnModem();

    Serial.println("[MODEM] Initializing SerialAT...");
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

    Serial.println("[MODEM] Restarting modem...");
    modem.restart();
    String modemInfo = modem.getModemInfo();
    Serial.print("[MODEM] ");
    Serial.println(modemInfo);

    // Set network mode if needed (optional)
    // modem.setNetworkMode(2); // 2G only for SIM800

    // Connect GPRS
    for (int i = 0; i < 3; ++i)
    {
        Serial.print("[GPRS] Try "); Serial.print(i + 1); Serial.println("/3");
        if (ensureGprs()) break;
        delay(5000);
    }

    // Setup MQTT
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
}

void loop()
{
    // Maintain MQTT connection
    if (!mqtt.connected())
    {
        if (!modem.isNetworkConnected() || !modem.isGprsConnected())
        {
            Serial.println("[NET] Lost network/GPRS, re-attaching...");
            ensureGprs();
        }
        if (!mqttReconnect())
        {
            // Light cooldown to avoid tight loops when broker is unavailable
            delay(2000);
        }
    }
    mqtt.loop();

    unsigned long now = millis();
    if (now - lastPublishMs >= publishIntervalMs)
    {
        lastPublishMs = now;
        publishBme();
    }
}