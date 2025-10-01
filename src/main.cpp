#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

void setup()
{
    Serial.begin(115200);

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
}

void loop()
{
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    delay(10000);
}