# ESP32 + BME280 → MQTT (WiFi/GSM)

Проект для ESP32, который читает температуру, влажность и давление с датчика BME280 и отправляет данные в MQTT брокер. Поддерживает подключение через WiFi. Если WiFi нет — автоматически переключается на GSM (SIM800L).

## Устройства и датчики

- LilyGO TTGO T-Call ESP32 SIM800L (если нужен только WiFi - подойдёт любой ESP32)
- Метеодатчик BME280 (подключается по I2C)

## Подключение

BME280 подключается к стандартным пинам I2C ESP32:
- SDA → GPIO 21
- SCL → GPIO 22
- VCC → 3.3V
- GND → GND


MQTT брокер и топик настраиваются в `src/main.cpp`:
- `MQTT_HOST` — адрес брокера
- `MQTT_TOPIC` — топик для публикации (по умолчанию `home/bme280`)

## Сборка и запуск

Перед сборкой нужно задать переменные окружения:

```bash
export WIFI_SSID="wifi-ssid"
export WIFI_PASSWORD="wifi-pass"
export MQTT_CLIENT_ID="client-id"
```

Или из `.env` файла:

```bash
set -o allexport; source .env; set +o allexport
```

Далее:
```bash
pio run -t upload && pio device monitor
```

## Формат данных

В MQTT топик `home/bme280` отправляется JSON:
```json
{"t": 23.45, "h": 65.20, "p": 1013.25}
```

Где `t` — температура в °C, `h` — влажность в %, `p` — давление в гПа.
