# HA Sensor — контекст проекта

## Железо

- **МК:** ESP32-C3 Super Mini
- **Радар:** HLK-LD2410C (mmWave 24GHz, присутствие + дистанция)
- **Планируется добавить:** VL53L0X (ToF лазер, I2C, смотрит строго вниз)
- **Планируется заменить:** LD2410C → LD2420 (другой протокол, нужен новый парсер)

## Распиновка (актуальная)

```
ESP32-C3        LD2410C
────────        ───────
GPIO20  (RX1) ←  TX сенсора       ← проверено осциллографом, рабочий пин
GPIO21  (TX1) →  RX сенсора
GPIO3        ←  OUT сенсора
3.3V         →  VCC
GND          →  GND


I2C для VL53L0X — свободные пины: GPIO4, GPIO5, GPIO6, GPIO7, GPIO8, GPIO9, GPIO10, GPIO0, GPIO1
DOOR_PIN — любой свободный GPIO (через оптопару, не напрямую)

## Структура файлов

```
HA Sensor/
├── src/
│   ├── main.cpp       — основная логика, WiFi, MQTT, loop
│   ├── ld2410.h       — заголовок драйвера LD2410C
│   ├── ld2410.cpp     — парсер UART фреймов LD2410C
│   ├── pins.h         — только константы пинов
│   └── db_keys.h      — ключи GyverDB (макрос DB_KEYS)
├── platformio.ini
└── CONTEXT.md         — этот файл
```

## Библиотеки (platformio.ini)

```ini
lib_deps =
    ; MQTT
    knolleary/PubSubClient @ ^2.8.0
    ; JSON
    bblanchon/ArduinoJson  @ ^7.0.0
    ; Web-интерфейс настроек (подтягивает GyverHTTP, GyverDB, GTL и др. автоматически)
    gyverlibs/Settings       @ ^1.3.0
    GyverLibs/StringUtils
	GyverLibs/Pairs
	GyverLibs/GSON
	GyverLibs/Settings
    ; WebSocket для live-обновлений в вебморде (SettingsGyverWS требует его)
    links2004/WebSockets     @ ^2.4.0
    ; Асинхронное подключение к WiFi + AP fallback
    gyverlibs/WiFiConnector  @ ^1.0.0
```

## Что реализовано

### WiFi
- `WiFiConnector` — асинхронное подключение
- При пустом SSID или ошибке → AP "PresenceSensor" / "12345678"
- `onConnect` → `setupIds()` + `connectMQTT()`

### Веб-интерфейс (SettingsGyverWS)
- HTTP порт 80, WebSocket порт 81
- Настраивается через браузер: WiFi, MQTT, параметры сенсора
- Live-обновление секции "Состояние" через запись в `db[kk::lbl_*]`
- OTA встроено в Settings, работает без `b.OTA()`

### GyverDB
- Файл `/config.db` на LittleFS
- Ключи через `DB_KEYS(kk, ...)` в `db_keys.h`
- Значения по умолчанию через `db.init()`

### MQTT
- Брокер: 192.168.1.201:1883, user: frigate_user, pass: frigate_2449226
- MQTT Discovery → автоматически создаёт устройство в HA
- Топики: `presence/<device_id>/state` (JSON), `presence/<device_id>/avail`
- Публикует: presence, moving, stationary, moving_dist, moving_energy, static_dist, static_energy, detect_dist, out_pin
- `pl_on/pl_off` используют `{{ value_json.X | lower }}` для binary_sensor

### LD2410C драйвер
- UART1 / Serial1, 256000 baud, SERIAL_8N1
- Инициализация: `Serial1.begin(256000, SERIAL_8N1, RX, TX)` — снаружи, НЕ внутри драйвера
- Парсер: ищет заголовок `F4 F3 F2 F1`, футер `F8 F7 F6 F5`
- Структура payload (после заголовка):
  ```
  p[0][1] = длина (0x0D 0x00)
  p[2]    = тип (0x02 = базовый режим)
  p[3]    = маркер 0xAA
  p[4]    = target status (0x00/0x01/0x02/0x03)
  p[5][6] = moving dist (LE, см)
  p[7]    = moving energy (0-100)
  p[8][9] = static dist (LE, см)
  p[10]   = static energy (0-100)
  p[11][12] = detect dist (LE, см)
  ```


## Текущий статус

✅ ESP32 работает  
✅ Веб-интерфейс работает (http://\<IP\>/)  
✅ WiFi подключается, AP fallback работает  
✅ MQTT подключён, устройство видно в HA  
✅ LD2410C читается, данные корректные  
✅ presence/moving/distance публикуются в HA  

## Следующие шаги

### 1. Добавить VL53L0X
- Библиотека: `pololu/VL53L0X @ ^1.3.1`
- Подключение: I2C (SDA/SCL — выбрать свободные пины)
- Смотрит **строго вниз**, контролирует зону под устройством
- Новые ключи в DB_KEYS: `vl53_threshold` (порог срабатывания, мм)
- Публикует в MQTT: `zone_blocked` (binary_sensor)

### 2. Логика двери (`door_logic.h/.cpp`)
Устройство крепится на высоте 2–3м, LD2410C смотрит вперёд/вниз под углом ~45°

**Сценарий:**
```
LD2410C: объект приближается (dist уменьшается) 
    → ESP отправляет сигнал "открыть" на DOOR_PIN

VL53L0X: объект в проёме (dist < порог)
    → держать DOOR_PIN активным (дверь не закрывать)
    
VL53L0X: проём свободен И LD2410C: нет приближения
    → деактивировать DOOR_PIN (разрешить закрытие)
```

**Настройки через вебморду:**
- `door_open_dist` — дистанция от LD2410C при которой открывать (см)
- `door_approach_delta` — на сколько см должно уменьшиться расстояние чтобы считать "приближение"
- `vl53_threshold` — порог VL53L0X "объект в проёме" (мм)
- `door_close_delay` — задержка закрытия после освобождения проёма (мс)
- `door_pin_active` — HIGH или LOW активирует дверь (зависит от оптопары)

**DOOR_PIN** — абстрактный выход, реально идёт через оптопару

### 3. Заменить LD2410C → LD2420 (отдельный этап)
- Полностью переписать `ld2410.cpp` и `ld2410.h`
- Протокол LD2420: текстовый (ASCII), строки через `\n`
- Baud rate: 256000 (fw < 1.5.3) или 115200 (fw ≥ 1.5.3)
- Остальной код (main.cpp, MQTT, вебморда) — не меняется

## Параметры MQTT брокера

```
host: 192.168.1.201
port: 1883
user: frigate_user
pass: frigate_2449226
```

Брокер используется совместно с Frigate NVR + Double Take.
Топики Frigate: `frigate/...`, Double Take: `doubletake/...` — конфликтов нет.

## Примечания

- ESP32-C3 Super Mini: `ARDUINO_USB_CDC_ON_BOOT=1` и `ARDUINO_USB_MODE=1` обязательны в build_flags
- `board_build.filesystem = littlefs` обязателен для GyverDBFile
- `sett.begin()` вызывать ПОСЛЕ `WiFiConnector.connect()` — иначе DNS captive portal не работает
- LD2410C питание: 3.3V–5V, логика 3.3V — совместимо с ESP32-C3
- VL53L0X питание: 3.3V, I2C адрес 0x29 (можно изменить через XSHUT)
- На данный момент код полностью рабочий. За исключением обновлений значений в веб интерфейсе, почему то обновление в базе данных не обновляет значения в
  вебинтерфейсе. В HA все работает.
- В планах заменить датчик LD2410C на LD2420, возможно есть возможность поддерживать оба этих сенсора в одном проекте и выбирать какой под какой датчик компилировать код через дефайны

