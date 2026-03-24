#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <GyverDBFile.h>
#include <SettingsGyverWS.h>   // GyverHTTP + WebSocket (live-обновления)
#include <WiFiConnector.h>

#include <Wire.h>
#include <VL53L1X.h>

#include "pins.h"
#include "db_keys.h"
#include "ld2410.h"
#include "door_logic.h"
#include <HardwareSerial.h>

// ============================================================
//  База данных и веб-интерфейс
// ============================================================
GyverDBFile    db(&LittleFS, "/config.db");
SettingsGyverWS sett("Presence Sensor", &db);

// ============================================================
//  MQTT и сенсоры
// ============================================================
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
LD2410       sensor;
VL53L1X      vl53;
DoorLogic    door;
bool         vl53ok = false;
bool         zoneBlocked = false;
uint16_t     vl53dist = 9999;

char deviceId[32];
char topicState[80];
char topicAvail[80];

unsigned long timerMQTT;
unsigned long timerSettUpdate;
unsigned long timerApLog;

bool     prevPresence = false;
bool     firstPublish = true;
unsigned long lastPublish   = 0;
unsigned long lastMqttCheck = 0;

// Флаг: пересоздать MQTT-соединение (настройки изменились)
bool mqttNeedsReconnect = false;
bool mqttNeedsConnect   = false;

// ============================================================
//  ID устройства и MQTT топики
// ============================================================
void setupIds() {
    String id = (String)db[kk::device_id];
    id.trim();
    if (id.isEmpty()) {
        uint64_t chipId = ESP.getEfuseMac();
        id = String((uint32_t)(chipId & 0xFFFFFF), HEX);
    }
    id.toCharArray(deviceId, sizeof(deviceId));
    snprintf(topicState, sizeof(topicState), "presence/%s/state", deviceId);
    snprintf(topicAvail, sizeof(topicAvail), "presence/%s/avail", deviceId);
}

// ============================================================
//  MQTT Discovery
// ============================================================
void publishDiscovery() {
    char     cfgTopic[120];
    String   payload;
    JsonDocument doc;

    auto makeDevice = [&]() {
        doc["device"]["ids"][0] = deviceId;
        doc["device"]["name"]   = String("Presence ") + deviceId;
        doc["device"]["model"]  = "ESP32-C3 + LD2410C";
        doc["device"]["mf"]     = "DIY";
        doc["avty_t"]           = topicAvail;
        doc["pl_avail"]         = "online";
        doc["pl_not_avail"]     = "offline";
    };

    struct Entity {
        const char* component;
        const char* suffix;
        const char* name;
        const char* valTpl;
        const char* devClass;
        const char* stateClass;
        const char* unit;
        const char* plOn;
        const char* plOff;
    };

    const Entity entities[] = {
        {"binary_sensor", "presence",     "Presence",        "{{ value_json.presence | lower }}",     "occupancy", nullptr,       nullptr, "true", "false"},
        {"binary_sensor", "moving",       "Moving",          "{{ value_json.moving | lower }}",       "motion",    nullptr,       nullptr, "true", "false"},
        {"binary_sensor", "zone_blocked", "Zone Blocked",    "{{ value_json.zone_blocked | lower }}", nullptr,     nullptr,       nullptr, "true", "false"},
        {"sensor",        "mov_dist",     "Moving distance", "{{ value_json.moving_dist }}",          "distance",  "measurement", "cm",    nullptr, nullptr},
        {"sensor",        "stat_dist",    "Stationary dist", "{{ value_json.static_dist }}",          "distance",  "measurement", "cm",    nullptr, nullptr},
        {"sensor",        "mov_nrg",      "Moving energy",   "{{ value_json.moving_energy }}",        nullptr,     "measurement", "%",     nullptr, nullptr},
        {"sensor",        "vl53_dist",    "Zone distance",   "{{ value_json.vl53_dist }}",            "distance",  "measurement", "mm",    nullptr, nullptr},
    };

    for (auto& e : entities) {
        doc.clear();
        makeDevice();
        doc["name"]     = e.name;
        doc["uniq_id"]  = String(deviceId) + "_" + e.suffix;
        doc["stat_t"]   = topicState;
        doc["val_tpl"]  = e.valTpl;
        if (e.devClass)   doc["dev_cla"]    = e.devClass;
        if (e.stateClass) doc["state_class"] = e.stateClass;
        if (e.unit)       doc["unit_of_meas"] = e.unit;
        if (e.plOn)     { doc["pl_on"] = e.plOn; doc["pl_off"] = e.plOff; }
        snprintf(cfgTopic, sizeof(cfgTopic),
                 "homeassistant/%s/%s_%s/config", e.component, deviceId, e.suffix);
        serializeJson(doc, payload);
        mqttClient.publish(cfgTopic, payload.c_str(), true);
        payload = "";
    }

    Serial.println("[MQTT] Discovery опубликован");
}

// ============================================================
//  MQTT подключение
// ============================================================
bool connectMQTT() {
    if (mqttClient.connected()) return true;
    if (!WiFiConnector.connected()) return false;

    String   broker = (String)db[kk::mqtt_broker];
    uint16_t port   = (int)db[kk::mqtt_port];
    String   user   = (String)db[kk::mqtt_user];
    String   pass   = (String)db[kk::mqtt_pass];

    if (broker.isEmpty()) return false;

    mqttClient.setServer(broker.c_str(), port ? port : 1883);
    mqttClient.setBufferSize(512);
    mqttClient.setSocketTimeout(3);  // не блокировать loop дольше 3 сек
    mqttClient.setKeepAlive(10);     // ping брокера каждые 10 сек

    Serial.printf("[MQTT] Подключаюсь к %s:%d...\n", broker.c_str(), port);

    bool ok = mqttClient.connect(
        deviceId, user.c_str(), pass.c_str(),
        topicAvail, 1, true, "offline"
    );

    if (ok) {
        Serial.println("[MQTT] Подключён");
        mqttClient.publish(topicAvail, "online", true);
        publishDiscovery();
        firstPublish = true;
    } else {
        Serial.printf("[MQTT] Ошибка: %d\n", mqttClient.state());
    }
    return ok;
}

// ============================================================
//  Публикация состояния сенсора
// ============================================================
void publishState(const LD2410Data& d) {
    JsonDocument doc;
    doc["presence"]      = d.presence();
    doc["moving"]        = d.isMoving();
    doc["stationary"]    = d.isStatic();
    doc["moving_dist"]   = d.movingDist;
    doc["moving_energy"] = d.movingEnergy;
    doc["static_dist"]   = d.staticDist;
    doc["static_energy"] = d.staticEnergy;
    doc["detect_dist"]   = d.detectDist;
    doc["out_pin"]       = d.outPin;
    doc["zone_blocked"]  = zoneBlocked;
    doc["vl53_dist"]     = vl53dist;

    char payload[384];
    serializeJson(doc, payload);
    mqttClient.publish(topicState, payload);

    Serial.printf("[STATE] presence=%s md=%dcm sd=%dcm vl53=%dmm\n",
        d.presence() ? "YES" : "NO", d.movingDist, d.staticDist, vl53dist);
}

// ============================================================
//  Веб-интерфейс (onBuild)
// ============================================================
void buildUI(sets::Builder& b) {

    // ---- WiFi ----
    {
        sets::Group g(b, "WiFi");
        b.Input(kk::wifi_ssid, "SSID");
        b.Pass (kk::wifi_pass, "Пароль", "***");
        if (b.Button(kk::wifi_apply, "Подключить")) {
            db.update();
            WiFiConnector.connect(db[kk::wifi_ssid], db[kk::wifi_pass]);
        }
    }

    // ---- MQTT ----
    {
        sets::Group g(b, "MQTT");
        b.Input(kk::mqtt_broker, "Broker IP");
        b.Input(kk::mqtt_port,   "Порт");
        b.Input(kk::mqtt_user,   "Пользователь");
        b.Pass (kk::mqtt_pass,   "Пароль", "***");
        b.Input(kk::device_id,   "Device ID (пусто = авто)");
        if (b.Button(kk::mqtt_apply, "Применить")) {
            db.update();
            mqttNeedsReconnect = true;
        }
    }

    // ---- Параметры сенсора ----
    {
        sets::Group g(b, "Сенсор");
        b.Slider(kk::pub_interval,   "Интервал публикации", 100, 5000, 100, "мс");
        b.Slider(kk::sensor_maxdist, "Макс. дистанция",      75,  600,  75, "см");
        b.Slider(kk::vl53_threshold, "Порог зоны (VL53)",    50, 3000,  50, "мм");
    }

    // ---- Логика двери ----
    {
        sets::Group g(b, "Дверь");
        b.Slider(kk::door_approach_delta, "Порог приближения", 5,  50,  5, "см/с");
        b.Slider(kk::door_open_dist,      "Дистанция открытия", 50, 500, 25, "см");
        b.Slider(kk::door_close_delay,    "Задержка закрытия",  500, 10000, 500, "мс");
    }

    // ---- Live-состояние (обновляется по WebSocket) ----
    {
        sets::Group g(b, "Состояние");
        b.Label(kk::lbl_wifi,      "WiFi");
        b.Label(kk::lbl_mqtt,      "MQTT");
        b.Label(kk::lbl_presence,  "Присутствие");
        b.Label(kk::lbl_moving,    "Движение");
        b.Label(kk::lbl_static,    "Стоит");
        b.Label(kk::lbl_mov_dist,  "Дист. (движ.)");
        b.Label(kk::lbl_stat_dist, "Дист. (стат.)");
        b.Label(kk::lbl_energy,    "Сигнал");
        b.Label(kk::lbl_vl53,      "Зона (VL53)");
        b.Label(kk::lbl_door,      "Дверь");
    }

}


// ============================================================
//  Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    // USB CDC: ждём подключения терминала (до 5 сек), потом всё равно идём дальше
    {
        uint32_t t = millis();
        while (!Serial && millis() - t < 5000) delay(10);
    }
    Serial1.begin(256000, SERIAL_8N1, LD2410_RX_PIN, LD2410_TX_PIN);
    Serial.println("\n=== Presence Sensor v2.1 ===");

    // LittleFS — до db.begin()
    LittleFS.begin(true);

    // БД — значения по умолчанию записываются только если ячейка пуста
    db.begin();
    db.init(kk::wifi_ssid,      (String)"");
    db.init(kk::wifi_pass,      (String)"");
    db.init(kk::mqtt_broker,    (String)"");
    db.init(kk::mqtt_port,      (uint16_t)1883);
    db.init(kk::mqtt_user,      (String)"");
    db.init(kk::mqtt_pass,      (String)"");
    db.init(kk::device_id,      (String)"");
    db.init(kk::pub_interval,   (uint16_t)500);
    db.init(kk::sensor_maxdist, (uint16_t)300);
    db.init(kk::vl53_threshold,     (uint16_t)500);
    db.init(kk::door_approach_delta,(uint16_t)15);   // 15 см/сек
    db.init(kk::door_open_dist,     (uint16_t)200);  // до 2м
    db.init(kk::door_close_delay,   (uint16_t)3000); // 3 сек

    // Выходы на оптопары
    pinMode(DOOR_OPEN_PIN, OUTPUT);
    digitalWrite(DOOR_OPEN_PIN, LOW);
    pinMode(DOOR_ZONE_PIN, OUTPUT);
    digitalWrite(DOOR_ZONE_PIN, LOW);
    door.begin(DOOR_OPEN_PIN);

    // VL53L1X
    Wire.begin(VL53_SDA_PIN, VL53_SCL_PIN);
    Wire.setClock(100000);
    delay(200);

    vl53.setTimeout(500);
    vl53ok = vl53.init();
    Serial.printf("[VL53L1X] init() = %s\n", vl53ok ? "OK" : "FAIL");
    if (vl53ok) {
        vl53.setDistanceMode(VL53L1X::Short);   // до 1.3м, лучше при освещении
        vl53.setMeasurementTimingBudget(50000);  // 50мс на замер
        vl53.startContinuous(100);
        delay(150);
        vl53.read();
        Serial.printf("[VL53L1X] Тест: %d мм  timeout=%s\n",
            vl53.ranging_data.range_mm, vl53.timeoutOccurred() ? "ДА" : "нет");
    }

    // Сенсор LD2410C
    sensor.begin(Serial1, LD2410_RX_PIN, LD2410_TX_PIN, LD2410_OUT_PIN);
    Serial.println("[LD2410] UART1 инициализирован (256000 baud)");
    Serial.printf("[LD2410] RX=%d TX=%d OUT=%d\n", LD2410_RX_PIN, LD2410_TX_PIN, LD2410_OUT_PIN);

    // WiFiConnector
    WiFiConnector.onConnect([]() {
        Serial.printf("[WiFi] Подключён: %s\n", WiFi.localIP().toString().c_str());
        setupIds();
        mqttNeedsConnect = true;   // подключимся к MQTT в loop, не здесь
        WiFiConnector.closeAP(close);
    });
    WiFiConnector.onError([]() {
        const char* reason = "неизвестно";
        switch (WiFi.status()) {
            case WL_NO_SSID_AVAIL:   reason = "сеть не найдена"; break;
            case WL_CONNECT_FAILED:  reason = "неверный пароль или отказ"; break;
            case WL_CONNECTION_LOST: reason = "соединение потеряно"; break;
            case WL_DISCONNECTED:    reason = "отключён"; break;
        }
        Serial.printf("[WiFi] Ошибка (%s), поднята AP: %s\n",
            reason, WiFi.softAPIP().toString().c_str());
    });

    // Запуск подключения — если SSID пустой, пробуем дефолтную сеть
    {
        String ssid = (String)db[kk::wifi_ssid];
        String pass = (String)db[kk::wifi_pass];
        if (ssid.isEmpty()) {
            // ssid = "YOUR_SSID";
            // pass = "YOUR_PASS";
        }
        // Детальные события WiFi
        WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
            switch (event) {
                case ARDUINO_EVENT_WIFI_STA_CONNECTED:
                    Serial.printf("[WiFi] Ассоциирован с '%s'\n",
                        (char*)info.wifi_sta_connected.ssid);
                    break;
                case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
                    uint8_t reason = info.wifi_sta_disconnected.reason;
                    const char* txt = "неизвестно";
                    if      (reason == 15)  txt = "неверный пароль";
                    else if (reason == 200) txt = "AP пропала";
                    else if (reason == 201) txt = "AP не найдена";
                    else if (reason == 202) txt = "auth failed";
                    Serial.printf("[WiFi] Разрыв, причина %d: %s\n", reason, txt);
                    break;
                }
                default: break;
            }
        });

        // Скан — ищем AP с нужным SSID с лучшим сигналом
        Serial.println("[WiFi] Сканирую сети...");
        int n = WiFi.scanNetworks();
        int bestIdx = -1;
        int bestRssi = -999;
        for (int i = 0; i < n; i++) {
            Serial.printf("  '%s'  RSSI:%d  BSSID:%s\n",
                WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.BSSIDstr(i).c_str());
            if (WiFi.SSID(i) == ssid && WiFi.RSSI(i) > bestRssi) {
                bestRssi = WiFi.RSSI(i);
                bestIdx  = i;
            }
        }

        WiFi.setTxPower(WIFI_POWER_11dBm);  // снижаем с 20dBm до 11dBm
        WiFiConnector.setName("PresenceSensor");
        WiFiConnector.setPass("12345678");

        if (bestIdx >= 0) {
            int bestCh = WiFi.channel(bestIdx);
            uint8_t bestBssid[6];
            memcpy(bestBssid, WiFi.BSSID(bestIdx), 6);
            Serial.printf("[WiFi] Подключаюсь к '%s' BSSID:%s ch:%d pass:'%s'\n",
                ssid.c_str(), WiFi.BSSIDstr(bestIdx).c_str(), bestCh, pass.c_str());
            WiFi.scanDelete();
            // WiFiConnector делает WiFi.begin() без BSSID — сразу переопределяем с BSSID
            WiFiConnector.connect(ssid, pass);
            WiFi.begin(ssid.c_str(), pass.c_str(), bestCh, bestBssid);
        } else {
            Serial.printf("[WiFi] '%s' не найдена, поднимаю AP\n", ssid.c_str());
            WiFi.scanDelete();
            WiFiConnector.connect("", "");
        }
    }

    // Сервер — ПОСЛЕ connect(), иначе DNS captive portal не работает
    sett.onBuild(buildUI);
    sett.begin();

    setupIds();
    Serial.printf("[INFO] Device ID: %s\n", deviceId);
    timerMQTT = millis();
    timerSettUpdate = millis();
    timerApLog = millis();
}

// ============================================================
//  Loop
// ============================================================
void loop() {
    delay(1);  // даём FreeRTOS idle task включить power management

    // WiFiConnector — асинхронное управление соединением
    WiFiConnector.tick();

    // Если WiFi не подключён — раз в секунду печатаем статус
    if (!WiFiConnector.connected() && millis() - timerApLog >= 1000) {
        timerApLog = millis();
        if (WiFiConnector.connecting()) {
            Serial.printf("[WiFi] Подключаюсь... (%lu сек)\n",
                (millis() / 1000));
        } else {
            Serial.printf("[AP] Нет WiFi. Подключись к AP и открой http://%s\n",
                WiFi.softAPIP().toString().c_str());
        }
    }

    // Веб-интерфейс настроек (HTTP + WebSocket)
    sett.tick();

    // Сенсор и MQTT — только при наличии WiFi
    if (WiFiConnector.connected()) {
        unsigned long now = millis();

        // UI обновление раз в секунду
        if (now - timerSettUpdate >= 1000) {
            timerSettUpdate = now;

            // VL53L1X — чтение из continuous mode
            if (vl53ok) {
                vl53.read();
                if (!vl53.timeoutOccurred()) {
                    vl53dist    = vl53.ranging_data.range_mm;
                    zoneBlocked = vl53dist < (uint16_t)(int)db[kk::vl53_threshold];
                    digitalWrite(DOOR_ZONE_PIN, zoneBlocked ? HIGH : LOW);
                }
            }

            // Логика двери
            door.update(
                d.movingDist,
                d.presence(),
                zoneBlocked,
                (int)db[kk::door_approach_delta],
                (int)db[kk::door_open_dist],
                (uint32_t)(int)db[kk::door_close_delay]
            );

            const LD2410Data& d = sensor.data();
            String vWifi  = WiFi.localIP().toString();
            String vMqtt  = mqttClient.connected() ? "OK" : "отключён";
            String vPres  = d.presence() ? "Да" : "Нет";
            String vMov   = d.isMoving()  ? "Да" : "Нет";
            String vStat  = d.isStatic()  ? "Да" : "Нет";
            String vMDist = String(d.movingDist)   + " см";
            String vSDist = String(d.staticDist)   + " см";
            String vEnrg  = String(d.movingEnergy) + "%";
            String vVl53  = vl53ok ? (String(vl53dist) + " мм" + (zoneBlocked ? " [заблок.]" : "")) : "нет датчика";
        String vDoor  = String(door.directionStr()) + (door.isDoorOpen() ? " | ОТКРЫТА" : " | закрыта");

            db[kk::lbl_wifi]      = vWifi;
            db[kk::lbl_mqtt]      = vMqtt;
            db[kk::lbl_presence]  = vPres;
            db[kk::lbl_moving]    = vMov;
            db[kk::lbl_static]    = vStat;
            db[kk::lbl_mov_dist]  = vMDist;
            db[kk::lbl_stat_dist] = vSDist;
            db[kk::lbl_energy]    = vEnrg;
            db[kk::lbl_vl53]      = vVl53;
        db[kk::lbl_door]      = vDoor;

            sett.updater()
                .update(kk::lbl_wifi,      vWifi)
                .update(kk::lbl_mqtt,      vMqtt)
                .update(kk::lbl_presence,  vPres)
                .update(kk::lbl_moving,    vMov)
                .update(kk::lbl_static,    vStat)
                .update(kk::lbl_mov_dist,  vMDist)
                .update(kk::lbl_stat_dist, vSDist)
                .update(kk::lbl_energy,    vEnrg)
                .update(kk::lbl_vl53,      vVl53)
            .update(kk::lbl_door,      vDoor);
        }

        // MQTT — первое подключение после WiFi
        if (mqttNeedsConnect) {
            mqttNeedsConnect = false;
            lastMqttCheck = now;
            connectMQTT();
        }

        // MQTT watchdog — каждые 15 сек
        if (now - lastMqttCheck >= 15000) {
            lastMqttCheck = now;
            if (mqttNeedsReconnect) {
                mqttNeedsReconnect = false;
                mqttClient.disconnect();
                setupIds();
            }
            if (!mqttClient.connected()) connectMQTT();
        }
        if (mqttClient.connected()) {
            if (!mqttClient.loop()) {
                Serial.println("[MQTT] Соединение разорвано");
            }
        }

        // Сенсор LD2410C
        sensor.update();
        const LD2410Data& d = sensor.data();

        // Публикация в MQTT
        uint16_t interval = (int)db[kk::pub_interval];
        if (!interval) interval = 500;

        bool presenceChanged = (d.presence() != prevPresence);
        if ((presenceChanged || (now - lastPublish >= interval) || firstPublish)
            && mqttClient.connected()) {
            publishState(d);
            prevPresence = d.presence();
            lastPublish  = now;
            firstPublish = false;
        }
    }
}
