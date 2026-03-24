#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

#include <GyverDBFile.h>
#include <SettingsGyverWS.h>
#include <WiFiConnector.h>

#ifdef USE_MQTT
#include <PubSubClient.h>
#endif

#ifdef USE_VL53
#include <Wire.h>
#include <VL53L1X.h>
#endif

#include "pins.h"
#include "db_keys.h"
#include "ld2410.h"
#include "door_logic.h"

// ============================================================
//  База данных и веб-интерфейс
// ============================================================
GyverDBFile     db(&LittleFS, "/config.db");
SettingsGyverWS sett("Presence Sensor", &db);

// ============================================================
//  Сенсоры и логика двери
// ============================================================
LD2410    sensor;
DoorLogic door;

#ifdef USE_VL53
VL53L1X   vl53;
bool      vl53ok     = false;
volatile bool vl53Failed = false;  // runtime-сбой: датчик перестал отвечать
#endif

// ============================================================
//  Снимок данных сенсоров — пишет sensorTask, читает loop
// ============================================================
struct SensorSnap {
    volatile bool     presence;
    volatile bool     isMoving;
    volatile bool     isStatic;
    volatile uint16_t movingDist;
    volatile uint16_t staticDist;
    volatile uint8_t  movingEnergy;
    volatile uint16_t vl53dist     = 9999;
    volatile bool     zoneBlocked  = false;
    volatile bool     doorOpen     = false;
    volatile uint8_t  moveDir      = 0;  // MoveDir as uint8_t
};
SensorSnap snap;

// Кэш настроек из БД — loop пишет, sensorTask читает
volatile uint16_t cfg_vl53Threshold  = 500;
volatile uint16_t cfg_approachDelta  = 15;
volatile uint16_t cfg_openDist       = 200;
volatile uint32_t cfg_closeDelay     = 3000;

// ============================================================
//  MQTT
// ============================================================
#ifdef USE_MQTT
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

char topicState[80];
char topicAvail[80];

bool mqttNeedsConnect   = false;
bool mqttNeedsReconnect = false;
bool firstPublish       = true;
unsigned long lastPublish   = 0;
unsigned long lastMqttCheck = 0;
#endif

// ============================================================
//  Устройство
// ============================================================
char deviceId[32];

void setupIds() {
    String id = (String)db[kk::device_id];
    id.trim();
    if (id.isEmpty()) {
        uint64_t chipId = ESP.getEfuseMac();
        id = String((uint32_t)(chipId & 0xFFFFFF), HEX);
    }
    id.toCharArray(deviceId, sizeof(deviceId));
#ifdef USE_MQTT
    snprintf(topicState, sizeof(topicState), "presence/%s/state", deviceId);
    snprintf(topicAvail, sizeof(topicAvail), "presence/%s/avail", deviceId);
#endif
}

// ============================================================
//  MQTT
// ============================================================
#ifdef USE_MQTT
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
#ifdef USE_VL53
        {"sensor",        "vl53_dist",    "Zone distance",   "{{ value_json.vl53_dist }}",            "distance",  "measurement", "mm",    nullptr, nullptr},
#endif
    };

    for (auto& e : entities) {
        doc.clear();
        makeDevice();
        doc["name"]    = e.name;
        doc["uniq_id"] = String(deviceId) + "_" + e.suffix;
        doc["stat_t"]  = topicState;
        doc["val_tpl"] = e.valTpl;
        if (e.devClass)   doc["dev_cla"]     = e.devClass;
        if (e.stateClass) doc["state_class"]  = e.stateClass;
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
    mqttClient.setSocketTimeout(3);
    mqttClient.setKeepAlive(10);

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

void publishState() {
    JsonDocument doc;
    doc["presence"]      = snap.presence;
    doc["moving"]        = snap.isMoving;
    doc["stationary"]    = snap.isStatic;
    doc["moving_dist"]   = snap.movingDist;
    doc["moving_energy"] = snap.movingEnergy;
    doc["static_dist"]   = snap.staticDist;
    doc["zone_blocked"]  = snap.zoneBlocked;
#ifdef USE_VL53
    doc["vl53_dist"]     = snap.vl53dist;
#endif
    char payload[384];
    serializeJson(doc, payload);
    mqttClient.publish(topicState, payload);
    Serial.printf("[STATE] presence=%s md=%dcm sd=%dcm\n",
        snap.presence ? "YES" : "NO", snap.movingDist, snap.staticDist);
}
#endif // USE_MQTT

// ============================================================
//  Веб-интерфейс
// ============================================================
void buildUI(sets::Builder& b) {

    // ---- Состояние ----
    {
        sets::Group g(b, "Состояние");
        b.Label(kk::lbl_wifi,      "Сеть");
#ifdef USE_MQTT
        b.Label(kk::lbl_mqtt,      "MQTT");
#endif
        b.Label(kk::lbl_mov_dist,  "Расстояние до объекта");
        b.Label(kk::lbl_door,      "Направление движения");
#ifdef USE_VL53
        b.Label(kk::lbl_vl53,      "Зона прохода (VL53)");
#endif
        b.Label(kk::lbl_presence,  "Выход: открыть дверь");
#ifdef USE_VL53
        b.Label(kk::lbl_static,    "Выход: человек в проёме");
#endif
    }

    // ---- WiFi ----
    {
        sets::Menu m(b, "Настройки WiFi");
        b.Input(kk::wifi_ssid, "Название сети (SSID)");
        b.Pass (kk::wifi_pass, "Пароль сети", "***");
        if (b.Button(kk::wifi_apply, "Подключиться")) {
            db.update();
            WiFiConnector.connect(db[kk::wifi_ssid], db[kk::wifi_pass]);
        }
    }

#ifdef USE_MQTT
    // ---- MQTT ----
    {
        sets::Menu m(b, "Настройки MQTT");
        b.Input(kk::mqtt_broker, "IP адрес брокера");
        b.Input(kk::mqtt_port,   "Порт (обычно 1883)");
        b.Input(kk::mqtt_user,   "Имя пользователя");
        b.Pass (kk::mqtt_pass,   "Пароль", "***");
        b.Input(kk::device_id,   "ID устройства (пусто = авто по MAC)");
        b.Input(kk::pub_interval,"Интервал публикации (мс)");
        if (b.Button(kk::mqtt_apply, "Применить")) {
            db.update();
            mqttNeedsReconnect = true;
        }
    }
#endif

    // ---- LD2410C ----
    {
        sets::Menu m(b, "Радар LD2410C");
        b.Slider(kk::sensor_maxdist, "Максимальная дистанция обнаружения", 75, 600, 75, "см");
    }

#ifdef USE_VL53
    // ---- VL53L1X ----
    {
        sets::Menu m(b, "Датчик зоны VL53L1X");
        b.Slider(kk::vl53_threshold, "Порог занятости проёма", 50, 3000, 50, "мм");
        b.Label(kk::lbl_vl53, "Текущее расстояние");
    }
#endif

    // ---- Логика двери ----
    {
        sets::Menu m(b, "Логика двери");
        b.Slider(kk::door_approach_delta, "Скорость приближения для открытия",  5,  50,   5, "см/с");
        b.Slider(kk::door_open_dist,      "Дистанция при которой открывать",   25, 500,  25, "см");
        b.Slider(kk::door_close_delay,    "Задержка закрытия после прохода",    0, 10000, 500, "мс");
    }
}

// ============================================================
//  Сенсорный таск — высокий приоритет, Core 1
//  Всегда работает независимо от WiFi/MQTT
// ============================================================
void sensorTask(void*) {
    esp_task_wdt_add(NULL);  // регистрируем таск в TWDT

    TickType_t    lastWake      = xTaskGetTickCount();
    unsigned long lastSecond    = 0;
    uint8_t       heartbeatTick = 0;

#ifdef USE_VL53
    unsigned long lastVl53Read  = millis();  // время последнего успешного чтения
#endif

    while (true) {
        esp_task_wdt_reset();

        // ── Heartbeat LED — мигает каждые 500мс (10 тиков × 50мс) ──
        if (++heartbeatTick >= 10) {
            heartbeatTick = 0;
            digitalWrite(LED_HEARTBEAT_PIN, !digitalRead(LED_HEARTBEAT_PIN));
        }

        // ── LD2410C — опрос каждые 50мс ──────────────────────
        sensor.update();
        const LD2410Data& d = sensor.data();

        snap.presence     = d.presence();
        snap.isMoving     = d.isMoving();
        snap.isStatic     = d.isStatic();
        snap.movingDist   = d.movingDist;
        snap.staticDist   = d.staticDist;
        snap.movingEnergy = d.movingEnergy;

#ifdef USE_VL53
        // ── VL53L1X — неблокирующее чтение ───────────────────
        if (vl53ok && !vl53Failed) {
            if (vl53.dataReady()) {
                vl53.read(false);
                if (!vl53.timeoutOccurred()) {
                    lastVl53Read     = millis();
                    snap.vl53dist    = vl53.ranging_data.range_mm;
                    snap.zoneBlocked = snap.vl53dist < cfg_vl53Threshold;
                    digitalWrite(DOOR_ZONE_PIN, snap.zoneBlocked ? HIGH : LOW);
                }
            }
            // 3 секунды без данных → аварийный режим
            if (millis() - lastVl53Read > 3000) {
                vl53Failed       = true;
                snap.zoneBlocked = false;
                digitalWrite(DOOR_ZONE_PIN, LOW);
                Serial.println("[VL53] Датчик не отвечает — аварийный режим");
            }
        }
#endif

        // ── Error LED ─────────────────────────────────────────
        bool anyError = false;
#ifdef USE_VL53
        anyError = vl53Failed;
#endif
        digitalWrite(LED_ERROR_PIN, anyError ? HIGH : LOW);

        // ── Логика двери — раз в секунду ─────────────────────
        unsigned long now = millis();
        if (now - lastSecond >= 1000) {
            lastSecond = now;

            // Если VL53 сломан — гарантируем минимальную задержку закрытия 1.5с
            uint32_t safeCloseDelay = cfg_closeDelay;
#ifdef USE_VL53
            if (vl53Failed && safeCloseDelay < 1500) safeCloseDelay = 1500;
#endif
            door.update(
                d.movingDist,
                d.presence(),
                snap.zoneBlocked,
                cfg_approachDelta,
                cfg_openDist,
                safeCloseDelay
            );
            snap.doorOpen = door.isDoorOpen();
            snap.moveDir  = (uint8_t)door.direction();
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(50));
    }
}

// ============================================================
//  Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    {
        uint32_t t = millis();
        while (!Serial && millis() - t < 5000) delay(10);
    }
    Serial.println("\n=== Presence Sensor ===");

    LittleFS.begin(true);

    db.begin();
    db.init(kk::wifi_ssid,          (String)"");
    db.init(kk::wifi_pass,          (String)"");
    db.init(kk::mqtt_broker,        (String)"");
    db.init(kk::mqtt_port,          (uint16_t)1883);
    db.init(kk::mqtt_user,          (String)"");
    db.init(kk::mqtt_pass,          (String)"");
    db.init(kk::device_id,          (String)"");
    db.init(kk::pub_interval,       (uint16_t)500);
    db.init(kk::sensor_maxdist,     (uint16_t)300);
    db.init(kk::vl53_threshold,     (uint16_t)500);
    db.init(kk::door_approach_delta,(uint16_t)15);
    db.init(kk::door_open_dist,     (uint16_t)200);
    db.init(kk::door_close_delay,   (uint16_t)3000);

    // Кэш настроек для sensorTask
    cfg_vl53Threshold = (int)db[kk::vl53_threshold];
    cfg_approachDelta = (int)db[kk::door_approach_delta];
    cfg_openDist      = (int)db[kk::door_open_dist];
    cfg_closeDelay    = (uint32_t)(int)db[kk::door_close_delay];

    setupIds();

    // Выходы на оптопары
    pinMode(DOOR_OPEN_PIN, OUTPUT);  digitalWrite(DOOR_OPEN_PIN, LOW);
    pinMode(DOOR_ZONE_PIN, OUTPUT);  digitalWrite(DOOR_ZONE_PIN, LOW);
    door.begin(DOOR_OPEN_PIN);

    // Индикация
    pinMode(LED_HEARTBEAT_PIN, OUTPUT);  digitalWrite(LED_HEARTBEAT_PIN, LOW);
    pinMode(LED_ERROR_PIN,     OUTPUT);  digitalWrite(LED_ERROR_PIN,     LOW);

#ifdef USE_VL53
    Wire.begin(VL53_SDA_PIN, VL53_SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeOut(10);  // 10мс макс на I2C транзакцию — защита от lockup
    delay(200);
    vl53.setTimeout(500);
    vl53ok = vl53.init();
    Serial.printf("[VL53L1X] init() = %s\n", vl53ok ? "OK" : "FAIL");
    if (vl53ok) {
        vl53.setDistanceMode(VL53L1X::Short);
        vl53.setMeasurementTimingBudget(50000);
        vl53.startContinuous(100);
    }
#endif

    Serial1.begin(256000, SERIAL_8N1, LD2410_RX_PIN, LD2410_TX_PIN);
    sensor.begin(Serial1, LD2410_RX_PIN, LD2410_TX_PIN, LD2410_OUT_PIN);
    Serial.printf("[LD2410] RX=%d TX=%d OUT=%d\n", LD2410_RX_PIN, LD2410_TX_PIN, LD2410_OUT_PIN);

    // WiFi события для диагностики
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        switch (event) {
            case ARDUINO_EVENT_WIFI_STA_CONNECTED:
                Serial.printf("[WiFi] Ассоциирован с '%s'\n",
                    (char*)info.wifi_sta_connected.ssid);
                break;
            case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
                uint8_t r = info.wifi_sta_disconnected.reason;
                const char* txt =
                    r == 15  ? "неверный пароль" :
                    r == 200 ? "AP пропала"      :
                    r == 201 ? "AP не найдена"   : "другая";
                Serial.printf("[WiFi] Разрыв, причина %d: %s\n", r, txt);
                break;
            }
            default: break;
        }
    });

    WiFiConnector.onConnect([]() {
        Serial.printf("[WiFi] Подключён: %s\n", WiFi.localIP().toString().c_str());
        setupIds();
#ifdef USE_MQTT
        mqttNeedsConnect = true;
#endif
        WiFiConnector.closeAP(true);
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

    // Скан + подключение к лучшему AP
    {
        String ssid = (String)db[kk::wifi_ssid];
        String pass = (String)db[kk::wifi_pass];
        if (ssid.isEmpty()) {
            // ssid = "YOUR_SSID";
            // pass = "YOUR_PASS";
        }

        WiFi.setTxPower(WIFI_POWER_11dBm);
        WiFiConnector.setName(String("Sensor-") + deviceId);
        WiFiConnector.setPass("12345678");

        if (!ssid.isEmpty()) {
            Serial.println("[WiFi] Сканирую сети...");
            int n = WiFi.scanNetworks();
            int bestIdx = -1; int bestRssi = -999;
            for (int i = 0; i < n; i++) {
                Serial.printf("  '%s' RSSI:%d BSSID:%s\n",
                    WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.BSSIDstr(i).c_str());
                if (WiFi.SSID(i) == ssid && WiFi.RSSI(i) > bestRssi) {
                    bestRssi = WiFi.RSSI(i); bestIdx = i;
                }
            }
            if (bestIdx >= 0) {
                int     bestCh = WiFi.channel(bestIdx);
                uint8_t bestBssid[6];
                memcpy(bestBssid, WiFi.BSSID(bestIdx), 6);
                Serial.printf("[WiFi] -> '%s' BSSID:%s ch:%d\n",
                    ssid.c_str(), WiFi.BSSIDstr(bestIdx).c_str(), bestCh);
                WiFi.scanDelete();
                WiFiConnector.connect(ssid, pass);
                WiFi.begin(ssid.c_str(), pass.c_str(), bestCh, bestBssid);
            } else {
                Serial.printf("[WiFi] '%s' не найдена, поднимаю AP\n", ssid.c_str());
                WiFi.scanDelete();
                WiFiConnector.connect("", "");
            }
        } else {
            WiFiConnector.connect("", "");
        }
    }

    sett.setTitle(String("Sensor-") + deviceId);
    sett.onBuild(buildUI);
    sett.begin();

    Serial.printf("[INFO] Device ID: %s  Build: %s%s\n",
        deviceId,
#ifdef USE_MQTT
        "MQTT "
#else
        ""
#endif
        ,
#ifdef USE_VL53
        "VL53"
#else
        ""
#endif
    );

    // Task Watchdog — 5 сек, при срабатывании паника → ресет
    {
        const esp_task_wdt_config_t wdt_cfg = { .timeout_ms = 5000, .idle_core_mask = 0, .trigger_panic = true };
        esp_task_wdt_reconfigure(&wdt_cfg);
    }

    // Запуск сенсорного таска — в самом конце setup()
    xTaskCreatePinnedToCore(
        sensorTask,
        "sensor",
        4096,    // стек
        nullptr,
        5,       // приоритет 5 > Arduino loop (1)
        nullptr,
        1        // Core 1 (Arduino тоже на Core 1, но с меньшим приоритетом)
    );
}

// ============================================================
//  Loop — WiFi, MQTT, веб-интерфейс
//  Sensor task вытесняет этот loop когда активен
// ============================================================
static unsigned long timerApLog    = 0;
static unsigned long timerUiUpdate = 0;

void loop() {
    delay(1);  // FreeRTOS idle

    WiFiConnector.tick();
    sett.tick();

    // AP статус пока нет WiFi
    if (!WiFiConnector.connected() && millis() - timerApLog >= 1000) {
        timerApLog = millis();
        if (WiFiConnector.connecting()) {
            Serial.printf("[WiFi] Подключаюсь... (%lus)\n", millis() / 1000);
        } else {
            Serial.printf("[AP] http://%s  (Sensor-%s / 12345678)\n",
                WiFi.softAPIP().toString().c_str(), deviceId);
        }
    }

    if (!WiFiConnector.connected()) return;

    unsigned long now = millis();

    // Кэш настроек — обновляем раз в секунду
    if (now - timerUiUpdate >= 1000) {
        timerUiUpdate = now;

        cfg_vl53Threshold = (int)db[kk::vl53_threshold];
        cfg_approachDelta = (int)db[kk::door_approach_delta];
        cfg_openDist      = (int)db[kk::door_open_dist];
        cfg_closeDelay    = (uint32_t)(int)db[kk::door_close_delay];

        // Расстояние: движущийся приоритетнее статического
        uint16_t displayDist = snap.movingDist ? snap.movingDist : snap.staticDist;
        String vDist    = snap.presence
            ? (String(displayDist) + " см")
            : "объектов нет";
        String vWifi    = WiFi.localIP().toString();
        String vDir     = door.directionStr();
        String vDoorOut = snap.doorOpen   ? "АКТИВЕН" : "неактивен";

        db[kk::lbl_wifi]     = vWifi;
        db[kk::lbl_mov_dist] = vDist;
        db[kk::lbl_door]     = vDir;
        db[kk::lbl_presence] = vDoorOut;

        auto upd = sett.updater()
            .update(kk::lbl_wifi,     vWifi)
            .update(kk::lbl_mov_dist, vDist)
            .update(kk::lbl_door,     vDir)
            .update(kk::lbl_presence, vDoorOut);

#ifdef USE_MQTT
        String vMqtt = mqttClient.connected() ? "подключён" : "отключён";
        db[kk::lbl_mqtt] = vMqtt;
        upd.update(kk::lbl_mqtt, vMqtt);
#endif

#ifdef USE_VL53
        String vVl53 = !vl53ok
            ? "ОШИБКА: датчик не найден при старте"
            : vl53Failed
                ? "ОШИБКА: датчик не отвечает (задержка закрытия ≥1.5с)"
                : (String(snap.vl53dist) + " мм  — " + (snap.zoneBlocked ? "ЗАНЯТ" : "свободен"));
        String vZoneOut = snap.zoneBlocked ? "АКТИВЕН" : "неактивен";
        db[kk::lbl_vl53]   = vVl53;
        db[kk::lbl_static] = vZoneOut;
        upd.update(kk::lbl_vl53,   vVl53)
           .update(kk::lbl_static, vZoneOut);
#endif
    }

#ifdef USE_MQTT
    // Первое подключение после WiFi
    if (mqttNeedsConnect) {
        mqttNeedsConnect = false;
        lastMqttCheck = now;
        connectMQTT();
    }

    // MQTT watchdog
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

        uint16_t interval = (int)db[kk::pub_interval];
        if (!interval) interval = 500;

        static bool prevPresence = false;
        bool presenceChanged = (snap.presence != prevPresence);
        if (presenceChanged || (now - lastPublish >= interval) || firstPublish) {
            publishState();
            prevPresence = snap.presence;
            lastPublish  = now;
            firstPublish = false;
        }
    }
#endif // USE_MQTT
}
