#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <functional>

// ============================================================
//  WifiManager — замена WiFiConnector
//
//  Логика:
//   1. begin(ssid, pass) → скан для лучшего BSSID (mesh)
//   2. Поднимаем AP+STA пока ждём подключения
//   3. Подключились → закрываем AP, режим STA
//   4. Потеряли соединение → переподключаемся (без ресканирования)
//   5. N неудачных попыток → ресканирование + AP fallback
//   6. begin("","") → сразу AP
// ============================================================
class WifiManager {
public:
    using Cb    = std::function<void()>;
    using ErrCb = std::function<void(const char*)>;

    enum class State : uint8_t { Idle, Connecting, Connected, APOnly };

    // ── Конфигурация ────────────────────────────────────────
    void setAPCredentials(const String& name, const String& pass) {
        _apName = name;
        _apPass = pass;
    }
    void setTimeout(uint16_t sec)        { _timeout  = sec * 1000UL; }
    void setTxPower(wifi_power_t power)  { _txPower  = power; }
    void setMaxRetries(uint8_t n)        { _maxRetry = n; }

    // ── Коллбеки ────────────────────────────────────────────
    void onConnect   (Cb    cb) { _cbConnect    = cb; }
    void onDisconnect(Cb    cb) { _cbDisconnect = cb; }
    void onAPStart   (Cb    cb) { _cbAP         = cb; }
    void onError     (ErrCb cb) { _cbError      = cb; }

    // ── Запуск ──────────────────────────────────────────────
    // ssid пустой → сразу AP, иначе скан + подключение
    void begin(const String& ssid, const String& pass) {
        _ssid = ssid;
        _pass = pass;
        _retryCount = 0;

        // WiFi.mode() ОБЯЗАТЕЛЕН до setTxPower
        WiFi.mode(WIFI_STA);
        WiFi.setTxPower(_txPower);

        if (ssid.isEmpty()) {
            _launchAP("SSID не задан");
            return;
        }

        if (_scan()) {
            _connect();
        } else {
            _launchAP("сеть не найдена при скане");
        }
    }

    // ── tick() — вызывать в loop() ──────────────────────────
    void tick() {
        if (_state != State::Connecting) return;

        wl_status_t status = WiFi.status();

        if (status == WL_CONNECTED) {
            _retryCount = 0;
            _state = State::Connected;
            _closeAP();
            Serial.printf("[WiFi] Подключён: %s\n",
                WiFi.localIP().toString().c_str());
            if (_cbConnect) _cbConnect();
            return;
        }

        if (millis() - _connectStart < _timeout) return;

        // Таймаут
        const char* err = _statusStr(status);
        Serial.printf("[WiFi] Таймаут (%s), попытка %d/%d\n",
            err, _retryCount + 1, _maxRetry);

        if (++_retryCount < _maxRetry) {
            // Повторяем с тем же BSSID
            _connect();
        } else {
            // Исчерпали попытки → рескан
            Serial.println("[WiFi] Пересканирую...");
            _retryCount = 0;
            if (!_scan()) {
                if (_cbError) _cbError(err);
                _launchAP(err);
            } else {
                _connect();
            }
        }
    }

    // Вызвать когда WiFi упал (из WiFi.onEvent или poll в loop)
    void notifyDisconnect() {
        if (_state != State::Connected) return;
        Serial.println("[WiFi] Соединение потеряно, переподключаюсь...");
        _state = State::Connecting;
        if (_cbDisconnect) _cbDisconnect();
        _connect();
    }

    // Сброс и повторное подключение (напр., при смене credentials)
    void reconnect(const String& ssid, const String& pass) {
        WiFi.disconnect(true);
        _state = State::Idle;
        begin(ssid, pass);
    }

    // ── Состояние ────────────────────────────────────────────
    bool  connected()  const { return _state == State::Connected;  }
    bool  connecting() const { return _state == State::Connecting; }
    bool  apActive()   const { return _apActive; }
    State state()      const { return _state; }

    const char* stateStr() const {
        switch (_state) {
            case State::Idle:       return "idle";
            case State::Connecting: return "connecting";
            case State::Connected:  return "connected";
            case State::APOnly:     return "ap_only";
        }
        return "?";
    }

private:
    String  _ssid, _pass;
    String  _apName  = "ESP-AP";
    String  _apPass  = "";
    uint8_t _bssid[6] = {};
    int     _channel   = 0;
    bool    _hasBssid  = false;

    uint32_t     _timeout      = 20000;    // 20 сек на попытку
    uint32_t     _connectStart = 0;
    wifi_power_t _txPower      = WIFI_POWER_11dBm;
    State        _state        = State::Idle;
    bool         _apActive     = false;
    uint8_t      _retryCount   = 0;
    uint8_t      _maxRetry     = 3;

    Cb    _cbConnect, _cbDisconnect, _cbAP;
    ErrCb _cbError;

    // Сканирует и сохраняет лучший BSSID. Возвращает false если не нашёл.
    bool _scan() {
        Serial.println("[WiFi] Сканирую...");
        int n = WiFi.scanNetworks(false, false);

        int bestIdx  = -1;
        int bestRssi = -999;
        for (int i = 0; i < n; i++) {
            Serial.printf("  [%d] '%s'  RSSI:%d  BSSID:%s  ch:%d\n",
                i, WiFi.SSID(i).c_str(), WiFi.RSSI(i),
                WiFi.BSSIDstr(i).c_str(), WiFi.channel(i));
            if (WiFi.SSID(i) == _ssid && WiFi.RSSI(i) > bestRssi) {
                bestRssi = WiFi.RSSI(i);
                bestIdx  = i;
            }
        }

        if (bestIdx < 0) {
            WiFi.scanDelete();
            Serial.printf("[WiFi] '%s' не найдена\n", _ssid.c_str());
            _hasBssid = false;
            return false;
        }

        memcpy(_bssid, WiFi.BSSID(bestIdx), 6);
        _channel  = WiFi.channel(bestIdx);
        _hasBssid = true;
        Serial.printf("[WiFi] Выбран BSSID:%s  ch:%d  RSSI:%d\n",
            WiFi.BSSIDstr(bestIdx).c_str(), _channel, bestRssi);
        WiFi.scanDelete();
        return true;
    }

    void _connect() {
        _state        = State::Connecting;
        _connectStart = millis();

        // AP+STA пока ждём — чтобы можно было зайти в UI при проблемах
        WiFi.mode(WIFI_AP_STA);
        WiFi.setTxPower(_txPower);
        if (!_apActive) {
            WiFi.softAP(_apName.c_str(), _apPass.c_str());
            _apActive = true;
        }

        if (_hasBssid) {
            WiFi.begin(_ssid.c_str(), _pass.c_str(), _channel, _bssid);
        } else {
            WiFi.begin(_ssid.c_str(), _pass.c_str());
        }
        Serial.printf("[WiFi] Подключаюсь к '%s'  попытка %d/%d...\n",
            _ssid.c_str(), _retryCount + 1, _maxRetry);
    }

    void _launchAP(const char* reason) {
        _state = State::APOnly;
        // Не вызываем WiFi.disconnect(true)/WiFi.mode() из loop() —
        // на одноядерном ESP32-C3 это может конфликтовать с WiFi-стеком.
        // Просто поднимаем AP поверх текущего состояния.
        WiFi.softAP(_apName.c_str(), _apPass.c_str());
        WiFi.setTxPower(_txPower);
        _apActive = true;
        Serial.printf("[WiFi] AP '%s' запущена (причина: %s), IP: %s\n",
            _apName.c_str(), reason, WiFi.softAPIP().toString().c_str());
        if (_cbAP) _cbAP();
    }

    void _closeAP() {
        if (!_apActive) return;
        // НЕ меняем WiFi.mode() здесь — на ESP32-C3 один физический
        // процессор, вызов mode() из loop() пока WiFi-стек обрабатывает
        // события connect приводит к Store access fault (Core 0 panic).
        // softAPdisconnect(true) физически останавливает AP,
        // режим AP_STA остаётся — это нормально.
        WiFi.softAPdisconnect(true);
        WiFi.setTxPower(_txPower);
        _apActive = false;
        Serial.println("[WiFi] AP закрыта");
    }

    static const char* _statusStr(wl_status_t s) {
        switch (s) {
            case WL_NO_SSID_AVAIL:   return "сеть не найдена";
            case WL_CONNECT_FAILED:  return "неверный пароль";
            case WL_CONNECTION_LOST: return "соединение потеряно";
            case WL_DISCONNECTED:    return "отключён";
            default:                 return "неизвестная ошибка";
        }
    }
};

// Глобальный экземпляр
inline WifiManager wifiMgr;
