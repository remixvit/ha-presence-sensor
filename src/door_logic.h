#pragma once
#include <Arduino.h>

// ============================================================
//  Направление движения объекта
// ============================================================
enum class MoveDir : uint8_t {
    Unknown    = 0,
    Approaching,   // объект приближается к сенсору
    Leaving,       // объект удаляется от сенсора
    Stationary,    // объект есть, но не движется
};

// ============================================================
//  DoorLogic
//
//  Вызывать update() раз в секунду с текущими данными.
//  Управляет DOOR_OPEN_PIN через digitalWrite.
// ============================================================
class DoorLogic {
public:
    // openPin  — GPIO на оптопару "открыть дверь"
    void begin(uint8_t openPin) {
        _pin = openPin;
    }

    // Вызывать раз в секунду (в таймере 1000мс)
    // movingDist    — расстояние до движущегося объекта (см), 0 если нет
    // presence      — есть ли объект вообще (LD2410C)
    // zoneBlocked   — человек в проёме (VL53L1X)
    // approachDelta — порог изменения дистанции для "приближения" (см/сек)
    // openDist      — макс. дистанция при которой открывать (см)
    // closeDelay    — задержка закрытия после освобождения проёма (мс)
    void update(uint16_t movingDist, bool presence, bool zoneBlocked,
                uint16_t approachDelta, uint16_t openDist, uint32_t closeDelay)
    {
        // ── Определяем направление ──────────────────────────
        if (!presence || movingDist == 0) {
            _dir = MoveDir::Unknown;
        } else if (_prevDist == 0) {
            _dir = MoveDir::Unknown;       // первый замер, базы нет
        } else {
            int16_t delta = (int16_t)_prevDist - (int16_t)movingDist;
            // delta > 0  → dist уменьшилась → приближается
            // delta < 0  → dist увеличилась → удаляется
            if (delta >= (int16_t)approachDelta) {
                _dir = MoveDir::Approaching;
            } else if (delta <= -(int16_t)approachDelta) {
                _dir = MoveDir::Leaving;
            } else {
                _dir = presence ? MoveDir::Stationary : MoveDir::Unknown;
            }
        }
        _prevDist = movingDist;

        // ── Управление дверью ────────────────────────────────
        bool shouldOpen = (_dir == MoveDir::Approaching)
                          && presence
                          && (movingDist > 0)
                          && (movingDist <= openDist);

        if (shouldOpen) {
            _openDoor();
            _closeTimerActive = false;
        } else if (_doorOpen) {
            if (zoneBlocked) {
                // человек в проёме — держим открытой
                _closeTimerActive = false;
            } else if (!_closeTimerActive) {
                // проём освободился — запускаем таймер закрытия
                _closeTimer = millis();
                _closeTimerActive = true;
            } else if (millis() - _closeTimer >= closeDelay) {
                _closeDoor();
            }
        }
    }

    MoveDir direction()  const { return _dir; }
    bool    isDoorOpen() const { return _doorOpen; }

    const char* directionStr() const {
        switch (_dir) {
            case MoveDir::Approaching: return "приближается";
            case MoveDir::Leaving:     return "удаляется";
            case MoveDir::Stationary:  return "стоит";
            default:                   return "нет объекта";
        }
    }

private:
    uint8_t  _pin = 0;
    uint16_t _prevDist = 0;
    MoveDir  _dir = MoveDir::Unknown;
    bool     _doorOpen = false;
    bool     _closeTimerActive = false;
    unsigned long _closeTimer = 0;

    void _openDoor() {
        if (!_doorOpen) {
            _doorOpen = true;
            digitalWrite(_pin, HIGH);
            Serial.println("[DOOR] Открыть");
        }
    }

    void _closeDoor() {
        if (_doorOpen) {
            _doorOpen = false;
            _closeTimerActive = false;
            digitalWrite(_pin, LOW);
            Serial.println("[DOOR] Закрыть");
        }
    }
};
