#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

// ============================================================
//  Драйвер HLK-LD2410C — парсинг UART фреймов
//  Baud rate: 256000, фрейм данных ~23 байта
// ============================================================

// Статус цели
enum class TargetStatus : uint8_t {
    None       = 0x00,
    Moving     = 0x01,
    Stationary = 0x02,
    Both       = 0x03
};

struct LD2410Data {
    TargetStatus status        = TargetStatus::None;
    uint16_t     movingDist    = 0;   // см
    uint8_t      movingEnergy  = 0;   // 0–100
    uint16_t     staticDist    = 0;   // см
    uint8_t      staticEnergy  = 0;   // 0–100
    uint16_t     detectDist    = 0;   // см (общая дистанция обнаружения)
    bool         outPin        = false;

    bool presence()   const { return status != TargetStatus::None; }
    bool isMoving()   const { return (uint8_t)status & 0x01; }
    bool isStatic()   const { return (uint8_t)status & 0x02; }
};

class LD2410 {
public:
    // Инициализация: передаём ссылку на Serial1
    void begin(HardwareSerial& serial, int rxPin, int txPin, int outPin);

    // Вызывать в loop() — читает входной буфер, возвращает true при новом фрейме
    bool update();

    // Последние разобранные данные
    const LD2410Data& data() const { return _data; }

    // Команда включения инженерного режима (расширенные данные)
    void enableEngineeringMode();
    void disableEngineeringMode();

private:
    HardwareSerial* _serial  = nullptr;
    int             _outPin  = -1;

    // Буфер фрейма — максимальный размер с запасом
    static constexpr size_t FRAME_MAX = 64;
    uint8_t  _buf[FRAME_MAX];
    size_t   _bufLen = 0;

    LD2410Data _data;

    bool readFrame();
    bool parseDataFrame(const uint8_t* payload, size_t len);
    void sendCommand(const uint8_t* cmd, size_t len);

    // Заголовки фреймов протокола LD2410
    static constexpr uint8_t FRAME_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
    static constexpr uint8_t FRAME_FOOTER[4] = {0x04, 0x03, 0x02, 0x01};
    static constexpr uint8_t DATA_HEADER[4]  = {0xF4, 0xF3, 0xF2, 0xF1};
    static constexpr uint8_t DATA_FOOTER[4]  = {0xF8, 0xF7, 0xF6, 0xF5};
};
