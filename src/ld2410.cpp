#include "ld2410.h"

// ============================================================
//  Константы протокола — определения здесь, чтобы линкер
//  не ругался на static constexpr в заголовке (pre-C++17)
// ============================================================
constexpr uint8_t LD2410::FRAME_HEADER[4];
constexpr uint8_t LD2410::FRAME_FOOTER[4];
constexpr uint8_t LD2410::DATA_HEADER[4];
constexpr uint8_t LD2410::DATA_FOOTER[4];

void LD2410::begin(HardwareSerial& serial, int rxPin, int txPin, int outPin) {
    _serial = &serial;
    _outPin = outPin;

    // LD2410C работает на 256000 бод

    if (outPin >= 0) {
        pinMode(outPin, INPUT);
    }

    _bufLen = 0;
}

bool LD2410::update() {
    if (!_serial) return false;

    // Читаем OUT-пин (быстрый бинарный признак присутствия)
    if (_outPin >= 0) {
        _data.outPin = digitalRead(_outPin) == HIGH;
    }

    return readFrame();
}

// ---------- Разбор байтового потока ----------

bool LD2410::readFrame() {
    bool gotFrame = false;

    while (_serial->available()) {
        uint8_t b = _serial->read();

        // Добавляем байт в буфер (с защитой от переполнения)
        if (_bufLen < FRAME_MAX) {
            _buf[_bufLen++] = b;
        } else {
            // Сдвигаем буфер, отбрасываем старый байт
            memmove(_buf, _buf + 1, FRAME_MAX - 1);
            _buf[FRAME_MAX - 1] = b;
        }

        // Ищем конец фрейма данных (F8 F7 F6 F5)
        if (_bufLen >= 4) {
            size_t tail = _bufLen - 4;
            if (memcmp(_buf + tail, DATA_FOOTER, 4) == 0) {
                // Ищем заголовок фрейма данных (F4 F3 F2 F1)
                for (size_t i = 0; i + 4 <= tail; i++) {
                    if (memcmp(_buf + i, DATA_HEADER, 4) == 0) {
                        // payload: после заголовка (4 байта) до футера (4 байта)
                        size_t payloadStart = i + 4;
                        size_t payloadLen   = tail - payloadStart;
                        if (parseDataFrame(_buf + payloadStart, payloadLen)) {
                            gotFrame = true;
                        }
                        _bufLen = 0;
                        break;
                    }
                }
            }
        }
    }

    return gotFrame;
}

// ---------- Структура фрейма данных LD2410C ----------
//
//  [0]    — Data type: 0x02 = basic target info
//  [1]    — Target status (0x00 none / 0x01 moving / 0x02 static / 0x03 both)
//  [2-3]  — Moving target distance (cm, LE uint16)
//  [4]    — Moving target energy (0–100)
//  [5-6]  — Stationary target distance (cm, LE uint16)
//  [7]    — Stationary target energy (0–100)
//  [8-9]  — Detection distance (cm, LE uint16)
//
bool LD2410::parseDataFrame(const uint8_t* p, size_t len) {
    // Структура после DATA_HEADER (F4 F3 F2 F1):
    // [0][1]  — длина данных (LE), обычно 0x0D 0x00 = 13
    // [2][3]  — тип данных (LE): 0x02 0x00 = базовый режим
    // [4]     — маркер 0xAA
    // [5]     — target state
    // [6][7]  — moving distance (LE, см)
    // [8]     — moving energy (0–100)
    // [9][10] — static distance (LE, см)
    // [11]    — static energy (0–100)
    // [12][13]— detection distance (LE, см)
    // [14]    — 0x55 (хвост)
    // [15]    — 0x00 (контрольный)

    if (len < 13) return false;
    if (p[2] != 0x02) return false;  // тип: 0x02 = базовый режим
    if (p[3] != 0xAA) return false;  // маркер начала данных

    // p[0][1] = длина, p[2] = тип, p[3] = 0xAA, p[4..] = данные
    _data.status       = static_cast<TargetStatus>(p[4]);
    _data.movingDist   = (uint16_t)p[5]  | ((uint16_t)p[6]  << 8);
    _data.movingEnergy = p[7];
    _data.staticDist   = (uint16_t)p[8]  | ((uint16_t)p[9]  << 8);
    _data.staticEnergy = p[10];
    _data.detectDist   = (uint16_t)p[11] | ((uint16_t)p[12] << 8);

    return true;
}

// ---------- Команды конфигурации ----------

void LD2410::sendCommand(const uint8_t* cmd, size_t len) {
    if (!_serial) return;
    _serial->write(FRAME_HEADER, 4);
    _serial->write((uint8_t)(len & 0xFF));
    _serial->write((uint8_t)(len >> 8));
    _serial->write(cmd, len);
    _serial->write(FRAME_FOOTER, 4);
    _serial->flush();
}

void LD2410::enableEngineeringMode() {
    // Команда: открыть конфигурацию → включить engineering mode
    const uint8_t openCfg[] = {0xFF, 0x00, 0x01, 0x00, 0x00, 0x00};
    sendCommand(openCfg, sizeof(openCfg));
    delay(50);
    const uint8_t engMode[] = {0x62, 0x00};
    sendCommand(engMode, sizeof(engMode));
    delay(50);
    const uint8_t closeCfg[] = {0xFE, 0x00};
    sendCommand(closeCfg, sizeof(closeCfg));
}

void LD2410::disableEngineeringMode() {
    const uint8_t openCfg[] = {0xFF, 0x00, 0x01, 0x00, 0x00, 0x00};
    sendCommand(openCfg, sizeof(openCfg));
    delay(50);
    const uint8_t basic[] = {0x63, 0x00};
    sendCommand(basic, sizeof(basic));
    delay(50);
    const uint8_t closeCfg[] = {0xFE, 0x00};
    sendCommand(closeCfg, sizeof(closeCfg));
}