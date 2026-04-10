#include "PCF8563.h"

// ── BCD helpers ──────────────────────────────────────────────
uint8_t PCF8563::bcdToDec(uint8_t bcd) {
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}
uint8_t PCF8563::decToBcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

// ── Low-level I2C ─────────────────────────────────────────────
void PCF8563::writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(PCF8563_ADDRESS);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
}

uint8_t PCF8563::readReg(uint8_t reg) {
    _wire->beginTransmission(PCF8563_ADDRESS);
    _wire->write(reg);
    _wire->endTransmission(false);           // repeated start
    _wire->requestFrom(PCF8563_ADDRESS, 1);
    return _wire->available() ? _wire->read() : 0xFF;
}

// ── Public API ────────────────────────────────────────────────

// Caller is responsible for Wire.begin() before calling this.
// Only calls Wire.begin(sda, scl) if custom pins are supplied.
bool PCF8563::begin(TwoWire &wire, int sda, int scl) {
    _wire = &wire;
    if (sda >= 0 && scl >= 0)
        _wire->begin(sda, scl);
    // default (sda == scl == -1): assume caller already called Wire.begin()

    _wire->beginTransmission(PCF8563_ADDRESS);
    return (_wire->endTransmission() == 0);
}

bool PCF8563::isRunning() {
    // Bit 7 of seconds register is the VL (Voltage Low) flag.
    // VL = 0 → clock reliable; VL = 1 → power was lost.
    return !(readReg(PCF8563_REG_SEC) & 0x80);
}

void PCF8563::setDateTime(const DateTime &dt) {
    writeReg(PCF8563_REG_SEC,  decToBcd(dt.second) & 0x7F);   // clears VL
    writeReg(PCF8563_REG_MIN,  decToBcd(dt.minute) & 0x7F);
    writeReg(PCF8563_REG_HOUR, decToBcd(dt.hour)   & 0x3F);
    writeReg(PCF8563_REG_DAY,  decToBcd(dt.day)    & 0x3F);
    writeReg(PCF8563_REG_WDAY, dt.weekday           & 0x07);
    writeReg(PCF8563_REG_MON,  decToBcd(dt.month)  & 0x1F);
    writeReg(PCF8563_REG_YEAR, decToBcd(dt.year % 100));
}

DateTime PCF8563::getDateTime() {
    DateTime dt;
    dt.second  = bcdToDec(readReg(PCF8563_REG_SEC)  & 0x7F);
    dt.minute  = bcdToDec(readReg(PCF8563_REG_MIN)  & 0x7F);
    dt.hour    = bcdToDec(readReg(PCF8563_REG_HOUR) & 0x3F);
    dt.day     = bcdToDec(readReg(PCF8563_REG_DAY)  & 0x3F);
    dt.weekday =          readReg(PCF8563_REG_WDAY) & 0x07;
    dt.month   = bcdToDec(readReg(PCF8563_REG_MON)  & 0x1F);
    dt.year    = bcdToDec(readReg(PCF8563_REG_YEAR)) + 2000;
    return dt;
}

uint32_t PCF8563::getUnixTime() {
    DateTime dt = getDateTime();

    uint16_t y    = dt.year;
    uint32_t days = (y - 1970) * 365UL
                  + (y - 1969) / 4
                  - (y - 1901) / 100
                  + (y - 1601) / 400;

    static const uint16_t mdays[] = {0,31,59,90,120,151,181,212,243,273,304,334};
    days += mdays[dt.month - 1] + (dt.day - 1);

    bool leap = (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
    if (leap && dt.month > 2) days += 1;

    return days * 86400UL
         + dt.hour   * 3600UL
         + dt.minute * 60UL
         + dt.second;
}
