#pragma once
#include <Arduino.h>
#include <Wire.h>

#define PCF8563_ADDRESS   0x51
#define PCF8563_REG_CS1   0x00
#define PCF8563_REG_CS2   0x01
#define PCF8563_REG_SEC   0x02
#define PCF8563_REG_MIN   0x03
#define PCF8563_REG_HOUR  0x04
#define PCF8563_REG_DAY   0x05
#define PCF8563_REG_WDAY  0x06
#define PCF8563_REG_MON   0x07
#define PCF8563_REG_YEAR  0x08

struct DateTime {
    uint16_t year;    // e.g. 2025
    uint8_t  month;   // 1–12
    uint8_t  day;     // 1–31
    uint8_t  hour;    // 0–23
    uint8_t  minute;  // 0–59
    uint8_t  second;  // 0–59
    uint8_t  weekday; // 0–6 (0 = Sunday)
};

class PCF8563 {
public:
    // Call Wire.begin() before this. Only pass sda/scl for non-default pins.
    bool     begin(TwoWire &wire = Wire, int sda = -1, int scl = -1);
    bool     isRunning();
    void     setDateTime(const DateTime &dt);
    DateTime getDateTime();
    uint32_t getUnixTime();   // Unix epoch seconds (UTC)

private:
    TwoWire *_wire;

    uint8_t bcdToDec(uint8_t bcd);
    uint8_t decToBcd(uint8_t dec);
    void    writeReg(uint8_t reg, uint8_t val);
    uint8_t readReg(uint8_t reg);
};
