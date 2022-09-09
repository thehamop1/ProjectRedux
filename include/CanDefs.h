#pragma once
#include <string_view>
#include <linux/can.h>

constexpr std::string_view DEFAULT_CAN_CHANNEL{"can0"};
constexpr uint8_t DEFAULT_DLC_SIZE = 8;

constexpr canid_t BMC_ADDR=0x201;

constexpr uint8_t SPEED_ID = 0x31;
constexpr int MAX_RPM = 4100;

constexpr uint8_t SPEED_MODE_ID = 0x36;
constexpr uint8_t SPEED_MODE_ID_2 = 0x12;
constexpr uint8_t ANALOG_CONTROL = 0x20;
constexpr uint8_t DIGITAL_CONTROL = 0x00;

constexpr uint8_t CLEAR_CODES_ID = 0x8E;
constexpr uint8_t LBIT_CLEAR_CODE = 0x54;
constexpr uint8_t RBIT_CLEAR_CODE = 0x41;