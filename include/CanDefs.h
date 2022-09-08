#pragma once
#include <string_view>
#include <linux/can.h>

constexpr std::string_view DEFAULT_CAN_CHANNEL{"can0"};
constexpr uint8_t DEFAULT_DLC_SIZE = 8;

constexpr canid_t BMC_ADDR=0x201;

constexpr uint8_t SPEED_ID = 0x31;
constexpr uint8_t SPEED_MODE_ID = 0x36;
constexpr uint8_t CLEAR_CODES_ID = 0x8E;