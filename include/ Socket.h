#pragma once
#include <string_view>

constexpr std::string_view DEFAULT_CAN_CHANNEL{"can0"};

class Socket
{
public:
    bool open_port(std::string_view port=DEFAULT_CAN_CHANNEL);
private:
    int m_socket=0;
};