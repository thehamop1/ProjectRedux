#pragma once
#include "CanDefs.h"

#include <atomic>
#include <mutex>
#include <memory>
#include <string_view>
#include <linux/can.h>
#include <linux/can/raw.h>

class CanSocket
{
public:
    inline bool IsConnected() const{return m_connected;};
    bool Open(std::string_view interface=DEFAULT_CAN_CHANNEL);
    void Close();
    bool Send(const std::shared_ptr<can_frame> frame);
    bool Recieve(std::shared_ptr<can_frame> frame);
private:
    int m_socket=0;
    std::mutex m_socketLock;
    std::atomic<bool> m_connected=false;
};