#pragma once
#include "CanSocket.h"
#include <thread>
#include <atomic>
#include <functional>
#include <mutex>
#include <queue>
#include <array>
#include <cstddef>
#include <memory>
#include <unordered_map> 

using Callback = std::function<void(const std::shared_ptr<can_frame>)>;

class CanCommunication
{
public:
    CanCommunication(std::string_view interface=DEFAULT_CAN_CHANNEL);
    ~CanCommunication();

    void StopThreads() { m_alive = false; };
    inline bool IsSetup() const {return m_CanSocket.IsConnected();}

    bool AddCallback(const int16_t& id, const Callback& f);
    bool Send(std::shared_ptr<can_frame> frame);
    void StartThreads();
private:
    virtual void DefaultCallback(const std::shared_ptr<can_frame> frame);
    void SendThread();
    void RecieveThread();

    CanSocket m_CanSocket;
    std::string m_interface;
    std::atomic<bool> m_alive{true};
    std::thread m_ReceiveThread, m_SendThread;
    
    std::mutex m_queueLock;
    std::queue<std::shared_ptr<can_frame>> m_queue; 

    std::mutex m_callbackLock;
    std::unordered_map<int16_t, Callback> m_callbacks;
};
