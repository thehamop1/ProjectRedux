#include "CanCommunication.h"
#include <iostream>
#include <cstring>
#include <chrono>

CanCommunication::CanCommunication(std::string_view interface): m_interface(interface){
    if(!m_CanSocket.Open(m_interface)){
        std::cerr << "ERROR: Could not setup socket." << std::endl;
    }
};

void CanCommunication::StartThreads(){
    m_ReceiveThread = std::thread([&](){
        RecieveThread();
    });

    m_SendThread = std::thread([&](){
        SendThread();
    });
};

CanCommunication::~CanCommunication(){
    StopThreads();
    m_ReceiveThread.join();
    m_SendThread.join();
    m_CanSocket.Close();
};

void CanCommunication::SendThread(){
    bool successfullySent=false;

    while(m_alive){
        successfullySent = false;

        auto msg = [&] -> std::shared_ptr<can_frame> {
            std::scoped_lock<std::mutex> lock(m_queueLock);
            if(m_queue.empty()) return nullptr;
            auto msg = m_queue.front();
            m_queue.pop();
            return msg;
        }();

        if(msg == nullptr){
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            continue;
        }

        while(!successfullySent && m_alive){
            if(m_CanSocket.IsConnected()){
                successfullySent = m_CanSocket.Send(msg);
                if(!successfullySent){
                    std::cerr << "ERROR: Failed to send msg." << std::endl;
                }
            }else{
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }
};

void CanCommunication::RecieveThread(){
    std::shared_ptr<can_frame> frame = std::make_shared<can_frame>();

    while(m_alive){
        if(!m_CanSocket.IsConnected()){
            m_CanSocket.Open(m_interface);
            continue;
        }

        std::memset(frame.get(), 0, sizeof(can_frame));

        if(m_CanSocket.Recieve(frame)){
            auto f = [&] -> Callback{
                std::scoped_lock<std::mutex> lock(m_callbackLock);
                auto it = m_callbacks.find(frame->can_id);
                if(it==m_callbacks.end()){
                    return [&](const std::shared_ptr<can_frame> frame){DefaultCallback(frame);};
                }
                return it->second;
            }();
            
            f(frame);
        }else{
            std::this_thread::sleep_for(std::chrono::seconds(1));
        };
    }
};

void CanCommunication::DefaultCallback(const std::shared_ptr<can_frame> frame){
    std::cout << "WARNING: Unknown frame recieved!" << std::endl;
    std::cout << "ID: " << frame->can_id << std::endl;
}

bool CanCommunication::Send(std::shared_ptr<can_frame> frame){
    std::scoped_lock<std::mutex> lock(m_queueLock);
    m_queue.push(frame);
    return true;
};


bool CanCommunication::AddCallback(const int16_t& id, const Callback& f){
    std::scoped_lock<std::mutex> lock(m_callbackLock);
    if(m_callbacks.contains(id)){
        std::cerr << "WARNING: Callback already registered." << std::endl;
        return false;
    }

    m_callbacks[id] = f;
    return true;
};