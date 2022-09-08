#include "CanSocket.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>

bool CanSocket::Open(std::string_view interface)
{
    std::scoped_lock lock(m_socketLock);
    if(interface.size()>IF_NAMESIZE){
        std::cerr << "ERROR: Invalid interface name!" << std::endl;
    }

    struct ifreq ifr;
    struct sockaddr_can addr;

    /* open socket */
    m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_socket < 0)
    {
        std::cerr << "ERROR: Could not create socket!" << std::endl;
        return false;
    }

    addr.can_family = AF_CAN;

    std::memcpy(ifr.ifr_name, interface.data(), interface.size());

    if (ioctl(m_socket, SIOCGIFINDEX, &ifr) < 0)
    {
        std::cerr << "ERROR: Could not return interface index!" << std::endl;
        return false;
    }

    addr.can_ifindex = ifr.ifr_ifindex;

    if (fcntl(m_socket, F_SETFL, O_NONBLOCK) < 0)
    {
        std::cerr << "WARNING: Could not set socket to Non-Blocking!" << std::endl;
    }

    if (bind(m_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        std::cerr << "ERROR: Could not bind socket!" << std::endl;
        return false;
    }
    m_connected=true;
    return true;
};

void CanSocket::Close()
{
    std::scoped_lock lock(m_socketLock);
    if(close(m_socket) < 0){
        std::cerr << "ERROR: Could not close socket file descriptor." << std::endl;
    }
}

bool CanSocket::Send(const std::shared_ptr<can_frame> frame) {
    if(frame==nullptr){
        std::cerr << "WARNING: Invalid frame send." << std::endl;
        return false;
    } 

    int bytesToWrite = sizeof(can_frame);
    int ret=0;
    while(bytesToWrite>0){
        ret = write(m_socket, frame.get(), bytesToWrite);
        if(ret<0){
            m_connected=false;
            std::cerr << "ERROR: Socket send error socket invalid." << std::endl;
            return false;
        }else{
            bytesToWrite-=ret;
        }
    }
    return true;
};

bool CanSocket::Recieve(std::shared_ptr<can_frame> frame){
    if(frame==nullptr){
        std::cerr << "WARNING: Invalid pointer given." << std::endl;
        return false;
    } 
    int bytesToRead = sizeof(frame);
    int bytes=0;

    while(bytesToRead>0){
        bytes = read(m_socket, frame.get(), bytesToRead);
        if (bytes<0)
        {
            m_connected=false;
            std::cerr << "ERROR: Socket receive error socket invalid" << std::endl;
            return false;
        }
        bytesToRead-=bytes;
    }
    return true;
};