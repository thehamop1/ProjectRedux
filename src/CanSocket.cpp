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
    std::memset(&ifr, 0, sizeof(ifr));
    std::memset(&addr, 0, sizeof(addr));

    /* open socket */
    m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_socket < 0)
    {
        std::cerr << "ERROR: Could not create socket!" << std::endl;
        return false;
    }

    std::strcpy(ifr.ifr_name, interface.data());
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (ioctl(m_socket, SIOCGIFINDEX, &ifr) < 0)
    {
        std::cerr << "ERROR: Could not return interface index!" << std::endl;
        perror("ERROR: ");
        return false;
    }

    addr.can_ifindex = ifr.ifr_ifindex;

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

    char* dataLocation = reinterpret_cast<char*>(frame.get());
    int bytesToWrite = sizeof(can_frame);
    int ret=0;
    while(bytesToWrite>0 && m_connected){
        ret = write(m_socket, dataLocation + (sizeof(can_frame)-bytesToWrite), bytesToWrite);
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

    char* dataLocation = reinterpret_cast<char*>(frame.get());
    int bytesToRead = sizeof(can_frame);
    int bytes=0;
    while(bytesToRead>0 && m_connected){
        bytes = recv(m_socket, dataLocation + (sizeof(can_frame)-bytesToRead), bytesToRead, MSG_WAITALL);
        if (bytes<0)
        {
            m_connected=false;
            perror("ERROR: ");
            std::cerr << "ERROR: Socket receive error socket invalid" << std::endl;
            return false;
        }
        bytesToRead-=bytes;
    }
    return true;
};