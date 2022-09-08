#include "Socket.h"

#include <linux/can.h>
#include <linux/can/raw.h>

bool Socket::open_port(std::string_view port)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    /* open socket */
    m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket < 0)
    {
        std::cerr << "ERROR: Could not create socket!" << std::endl;
        return false;
    }

    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, port);

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

    return true;
};