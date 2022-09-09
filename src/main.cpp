/* A simple SocketCAN example */

#include <thread>
#include <atomic>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

// https://docs.huihoo.com/doxygen/linux/kernel/3.7/can_8h_source.html

void send_port(const fsae_electric_vehicle::can_message &msg)
{
    struct can_frame *frame;
    int retval;
    unsigned char data[8];
    int16_t target = 4100;
    float percentage = (float)msg.speed;
    if (percentage == 0)
    {
        target = 0;
    }
    else
    {
        target = target * (percentage / 100);
    }
    std::cout << target << std::endl;
    std::cout << msg.data << std::endl;

    if (msg.data == "accelerate")
    {
        set_speed(target, frame);
    }
    else if (msg.data == "stop")
    {
        stop_motor(frame);
    }
    else if (msg.data == "clear")
    {
        clear_codes(frame);
    }
    else if (msg.data == "toggle_digital")
    {
        toggle_control(true, frame);
    }
    else if (msg.data == "toggle_analog")
    {
        toggle_control(false, frame);
    }
}

int main(int argc, char **argv)
{
    //   ros::init(argc, argv, "can_bus");
    //   ros::NodeHandle n;
    //   ros::Publisher CAN_BUS = n.advertise<fsae_electric_vehicle::can_message>("can_bus", 1000);
    //   ros::Subscriber CAN_BUS_COMMANDS = n.subscribe("can_bus_commands", 1000, send_port);
    //   ros::Rate loop_rate(1000);

    if (!open_port("can0"))
    {
        std::cerr << "ERROR: Could not properly setup socket!" << std::endl;
        std::exit(0);
    };

    while (!die)
    {
        can_message = read_port(can_message);
        CAN_BUS.publish(can_message);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}