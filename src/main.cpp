/* A simple SocketCAN example */

#include <thread>
#include <atomic>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

// https://docs.huihoo.com/doxygen/linux/kernel/3.7/can_8h_source.html

///////////////////////////////////////////////////////
// TODO: FOR LOOP CAUSEING SEG FAULT??
///////////////////////////////////////////////////////
void set_speed(int16_t target, struct can_frame *frame)
{
    int retval;
    unsigned char data[8];
    frame->can_id = 0x201;
    frame->can_dlc = 8;
    data[0] = 0x31;
    data[1] = target & 0xFF;
    data[2] = (target >> 8);
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    frame->data[0] = data[0];
    frame->data[1] = data[1];
    frame->data[2] = data[2];
    frame->data[3] = data[3];
    frame->data[4] = data[4];
    frame->data[5] = data[5];
    frame->data[6] = data[6];
    frame->data[7] = data[7];
    retval = write(soc, frame, sizeof(struct can_frame));
}

void stop_motor(struct can_frame *frame)
{
    int retval;
    unsigned char data[8];
    frame->can_id = 0x201;
    frame->can_dlc = 8;
    data[0] = 0x31;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    frame->data[0] = data[0];
    frame->data[1] = data[1];
    frame->data[2] = data[2];
    frame->data[3] = data[3];
    frame->data[4] = data[4];
    frame->data[5] = data[5];
    frame->data[6] = data[6];
    frame->data[7] = data[7];
    retval = write(soc, frame, sizeof(struct can_frame));
}

void clear_codes(struct can_frame *frame)
{
    int retval;
    unsigned char data[8];
    frame->can_id = 0x201;
    frame->can_dlc = 8;
    data[0] = 0x8E;
    data[1] = 0x54;
    data[2] = 0x41;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    frame->data[0] = data[0];
    frame->data[1] = data[1];
    frame->data[2] = data[2];
    frame->data[3] = data[3];
    frame->data[4] = data[4];
    frame->data[5] = data[5];
    frame->data[6] = data[6];
    frame->data[7] = data[7];
    retval = write(soc, frame, sizeof(struct can_frame));
}

void toggle_control(bool digital, struct can_frame *frame)
{
    int retval;
    unsigned char data[8];
    frame->can_id = 0x201;
    frame->can_dlc = 8;
    data[0] = 0x36;
    if (digital)
        data[2] = 0x00;
    else
        data[2] = 0x20;
    data[1] = 0x12;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    frame->data[0] = data[0];
    frame->data[1] = data[1];
    frame->data[2] = data[2];
    frame->data[3] = data[3];
    frame->data[4] = data[4];
    frame->data[5] = data[5];
    frame->data[6] = data[6];
    frame->data[7] = data[7];
    retval = write(soc, frame, sizeof(struct can_frame));
}

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