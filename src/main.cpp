/* A simple SocketCAN example */

#include "BMCCommunication.h"
#include <iostream>

enum ACTIONS{
ACC=1,
STOP=2,
CLEAR=3,
TOGGLE_DIGITAL=4,
TOGGLE_ANALOG=5
};

int Menu();

int main()
{
    BMCCommunication motor;

    motor.StartThreads();

    while (true)
    {
        switch(Menu()){
            case ACC:
            {
                int speed=0;
                system("clear");
                std::cout << "How fast? " << std::endl;
                std::cin >> speed;
                motor.SetSpeed(speed);
            }
            break; 
            case STOP:
            motor.StopMotor();
            break;
            case CLEAR:
            motor.ClearCodes();
            break;
            case TOGGLE_DIGITAL:
            motor.EnableDigitalControl();
            break;
            case TOGGLE_ANALOG:
            motor.EnableAnalogControl();
            break;
            default:
            std::cerr << "WARNING: Unexpected action to motor!";
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

int Menu(){
    int ret = 0;
    system("clear");
    std::cout << "==== Main Menu ====" << std::endl;
    std::cout << "1. Set Speed " << std::endl;
    std::cout << "2. Stop Motor" << std::endl;
    std::cout << "3. Clear Codes" << std::endl;
    std::cout << "4. Enable Digital Control" << std::endl;
    std::cout << "5. Enable Analog Control" << std::endl;
    std::cin >> ret;
    return ret;
}