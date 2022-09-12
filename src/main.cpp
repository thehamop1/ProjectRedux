/* A simple SocketCAN example */

#include "BMCCommunication.h"
#include <iostream>

enum ACTIONS{
ACC=1,
STOP=2,
CLEAR=3,
TOGGLE_DIGITAL=4,
TOGGLE_ANALOG=5,
QUIT=6
};

int Menu();

void Test(std::shared_ptr<can_frame> ptr){
    std::cout << "This is my test" << std::endl;
}

class MyClass{
    public:
    void AnotherTest(std::shared_ptr<can_frame> frame){
        std::cout << "Here's a test from a class" << std::endl;
    };
};

int main()
{

    bool alive=true;

    BMCCommunication motor;

    motor.StartThreads();

    //Example One: Class Function with lambda
    // MyClass newClass;
    // motor.AddCallback(0x123, [&](std::shared_ptr<can_frame> frame){newClass.AnotherTest(frame);});

    //Example Two: Class Function with std::bind
    // MyClass newClass;
    // motor.AddCallback(0x123, std::bind(&MyClass::AnotherTest, newClass, std::placeholders::_1));


    //Example Three: Function with std::Bind
    // motor.AddCallback(0x123, std::bind(Test, std::placeholders::_1));

    //Example Four: Lambda
    // motor.AddCallback(0x123, [](std::shared_ptr<can_frame> frame){
    //     std::cout << "I GOT MY FRAME THAT I EXPECTED" << std::endl;
    //     std::cout << "CAN ID: " << std::hex << frame->can_id << std::endl;
    // });

    while (alive)
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
            case QUIT:
            alive=false;
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
    std::cout << "6. Quit" << std::endl;
    std::cin >> ret;
    return ret;
}