#pragma once
#include "CanCommunication.h"

class BMCCommunication: public CanCommunication
{
public:
    void SetSpeed(int16_t target);
    void StopMotor();
    void ClearCodes();
    void EnableDigitalControl();
    void EnableAnalogControl();
    std::shared_ptr<can_frame> BaseMsg(const uint8_t topic);
};