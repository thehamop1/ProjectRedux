#include "BMCCommunication.h"
#include <cstring>

void BMCCommunication::SetSpeed(int16_t target){
    target = target * (MAX_RPM / 100);

    auto frame = BaseMsg(SPEED_ID);
    uint8_t* data = frame->data;
    data[1] = target & 0xFF;
    data[2] = (target >> 8);
    CanCommunication::Send(frame);
};

void BMCCommunication::StopMotor(){
    CanCommunication::Send(BaseMsg(SPEED_ID));
};

void BMCCommunication::ClearCodes(){
    auto frame = BaseMsg(CLEAR_CODES_ID);
    uint8_t* data = frame->data;
    data[1] = LBIT_CLEAR_CODE;
    data[2] = RBIT_CLEAR_CODE;
    CanCommunication::Send(frame);
};

void BMCCommunication::EnableDigitalControl(){
    auto frame = BaseMsg(SPEED_MODE_ID);
    uint8_t* data = frame->data;
    data[1] = SPEED_MODE_ID_2;
    data[2] = DIGITAL_CONTROL;
    CanCommunication::Send(frame);
};

void BMCCommunication::EnableAnalogControl(){
    auto frame = BaseMsg(SPEED_MODE_ID);
    uint8_t* data = frame->data;
    data[1] = SPEED_MODE_ID_2;
    data[2] = ANALOG_CONTROL;
    CanCommunication::Send(frame);
};

std::shared_ptr<can_frame> BaseMsg(const uint8_t topic) {
    auto frame = std::make_shared<can_frame>();
    std::memset(frame.get(), 0, sizeof(can_frame));
    frame->can_id = BMC_ADDR;
    frame->can_dlc = DEFAULT_DLC_SIZE;
    return frame;
};