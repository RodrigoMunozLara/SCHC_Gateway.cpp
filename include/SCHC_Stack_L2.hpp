#ifndef SCHC_Stack_L2_hpp
#define SCHC_Stack_L2_hpp

#include <cstdint>
#include <mosquitto.h>

class SCHC_Stack_L2
{
public:
    virtual uint8_t initialize_stack(void) = 0;
    virtual uint8_t send_frame(uint8_t ruleID, char* msg, int len) = 0;
    virtual int getMtu(bool consider_Fopt) = 0;
};

#endif