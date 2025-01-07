#ifndef SCHC_TTN_Stack_hpp
#define SCHC_TTN_Stack_hpp

#include "SCHC_Stack_L2.hpp"
#include <cstdint>
#include <spdlog/spdlog.h>

class SCHC_TTN_MQTT_Stack: public SCHC_Stack_L2
{
    public:
        uint8_t initialize_stack(void);
        uint8_t send_frame(uint8_t ruleID, char* msg, int len);
        int     getMtu(bool consider_Fopt);
        uint8_t set_mqtt_stack(mosquitto* mosqStack);
        uint8_t set_topic(char* send_topic);
    private:
        struct mosquitto*   _mosq;
        char*               _topic;
};


#endif