#ifndef SCHC_Fragmenter_GW_hpp
#define SCHC_Fragmenter_GW_hpp

#include "SCHC_Macros.hpp"
#include "SCHC_Stack_L2.hpp"
#include "SCHC_TTN_MQTT_Stack.hpp"
#include "SCHC_Session_GW.hpp"
#include <cstdint>
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <mosquitto.h>
#include "SCHC_ThreadSafeQueue.hpp"
#include "SCHC_TTN_Parser.hpp"


class SCHC_Fragmenter_GW
{
    public:
        uint8_t     set_mqtt_stack(mosquitto* mosqStack);
        uint8_t     initialize(uint8_t protocol);
        uint8_t     listen_messages(char *buffer);
        int         get_free_session_id(uint8_t direction);
        uint8_t     associate_session_id(std::string deviceId, int sessionId);
        uint8_t     disassociate_session_id(std::string deviceId);
        int         get_session_id(std::string deviceId);
    private:
        uint8_t                                 _protocol;
        SCHC_Session_GW                         _uplinkSessionPool[_SESSION_POOL_SIZE];
        SCHC_Session_GW                         _downlinkSessionPool[_SESSION_POOL_SIZE];
        SCHC_Stack_L2*                          _stack;
        std::unordered_map<std::string, int>    _associationMap;
        struct mosquitto*                       _mosq;
};
#endif  // SCHC_Fragmenter_GW_hpp