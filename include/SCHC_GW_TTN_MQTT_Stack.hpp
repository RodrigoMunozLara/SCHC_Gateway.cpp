#ifndef SCHC_TTN_Stack_hpp
#define SCHC_TTN_Stack_hpp

#include "SCHC_GW_Stack_L2.hpp"
#include <cstdint>
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "SimpleIni.h"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

class SCHC_GW_TTN_MQTT_Stack: public SCHC_GW_Stack_L2
{
    public:
        uint8_t     initialize_stack(void);
        uint8_t     send_downlink_frame(std::string dev_id, uint8_t ruleID, char* msg, int len);
        int         getMtu(bool consider_Fopt);
        uint8_t     set_mqtt_stack(mosquitto* mosqStack);
        void        set_application_id(std::string app);
        void        set_tenant_id(std::string tenant);
        std::string base64_encode(const char* buffer, int len);
    private:
        struct mosquitto*   _mosq;
        std::string         _application_id;
        std::string         _tenant_id;
        const char*         _mqqt_username;
};


#endif