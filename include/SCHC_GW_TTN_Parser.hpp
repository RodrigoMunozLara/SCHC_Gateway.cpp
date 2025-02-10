#ifndef SCHC_GW_TTN_Parser_hpp
#define SCHC_GW_TTN_Parser_hpp

#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <iostream>
#include "SCHC_GW_Message.hpp"

using json = nlohmann::json;

class SCHC_GW_TTN_Parser
{
    public:
        int         initialize_parser(char *buffer);
        char*       get_decoded_payload();
        int         get_payload_len();
        std::string get_device_id();
        int         get_rule_id();  
        void        delete_decoded_payload();
    private:
        uint8_t     base64_decode(std::string encoded, char*& decoded_buffer, int& len);
        char*       _decoded_payload = nullptr;    // frm_payload decoded
        int         _len;                       // frmPayload length
        std::string _deviceId;                  // LoRaWAN deviceID
        int         _rule_id;
};

#endif