#ifndef SCHC_TTN_Parser_hpp
#define SCHC_TTN_Parser_hpp

#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>
#include <spdlog/spdlog.h>
#include <iostream>

using json = nlohmann::json;

class SCHC_TTN_Parser
{
    public:
        int     initialize_parser(char *buffer);
        std::string get_decoded_payload();
        int         get_payload_len();
        std::string get_device_id();
        int         get_rule_id();  
    private:
        std::string base64_decode(const std::string& encoded);
        std::string _decoded_payload;   // frm_payload decoded
        int         _len;               // frmPayload length
        std::string _deviceId;          // LoRaWAN deviceID
        int         _rule_id;
};


#endif