#ifndef SCHC_Message_hpp
#define SCHC_Message_hpp

#include "SCHC_Macros.hpp"
#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <spdlog/spdlog.h>
#include <bitset>
#include <thread>
#include <sstream>
#include <string>

class SCHC_Message
{
    public:
        SCHC_Message();
        int     createRegularFragment(uint8_t ruleID, uint8_t dtag, uint8_t w, uint8_t fcn, char *payload, int payload_len, char *buffer);
        int     createACKRequest(uint8_t ruleID, uint8_t dtag, uint8_t w, char *buffer);
        int     createSenderAbort(uint8_t ruleID, uint8_t dtag, uint8_t w, char *buffer);
        uint8_t decode_message(uint8_t protocol, int rule_id, char *msg, int len);
        uint8_t decode_schc_fragment(char *msg, int len);
        void    printMsg(uint8_t protocol, uint8_t msgType, char *msg, int len);
        void    printBin(uint8_t val);
        void    print_string_in_hex(std::string str, int len);
        uint8_t get_w();
        uint8_t get_fcn();
        uint8_t get_msg_type();
        uint8_t get_c();
        std::string get_schc_payload();
        std::string get_rcs();
    private:
        uint8_t     _w = -1;
        uint8_t     _fcn = -1;
        uint8_t     _msg_type = -1;
        uint8_t     _c = -1;
        std::string _schc_payload = "";
        std::string _rcs = "";
    
};

#endif