#ifndef SCHC_Message_hpp
#define SCHC_Message_hpp

#include "SCHC_Macros.hpp"
#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <sstream>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>


class SCHC_Message
{
    public:
        SCHC_Message();
        uint8_t     create_schc_ack(uint8_t rule_id, uint8_t dtag, uint8_t w, uint8_t c, std::vector<uint8_t> bitmap_vector, char*& buffer, int& len);
        uint8_t     get_msg_type(uint8_t protocol, int rule_id, char *msg, int len);
        uint8_t     decode_message(uint8_t protocol, int rule_id, char *msg, int len);
        uint8_t     get_w();
        uint8_t     get_fcn();
        uint8_t     get_dtag();
        int         get_schc_payload_len();
        uint8_t     get_schc_payload(char* schc_payload);
        uint32_t    get_rcs();
        uint8_t     get_bitmap(char*& bitmap);
        void        printMsg(uint8_t protocol, uint8_t msgType, char *msg, int len);
        static void print_buffer_in_hex(char* buffer, int len);
        void        delete_schc_payload();
    private:
        uint8_t     _msg_type;
        uint8_t     _w;
        uint8_t     _fcn;
        uint8_t     _dtag;
        int         _schc_payload_len;
        char*       _schc_payload = nullptr;
        uint32_t    _rcs;        
};

#endif