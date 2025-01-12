#ifndef SCHC_Ack_on_error_hpp
#define SCHC_Ack_on_error_hpp

#include "SCHC_Macros.hpp"
#include "SCHC_State_Machine.hpp"
#include "SCHC_Message.hpp"

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <math.h>
#include <cstdint>
#include <vector>
#include <map>


class SCHC_Ack_on_error: public SCHC_State_Machine
{
    public:
        SCHC_Ack_on_error();
        ~SCHC_Ack_on_error();
        uint8_t init(uint8_t ruleID, uint8_t dTag, uint8_t windowSize, uint8_t tileSize, uint8_t n, uint8_t m, uint8_t ackMode, SCHC_Stack_L2* stack_ptr, int retTimer, uint8_t ackReqAttempts);
        uint8_t execute_machine(int rule_id=0, char *msg=NULL, int len=0);
        void                    set_device_id(std::string dev_id);
    private: 
        uint8_t                 RX_INIT_recv_fragments(int rule_id, char *msg, int len);
        uint8_t                 RX_RCV_WIN_recv_fragments(int rule_id, char *msg, int len);
        void                    print_tiles_array();
        uint8_t                 mtu_upgrade(int mtu);
        std::string             get_bitmap_array_str(uint8_t window);
        std::vector<uint8_t>    get_compress_bitmap(uint8_t window);
        uint8_t                 get_c(uint8_t window);
        

        /* Static SCHC parameters */
        uint8_t         _ruleID;        // Rule ID -> https://www.rfc-editor.org/rfc/rfc9011.html#name-ruleid-management
        uint8_t         _dTag;          // not used in LoRaWAN
        uint8_t         _windowSize;    // in tiles. In LoRaWAN: 63
        int             _nMaxWindows;   // In LoRaWAN: 4
        uint8_t         _nTotalTiles;   // in tiles. In LoRaWAN: 252
        uint8_t         _lastTileSize;  // in bits
        uint8_t         _tileSize;      // in bytes. In LoRaWAN: 10 bytes
        uint8_t         _ackMode;       // Modes defined in SCHC_Macros.hpp
        uint32_t        _retransTimer;
        uint8_t         _maxAckReq;
        std::string     _dev_id;

        /* Dynamic SCHC parameters */
        uint8_t         _currentState;
        uint8_t         _last_window_received;
        int             _last_fcn_received;
        std::map<uint8_t, int>                          _last_bitmap_ptr;   // key: w, value: last fcn
        std::map<std::pair<uint8_t, uint8_t>, char*>    _tiles_map;         // key.first: w, key.second: fcn
        std::map<std::pair<uint8_t, uint8_t>, int8_t>   _bitmap_map;        // key.first: w, key.second: fcn

        /* Static LoRaWAN parameters*/
        int             _current_L2_MTU;
        SCHC_Stack_L2*  _stack;

        /* Control Flags*/
        bool            _is_first_schc_fragment;
};

#endif