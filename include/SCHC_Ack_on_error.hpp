#ifndef SCHC_Ack_on_error_hpp
#define SCHC_Ack_on_error_hpp

#include "SCHC_Macros.hpp"
#include "SCHC_State_Machine.hpp"
#include "SCHC_Message.hpp"
#include "SCHC_ThreadSafeQueue.hpp"

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <math.h>
#include <cstdint>
#include <vector>
#include <map>
#include <thread>
#include <functional>

class SCHC_Session_GW;

class SCHC_Ack_on_error: public SCHC_State_Machine, public std::enable_shared_from_this<SCHC_Ack_on_error>
{
    public:
        SCHC_Ack_on_error();
        ~SCHC_Ack_on_error();
        uint8_t                 init(std::string dev_id, uint8_t ruleID, uint8_t dTag, uint8_t windowSize, uint8_t tileSize, uint8_t n, uint8_t m, uint8_t ackMode, SCHC_Stack_L2* stack_ptr, int retTimer, uint8_t ackReqAttempts);
        uint8_t                 execute_machine(int rule_id=0, char *msg=NULL, int len=0);
        uint8_t                 queue_message(int rule_id, char* msg, int len);
        void                    message_reception_loop();
        bool                    is_processing();
        void                    set_end_callback(std::function<void()> callback);
        static void             thread_entry_point(std::shared_ptr<SCHC_Ack_on_error> instance);
        void                    set_error_prob(uint8_t error_prob);
    private: 
        uint8_t                 RX_INIT_recv_fragments(int rule_id, char *msg, int len);
        uint8_t                 RX_RCV_WIN_recv_fragments(int rule_id, char *msg, int len);
        uint8_t                 RX_END_end_session(int rule_id = 0, char *msg=nullptr, int len=0);
        uint8_t                 RX_WAIT_x_MISSING_FRAGS_recv_fragments(int rule_id, char *msg, int len);
        std::string             get_bitmap_array_str(uint8_t window);
        std::vector<uint8_t>    get_bitmap_array_vec(uint8_t window);
        uint8_t                 get_c_from_bitmap(uint8_t window);
        bool                    check_rcs(uint32_t rcs);
        uint32_t                calculate_crc32(const char *data, size_t length);
        int                     get_tile_ptr(uint8_t window, uint8_t fcn);
        int                     get_bitmap_ptr(uint8_t fcn);
        void                    print_tail_array_hex();
        void                    print_bitmap_array_str();
        

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
        char*           _last_tile;     // almacena el ultimo tile
        char**          _tilesArray;
        uint8_t**       _bitmapArray;
        int             _last_window;   // almacena el numero de la ultima ventana
        uint32_t        _rcs;
        uint8_t         _error_prob;


        /* Dynamic SCHC parameters */
        uint8_t         _currentState;
        uint8_t         _currentWindow;
        int             _currentTile_ptr;

        /* Static LoRaWAN parameters*/
        int                 _current_L2_MTU;
        SCHC_Stack_L2*      _stack;

        /* Thread and Queue Message*/
        SCHC_ThreadSafeQueue    _queue;
        std::atomic<bool>       _processing;               // atomic flag for the thread
        std::string             _name;                  // thread name
        std::thread             _process_thread;        // thread
        std::function<void()>   _end_callback;
};

#endif