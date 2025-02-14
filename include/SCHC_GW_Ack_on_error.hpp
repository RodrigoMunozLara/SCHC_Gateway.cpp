#ifndef SCHC_Ack_on_error_hpp
#define SCHC_Ack_on_error_hpp

#include "SCHC_GW_Macros.hpp"
#include "SCHC_GW_State_Machine.hpp"
#include "SCHC_GW_Message.hpp"
#include "SCHC_GW_ThreadSafeQueue.hpp"

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <math.h>
#include <cstdint>
#include <vector>
#include <map>
#include <thread>
#include <functional>

using namespace std;

class SCHC_GW_Ack_on_error: public SCHC_GW_State_Machine, public enable_shared_from_this<SCHC_GW_Ack_on_error>
{
    public:
        SCHC_GW_Ack_on_error();
        ~SCHC_GW_Ack_on_error();
        uint8_t                 init(string dev_id, uint8_t ruleID, uint8_t dTag, uint8_t windowSize, uint8_t tileSize, uint8_t n, uint8_t m, uint8_t ackMode, SCHC_GW_Stack_L2* stack_ptr, int retTimer, uint8_t ackReqAttempts) override;
        uint8_t                 execute_machine(int rule_id=0, char *msg=NULL, int len=0) override;
        uint8_t                 queue_message(int rule_id, char* msg, int len) override;
        void                    message_reception_loop() override;
        bool                    is_processing() override;
        void                    set_end_callback(function<void()> callback) override;
        void                    set_error_prob(uint8_t error_prob) override;
    private: 
        uint8_t                 RX_INIT_recv_fragments(int rule_id, char *msg, int len);
        uint8_t                 RX_RCV_WIN_recv_fragments(int rule_id, char *msg, int len);
        uint8_t                 RX_END_end_session(int rule_id = 0, char *msg=nullptr, int len=0);
        uint8_t                 RX_WAIT_x_MISSING_FRAGS_recv_fragments(int rule_id, char *msg, int len);
        string                  get_bitmap_array_str(uint8_t window);
        vector<uint8_t>         get_bitmap_array_vec(uint8_t window);
        uint8_t                 get_c_from_bitmap(uint8_t window);
        bool                    check_rcs(uint32_t rcs);
        uint32_t                calculate_crc32(const char *data, size_t length);
        int                     get_tile_ptr(uint8_t window, uint8_t fcn);
        int                     get_bitmap_ptr(uint8_t fcn);
        void                    print_tail_array_hex();
        void                    print_bitmap_array_str();
        static void             thread_entry_point(shared_ptr<SCHC_GW_Ack_on_error> instance);
        
        
        /* Static SCHC parameters */
        uint8_t         _ruleID;        // Rule ID -> https://www.rfc-editor.org/rfc/rfc9011.html#name-ruleid-management
        uint8_t         _dTag;          // not used in LoRaWAN
        uint8_t         _windowSize;    // in tiles. In LoRaWAN: 63
        int             _nMaxWindows;   // In LoRaWAN: 4
        uint8_t         _nTotalTiles;   // in tiles. In LoRaWAN: 252
        uint8_t         _lastTileSize;  // in bits
        uint8_t         _tileSize;      // in bytes. In LoRaWAN: 10 bytes
        uint8_t         _ackMode;       // Modes defined in SCHC_GW_Macros.hpp
        uint32_t        _retransTimer;
        uint8_t         _maxAckReq;
        string          _dev_id;
        char*           _last_tile;     // almacena el ultimo tile
        char**          _tilesArray;
        uint8_t**       _bitmapArray;
        int             _last_window;   // almacena el numero de la ultima ventana
        uint32_t        _rcs;
        uint8_t         _error_prob;


        /* Dynamic SCHC parameters */
        uint8_t         _currentState;
        int             _currentTile_ptr;
        uint8_t         _last_confirmed_window;

        /* Static LoRaWAN parameters*/
        int                 _current_L2_MTU;
        SCHC_GW_Stack_L2*   _stack;

        /* Thread and Queue Message*/
        SCHC_GW_ThreadSafeQueue    _queue;
        atomic<bool>       _processing;            // atomic flag for the thread
        string             _name;                  // thread name
        thread             _process_thread;        // thread
        function<void()>   _end_callback;

        /* Flags */
        bool                    _wait_pull_ack_req_flag;    // "true": si llega un ACK REQ lo considera un PULL ACK REQ (descarta el ACK REQ y no envía nada). "false": si llega un ACK REQ responde con un ACK.
        bool                    _first_ack_sent_flag;       // "true": si se envió el primer ACK para una ventana.
        
        /* Counters */
        int _counter;       // counter for select the messages that are dropped.
};

#endif