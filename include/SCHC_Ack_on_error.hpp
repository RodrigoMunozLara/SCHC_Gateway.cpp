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
#include <thread>
#include <sstream>


class SCHC_Ack_on_error: public SCHC_State_Machine
{
    public:
        SCHC_Ack_on_error();
        ~SCHC_Ack_on_error();
        virtual uint8_t init(uint8_t ruleID, uint8_t dTag, uint8_t windowSize, uint8_t tileSize, uint8_t n, uint8_t ackMode, SCHC_Stack_L2* stack_ptr, int retTimer, uint8_t ackReqAttempts);
        virtual uint8_t start(int rule_id, char *buffer, int len);
        virtual uint8_t execute_machine(int rule_id=0, char *msg=NULL, int len=0);
    private: 
        uint8_t RX_INIT_recv_fragments(int rule_id, char *msg, int len);
        uint8_t RX_RCV_WIN_recv_fragments(int rule_id, char *msg, int len);
        uint8_t mtuUpgrade(int mtu);
        uint8_t divideInTiles(char *buffer, int len);
        uint8_t extractTiles(uint8_t firstTileID, uint8_t nTiles, char *buff);
        uint8_t getCurrentWindow(int tile_ptr);
        uint8_t getCurrentFCN(int tile_ptr);
        uint8_t setBitmapArray(int tile_ptr, int tile_sent);
        uint8_t getCurrentBitmap_ptr(int tile_ptr);
        uint8_t createBitmapArray();
        void printCurrentBitmap(uint8_t nWindow);
        void printTileArray();
        void printBitmapArray();

        /* Static SCHC parameters */
        uint8_t _ruleID;
        uint8_t _dTag;
        uint8_t _windowSize;
        uint8_t _nMaxWindows;
        uint8_t _nWindows;
        uint8_t _nFullTiles;    // in tiles
        uint8_t _lastTileSize;  // in bytes
        uint8_t _tileSize;      // in bytes
        char **_tilesArray;
        char* _lastTile;
        uint8_t _ackMode;
        bool** _currentBitmapArray;
        uint32_t _retransTimer;
        uint8_t _maxAckReq;

        /* Dynamic SCHC parameters */
        uint8_t _currentState;
        uint8_t _currentWindow;
        uint8_t _currentFcn;
        uint8_t _currentBitmap_ptr;
        int _currentTile_ptr;
        
        /* Static LoRaWAN parameters*/
        int _current_L2_MTU;
        SCHC_Stack_L2* _stack;
};

#endif