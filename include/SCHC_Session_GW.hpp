#ifndef SCHC_Session_GW_hpp
#define SCHC_Session_GW_hpp

#include "SCHC_Macros.hpp"
#include "SCHC_Ack_on_error.hpp"
#include "SCHC_State_Machine.hpp"
#include "SCHC_Stack_L2.hpp"
#include "SCHC_ThreadSafeQueue.hpp"
#include <spdlog/spdlog.h>
#include <mosquitto.h>
#include <thread>
#include <sstream>

class SCHC_Session_GW
{
    public:
        uint8_t initialize(uint8_t protocol, uint8_t direction, uint8_t session_id, SCHC_Stack_L2* stack_ptr);
        void    start();
        uint8_t queue_message(int rule_id, std::string msg, int len);
        void    process_message();
        bool    getIsUsed();
        void    setIsUsed(bool isUsed);
    private:
        uint8_t createStateMachine();
        uint8_t destroyStateMachine();
        bool _isUsed;
        uint8_t                 _session_id;
        uint8_t                 _protocol;
        uint8_t                 _direction;
        uint8_t                 _ruleID;
        //uint8_t                 _dTag;
        uint8_t                 _tileSize;              // tile size in bytes
        uint8_t                 _m;                     // bits of the W field
        uint8_t                 _n;                     // bits of the FCN field
        uint8_t                 _windowSize;            // tiles in a SCHC window
        uint8_t                 _t;                     // bits of the DTag field
        uint8_t                 _maxAckReq;             // max number of ACK Request msg
        int                     _retransTimer;          // Retransmission timer in seconds
        int                     _inactivityTimer;       // Inactivity timer in seconds
        uint8_t                 _txAttemptsCounter;     // transmission attempt counter
        uint8_t                 _rxAttemptsCounter;     // reception attempt counter
        int                     _maxMsgSize;            // Maximum size of a SCHC packet in bytes
        SCHC_State_Machine*     _stateMachine;
        SCHC_Stack_L2*          _stack;
        SCHC_ThreadSafeQueue    _queue;

        std::string             _name;                  // thread name
        std::atomic<bool>       _running;               // atomic flag for the htread
        std::thread             _process_thread;        // thread

        std::atomic<bool>       _is_first_msg;
};

#endif
