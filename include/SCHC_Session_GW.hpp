#ifndef SCHC_Session_GW_hpp
#define SCHC_Session_GW_hpp

#include "SCHC_Macros.hpp"
#include "SCHC_Ack_on_error.hpp"
#include "SCHC_State_Machine.hpp"
#include "SCHC_Stack_L2.hpp"
#include "SCHC_ThreadSafeQueue.hpp"
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <mosquitto.h>
#include <sstream>
#include <functional>

class SCHC_Fragmenter_GW;

class SCHC_Session_GW
{
    public:
        uint8_t initialize(SCHC_Fragmenter_GW* frag, uint8_t protocol, uint8_t direction, uint8_t session_id, SCHC_Stack_L2* stack_ptr, uint8_t ack_mode, uint8_t error_prob);
        void    process_message(std::string dev_id, int rule_id, char* msg, int len);
        bool    is_running();
        void    set_running(bool status);
        bool    is_first_msg();
        void    set_is_first_msg(bool status);
        void    destroyStateMachine();
    private:
        uint8_t                 _session_id;
        uint8_t                 _protocol;
        uint8_t                 _direction;
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
        //SCHC_State_Machine*     _stateMachine;
        std::shared_ptr<SCHC_State_Machine> _stateMachine;
        SCHC_Stack_L2*          _stack;
        SCHC_Fragmenter_GW*     _frag;
        std::string             _dev_id;
        uint8_t                 _ack_mode;
        uint8_t                 _error_prob;

        std::atomic<bool>       _is_running;            // controla si la sesion está siendo usada o puede ser destruida
        std::atomic<bool>       _is_first_msg;          // controla si la sesion ha recibido antes algun mensaje
};

#endif
