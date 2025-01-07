#include "SCHC_Session_GW.hpp"
#include "SCHC_ThreadSafeQueue.hpp"

uint8_t SCHC_Session_GW::initialize(uint8_t protocol, uint8_t direction, uint8_t session_id, SCHC_Stack_L2 *stack_ptr)
{
    SPDLOG_TRACE("Entering the function");

    _session_id = session_id;

    _isUsed = false;        // at the beginning, the sessions are not being used
    _running = false;       // the flag allows to verify if the thread is running
    _is_first_msg = true;   // the flag allows to create the state machine only with the first message

    if(direction==SCHC_FRAG_DIRECTION_UPLINK && protocol==SCHC_FRAG_PROTOCOL_LORAWAN)
    {
        // SCHC session initialisation with LoRaWAN profile parameters (see RFC9011)
        _protocol = SCHC_FRAG_PROTOCOL_LORAWAN;
        _direction = SCHC_FRAG_DIRECTION_UPLINK;
        _ruleID = 20;
        _tileSize = 10;                         // tile size in bytes
        _m = 2;                                 // bits of the W field
        _n = 6;                                 // bits of the FCN field
        _windowSize = 63;                       // tiles in a SCHC window
        _t = 0;                                 // bits of the DTag field
        _maxAckReq = 8;                         // max number of ACK Request msg
        _retransTimer = 10;                     // Retransmission timer in seconds
        _inactivityTimer = 12*60*60;            // Inactivity timer in seconds
        _maxMsgSize = _tileSize*_windowSize*4;  // Maximum size of a SCHC packet in bytes
        _stack = stack_ptr;                     // Pointer to L2 stack
        _txAttemptsCounter = 0;                 // transmission attempt counter
    }
    else if(direction==SCHC_FRAG_DIRECTION_DOWNLINK && protocol==SCHC_FRAG_PROTOCOL_LORAWAN)
    {
        _protocol = SCHC_FRAG_PROTOCOL_LORAWAN;
        _direction = SCHC_FRAG_DIRECTION_DOWNLINK;
        _ruleID = 21;
        _tileSize = 0;                          // tile size in bytes
        _m = 1;                                 // bits of the W field
        _n = 1;                                 // bits of the FCN field
        _windowSize = 1;                        // tiles in a SCHC window
        _t = 0;                                 // bits of the DTag field
        _maxAckReq = 8;
        _retransTimer = 12*60*60;               // Retransmission timer in seconds
        _inactivityTimer = 12*60*60;            // Inactivity timer in seconds
        _maxMsgSize = _tileSize*_windowSize*2;  // Maximum size of a SCHC packet in bytes
        _stack = stack_ptr;                     // Pointer to L2 stack
    }

    SPDLOG_TRACE("Leaving the function");
    return 0;
}

void SCHC_Session_GW::start()
{
    if (!this->_running)
    {
        this->_running = true;
        _process_thread = std::thread(&SCHC_Session_GW::process_message, this);
        _process_thread.detach(); // Desatachar el hilo para que sea independiente
    }

}

uint8_t SCHC_Session_GW::queue_message(int rule_id, std::string msg, int len)
{
    SPDLOG_TRACE("Entering the function");
    _queue.push(rule_id, msg, len);
    SPDLOG_TRACE("Leaving the function");
    return 0;
}

void SCHC_Session_GW::process_message()
{
    SPDLOG_TRACE("Entering the function");

    SPDLOG_INFO("Starting threadID for sessionID: {}", _session_id);

    while (_running)
    {
        if(_queue.size() != 0)
        {
            SPDLOG_INFO("\033[31mExtracting message from the queue.\033[0m");
            SPDLOG_DEBUG("Current queue size: {}", _queue.size());
            std::string msg;
            int len;
            int rule_id;
            _queue.pop(rule_id, msg, len);

            if(_is_first_msg)
            {
                /* Creando e inicializando maquina de estado*/
                uint8_t res = this->createStateMachine();
                if(res==1)
                {
                    SPDLOG_ERROR("Unable to create state machine");
                }
                this->_is_first_msg = false;
            }


            /* Arrancando maquina de estado con el primer mensaje */
            char* buff = new char[len];
            strcpy(buff, msg.c_str());
            this->_stateMachine->start(rule_id, buff, len);
            delete[] buff;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    SPDLOG_TRACE("Leaving the function");
}

bool SCHC_Session_GW::getIsUsed()
{
    return this->_isUsed;
}

void SCHC_Session_GW::setIsUsed(bool isUsed)
{
    this->_isUsed = isUsed;
}

uint8_t SCHC_Session_GW::createStateMachine()
{
    SPDLOG_TRACE("Entering the function");

    if(_protocol==SCHC_FRAG_PROTOCOL_LORAWAN && _direction==SCHC_FRAG_DIRECTION_UPLINK)
    {
        _stateMachine = new SCHC_Ack_on_error();

        /* Inicializando maquina de estado */
        _stateMachine->init(_ruleID, -1, _windowSize, _tileSize, _n, ACK_MODE_ACK_END_WIN, _stack, _retransTimer, _maxAckReq);

        SPDLOG_INFO("State machine successfully created, initiated, and started");

        SPDLOG_TRACE("Leaving the function");
        return 0;
    }
    return 1;
}

uint8_t SCHC_Session_GW::destroyStateMachine()
{
    delete this->_stateMachine;
    SPDLOG_INFO("State machine successfully destroyed");
    return 0;
}
