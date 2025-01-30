#include "SCHC_Session_GW.hpp"
#include "SCHC_Fragmenter_GW.hpp"

uint8_t SCHC_Session_GW::initialize(SCHC_Fragmenter_GW* frag, uint8_t protocol, uint8_t direction, uint8_t session_id, SCHC_Stack_L2 *stack_ptr, uint8_t ack_mode, uint8_t error_prob)
{
    SPDLOG_TRACE("Entering the function");

    _session_id = session_id;
    _frag       = frag;
    _ack_mode   = ack_mode;
    _error_prob = error_prob;

    set_running(false);         // at the beginning, the sessions are not being used
    set_is_first_msg(true);     // the flag allows to create the state machine only with the first message


    if(direction==SCHC_FRAG_UP && protocol==SCHC_FRAG_LORAWAN)
    {
        // SCHC session initialisation with LoRaWAN profile parameters (see RFC9011)
        _protocol = SCHC_FRAG_LORAWAN;
        _direction = SCHC_FRAG_UP;
        _tileSize = 10;                         // tile size in bytes
        _m = 2;                                 // bits of the W field
        _n = 6;                                 // bits of the FCN field
        _windowSize = 63;                       // tiles in a SCHC window
        _t = 0;                                 // bits of the DTag field
        _maxAckReq = 8;                         // max number of ACK Request msg
        _retransTimer = 12*60*60;               // Retransmission timer in seconds
        _inactivityTimer = 12*60*60;            // Inactivity timer in seconds
        _maxMsgSize = _tileSize*_windowSize*4;  // Maximum size of a SCHC packet in bytes
        _stack = stack_ptr;                     // Pointer to L2 stack
        _txAttemptsCounter = 0;                 // transmission attempt counter
    }
    else if(direction==SCHC_FRAG_DOWN && protocol==SCHC_FRAG_LORAWAN)
    {
        _protocol = SCHC_FRAG_LORAWAN;
        _direction = SCHC_FRAG_DOWN;
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

void SCHC_Session_GW::process_message(std::string dev_id, int rule_id, char* msg, int len)
{

    SPDLOG_TRACE("Entering the function.");

    if(_protocol==SCHC_FRAG_LORAWAN && _direction==SCHC_FRAG_UP)
    {
        if(is_first_msg())
        {
            SPDLOG_WARN("\033[34mReceiving first message from: {}\033[0m", dev_id);
            _dev_id     = dev_id;

            /* Creando e inicializando maquina de estado*/
            _stateMachine = std::make_shared<SCHC_Ack_on_error>();

            _stateMachine->set_end_callback(std::bind(&SCHC_Session_GW::destroyStateMachine, this));
            _stateMachine->set_error_prob(_error_prob);
            SPDLOG_DEBUG("State machine successfully created.");

            /* Inicializando maquina de estado */
            _stateMachine->init(dev_id, rule_id, 0, _windowSize, _tileSize, _n, _m, _ack_mode, _stack, _retransTimer, _maxAckReq);
            SPDLOG_DEBUG("State machine successfully initiated.");

            set_is_first_msg(false);
        }

        _stateMachine->queue_message(rule_id, msg, len);
        SPDLOG_DEBUG("Message successfully queue in the state machine.");
    }
    else if (_protocol==SCHC_FRAG_LORAWAN && _direction==SCHC_FRAG_DOWN)
    {
        if(is_first_msg())
        {
            /* Creando e inicializando maquina de estado*/
            // TODO: Instanciar un SCHC_ACK_Always()  

            /* Inicializando maquina de estado */
            _stateMachine->init(dev_id, rule_id, 0, _windowSize, _tileSize, _n, _m, ACK_MODE_ACK_END_WIN, _stack, _retransTimer, _maxAckReq);

            SPDLOG_DEBUG("State machine successfully created, initiated, and started");

            set_is_first_msg(false);
        }    

        _stateMachine->queue_message(rule_id, msg, len);    
    }
    
    SPDLOG_TRACE("Leaving the function");
}

bool SCHC_Session_GW::is_running()
{
    return _is_running.load();
}

void SCHC_Session_GW::set_running(bool status)
{
    _is_running.store(status);
}

bool SCHC_Session_GW::is_first_msg()
{
    return _is_first_msg.load();
}

void SCHC_Session_GW::set_is_first_msg(bool status)
{
    _is_first_msg.store(status);
}

void SCHC_Session_GW::destroyStateMachine()
{
    set_running(false);
    set_is_first_msg(true);
    SPDLOG_WARN("Blocking new message reception (is_running = false).");
    _stateMachine.reset();
    SPDLOG_WARN("State machine successfully destroyed");
    _frag->disassociate_session_id(_dev_id);
    SPDLOG_WARN("Session successfully disassociate");
    return;
}
