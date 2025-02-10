#include "SCHC_GW_Fragmenter.hpp"

uint8_t SCHC_GW_Fragmenter::set_mqtt_stack(mosquitto *mosqStack)
{
        _mosq = mosqStack;
        return 0;
}

uint8_t SCHC_GW_Fragmenter::initialize(uint8_t protocol, uint8_t ack_mode, uint8_t error_prob)
{
        SPDLOG_TRACE("Entering the function");
        _protocol = protocol;
        _error_prob = error_prob;

        if(protocol==SCHC_FRAG_LORAWAN)
        {
                SPDLOG_DEBUG("Initializing mqtt stack to connect to ttn-mqtt broker");

                _stack = nullptr;
                _stack = new SCHC_GW_TTN_MQTT_Stack();
                SCHC_GW_TTN_MQTT_Stack* stack_ttn_mqtt = dynamic_cast<SCHC_GW_TTN_MQTT_Stack*>(_stack); 
                stack_ttn_mqtt->set_mqtt_stack(_mosq);
                stack_ttn_mqtt->initialize_stack();

                /* initializing the session pool */

                SPDLOG_DEBUG("Initializing SCHC session pool with {} sessions",_SESSION_POOL_SIZE);

                for(uint8_t i=0; i<_SESSION_POOL_SIZE; i++)
                {
                _uplinkSessionPool[i].initialize(this,
                                                SCHC_FRAG_LORAWAN,
                                                SCHC_FRAG_UP,
                                                i,
                                                stack_ttn_mqtt,
                                                ack_mode,
                                                error_prob);
                _downlinkSessionPool[i].initialize(this,
                                                SCHC_FRAG_LORAWAN,
                                                SCHC_FRAG_DOWN,
                                                i,
                                                stack_ttn_mqtt,
                                                ack_mode,
                                                error_prob);
                
                }
        }

        SPDLOG_TRACE("Leaving the function");

        return 0;
}

uint8_t SCHC_GW_Fragmenter::listen_messages(char *buffer)
{
        SPDLOG_DEBUG(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
        SPDLOG_TRACE("\033[1mEntering the function\033[0m");

        SCHC_GW_TTN_Parser parser;
        parser.initialize_parser(buffer);
        SPDLOG_DEBUG("Receiving messages from: {}", parser.get_device_id());


        // Valida si existe una sesión asociada al deviceId.
        // Si no existe, solicita una sesion nueva.
        std::string device_id = parser.get_device_id();
        int id = this->get_session_id(device_id);
        if(id == -1)
        {
                /* No existen sesiones asociadas al device_id */

                /* Obteniendo una nueva sesion de uplink */
                id = this->get_free_session_id(SCHC_FRAG_UP);
                if(id == -1)
                {
                        return -1;
                }
                else
                {       
                        SPDLOG_DEBUG("Associating deviceid: {} with session id: {}", device_id, id);
                        this->associate_session_id(device_id, id);
                        this->_uplinkSessionPool[id].set_running(true);
                }
        }

        if(_uplinkSessionPool[id].is_running())
        {
                SPDLOG_DEBUG("Sending messages from {} to the session with id: {}", device_id, id);
                this->_uplinkSessionPool[id].process_message(device_id, parser.get_rule_id(), parser.get_decoded_payload(), parser.get_payload_len()); 
        }
        else
        {
                SPDLOG_ERROR("The session is not running. Discarting message");
        }

        SPDLOG_TRACE("\033[1mLeaving the function\033[0m");
        SPDLOG_DEBUG("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
        return 0;
}

int SCHC_GW_Fragmenter::get_free_session_id(uint8_t direction)
{
        if(_protocol==SCHC_FRAG_LORAWAN && direction==SCHC_FRAG_UP)
        {
                for(uint8_t i=0; i<_SESSION_POOL_SIZE;i++)
                {
                        if(!_uplinkSessionPool[i].is_running())
                        {
                                _uplinkSessionPool[i].set_running(true);
                                SPDLOG_TRACE("Selecting the session {}", i);
                                return i;
                        }
                }
                SPDLOG_ERROR("All sessiones are used");
        }
        return -1;
}

uint8_t SCHC_GW_Fragmenter::associate_session_id(std::string deviceId, int sessionId)
{
        auto result = _associationMap.insert({deviceId, sessionId});
        if (result.second)
        {
                SPDLOG_DEBUG("Key and value successfully inserted in the map.");
                return 0;
        } else
        {
                SPDLOG_ERROR("The key already exists in the map. Key: {}", deviceId);
                return -1;
        }
}

uint8_t SCHC_GW_Fragmenter::disassociate_session_id(std::string deviceId)
{
        std::this_thread::sleep_for(std::chrono::seconds(10));
        size_t res = _associationMap.erase(deviceId);
        if(res == 0)
        {
                SPDLOG_ERROR("Key not found. Could not disassociate. Key: {}", deviceId);
                return -1;
        }
        else if(res == 1)
        {
                SPDLOG_DEBUG("Key successfully disassociated. Key: {}", deviceId);
                return 0;
        }
        return -1;
}

int SCHC_GW_Fragmenter::get_session_id(std::string deviceId)
{
        auto it = _associationMap.find(deviceId);
        if (it != _associationMap.end())
        {
                SPDLOG_DEBUG("Recovering the session id: {} with Key: {}", it->second, deviceId);
                return it->second;
        }
        else
        {
                SPDLOG_DEBUG("Session does not exist for the Key: {}", deviceId);
                return -1;
        }
}
