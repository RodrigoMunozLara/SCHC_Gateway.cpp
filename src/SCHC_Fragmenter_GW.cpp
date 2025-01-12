#include "SCHC_Fragmenter_GW.hpp"

uint8_t SCHC_Fragmenter_GW::set_mqtt_stack(mosquitto *mosqStack)
{
        _mosq = mosqStack;
        return 0;
}

uint8_t SCHC_Fragmenter_GW::initialize(uint8_t protocol)
{
        SPDLOG_TRACE("Entering the function");
        _protocol = protocol;

        if(protocol==SCHC_FRAG_PROTOCOL_LORAWAN)
        {
                SPDLOG_INFO("Initializing mqtt stack to connect to ttn-mqtt broker");

                _stack = nullptr;
                _stack = new SCHC_TTN_MQTT_Stack();
                SCHC_TTN_MQTT_Stack* stack_ttn_mqtt = dynamic_cast<SCHC_TTN_MQTT_Stack*>(_stack); 
                stack_ttn_mqtt->set_mqtt_stack(_mosq);
                stack_ttn_mqtt->initialize_stack();

                /* initializing the session pool */

                SPDLOG_INFO("Initializing SCHC session pool with {} sessions",_SESSION_POOL_SIZE);

                for(uint8_t i=0; i<_SESSION_POOL_SIZE; i++)
                {
                _uplinkSessionPool[i].initialize(SCHC_FRAG_PROTOCOL_LORAWAN,
                                                SCHC_FRAG_DIRECTION_UPLINK,
                                                i,
                                                stack_ttn_mqtt);
                _downlinkSessionPool[i].initialize(SCHC_FRAG_PROTOCOL_LORAWAN,
                                                SCHC_FRAG_DIRECTION_DOWNLINK,
                                                i,
                                                stack_ttn_mqtt);
                
                }
        }


        SPDLOG_TRACE("Leaving the function");

        return 0;
}

uint8_t SCHC_Fragmenter_GW::listen_messages(char *buffer)
{
        SPDLOG_TRACE("Entering the function");

        SCHC_TTN_Parser parser;
        parser.initialize_parser(buffer);
        SPDLOG_INFO("\033[1mReceiving messages from: {}\033[0m", parser.get_device_id());

        // Valida si existe una sesión asociada al deviceId.
        // Si no existe, solicita una sesion nueva.
        std::string device_id = parser.get_device_id();
        int id = this->get_session_id(device_id);
        if(id == -1)
        {
                id = this->get_free_session_id(SCHC_FRAG_DIRECTION_UPLINK);
                if(id == -1)
                {
                        return -1;
                }
                else
                {       
                        SPDLOG_INFO("Associating deviceid: {} with session id: {}", device_id, id);
                        this->associate_session_id(device_id, id);
                        this->_uplinkSessionPool[id].start(); 
                }
        }
        else
        {
                SPDLOG_INFO("Obtaining session id: {} to deviceid: {}", id, device_id);
        }

        SPDLOG_DEBUG("Queuing messages in the session with id: {}", id);
        this->_uplinkSessionPool[id].queue_message(device_id, parser.get_rule_id(), parser.get_decoded_payload(), parser.get_payload_len());

        SPDLOG_TRACE("Leaving the function");
        return 0;
}

int SCHC_Fragmenter_GW::get_free_session_id(uint8_t direction)
{
        SPDLOG_TRACE("Entering the function");

        if(_protocol==SCHC_FRAG_PROTOCOL_LORAWAN && direction==SCHC_FRAG_DIRECTION_UPLINK)
        {
                for(uint8_t i=0; i<_SESSION_POOL_SIZE;i++)
                {
                        if(!_uplinkSessionPool[i].getIsUsed())
                        {
                                SPDLOG_TRACE("Leaving the function");
                                return i;
                        }
                }
                SPDLOG_ERROR("All sessiones are used");
        }

        SPDLOG_TRACE("Leaving the function");

        return -1;
}

uint8_t SCHC_Fragmenter_GW::associate_session_id(std::string deviceId, int sessionId)
{
        auto result = _associationMap.insert({deviceId, sessionId});
        if (result.second)
        {
                SPDLOG_DEBUG("Key and value successfully inserted in the map.");
                return 0;
        } else
        {
                SPDLOG_ERROR("The key already existsin the map. Key: {}", deviceId);
                return -1;
        }
}

uint8_t SCHC_Fragmenter_GW::disassociate_session_id(std::string deviceId)
{
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

int SCHC_Fragmenter_GW::get_session_id(std::string deviceId)
{
        auto it = _associationMap.find(deviceId);
        if (it != _associationMap.end())
        {
                SPDLOG_INFO("Recovering the session id: {} with Key: {}", it->second, deviceId);
                return it->second;
        }
        else
        {
                SPDLOG_INFO("Session does not exist for the Key: {}", deviceId);
                return -1;
        }
}
