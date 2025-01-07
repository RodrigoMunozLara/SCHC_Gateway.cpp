#include "SCHC_TTN_MQTT_Stack.hpp"

uint8_t SCHC_TTN_MQTT_Stack::initialize_stack(void)
{
    //TBD
    return 0;
}

uint8_t SCHC_TTN_MQTT_Stack::send_frame(uint8_t ruleID, char *msg, int len)
{
    int result = mosquitto_publish(_mosq, nullptr, _topic, strlen(msg), msg, 0, false);
    if (result != MOSQ_ERR_SUCCESS) {
        SPDLOG_ERROR("The message could not be published. Code: {}" , result);
        return 1;
    }
    return 0;
}

int SCHC_TTN_MQTT_Stack::getMtu(bool consider_Fopt)
{
    //TBD: obtener el MTU para enviar mensajes a TTN por MQTT. 
    //TBD: ¿Que pasa cuando el mensaje es mas grande que el MTU soportado por el DR en el downlink?
    return 0;
}

uint8_t SCHC_TTN_MQTT_Stack::set_mqtt_stack(mosquitto *mosqStack)
{
    this->_mosq = mosqStack;
    return 0;
}

uint8_t SCHC_TTN_MQTT_Stack::set_topic(char *send_topic)
{
    this->_topic = send_topic;
    return 0;
}
