#include "SCHC_TTN_MQTT_Stack.hpp"

uint8_t SCHC_TTN_MQTT_Stack::initialize_stack(void)
{
    CSimpleIniA ini;
    ini.SetUnicode();
    ini.LoadFile("config.ini");

    // Cargar el archivo .ini
    SI_Error rc = ini.LoadFile("../config/config.ini");
    if (rc < 0) {
        SPDLOG_ERROR("Error loading config.ini file");
        return 1;
    }

    _mqqt_username = ini.GetValue("mqtt", "username", "Desconocido");


    return 0;
}

uint8_t SCHC_TTN_MQTT_Stack::send_downlink_frame(std::string dev_id, uint8_t ruleID, char *msg, int len)
{
    std::string topic = "v3/" + std::string(_mqqt_username) + "/devices/" + dev_id + "/down/push";

    // Variables con valores dinámicos
    int         f_port_value        = ruleID;
    std::string frm_payload_value   = this->base64_encode(msg, len);
    std::string priority_value      = "NORMAL";

    // Crear un objeto JSON con valores provenientes de variables
    json json_object = {
        {"downlinks", {
            {
                {"f_port", f_port_value},
                {"frm_payload", frm_payload_value},
                {"priority", priority_value}
            }
        }}
    };

    // Convertir el objeto JSON a una cadena (string) y luego a char*
    std::string json_string = json_object.dump();
    const char* json_c_str = json_string.c_str();

    SPDLOG_DEBUG("Downlink topic: {}", topic);
    SPDLOG_DEBUG("Downlink JSON: {}", json_string);

    int result = mosquitto_loop(_mosq, -1, 1);
    if (result != MOSQ_ERR_SUCCESS) {
        SPDLOG_ERROR("Connection lost with mqtt broker: {}", mosquitto_strerror(result));
        mosquitto_disconnect(_mosq);
        mosquitto_destroy(_mosq);
        return EXIT_FAILURE;
    }
    else
    {
        SPDLOG_DEBUG("Connection with mqtt broker.... OK");
    }


    result = mosquitto_publish(_mosq, nullptr, topic.c_str(), strlen(json_c_str), json_c_str, 0, false);
    if (result != MOSQ_ERR_SUCCESS) {
        SPDLOG_ERROR("The message could not be published. Code: {}" , mosquitto_strerror(result));
        return 1;
    }
    else
    {
        SPDLOG_DEBUG("Message sent successfully");
    }
    return 0;
}

int SCHC_TTN_MQTT_Stack::getMtu(bool consider_Fopt)
{
    // TODO: obtener el MTU para enviar mensajes a TTN por MQTT. 
    // ? ¿Que pasa cuando el mensaje es mas grande que el MTU soportado por el DR en el downlink?
    return 0;
}

uint8_t SCHC_TTN_MQTT_Stack::set_mqtt_stack(mosquitto *mosqStack)
{
    this->_mosq = mosqStack;
    return 0;
}

void SCHC_TTN_MQTT_Stack::set_application_id(std::string app)
{
    _application_id = app;
}

void SCHC_TTN_MQTT_Stack::set_tenant_id(std::string tenant)
{
    _tenant_id = tenant;
}

std::string SCHC_TTN_MQTT_Stack::base64_encode(const char* buffer, int len) {
    static const std::string base64_chars = 
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";

    std::string encoded;
    int val = 0, valb = -6;

    for (int i = 0; i < len; ++i) {
        val = (val << 8) + (unsigned char)buffer[i];
        valb += 8;
        while (valb >= 0) {
            encoded.push_back(base64_chars[(val >> valb) & 0x3F]);
            valb -= 6;
        }
    }

    if (valb > -6) {
        encoded.push_back(base64_chars[((val << 8) >> (valb + 8)) & 0x3F]);
    }

    while (encoded.size() % 4) {
        encoded.push_back('=');
    }

    return encoded;
}