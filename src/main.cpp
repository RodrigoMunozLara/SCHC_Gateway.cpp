#include <mosquitto.h>
#include <iostream>
#include <cstring>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <string>
#include "SimpleIni.h"
#include "SCHC_Fragmenter_GW.hpp"

//Global variables
const char* topic_1_char;
const char *host;
SCHC_Fragmenter_GW frag;

// Callbacks declaration
void on_connect(struct mosquitto *mosq, void *obj, int rc);
void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg);


int main() {
    CSimpleIniA ini;
    ini.SetUnicode();
    ini.LoadFile("config.ini");

    // Cargar el archivo .ini
    SI_Error rc = ini.LoadFile("../config/config.ini");
    if (rc < 0) {
        std::cerr << "Error al cargar config.ini" << std::endl;
        return 1;
    }

    // Crear el logger para la consola
    auto console_logger = spdlog::stdout_color_mt("console");
    spdlog::set_default_logger(console_logger);

    // Establecer el patrón de salida
    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

    // Configurar el nivel de log
    std::string log_level = std::string(ini.GetValue("logging", "log_level", "Desconocido"));
    if(log_level.compare("TRACE") == 0)
    {
        spdlog::set_level(spdlog::level::trace);
        SPDLOG_CRITICAL("Using SPDLOG parameter - log level: TRACE");
    }
    else if(log_level.compare("DEBUG") == 0)
    {
        spdlog::set_level(spdlog::level::debug);
        SPDLOG_CRITICAL("Using SPDLOG parameter - log level: DEBUG");
    }
    else if(log_level.compare("INFO") == 0)
    {
        spdlog::set_level(spdlog::level::info);
        SPDLOG_CRITICAL("Using SPDLOG parameter - log level: INFO");
    }
    else if(log_level.compare("WARN") == 0)
    {
        spdlog::set_level(spdlog::level::warn);
        SPDLOG_CRITICAL("Using SPDLOG parameter - log level: WARN");
    }
    else if(log_level.compare("ERROR") == 0)
    {
        spdlog::set_level(spdlog::level::err);
        SPDLOG_CRITICAL("Using SPDLOG parameter - log level: ERROR");
    }
    else if(log_level.compare("CRITIAL") == 0)
    {
        spdlog::set_level(spdlog::level::critical);
        SPDLOG_CRITICAL("Using SPDLOG parameter - log level: CRITICAL");
    }
    else if(log_level.compare("OFF") == 0)
    {
        spdlog::set_level(spdlog::level::err);
        SPDLOG_CRITICAL("Using SPDLOG parameter - log level: OFF");
    }
    
    // MQTT parameters
    host                    = ini.GetValue("mqtt", "host", "Desconocido");
    const char *port_char   = ini.GetValue("mqtt", "port", "Desconocido");
    const int port          = std::stoi(port_char);
    const char *username    = ini.GetValue("mqtt", "username", "Desconocido");
    const char *password    = ini.GetValue("mqtt", "password", "Desconocido");
    const char *device_id_1 = ini.GetValue("lorawan", "deviceId_1", "Desconocido");
    std::string topic_1 = "v3/" + std::string(username) + "/devices/+/up";
    topic_1_char = topic_1.c_str(); // Conversión a char* (solo lectura)

    SPDLOG_CRITICAL("Using MQTT parameter - host: {}", host);
    SPDLOG_CRITICAL("Using MQTT parameter - port: {}", port);
    SPDLOG_CRITICAL("Using MQTT parameter - username: {}", username);
    SPDLOG_CRITICAL("Using MQTT parameter - device_id_1: {}", device_id_1);
    SPDLOG_CRITICAL("Using MQTT parameter - topic: {}", topic_1);

    // SCHC parameter
    const char* ack_mode_char   = ini.GetValue("schc", "schc_ack_mode", "Desconocido");
    const uint8_t ack_mode      = std::stoi(ack_mode_char);
    if(ack_mode == 1)
        SPDLOG_CRITICAL("Using SCHC parameter - ack_mode: ACK_MODE_ACK_END_WIN");
    else if(ack_mode == 2)
        SPDLOG_CRITICAL("Using SCHC parameter - ack_mode: ACK_MODE_ACK_END_SES");
    else if(ack_mode == 3)
        SPDLOG_CRITICAL("Using SCHC parameter - ack_mode: ACK_MODE_COMPOUND_ACK");



    const char* error_prob_char = ini.GetValue("schc", "error_prob", "Desconocido");
    const uint8_t error_prob    = std::stoi(error_prob_char);
    SPDLOG_CRITICAL("Using SCHC parameter - error_prob: {}", error_prob);

    mosquitto_lib_init();

    // Crear una instancia del cliente MQTT
    struct mosquitto *mosq = mosquitto_new(nullptr, true, nullptr);
    if (!mosq) {
        SPDLOG_ERROR("Failed to create mosquitto instance.");
        return 1;
    }

    // Configurar las credenciales de usuario
    if (mosquitto_username_pw_set(mosq, username, password) != MOSQ_ERR_SUCCESS) {
        SPDLOG_ERROR("Failed to set username and password.");
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    // Configurar los callbacks
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_message_callback_set(mosq, on_message);

    // Conectar al broker remoto
    if (mosquitto_connect(mosq, host, port, 60) != MOSQ_ERR_SUCCESS) {
        SPDLOG_ERROR("Could not connect to the broker.");
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    // Initialize a SCHC_Fragmenter_GW to process the uplink and downlink messages
    frag.set_mqtt_stack(mosq);
    frag.initialize(SCHC_FRAG_LORAWAN, ack_mode, error_prob);
    
    // Iniciar el bucle de la biblioteca para manejar mensajes
    mosquitto_loop_forever(mosq, 30000, 1);

    SPDLOG_CRITICAL("Disconnection of the mqtt broker");
    mosquitto_loop_stop(mosq, true);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    return 0;
}

void on_connect(struct mosquitto *mosq, void *obj, int rc) {
    if (rc == 0) {
        SPDLOG_CRITICAL("Connected to the {} broker successfully!", host);
        SPDLOG_CRITICAL("Waiting MQTT messages................");
        mosquitto_subscribe(mosq, nullptr, topic_1_char, 0);
    } else {
        SPDLOG_ERROR("Failed to connect, return code: {}", rc);
    }
}


void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg) {
    //SPDLOG_TRACE("Received message on topic: {} --> {}", msg->topic, static_cast<char*>(msg->payload));

    if (msg->payloadlen > 0)
    {
        frag.listen_messages(static_cast<char*>(msg->payload));
    }
    return;
}

