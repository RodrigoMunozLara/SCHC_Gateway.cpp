#include "SCHC_TTN_Parser.hpp"

int SCHC_TTN_Parser::initialize_parser(char *buffer)
{
    SPDLOG_TRACE("Entering the function");
    try{
        // Parsear el char* a un objeto JSON
        json parsed_json = json::parse(buffer);

        // Obtener y almacenar el valor de "device_id"
        if(parsed_json.contains("end_device_ids") && parsed_json["end_device_ids"].contains("device_id"))
        {
            _deviceId = parsed_json["end_device_ids"]["device_id"];

            SPDLOG_TRACE("DeviceID: {}", _deviceId);
        }
        else
        {
            SPDLOG_ERROR("The mqtt JSON message not include the \"end_device_ids\" key");
            return -1;        
        }

        // Obtener y almacenar el valor de frm_payload decodificado
        if(parsed_json.contains("uplink_message") && parsed_json["uplink_message"].contains("frm_payload"))
        {
            // save and print encoded buffer
            std::string frm_payload = parsed_json["uplink_message"]["frm_payload"];

            // save and print decoded buffer (text format)
            base64_decode(frm_payload, _decoded_payload, _len);
            SPDLOG_TRACE("Decoded Payload (hex format):");
            SCHC_Message::print_buffer_in_hex(_decoded_payload, _len);
            
            SPDLOG_TRACE("Length: {}", _len);
        }
        else
        {
            SPDLOG_WARN("The mqtt JSON message not include the \"uplink_message\" and \"frm_payload\" keys");
            return -1; 
        }


        // Obtener y almacenar el rule ID
        if(parsed_json.contains("uplink_message") && parsed_json["uplink_message"].contains("f_port"))
        {
            // save and print encoded buffer
            _rule_id = parsed_json["uplink_message"]["f_port"];
            SPDLOG_TRACE("Rule ID: {}", _rule_id);
        }
        else
        {
            SPDLOG_ERROR("The mqtt JSON message not include the \"uplink_message\" and \"f_port\" keys");
            return -1; 
        }
    }
    catch (const std::exception& e) {
        // Manejo de errores
        SPDLOG_ERROR("JSON error parsing: {}", e.what());
        return 1;
    }
    SPDLOG_TRACE("Leaving the function");

    return 0;
}

char* SCHC_TTN_Parser::get_decoded_payload()
{
    return _decoded_payload;
}

int SCHC_TTN_Parser::get_payload_len()
{
    return _len;
}

std::string SCHC_TTN_Parser::get_device_id()
{
    return _deviceId;
}

int SCHC_TTN_Parser::get_rule_id()
{
    return _rule_id;
}

void SCHC_TTN_Parser::delete_decoded_payload()
{
    delete[] _decoded_payload;
}

uint8_t SCHC_TTN_Parser::base64_decode(std::string encoded, char*& decoded_buffer, int& len)
{
    static const std::string base64_chars = 
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    std::string decoded;
    int val = 0, valb = -8;
    for (unsigned char c : encoded) {
        if (base64_chars.find(c) == std::string::npos) break;
        val = (val << 6) + base64_chars.find(c);
        valb += 6;
        if (valb >= 0) {
            decoded.push_back(char((val >> valb) & 0xFF));
            valb -= 8;
        }
    }
    len = decoded.size();
    // Esta memoria debe liberarse cuando ya no se use mas el mensaje
    char* buffer = new char[len];       // * Liberada en SCHC_Message::decode_message(), linea 122
    std::strcpy(buffer, decoded.c_str());
    decoded_buffer = buffer;   
    return 0;
}
