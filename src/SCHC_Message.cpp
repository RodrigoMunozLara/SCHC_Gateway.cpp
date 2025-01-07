#include "SCHC_Message.hpp"

SCHC_Message::SCHC_Message()
{
    _w = -1;
    _fcn = -1;
    _msg_type = -1;
    _c = -1;
    _schc_payload = "";
    _rcs = "";
}

int SCHC_Message::createRegularFragment(uint8_t ruleID, uint8_t dtag, uint8_t w, uint8_t fcn, char *payload, int payload_len, char *buffer)
{
    /* Mask definition */ 
    uint8_t w_mask = 0xC0;
    uint8_t fcn_mask = 0x3F;
    //byte c_mask = 0x20;

    /* SCHC header construction */
    uint8_t new_w = (w << 6) & w_mask;
    uint8_t new_fcn = (fcn & fcn_mask);
    uint8_t header = new_w | new_fcn;
    buffer[0] = header;

    /* SCHC payload construction */
    for(int i=0;i<payload_len;i++)
    {
        buffer[i+1] = payload[i];
    }

    return (payload_len + 1);
}

int SCHC_Message::createACKRequest(uint8_t ruleID, uint8_t dtag, uint8_t w, char *buffer)
{
    /* Mask definition */ 
    uint8_t w_mask = 0xC0;
    //byte fcn_mask = 0x3F;
    //byte c_mask = 0x20;

    /* SCHC header construction */
    uint8_t new_w = (w << 6) & w_mask;
    uint8_t new_fcn = 0x00;
    uint8_t header = new_w | new_fcn;
    buffer[0] = header;

    return 1;
}

int SCHC_Message::createSenderAbort(uint8_t ruleID, uint8_t dtag, uint8_t w, char *buffer)
{
        /* Mask definition */ 
    uint8_t w_mask = 0xC0;
    //byte fcn_mask = 0x3F;
    //byte c_mask = 0x20;

    /* SCHC header construction */
    uint8_t new_w = (w << 6) & w_mask;
    uint8_t new_fcn = 0x3F;
    uint8_t header = new_w | new_fcn;
    buffer[0] = header;

    return 1;
}

uint8_t SCHC_Message::decode_message(uint8_t protocol, int rule_id, char *msg, int len)
{
    if(protocol==SCHC_FRAG_PROTOCOL_LORAWAN)
    {
        // Mask definition
        uint8_t w_mask = 0xC0;
        uint8_t fcn_mask = 0x3F;
        //uint8_t c_mask = 0x20;

        uint8_t schc_header = msg[0];
        _w = w_mask & schc_header;
        _fcn = fcn_mask & schc_header;

        SPDLOG_INFO("Rule_id: {},  w header: {}, fcn header: {}", rule_id, _w, _fcn);

        if(rule_id==20 && len==1 && _fcn==0)
        {
            SPDLOG_INFO("Decoding SCHC ACK REQ message");
            _msg_type = SCHC_ACK_REQ_MSG;
            //TBD
        }
        else if(rule_id==20 && len==1 && _fcn==63)
        {
            SPDLOG_INFO("Decoding SCHC Sender-Abort message");
            _msg_type = SCHC_SENDER_ABORT_MSG;
            //TBD            
        }
        else if (rule_id==20 && len>1 && _fcn==63)
        {
            SPDLOG_INFO("Decoding All-1 SCHC message");
            _msg_type = SCHC_ALL1_FRAGMENT_MSG;
            //TBD 
        }
        else if (rule_id==20 && len>1)
        {
            SPDLOG_INFO("Decoding SCHC Regular message");
            _msg_type = SCHC_REGULAR_FRAGMENT_MSG;
            this->decode_schc_fragment(msg, len);
            //TBD 
        }   
    }

    return 0;
}

uint8_t SCHC_Message::decode_schc_fragment(char *msg, int len)
{
    int schc_payload_size = len - 1;    // largo del mensaje schc menos 1 byte del header
    std::string schc_payload(msg + 1, schc_payload_size);
    _schc_payload = schc_payload;
    return 0;
}

void SCHC_Message::printMsg(uint8_t protocol, uint8_t msgType, char *msg, int len)
{
    // if(protocol==SCHC_FRAG_PROTOCOL_LORAWAN)
    // {
    //     Serial.print("SCHC Header ---> ");
    //     printBin((uint8_t)msg[0]);
    //     Serial.println();

    //     Serial.print("SCHC Payload --> ");
    //     for(int i=1; i<len; i++)
    //     {
    //         Serial.print(msg[i]);
    //     }
    //     Serial.println();
    // }

    // |-----W=0, FCN=27----->| 4 tiles sent

    char* buff = new char[100];
    if(msgType==SCHC_REGULAR_FRAGMENT_MSG)
    {
        uint8_t w = (msg[0] & 0xC0) >> 6;
        uint8_t fcn = (msg[0] & 0x3F);
        sprintf(buff, "|-----W=");
        sprintf(buff,"%d",w);
        sprintf(buff, ", FCN=");
        sprintf(buff,"%d",fcn);
        if(fcn>9)
        {
            sprintf(buff, "----->| ");
        }
        else
        {
            sprintf(buff, " ----->| ");
        }

        int tile_size = 10;          // hardcoding warning - tile size = 10
        int n_tiles = (len-1)/tile_size;   
        if(n_tiles>9)
        {
            sprintf(buff, "%d", n_tiles);
            sprintf(buff, " tiles sent");
        }
        else
        {
            sprintf(buff, " %d", n_tiles);
            sprintf(buff, " tiles sent");
        }      
    }
    else if(msgType==SCHC_ACK_REQ_MSG)
    {
        uint8_t w = (msg[0] & 0xC0) >> 6;
        uint8_t fcn = (msg[0] & 0x3F);
        sprintf(buff, "|-----W=");
        sprintf(buff,"%d",w);
        sprintf(buff, ", FCN=");
        sprintf(buff,"%d",fcn);
        if(fcn>9)
        {
            sprintf(buff, "----->| ");
        }
        else
        {
            sprintf(buff, " ----->| ");
        }
    }
    else if(msgType==SCHC_SENDER_ABORT_MSG)
    {
        uint8_t w = (msg[0] & 0xC0) >> 6;
        uint8_t fcn = (msg[0] & 0x3F);
        sprintf(buff, "|-----W=");
        sprintf(buff,"%d",w);
        sprintf(buff, ", FCN=");
        sprintf(buff,"%d",fcn);
        if(fcn>9)
        {
            sprintf(buff,"----->| ");
        }
        else
        {
            sprintf(buff," ----->| ");
        }
    }
    SPDLOG_INFO("{}",buff);
    delete buff;
}

void SCHC_Message::printBin(uint8_t val)
{
    SPDLOG_INFO("{}", std::bitset<8>(val).to_string());

    // if(val<2)
    // {
    //     Serial.print("0000000");
    //     Serial.print(val,BIN);
    // }
    // else if (val<4)
    // {
    //     Serial.print("000000");
    //     Serial.print(val,BIN);
    // }
    // else if (val<8)
    // {
    //     Serial.print("00000");
    //     Serial.print(val,BIN);
    // }
    // else if (val<16)
    // {
    //     Serial.print("0000");
    //     Serial.print(val,BIN);
    // }
    // else if (val<32)
    // {
    //     Serial.print("000");
    //     Serial.print(val,BIN);
    // }
    // else if (val<64)
    // {
    //     Serial.print("00");
    //     Serial.print(val,BIN);
    // }
    // else if (val<128)
    // {
    //     Serial.print("0");
    //     Serial.print(val,BIN);
    // }
    // else if (val<256)
    // {
    //     Serial.print(val,BIN);
    // }
    
}

void SCHC_Message::print_string_in_hex(std::string str, int len)
{
    std::string hex_result;
    char buffer[3];                         // Almacena un byte hexadecimal como texto (2 dígitos + terminador nulo)
    for (size_t i = 0; i < str.size(); ++i) 
    {
        unsigned char c = str[i];           // Acceder al carácter por índice
        std::sprintf(buffer, "%02X", c);    // Convertir a hexadecimal en mayúsculas
        hex_result += buffer;               // Agregar al resultado
        hex_result += " ";                  // Espacio entre bytes
    }

    // Usar spdlog para imprimir el mensaje
    SPDLOG_INFO("{}", hex_result);
}

uint8_t SCHC_Message::get_w()
{
    return _w;
}

uint8_t SCHC_Message::get_fcn()
{
    return _fcn;
}

uint8_t SCHC_Message::get_msg_type()
{
    return _msg_type;
}

uint8_t SCHC_Message::get_c()
{
    return _c;
}

std::string SCHC_Message::get_schc_payload()
{
    return _schc_payload;
}

std::string SCHC_Message::get_rcs()
{
    return _rcs;
}
