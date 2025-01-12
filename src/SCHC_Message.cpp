#include "SCHC_Message.hpp"

SCHC_Message::SCHC_Message()
{
}

uint8_t SCHC_Message::create_schc_ack(uint8_t rule_id, uint8_t dtag, uint8_t w, uint8_t c, std::vector<uint8_t> bitmap_vector, char*& buffer, int& len)
{
    uint8_t w_mask      = 0xC0;
    uint8_t c_mask      = 0x20;

    if(c == 1)
    {
        // No hay errores, se agregan 5 bits de padding
        char* schc_header   = new char[1];
        schc_header[0]  = ((w << 6)& w_mask) | ((c << 5) & c_mask) | 0x00;
        buffer = schc_header;
        len = 1;
    }
    else
    {
        // hay errores, se deben calcular los bits de padding según:
        // https://www.rfc-editor.org/rfc/rfc8724.html#name-schc-ack-format


        int n_padding = 8 - ((bitmap_vector.size()+3)%8);

        for(int i=0; i<n_padding; i++)
        {
            bitmap_vector.push_back(0);
        } 

        // consturye los bits del SCHC packet (header + bitmap) como un vector
        std::vector<uint8_t> bits;



        // w está compuesto por 2 bits. Cada bit lo almacena en un uint8_t
        bits.insert(bits.begin(), w & 0b00000001); // 0b00000010
        bits.insert(bits.begin(), ((w & 0b00000010) >> 1));

        SPDLOG_INFO("Compress Bitmap size: {}", bits.size());
    }
    return 0;
}

uint8_t SCHC_Message::get_msg_type(uint8_t protocol, int rule_id, char *msg, int len)
{
    if(protocol==SCHC_FRAG_PROTOCOL_LORAWAN)
    {
        uint8_t schc_header = msg[0];
        uint8_t fcn_mask = 0x3F;                // Mask definition
        uint8_t _fcn = fcn_mask & schc_header;
        uint8_t _dtag = 0;                      // In LoRaWAN, dtag is not used

        if(rule_id==20 && len==1 && _fcn==0)
            _msg_type = SCHC_ACK_REQ_MSG;
        else if(rule_id==20 && len==1 && _fcn==63)
            _msg_type = SCHC_SENDER_ABORT_MSG;
        else if (rule_id==20 && len>1 && _fcn==63)
            _msg_type = SCHC_ALL1_FRAGMENT_MSG;
        else if (rule_id==20 && len>1)
            _msg_type = SCHC_REGULAR_FRAGMENT_MSG;
    }

    return _msg_type;

}

uint8_t SCHC_Message::decode_message(uint8_t protocol, int rule_id, char *msg, int len)
{
    if(protocol==SCHC_FRAG_PROTOCOL_LORAWAN)
    {
        // Mask definition
        uint8_t w_mask = 0xC0;
        uint8_t fcn_mask = 0x3F;
        uint8_t c_mask = 0x20;

        uint8_t schc_header = msg[0];
        _w = w_mask & schc_header;
        _fcn = fcn_mask & schc_header;
        _dtag = 0;  // In LoRaWAN, dtag is not used

        SPDLOG_INFO("Rule_id: {},  w header: {}, fcn header: {}", rule_id, _w, _fcn);

        if(rule_id==20 && len==1 && _fcn==0)
        {
            SPDLOG_INFO("Decoding SCHC ACK REQ message");
            //TBD
        }
        else if(rule_id==20 && len==1 && _fcn==63)
        {
            SPDLOG_INFO("Decoding SCHC Sender-Abort message");
            //TBD            
        }
        else if (rule_id==20 && len>1 && _fcn==63)
        {
            SPDLOG_INFO("Decoding All-1 SCHC message");
            //TBD 
        }
        else if (rule_id==20 && len>1)
        {
            SPDLOG_INFO("Decoding SCHC Regular message");
            
            _schc_payload_len   = (len - 1)*8;                          // in bits
            _schc_payload       = new char[_schc_payload_len/8];    // largo del mensaje menos 1 byte del header
            std::memcpy(_schc_payload, msg + 1, _schc_payload_len/8);
            
            return 0;
        }   
    }

    return{};
}

uint8_t SCHC_Message::get_w()
{
    return _w;
}

uint8_t SCHC_Message::get_fcn()
{
    return _fcn;
}

uint8_t SCHC_Message::get_dtag()
{
    return _dtag;
}

int SCHC_Message::get_schc_payload_len()
{
    return _schc_payload_len;
}

uint8_t SCHC_Message::get_schc_payload(char* schc_payload)
{
    memcpy(schc_payload, _schc_payload, _schc_payload_len/8);
    return 0;
}

uint8_t SCHC_Message::get_rcs(char*& rcs)
{
    rcs = _rcs;
    return 0;
}

uint8_t SCHC_Message::get_bitmap(char*& bitmap)
{
    bitmap = _bitmap;
    return 0;
}

void SCHC_Message::print_buffer_in_hex(char* buffer, int len)
{
    std::ostringstream oss;
    for (int i = 0; i < len; ++i) 
    {
        oss << fmt::format("{:02x} ", static_cast<unsigned char>(buffer[i]));
    }
    SPDLOG_DEBUG("{}", oss.str());
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