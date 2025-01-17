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
        char* schc_header   = new char[1];  // * Liberada en SCHC_Ack_on_error::RX_RCV_WIN_recv_fragments (linea 220 y 255) y 
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

        SPDLOG_DEBUG("Compress Bitmap size: {}", bits.size());
    }
    return 0;
}

uint8_t SCHC_Message::get_msg_type(uint8_t protocol, int rule_id, char *msg, int len)
{
    if(protocol==SCHC_FRAG_LORAWAN)
    {
        uint8_t schc_header = msg[0];
        uint8_t fcn_mask = 0x3F;                // Mask definition
        uint8_t _fcn = fcn_mask & schc_header;
        uint8_t _dtag = 0;                      // In LoRaWAN, dtag is not used

        if(rule_id==SCHC_FRAG_UPDIR_RULE_ID && len==1 && _fcn==0)
            _msg_type = SCHC_ACK_REQ_MSG;
        else if(rule_id==SCHC_FRAG_UPDIR_RULE_ID && len==1 && _fcn==63)
            _msg_type = SCHC_SENDER_ABORT_MSG;
        else if (rule_id==SCHC_FRAG_UPDIR_RULE_ID && len>1 && _fcn==63)
            _msg_type = SCHC_ALL1_FRAGMENT_MSG;
        else if (rule_id==SCHC_FRAG_UPDIR_RULE_ID && len>1)
            _msg_type = SCHC_REGULAR_FRAGMENT_MSG;
    }

    return _msg_type;

}

uint8_t SCHC_Message::decode_message(uint8_t protocol, int rule_id, char *msg, int len)
{
    if(protocol==SCHC_FRAG_LORAWAN)
    {
        // Mask definition
        uint8_t w_mask = 0xC0;
        uint8_t fcn_mask = 0x3F;
        uint8_t c_mask = 0x20;

        uint8_t schc_header = msg[0];
        _w      = (w_mask & schc_header) >> 6;
        _fcn    = fcn_mask & schc_header;

        SPDLOG_DEBUG("Rule_id: {},  w header: {}, fcn header: {}", rule_id, _w, _fcn);

        if(rule_id==SCHC_FRAG_UPDIR_RULE_ID && len==1 && _fcn==0)
        {
            SPDLOG_DEBUG("Decoding SCHC ACK REQ message");
            // TODO: implemtar la decodificacion de un SCHC ACK REQ
        }
        else if(rule_id==SCHC_FRAG_UPDIR_RULE_ID && len==1 && _fcn==63)
        {
            SPDLOG_DEBUG("Decoding SCHC Sender-Abort message");
            // TODO: implemtar la decodificacion de un SCHC Sender-Abort          
        }
        else if (rule_id==SCHC_FRAG_UPDIR_RULE_ID && len>1 && _fcn==63)
        {
            SPDLOG_DEBUG("Decoding All-1 SCHC message");

            // Crear el uint32_t a partir de los bytes
            _rcs = (static_cast<uint32_t>(msg[1] << 24)) & 0xFF000000 | (static_cast<uint32_t>(msg[2] << 16)) & 0x00FF0000 |
                   (static_cast<uint32_t>(msg[3] << 8)) & 0x0000FF00  | (static_cast<uint32_t>(msg[4])) & 0x000000FF;

            _schc_payload_len   = (len - 5)*8;                          // in bits
            _schc_payload       = new char[_schc_payload_len/8];        // * Liberada en linea 172. largo del mensaje menos 1 byte del header
            std::memcpy(_schc_payload, msg + 5, _schc_payload_len/8);
             
        }
        else if (rule_id==SCHC_FRAG_UPDIR_RULE_ID && len>1)
        {
            SPDLOG_DEBUG("Decoding SCHC Regular message");

            uint8_t schc_header = msg[0];
            _w      = (w_mask & schc_header) >> 6;
            _fcn    = fcn_mask & schc_header;
            _dtag   = 0;                                            // In LoRaWAN, dtag is not used

            _schc_payload_len   = (len - 1)*8;                      // in bits
            _schc_payload       = new char[_schc_payload_len/8];    // * Liberada en linea 172. Largo del mensaje menos 1 byte del header
            std::memcpy(_schc_payload, msg + 1, _schc_payload_len/8);
        }   
    }

    delete[] msg;   // * Libera la memoria solicitada en SCHC_TTN_Parser::base64_decode(), linea 110
    return 0;
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

uint32_t SCHC_Message::get_rcs()
{
    return _rcs;
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

void SCHC_Message::delete_schc_payload()
{
    delete[] _schc_payload;
}

void SCHC_Message::printMsg(uint8_t protocol, uint8_t msgType, char *msg, int len)
{
    char* buff = new char[100]; // * Liberada en linea 240
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
    SPDLOG_DEBUG("{}",buff);
    delete[] buff;
}