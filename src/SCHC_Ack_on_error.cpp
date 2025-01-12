#include "SCHC_Ack_on_error.hpp"

SCHC_Ack_on_error::SCHC_Ack_on_error()
{
}

SCHC_Ack_on_error::~SCHC_Ack_on_error()
{
}

uint8_t SCHC_Ack_on_error::init(uint8_t ruleID, uint8_t dTag, uint8_t windowSize, uint8_t tileSize, uint8_t n, uint8_t m, uint8_t ackMode, SCHC_Stack_L2 *stack_ptr, int retTimer, uint8_t ackReqAttempts)
{
    SPDLOG_TRACE("Entering the function");

    /* Static SCHC parameters */
    _ruleID = ruleID;                       // Rule ID -> https://www.rfc-editor.org/rfc/rfc9011.html#name-ruleid-management
    _dTag = dTag;                           // not used in LoRaWAN
    _windowSize = windowSize;               // in tiles. In LoRaWAN: 63
    _nMaxWindows = pow(2,m);                // In LoRaWAN: 4
    _nTotalTiles = windowSize * pow(2,m);   // in tiles. In LoRaWAN: 252
    _lastTileSize = 0;                      // in bits
    _tileSize = tileSize;                   // in bytes. In LoRaWAN: 10 bytes
    _ackMode = ackMode;                     // Modes defined in SCHC_Macros.hpp
    _retransTimer = retTimer;               // in minutes. In LoRaWAN: 12*60*60 minutes
    _maxAckReq = ackReqAttempts;            // in minutes. In LoRaWAN: 12*60*60 minutes

    /* Static LoRaWAN parameters*/
    _current_L2_MTU = stack_ptr->getMtu(true);
    _stack = stack_ptr;

    _currentState = STATE_RX_INIT;

    /* Reserving memory for the Tails Array*/
    for(int i=0; i<_nMaxWindows; i++)
    {
        for(int j=(_windowSize-1); j>=0; j--)
        {
            _tiles_map[{i,j}] = new char[_tileSize];
        }
    }

    /* Reserving memory for the Bitmap Array*/
    for(int i=0; i<_nMaxWindows; i++)
    {
        for(int j=(_windowSize-1); j>=0; j--)
        {
            _bitmap_map[{i,j}] = 0;
        }
    }

    SPDLOG_TRACE("Leaving the function");

    return 0;
}

uint8_t SCHC_Ack_on_error::execute_machine(int rule_id, char *msg, int len)
{
    SPDLOG_TRACE("Entering the function");

    if(_currentState==STATE_RX_INIT)
    {
        SPDLOG_INFO("Calling to RX_INIT_recv_fragments() method");
        this->RX_INIT_recv_fragments(rule_id, msg, len);
    }
    else if (_currentState==STATE_RX_RCV_WINDOW)
    {
        SPDLOG_INFO("Calling to RX_RCV_WIN_recv_fragments() method");
        this->RX_RCV_WIN_recv_fragments(rule_id, msg, len);
    }
    else
    {
        SPDLOG_ERROR("State not defined");
    }

    SPDLOG_TRACE("Leaving the function");

    return 0;
}

void SCHC_Ack_on_error::set_device_id(std::string dev_id)
{
    _dev_id = dev_id;
}

uint8_t SCHC_Ack_on_error::RX_INIT_recv_fragments(int rule_id, char *msg, int len)
{
    SPDLOG_INFO("Changing STATE: From STATE_RX_INIT --> STATE_RX_RCV_WINDOW");
    this->_currentState = STATE_RX_RCV_WINDOW;

    _last_window_received = 0;
    _last_fcn_received = (_windowSize)-1;

    this->execute_machine(rule_id, msg, len);
    return 0;
}

uint8_t SCHC_Ack_on_error::RX_RCV_WIN_recv_fragments(int rule_id, char *msg, int len)
{
    SCHC_Message decoder;
    uint8_t msg_type;
    uint8_t w;              // w recibido en el mensaje
    uint8_t dtag;           // dtag recibido en el mensajes
    uint8_t fcn;            // fcn recibido en el mensaje
    char*   payload;
    int     payload_len;    // in bits
    char*   rcs;

    msg_type = decoder.get_msg_type(SCHC_FRAG_PROTOCOL_LORAWAN, rule_id, msg, len);

    if(msg_type == SCHC_REGULAR_FRAGMENT_MSG)
    {
        SPDLOG_INFO("Receiving a SCHC Regular fragment");
        decoder.decode_message(SCHC_FRAG_PROTOCOL_LORAWAN, rule_id, msg, len);

        payload_len     = decoder.get_schc_payload_len();   // largo del payload SCHC. En bits
        fcn             = decoder.get_fcn();
        w               = decoder.get_w();
        char*   payload = new char[payload_len/8];          // buffer para recibir el SCHC payload (no olvidar liberar)
        decoder.get_schc_payload(payload);                  // obtiene el SCHC payload


        //Obteniendo la cantidad de tiles en el mensaje
        int tiles_in_payload = (payload_len/8)/_tileSize;

        for(int i=0; i<tiles_in_payload; i++)
        {
            char* tile_store = _tiles_map[{w, fcn-i}];
            memcpy(tile_store, payload + (i*_tileSize), _tileSize);
            _bitmap_map[{w, fcn-i}] = 1;
        }

        /* almacena el numero del ultimo fcn recibido*/
        _last_fcn_received  = _last_fcn_received - tiles_in_payload ;
        _last_bitmap_ptr[w] = _last_fcn_received+1;   


        //this->print_tiles_array();

        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
        SPDLOG_WARN("|--- W={:<1}, FCN={:<2} --->| {:>2} tiles", w, fcn, tiles_in_payload);
        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

        if(_last_fcn_received <= 0)
        {
            SPDLOG_INFO("Sending SCHC ACK");
            uint8_t c = get_c(w);                               // obtiene el valor de c en base al _bitmap_array
            std::vector bitmap_vector = get_compress_bitmap(w); // obtiene el bitmap expresado como un arreglo de char    
            int len;
            char* buffer = nullptr;
            decoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len);
            _stack->send_downlink_frame(_dev_id, SCHC_FRAG_DOWNLINK_DIRECTION_RULE_ID, buffer, len);
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
            SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap: {}", w, c, get_bitmap_array_str(w));
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");
        }
        
    }

    return 0;
}

void SCHC_Ack_on_error::print_tiles_array()
{ 
    for(uint8_t w=0; w < _nMaxWindows; w++)
    {
        for(int fcn=(_windowSize-1); fcn>=0; fcn--)
        {
            if (_tiles_map.find({w,fcn}) != _tiles_map.end()) 
            {
                std::ostringstream oss;
                char* buff = _tiles_map[{w,(uint8_t)fcn}];
                for(int i=0; i<_tileSize;i++)
                {
                    oss << fmt::format("{:02x} ", static_cast<unsigned char>(buff[i]));
                }
                SPDLOG_INFO("w:{}, fcn:{}, {}", w, fcn, oss.str());
            }
        }
    }
}

uint8_t SCHC_Ack_on_error::mtu_upgrade(int mtu)
{
    SPDLOG_TRACE("Entering the function");

    SPDLOG_INFO("updating the MTU to: {} bytes", mtu);
    _current_L2_MTU = mtu;

    SPDLOG_TRACE("Leaving the function");

    return 0;
}

std::string SCHC_Ack_on_error::get_bitmap_array_str(uint8_t window)
{
    std::string bitmap_str = "";
    for(int fcn=(_windowSize-1); fcn>=0; fcn--)
    {
        if(_bitmap_map[{window,fcn}]==-1)
        {
            bitmap_str += std::to_string(0);
        }
        else
        {
            bitmap_str += std::to_string(_bitmap_map[{window,fcn}]);
        }
        
    }
    return bitmap_str;
}

std::vector<uint8_t> SCHC_Ack_on_error::get_compress_bitmap(uint8_t window)
{
    std::vector<uint8_t> compress_bitmap;

    uint8_t c = get_c(window);
    if(c == 0)
    {
        int n_bits_padding;
        int bitmap_ptr = _last_bitmap_ptr[window];
        std::vector<uint8_t> real_bitmap;

        /* Traspasa a un vector todos los bitmap que son usados en la ventana (1s, 0s y -1s).
        Los bitmaps traspasados son solo los usados. Definidos por el valor de bitmap_prt*/
        for(int i = (_windowSize-1); i >= bitmap_ptr; i++)
        {
            real_bitmap.push_back(_bitmap_map[{window,i}]);
        }

        /* Transforma los bitmap con valor -1 a 0 */
        for(uint8_t bit: real_bitmap)
        {
            if(bit == -1)
                bit = 0;
        }

        // Encontrar la posición del último 0
        size_t last_zero_position = -1;
        for (size_t i = 0; i < real_bitmap.size(); ++i) {
            if (real_bitmap[i] == 0) {
                last_zero_position = i;  // Actualizar la posición del último 0
            }
        }

        // Crear un nuevo vector con los valores hasta la posición del último 0
        compress_bitmap.assign(real_bitmap.begin(), real_bitmap.begin() + last_zero_position + 1);

    }

    return compress_bitmap;
}

uint8_t SCHC_Ack_on_error::get_c(uint8_t window)
{
    for (const auto& [key, value] : _bitmap_map)
    {
        if (key.first == window && value == 0) // Filtrar por la primera parte de la clave
        {  
            return 0;
        }
    }

    return 1;
}

