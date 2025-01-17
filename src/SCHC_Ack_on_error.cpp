#include "SCHC_Ack_on_error.hpp"
#include "SCHC_Fragmenter_GW.hpp"

SCHC_Ack_on_error::SCHC_Ack_on_error()
{
    SPDLOG_DEBUG("Calling SCHC_Ack_on_error constructor");
}

SCHC_Ack_on_error::~SCHC_Ack_on_error()
{
    SPDLOG_DEBUG("Calling SCHC_Ack_on_error destructor");
}

uint8_t SCHC_Ack_on_error::init(std::string dev_id, uint8_t ruleID, uint8_t dTag, uint8_t windowSize, uint8_t tileSize, uint8_t n, uint8_t m, uint8_t ackMode, SCHC_Stack_L2 *stack_ptr, int retTimer, uint8_t ackReqAttempts)
{
    SPDLOG_TRACE("Entering the function");

    /* Static SCHC parameters */
    _ruleID             = ruleID;                   // Rule ID -> https://www.rfc-editor.org/rfc/rfc9011.html#name-ruleid-management
    _dTag               = dTag;                     // not used in LoRaWAN
    _windowSize         = windowSize;               // in tiles. In LoRaWAN: 63
    _nMaxWindows        = pow(2,m);                 // In LoRaWAN: 4
    _nTotalTiles        = windowSize * pow(2,m);    // in tiles. In LoRaWAN: 252
    _lastTileSize       = 0;                        // in bits
    _tileSize           = tileSize;                 // in bytes. In LoRaWAN: 10 bytes
    _ackMode            = ackMode;                  // Modes defined in SCHC_Macros.hpp
    _retransTimer       = retTimer;                 // in minutes. In LoRaWAN: 12*60*60 minutes
    _maxAckReq          = ackReqAttempts;           // in minutes. In LoRaWAN: 12*60*60 minutes
    _last_window        = -1;
    _currentWindow      = 0;
    _currentTile_ptr    = 0;
    _dev_id             = dev_id;
    _processing.store(false);

    /* Static LoRaWAN parameters*/
    _current_L2_MTU = stack_ptr->getMtu(true);
    _stack = stack_ptr;

   _currentState = STATE_RX_INIT;


    /* Se arranca el thread */
    _processing.store(true);
    auto self = shared_from_this();     // Crear un shared_ptr que apunta a este objeto

    _process_thread = std::thread(&SCHC_Ack_on_error::thread_entry_point, self);
    _process_thread.detach();           // Desatachar el hilo para que sea independiente


    SPDLOG_TRACE("Leaving the function");

    return 0;
}

uint8_t SCHC_Ack_on_error::execute_machine(int rule_id, char *msg, int len)
{
    SPDLOG_TRACE("Entering the function");

    if(msg!=NULL)
    {
        if(_currentState==STATE_RX_INIT)
        {
            SPDLOG_DEBUG("Calling to RX_INIT_recv_fragments() method");
            this->RX_INIT_recv_fragments(rule_id, msg, len);
        }
        else if (_currentState==STATE_RX_RCV_WINDOW)
        {
            SPDLOG_DEBUG("Calling to RX_RCV_WIN_recv_fragments() method");
            this->RX_RCV_WIN_recv_fragments(rule_id, msg, len);
        }
        else if(_currentState==STATE_RX_END)
        {
            SPDLOG_DEBUG("Calling to RX_END_end_session() method");
            this->RX_END_end_session(rule_id, msg, len);
        }
        else
        {
            SPDLOG_ERROR("State not defined");
        }
    }
    else
    {
        
    }


    SPDLOG_TRACE("Leaving the function");

    return 0;
}

uint8_t SCHC_Ack_on_error::queue_message(int rule_id, char *msg, int len)
{
    _queue.push(rule_id, msg, len);
    return 0;
}

void SCHC_Ack_on_error::thread_entry_point(std::shared_ptr<SCHC_Ack_on_error> instance)
{
    if (instance)
    {
        instance->message_reception_loop();
    }
}

void SCHC_Ack_on_error::message_reception_loop()
{
    SPDLOG_INFO("Entering message_reception_loop()");
    while(_processing.load())
    {
        if(_queue.size() != 0)
        {
            SPDLOG_DEBUG("\033[31mExtracting message from the queue.\033[0m");
            SPDLOG_DEBUG("Current queue size: {}", _queue.size());
            char* msg = nullptr;
            int len;
            _queue.pop(_ruleID, msg, len);
            this->execute_machine(_ruleID, msg, len);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Llamar al callback al finalizar
    if (_end_callback)
    {
        SPDLOG_WARN("Releasing memory resources in the state machine");
        /* Liberando memoria de _tailArray*/
        for(int i = 0 ; i < _nTotalTiles ; i++ )
        {
            delete[] _tilesArray[i];
        }
        delete[] _tilesArray;

        /* Liberando memoria de _bitmapArray*/
        for(int i = 0 ; i < _nMaxWindows ; i++)
        {
            delete[] _bitmapArray[i];
        }
        delete[] _bitmapArray;

        _end_callback();
    }

    SPDLOG_WARN("Thread finished");
    return;
}

bool SCHC_Ack_on_error::is_processing()
{
    return _processing.load();
}

void SCHC_Ack_on_error::set_end_callback(std::function<void()> callback)
{
    _end_callback = callback;
}

uint8_t SCHC_Ack_on_error::RX_INIT_recv_fragments(int rule_id, char *msg, int len)
{
    /* memory allocated for pointers of each tile. */
    _tilesArray = new char*[_nTotalTiles];          // * Liberada en SCHC_Ack_on_error::destroy_machine()

    /* memory allocated for the bytes for each tile. */  
    for(int i = 0 ; i < _nTotalTiles ; i++ )
    {
        _tilesArray[i] = new char[_tileSize];       // * Liberada en SCHC_Ack_on_error::destroy_machine()
    }



    /* memory allocated for pointers of each bitmap. */
    _bitmapArray = new uint8_t*[_nMaxWindows];      // * Liberada en SCHC_Ack_on_error::destroy_machine()

    /* memory allocated for the 1s and 0s for each bitmap. */ 
    for(int i = 0 ; i < _nMaxWindows ; i++ )
    {
        _bitmapArray[i] = new uint8_t[_windowSize]; // * Liberada en SCHC_Ack_on_error::destroy_machine()
    }

    /* Setting all bitmaps in 0*/
    for(int i=0; i<_nMaxWindows; i++)
    {
        for(int j = 0 ; j < _windowSize ; j++)
        {
            _bitmapArray[i][j] = 0;
        }
    }


    SPDLOG_INFO("Changing STATE: From STATE_RX_INIT --> STATE_RX_RCV_WINDOW");
    this->_currentState = STATE_RX_RCV_WINDOW;

    this->execute_machine(rule_id, msg, len);
    return 0;
}

uint8_t SCHC_Ack_on_error::RX_RCV_WIN_recv_fragments(int rule_id, char *msg, int len)
{
    SCHC_Message    decoder;
    uint8_t         msg_type;       // message type decoded. See message type in SCHC_Macros.hpp
    uint8_t         w;              // w recibido en el mensaje
    uint8_t         dtag;           // dtag recibido en el mensajes
    uint8_t         fcn;            // fcn recibido en el mensaje
    char*           payload;
    int             payload_len;    // in bits
    uint32_t        rcs;

    msg_type = decoder.get_msg_type(SCHC_FRAG_LORAWAN, rule_id, msg, len);

    if(msg_type == SCHC_REGULAR_FRAGMENT_MSG)
    {
        SPDLOG_DEBUG("Receiving a SCHC Regular fragment");

        /* Decoding el SCHC fragment */
        decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
        payload_len     = decoder.get_schc_payload_len();   // largo del payload SCHC. En bits
        fcn             = decoder.get_fcn();
        w               = decoder.get_w();

        /* Si se reciben tiles de otra ventana que aun no se procesa, se descarta el mensaje */
        if(w != _currentWindow)
        {
            SPDLOG_INFO("Receiving a SCHC Fragment with tiles of other window. Discarding message");
            return 0;
        }

        /* Creacion de buffer para almacenar el schc payload del SCHC fragment */
        char* payload   = new char[payload_len/8];          // * Liberada en linea 158
        decoder.get_schc_payload(payload);                  // obtiene el SCHC payload


        /* Obteniendo la cantidad de tiles en el mensaje */
        int tiles_in_payload = (payload_len/8)/_tileSize;

        /* Se almacenan los tiles en el mapa de recepción de tiles */
        int tile_ptr    = this->get_tile_ptr(w, fcn);   // tile_ptr: posicion donde se debe almacenar el tile en el _tileArray.
        int bitmap_ptr  = this->get_bitmap_ptr(fcn);    // bitmap_ptr: posicion donde se debe comenzar escribiendo un 1 en el _bitmapArray.
        for(int i=0; i<tiles_in_payload; i++)
        {
            memcpy(_tilesArray[tile_ptr + i], payload + (i*_tileSize), _tileSize);  // se almacenan los bytes de un tile recibido
            _bitmapArray[w][bitmap_ptr + i] = 1;                                    // en el bitmap, se establece en 1 los correspondientes tiles recibidos
        }
        _currentTile_ptr = _currentTile_ptr + tiles_in_payload;
        delete[] payload;

        /* Se imprime mensaje de la llegada de un SCHC fragment*/
        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
        SPDLOG_WARN("|--- W={:<1}, FCN={:<2} --->| {:>2} tiles", w, fcn, tiles_in_payload);
        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

        /* Si los tiles recibido son los ultimo de la ventana, se envia un SCHC ACK */
        if((fcn - tiles_in_payload) <= 0)
        {
            SPDLOG_DEBUG("Sending SCHC ACK");
            uint8_t c                   = this->get_c_from_bitmap(w);         // obtiene el valor de c en base al _bitmap_array
            std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
            int len;
            char* buffer                = nullptr;
            decoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len);
            _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
            delete[] buffer;

            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
            SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

            _currentWindow = _currentWindow + 1;
        }
        
    }
    else if(msg_type == SCHC_ALL1_FRAGMENT_MSG)
    {
        SPDLOG_DEBUG("Receiving a SCHC All-1 message");
        decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);

        _lastTileSize   = decoder.get_schc_payload_len();   // largo del payload SCHC. En bits
        w               = decoder.get_w();
        _last_window    = w;
        rcs             = decoder.get_rcs();

        _last_tile   = new char[_lastTileSize/8];       // * Liberada en linea 222
        decoder.get_schc_payload(_last_tile);           // obtiene el SCHC payload

        _bitmapArray[w][_windowSize-1] = 1;

        if(this->check_rcs(rcs))
        {
            SPDLOG_INFO("Integrity check: success");
            SPDLOG_DEBUG("Sending SCHC ACK");
            uint8_t c                   = get_c_from_bitmap(w);                     // obtiene el valor de c en base al _bitmap_array
            std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
            int len;
            char* buffer                = nullptr;
            decoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len);
            _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
            delete[] buffer;

            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
            SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

            SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_END");
            _currentState = STATE_RX_END;
        }
        else
        {
            SPDLOG_INFO("Integrity check: failure");
            // TODO: Enviar un ACK con el bitmap con error

        }

        delete[] _last_tile;


    }
    else if(msg_type == SCHC_ACK_REQ_MSG)
    {
        SPDLOG_WARN("Receiving SCHC ACK REQ. Discarding message");
    }
    else
    {
        SPDLOG_ERROR("Receiving an unexpected type of message. Discarding message");
    }

    return 0;
}

uint8_t SCHC_Ack_on_error::RX_END_end_session(int rule_id, char *msg, int len)
{
    SCHC_Message    decoder;
    uint8_t         msg_type;       // message type decoded. See message type in SCHC_Macros.hpp
    uint8_t         w;              // w recibido en el mensaje
    uint8_t         dtag;           // dtag recibido en el mensajes
    uint8_t         fcn;            // fcn recibido en el mensaje
    char*           payload;
    int             payload_len;    // in bits
    uint32_t        rcs;

    msg_type = decoder.get_msg_type(SCHC_FRAG_LORAWAN, rule_id, msg, len);

    if(msg_type == SCHC_ACK_REQ_MSG)
    {
        SPDLOG_DEBUG("Receiving a SCHC ACK REQ  fragment");

        SPDLOG_INFO("Ending Session...");

        _processing.store(false);

        // Asegurar que el hilo finalice
        if (_process_thread.joinable())
        {
            _process_thread.join();
        }

    }

    return 0;
}

std::string SCHC_Ack_on_error::get_bitmap_array_str(uint8_t window)
{
    std::string bitmap_str = "";
    for(int i=0; i<_windowSize; i++)
    {
        bitmap_str = bitmap_str + std::to_string(_bitmapArray[window][i]);
    }
    return bitmap_str;
}

uint8_t SCHC_Ack_on_error::get_c_from_bitmap(uint8_t window)
{
    /* La funcion indica si faltan tiles para la ventana pasada como argumento.
    Retorna un 1 si no faltan tiles y 0 si faltan tiles */
    if(_last_window == window)
    {
        // TODO: calcular si hay errores en la ultima ventana
    }
    else if(_last_window != -1 && _last_window != window)
    {
        for (int i=0; i<_windowSize; i++)
        {
            if(_bitmapArray[window][i] == 0)
                return 0;
        }
    }


    return 1;
}

std::vector<uint8_t> SCHC_Ack_on_error::get_bitmap_array_vec(uint8_t window)
{
    std::vector<uint8_t> bitmap_v;
    for(int i=0; i<_windowSize; i++)
    {
        bitmap_v.push_back(_bitmapArray[window][i]);
    }

    return bitmap_v;
}

bool SCHC_Ack_on_error::check_rcs(uint32_t rcs)
{
    // Calcular el tamaño total necesario para el buffer de todos los tiles
    int total_size = (_currentTile_ptr * _tileSize) + _lastTileSize/8;

    // Crear un buffer para almacenar todos los valores
    char buffer[total_size];

    int k=0;
    for(int i=0; i<_currentTile_ptr; i++)
    {
        for(int j=0; j<_tileSize; j++)
        {
            buffer[k] = _tilesArray[i][j];
            k++;
        } 
    }

    for(int i=0; i<_lastTileSize/8; i++)
    {
            buffer[k] = _last_tile[i];
            k++;
    }

    uint32_t rcs_calculed = this->calculate_crc32(buffer, total_size);

    SPDLOG_DEBUG("calculated RCS: {}", rcs_calculed);
    SPDLOG_DEBUG("  received RCS: {}", rcs);

    if(rcs_calculed == rcs)
        return true;
    else
        return false;
}

uint32_t SCHC_Ack_on_error::calculate_crc32(const char *data, size_t length) 
{
    // Polinomio CRC32 (reflejado)
    const uint32_t polynomial = 0xEDB88320;
    uint32_t crc = 0xFFFFFFFF;

    // Procesar cada byte en el buffer
    for (size_t i = 0; i < length; i++) {
        crc ^= static_cast<uint8_t>(data[i]); // Asegúrate de que el dato sea tratado como uint8_t

        // Procesar los 8 bits del byte
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ polynomial;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFF;
}

int SCHC_Ack_on_error::get_tile_ptr(uint8_t window, uint8_t fcn)
{
    if(window==0)
    {
        return (_windowSize - 1) - fcn;
    }
    else if(window == 1)
    {
        return (2*_windowSize - 1) - fcn;
    }
    else if(window == 2)
    {
        return (3*_windowSize - 1) - fcn;
    }
    else if(window == 3)
    {
        return (4*_windowSize - 1) - fcn;
    }
    return -1;
}

int SCHC_Ack_on_error::get_bitmap_ptr(uint8_t fcn)
{
    return (_windowSize - 1) - fcn;
}

void SCHC_Ack_on_error::print_tail_array_str()
{
    // Calcular el tamaño total necesario para el buffer de todos los tiles
    int len = _currentTile_ptr * _tileSize;   // 2520 bytes

    // Crear un buffer para almacenar todos los valores
    char buffer[len];

    int k=0;
    for(int i=0; i<_currentTile_ptr; i++)
    {
        for(int j=0; j<_tileSize; j++)
        {
            buffer[k] = _tilesArray[i][j];
            k++;
        } 
    }

    std::string resultado;

    for (size_t i = 0; i < len; ++i) {
        unsigned char valor = static_cast<unsigned char>(buffer[i]);
        if (std::isprint(valor)) {
            // Agregar caracteres imprimibles directamente
            resultado += valor;
        } else {
            // Convertir caracteres no imprimibles a formato hexadecimal
            resultado += fmt::format("{:02X}", valor);
        }
    }

    // Imprimir usando spdlog
    SPDLOG_DEBUG("Buffer string content: {}", resultado);
}

void SCHC_Ack_on_error::print_tail_array_hex()
{
    // Calcular el tamaño total necesario para el buffer de todos los tiles
    int len = _currentTile_ptr * _tileSize;   // 2520 bytes

    // Crear un buffer para almacenar todos los valores
    char buffer[len];

    int k=0;
    for(int i=0; i<_currentTile_ptr; i++)
    {
        for(int j=0; j<_tileSize; j++)
        {
            buffer[k] = _tilesArray[i][j];
            k++;
        } 
    }

    std::string resultado;

    for (size_t i = 0; i < len; ++i) 
    {
        unsigned char valor = static_cast<unsigned char>(buffer[i]);
        resultado += fmt::format("{:02X} ", valor);
    }

    // Imprimir usando spdlog
    SPDLOG_DEBUG("Buffer hex content: {}", resultado);
}

