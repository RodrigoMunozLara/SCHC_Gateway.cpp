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
    _last_window        = 0;
    _dev_id             = dev_id;
    _processing.store(false);
    _rcs                = 0;


    /* Dynamic SCHC parameters */
    _currentState           = STATE_RX_INIT;
    _currentTile_ptr        = 0;
    _last_confirmed_window  = 0;


    /* Static LoRaWAN parameters*/
    _current_L2_MTU = stack_ptr->getMtu(true);
    _stack = stack_ptr;


    /* Thread and Queue Message*/
    _processing.store(true);
    auto self       = shared_from_this();     // Crear un shared_ptr que apunta a este objeto
    _process_thread = std::thread(&SCHC_Ack_on_error::thread_entry_point, self);
    _process_thread.detach();           // Desatachar el hilo para que sea independiente


    /* Flags */
    _all_tiles_received_flag    = false;
    _wait_pull_ack_req_flag     = false;


    /* Contador para saber que mensaje eliminar de forma manual*/
    _counter = 1;

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
        else if(_currentState==STATE_RX_WAIT_x_MISSING_FRAGS)
        {
            SPDLOG_DEBUG("Calling to RX_WAIT_x_MISSING_FRAGS_recv_fragments() method");
            this->RX_WAIT_x_MISSING_FRAGS_recv_fragments(rule_id, msg, len);
        }
        else
        {
            SPDLOG_ERROR("State not defined");
        }
    }
    else
    {
        if(_currentState==STATE_RX_END)
        {
            SPDLOG_DEBUG("Calling to RX_END_end_session() method");
            this->RX_END_end_session();
        }
        else
        {
            SPDLOG_ERROR("State not defined");
        }        
    }

    SPDLOG_DEBUG("\033[32mLeaving the function\033[0m");
    SPDLOG_DEBUG("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
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

void SCHC_Ack_on_error::set_error_prob(uint8_t error_prob)
{
    this->_error_prob = error_prob;
}

void SCHC_Ack_on_error::message_reception_loop()
{
    SPDLOG_INFO("Entering message_reception_loop()");
    while(_processing.load())
    {
        if(_queue.size() != 0)
        {
            SPDLOG_DEBUG(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
            SPDLOG_DEBUG("\033[32mExtracting message from the queue.\033[0m");
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

        delete[] _last_tile;

        /* Liberando memoria de _bitmapArray*/
        for(int i = 0 ; i < _nMaxWindows ; i++)
        {
            delete[] _bitmapArray[i];
        }
        delete[] _bitmapArray;

        _end_callback();
    }

    SPDLOG_WARN("\033[1mThread finished\033[0m");
    SPDLOG_WARN("");
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

    /* Setting all tiles in 0x00 (for good printing in string format) */
    for(int i = 0 ; i < _nTotalTiles ; i++ )
    {
        for(int j=0; j < _tileSize; j++)
        {
            _tilesArray[i][j] = 0x00;
        }
    }


    /* memory allocated for the bytes for the last tile. */  
    _last_tile = new char[_tileSize];       // * Liberada en SCHC_Ack_on_error::destroy_machine()
    for(int i=0; i < _tileSize; i++)
        {
            _last_tile[i] = 0x00;
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

    this->RX_RCV_WIN_recv_fragments(rule_id, msg, len);
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

    msg_type = decoder.get_msg_type(SCHC_FRAG_LORAWAN, rule_id, msg, len);

    if(_ackMode == ACK_MODE_ACK_END_WIN)
    {
        if(msg_type == SCHC_REGULAR_FRAGMENT_MSG)
        {
            /* Codigo para poder eliminar mensajes de entrada */
            // std::random_device rd;
            // std::mt19937 gen(rd());
            // std::uniform_int_distribution<int> dist(0, 100);
            // int random_number = dist(gen);
            // //if(random_number < _error_prob)
            // if(_counter == 2 || _counter == 4)
            // {
            //         SPDLOG_WARN("\033[31mMessage discarded due to error probability\033[0m");   
            //         _counter++;
            //         return 0;
            // }
            // _counter++;
            

            SPDLOG_DEBUG("Receiving a SCHC Regular fragment");

            /* Decoding el SCHC fragment */
            decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
            payload_len     = decoder.get_schc_payload_len();   // largo del payload SCHC. En bits
            fcn             = decoder.get_fcn();
            w               = decoder.get_w();

            if(w > _last_window)
                _last_window    = w;    // aseguro que el ultimo fragmento recibido va a marcar cual es la ultima ventana recibida

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
            delete[] payload;

            /* Se almacena el puntero al siguiente tile esperado */
            if((tile_ptr + tiles_in_payload) > _currentTile_ptr)
            {
                _currentTile_ptr = tile_ptr + tiles_in_payload;
                SPDLOG_DEBUG("Updating _currentTile_ptr. New value is: {}", _currentTile_ptr);
            }
            else
            {
                SPDLOG_DEBUG("_currentTile_ptr is not updated. The previous value is kept {}", _currentTile_ptr);
            }
                
            
            /* Se imprime mensaje de la llegada de un SCHC fragment*/
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
            SPDLOG_WARN("|--- W={:<1}, FCN={:<2} --->| {:>2} tiles", w, fcn, tiles_in_payload);
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");


            /* Si los tiles recibido son los ultimos de la ventana, se envia un SCHC ACK */
            if((fcn - tiles_in_payload) <= 0)
            {
                SPDLOG_DEBUG("Sending SCHC ACK");
                SCHC_Message    encoder;
                uint8_t c                   = this->get_c_from_bitmap(w);    // obtiene el valor de c en base al _bitmap_array
                std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
                int len;
                char* buffer                = nullptr;
                encoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len);

                _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                delete[] buffer;

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                if(c==0)
                {
                    SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                    _currentState = STATE_RX_WAIT_x_MISSING_FRAGS;
                }
                
                _wait_pull_ack_req_flag = true;
            }
            
        }
        else if(msg_type == SCHC_ALL1_FRAGMENT_MSG)
        {
            // if(true)
            // {
            //         SPDLOG_WARN("\033[31mMessage discarded due to error probability\033[0m");   
            //         _counter++;
            //         return 0;
            // }

            SPDLOG_DEBUG("Receiving a SCHC All-1 message");
            decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);

            _lastTileSize   = decoder.get_schc_payload_len();   // largo del payload SCHC. En bits
            w               = decoder.get_w();
            _last_window    = w;
            _rcs            = decoder.get_rcs();
            fcn             = decoder.get_fcn();
            decoder.get_schc_payload(_last_tile);           // obtiene el SCHC payload

            bool rcs_result = this->check_rcs(_rcs);

            if(rcs_result)  // * Integrity check: success
            {
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|- W={:<1}, FCN={:<2}+RCS ->| {:>2} bits - Integrity check: success", w, fcn, _lastTileSize);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                _bitmapArray[w][_windowSize-1] = 1;

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
            else                // * Integrity check: failure
            {
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|- W={:<1}, FCN={:<2}+RCS ->| {:>2} bits - Integrity check: failure", w, fcn, _lastTileSize);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                _bitmapArray[w][_windowSize-1] = 1;

                SPDLOG_DEBUG("Sending SCHC ACK");
                SCHC_Message    encoder;
                uint8_t c                   = 0;
                std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
                int len;
                char* buffer                = nullptr;
                encoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len, false);

                _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                delete[] buffer;

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                _currentState = STATE_RX_WAIT_x_MISSING_FRAGS;

                _wait_pull_ack_req_flag = true;

            }
        
        }
        else if(msg_type == SCHC_ACK_REQ_MSG)
        {
            if(_wait_pull_ack_req_flag == true)
            {
                SPDLOG_DEBUG("Receiving SCHC ACK REQ");
                decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
                w               = decoder.get_w();

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|--- ACK REQ, W={:<1} -->| pull ACK REQ discarded", w);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");
                _wait_pull_ack_req_flag = false;
            }
            else
            {
                SPDLOG_DEBUG("Receiving SCHC ACK REQ");
                decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
                w               = decoder.get_w();

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|--- ACK REQ, W={:<1} -->| ", w);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                SPDLOG_DEBUG("Sending SCHC ACK");
                SCHC_Message    encoder;
                uint8_t c                   = this->get_c_from_bitmap(w);    // obtiene el valor de c en base al _bitmap_array
                std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char 
                int len;
                char* buffer                = nullptr;
                encoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len);

                _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                delete[] buffer;

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                if(c==0)
                {
                    SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                    _currentState = STATE_RX_WAIT_x_MISSING_FRAGS;
                    _wait_pull_ack_req_flag = true;
                }

                
            }
            

        }
        else
        {
            SPDLOG_ERROR("Receiving an unexpected type of message. Discarding message");
        }
    }
    else if(_ackMode == ACK_MODE_ACK_END_SES || _ackMode == ACK_MODE_COMPOUND_ACK)
    {
        if(msg_type == SCHC_REGULAR_FRAGMENT_MSG)
        {
            SPDLOG_DEBUG("Receiving a SCHC Regular fragment");

            /* Codigo para poder eliminar mensajes de entrada */
            // std::random_device rd;
            // std::mt19937 gen(rd());
            // std::uniform_int_distribution<int> dist(0, 100);
            // int random_number = dist(gen);
            
            //if(random_number < _error_prob)
            // if(_counter == 3 || _counter == 5 )
            // {
            //         SPDLOG_WARN("\033[31mMessage discarded due to error probability\033[0m");   
            //         _counter++;
            //         return 0;
            // }
            // _counter++;

            /* Decoding el SCHC fragment */
            decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
            payload_len     = decoder.get_schc_payload_len();   // largo del payload SCHC. En bits
            fcn             = decoder.get_fcn();
            w               = decoder.get_w();

            if(w > _last_window)
                _last_window    = w;    // aseguro que el ultimo fragmento recibido va a marcar cual es la ultima ventana recibida

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
                
                if((bitmap_ptr + i) > (_windowSize - 1))
                {
                    /* ha finalizado la ventana w y ha comenzado la ventana w+1*/
                    _bitmapArray[w+1][bitmap_ptr + i - _windowSize] = 1;
                }
                else
                {
                    _bitmapArray[w][bitmap_ptr + i] = 1;
                }
                
            }
            delete[] payload;

            /* Se almacena el puntero al siguiente tile esperado */
            if((tile_ptr + tiles_in_payload) > _currentTile_ptr)
            {
                _currentTile_ptr = tile_ptr + tiles_in_payload;
                SPDLOG_DEBUG("Updating _currentTile_ptr. New value is: {}", _currentTile_ptr);
            }
            else
            {
                SPDLOG_DEBUG("_currentTile_ptr is not updated. The previous value is kept {}", _currentTile_ptr);
            }

            /* Se imprime mensaje de la llegada de un SCHC fragment*/
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
            SPDLOG_WARN("|--- W={:<1}, FCN={:<2} --->| {:>2} tiles", w, fcn, tiles_in_payload);
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");
            
        }
        else if(msg_type == SCHC_ALL1_FRAGMENT_MSG) 
        { 
            // SPDLOG_WARN("\033[31mMessage discarded due to error probability\033[0m"); 
            // return 0;

            SPDLOG_DEBUG("Receiving a SCHC All-1 message");
            decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);

            _lastTileSize   = decoder.get_schc_payload_len();   // largo del payload SCHC. En bits
            w               = decoder.get_w();
            _last_window    = w;
            _rcs            = decoder.get_rcs();
            fcn             = decoder.get_fcn();
            decoder.get_schc_payload(_last_tile);           // obtiene el SCHC payload

            bool rcs_result = this->check_rcs(_rcs);

            if(rcs_result)    // * Integrity check: success
            {
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|- W={:<1}, FCN={:<2}+RCS ->| {:>2} bits - Integrity check: success", w, fcn, _lastTileSize);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                _bitmapArray[w][_windowSize-1] = 1;

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
            else                        // * Integrity check: failure
            {
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|- W={:<1}, FCN={:<2}+RCS ->| {:>2} bits - Integrity check: failure", w, fcn, _lastTileSize);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                _bitmapArray[w][_windowSize-1] = 1;

                if(_ackMode == ACK_MODE_ACK_END_SES)        // * SCH ACK at the end of the session
                {
                    /* Revisa cual ventana tiene errores y envia un ACK para esa ventana */
                    for(int i = _last_confirmed_window; i<_last_window; i++)
                    {
                        int len;
                        char* buffer                = nullptr;
                        uint8_t c                   = get_c_from_bitmap(i);
                        if(c == 0)
                        {
                            SPDLOG_DEBUG("Sending SCHC ACK");

                            SCHC_Message    encoder;
                            _last_confirmed_window      = i;
                            std::vector bitmap_vector   = this->get_bitmap_array_vec(i);

                            encoder.create_schc_ack(_ruleID, dtag, i, c, bitmap_vector, buffer, len);

                            _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                            delete[] buffer;

                            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                            SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", i, c, get_bitmap_array_str(i));
                            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                            SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                            _currentState = STATE_RX_WAIT_x_MISSING_FRAGS;   

                            _wait_pull_ack_req_flag = true;

                            return 0;                     
                        }
                        else
                        {
                            SPDLOG_WARN("SCHC Window {} has received all tiles. No ACK sent", i);
                        }
                    }

                    /* Si llegó a esta parte del codigo es porque ninguna ventana tiene errores. 
                    Por lo tanto la ventana con errores es la última.*/
                    SPDLOG_DEBUG("Sending SCHC ACK");
                    SCHC_Message    encoder;
                    std::vector bitmap_vector   = this->get_bitmap_array_vec(_last_window); // obtiene el bitmap expresado como un arreglo de char    
                    int len;
                    char* buffer                = nullptr;
                    int c                       = 0;
                    encoder.create_schc_ack(_ruleID, dtag, _last_window, c, bitmap_vector, buffer, len);

                    _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                    delete[] buffer;

                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                    SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", _last_window, c, get_bitmap_array_str(_last_window));
                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");
                    _last_confirmed_window = _last_window; 

                    SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                    _currentState = STATE_RX_WAIT_x_MISSING_FRAGS; 

                    _wait_pull_ack_req_flag = true;

                }
                else if(_ackMode == ACK_MODE_COMPOUND_ACK)  // * SCHC Compound ACK at the end of the session
                {
                    SPDLOG_DEBUG("Sending SCHC Compound ACK");
                    int len;
                    char* buffer                = nullptr;
                    int c                       = 0;
                    decoder.create_schc_ack_compound(_ruleID, dtag, _last_window, c, _bitmapArray, _windowSize, buffer, len);

                    _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                    delete[] buffer;

                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                    SPDLOG_WARN("|<-- ACK, C={:<1} --------| {}", c, decoder.get_compound_bitmap_str());
                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                    SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                    _currentState = STATE_RX_WAIT_x_MISSING_FRAGS; 
                }
            }
        }
        else if(msg_type == SCHC_ACK_REQ_MSG)
        {
            if(_wait_pull_ack_req_flag == true)
            {
                SPDLOG_DEBUG("Receiving SCHC ACK REQ");
                decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
                w               = decoder.get_w();

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|--- ACK REQ, W={:<1} -->| pull ACK REQ discarded", w);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                _wait_pull_ack_req_flag = false;
            }
            else
            {
                SPDLOG_DEBUG("Receiving SCHC ACK REQ");
                decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
                uint8_t w_received          = decoder.get_w();

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|--- ACK REQ, W={:<1} -->| ", w_received);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                if(w_received > _last_window)
                    _last_window    = w_received;    // aseguro que el ultimo fragmento recibido va a marcar cual es la ultima ventana recibida



                /* Revisa cual ventana tiene errores y envia un ACK para esa ventana */
                for(int i = _last_confirmed_window; i<_last_window; i++)
                {
                    int len;
                    char* buffer                = nullptr;
                    uint8_t c                   = get_c_from_bitmap(i);
                    if(c == 0)
                    {
                        SPDLOG_DEBUG("Sending SCHC ACK");

                        SCHC_Message    encoder;
                        _last_confirmed_window      = i;
                        std::vector bitmap_vector   = this->get_bitmap_array_vec(i);

                        encoder.create_schc_ack(_ruleID, dtag, i, c, bitmap_vector, buffer, len);

                        _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                        delete[] buffer;

                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                        SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", i, c, get_bitmap_array_str(i));
                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                        SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                        _currentState = STATE_RX_WAIT_x_MISSING_FRAGS;   

                        _wait_pull_ack_req_flag = true;

                        return 0;                     
                    }
                    else
                    {
                        SPDLOG_WARN("SCHC Window {} has received all tiles. No ACK sent", i);
                    }
                }

                /* Si llegó a esta parte del codigo es porque ninguna ventana tiene errores. 
                Por lo tanto la ventana con errores es la última.*/
                SPDLOG_DEBUG("Sending SCHC ACK");
                SCHC_Message    encoder;
                std::vector bitmap_vector   = this->get_bitmap_array_vec(_last_window); // obtiene el bitmap expresado como un arreglo de char    
                int len;
                char* buffer                = nullptr;
                int c                       = 0;
                encoder.create_schc_ack(_ruleID, dtag, _last_window, c, bitmap_vector, buffer, len);

                _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                delete[] buffer;

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", _last_window, c, get_bitmap_array_str(_last_window));
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");
                _last_confirmed_window = _last_window; 

                SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                _currentState = STATE_RX_WAIT_x_MISSING_FRAGS; 

                _wait_pull_ack_req_flag = true;
                
            }
 
        }
        else
        {
            SPDLOG_ERROR("Receiving an unexpected type of message. Discarding message");
        }        
    }


    return 0;
}

uint8_t SCHC_Ack_on_error::RX_END_end_session(int rule_id, char *msg, int len)
{
    if(msg!=nullptr)
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
            /* Decoding el SCHC fragment */
            decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
            w               = decoder.get_w();

            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
            SPDLOG_WARN("|--- ACK REQ, W={:<1} -->| pull ACK REQ discarded", w);
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");
        }
    }

    SPDLOG_WARN("Ending Session...");

    _processing.store(false);

    // Asegurar que el hilo finalice
    if (_process_thread.joinable())
    {
        _process_thread.join();
    }

    return 0;
}

uint8_t SCHC_Ack_on_error::RX_WAIT_x_MISSING_FRAGS_recv_fragments(int rule_id, char *msg, int len)
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

    if(_ackMode == ACK_MODE_ACK_END_WIN)
    {
        if(msg_type == SCHC_REGULAR_FRAGMENT_MSG)
        {
            SPDLOG_DEBUG("Receiving a SCHC Regular fragment");

            // /* Codigo para poder eliminar mensajes de entrada */
            // std::random_device rd;
            // std::mt19937 gen(rd());
            // std::uniform_int_distribution<int> dist(0, 100);
            // int random_number = dist(gen);
            // //if(random_number < _error_prob)
            // if(_counter == 5)
            // {
            //         SPDLOG_WARN("\033[31mMessage discarded due to error probability\033[0m");   
            //         _counter++;
            //         return 0;
            // }
            // _counter++;

            /* Decoding el SCHC fragment */
            decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
            payload_len     = decoder.get_schc_payload_len();   // largo del payload SCHC. En bits
            fcn             = decoder.get_fcn();
            w               = decoder.get_w();

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
            delete[] payload;


            /* Se almacena el puntero al siguiente tile esperado */
            if((tile_ptr + tiles_in_payload) > _currentTile_ptr)
            {
                _currentTile_ptr = tile_ptr + tiles_in_payload;
                SPDLOG_DEBUG("Updating _currentTile_ptr. New value is: {}", _currentTile_ptr);
            }
            else
            {
                SPDLOG_DEBUG("_currentTile_ptr is not updated. The previous value is kept {}", _currentTile_ptr);
            }

            /* Se imprime mensaje de la llegada de un SCHC fragment*/
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
            SPDLOG_WARN("|--- W={:<1}, FCN={:<2} --->| {:>2} tiles", w, fcn, tiles_in_payload);
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

            /* Valida si se han recibido todos los tiles retransmitidos por el sender */
            uint8_t c = this->get_c_from_bitmap(w);
            
            if((w == _last_window) && (c==1))
            {
                bool valid_rcs = this->check_rcs(_rcs);
                if(valid_rcs)
                {
                    SPDLOG_DEBUG("Sending SCHC ACK");
                    std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
                    int len;
                    char* buffer                = nullptr;
                    decoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len);

                    _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                    delete[] buffer;

                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                    SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                    SPDLOG_INFO("Changing STATE: From STATE_RX_WAIT_x_MISSING_FRAGS --> STATE_RX_END");
                    _currentState = STATE_RX_END;
                }
                else if(!valid_rcs)
                {
                    SPDLOG_DEBUG("Sending SCHC ACK");

                    SCHC_Message    encoder;
                    std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
                    int len;
                    char* buffer                = nullptr;
                    c                           = 0;

                    encoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len);

                    _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                    delete[] buffer;

                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                    SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                    _wait_pull_ack_req_flag = true;               
                }
            }
            else if((w != _last_window) && (c==1))
            {
                SPDLOG_DEBUG("Sending SCHC ACK");
                SCHC_Message    encoder;

                std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
                int len;
                char* buffer                = nullptr;
                encoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len);

                _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                delete[] buffer;

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                SPDLOG_INFO("Changing STATE: From STATE_RX_WAIT_x_MISSING_FRAGS --> STATE_RX_RCV_WINDOW");
                _currentState = STATE_RX_RCV_WINDOW;

                _wait_pull_ack_req_flag = true;              
            }
        }
        else if(msg_type == SCHC_ACK_REQ_MSG)
        {
            SPDLOG_DEBUG("Receiving SCHC ACK REQ");

            if(_wait_pull_ack_req_flag == true)
            {
                /* Decoding el SCHC fragment */
                decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
                w               = decoder.get_w();

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|--- ACK REQ, W={:<1} -->| pull ACK REQ discarded", w);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");
                _wait_pull_ack_req_flag = false;
            }
            else
            {
                /* Decoding el SCHC fragment */
                decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
                w               = decoder.get_w();

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|--- ACK REQ, W={:<1} -->| ", w);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                /* Valida si se han recibido todos los tiles retransmitidos por el sender */
                uint8_t c = this->get_c_from_bitmap(w);
                
                if((w == _last_window) && (c==1))
                {
                    bool valid_rcs = this->check_rcs(_rcs);
                    if(valid_rcs)
                    {
                        SPDLOG_DEBUG("Sending SCHC ACK");
                        std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
                        int len;
                        char* buffer                = nullptr;
                        decoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len);

                        _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                        delete[] buffer;

                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                        SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                        SPDLOG_INFO("Changing STATE: From STATE_RX_WAIT_x_MISSING_FRAGS --> STATE_RX_END");
                        _currentState = STATE_RX_END;
                    }
                    else if(!valid_rcs)
                    {
                        SPDLOG_DEBUG("Sending SCHC ACK");

                        SCHC_Message    encoder;
                        std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
                        int len;
                        char* buffer                = nullptr;
                        c                           = 0;

                        encoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len);

                        _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                        delete[] buffer;

                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                        SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");                
                    
                        _wait_pull_ack_req_flag = true;
                    }
                }
                else if((w != _last_window) && (c==1))
                {
                    SPDLOG_DEBUG("Sending SCHC ACK");

                    SCHC_Message    encoder;
                    std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
                    int len;
                    char* buffer                = nullptr;
                    encoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len);

                    _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                    delete[] buffer;

                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                    SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                    SPDLOG_INFO("Changing STATE: From STATE_RX_WAIT_x_MISSING_FRAGS --> STATE_RX_RCV_WINDOW");
                    _currentState = STATE_RX_RCV_WINDOW;                  
                
                    _wait_pull_ack_req_flag = true;
                }
                else
                {
                    SPDLOG_DEBUG("Sending SCHC ACK");

                    SCHC_Message    encoder;
                    std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
                    int len;
                    char* buffer                = nullptr;

                    encoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len);

                    _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                    delete[] buffer;

                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                    SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");                
                
                    _wait_pull_ack_req_flag = true;
                }

            }


        }
        else if(msg_type == SCHC_ALL1_FRAGMENT_MSG)
        {
            SPDLOG_DEBUG("Receiving a SCHC All-1 message");
            decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);

            _lastTileSize   = decoder.get_schc_payload_len();   // largo del payload SCHC. En bits
            w               = decoder.get_w();
            _last_window    = w;
            _rcs            = decoder.get_rcs();
            fcn             = decoder.get_fcn();
            decoder.get_schc_payload(_last_tile);           // obtiene el SCHC payload

            bool rcs_result = this->check_rcs(_rcs);

            if(rcs_result)  // * Integrity check: success
            {
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|- W={:<1}, FCN={:<2}+RCS ->| {:>2} bits - Integrity check: success", w, fcn, _lastTileSize);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                _bitmapArray[w][_windowSize-1] = 1;

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
            else            // * Integrity check: failure
            {
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|- W={:<1}, FCN={:<2}+RCS ->| {:>2} bits - Integrity check: failure", w, fcn, _lastTileSize);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                _bitmapArray[w][_windowSize-1] = 1;

                SPDLOG_DEBUG("Sending SCHC ACK");

                SCHC_Message    encoder;
                uint8_t c                   = 0;
                std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
                int len;
                char* buffer                = nullptr;

                encoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len, false);

                _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                delete[] buffer;

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                _currentState = STATE_RX_WAIT_x_MISSING_FRAGS;

                _wait_pull_ack_req_flag = true;
            }
        }
        else
        {
            SPDLOG_ERROR("Receiving an unexpected type of message. Discarding message");
        }
    }
    else if(_ackMode == ACK_MODE_ACK_END_SES)
    {
        if(msg_type == SCHC_REGULAR_FRAGMENT_MSG)
        {
            SPDLOG_DEBUG("Receiving a SCHC Regular fragment");

            /* Decoding el SCHC fragment */
            decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
            payload_len     = decoder.get_schc_payload_len();   // largo del payload SCHC. En bits
            fcn             = decoder.get_fcn();
            w               = decoder.get_w();

            if(w > _last_window)
                _last_window    = w;    // aseguro que el ultimo fragmento recibido va a marcar cual es la ultima ventana recibida


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
            delete[] payload;


            /* Se almacena el puntero al siguiente tile esperado */
            if((tile_ptr + tiles_in_payload) > _currentTile_ptr)
            {
                _currentTile_ptr = tile_ptr + tiles_in_payload;
                SPDLOG_DEBUG("Updating _currentTile_ptr. New value is: {}", _currentTile_ptr);
            }
            else
            {
                SPDLOG_DEBUG("_currentTile_ptr is not updated. The previous value is kept {}", _currentTile_ptr);
            }

            /* Se imprime mensaje de la llegada de un SCHC fragment*/
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
            SPDLOG_WARN("|--- W={:<1}, FCN={:<2} --->| {:>2} tiles", w, fcn, tiles_in_payload);
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

            /* Valida en el bitmap si se han recibido todos los tiles retransmitidos por el sender */
            uint8_t c = this->get_c_from_bitmap(w);

            if(c==1 && w!=_last_window)
            {
                int next_window = w + 1;
                if(next_window == _last_window)
                {
                    int len;
                    char* buffer        = nullptr;
                    bool valid_rcs = this->check_rcs(_rcs);

                    if(valid_rcs)
                    {
                        SPDLOG_DEBUG("Sending SCHC ACK");
                        std::vector bitmap_vector   = this->get_bitmap_array_vec(next_window); // obtiene el bitmap expresado como un arreglo de char    
                        int len;
                        char* buffer                = nullptr;
                        decoder.create_schc_ack(_ruleID, dtag, next_window, 1, bitmap_vector, buffer, len);

                        _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                        delete[] buffer;

                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                        SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", next_window, 1, get_bitmap_array_str(next_window));
                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                        SPDLOG_INFO("Changing STATE: From STATE_RX_WAIT_x_MISSING_FRAGS --> STATE_RX_END");
                        _currentState = STATE_RX_END;
                    }
                    else
                    {
                        SPDLOG_DEBUG("Sending SCHC ACK");

                        SCHC_Message    encoder;
                        std::vector bitmap_vector   = this->get_bitmap_array_vec(_last_window); // obtiene el bitmap expresado como un arreglo de char    
                        int len;
                        char* buffer                = nullptr;
                        c                           = 0;

                        encoder.create_schc_ack(_ruleID, dtag, _last_window, c, bitmap_vector, buffer, len);

                        _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                        delete[] buffer;

                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                        SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", _last_window, c, get_bitmap_array_str(_last_window));
                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");
                        _last_confirmed_window = _last_window; 

                        _wait_pull_ack_req_flag = true;
                    }
                }
                else
                {
                    for(int i = next_window; i<_last_window; i++)
                    {
                        int len;
                        char* buffer    = nullptr;
                        int c_i         = get_c_from_bitmap(i);
                        if(c_i == 0)
                        {
                            SPDLOG_DEBUG("Sending SCHC ACK");

                            SCHC_Message    encoder;
                            std::vector bitmap_vector   = this->get_bitmap_array_vec(i);

                            encoder.create_schc_ack(_ruleID, dtag, i, c_i, bitmap_vector, buffer, len);

                            _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                            delete[] buffer;

                            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                            SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", i, c_i, get_bitmap_array_str(i));
                            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                            _last_confirmed_window = i; 
                            _wait_pull_ack_req_flag = true;
                            break;  
                        }
                        else
                        {
                            SPDLOG_WARN("The SCHC gateway correctly received the tiles for window {}.", i);
                        }

                    }// cierre del for
                }
            }
            else if(c==1 && w==_last_window)
            {
                int len;
                char* buffer        = nullptr;
                bool valid_rcs = this->check_rcs(_rcs);
                if(valid_rcs)
                {
                    SPDLOG_DEBUG("Sending SCHC ACK");
                    std::vector bitmap_vector   = this->get_bitmap_array_vec(_last_window); // obtiene el bitmap expresado como un arreglo de char    
                    int len;
                    char* buffer                = nullptr;
                    decoder.create_schc_ack(_ruleID, dtag, _last_window, 1, bitmap_vector, buffer, len);

                    _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                    delete[] buffer;

                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                    SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", _last_window, 1, get_bitmap_array_str(_last_window));
                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                    SPDLOG_INFO("Changing STATE: From STATE_RX_WAIT_x_MISSING_FRAGS --> STATE_RX_END");
                    _currentState = STATE_RX_END;
                }
                else
                {
                    SPDLOG_DEBUG("Sending SCHC ACK");

                    SCHC_Message    encoder;
                    std::vector bitmap_vector   = this->get_bitmap_array_vec(_last_window); // obtiene el bitmap expresado como un arreglo de char    
                    int len;
                    char* buffer                = nullptr;
                    c                           = 0;
                    encoder.create_schc_ack(_ruleID, dtag, _last_window, c, bitmap_vector, buffer, len);

                    _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                    delete[] buffer;

                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                    SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", _last_window, c, get_bitmap_array_str(_last_window));
                    spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");
                    _last_confirmed_window  = _last_window; 

                    _wait_pull_ack_req_flag = true;
                }
            }
        }
        else if(msg_type == SCHC_ACK_REQ_MSG)
        {
            SPDLOG_DEBUG("Receiving SCHC ACK REQ");
            if(_wait_pull_ack_req_flag == true)
            {
                /* Decoding el SCHC fragment */
                decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
                w               = decoder.get_w();

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|--- ACK REQ, W={:<1} -->| pull ACK REQ discarded", w);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");
                _wait_pull_ack_req_flag = false;
            }
            else
            {
                /* Decoding el SCHC fragment */
                decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
                uint8_t w_received               = decoder.get_w();

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|--- ACK REQ, W={:<1} -->| ", w_received);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");


                /* Revisa cual ventana tiene errores y envia un ACK para esa ventana */
                for(int i = _last_confirmed_window; i<_last_window; i++)
                {
                    int len;
                    char* buffer                = nullptr;
                    uint8_t c                   = get_c_from_bitmap(i);
                    if(c == 0)
                    {
                        SPDLOG_DEBUG("Sending SCHC ACK");

                        SCHC_Message    encoder;
                        _last_confirmed_window      = i;
                        std::vector bitmap_vector   = this->get_bitmap_array_vec(i);

                        encoder.create_schc_ack(_ruleID, dtag, i, c, bitmap_vector, buffer, len);

                        _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                        delete[] buffer;

                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                        SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", i, c, get_bitmap_array_str(i));
                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                        _wait_pull_ack_req_flag = true;

                        return 0;                     
                    }
                    else
                    {
                        SPDLOG_WARN("SCHC Window {} has received all tiles. No ACK sent", i);
                    }
                }

                /* Si llegó a esta parte del codigo es porque ninguna ventana tiene errores. 
                Por lo tanto la ventana con errores es la última.*/
                SPDLOG_DEBUG("Sending SCHC ACK");
                SCHC_Message    encoder;
                std::vector bitmap_vector   = this->get_bitmap_array_vec(_last_window); // obtiene el bitmap expresado como un arreglo de char    
                int len;
                char* buffer                = nullptr;
                int c                       = 0;
                encoder.create_schc_ack(_ruleID, dtag, _last_window, c, bitmap_vector, buffer, len);

                _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                delete[] buffer;

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", _last_window, c, get_bitmap_array_str(_last_window));
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");
                _last_confirmed_window = _last_window; 

                _wait_pull_ack_req_flag = true;

            }
            
        }
        else if(msg_type == SCHC_ALL1_FRAGMENT_MSG)
        {
            SPDLOG_DEBUG("Receiving a SCHC All-1 message");
            decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);

            _lastTileSize   = decoder.get_schc_payload_len();   // largo del payload SCHC. En bits
            w               = decoder.get_w();
            _last_window    = w;
            _rcs            = decoder.get_rcs();
            fcn             = decoder.get_fcn();
            decoder.get_schc_payload(_last_tile);           // obtiene el SCHC payload

            bool rcs_result = this->check_rcs(_rcs);

            if(rcs_result)  // * Integrity check: success
            {
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|- W={:<1}, FCN={:<2}+RCS ->| {:>2} bits - Integrity check: success", w, fcn, _lastTileSize);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                _bitmapArray[w][_windowSize-1] = 1;

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
            else            // * Integrity check: failure
            {
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|- W={:<1}, FCN={:<2}+RCS ->| {:>2} bits - Integrity check: failure", w, fcn, _lastTileSize);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                _bitmapArray[w][_windowSize-1] = 1;

                SPDLOG_DEBUG("Sending SCHC ACK");

                SCHC_Message    encoder;
                uint8_t c                   = 0;
                std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
                int len;
                char* buffer                = nullptr;

                encoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len, false);

                _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                delete[] buffer;

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                _currentState = STATE_RX_WAIT_x_MISSING_FRAGS;

                _wait_pull_ack_req_flag = true;
            }
        }

    }
    else if(_ackMode == ACK_MODE_COMPOUND_ACK)
    {
        if(msg_type == SCHC_REGULAR_FRAGMENT_MSG)
        {
            SPDLOG_DEBUG("Receiving a SCHC Regular fragment");

            // if(_counter == 6 )
            // {
            //         SPDLOG_WARN("\033[31mMessage discarded due to error probability\033[0m");   
            //         _counter++;
            //         return 0;
            // }
            // _counter++;


            /* Decoding el SCHC fragment */
            decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
            payload_len     = decoder.get_schc_payload_len();   // largo del payload SCHC. En bits
            fcn             = decoder.get_fcn();
            w               = decoder.get_w();

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
            delete[] payload;


            /* Se almacena el puntero al siguiente tile esperado */
            if((tile_ptr + tiles_in_payload) > _currentTile_ptr)
            {
                _currentTile_ptr = tile_ptr + tiles_in_payload;
                SPDLOG_DEBUG("Updating _currentTile_ptr. New value is: {}", _currentTile_ptr);
                _all_tiles_received_flag = true;
            }
            else
            {
                SPDLOG_DEBUG("_currentTile_ptr is not updated. The previous value is kept {}", _currentTile_ptr);
            }

            /* Se imprime mensaje de la llegada de un SCHC fragment*/
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
            SPDLOG_WARN("|--- W={:<1}, FCN={:<2} --->| {:>2} tiles", w, fcn, tiles_in_payload);
            spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");


            if(_all_tiles_received_flag)
            {
                /* Valida en el bitmap si se han recibido todos los tiles retransmitidos por el sender */
                if(this->check_rcs(_rcs))    // * Integrity check: success
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
                else                        // * Integrity check: failure
                {
                    SPDLOG_INFO("Integrity check: failure");

                    if(_ackMode == ACK_MODE_ACK_END_SES)        // * SCH ACK at the end of the session
                    {
                        for(int i = _last_confirmed_window; i<_last_window; i++)
                        {
                            int len;
                            char* buffer                = nullptr;
                            uint8_t c                   = get_c_from_bitmap(i);
                            if(c == 0)
                            {
                                _last_confirmed_window = i;
                                std::vector bitmap_vector   = this->get_bitmap_array_vec(i);
                                decoder.create_schc_ack(_ruleID, dtag, i, c, bitmap_vector, buffer, len);
                                _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                                delete[] buffer;

                                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                                SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", i, c, get_bitmap_array_str(i));
                                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                                SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                                _currentState = STATE_RX_WAIT_x_MISSING_FRAGS;   
                                return 0;                     
                            }
                            else
                            {
                                SPDLOG_WARN("SCHC Window {} has received all tiles. No ACK sent", i);
                            }
                        }

                        /* Si llegó a esta parte del codigo es porque ninguna ventana tiene errores. 
                        Por lo tanto la ventana con errores es la última.*/
                        SPDLOG_DEBUG("Sending SCHC ACK");
                        std::vector bitmap_vector   = this->get_bitmap_array_vec(_last_window); // obtiene el bitmap expresado como un arreglo de char    
                        int len;
                        char* buffer                = nullptr;
                        int c                       = 0;
                        decoder.create_schc_ack(_ruleID, dtag, _last_window, c, bitmap_vector, buffer, len);

                        _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                        delete[] buffer;

                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                        SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", _last_window, c, get_bitmap_array_str(_last_window));
                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");
                        _last_confirmed_window = _last_window; 

                        SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                        _currentState = STATE_RX_WAIT_x_MISSING_FRAGS; 

                    }
                    else if(_ackMode == ACK_MODE_COMPOUND_ACK)  // * SCHC Compound ACK at the end of the session
                    {
                        SPDLOG_DEBUG("Sending SCHC Compound ACK");
                        int len;
                        char* buffer                = nullptr;
                        int c                       = 0;
                        decoder.create_schc_ack_compound(_ruleID, dtag, _last_window, c, _bitmapArray, _windowSize, buffer, len);

                        _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                        delete[] buffer;

                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                        SPDLOG_WARN("|<-- ACK, C={:<1} --| {}", c, decoder.get_compound_bitmap_str());
                        spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                        SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                        _currentState = STATE_RX_WAIT_x_MISSING_FRAGS; 
                    }
                }

            }
  
        }        
        else if(msg_type == SCHC_ACK_REQ_MSG)
        {
            SPDLOG_DEBUG("Receiving SCHC ACK REQ");
            if(_wait_pull_ack_req_flag == true)
            {
                /* Decoding el SCHC fragment */
                decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
                w               = decoder.get_w();

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|--- ACK REQ, W={:<1} -->| pull ACK REQ discarded", w);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");
                _wait_pull_ack_req_flag = false;
            }
            else
            {
                /* Decoding el SCHC fragment */
                decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);
                w               = decoder.get_w();

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|--- ACK REQ, W={:<1} -->| ", w);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                SPDLOG_DEBUG("Sending SCHC ACK");

                SCHC_Message    encoder;
                uint8_t c                   = this->get_c_from_bitmap(w);
                std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
                int len;
                char* buffer                = nullptr;
                encoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len);

                _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                delete[] buffer;

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");     

                _wait_pull_ack_req_flag = true;
            }
            
        
        }
        else if(msg_type == SCHC_ALL1_FRAGMENT_MSG)
        {
            SPDLOG_DEBUG("Receiving a SCHC All-1 message");
            decoder.decode_message(SCHC_FRAG_LORAWAN, rule_id, msg, len);

            _lastTileSize   = decoder.get_schc_payload_len();   // largo del payload SCHC. En bits
            w               = decoder.get_w();
            _last_window    = w;
            _rcs            = decoder.get_rcs();
            fcn             = decoder.get_fcn();
            decoder.get_schc_payload(_last_tile);           // obtiene el SCHC payload

            bool rcs_result = this->check_rcs(_rcs);

            if(rcs_result)  // * Integrity check: success
            {
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|- W={:<1}, FCN={:<2}+RCS ->| {:>2} bits - Integrity check: success", w, fcn, _lastTileSize);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                _bitmapArray[w][_windowSize-1] = 1;

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
            else            // * Integrity check: failure
            {
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|- W={:<1}, FCN={:<2}+RCS ->| {:>2} bits - Integrity check: failure", w, fcn, _lastTileSize);
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                _bitmapArray[w][_windowSize-1] = 1;

                SPDLOG_DEBUG("Sending SCHC ACK");

                SCHC_Message    encoder;
                uint8_t c                   = 0;
                std::vector bitmap_vector   = this->get_bitmap_array_vec(w); // obtiene el bitmap expresado como un arreglo de char    
                int len;
                char* buffer                = nullptr;

                encoder.create_schc_ack(_ruleID, dtag, w, c, bitmap_vector, buffer, len, false);

                _stack->send_downlink_frame(_dev_id, SCHC_FRAG_UPDIR_RULE_ID, buffer, len);
                delete[] buffer;

                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t] %v");
                SPDLOG_WARN("|<-- ACK, W={:<1}, C={:<1} --| Bitmap:{}", w, c, get_bitmap_array_str(w));
                spdlog::set_pattern("[%H:%M:%S.%e][%^%L%$][%t][%-8!s][%-8!!] %v");

                SPDLOG_INFO("Changing STATE: From STATE_RX_RCV_WINDOW --> STATE_RX_WAIT_x_MISSING_FRAGS");
                _currentState = STATE_RX_WAIT_x_MISSING_FRAGS;

                _wait_pull_ack_req_flag = true;
            }
        }
    }

    return 0;
}

uint8_t SCHC_Ack_on_error::get_c_from_bitmap(uint8_t window)
{
    /* La funcion indica si faltan tiles para la ventana pasada como argumento.
    Retorna un 1 si no faltan tiles y 0 si faltan tiles */

    if(window ==_last_window)
    {
        int bitmap_ptr = _currentTile_ptr - (_last_window*_windowSize);
        //SPDLOG_WARN("bitmap_ptr: {}",bitmap_ptr);
        for (int i=0; i<bitmap_ptr; i++)
        {
            if(_bitmapArray[window][i] == 0)
                return 0;
        }

        if(_bitmapArray[window][_windowSize-1] == 0)
                return 0;
    }
    else
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

    SPDLOG_INFO("calculated RCS: {}", rcs_calculed);
    SPDLOG_INFO("  received RCS: {}", rcs);

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

void SCHC_Ack_on_error::print_tail_array_hex()
{
    // Calcular el tamaño total necesario para el buffer de todos los tiles
    int len = _nTotalTiles * _tileSize;   // 2520 bytes

    // Crear un buffer para almacenar todos los valores
    char buffer[len];

    int k=0;
    for(int i=0; i<_nTotalTiles; i++)
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
        resultado += fmt::format("{:02X}", valor);
    }

    // Imprimir usando spdlog
    SPDLOG_WARN("Tile Array (hex): {}", resultado);

    std::string resultado2;
    for (size_t i = 0; i < _tileSize; ++i) 
    {
        unsigned char valor = static_cast<unsigned char>(_last_tile[i]);
        resultado2 += fmt::format("{:02X}", valor);
    }
    SPDLOG_WARN("Last Tile (hex): {}", resultado2);


}

void SCHC_Ack_on_error::print_bitmap_array_str()
{
    for(int i=0; i<_nMaxWindows; i++)
    {
        std::ostringstream oss;
        oss << "Bitmap window " + std::to_string(i) + ": " + get_bitmap_array_str(i);
        SPDLOG_WARN("{}", oss.str());
    }
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
