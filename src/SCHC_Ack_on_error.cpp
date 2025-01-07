#include "SCHC_Ack_on_error.hpp"

SCHC_Ack_on_error::SCHC_Ack_on_error()
{
}

SCHC_Ack_on_error::~SCHC_Ack_on_error()
{
}

uint8_t SCHC_Ack_on_error::init(uint8_t ruleID, uint8_t dTag, uint8_t windowSize, uint8_t tileSize, uint8_t n, uint8_t ackMode, SCHC_Stack_L2 *stack_ptr, int retTimer, uint8_t ackReqAttempts)
{
    SPDLOG_TRACE("Entering the function");

    /* Static SCHC parameters */
    _ruleID = ruleID;
    _dTag = dTag;
    _windowSize = windowSize;
    _nMaxWindows = pow(2,n);
    _nFullTiles = 0;                // in tiles
    _lastTileSize = 0;              // in bytes
    _tileSize = tileSize;           // in bytes
    _ackMode = ackMode;
    _retransTimer = retTimer;       // in millis
    _maxAckReq = ackReqAttempts;

    /* Static LoRaWAN parameters*/
    _current_L2_MTU = stack_ptr->getMtu(true);
    _stack = stack_ptr;

    SPDLOG_TRACE("Leaving the function");

    return 0;
}

uint8_t SCHC_Ack_on_error::start(int rule_id, char *buffer, int len)
{
    SPDLOG_TRACE("Entering the function");

    /* Dynamic SCHC parameters */
    _currentState = STATE_RX_INIT;
    _currentWindow = 0;
    _currentFcn = (_windowSize)-1;
    _currentBitmap_ptr = 0;
    _currentTile_ptr = 0;

    uint8_t res;
    res = this->execute_machine(rule_id, buffer, len);

    SPDLOG_TRACE("Leaving the function");

    return res;
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

uint8_t SCHC_Ack_on_error::RX_INIT_recv_fragments(int rule_id, char *msg, int len)
{
    SPDLOG_INFO("Changing STATE: From STATE_RX_INIT --> STATE_RX_RCV_WINDOW");
    this->_currentState = STATE_RX_RCV_WINDOW;
    this->execute_machine(rule_id, msg, len);
    return 0;
}

uint8_t SCHC_Ack_on_error::RX_RCV_WIN_recv_fragments(int rule_id, char *msg, int len)
{
    SCHC_Message decoder;
    decoder.decode_message(SCHC_FRAG_PROTOCOL_LORAWAN, rule_id, msg, len);
    if(decoder.get_msg_type() == SCHC_REGULAR_FRAGMENT_MSG)
    {
        SPDLOG_INFO("Receiving a SCHC Regular fragment");
        std::string schc_payload = decoder.get_schc_payload();
        int tiles_in_payload = schc_payload.size()/_tileSize;
        SPDLOG_WARN("|-----W={:<1}, FCN={:<2}----->| {:>2} tiles recv", decoder.get_w(), decoder.get_fcn(), tiles_in_payload);

    }

    return 0;
}

uint8_t SCHC_Ack_on_error::mtuUpgrade(int mtu)
{
    SPDLOG_TRACE("Entering the function");

    SPDLOG_INFO("updating the MTU to: {} bytes", mtu);
    _current_L2_MTU = mtu;

    SPDLOG_TRACE("Leaving the function");

    return 0;
}

uint8_t SCHC_Ack_on_error::divideInTiles(char *buffer, int len)
{
/* Creates an array of size _nFullTiles x _tileSize to store the message. 
Each row of the array is a tile of tileSize bytes. It also determines 
the number of SCHC windows.*/

    SPDLOG_TRACE("Entering the function");


/* RFC9011
 5.6.2. Uplink Fragmentation: From Device to SCHC Gateway

Last tile: It can be carried in a Regular SCHC Fragment, alone 
in an All-1 SCHC Fragment, or with any of these two methods. 
Implementations must ensure that:
*/
    if((len%_tileSize)==0)
    {
        /* El ultimo tile es del tamaño de _TileSize */
        _nFullTiles = (len/_tileSize)-1;
        _lastTileSize = _tileSize;  
    }
    else
    {
        /* El ultimo tile es menor _TileSize */
        _nFullTiles = (len/_tileSize);
        _lastTileSize = len%_tileSize;
    }


    SPDLOG_INFO("Full Tiles number: {} tiles", _nFullTiles);
    SPDLOG_INFO("Last Tile Size: {} bytes", _lastTileSize);

    // memory allocated for elements of rows.
    _tilesArray = new char*[_nFullTiles];

    // memory allocated for  elements of each column.  
    for(int i = 0 ; i < _nFullTiles ; i++ )
    {
        _tilesArray[i] = new char[_tileSize];
    }

    uint8_t k=0;
    for(int i=0; i<_nFullTiles; i++)
    {
        for(int j=0; j<_tileSize;j++)
        {
            _tilesArray[i][j] = buffer[k];
            k++;
        }
    }

    _lastTile = new char[_lastTileSize];
    for(int i=0; i<_lastTileSize; i++)
    {
        _lastTile[i] = buffer[k];
        k++;
    }

    /* Numero de ventanas SCHC */
    if(len>(_tileSize*_windowSize*3))
    {
        _nWindows = 4;
    }
    else if(len>(_tileSize*_windowSize*2))
    {
        _nWindows = 3;
    }
    else if (len>(_tileSize*_windowSize))
    {
        _nWindows = 2;
    }
    else
    {
        _nWindows = 1;
    }

    SPDLOG_INFO("SCHC Windows number: {}", _nWindows);


    // // free the allocated memory 
    // for( uint8_t i = 0 ; i < len ; i++ ) {
    //     delete [] _tilesArray[i];
    // }
    // delete [] _tilesArray;

    SPDLOG_TRACE("Leaving the function");

    return 0;
}

uint8_t SCHC_Ack_on_error::extractTiles(uint8_t firstTileID, uint8_t nTiles, char *buff)
{
    SPDLOG_TRACE("Entering the function");


    int k=0;
    for(int i=firstTileID; i<(firstTileID+nTiles); i++)
    {
        for(int j=0;j<_tileSize;j++)
        {
            buff[k] = _tilesArray[i][j];
            k++;
        }
    }

    SPDLOG_TRACE("Leaving the function");

    return 0;
}

uint8_t SCHC_Ack_on_error::getCurrentWindow(int tile_ptr)
{
    if(tile_ptr < _windowSize)
    {
        return 0;
    }
    else if(tile_ptr < 2 * _windowSize)
    {
        return 1;
    }
    else if(tile_ptr < 3 * _windowSize)
    {
        return 2;
    }
    else if(tile_ptr < 4 * _windowSize)
    {
        return 3;
    }
    else
    {
        SPDLOG_ERROR("In LoRaWAN, it is not possible that more than (4 * _windowSize)) tiles");
    }
    return 0;
}

uint8_t SCHC_Ack_on_error::getCurrentFCN(int tile_ptr)
{
    if(tile_ptr < _windowSize)
    {
        return (_windowSize - 1 - tile_ptr);
    }
    else if(tile_ptr < 2 * _windowSize)
    {
        return (2*_windowSize - 1 - tile_ptr);
    }
    else if(tile_ptr < 3 * _windowSize)
    {
        return (3*_windowSize - 1 - tile_ptr);
    }
    else if(tile_ptr < 4 * _windowSize)
    {
        return (4*_windowSize - 1 - tile_ptr);
    }
    else
    {
        SPDLOG_ERROR("In LoRaWAN, it is not possible that more than (4 * _windowSize)) tiles");
    }
    return 0;
}

uint8_t SCHC_Ack_on_error::setBitmapArray(int tile_ptr, int tile_sent)
{
    /* Se obtiene la posicion al Bitmap Array
    a partir de la posicion del tile que se está enviando */
    uint8_t bitmap_ptr = this->getCurrentBitmap_ptr(tile_ptr);

    if((bitmap_ptr + tile_sent) <= _windowSize)
    {
        /* En este caso todos los tiles enviados
        pertenecen a la misma ventana */
        for(int i=bitmap_ptr; i<(bitmap_ptr + tile_sent); i++)
        {
            _currentBitmapArray[_currentWindow][i] = true;
        }
        return bitmap_ptr + tile_sent;
    }
    else
    {
        /* En este caso los tiles enviados
        pertenecen a la misma ventana (i) y a la (i+1)*/
        for(int i=bitmap_ptr; i<(_windowSize); i++)
        {
            SPDLOG_DEBUG("{}",i);
            _currentBitmapArray[_currentWindow][i] = true;
        }

        for(int i=0; i<((bitmap_ptr+tile_sent)-(_windowSize)); i++)
        {
            SPDLOG_DEBUG("{}",i);
            _currentBitmapArray[_currentWindow+1][i] = true;
        }
        return (bitmap_ptr+tile_sent)-(_windowSize);
    }


    return 0;
}

uint8_t SCHC_Ack_on_error::getCurrentBitmap_ptr(int tile_ptr)
{
    if(tile_ptr < _windowSize)
    {
        return (tile_ptr);
    }
    else if(tile_ptr < 2 * _windowSize)
    {
        return (tile_ptr - _windowSize);
    }
    else if(tile_ptr < 3 * _windowSize)
    {
        return (tile_ptr - 2*_windowSize);
    }
    else if(tile_ptr < 4 * _windowSize)
    {
        return (tile_ptr - 3*_windowSize);
    }
    else
    {
        SPDLOG_ERROR("In LoRaWAN, it is not possible that more than (4 * _windowSize)) tiles");
    }
    return 0;
}

uint8_t SCHC_Ack_on_error::createBitmapArray()
{
    // memory allocated for elements of rows.
    _currentBitmapArray = new bool*[_nWindows];

    // memory allocated for  elements of each column.  
    for(int i = 0 ; i < _nWindows ; i++ )
    {
        _currentBitmapArray[i] = new bool[_windowSize];
    }

    int k=0;
    for(int i=0; i<_nWindows; i++)
    {
        for(int j=0; j<_windowSize;j++)
        {
            _currentBitmapArray[i][j] = false;
            k++;
        }
    }

    return 0;
}

void SCHC_Ack_on_error::printCurrentBitmap(uint8_t nWindow)
{
    char* buff = new char[100];
    sprintf(buff," - Bitmap: "); 
    for(int i=0; i<_windowSize; i++)
    {
        if(_currentBitmapArray[nWindow][i])
        {
            sprintf(buff,"1");
        }
        else
        {
            sprintf(buff,"0");
        }
    }
    SPDLOG_INFO("{}",buff);
    delete buff;
}

void SCHC_Ack_on_error::printTileArray()
{
    SPDLOG_DEBUG("***********************************************");
    char* buff = new char[100];
    for(int i=0; i<_nFullTiles; i++)
    {
        if(i+1<10)
        {
            sprintf(buff,"Tile 0");
            sprintf(buff,"%d",i+1);
            sprintf(buff," --> ");
        }
        else
        {
            sprintf(buff,"Tile ");
            sprintf(buff,"%d",i+1);
            sprintf(buff," --> ");
        }
        for(int j=0; j<_tileSize;j++)
        {
            sprintf(buff,"%d",_tilesArray[i][j]);
            sprintf(buff," ");       
        }
        SPDLOG_DEBUG("{}",buff);
    }

    if(_nFullTiles+1<10)
    {
        sprintf(buff,"Tile 0");
        sprintf(buff,"%d",_nFullTiles+1);
        sprintf(buff," --> ");
    }
    else
    {
        sprintf(buff,"Tile ");
        sprintf(buff,"%d",_nFullTiles+1);
        sprintf(buff," --> ");
    }
    for(int i=0; i<_lastTileSize; i++)
    {
        sprintf(buff,"%d",_lastTile[i]);
        sprintf(buff," ");       
    }
    SPDLOG_DEBUG("{}",buff);
    SPDLOG_DEBUG("***********************************************");
}

void SCHC_Ack_on_error::printBitmapArray()
{
    char* buff = new char[100];

    for(int i=0; i<_nWindows; i++)
    {
        for(int j=0; j<_windowSize; j++)
        {
            if(_currentBitmapArray[i][j])
            {
                sprintf(buff,"1");
            }
            else
            {
                sprintf(buff,"0");
            }
        }
        SPDLOG_INFO("{}",buff);
        delete buff;      
    }
}

