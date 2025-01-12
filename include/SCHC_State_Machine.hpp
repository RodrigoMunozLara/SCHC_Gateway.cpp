#ifndef SCHC_State_Machine_hpp
#define SCHC_State_Machine_hpp

#include "SCHC_Stack_L2.hpp"
#include <cstddef>

class SCHC_State_Machine
{
    public:
        virtual ~SCHC_State_Machine()=0;
        virtual uint8_t init(uint8_t ruleID, uint8_t dTag, uint8_t windowSize, uint8_t tileSize, uint8_t n, uint8_t m, uint8_t ackMode, SCHC_Stack_L2* stack_ptr, int retTimer, uint8_t ackReqAttempts) = 0;
        virtual uint8_t execute_machine(int rule_id=0, char *msg=NULL, int len=0) = 0;
        virtual void    set_device_id(std::string dev_id) = 0;
};

#endif