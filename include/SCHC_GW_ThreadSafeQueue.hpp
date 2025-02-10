#ifndef SCHC_GW_ThreadSafeQueue_hpp
#define SCHC_GW_ThreadSafeQueue_hpp

#include <iostream>
#include <queue>
#include <mutex>
#include <thread>

class SCHC_GW_ThreadSafeQueue {
    public:
        void push(uint8_t rule_id, char* mesg, int len);
        bool pop(uint8_t& rule_id, char*& mesg, int& len);
        bool empty();
        size_t size();
    private:
        std::queue<std::tuple<uint8_t, char*, int>> _queue;
        std::mutex _mutex;
};
#endif