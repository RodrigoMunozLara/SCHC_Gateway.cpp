#ifndef SCHC_ThreadSafeQueue_hpp
#define SCHC_ThreadSafeQueue_hpp

#include <iostream>
#include <queue>
#include <mutex>
#include <thread>

class SCHC_ThreadSafeQueue {
    public:
        void push(std::string dev_id, int rule_id, char* mesg, int len);
        bool pop(std::string& dev_id, int& rule_id, char*& mesg, int& len);
        bool empty();
        size_t size();
    private:
        std::queue<std::tuple<std::string, int, char*, int>> _queue;
        std::mutex _mutex;
};
#endif