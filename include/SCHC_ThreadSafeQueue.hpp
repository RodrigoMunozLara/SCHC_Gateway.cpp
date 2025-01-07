#ifndef SCHC_ThreadSafeQueue_hpp
#define SCHC_ThreadSafeQueue_hpp

#include <iostream>
#include <queue>
#include <mutex>
#include <thread>

class SCHC_ThreadSafeQueue {
    public:
        void push(int rule_id, std::string mesg, int len);
        bool pop(int& rule_id, std::string& mesg, int& len);
        bool empty();
        size_t size();
    private:
        std::queue<std::tuple<int, std::string, int>> _queue;
        std::mutex _mutex;
};
#endif