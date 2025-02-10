#include "SCHC_GW_ThreadSafeQueue.hpp"

void SCHC_GW_ThreadSafeQueue::push(uint8_t rule_id, char* mesg, int len) {
    std::lock_guard<std::mutex> lock(_mutex);
    _queue.push({rule_id, mesg, len});
}

bool SCHC_GW_ThreadSafeQueue::pop(uint8_t& rule_id, char*& mesg, int& len) {
    std::lock_guard<std::mutex> lock(_mutex);
    if(_queue.empty())
    {
        return false;
    }
    else
    {
        auto tuple = _queue.front();
        rule_id = std::get<0>(tuple);
        mesg = std::get<1>(tuple);
        len = std::get<2>(tuple);
        _queue.pop();
        return true;
    }
}

bool SCHC_GW_ThreadSafeQueue::empty() {
    std::lock_guard<std::mutex> lock(_mutex);
    return _queue.empty();
}

size_t SCHC_GW_ThreadSafeQueue::size()
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _queue.size();
}
