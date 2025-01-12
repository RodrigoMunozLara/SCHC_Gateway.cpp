#include "SCHC_ThreadSafeQueue.hpp"

void SCHC_ThreadSafeQueue::push(std::string dev_id, int rule_id, char* mesg, int len) {
    std::lock_guard<std::mutex> lock(_mutex);
    _queue.push({dev_id, rule_id, mesg, len});
}

bool SCHC_ThreadSafeQueue::pop(std::string& dev_id, int& rule_id, char*& mesg, int& len) {
    std::lock_guard<std::mutex> lock(_mutex);
    if(_queue.empty())
    {
        return false;
    }
    else
    {
        auto tuple = _queue.front();
        dev_id = std::get<0>(tuple);
        rule_id = std::get<1>(tuple);
        mesg = std::get<2>(tuple);
        len = std::get<3>(tuple);
        _queue.pop();
        return true;
    }
}

bool SCHC_ThreadSafeQueue::empty() {
    std::lock_guard<std::mutex> lock(_mutex);
    return _queue.empty();
}

size_t SCHC_ThreadSafeQueue::size()
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _queue.size();
}
