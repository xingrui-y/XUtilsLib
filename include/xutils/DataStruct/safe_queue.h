#ifndef XUTILS_DATA_STRUCT_SAFE_QUEUE_H
#define XUTILS_DATA_STRUCT_SAFE_QUEUE_H

#include <queue>
#include <mutex>

namespace xutils
{

template <class T>
class SafeQueue
{
public:
    void push(const T &val);
    void push_safe(const T &val);
    void pop_safe();
    T front_safe();
    size_t size_safe();

private:
    std::mutex lock;
    std::queue<T> queue;
};

template <class T>
void SafeQueue<T>::push(const T &val)
{
    queue.push(val);
}

template <class T>
void SafeQueue<T>::push_safe(const T &val)
{
    std::unique_lock<std::mutex> ulock(lock);
    queue.push(val);
}

template <class T>
void SafeQueue<T>::pop_safe()
{
    std::unique_lock<std::mutex> ulock(lock);
    queue.pop();
}

template <class T>
T SafeQueue<T>::front_safe()
{
    std::unique_lock<std::mutex> ulock(lock);
    queue.front();
}

template <class T>
size_t SafeQueue<T>::size_safe()
{
    std::unique_lock<std::mutex> ulock(lock);
    return queue.size();
}

} // namespace xutils

#endif