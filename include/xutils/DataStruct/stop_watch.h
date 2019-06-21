#ifndef XUTILS_DATA_STRUCTURE_STOP_WATCH_H
#define XUTILS_DATA_STRUCTURE_STOP_WATCH_H

#include <ctime>
#include <chrono>
#include <iostream>

namespace xutils
{

class StopWatch
{
    typedef std::chrono::high_resolution_clock clock;
    typedef std::chrono::milliseconds milliseconds;

public:
    StopWatch(bool run = false);
    void reset();
    milliseconds Elapsed() const;

private:
    clock::time_point start;
};

std::ostream &operator<<(std::ostream &os, const StopWatch &timer);

} // namespace xutils

#endif