#include "PeriodicTimer.hpp"

PeriodicTimer::PeriodicTimer(std::function<void()> funct, const double period_seconds)
    : funct(funct), running(false)
{
    period_us = std::chrono::duration<uint, std::micro>(uint(period_seconds*1e6));
}

void PeriodicTimer::start() {
    // only start thread if period is above 0
    if(!(period_us.count()>0))
        return;

    thread_ptr = std::shared_ptr<std::thread>(new std::thread(
    [this]{
        running = true;
        while(running) {
            // call function and wait until end of period
            const auto tstart = std::chrono::high_resolution_clock::now();
            funct();
            std::this_thread::sleep_until(tstart+period_us);
        } // while running
    }
    ));
}

void PeriodicTimer::stop() {
    running = false;
    if(thread_ptr!=nullptr && thread_ptr->joinable()) {
        thread_ptr->join();
    }
}

void PeriodicTimer::setPeriod(const double period_seconds) {
    stop();
    period_us = std::chrono::duration<uint, std::micro>(uint(period_seconds*1e6));
    start();
}
