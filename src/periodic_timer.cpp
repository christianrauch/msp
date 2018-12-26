#include "periodic_timer.hpp"

namespace msp {

PeriodicTimer::PeriodicTimer(std::function<void()> funct, const double period_seconds)
    : funct(funct), running_(ATOMIC_FLAG_INIT)//, running(false)
{
//    running_ = ATOMIC_FLAG_INIT;
    period_us = std::chrono::duration<size_t, std::micro>(size_t(period_seconds*1e6));
}

bool PeriodicTimer::start() {
    // only start thread if period is above 0
    if(!(period_us.count()>0))
        return false;
    //only start once
    if (running_.test_and_set()) return false;
    //lock mutex so that the running thread always times out
    mutex_timer.lock();
    //start the thread
    thread_ptr = std::shared_ptr<std::thread>(new std::thread(
    [this]{
        //running = true;
        while(true) {
            // call function and wait until end of period or stop is called
            const auto tstart = std::chrono::high_resolution_clock::now();
            funct();
            if (mutex_timer.try_lock_until(tstart + period_us)) {
                //gets here if lock was acquired (means someone called stop and manually unlocked the mutex)
                mutex_timer.unlock();
                break;
            }
        } // while running
    }
    ));
    return true;
}

bool PeriodicTimer::stop() {
    bool rc = false;
    if (running_.test_and_set()) {
        //was already running, so let the thread finish
        mutex_timer.unlock();
        if(thread_ptr!=nullptr && thread_ptr->joinable()) {
            thread_ptr->join();
        }
        rc = true;
    }
    running_.clear();
    return rc;
}

void PeriodicTimer::setPeriod(const double& period_seconds) {
    stop();
    period_us = std::chrono::duration<size_t, std::micro>(size_t(period_seconds*1e6));
    start();
}

} // namespace msp
