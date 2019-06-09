#ifndef PERIODIC_TIMER_HPP
#define PERIODIC_TIMER_HPP

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace msp {

class PeriodicTimer {
public:
    /**
     * @brief PeriodicTimer define a periodic timer
     * @param funct function that is called periodically
     * @param period_seconds period in seconds
     */
    PeriodicTimer(const std::function<void()> funct,
                  const double period_seconds);

    /**
     * @brief PeriodicTimer destructor
     */
    ~PeriodicTimer() { stop(); }

    /**
     * @brief start define and start background thread
     */
    bool start();

    /**
     * @brief stop tell thread to stop and wait for end
     */
    bool stop();

    /**
     * @brief getPeriod get period in seconds
     * @return period in seconds
     */
    double getPeriod() const { return period_us.count() / 1.e6; }

    /**
     * @brief setPeriod change the update period of timer thread
     * This will stop and restart the thread.
     * @param period_seconds period in seconds
     */
    void setPeriod(const double& period_seconds);

private:
    std::shared_ptr<std::thread> thread_ptr;
    std::function<void()> funct;
    std::chrono::duration<size_t, std::micro> period_us;
    std::timed_mutex mutex_timer;
    std::chrono::steady_clock::time_point tstart;

    std::atomic_flag running_ = ATOMIC_FLAG_INIT;
};

}  // namespace msp

#endif
