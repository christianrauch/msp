#ifndef PERIODICTIMER_HPP
#define PERIODICTIMER_HPP

#include <functional>
#include <thread>

class PeriodicTimer {
public:
    /**
     * @brief PeriodicTimer define a periodic timer
     * @param funct function that is called periodically
     * @param period_seconds period in seconds
     */
    PeriodicTimer(const std::function<void()> funct, const double period_seconds);

    /**
     * @brief start define and start background thread
     */
    void start();

    /**
     * @brief stop tell thread to stop and wait for end
     */
    void stop();

private:
    std::shared_ptr<std::thread> thread_ptr;
    std::function<void()> funct;
    std::chrono::duration<uint, std::micro> period_us;
    bool running;
};

#endif // PERIODICTIMER_HPP
