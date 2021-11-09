#ifndef STOP_SIGN_H
#define STOP_SIGN_H

#include <mutex>
#include <deque>
#include <condition_variable>
#include <vector>

// mrs - mobicarroscontroller
namespace mrs {

// forward declarations to avoid include cycle
class MobicarRosController;
class Vehicle;

template <typename T>
class MessageQueue
{
public:
    MessageQueue() {}
    // Takes an rvalue reference of a particular message type 
    void send(T&& msg);
    // Return the message
    T receive();

private:
    // Stores objects of a particular message type
    std::deque<T> _queue;
    std::mutex _mutex;
    std::condition_variable _cond;
};

class StopSign : public std::enable_shared_from_this<StopSign>
{
public:
    // constructor / desctructor
    StopSign();
    virtual ~StopSign() = default;

    // getters / setters
    bool getCurrentPhase();
    void setCurrentPhase(const bool& phase);

    // typical behaviour methods
    void init(std::shared_ptr<std::vector<std::thread>>& threads);
    void addVehicleToQueue(std::string vid);
    void scheduleVehicle(std::string vid);
    void setRosController(std::shared_ptr<MobicarRosController> rctrl);

    // miscellaneous
    std::shared_ptr<StopSign> get_shared_this() { return shared_from_this(); }

private:
    // typical behaviour methods
    void processVehicles();

    // Queue for storing each vehicle's name in order of arrival to stop sign.
    std::shared_ptr<MessageQueue<std::string>> _ssvehiclesqueue;
 
    std::shared_ptr<MobicarRosController> _rosController;

    // To monitor the occupancy of intersection
    bool _intersection_busy{false};

    // Used to stop processVehicles()
    bool _end_processing{false};

    std::mutex _mutex;
};

} // namespace mrs

#endif // STOP_SIGN_H
