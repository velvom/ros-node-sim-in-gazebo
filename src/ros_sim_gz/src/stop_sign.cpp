#include <iostream>
#include <algorithm>
#include <future>
#include <stop_sign.h>
#include <mobicar_ros_controller.h>
#include <vehicle.h>

// mrs - mobicarroscontroller
namespace mrs {

// Implementation of class "MessageQueue"
template <typename T>
T MessageQueue<T>::receive()
{
    // Wait for and receive new messages
    std::unique_lock<std::mutex> lck(_mutex);
    _cond.wait(lck, [this] { return !_queue.empty(); });

    // Remove from front of the queue
    T msg = std::move(_queue.front());
    _queue.pop_front();

    return msg;
}

template <typename T>
void MessageQueue<T>::send(T&& msg)
{
    // perform deque modification under the lock
    std::lock_guard<std::mutex> lck(_mutex);

    // add message to deque
    _queue.push_back(std::move(msg));
    _cond.notify_one(); // notify client after pushing new message into deque
}

// Implementation of class "StopSign"
StopSign::StopSign()
{
    _ssvehiclesqueue = std::make_shared<MessageQueue<std::string>>();
}

void StopSign::setRosController(std::shared_ptr<MobicarRosController> rctrl)
{
    _rosController = rctrl;
}

void StopSign::addVehicleToQueue(std::string vid)
{
    if (!vid.empty()) {
        _ssvehiclesqueue->send(std::move(vid));
    }
}

void StopSign::setCurrentPhase(const bool& phase)
{
    std::lock_guard<std::mutex> lck(_mutex);
    _intersection_busy = phase;;
}

bool StopSign::getCurrentPhase()
{
    std::lock_guard<std::mutex> lck(_mutex);
    return _intersection_busy;
}

void StopSign::init(std::shared_ptr<std::vector<std::thread>>& threads)
{
    threads->emplace_back(std::thread(&StopSign::processVehicles, this));
}

void StopSign::processVehicles()
{
    while (true)
    {
        // Sleep at every iteration to reduce CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // Process the next vehicle in the queue if no vehicle is in the intersection
        if (!getCurrentPhase()) {
            ROS_INFO_STREAM("StopSign::processVehicles() in thread: " << std::hex << ::std::this_thread::get_id());
            auto msg = _ssvehiclesqueue->receive();

            // request entry to the current intersection (using async)
            auto ftrExitConfirmed = std::async(std::launch::async, &MobicarRosController::scheduleVehicle, _rosController, msg);

            // wait until vehicle is out of intersection
            ftrExitConfirmed.get();

            ROS_INFO_STREAM("StopSign::processVehicles(), Vehicle " << msg << " has exited intersection.");
        }
    }
}

} // namespace mrs
