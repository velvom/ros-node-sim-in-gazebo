#ifndef MOBICAR_ROS_CONTROLLER_H
#define MOBICAR_ROS_CONTROLLER_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <gazebo_msgs/ModelStates.h>
#include <thread>
#include <vector>

// mrs - mobicarroscontroller
namespace mrs {

// forward declarations to avoid include cycle
class StopSign;
class Vehicle;

class MobicarRosController : public std::enable_shared_from_this<MobicarRosController>
{
public:
    explicit MobicarRosController(const ros::NodeHandle& nh);
    virtual ~MobicarRosController();

    // Initialize the publishers and subscriber
    void init();

    // Create vehicle objects and store the model names and their indices 
    void createVehicles();

    // Starts the vehicles
    void startVehicles();

    // Subscriber callback function
    void modelstateCallback(const gazebo_msgs::ModelStates::ConstPtr& modelstate);

    // To let the "stopped" vehicles drive thru intersection
    void scheduleVehicle(std::string vid);

    // Miscellaneous
    std::shared_ptr<MobicarRosController> get_shared_this() { return shared_from_this(); }

private:
    // public ros node handle
    ros::NodeHandle nh_;

    // This ROS node name
    std::string node_name_{""};
    
    // Subscriber to Gazebo node's ModelStates
    ros::Subscriber sub_;
    
    // To count subs to ModelStates
    unsigned int sub_callback_count_;

    // Used for throttling ModelStates topic messages
    ros::Time prev_time_;
    ros::Duration period_;

    // Pointer to an instance of the StopSign intersection
    std::shared_ptr<StopSign> ss_; 

    // To store additional threads
    std::shared_ptr<std::vector<std::thread>> threads_;

    // To store pointers to vehicle objects representing Mobicars
    std::vector<std::shared_ptr<Vehicle>> vehicles_{};
};

} // namespace mrs

#endif // MOBICAR_ROS_CONTROLLER_H
