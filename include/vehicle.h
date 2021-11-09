#ifndef VEHICLE_H
#define VEHICLE_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <mutex>

// mrs - mobicarroscontroller
namespace mrs {

class Vehicle : public std::enable_shared_from_this<Vehicle>
{
public:
    enum Direction {
        DIR_NONE = -1,
        DIR_TOWARDS_STOPSIGN = 0,
        DIR_AWAY_STOPSIGN = 1
    };

    enum State {
        STATE_STOPPED = 0,
        STATE_INMOTION = 1
    };

    // constructor / desctructor
    Vehicle(const ros::NodeHandle& nh,
            const std::string& vid,
            const int& model_idx);
    virtual ~Vehicle();

    void init();

    // behaviour methods

    // Publish desired velocity to Mobicar
    void pubVelocity(const double& vel);

    int getModelIndex() const {return model_idx_;}
    std::string getModelName() const {return model_name_;}

    void storeVehiclePosition(const geometry_msgs::Point& pos);
    const geometry_msgs::Point& retrieveVehiclePosition();
    void storeVehicleLinearVelocity(const geometry_msgs::Vector3& linear_vel);
    const geometry_msgs::Vector3& getVehicleLinearVelocity();
    double calculateDistance();
    double calculateVelocity();

    void setVehicleDirection(Direction dir);
    Direction getVehicleDirection();
    void setVehicleState(State state);
    State getVehicleState();
    void setVehicleAtIntersection(bool intersection);
    bool getVehicleAtIntersection();

    // miscellaneous
    std::shared_ptr<Vehicle> get_shared_this() { return shared_from_this(); }

private:
    // ros node handle
    ros::NodeHandle nh_;

    // Publisher to send velocity to mobicar
    ros::Publisher pub_;

    // Vehicle model index in Gazebo list of models
    int model_idx_;

    // Vehicle ID
    std::string model_name_;

    // Vehicle Position
    geometry_msgs::Point pos_{};
   
    // Vehicle linear velocity
    geometry_msgs::Vector3 linear_vel_{};

    // To track vehicle direction: towards or away from STOPSIGN
    Direction dir_;
    std::mutex mutex_vehicle_dir_;

    // State of vehicle: Running, stopped, etc.
    State state_;
    std::mutex mutex_vehicle_state_;

    bool vehicleAtIntersection_;
};

} // namespace mrs

#endif // VEHICLE_H
