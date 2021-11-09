#include <vehicle.h>
#include <std_msgs/Float32.h>

// mrs - mobicarroscontroller
namespace mrs {

Vehicle::Vehicle(const ros::NodeHandle& nh,
                 const std::string& model_name,
                 const int& model_idx)
    : nh_(nh),
      model_name_(model_name),
      model_idx_(model_idx),
      dir_(Vehicle::Direction::DIR_NONE),
      state_(Vehicle::State::STATE_STOPPED),
      vehicleAtIntersection_(false)

{
  // Not Implemented
}

Vehicle::~Vehicle()
{
  // Not Implemented
}

void Vehicle::init()
{
  // Create publisher object for setting mobicar velocity
  pub_ = nh_.advertise<std_msgs::Float32>("/" + this->model_name_ + "/vel_cmd", 2);
}

// Publish desired velocity to a Mobicar
void Vehicle::pubVelocity(const double& vel)
{
  // Create a a std_msgs::Float32 message and set the desired velocity
  std_msgs::Float32 msg;
 
  msg.data = vel;

  // Send the message
  pub_.publish(msg);
}

// Store Vehicle's current position
void Vehicle::storeVehiclePosition(const geometry_msgs::Point& pos)
{
  // {pos.x, pos.y, pos.z}, unit: cm
  pos_ = pos;
}

// Retrieve Vehicle's current position
const geometry_msgs::Point& Vehicle::retrieveVehiclePosition()
{
  // {pos.x, pos.y, pos.z}, unit: cm
  return pos_;
}

double Vehicle::calculateDistance()
{
  return sqrt((pos_.x - 0.0) * (pos_.x - 0.0) + (pos_.y - 0.0) * (pos_.y - 0.0));
}

// Store Vehicle's current linear velocity (this is not the joint's velocity, but the chassis's velocity)
void Vehicle::storeVehicleLinearVelocity(const geometry_msgs::Vector3& linear_vel)
{
  // unit: cm/s
  linear_vel_ = linear_vel;
}

// Retrieve Vehicle's current linear velocity (this is not the joint's velocity, but the chassis's velocity)
const geometry_msgs::Vector3& Vehicle::getVehicleLinearVelocity()
{
  // unit: cm/s
  return linear_vel_;
}

// Calculate linear velocity
double Vehicle::calculateVelocity()
{
  return sqrt(linear_vel_.x * linear_vel_.x + linear_vel_.y * linear_vel_.y);
}

// Set the direction of the vehicle
void Vehicle::setVehicleDirection(Direction dir)
{
  std::lock_guard<std::mutex> lck(mutex_vehicle_dir_);
  dir_ = dir;
}

// Get the direction of the vehicle
Vehicle::Direction Vehicle::getVehicleDirection()
{
  std::lock_guard<std::mutex> lck(mutex_vehicle_dir_);
  return dir_;
}

// Set the state of the vehicle
void Vehicle::setVehicleState(State state)
{
  std::lock_guard<std::mutex> lck(mutex_vehicle_state_);
  state_ = state;
}

// Get the state of the vehicle
Vehicle::State Vehicle::getVehicleState()
{
  std::lock_guard<std::mutex> lck(mutex_vehicle_state_);
  return state_;
}

void Vehicle::setVehicleAtIntersection(bool intersection)
{
  vehicleAtIntersection_ = intersection;
}

bool Vehicle::getVehicleAtIntersection()
{
  return vehicleAtIntersection_;
}

} // namespace mrs


