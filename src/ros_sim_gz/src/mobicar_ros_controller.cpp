#include <string>
#include <mobicar_ros_controller.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <stop_sign.h>
#include <vehicle.h>

// mrs - mobicarroscontroller
namespace mrs {

const double range_from_intersection = 2.5;

MobicarRosController::MobicarRosController(const ros::NodeHandle& nh)
    : nh_(nh),
      sub_callback_count_(0)
{
  ss_ = std::make_shared<StopSign>();
  threads_ = std::make_shared<std::vector<std::thread>>();
}

MobicarRosController::~MobicarRosController()
{
  // Set up thread barrier before this object is destroyed
  std::for_each(threads_->begin(), threads_->end(), [](std::thread& t) {
      t.join();
  });
}

void MobicarRosController::init()
{
  // This will wait until Gazebo node is up and running.
  // Should be called before subscribing to MobicarRosController::modelstateCallback.
  this->createVehicles();

  // Initialize publishers from this ROS node
  for (const auto& p: vehicles_) {
    p->init();
  }

  // Sleep for 300ms for Vehicle publishers from this ROS node
  // to get hooked to subscribers at Gazebo node
  ros::Duration(0.3).sleep(); // sleep for 300 ms

  // To listen to gazebo node's model states. 
  // Extract each mobicar's position from model states
  sub_ = nh_.subscribe("/gazebo/model_states",
                        1,
                        &MobicarRosController::modelstateCallback,
                        this);

  // Init StopSign and get it to run in a seprate thread
  ss_->setRosController(get_shared_this());
  ss_->init(threads_);

  // Start the vehicles running, pubs should be valid at this point
  this->startVehicles();

  // These are for throttling ModelStates topic messages
  this->prev_time_ = ros::Time::now();
  this->period_ = ros::Duration(1.0 / 1000); // 1 ms

  ROS_INFO_STREAM("MobicarRosController::init() successful in thread "
                  << boost::this_thread::get_id());
}

// Store the model names and their indices of interest to us
// This is a one-time service call at MobicarRosController::init()
void MobicarRosController::createVehicles()
{
  ROS_DEBUG_STREAM("MobicarRosController::createVehicles(), Calling get_world_properties in thread "
                  << boost::this_thread::get_id());
  
  ros::ServiceClient world_properties_client = nh_.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
  world_properties_client.waitForExistence();
  
  // Send the request to the world properties server
  gazebo_msgs::GetWorldProperties world_properties_srv_msg;
  world_properties_client.call(world_properties_srv_msg);

  //make sure service call was successful
  bool result = world_properties_srv_msg.response.success;
  if (result) {
    int count = 0;
    // Store the model names and their indices in a map
    for (const auto& model_name: world_properties_srv_msg.response.model_names) {
      if (model_name == "mobicar1" || 
          model_name == "mobicar2" ||
          model_name == "mobicar3" ||
          model_name == "mobicar4") {
        //model_indices_[model_name] = count;
        vehicles_.push_back(std::make_shared<Vehicle>(nh_, model_name, count));
      }  
      count++;    
    }
  } else {
    ROS_WARN("service call to get_world_properties failed!");
  }
}

// Called at ROS node init to start the cars
void MobicarRosController::startVehicles()
{
  // Start the vehicles running
  double v = 20.0;
  for (const auto& p: vehicles_) {
    p->pubVelocity(v);
    p->setVehicleDirection(mrs::Vehicle::Direction::DIR_TOWARDS_STOPSIGN);
    p->setVehicleState(mrs::Vehicle::State::STATE_INMOTION);
    v -= 0.5;
  }
}

// Callback method handling incoming model state messages
void MobicarRosController::modelstateCallback(const gazebo_msgs::ModelStates::ConstPtr& modelstate)
{
  ROS_DEBUG_STREAM("subscriber callback "
                  << sub_callback_count_ << ", in thread:"
                  << boost::this_thread::get_id());
  sub_callback_count_++;

  // Throttle ModelStates topic messages
  const ros::Time curr_time  = ros::Time::now();
  const ros::Duration elapsed = curr_time - this->prev_time_;
  ros::Duration remaining = this->period_ - elapsed;
  if (remaining.toSec() > 0)
  {
    remaining.sleep();
    ROS_DEBUG_STREAM("MobicarRosController::modelstateCallback, Throttle topic msgs " << remaining);
    return;
  }

  geometry_msgs::Pose mobicar_pose{};
  geometry_msgs::Twist mobicar_twist{};
  double d{0.0};
  double v{0.0};
  double brake_dist_to_stopsign = 14.0;
  const double brake_dist_to_deadend = 5.5;

  for (const auto& p: vehicles_) {
    // Store vehicle position and linear velocity data
    mobicar_pose = modelstate->pose[p->getModelIndex()];
    mobicar_twist = modelstate->twist[p->getModelIndex()];
    p->storeVehiclePosition(mobicar_pose.position);
    p->storeVehicleLinearVelocity(mobicar_twist.linear);

    // Handle STOPPING the vehicle
    d = p->calculateDistance();
    if (p->getVehicleState() == mrs::Vehicle::State::STATE_INMOTION) {
      if ( (p->getVehicleDirection() == mrs::Vehicle::Direction::DIR_TOWARDS_STOPSIGN && d < brake_dist_to_stopsign) ||
           (p->getVehicleDirection() == mrs::Vehicle::Direction::DIR_AWAY_STOPSIGN && d > brake_dist_to_deadend) ) {
        p->pubVelocity(0.0);
        p->setVehicleState(mrs::Vehicle::State::STATE_STOPPED);
      }
    }

    // Range vehicle at intersection
    // Queue vehicles to be scheduled to take turn at stop sign
    if (d <= range_from_intersection &&
        p->getVehicleState() == mrs::Vehicle::State::STATE_STOPPED &&
        p->getVehicleAtIntersection() == false) {
      p->setVehicleAtIntersection(true);
      ss_->addVehicleToQueue(p->getModelName());
      ROS_DEBUG_STREAM("MobicarRosController::modelstateCallback, Adding vehicle " 
                      << p->getModelName() << " to Q" << " dist = " << d);
    }

    brake_dist_to_stopsign -= 0.5;
  }

  this->prev_time_ = curr_time;
}

// // This is to let the "stopped" vehicles thru intersection
void MobicarRosController::scheduleVehicle(std::string vid)
{
  ROS_INFO_STREAM("MobicarRosController::scheduleVehicle "
                  << vid << ", in thread:"
                  << boost::this_thread::get_id());
  for (const auto& p: vehicles_) {
    if (vid == p->getModelName()) {
      ss_->setCurrentPhase(true);
      p->setVehicleDirection(mrs::Vehicle::Direction::DIR_AWAY_STOPSIGN);
      p->setVehicleState(mrs::Vehicle::State::STATE_INMOTION);
      p->pubVelocity(16.0);

      // Wait until the vehicle leaves the intersection
      while (p->calculateDistance() <= range_from_intersection) {
        ros::Duration(0.1).sleep(); // sleep for 100 ms
      }

      ss_->setCurrentPhase(false);
      return;
    }
  }
}

} // namespace mrs
