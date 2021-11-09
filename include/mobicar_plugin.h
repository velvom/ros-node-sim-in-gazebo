#ifndef _MOBICAR_PLUGIN_HH_
#define _MOBICAR_PLUGIN_HH_

// Model Control Plugin
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <sdf/sdf.hh>
#include <vector>
#include <map>

// Model Control Plugin to ROS Middleware Interaction
#include <thread>
#include <ros/ros.h>
#include <signal.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/advertise_options.h>
#include <std_msgs/Float32.h>

namespace gazebo
{
  
/// \brief A plugin to control my mobicar.
class MobicarPlugin : public ModelPlugin
{
public:
  /// \brief Constructor
  MobicarPlugin() {}

  /// \brief Destructor
  virtual ~MobicarPlugin();

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Set the target velocity to a wheel-joint
  /// \param[in] _vel New target velocity
  void SetJointVelocity(const physics::JointPtr& _joint, const double& _vel);

  /// \brief Apply target velocity to all joints
  void ApplyJointsVelocity(const double& _vel);  

  /// \brief Apply PID control to all joints
  void ApplyPIDControl();

  /// \brief Handle an incoming message from ROS
  /// \param[in] _msg A float value that is used to set the velocity
  /// of the Mobicar.
  void OnRosMsg(const std_msgs::Float32ConstPtr& _msg);

  /// \brief Called by the world update start event
  void OnUpdate();

private:
  /// \brief ROS helper function that processes incoming messages
  void ProcessRosMsgs();

  /// \brief Pointer to the car model
  physics::ModelPtr model_;

  /// \brief Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  /// \brief Gazebo world pointer.
  physics::WorldPtr world_;

  /// \brief Link that holds the joints/wheels
  physics::LinkPtr carlink_;

  /// \brief The car model name
  std::string name_;

  // /// \brief Map containing {Pointer to each joint : PID controller for each joint}
  std::map<physics::JointPtr, common::PID> joints_pids_;

  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosNode_;

  /// \brief A ROS subscriber
  ros::Subscriber rosSub_;

  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosCbQueue_;

  /// \brief A thread the keeps running the rosQueue
  std::thread rosCbQueueThread_;

  /// \brief Last time ROS msg received
  ros::Time last_msg_received_time_;

  /// \brief Track car (chassis) position
  ignition::math::Pose3d lastPose_;

  /// \brief Last time Plugin published position to ROS
  common::Time last_pose_publish_time_;

  std::mutex mutex_;
};

}
#endif // _MOBICAR_PLUGIN_HH_
