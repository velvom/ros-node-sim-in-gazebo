#include "mobicar_plugin.h"
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Rand.hh>
#include <ros/console.h> // logger, view in rqt_console tool

using namespace gazebo;

// Register this plugin with Gazebo simulator
// so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(MobicarPlugin)

/// \brief Destructor
MobicarPlugin::~MobicarPlugin()
{
  //std::cout << "Dectroy plugin" << std::endl;
  this->updateConnection_.reset();
  // Finalize the controller
  this->rosNode_->shutdown();
  this->rosCbQueue_.clear();
  this->rosCbQueue_.disable();
  this->rosCbQueueThread_.join();

}

/// \brief The load function is called by Gazebo when the plugin is
/// inserted into simulation
/// \param[in] _model A pointer to the model that this plugin is
/// attached to.
/// \param[in] _sdf A pointer to the plugin's SDF element.
void MobicarPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Safety check
  if (_model->GetJointCount() < 4)
  {
    ROS_FATAL("Invalid joints count, Mobicar plugin not loaded");
    return;
  }
  
  // Store the model and world pointers for convenience.
  this->model_ = _model;
  this->world_ = this->model_->GetWorld();

  // Model Name: "nava_mobicar"
  this->name_ = this->model_->GetName();
  ROS_INFO_STREAM("The Mobicar plugin is attached to model[" <<
                  this->name_ << "] in the world[" << this->world_->Name() << "]");

  // Get the position of the model
  this->lastPose_ = this->model_->WorldPose();
  ignition::math::Vector3<double> position =  this->lastPose_.Pos();
  double pos[3] = {position.X(), position.Y(), position.Z()};

  // This should happen before applying velocity below
  this->ApplyPIDControl();

  // Check that the velocity element exists, then read the value
  // Note: Set the velocity element to 0 in model.sdf when using
  // a stand-alone ROS mode to set velocity
  double velocity = 0.0;
  if (_sdf->HasElement("velocity")) {
    velocity = _sdf->Get<double>("velocity");
    velocity *= ignition::math::Rand::DblUniform(0.5, 1.5);
  } 
  
  if (velocity > 0.0) {
    this->ApplyJointsVelocity(velocity);
  }

  // ==========
  // ROS Stuff
  // ==========
  // Initialize ROS if it has not already been initialized.
  // ROS must be initialized if gazebo is launced by
  // "rosrun gazebo_ros gazebo --verbose ../mobicar.world".
  // This loads the Gazebo system plugin 'libgazebo_ros_api_plugin.so'
  // in the gazebo_ros package.
  if (!ros::isInitialized()) {
    ROS_INFO_STREAM("A ROS node for Gazebo has not been initialized,"
                    << "unable to load plugin. Load the Gazebo system plugin "
                    << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
    return;
  }

  // Get a ROS node handle and set its namespace to "/my_sub_vel"
  this->rosNode_ = std::make_unique<ros::NodeHandle>("my_sub_vel");

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<std_msgs::Float32>(
        "/" + this->name_ + "/vel_cmd",
        2,
        boost::bind(&MobicarPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosCbQueue_);
  this->rosSub_ = this->rosNode_->subscribe(so);

  // To track Gazebo's simulation update time
  this->last_pose_publish_time_ = this->world_->SimTime();

  // Spin up the queue helper thread.
  this->rosCbQueueThread_ = std::thread(std::bind(&MobicarPlugin::ProcessRosMsgs, this));
}

/// \brief Set the target velocity to a wheel-joint
/// \param[in] _vel New target velocity
void MobicarPlugin::SetJointVelocity(const physics::JointPtr& _joint, const double& _vel)
{
  // Set the joint's target velocity.
  this->model_->GetJointController()->SetVelocityTarget(
    _joint->GetScopedName(), _vel);
}

/// \brief Apply target velocity to all joints
void MobicarPlugin::ApplyJointsVelocity(const double& _vel)
{
  for (const auto& kv: joints_pids_) {
    this->SetJointVelocity(kv.first, _vel);
  }
}

/// \brief Apply PID Control to all joints
void MobicarPlugin::ApplyPIDControl()
{
  for (auto& j: this->model_->GetJoints()) {
    // Setup a P-controller, with a gain of 0.1.
    common::PID temp_pid = common::PID(0.1, 0, 0);

    // Add {jointPtr : PID controller} to the map 
    this->joints_pids_[j] = temp_pid;

    // Apply the P-controller to the joint.
    this->model_->GetJointController()->SetVelocityPID(
      j->GetScopedName(), temp_pid);
  }
}

/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Mobicar.
void MobicarPlugin::OnRosMsg(const std_msgs::Float32ConstPtr& _msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  this->last_msg_received_time_ = ros::Time::now();

  for (const auto& kv: joints_pids_) {
    this->SetJointVelocity(kv.first, _msg->data);
  }
}

/// \brief ROS helper function that processes messages
void MobicarPlugin::ProcessRosMsgs()
{
  static const double timeout = 0.01;
  while (this->rosNode_->ok()) {
    this->rosCbQueue_.callAvailable(ros::WallDuration(timeout));
  }
}

/// \brief Called by the world update start event
void MobicarPlugin::OnUpdate()
{
  common::Time current_time = this->world_->SimTime();
  last_pose_publish_time_ = current_time;

  // Get the position of the model
  this->lastPose_ = this->model_->WorldPose();
  ignition::math::Vector3<double> position = this->lastPose_.Pos();
  double pos[3] = {position.X(), position.Y(), position.Z()};
}
