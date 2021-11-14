#include <string>
#include "mobicar_ros_controller.h"

int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  std::string node_name = "mobicar_ctrl_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("my_pub_vel");

  // Init a MobicarRosController instance.
  std::shared_ptr<mrs::MobicarRosController> mrc = std::make_shared<mrs::MobicarRosController>(nh);
  mrc->init();

  // Spin from two threads using AsyncSpinner (does not block the calling thread, eg. main thread) 
  ros::AsyncSpinner spinner(0);
  ROS_INFO_STREAM("Main loop in thread:" << boost::this_thread::get_id());
  spinner.start();
  
  ros::waitForShutdown();

  return 0;
}
