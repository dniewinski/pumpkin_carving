#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

Eigen::Affine3d uFrame;
moveit::planning_interface::MoveGroupInterface *move_group;
Eigen::Affine3d worldFrame;

bool follow_path(pumpkin_carving::cart_path::Request &req, pumpkin_carving::cart_path::Response &res)
{
  ROS_INFO("Received new path");

  geometry_msgs::Pose start_pos = move_group->getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pos);

  for (unsigned i=0; i < req.path.poses.size(); i++) {
    waypoints.push_back(req.path.poses[i]);
  }

  move_group->setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.001;
  double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO("Computed Pass <%.2f%%>", fraction * 100.0);

  moveit::planning_interface::MoveGroupInterface::Plan cPlan;
  cPlan.trajectory_ = trajectory;

  ROS_INFO("Moving");

  if(move_group->execute(cPlan) == moveit_msgs::MoveItErrorCodes::SUCCESS){
     ROS_INFO("Complete");
   }

  ROS_INFO("Path Complete")
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_mover");
  ros::NodeHandle node_handle("~");

  ROS_INFO("Initializing uFrame");

  /////////////////////////////////////////////////////////////
  geometry_msgs::Pose uPose;
  uPose.position.x = 0.0;
  uPose.position.y = 0.0;
  uPose.position.z = 0.0;
  uPose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
  tf::poseMsgToEigen (uPose, uFrame);
  /////////////////////////////////////////////////////////////

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string group_name, robot_world_name;
  node_handle.param<std::string>("group_name", group_name, "manipulator");
  node_handle.param<std::string>("robot_world_name", robot_world_name, "/base");

  move_group = new moveit::planning_interface::MoveGroupInterface(group_name);

  ROS_INFO("Planning frame: %s", move_group->getPlanningFrame().c_str());
  ROS_INFO("Tool frame: %s", move_group->getEndEffectorLink().c_str());
  ROS_INFO("move_group Initialized");

  tf::TransformListener listener;
  tf::StampedTransform transform;
  bool arm_world_found = false;

  ROS_INFO("Initializing robot world");
  while(ros::ok() && !arm_world_found){
    try{
      listener.lookupTransform(move_group->getPlanningFrame(), robot_world_name, ros::Time(0), transform);
      tf::transformTFToEigen(transform, worldFrame);
      arm_world_found = true;
    }
    catch (tf::TransformException ex){
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  ROS_INFO("World Initialized");

  ros::spin();
  return 0;
}
