#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose.h>
#include "ros/ros.h"
#include "pumpkin_carving/cart_path.h"

Eigen::Affine3d uFrame;
moveit::planning_interface::MoveGroupInterface *move_group;
Eigen::Affine3d worldFrame;

geometry_msgs::Pose calcUFramePose(const geometry_msgs::Pose &poseIn){
  geometry_msgs::Pose shiftedPose;
  Eigen::Affine3d pointMTX;
  tf::poseMsgToEigen (poseIn, pointMTX);
  Eigen::Affine3d worldInPlan = worldFrame * uFrame;
  tf::poseEigenToMsg	(worldInPlan * pointMTX, shiftedPose);

  return shiftedPose;
}

bool follow_path(pumpkin_carving::cart_path::Request &req, pumpkin_carving::cart_path::Response &res)
{
  ROS_INFO("Received new path");

  std::vector<geometry_msgs::Pose> waypoints;

  for (unsigned i=0; i < req.path.poses.size(); i++) {
    waypoints.push_back(calcUFramePose(req.path.poses[i]));
  }

  move_group->setMaxVelocityScalingFactor(0.5);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.001;
  double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO("Computed Pass <%.2f%%>", fraction * 100.0);

  if(fraction > 0.99)
  {
    moveit::planning_interface::MoveGroupInterface::Plan cPlan;
    cPlan.trajectory_ = trajectory;

    ROS_INFO("Moving");

    if(move_group->execute(cPlan) == moveit_msgs::MoveItErrorCodes::SUCCESS){
       ROS_INFO("Complete");
     }
     else{
       res.status = -2;
       ROS_WARN("Path Execution Failure");
       return true;
     }

    ROS_INFO("Path Complete");
  }
  else
  {
    res.status = -1;
    ROS_WARN("Path Planning Incomplete");
    return true;
  }

  res.status = 0;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_mover");
  ros::NodeHandle node_handle("~");

  ROS_INFO("Initializing uFrame");

  /////////////////////////////////////////////////////////////
  geometry_msgs::Pose uPose;
  uPose.position.x = -0.6;
  uPose.position.y = 0.0;
  uPose.position.z = 0.2;
  uPose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-1.5708, 0.0, 3.14159);
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

  ros::ServiceServer service = node_handle.advertiseService("follow_path", follow_path);

  ros::spin();
  return 0;
}
