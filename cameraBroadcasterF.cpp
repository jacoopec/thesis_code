#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>



//trasmette la posizione relativa tra la camera e ilbase_link




int main(int argc, char**argv)
  {
  ros::init(argc, argv, "cameraBroadcaster");
  ros::AsyncSpinner spinner(1);
  ros::NodeHandle n;
  
  static tf2_ros::TransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  
  static_transformStamped.header.frame_id = "ee_link";
  static_transformStamped.child_frame_id  = "Camera";
  static_transformStamped.transform.translation.x = 0;
  static_transformStamped.transform.translation.y = 0;
  static_transformStamped.transform.translation.z = 0.4;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
 
  	
  ros::Rate rate(10.0);
  
  while (n.ok())
  {
    static_transformStamped.header.stamp = ros::Time::now();
    static_broadcaster.sendTransform(static_transformStamped);
    rate.sleep();
  }
}


	/*                                                            //MAIN
  int main(int argc, char**argv)
  {
  ros::init(argc, argv, "ctrl");
  ros::AsyncSpinner spinner(1);
  spinner.start();
 
  static const std::string PLANNING_GROUP = "manipulator";
  
  tf2_ros::TransformBroadcaster ggg;
  geometry_msgs::TransformStamped transformStamped;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ros::NodeHandle private_node_handle("~");
  controller ctrl;
  	
  moveit::planning_interface::MoveGroupInterface        move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface    planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =  move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  
  
    
  transformStamped.header.frame_id = "ee_link";
  transformStamped.child_frame_id  = "camera";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.5;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  ros::Rate rate(10.0);
  while (1){
    transformStamped = tfBuffer.lookupTransform("camera", "world", ros::Time(0));
    transformStamped.header.stamp = ros::Time::now();
    ggg.sendTransform(transformStamped);
    rate.sleep();
  }


  
  
 return 0;
}


	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
	ROS_INFO_STREAM(q); 
	q.normalize();
	ROS utilizza due tipi di dati per i quaternionimsg e tf per convertirli tra di loro si usano i metodi della classe tf2_geometry_msgs
	tf2::Quaternion quat_tf;
	geometry_msgs::Quaternion quat_msg = ...;

	tf2::convert(quat_msg , quat_tf);
	// or
	tf2::fromMsg(quat_msg, quat_tf);
	// or for the other conversion direction
	quat_msg = tf2::toMsg(quat_tf);
	applicare una rotazione con i quaternioni ad una posa: si moltiplica il quaternione della posa per il quaternione ella rotazione desiderata.
	tf2::Quaternion q_orig, q_rot, q_new;
	tf2::convert(commanded_pose.pose.orientation , q_orig);
	
	double r=3.14159, p=0, y=0;  // Rotate the previous pose by 180* about X
	q_rot.setRPY(r, p, y);

	q_new = q_rot*q_orig;  // Calculate the new orientation
	q_new.normalize();

	// Stuff the new rotation back into the pose. This requires conversion into a msg type
	tf2::convert(q_new, commanded_pose.pose.orientation);
*/



