#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

geometry_msgs::PoseStamped actual_pose;
geometry_msgs::PoseStamped des_pose;
geometry_msgs::PoseStamped inj_pose;
bool actual_pose_flag;
bool des_pose_flag;
double freq_mess = 500;
double eps_m     = 0.001;                      //tolleranze
double eps_c     = 0.01;
double eps_d     = 0.5;
double max_e     = 1;

void actual_pose_callback(const geometry_msgs::PoseStamped msg){actual_pose = msg; 	actual_pose_flag = true;}

void inj_point_callback(const geometry_msgs::PoseStamped injMsg)
{
	inj_pose.pose.position.x = injMsg.pose.position.x; 
	inj_pose.pose.position.y = injMsg.pose.position.y; 
	inj_pose.pose.position.z = injMsg.pose.position.z; 
	inj_pose.pose.orientation.x = des_pose.orientation.x; 
	inj_pose.pose.orientation.y = des_pose.orientation.y;
	inj_pose.pose.orientation.z = des_pose.orientation.z; 
	inj_pose.pose.orientation.w = des_pose.orientation.w; 		
	inj_pose_flag = true;
}

void des_pose_callback(const geometry_msgs::PoseStamped desMsg)
{	des_pose = desMsg;
	des_pose_flag = true;
}

Eigen::Matrix<double, 6, 1> compute_pose_error(Eigen::Matrix<double, 4, 4> T_des, Eigen::Matrix<double, 4, 4> T){

	Eigen::Matrix<double, 6, 1> err;

	err.block<3,1>(0,0) = T.block<3,1>(0,3) - T_des.block<3,1>(0,3);

	Eigen::Quaterniond orientation_quat_des = Eigen::Quaterniond(T_des.block<3,3>(0,0));
	Eigen::Quaterniond orientation_quat = Eigen::Quaterniond(T.block<3,3>(0,0));

	if (orientation_quat_des.coeffs().dot(orientation_quat.coeffs()) < 0.0) {
		orientation_quat.coeffs() << -orientation_quat.coeffs();
	}

	Eigen::Quaterniond orientation_quat_error(orientation_quat.inverse() * orientation_quat_des);

	err.block<3,1>(3,0) << orientation_quat_error.x(), orientation_quat_error.y(), orientation_quat_error.z();
	err.block<3,1>(3,0) << - T.block<3,3>(0,0) * err.block<3,1>(3,0);

	return err;

}

Eigen::Matrix<double, 4, 4> pose2eigen (geometry_msgs::PoseStamped pose){

	Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();

	T(0,3) = pose.pose.position.x;
	T(1,3) = pose.pose.position.y;
	T(2,3) = pose.pose.position.z;

	Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);

	T.block<3,3>(0,0) = q.normalized().toRotationMatrix();

	return T;

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "speed_controller");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	
	ros::Subscriber des_pose_sub    = n.subscribe("preInjectionPose", 1, &des_pose_callback);
	ros::Subscriber inj_pose_sub   = n.subscribe("injectionPose", 1, &inj_pose_callback);
	ros::Subscriber actual_pose_sub = n.subscribe("pose", 1, &actual_pose_callback);
	ros::Publisher  twist_cmd_pub   = n.advertise<geometry_msgs::TwistStamped>("twist_cmd", 1);

	Eigen::Matrix<double, 4, 4> T_des, T;

	Eigen::Matrix<double, 6, 1> error, velocity;



	geometry_msgs::TwistStamped twist_cmd;



	actual_pose_flag = false;
	des_pose_flag    = false;
	
	std::cout << "WAITING ROBOT POSE" << std::endl;
	while (actual_pose_flag == false || des_pose_flag == false){ros::spinOnce();};
	std::cout << "CONTROLLER STARTED" << std::endl;
	
	T_des = pose2eigen(des_pose);
	T     = pose2eigen(actual_pose);
	error = - compute_pose_error(T_des, T);
	
	while (((abs(error[0])>eps_m) || (abs(error[1])>eps_m ) || (abs(error[2])>eps_m ) || (abs(error[3])>eps_m ) || (abs(error[4])>eps_m)  || (abs(error[5])>eps_m ) ))
	{
		T     = pose2eigen(actual_pose);
		error = - compute_pose_error(T_des, T);

		velocity = 2.0 * error;
		for(int i=0; i<6; i++)
		{
			if((velocity(i,0) > 0.01))
			{
				velocity(i,0) =  0.01;
			}
			else if((velocity(i,0) < -0.01))
			{
				velocity(i,0) =  -0.01;
			}
		}
	
		
		twist_cmd.twist.linear.x = velocity(0,0);
		twist_cmd.twist.linear.y = velocity(1,0);
		twist_cmd.twist.linear.z = velocity(2,0);

		twist_cmd.twist.angular.x = velocity(3,0);
		twist_cmd.twist.angular.y = velocity(4,0);
		twist_cmd.twist.angular.z = velocity(5,0);
		loop_rate.sleep();
		twist_cmd_pub.publish(twist_cmd);
		ros::spinOnce();
		loop_rate.sleep();
	
	}
	
	
	T_des = pose2eigen(inj_pose);
	T     = pose2eigen(actual_pose);
	error = - compute_pose_error(T_des, T);
  

  while ((abs(error(0,0))>eps_c) || (abs(error(1,0))>eps_c ) || (abs(error(2,0))>eps_c )) 
  {

	T     = pose2eigen(actual_pose);
	error = - compute_pose_error(T_des, T);
 	
	double h[3];
 	double k[3];
 	
	for(int y = 0; y<3; y++)
	{
		if(error(y,0)>0)
		{ k[y] = -1;}
		else
		{ k[y] = 1;}
	}
	
 	double m = abs(error[0]) + abs(error[1]) + abs(error[2]);
	h[0]= abs(error[0])/m;
	h[1]= abs(error[1])/m;
	h[2]= abs(error[2])/m;
 	double v = sqrt(error[0]*error[0] + error[1]*error[1] + error[2]*error[2]);
 	if(v>0.1)
	{v=0.1;}
	velocity = 2.0 * error;
	velocity(0,0) = h[0] * v * k[0];
	velocity(1,0) = h[1] * v * k[1];
	velocity(2,0) = h[2] * v * k[2];
	velocity(3,0) = 0;
	velocity(4,0) = 0;
	velocity(5,0) = 0;
  
		
	twist_cmd.twist.linear.x  = velocity(0,0);
	twist_cmd.twist.linear.y  = velocity(1,0);
	twist_cmd.twist.linear.z  = velocity(2,0);
	twist_cmd.twist.angular.x = velocity(3,0);
	twist_cmd.twist.angular.y = velocity(4,0);
	twist_cmd.twist.angular.z = velocity(5,0);
	loop_rate.sleep();
	twist_cmd_pub.publish(twist_cmd);
	ros::spinOnce();
	loop_rate.sleep();
 }
 
 
	T_des = pose2eigen(des_pose);
	T     = pose2eigen(actual_pose);
	error = - compute_pose_error(T_des, T);
  

  while ((abs(error(0,0))>eps_c) || (abs(error(1,0))>eps_c ) || (abs(error(2,0))>eps_c )) 
  {

	T     = pose2eigen(actual_pose);
	error = - compute_pose_error(T_des, T);
 	
	double h[3];
 	double k[3];
 	
	for(int y = 0; y<3; y++)
	{
		if(error(y,0)>0)
		{ k[y] = -1;}
		else
		{ k[y] = 1;}
	}
	
 	double m = abs(error[0]) + abs(error[1]) + abs(error[2]);
	h[0]= abs(error[0])/m;
	h[1]= abs(error[1])/m;
	h[2]= abs(error[2])/m;
 	double v = sqrt(error[0]*error[0] + error[1]*error[1] + error[2]*error[2]);
 	if(v>0.1)
	{v=0.1;}
	velocity = 2.0 * error;
	velocity(0,0) = h[0] * v * k[0];
	velocity(1,0) = h[1] * v * k[1];
	velocity(2,0) = h[2] * v * k[2];
	velocity(3,0) = 0;
	velocity(4,0) = 0;
	velocity(5,0) = 0;
  
		
	twist_cmd.twist.linear.x  = velocity(0,0);
	twist_cmd.twist.linear.y  = velocity(1,0);
	twist_cmd.twist.linear.z  = velocity(2,0);
	twist_cmd.twist.angular.x = velocity(3,0);
	twist_cmd.twist.angular.y = velocity(4,0);
	twist_cmd.twist.angular.z = velocity(5,0);
	loop_rate.sleep();
	twist_cmd_pub.publish(twist_cmd);
	ros::spinOnce();
	loop_rate.sleep();
 }

	return 0;
}


