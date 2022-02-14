#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include <vector>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>
#include <string>


double* pose     = new double[7];
double* poseAtt  = new double[7];
double* vel      = new double[6];
double freq_mess = 300;
double eps_m     = 0.0005;
double eps_c     = 0.01;
double eps_d     = 0.5;
static std::string frameR;


  
 struct EulerAngles {
    double roll, pitch, yaw;
 };
 
  void Callback(const geometry_msgs::PoseStamped::ConstPtr& posaStamped)
 {	
  double x    = posaStamped->pose.position.x;
  double y    = posaStamped->pose.position.y;
  double z    = posaStamped->pose.position.z;
  double or_x = posaStamped->pose.orientation.x;
  double or_y = posaStamped->pose.orientation.y;
  double or_z = posaStamped->pose.orientation.z;
  double or_w = posaStamped->pose.orientation.w;
  pose[0]= x;
  pose[1]= y;
  pose[2]= z;
  pose[3]= or_x;
  pose[4]= or_y;
  pose[5]= or_z;
  pose[6]= or_w;
 }
 
 
 
   void CallbackAtt(const geometry_msgs::PoseStamped::ConstPtr& posaStamped)
 {	
  frameR      = posaStamped->header.frame_id;
  double x    = posaStamped->pose.position.x;
  double y    = posaStamped->pose.position.y;
  double z    = posaStamped->pose.position.z;
  double or_x = posaStamped->pose.orientation.x;
  double or_y = posaStamped->pose.orientation.y;
  double or_z = posaStamped->pose.orientation.z;
  double or_w = posaStamped->pose.orientation.w;
  poseAtt[0]= x;
  poseAtt[1]= y;
  poseAtt[2]= z;
  poseAtt[3]= or_x;
  poseAtt[4]= or_y;
  poseAtt[5]= or_z;
  poseAtt[6]= or_w;

  
 }
 
 
 
 class controller {
	public:
		controller(){
			sub  = n.subscribe("/posa", 10, &Callback);
			sub2 = n.subscribe("/pose", 10, &CallbackAtt);
			pub  = n.advertise<geometry_msgs::TwistStamped>("/twist_cmd",100);
			}
		ros::NodeHandle n;
		
		ros::Subscriber sub, sub2;
		ros::Publisher pub;
		
	};
 
 void azzeravel( double vel[], controller ctrl, geometry_msgs::TwistStamped twistmsg )
 {
 	vel[0] = 0;
 	vel[1] = 0;
 	vel[2] = 0;
 	vel[3] = 0;
 	vel[4] = 0;
 	vel[5] = 0;
 	twistmsg.twist.linear.x = vel[0];
 	twistmsg.twist.linear.y = vel[1];
 	twistmsg.twist.linear.z = vel[2];
 	twistmsg.twist.angular.x = vel[3];
  	twistmsg.twist.angular.y = vel[4];
 	twistmsg.twist.angular.z = vel[5];
 	ctrl.pub.publish(twistmsg);
 }

  



void CompVel( double kp, double kp2,  Eigen::Matrix <double,6,1> e)
{
	double v;
	double d = 7;
	double k = kp;
	if(abs(e[0]>eps_d) && abs(e[1]>eps_d) && abs(e[2]>eps_d))
	{ d = 3;}
	else
	{d=7;}
	for(int i = 0; i<d; i++)
	{
	if(i>2)
	{k = kp2;}
	v = -kp * e(i,1);
		if(v>0.1)
		{
			 vel[i] =0.1;
		}
		else if(v<-0.1)
		{
			vel[i] = -0.1;
		}
		else
		{
			vel[i] = v;
		}
	}
}



Eigen::Matrix<double, 6, 1>          compute_pose_error(Eigen::Matrix<double, 4, 4> T_des, Eigen::Matrix<double, 4, 4> T)
{
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


	

	
 

EulerAngles ToEulerAngles(Eigen::Quaterniond q) 
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}








	                                                            //MAIN
int main(int argc, char**argv)
{
  ros::init(argc, argv, "controller3");
  ros::AsyncSpinner spinner(1);
  spinner.start();
 
  
  
  double Kp  = 2;
  double Kp2 = 1.5;
  
  controller ctrl;



  ctrl.sub;
  ctrl.sub2;
 
 
  int count =1;
  geometry_msgs::TwistStamped twistmsg;
  azzeravel(vel, ctrl, twistmsg);
 
  ros::Rate loop_rate(freq_mess);
 
 
 
  //creazione matrice D_des	
  Eigen::Quaterniond q_des;
  q_des.x() = pose[3];
  q_des.y() = pose[4];
  q_des.z() = pose[5];
  q_des.w() = pose[6];
  EulerAngles angoliDes = ToEulerAngles(q_des);
  Eigen::Matrix3d R_des = q_des.toRotationMatrix();	
  Eigen::Matrix4d D_des;
  D_des << R_des(0,0) , R_des(0,1) ,R_des(0,2) , pose[0] , R_des(1,0) , R_des(1,1) , R_des(1,2) , pose[1], R_des(2,0) ,R_des(2,1) ,        R_des(2,2), pose[2], 0, 0, 0, 1; 
   	 
 double x_att;
 double y_att;
 double z_att;
 double or_x_att;
 double or_y_att; 
 double or_z_att;
 double or_w_att;

 
 // calcolo matrice attuale
 Eigen::Quaterniond q_att;
 x_att    = poseAtt[0];
 y_att    = poseAtt[1];
 z_att    = poseAtt[2];
 or_x_att = poseAtt[3];
 or_y_att = poseAtt[4]; 
 or_z_att = poseAtt[5];
 or_w_att = poseAtt[6];
 q_att.x() = or_x_att;
 q_att.y() = or_y_att;
 q_att.z() = or_z_att;
 q_att.w() = or_w_att;
 Eigen::Matrix3d R = q_att.toRotationMatrix();
 Eigen::Matrix4d D_att;
 D_att << R(0,0) , R(0,1) ,R(0,2) , x_att , R(1,0) , R(1,1) , R(1,2) , y_att, R(2,0) ,R(2,1) , R(2,2), z_att, 0, 0, 0, 1; 
 
 //calcolo roll, pitch, yaw attuali
 EulerAngles angoliAttuali = ToEulerAngles(q_att);
	
 //calcolo errore di partenza
 Eigen::Matrix <double,6,1> error;
 error= compute_pose_error(D_des , D_att);
 
 
                      //AVVICINAMENTO
 while (abs(error[0])>eps_m || abs(error[1])>eps_m|| abs(error[2])>eps_m || abs(error[3])>eps_m || abs(error[4])>eps_m || abs(error[5])>eps_m )
  {
                                                              //calcolo della VELOCITÀ
 	//CompVel( Kp, Kp2,error);
        vel[0] = -0.01;
	vel[1] = 0;
 	vel[2]= 0;
 	vel[3]= 0;
 	vel[4]= 0;
 	vel[5]= 0;                                                      //inserimento delle velocità nel messaggio
 	twistmsg.header.frame_id = frameR;
 	twistmsg.twist.linear.x  = vel[0];
	twistmsg.twist.linear.y  = vel[1];
 	twistmsg.twist.linear.z  = vel[2];
 	twistmsg.twist.angular.x = vel[3];
 	twistmsg.twist.angular.y = vel[4];
 	twistmsg.twist.angular.z = vel[5];
 	
 	
 	ROS_INFO("poseAtt[0][%f]",  poseAtt[0]);
 	ROS_INFO("poseAtt[1][%f]",  poseAtt[1]);
 	ROS_INFO("poseAtt[2][%f]",  poseAtt[2]);
 	ROS_INFO("poseAtt[3][%f]",  poseAtt[3]);
 	ROS_INFO("poseAtt[4][%f]",  poseAtt[4]);
 	ROS_INFO("poseAtt[5][%f]",  poseAtt[5]);
 	ROS_INFO("poseAtt[6][%f]",  poseAtt[6]);
 	ROS_INFO("pose[0][%f]",  pose[0]);
 	ROS_INFO("pose[1][%f]",  pose[1]);
 	ROS_INFO("pose[2][%f]",  pose[2]);
 	ROS_INFO("pose[3][%f]",  pose[3]);
 	ROS_INFO("pose[4][%f]",  pose[4]);
 	ROS_INFO("pose[5][%f]",  pose[5]);
 	ROS_INFO("pose[6][%f]",  pose[6]);
 	ROS_INFO("error[0][%f]",  error[0]);
 	ROS_INFO("error[1][%f]",  error[1]);
 	ROS_INFO("error[2][%f]",  error[2]);
 	ROS_INFO("error[3][%f]",  error[3]);
 	ROS_INFO("error[4][%f]",  error[4]);
 	ROS_INFO("error[5][%f]",  error[5]);
 	ROS_INFO("error[6][%f]",  error[6]);
 
	
    	
 	ctrl.pub.publish(twistmsg);
	ros::spinOnce();
 	loop_rate.sleep();
 	++count;
 	
 	                                      //creazione matrice D_att	
 	x_att =    poseAtt[0];
 	y_att =    poseAtt[1];
 	z_att =    poseAtt[2];
 	or_x_att = poseAtt[3];
 	or_y_att = poseAtt[4];
 	or_z_att = poseAtt[5];
 	or_w_att = poseAtt[6];
 	 					                 
 	q_att.x() = or_x_att;
 	q_att.y() = or_y_att;
 	q_att.z() = or_z_att;
 	q_att.w() = or_w_att;
 	R = q_att.toRotationMatrix();
 	
 	D_att << R(0,0) , R(0,1) ,R(0,2) , x_att , R(1,0) , R(1,1) , R(1,2) , y_att, R(2,0) ,R(2,1) , R(2,2), z_att, 0, 0, 0, 1; 
	                                                      //calcolo rpy attuali dall'orientamento attuale
 	angoliAttuali = ToEulerAngles(q_att);
   	                                                      //calcolo matrice  di errore
 	error= compute_pose_error(D_des , D_att);
  }
  
  //azzeramento velocità
  azzeravel(vel, ctrl, twistmsg);
 

  
 return 0;
}





