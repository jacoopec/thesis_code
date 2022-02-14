#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "message_filters/subscriber.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//questo nodo trasforma la posizione dell'occhio dal frame della camera a quello del world, lo visualizza a schermo e lo pubblica

class posTransformed
{
public:
  posTransformed() :
    tf2_(buffer_),  target_frame_("world"),
    tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
  {
    point_sub_.subscribe(n_, "eyePositionInCamera", 10);       //iscrizione al topic che pubblica la pos dell'occhio nel frame camera
    eyePosInWorld_pub  = n_.advertise<geometry_msgs::PointStamped>("eyePosInWorld", 100); //pub. posizione occhio nel world frame
    tf2_filter_.registerCallback( boost::bind(&posTransformed::msgCallback, this, _1) );
  }



  void msgCallback(const geometry_msgs::PointStampedConstPtr& point_ptr) 
  {
    geometry_msgs::PointStamped point_out;
    geometry_msgs::PointStamped eyePosInWorld;
    eyePosInWorld.header.frame_id = "world";
    
    try 
    {
      buffer_.transform(*point_ptr, eyePosInWorld, target_frame_);
      
      ROS_INFO("eyeCenter in the frame world: (x:%f y:%f z:%f)\n", eyePosInWorld.point.x, eyePosInWorld.point.y,eyePosInWorld.point.z);
      eyePosInWorld_pub.publish(eyePosInWorld);
             
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
  }


private:
  std::string                                              target_frame_;
  tf2_ros::Buffer                                          buffer_;
  tf2_ros::TransformListener                               tf2_;
  ros::NodeHandle                                          n_;
  message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
  tf2_ros::MessageFilter<geometry_msgs::PointStamped>      tf2_filter_;
  ros::Publisher                                           eyePosInWorld_pub;

};








int main(int argc, char ** argv)
{
  ros::init(argc, argv, "transformListener"); //Init ROS
  posTransformed pd; //Construct class
  ros::spin(); // Run until interupted 
  return 0;
};
