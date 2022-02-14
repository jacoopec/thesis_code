#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <geometry_msgs/PointStamped.h>

double checkPoint[20][3];
int count = 0;
geometry_msgs::PointStamped outputPoint;
double sum[3];
double midVal[3];


 void callback(const geometry_msgs::PointStampedConstPtr& msg)
 {
     		
			if( count < 20)          //salvataggio in questo array di 20 valori
			{
				checkPoint[count][0] = msg->point.x;
				checkPoint[count][1] = msg->point.y;
				checkPoint[count][2] = msg->point.z;
				count++;
			}
			
			
}
   	
 

									//MAIN
int main(int argc, char** argv)
{
 	 ros::init(argc, argv, "pointValidator");
  	 ros::NodeHandle                    nh;
	 ros::Subscriber                    sub = nh.subscribe("eyePosInWorld", 10,callback);
	 ros::Publisher                     pub = nh.advertise<geometry_msgs::PointStamped>("toController",100);
 	 ros::spin();
 	 
 	 if(count == 20)
			{
				for(int t= 0; t < 3; t++)
				{
 					for(int y= 0; y < 20; y++)
					{
						int indice;
						double savedVal;
						double minVal = checkPoint[y][t];
						savedVal = checkPoint[y][t];
		
						for(int j = y+1; j < 20; j++)
						{
							if(checkPoint[j][t] < minVal)
							{
								minVal = checkPoint[j][t];
								indice= j;
							}
			
						}
						checkPoint[indice][t] = savedVal;
						checkPoint[y][t] = minVal;
		
					}
				}
				
				for(int h= 0; h < 3; h++)
				{
					for(int g= 9; g < 16; g++)
					{
						sum[h] = sum[h] + checkPoint[g][h];
					}
				}
				midVal[0] = sum[0]/20;
				midVal[1] = sum[1]/20;
				midVal[2] = sum[2]/20;
				std::cout << sum[0]    << "   " << sum[1]    << "   "<< sum[2]    << std::endl;
				std::cout << midVal[0] << "   " << midVal[1] << "   "<< midVal[2] << std::endl;
				outputPoint.header.frame_id = midVal[0];
				outputPoint.point.x = midVal[0];
				outputPoint.point.y = midVal[1];
				outputPoint.point.z = midVal[2];
				pub.publish(outputPoint);
			}
 	
 	
 	 return 0;
 	 
}

// rosbag play 2021-07-12-11-18-12_0.bag  





