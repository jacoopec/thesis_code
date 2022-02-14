#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/camera_subscriber.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/time_synchronizer.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <tf/transform_listener.h>
#include <math.h>
#include <geometry_msgs/PointStamped.h>


               

 //PARAMETRI, I PRIMI 6 SONO GLI INTERVALLI DI COLORE,  POI QUELLI CHE REGOLANO LA DIMENSIONE DELLE MATRICI DEI KERNEL, E POI AREA
 int hmin = 74, smin  = 0, vmin = 47,hmax = 179, smax = 90, vmax  = 145, CLO = 6, DX=7 , g= 1, DY= 4, ERX = 1 , ERY = 2, OP = 9,aMin = 150 , aMax = 7000;
 
 double depth = 0;
 cv::Point2d processingProduct;
 
 
 static const std::string OPENCV_WINDOW = "findEye18";
 


 //COLORI
 const cv::Scalar RED       = cv::Scalar(0,0,255);
 const cv::Scalar PINK      = cv::Scalar(230,130,255);
 const cv::Scalar BLUE      = cv::Scalar(255,0,0);
 const cv::Scalar LIGHTBLUE = cv::Scalar(255,255,160);
 const cv::Scalar GREEN     = cv::Scalar(0,255,0);
 int tolerance = 10;

 
 //FUNZIONI PER EVITARE DI AVERE LA  STRUTTURA DEL KERNEL PARI
  
 int CLO1 (int CLO)
 {
	if(CLO%2==0){CLO=CLO+1;}
	return CLO;
 }


 int DX1 (int DX)
 {
	if(DX%2==0){DX=DX+1;}
	return DX;
 }

 int DY1 (int DY)
 {
	if(DY%2==0){DY=DY+1;}
	return DY;
 }


 int ER1X (int ERX)
 {
	if(ERX%2==0){ERX=ERX+1;}
	return ERX;
 }

 int ER1Y (int ERY)
 {
	if(ERY%2==0){ERY=ERY+1;}
	return ERY;
 }

 int OP1 (int OP)
 {
	if(OP%2==0){OP=OP+1;}
	return OP;
 }
 
	
	
 
 
 		                                       	
 cv::Point3d computeM(cv::Point3d pt, double depthVal)
{
	cv::Point3d pt2;
	pt2.x = pt.x*depthVal;
	pt2.y = pt.y*depthVal;
	pt2.z = pt.z*depthVal;
	return pt;
}

void imgBoundaryCheck(cv::Point2d pt)
{
	if(pt.x < 640 && pt.y <  480)
	{
    		processingProduct.x = pt.x;
    		processingProduct.y = pt.y;
    	}
}



cv::Point2d imageProc(cv::Mat m)
{
	cv::Point2d uv; //PUNTO RESTITUITO
	uv.x = 0;
	uv.y = 0;
	
	cv::Mat imgD, imgG,imgC, imgE, imgHSV,mask, invert, closed, opened, e;
	cv::cvtColor(m, imgHSV, cv::COLOR_BGR2HSV);
	cv::Scalar low(hmin, smin,vmin);
	
	cv::Scalar up(hmax, smax, vmax);
	cv::inRange(imgHSV, low, up, mask);
	cv::Mat kernelCl = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(CLO1(CLO),CLO1(CLO)));
	cv::Mat kernelOp = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(OP1(OP),  OP1(OP)));
	cv::Mat kernelD  = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(DX1(DX),  DY1(DY)));
	cv::Mat kernelEr = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(ER1X(ERX),ER1Y(ERY)));
	cv::morphologyEx(mask, closed, cv::MORPH_CLOSE, kernelCl);
	cv::morphologyEx(closed, opened, cv::MORPH_OPEN, kernelOp);
	cv::dilate(opened, imgD, kernelD);
	cv::erode(imgD, e, kernelEr);

	std::vector<std::vector<cv::Point>> contours;     //tutti i contorni
	std::vector<cv::Point> bestContour;              //miglior contorno
 	std::vector<cv::Vec4i> hierarchy;
 	std::vector<cv::Point> contornoPoligono;
 	
 	
	//trova tutti i contorni
	findContours(e, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	

 	cv::Point2f p; 
 	cv::Point2f bestC;                                    //centro  miglior ellisse approssimante
 	cv::Point2f Point(0,0);
 	float r;                                             //raggio cerchi
 	double biggerAreaRatio = 0;
 	int index;
 	

 	float areaEllFit, areaC, areaRatios;
 	float a,b;
 	cv::RotatedRect bestRotRect;
	
	
	for(int i = 0; i < contours.size(); i++)
	{
		cv::RotatedRect rotRect = cv::fitEllipse(contours[i]);
		a = (rotRect.size.width)/2;                   //CALCOLO ASSI ELLISSE
		b = (rotRect.size.height)/2;
				 
		areaEllFit = 3.14*b*a;   
		areaC = cv::contourArea(contours[i]);              
		areaRatios = areaC/areaEllFit;        
		p = rotRect.center;                     
		double n = cv::pointPolygonTest(contours[i], p,false);
		 
		
		if(  n ==1 && areaEllFit>aMin && areaEllFit<aMax && areaRatios >biggerAreaRatio && a/b <2 && b/a <2)  
		{
			bestContour=contours[i];
			biggerAreaRatio = areaRatios;
			index = i;
			bestC = p;
			bestRotRect = rotRect;
		}
		
	}

	cv::Moments momts = moments(bestContour, true);
	uv.x  = (momts.m10)/(momts.m00);                                    //calcolo baricentro del miglior contorno
	uv.y  = (momts.m01)/(momts.m00);

	return uv;
}
  
  

	
		
class findEye18
{
	public:
	  ros::NodeHandle                    nh;
          image_transport::ImageTransport    it;
          image_transport::CameraSubscriber  camSub;         
          image_transport::Subscriber        sub; 
          ros::Publisher                     pubPoint;
          image_geometry::PinholeCameraModel cam_model;
          typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CameraSubscriber;
	  CameraSubscriber info_sub;  
	  typedef message_filters::Subscriber<sensor_msgs::Image>      ImageSubscriber;
	  ImageSubscriber image_col;
	  ImageSubscriber image_depth;
	  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> AppTimePolicy;
	  message_filters::Synchronizer<AppTimePolicy> AppSync;
	  ros::Publisher pubEyePos;

           
  		findEye18(): it(nh),  image_col(nh , "/camera/color/image_raw", 25 ), image_depth(nh , "/camera/depth/image_rect_raw", 25), info_sub(nh , "/camera/color/camera_info",25 ),  AppSync(AppTimePolicy(100), image_col, image_depth,info_sub)
  		{
                       AppSync.registerCallback( boost::bind( &findEye18::callback, this, _1, _2, _3) );
  			pubEyePos = nh.advertise<geometry_msgs::PointStamped>("eyePositionInCamera",1000);
  		}
  		
  		

  		
 void callback(const sensor_msgs::ImageConstPtr& imgColor,const sensor_msgs::ImageConstPtr& imgDepth, const sensor_msgs::CameraInfoConstPtr& camInfo)
 {
			cv_bridge::CvImagePtr col_ptr       = cv_bridge::toCvCopy(imgColor,      sensor_msgs::image_encodings::BGR8);
			cv_bridge::CvImagePtr depth_ptr     = cv_bridge::toCvCopy(imgDepth,      sensor_msgs::image_encodings::TYPE_16UC1);
     		try
      		{
      			geometry_msgs::PointStamped eyePosInCamera;
			eyePosInCamera.header.frame_id = "Camera";
 			image_geometry::PinholeCameraModel model;
 			bool info = model.image_geometry::PinholeCameraModel::fromCameraInfo(camInfo);
 			
 			cv::Mat imgR1,imgR2, imgCol, depthImg;
 			imgCol   = col_ptr   -> image;
			depthImg = depth_ptr -> image;
   			
 			
    			cv::Point2d outputProcessingPoint    = imageProc(imgCol);                                //PUNTO DAL PROCESSING
    			imgBoundaryCheck(outputProcessingPoint);
    			 
    			cv::Point3d pt                       = model.projectPixelTo3dRay(processingProduct);  
    			depth                                = depth_ptr->image.at<u_int16_t>(pt.x, pt.y);
    			cv::Point3d pt3                      = computeM(pt, depth);
    			
    			
    			//PARTE DI SCRITTURA
    			std::string ix          = std::__cxx11::to_string(pt3.x);
    			std::string ips         = std::__cxx11::to_string(pt3.y);
    			std::string zi          = std::__cxx11::to_string(pt3.z);
    			std::string pointString = "x: " + ix + " y: " + ips + " z: " + zi ;
			putText(imgCol, pointString, processingProduct, cv::FONT_HERSHEY_PLAIN,0.7,GREEN,1);
			eyePosInCamera.point.x = pt3.x;
			eyePosInCamera.point.y = pt3.y;
			eyePosInCamera.point.z = pt3.z;
			pubEyePos.publish(eyePosInCamera);
	
			
			
			
			
   			imshow("colImg",   imgCol );              
   			cv::waitKey(1);
    		}
    		 
       	catch (cv_bridge::Exception& ex)
                    {
                       ROS_ERROR("FAILED");
                    }
    
               }

  		~findEye18()
 		 {
   			 cv::destroyWindow(OPENCV_WINDOW);
  		 }
};



									//MAIN
int main(int argc, char** argv)
{
 	 ros::init(argc, argv, "findEye18");

  	 findEye18 ic;
  	
 	 ros::spin();
 	 
 	
 	 return 0;
 	 
}

// rosbag play 2021-07-12-11-18-12_0.bag  





