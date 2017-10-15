#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include "opencv2/objdetect/objdetect.hpp"
using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher pub;
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    pub=nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1000);

    cv::namedWindow(OPENCV_WINDOW,CV_WINDOW_AUTOSIZE);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  
  float trans(float x1,float y1,float x0,float y0,float r2,float *theta)

{
       float t,R,a,b,c,H,X,tan;
       a=1.29;b=0.291;c=4.4;//parameters depending upon dimensions//
       H = 39; //Height of base of paraboloid reflector//
       R = sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1));
       t = R*1.89372/(2*a*r2);
       tan = -(a*t*t*t*t+(c+3*a)*t*t-2*b*t-c)/(b*t*t+2*(c+a)*t-b);
        
       *theta = 180*atan((y1-y0)/(x1-x0))/3.1415;
       //*theta = (y1-y0)/(x1-x0);
       
       X = H/tan;

       return X;


}

void gotospeed(float linearx,float angularz){
	geometry_msgs::Twist twist;
	
	twist.linear.x=linearx;
   	twist.angular.z=angularz;
     	pub.publish(twist);

}

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if ((*cv_ptr).image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      
      
 
 
 int iLowH =85;
 int iHighH = 132;

 int iLowS = 124;
 int iHighS = 255;

 int iLowV = 83;
 int iHighV = 255;

 

 Mat image=cv_ptr->image;

      
 //Create trackbars in "OPENCV_WINDOW" window
 cvCreateTrackbar("LowH", "OPENCV_WINDOW", &iLowH, 179); //Hue (0 - 179)
 cvCreateTrackbar("HighH", "OPENCV_WINDOW", &iHighH, 179);

 cvCreateTrackbar("LowS", "OPENCV_WINDOW", &iLowS, 255); //Saturation (0 - 255)
 cvCreateTrackbar("HighS", "OPENCV_WINDOW", &iHighS, 255);

 cvCreateTrackbar("LowV", "OPENCV_WINDOW", &iLowV, 255); //Value (0 - 255)
 cvCreateTrackbar("HighV", "OPENCV_WINDOW", &iHighV, 255);

 Mat imgHSV;
 double largest_area=0;
 int largest_contour_index=0;



  cvtColor(image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  Mat imgThresholded;

  inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (remove small objects from the foreground);
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (fill small holes in the foreground);
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  imshow("Thresholded Image", imgThresholded); //show the thresholded image
  
  ////////////////largest contour detection
  
  vector<Vec4i> hierarchy;//hierarchy of contours
	vector< vector<Point> > contours; // Vector for storing selected contour
       
 
	findContours(imgThresholded , contours, hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE); // Find the contours in the image
        
	for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
	{
		double a=contourArea( contours[i],false);  //  Find the area of contour
       
		if(a>largest_area){
		largest_area=a;
		largest_contour_index=i;                //Store the index of largest contour
       
		}
 
	}
	
	   Scalar color = Scalar(0, 0,255);
       drawContours( image, contours,largest_contour_index, color, 3, 8, hierarchy, 0, Point() );
  
     //Calculate the moments of the thresholded image
  //Moments oMoments = moments(contours[largest_contour_index]);
  Moments oMoments = moments(imgThresholded);
  double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  double dArea = oMoments.m00;
   
   int posX = dM10 / dArea;
   int posY = dM01 / dArea;
  
  
  float e,eX,r1,r2,X,x0,x1,y0,y1; float theta;

    //printf("Enter the radius of radius of inner circle in pixles: ");
    //scanf("%f",&r1);
    r1=53;/////////////////// radius of inner circle;
    
    //printf("Enter co ordinates of center of circle form MATLAB: ");
    //scanf("%f %f",&x0,&y0);
     x0=316;
     y0=207;
    r2 = r1/0.2597;
    e = (r1/r2-0.277929)/0.277929*100;//error function//
    e = sqrt(e*e);
    printf("Estimated error is readings= %f percent\n",e);

        //printf("Enter co ordinates of centroid from MATLAB:  ");
        //scanf("%f %f",&x1,&y1);
      x1=posX*1.0;
      y1=posY*1.0;
      float xx=x1-320,yy=y1-240;
      
       
      printf("posX =%f  posY=%f  centx %f  cent y%f \n",x1,y1,x0,y0);
        X = trans(x1,y1,x0,y0,r2,&theta);
        
        if(xx>0 ){
      		theta=90-theta;
      	}
      	else{
      	theta=-theta-90;
      	}
        X=abs(X);
        eX = X*e/100;
        X=X-2*eX;
        printf("Distance of Object form detector is: (%f) +- %f cm\n",X,eX);
        printf("with angle theta with horizontal axis:( %f degrees)\n",theta);

        float linearx=0,angularz=0;
        if(x1>0 && y1>0 && X>30 && X<170){
        if(theta>-8.0 && theta <8.0){  ////////straight;
        
              if(X<400){
              linearx=(X*0.4)/400;
              }
              else{
              linearx=0;
              }
              gotospeed(linearx,0);
              
        	
        }
        
        else if(theta<=-8.0 && theta>=-180){  ////left
              linearx=0;
              angularz=-(theta*2)/180;
              gotospeed(linearx,angularz);
           
           }
        else if(theta>=8.0 && theta<=180){	/////right
              linearx=0;
              angularz=-(theta*2)/180;
              gotospeed(linearx,angularz);
        
        }
        }
        else if(x1>0 && y1>0 && X<=30){
        
              linearx=0;
              angularz=0;
              gotospeed(linearx,angularz);
        
        }
        else if(x1>0 && y1>0 && X>=170){
              if(theta>-8.0 && theta <8.0){  ////////straight;
        
              
              linearx=0.25;
              
              gotospeed(linearx,0);
              
        	
        }
        
        else if(theta<=-8.0 && theta>=-180){  ////left
              linearx=0;
              angularz=-(theta*2)/180;
              gotospeed(linearx,angularz);
           
           }
        else if(theta>=8.0 && theta<=180){	/////right
              linearx=0;
              angularz=-(theta*2)/180;
              gotospeed(linearx,angularz);
        
        }
        }
        
        
   line( image, Point(x0,0), Point(x0,480 ), Scalar( 0, 0, 0 ),2,8 );
   line( image, Point(0,y0), Point(640,y0 ), Scalar( 0, 0, 0 ),2,8 );

 	
 	  
 
  circle( image, Point(posX, posY), 2, Scalar( 255, 255, 255 ), 2, 8 );
  imshow("Original", image); //show the original image
    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
