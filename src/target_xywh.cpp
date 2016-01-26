#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Float32MultiArray.h"
#include <sensor_msgs/image_encodings.h>
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class Target_XYWH
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher target_xywh_pub_;

  std_msgs::Float32MultiArray brray;


  double filter_x;
  double filter_y;
  double filter_width;
  double filter_height;
  int firstBodyFound = 0;

  CascadeClassifier face_cascade;
  String face_cascade_name = "/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml";

  //String face_cascade_name = "/usr/local/share/OpenCV/haarcascades/haarcascade_fullbody.xml";

  // my simple filter
  void myFilter(double x, double y, double width, double height)
  {
    double Kp = 0.8;
    double d_x = x - filter_x;
    double d_y = y - filter_y;
    double d_width = width - filter_width;
    double d_height = height - filter_height;

    filter_x += Kp * d_x;
    filter_y += Kp * d_y;
    filter_width  += Kp * d_width;
    filter_height += Kp * d_height;
  }

  
public:
  Target_XYWH()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, 
      &Target_XYWH::imageCb, this);
    image_pub_ = it_.advertise("/target_xywh/output_video", 1);
    target_xywh_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/target_xywh", 1000);

//    std_msgs::Float32MultiArray array;
    brray.data.clear();
    for (int i=0; i<4; i++)
        brray.data.push_back(0.0);
    
    
//    cv::namedWindow(OPENCV_WINDOW);

    //Load the cascades
    if (!face_cascade.load(face_cascade_name))  { 
        printf("--(!)Error loading face cascade\n"); 
    };

  }

  ~Target_XYWH()
  {
//    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    float t;
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

    // Now, the frame is in "cv_ptr->image" 

    std::vector<Rect> faces;
    Mat frame_gray;
    Mat frame(cv_ptr->image);
    resize(frame, frame, Size(160, 120));
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

    t = (double)cvGetTickCount();
    //-- Detect faces
    //face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(16, 12), Size(80, 60) ); // 30ms

    face_cascade.detectMultiScale( frame_gray, faces); // 35ms
    
    t = (double)cvGetTickCount() - t;
    printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );

    if (faces.size() > 0) {
        myFilter(faces[0].x * 4, faces[0].y * 4, faces[0].width * 4, faces[0].height * 4);
        firstBodyFound = 1;
    }

    if (firstBodyFound) {
        size_t i = 0;
        //Point center( (faces[i].x + faces[i].width/2) * 4, (faces[i].y + faces[i].height/2) * 4 );
        //ellipse( frameOrig, center, Size( faces[i].width/2 * 4, faces[i].height/2 *4), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

        rectangle(cv_ptr->image,
           Point(filter_x, filter_y),
           Point(filter_x + filter_width, filter_y + filter_height),
           Scalar(0, 255, 255),
           5,
           8);

        brray.data[0] = filter_x;
        brray.data[1] = filter_y;
        brray.data[2] = filter_width;
        brray.data[3] = filter_height;

        target_xywh_pub_.publish(brray);
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_xywh");
  Target_XYWH ic;
  ros::spin();
  return 0;
}
