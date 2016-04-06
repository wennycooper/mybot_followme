#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sys/time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/Float32MultiArray.h"
#define PI 3.14159

using namespace std;

struct point{
  float x;
  float y;
  float z;
};
class Target_2D_Lidar
{
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher target_pub_;
  std_msgs::Float32MultiArray brray;

  unsigned long  start_time, now;
  struct timeval tp;

  //int firstBodyFound = 0;

  double min_y_ = -0.3; /**< The minimum y position of the points in the box. */
  double max_y_ = 0.3; /**< The maximum y position of the points in the box. */
  double min_x_ = -0.3; /**< The minimum x position of the points in the box. */
  double max_x_ = 0.3; /**< The maximum x position of the points in the box. */
  double max_z_ = 1.3; /**< The maximum z position of the points in the box. */
  double goal_z_ = 0.6; /**< The distance away from the robot to hold the centroid */
  double z_scale_ = 0.5; /**< The scaling factor for translational robot speed */
  double x_scale_ = 2.0; /**< The scaling factor for rotational robot speed */
  
//  String face_cascade_name = "/home/odroid/catkin_ws/src/mybot_followme/res/haarcascades/haarcascade_frontalface_default.xml";


  
public:
  Target_2D_Lidar()
  {
    // Subscrive to input video feed and publish output video feed

    target_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/target_2d_lidar", 1000);

    scan_sub_= nh_.subscribe("/rplidar_bound_scan", 1, &Target_2D_Lidar::scanCb, this);


//    std_msgs::Float32MultiArray array;
    brray.data.clear();
    for (int i=0; i<4; i++)
        brray.data.push_back(0.0);
    
    gettimeofday(&tp, NULL);
    start_time = tp.tv_sec * 1000000 + tp.tv_usec;

  }

  ~Target_2D_Lidar()
  {
//    cv::destroyWindow(OPENCV_WINDOW);
  }

  void scanCb(const sensor_msgs::LaserScan& scan)
  {
    ROS_INFO("angle_min= %f\n",scan.angle_min);
    ROS_INFO("angle_max= %f\n",scan.angle_max);
    ROS_INFO("angle_increment= %f\n",scan.angle_increment);
    int num=(scan.angle_max-scan.angle_min)/scan.angle_increment;
    ROS_INFO("num= %d\n",num);
    struct point* p = new struct point[num];
    if (p == 0 )
	printf("allocate failed");

    for (int i=0;i<num;i++)
    {
	double distance = scan.ranges[i];
	double angle = (scan.angle_min + i * scan.angle_increment) - PI;
        ROS_INFO("ranges 1= %f angle=%f\n",distance,angle);

    }
    delete p;
  }

  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_2d_lidar");
  Target_2D_Lidar target;
  ros::spin();
  return 0;
}
