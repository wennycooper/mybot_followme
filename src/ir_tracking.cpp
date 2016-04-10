#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <sys/time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

#define MODE_TRACKING 1
#define MODE_STOP 2
#define MODE_FIND_PERSON_CW 3
#define MODE_FIND_PERSON_CCW 4
int MODE = MODE_STOP; 
double pre_v=0,pre_w=0;
double target_v=0,target_w=0;
double delta_v=0,delta_w=0;

double min_dist=25.0;
double keep_range_in=30.0,keep_range_out=50.0;
double track_dist=min_dist+keep_range_in;
double max_dist=min_dist+keep_range_in+keep_range_out;
double limit_dist=110;
double target_d,target_d_pre;
double target_th=0,target_th_pre=0;

double target_x=track_dist,target_x_pre=track_dist;
double target_y=0,target_y_pre=0;


class IR_Tracking
{
  public:
  ros::NodeHandle nh_;
  ros::Publisher cmd_pub_,ir_cmd_pub_;
  ros::Subscriber cmd_sub_;
  IR_Tracking()
  {
    cmd_sub_ = nh_.subscribe("/sensor/ir_value", 1 ,&IR_Tracking::DistCB,this);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/andbot/cmd_vel", 1);
    ir_cmd_pub_ = nh_.advertise<geometry_msgs::Vector3>("/sensor/ir_value_f", 1);


  }

void DistCB(const geometry_msgs::Vector3& msg){
  geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
  geometry_msgs::Vector3Ptr ir_cmd(new geometry_msgs::Vector3());

  double ir_value[3]={msg.x,msg.y,msg.z};
  double ir_pos[3]={-10.0,0.0,10.0};
  double m1=0,m2=0;
/*------------------------------
// limit range 160cm
-------------------------------*/
  for (int i=0;i<3;i++)
  {
	if (ir_value[i]>limit_dist)
	  ir_value[i]=limit_dist;
  }

  if (ir_value[0]>max_dist && ir_value[1]>max_dist && ir_value[2]>max_dist){   
    target_x=track_dist;
    target_y=0;
  }

  if (ir_value[0]<max_dist && ir_value[1]>max_dist && ir_value[2]>max_dist){   
    target_x=ir_value[0];
    target_y=20;
  }
  if (ir_value[0]<max_dist && ir_value[1]<max_dist && ir_value[2]>max_dist){    
    target_x=(float)(ir_value[0]+ir_value[1])/2;
    target_y=10;
  }
  if (ir_value[0]<max_dist && ir_value[1]<max_dist && ir_value[2]<max_dist){    
    target_x=ir_value[1];
    target_y=0;
  }
  if (ir_value[0]>max_dist && ir_value[1]<max_dist && ir_value[2]<max_dist){    
    target_x=(float)(ir_value[1]+ir_value[2])/2;
    target_y=-10;
  }
  if (ir_value[0]>max_dist && ir_value[1]>max_dist && ir_value[2]<max_dist){   
    target_x=ir_value[2];
    target_y=-20;
  }

    target_th=atan2(target_y,target_x);
    target_d=sqrt(target_y*target_y+target_x*target_x);
    target_v=0.06*(target_d-track_dist);
    target_w=4*target_th;

    delta_v=target_v-pre_v;
    double lim_dv=0.1,lim_dw=0.1,lim_tv=0.5,lim_tw=1.5;
    if (delta_v>=lim_dv) delta_v=lim_dv;
    else if (delta_v<=-lim_dv) delta_v=-lim_dv;

    delta_w=target_w-pre_w;
    if (delta_w>=lim_dw) delta_w=lim_dw;
    else if (delta_w<=-lim_dw) delta_w=-lim_dw;

    target_v=pre_v+delta_v;
    target_w=pre_w+delta_w;
    if (target_v>=lim_tv) target_v=lim_tv;
    else if (target_v<=-lim_tv) target_v=-lim_tv;
    if (target_w>=lim_tw) target_w=lim_tw;
    else if (target_w<=-lim_tw) target_w=-lim_tw;



    cmd->linear.x=target_v;
    cmd->angular.z=target_w;
    cmd_pub_.publish(cmd);
    pre_v=target_v;
    pre_w=target_w;
   ROS_INFO("v %f w %f",target_v,target_w);
   ROS_INFO("x %f y %f d %f th %f",target_x,target_y,target_d,target_th);

   ROS_INFO("ir_value %f %f %f ",ir_value[0],ir_value[1],ir_value[2]);
//   ROS_INFO("ir_value_f[1] %f",ir_value_f[1]);
//   ROS_INFO("ir_value_f[2] %f",ir_value_f[2]);
/*
  geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());


  cmd->linear.x = msg.x;
  cmd->angular.z = 0;
  cmd_pub_.publish(cmd);
*/

}


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ir_tracking");
  IR_Tracking it;
  ros::spin();
  return 0;
}
