#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
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
double v_pre=0,w_pre=0;
double v=0,w=0;

class IR_Tracking
{
  public:
  ros::NodeHandle nh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber cmd_sub_;
  IR_Tracking()
  {
    cmd_sub_ = nh_.subscribe("/sensor/ir_value", 1 ,&IR_Tracking::DistCB,this);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/andbot/cmd_vel", 1);

  }

void DistCB(const geometry_msgs::Vector3& msg){
  geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());

  double s[3]={msg.x,msg.y,msg.z};
/*------------------------------
// limit range 100cm
-------------------------------*/
  for (int i=0;i<3;i++)
  {
	if (s[i]>110)
	  s[i]=110;
  }

 
  if (s[0]>100 && s[1]>100 && s[2]>100)
	MODE = MODE_STOP;
  else if (s[0]<100 && s[1]<100 && s[2]<100)
	MODE = MODE_TRACKING;

  else if (s[0]>100 && s[1]<100 && s[2]<100)
	MODE = MODE_FIND_PERSON_CW;
  else if (s[0]>100 && s[1]>100 && s[2]<100)
	MODE = MODE_FIND_PERSON_CW;

  else if (s[0]<100 && s[1]<100 && s[2]>100)
	MODE = MODE_FIND_PERSON_CCW;
  else if (s[0]<100 && s[1]>100 && s[2]>100)
	MODE = MODE_FIND_PERSON_CCW;
  else
	MODE = MODE_STOP;

  switch (MODE){

    case MODE_TRACKING:
	v=0.01*(s[1]-50);
	w=0.01*(s[0]-s[2]);
     	break;

    case MODE_FIND_PERSON_CW:
	v=0;
	if (w!=0)
	  w=w_pre;
	else 
	  w=1;
     	break;

    case MODE_FIND_PERSON_CCW:
	v=0;
	if (w!=0)
	  w=w_pre;
	else 
	  w=-1;
     	break;

    default:
	v=0;w=0;
  } 
  cmd->linear.x=v;
  cmd->angular.z=w;
  cmd_pub_.publish(cmd);
  v_pre=v;
  w_pre=w;
//   ROS_INFO("v %f",v);
//   ROS_INFO("w %f",w);


/*
  geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());


  cmd->linear.x = msg.x;
  cmd->angular.z = 0;
  cmd_pub_.publish(cmd);
*/

   ROS_INFO("SensorValue 1 %f",s[0]);
   ROS_INFO("SensorValue 2 %f",s[1]);
   ROS_INFO("SensorValue 3 %f",s[2]);

}


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ir_tracking");
  IR_Tracking it;
  ros::spin();
  return 0;
}
