#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h> 
#include <std_msgs/Int16.h>
#include "control/serial.h"
#include "whurobot_msgs/GameInfo.h"
#include "whurobot_msgs/ArmorInfo.h"
using namespace std;

#define PI 3.1415926535898

//msg
int serial_ready_flag;
struct ArmorInfo
{
  int mode;
  float image_dx;
  float image_dy;
  float global_z;
  float pitch;
  float yaw;
}armor_info;



void cb_armorInfo(const whurobot_msgs::ArmorInfo& msg)
{
  armor_info.mode=msg.mode;
  armor_info.image_dx=msg.pose_image.x;
  armor_info.image_dy=msg.pose_image.y;
  armor_info.global_z=msg.pose_global.position.z;
  armor_info.pitch=msg.angle.y;
  armor_info.yaw=msg.angle.x;
  serial_ready_flag=1;
  
};


int main(int argc,char** argv)
{
  
  ros::init(argc,argv,"control");
  ros::NodeHandle nh;
  
  ros::Subscriber sub_armor_info=nh.subscribe("/vision/armor_info",1,&cb_armorInfo);
  ros::Publisher pub_game_info=nh.advertise<whurobot_msgs::GameInfo>("game_info",1);
  whurobot_msgs::GameInfo game_msg;
  
  ros::Publisher pub_odom_info=nh.advertise<nav_msgs::Odometry>("odom",1);
  tf::TransformBroadcaster odom_tf;
  tf::Transform odom_trans;
  nav_msgs::Odometry odom_msg;
  
  Serial serial("/dev/ttyTHS2");
  serial.configurePort();
  struct RobotMsgToMCU msg_tomcu;
  struct RobotMsgFromMCU msg_frommcu;
  serial_ready_flag=0;
  //ros::Rate loop_rate(300);
  
  while (ros::ok())
  {
    //msg_frommcu
    if (serial.ReadData(msg_frommcu))
    {
      game_msg.header.stamp=ros::Time::now();
      game_msg.header.frame_id="/";
      game_msg.remainingHP=msg_frommcu.remaining_HP;
      game_msg.attackArmorID=msg_frommcu.attack_armorID;
      //game_msg.bulletCount=msg_frommcu.remaining_bullet;
      game_msg.bulletCount=msg_frommcu.uwb_yaw;
      game_msg.UWBPose.position.x=msg_frommcu.uwb_x*1.0/100;
      game_msg.UWBPose.position.y=msg_frommcu.uwb_y*1.0/100;
      
      game_msg.UWBPose.orientation=tf::createQuaternionMsgFromYaw((msg_frommcu.uwb_yaw/100-280)*PI*2/360);
      
      game_msg.gimbalAngle=msg_frommcu.gimbal_chassis_angle*1.0/100;
      pub_game_info.publish(game_msg);
      //printf("msgg:  %d %d %d %d  %d %d %d\n",msg_frommcu.remaining_HP,msg_frommcu.attack_armorID,msg_frommcu.remaining_bullet,msg_frommcu.uwb_x,msg_frommcu.uwb_y,msg_frommcu.uwb_yaw,msg_frommcu.gimbal_chassis_angle);
      
      odom_msg.header.stamp=ros::Time::now();
      odom_msg.header.frame_id="odom";
      odom_msg.child_frame_id="base_link";
      odom_msg.pose.pose=game_msg.UWBPose;
      pub_odom_info.publish(odom_msg);
      
      odom_trans.setOrigin( tf::Vector3(msg_frommcu.uwb_x*1.0/100, msg_frommcu.uwb_y*1.0/100, 0.0) ); 
      odom_trans.setRotation( tf::Quaternion(0,0, game_msg.UWBPose.orientation.z, game_msg.UWBPose.orientation.w) );
      odom_tf.sendTransform(tf::StampedTransform(odom_trans, ros::Time::now(), "odom", "base_link")); 
      
      
      msg_tomcu.clear();	
    }
    
    if (serial_ready_flag==1){
      if (armor_info.mode==1)
	msg_tomcu.setMsg(2,0,0,0,armor_info.yaw,armor_info.pitch,armor_info.global_z);
      else 
	msg_tomcu.setMsg(2,0,0,0,0,0,0);
      //cout<<"send"<<ros::Time::now()<<endl;
      serial.SendData(msg_tomcu);
      serial_ready_flag=0;
    }
    
    
    ros::spinOnce();
    //loop_rate.sleep();
  }
}




