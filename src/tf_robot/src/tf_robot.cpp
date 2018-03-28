#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "whurobot_msgs/GameInfo.h"
#include "whurobot_msgs/ArmorInfo.h"
#include <iostream>
using namespace std;


geometry_msgs::Pose pose_uwb_msg;
geometry_msgs::Pose pose_enemy_msg;
float gimbal_chassis_angle;
void cb_gameInfo(const whurobot_msgs::GameInfo& msg){
  pose_uwb_msg=msg.UWBPose;
  gimbal_chassis_angle=msg.gimbalAngle;
}

void cb_armorInfo(const whurobot_msgs::ArmorInfo& msg){
  pose_enemy_msg=msg.pose_global;
}


int main(int argc,char** argv){
  ros::init(argc,argv,"tf_robot");
  ros::NodeHandle nh;
  
  ros::Subscriber sub_game_info=nh.subscribe("/game_info",1,&cb_gameInfo);
  ros::Subscriber sub_armor_info=nh.subscribe("/vision/armor_info",1,&cb_armorInfo);
  ros::Publisher pub_odom_info=nh.advertise<nav_msgs::Odometry>("odom",1);
  tf::TransformBroadcaster odom_tf;
  geometry_msgs::TransformStamped odom_trans;
  tf::TransformBroadcaster gimbal_tf;
  tf::Transform gimbal_trans;
  tf::TransformBroadcaster camera_tf;
  tf::Transform camera_trans;
  tf::TransformBroadcaster enemy_tf;
  tf::Transform enemy_trans;
  nav_msgs::Odometry odom_msg;
  
  while(ros::ok()){
    odom_msg.header.stamp=ros::Time::now();
    odom_msg.header.frame_id="odom";
    odom_msg.child_frame_id="base_link";
    odom_msg.pose.pose=pose_uwb_msg;
    pub_odom_info.publish(odom_msg);
    
    odom_trans.header.stamp=ros::Time::now();
    odom_trans.header.frame_id="odom";
    odom_trans.child_frame_id="base_link";
    odom_trans.transform.translation.x=pose_uwb_msg.position.x;
    odom_trans.transform.translation.y=pose_uwb_msg.position.y;
    odom_trans.transform.translation.z=0;
    odom_trans.transform.rotation=pose_uwb_msg.orientation;
    odom_tf.sendTransform(odom_trans);
    
    gimbal_trans.setOrigin( tf::Vector3(0, 0, 0.0) ); 
    tf::Quaternion q;
    q.setRPY(0, 30.0/360*3.14, 0); //yaw left+ right-   pitch up- down+
    gimbal_trans.setRotation(q);
    gimbal_tf.sendTransform(tf::StampedTransform(gimbal_trans, ros::Time::now(), "base_link", "gimbal_link")); 
    
    camera_trans.setOrigin( tf::Vector3(0.15, 0, 0.0) ); 
    q.setRPY(-1.57, 0,-1.57); 
    camera_trans.setRotation(q);
    camera_tf.sendTransform(tf::StampedTransform(camera_trans, ros::Time::now(), "gimbal_link", "camera_link")); 
    
    enemy_trans.setOrigin( tf::Vector3(pose_enemy_msg.position.x/100, pose_enemy_msg.position.y/100,pose_enemy_msg.position.z/100) ); 
    q.setRPY(0, 0, 0);
    enemy_trans.setRotation(q);
    enemy_tf.sendTransform(tf::StampedTransform(enemy_trans, ros::Time::now(), "camera_link", "enemy_link")); 
    ros::spinOnce();
  }
}