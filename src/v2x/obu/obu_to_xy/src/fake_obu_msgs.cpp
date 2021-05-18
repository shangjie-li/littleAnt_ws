
#include <iostream>
#include <ros/ros.h>
#include<ros/time.h>
#include <ros/duration.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <perception_msgs/ObstacleArray.h>
#include <perception_msgs/Obstacle.h>


#include "v2x_noval_convert_xy.hpp"

#include <obu_msgs/OBU_fusion.h>

using namespace std;


class fake_obu
{
private:
	//创建节点句柄
	ros::NodeHandle nh;
	//声明订阅器变量，未初始化，可声明多个
	// ros::Subscriber sub_lidar;
  // ros::Subscriber sub_obu;
	//声明发布器变量，未初始化，可声明多个
	ros::Publisher pub_obu;
  //定时器
  ros::Timer timer1;
	// twod_convert my_convert;

public:
  obu_msgs::OBU_fusion one_obu;



public:
	//构造函数
	fake_obu()
	{
		//订阅器初始化
		// sub_lidar = nh.subscribe("/fake_lidar_perception", 512, &obu_to_xy::callback_lidar, this);
    // sub_obu=nh.subscribe("/fake_obu",521,&obu_to_xy::callback_obu,this);
		//发布器初始化

    init_obu();
		pub_obu = nh.advertise<obu_msgs::OBU_fusion>("/fake_obu",512);
    timer1=nh.createTimer(ros::Duration(1), &fake_obu::timer1callback, this);
    
	}
  void init_obu();
  void timer1callback(const ros::TimerEvent& event);
};
void fake_obu::init_obu()
{
  ///////////////////////////////////////////////////////////////////////-1
  obu_msgs::OBU_fusion tmp;
  //
  tmp.header.frame_id = "fake";
  tmp.header.stamp = ros::Time::now();
  // Add light information here.
  tmp.light_state=-1;
  tmp.light_remain_time=-1;
  tmp.light_jd=-1;
  tmp.light_wd=-1;
  tmp.light_x=-1;
  tmp.light_y=-1;
  // Add ego vehicle information here.
  tmp.obu_jd=-1;
  tmp.obu_wd=-1;
  tmp.obu_angle=-1;
  tmp.obu_x=-1;
  tmp.obu_y=-1;
  tmp.obu_angle_rad=-1;
  // Add obstacle information here.
  tmp.event_jd=-1;
  tmp.event_wd=-1;
  tmp.event_x=-1;
  tmp.event_y=-1;
  tmp.event_length=-1;
  tmp.event_radius=-1;
  // Add park information here.
  tmp.park_jd=-1;
  tmp.park_wd=-1;
  tmp.park_x=-1;
  tmp.park_y=-1;
  // Add emergency car here.
  tmp.emergency_car_jd=-1;
  tmp.emergency_car_wd=-1;
  tmp.emergency_car_x=-1;
  tmp.emergency_car_y=-1;
  tmp.emergency_car_is_near=0;
  //
  ///////////////////////////////////////////////////////////////////////-1

  //[117.437797, 39.1925721 117.438298, 39.1938285, 2000.0, 260.0
  tmp.obu_jd=117.437797;
  tmp.obu_wd=39.1925721;
  tmp.event_jd=117.438298;
  tmp.event_wd=39.1938285;
  tmp.event_length=2000;
  tmp.event_radius=260;
  /////
  one_obu=tmp;
}

void fake_obu::timer1callback(const ros::TimerEvent& event)
{
    pub_obu.publish(one_obu);
    // std::cout<<"pub one markers all"<<endl;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "obu_markers");
    ros::NodeHandle nh("~");
    // omp_set_num_threads(4);
    fake_obu my_obu;

    ros::spin();
    return 0;
}