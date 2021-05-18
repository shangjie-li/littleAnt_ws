
#include <iostream>
#include <ros/ros.h>
#include<ros/time.h>
#include <ros/duration.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <perception_msgs/ObstacleArray.h>
#include <perception_msgs/Obstacle.h>

#include <obu_msgs/OBU_fusion.h>


#include "v2x_noval_convert_xy.hpp"

using namespace std;


class obu_to_xy
{
private:
	//创建节点句柄
	ros::NodeHandle nh;
	//声明订阅器变量，未初始化，可声明多个
	ros::Subscriber sub_obu;
	//声明发布器变量，未初始化，可声明多个
	ros::Publisher pub_markers;
  //定时器
  ros::Timer timer1;
	// twod_convert my_convert;

public:
  visualization_msgs::MarkerArray markers_all_this_frame;
  visualization_msgs::Marker marker_obu_this_frame;
  visualization_msgs::Marker marker_event_this_frame;


public:
	//构造函数
	obu_to_xy()
	{
		//订阅器初始化
		sub_obu = nh.subscribe("/fake_lidar_perception", 512, &obu_to_xy::callback_obu, this);
    // sub_obu=nh.subscribe("/fake_obu",521,&obu_to_xy::callback_obu,this);
		//发布器初始化

    init_arrays();
		pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/obu_all_markers",512);
    timer1=nh.createTimer(ros::Duration(0.1), &obu_to_xy::timer1callback, this);
    
	}
  void init_arrays();
  void callback_obu(obu_msgs::OBU_fusion t);
  void timer1callback(const ros::TimerEvent& event);
};
void obu_to_xy::init_arrays()
{

  cout<<"arrarys init finished"<<endl;
  // 117.437797, 39.1925721, 15.8], 117.438298, 39.1938285, 2000.0, 260.0
  double car_jd=117.4375922;
  double car_wd=39.1921173;
  double event_jd=117.438298;
  double event_wd=39.1938285;
  double car_x,car_y,event_x,event_y;

  noval_convert_xy a;
  
  a.jwd_to_xy(car_jd,car_wd,car_x,car_y);
  a.jwd_to_xy(event_jd,event_wd,event_x,event_y);
  std::cout<<"init_arrays"<<endl;

  printf("%20f",car_x);
  std::cout<<" ";
  printf("%20f",car_y);
  std::cout<<" ";
  printf("%20f",event_x);
  std::cout<<" ";
  printf("%20f",event_y);
  std::cout<<" "<<endl;


  ////////////////////////////////////////////////////
  //marker_obu
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.

  visualization_msgs::Marker marker_obu;
  marker_obu.header.frame_id = "fake";
  marker_obu.header.stamp = ros::Time::now();
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker_obu.ns = "obu_marker";
  marker_obu.id = 0;
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker_obu.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker_obu.action = visualization_msgs::Marker::ADD;
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker_obu.pose.position.x = 0;
  marker_obu.pose.position.y = 0;
  marker_obu.pose.position.z = 0;
  marker_obu.pose.orientation.x = 0.0;
  marker_obu.pose.orientation.y = 0.0;
  marker_obu.pose.orientation.z = 0.0;
  marker_obu.pose.orientation.w = 1.0;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_obu.scale.x = 5.0;
  marker_obu.scale.y = 2.0;
  marker_obu.scale.z = 1.0;
  // Set the color -- be sure to set alpha to something non-zero!
  marker_obu.color.r = 0.0f;
  marker_obu.color.g = 1.0f;
  marker_obu.color.b = 0.0f;
  marker_obu.color.a = 1.0;
  marker_obu.lifetime = ros::Duration(0.1);

  marker_obu_this_frame=marker_obu;
  //////////////////////////////////////////////////////////


  visualization_msgs::Marker marker_event;
  marker_event.header.frame_id = "fake";
  marker_event.header.stamp = ros::Time::now();
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker_event.ns = "obu_marker";
  marker_event.id = 1;
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker_event.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker_event.action = visualization_msgs::Marker::ADD;
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker_event.pose.position.x = car_x-event_x;
  marker_event.pose.position.y = car_y-event_y;
  marker_event.pose.position.z = 0;
  marker_event.pose.orientation.x = 0.0;
  marker_event.pose.orientation.y = 0.0;
  marker_event.pose.orientation.z = 0.0;
  marker_event.pose.orientation.w = 1.0;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_event.scale.x = 20.0;
  marker_event.scale.y = 3.0;
  marker_event.scale.z = 3.0;
  // Set the color -- be sure to set alpha to something non-zero!
  marker_event.color.r = 0.0f;
  marker_event.color.g = 1.0f;
  marker_event.color.b = 0.0f;
  marker_event.color.a = 1.0;
  marker_event.lifetime = ros::Duration(0.1);

  marker_event_this_frame=marker_event;
  //////////////////////////////////////////////////////////
  //markers_all
  // visualization_msgs::MarkerArray markers_all;
  // markers_all.markers.push_back(marker_obu_this_frame);

  markers_all_this_frame.markers.push_back(marker_obu);
  markers_all_this_frame.markers.push_back(marker_event);

}

void obu_to_xy::timer1callback(const ros::TimerEvent& event)
{
    pub_markers.publish(markers_all_this_frame);
    // std::cout<<"pub one markers all"<<endl;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "obu_markers");
    ros::NodeHandle nh("~");
    // omp_set_num_threads(4);
    obu_to_xy my_obu;

    ros::spin();
    return 0;
}