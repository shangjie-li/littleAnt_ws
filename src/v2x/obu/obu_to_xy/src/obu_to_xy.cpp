
#include <iostream>
#include <ros/ros.h>
#include<ros/time.h>
#include <ros/duration.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <perception_msgs/ObstacleArray.h>
#include <perception_msgs/Obstacle.h>

#include <obu_msgs/OBU_fusion.h>
#include <cmath>

#include "v2x_noval_convert_xy.hpp"

using namespace std;


class obu_to_xy
{
private:
	//创建节点句柄
	ros::NodeHandle nh;

	//声明订阅器
	ros::Subscriber sub_obu;
  ros::Subscriber sub_lidar;

	//声明发布器
	ros::Publisher pub_markers_obu;
  ros::Publisher pub_markers_lidar;

  ros::Publisher pub_obu;
  ros::Publisher pub_lidar;

  //定时器
  ros::Timer o_timer;



//每一帧的变量
public:
  noval_convert_xy a;  //经纬度转xy          a.jwd_to_xy
  perception_msgs::Obstacle o;
  int o_add_flag=0;
  double o_time=0;
  // visualization_msgs::MarkerArray markers_all_this_frame;
  // visualization_msgs::Marker marker_obu_this_frame;
  // visualization_msgs::Marker marker_event_this_frame;

//空msg函数
public:
  visualization_msgs::MarkerArray make_one_empty_markerarray();
  visualization_msgs::Marker make_one_empty_marker();
  perception_msgs::ObstacleArray make_one_empty_ObstacleArray();
  perception_msgs::Obstacle make_one_empty_Obstacle();
  obu_msgs::OBU_fusion make_one_empty_obu_fusion();


public:
	//构造函数
	obu_to_xy()
	{
    
		//订阅
		sub_obu = nh.subscribe("/I2V_info", 1, &obu_to_xy::callback_rec_obu, this);
    sub_lidar = nh.subscribe("/obstacle_array", 1, &obu_to_xy::callback_rec_lidar, this);

		//发布
		pub_markers_obu = nh.advertise<visualization_msgs::MarkerArray>("/pub_markers_obu",1);

    // pub_obu = nh.advertise<obu_msgs::OBU_fusion>("/pub_obu",512);

    pub_lidar = nh.advertise<perception_msgs::ObstacleArray>("/obstacle_array_obu",1);


    //定时
    // init_arrays();
    o_timer=nh.createTimer(ros::Duration(1), &obu_to_xy::o_timer_callback, this);
    
	};

  // void init_arrays();


  //回调函数
  void callback_rec_obu(obu_msgs::OBU_fusion t);

  void callback_rec_lidar(perception_msgs::ObstacleArray t);


  void o_timer_callback(const ros::TimerEvent& event);
};
/////////////////////空msg函数
visualization_msgs::MarkerArray obu_to_xy::make_one_empty_markerarray()
{
  visualization_msgs::MarkerArray t;
  return t;
}

visualization_msgs::Marker obu_to_xy::make_one_empty_marker()
{
  visualization_msgs::Marker t;
  t.header.frame_id = "fake";
  t.header.stamp = ros::Time::now();
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  t.ns = "obu_marker";
  t.id = 0;
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  t.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  t.action = visualization_msgs::Marker::ADD;
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  t.pose.position.x = 0;
  t.pose.position.y = 0;
  t.pose.position.z = 0;
  t.pose.orientation.x = 0.0;
  t.pose.orientation.y = 0.0;
  t.pose.orientation.z = 0.0;
  t.pose.orientation.w = 0.0;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  t.scale.x = 0.0;
  t.scale.y = 0.0;
  t.scale.z = 0.0;
  // Set the color -- be sure to set alpha to something non-zero!
  t.color.r = 0.0f;
  t.color.g = 1.0f;
  t.color.b = 0.0f;
  t.color.a = 1.0;
  t.lifetime = ros::Duration(1);
  return t;
}

perception_msgs::ObstacleArray obu_to_xy::make_one_empty_ObstacleArray()
{
  perception_msgs::ObstacleArray t;
}

perception_msgs::Obstacle obu_to_xy::make_one_empty_Obstacle()
{
  perception_msgs::Obstacle t;
  t.header.frame_id = "fake";
  t.header.stamp = ros::Time::now();
  t.ns="obstacle";
  t.id=0;

  t.label="obstacle";

  // 设置标记位姿
  t.pose.position.x = 0;
  t.pose.position.y = 0;
  t.pose.position.z = 0;
  t.pose.orientation.x = 0;
  t.pose.orientation.y = 0;
  t.pose.orientation.z = 0;
  t.pose.orientation.w = 0;

  // 设置标记尺寸
  t.scale.x = 0;
  t.scale.y = 0;
  t.scale.z = 0;

  t.v_validity=0;
  t.vx=0;
  t.vy=0;
  t.vz=0;

  t.a_validity=0;
  t.ax=0;
  t.ay=0;
  t.az=0;
  return t;
}

obu_msgs::OBU_fusion obu_to_xy::make_one_empty_obu_fusion()
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
  return tmp;
}
/////////////////////空msg函数


/////////////////////回调函数
void obu_to_xy::callback_rec_obu(obu_msgs::OBU_fusion t)
{
  std::cout<<"get one obu_fusion"<<endl;
  // cout<<"t.obu_jd "<<t.obu_jd<<endl;
  // cout<<"t.obu_wd "<<t.obu_jd<<endl;
  // cout<<"t.emergency_car_is_near "<<t.emergency_car_is_near<<endl;
  // cout<<endl;

  obu_msgs::OBU_fusion tmp;
  tmp=t;
  if(tmp.obu_jd!=-1)
  {
    //只有obu消息 没有事件消息
    if(tmp.event_jd==-1)
    {
      std::cout<<"only_obu"<<endl;
      double obu_jd=tmp.obu_jd;
      double obu_wd=tmp.obu_wd;
      double obu_x,obu_y;
 
      a.jwd_to_xy(obu_jd,obu_wd,obu_x,obu_y);

      /////////obu
      visualization_msgs::Marker tmp_marker_obu;
      tmp_marker_obu=make_one_empty_marker();
      tmp_marker_obu.id=0;
      tmp_marker_obu.pose.position.x=0;
      tmp_marker_obu.pose.position.y=0;
      tmp_marker_obu.scale.x=5;
      tmp_marker_obu.scale.y=2;
      tmp_marker_obu.scale.z=1;

      //double obu_hd=tmp.obu_angle*3.1415926/180;
      double temp_hd = (-tmp.obu_angle)*3.1415926/180 + 3.1415926/2;
      if(temp_hd < 0) temp_hd += 3.1415926*2;
      double obu_hd = temp_hd;

      // double temp_hd = (-271.123)*3.1415926/180 + 3.1415926/2;
      // double obu_hd = temp_hd;

      tmp_marker_obu.pose.orientation.z = sin(0.5*obu_hd);
      tmp_marker_obu.pose.orientation.w = cos(0.5*obu_hd);

      visualization_msgs::MarkerArray tmp_markerarray;
      tmp_markerarray=make_one_empty_markerarray();
      tmp_markerarray.markers.push_back(tmp_marker_obu);
      pub_markers_obu.publish(tmp_markerarray);
    }


    //有obu消息，也有事件消息
    if(tmp.event_jd!=-1)
    {
      //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++marker
      // 显示marker
      std::cout<<"obu and event"<<endl;
      double obu_jd=tmp.obu_jd;
      double obu_wd=tmp.obu_wd;
      double obu_x,obu_y;
      double event_jd=tmp.event_jd;
      double event_wd=tmp.event_wd;
      double event_x,event_y;
      a.jwd_to_xy(obu_jd,obu_wd,obu_x,obu_y);
      a.jwd_to_xy(event_jd,event_wd,event_x,event_y);
      /////////obu
      visualization_msgs::Marker tmp_marker_obu;
      tmp_marker_obu=make_one_empty_marker();
      tmp_marker_obu.id=0;
      tmp_marker_obu.pose.position.x=obu_x-event_x;
      tmp_marker_obu.pose.position.y=obu_y-event_y;
      tmp_marker_obu.scale.x=5;
      tmp_marker_obu.scale.y=2;
      tmp_marker_obu.scale.z=1;

      //double obu_hd=tmp.obu_angle*3.1415926/180;
      double temp_hd = (-tmp.obu_angle)*3.1415926/180 + 3.1415926/2;
      if(temp_hd < 0) temp_hd += 3.1415926*2;
      double obu_hd = temp_hd;

      cout<<"accident solid hd"<<obu_hd<<endl;

      // double temp_hd = (-271.123)*3.1415926/180 + 3.1415926/2;
      // double obu_hd = temp_hd;

      tmp_marker_obu.pose.orientation.z = sin(0.5*obu_hd);
      tmp_marker_obu.pose.orientation.w = cos(0.5*obu_hd);
      /////////event
      visualization_msgs::Marker tmp_marker_event;
      tmp_marker_event=make_one_empty_marker();
      tmp_marker_event.id=1;
      tmp_marker_event.pose.position.x=0;
      tmp_marker_event.pose.position.y=0;
      tmp_marker_event.scale.x=1;
      tmp_marker_event.scale.y=1;
      tmp_marker_event.scale.z=1;
      tmp_marker_event.color.r = 1.0f;
      tmp_marker_event.color.g = 0.0f;
      tmp_marker_event.color.b = 0.0f;


      double accident_hd=1.30725;
      tmp_marker_event.pose.orientation.z = sin(0.5*accident_hd);
      tmp_marker_event.pose.orientation.w = cos(0.5*accident_hd);

      /////////all
      visualization_msgs::MarkerArray tmp_markerarray;
      tmp_markerarray=make_one_empty_markerarray();
      tmp_markerarray.markers.push_back(tmp_marker_obu);
      tmp_markerarray.markers.push_back(tmp_marker_event);

      pub_markers_obu.publish(tmp_markerarray);
      //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++marker
      // 添加一个obs
      //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++obs
      
      perception_msgs::Obstacle obs;
      obs=make_one_empty_Obstacle();
      // 设置命名空间和ID，ID应该是独一无二的
      obs.ns = "obstacle";
      obs.id = 10002;

      // 设置位姿
      obs.pose.position.x = event_x-obu_x;
      obs.pose.position.y = event_y-obu_y;
      double tmp_x=obs.pose.position.x;
      double tmp_y=obs.pose.position.y;
      obs.pose.position.x = tmp_x * cos(-obu_hd) - tmp_y * sin(-obu_hd);
      obs.pose.position.y = tmp_x * sin(-obu_hd) + tmp_y * cos(-obu_hd);
      obs.pose.position.z = 0;
      obs.pose.orientation.x = 0;
      obs.pose.orientation.y = 0;
      obs.pose.orientation.z = sin(0.5*(accident_hd - obu_hd));
      obs.pose.orientation.w = cos(0.5*(accident_hd - obu_hd));

      // 设置尺寸
      obs.scale.x = 1;
      obs.scale.y = 1;
      obs.scale.z = 2;
      //
      o=obs;
      o_add_flag=1;
      o_time=ros::Time::now().toSec();
      //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++obs
    }
  }

  
}

void obu_to_xy::callback_rec_lidar(perception_msgs::ObstacleArray t)
{
  perception_msgs::ObstacleArray tmp_obs_array;
  perception_msgs::Obstacle tmp_obs;
  tmp_obs_array=t;
  //add new obs here
  if(o_add_flag==1)
  {
    tmp_obs_array.obstacles.push_back(o);
    cout<<"add one obs"<<endl;
  }
  //add new obs here
  pub_lidar.publish(tmp_obs_array);
}

void obu_to_xy::o_timer_callback(const ros::TimerEvent& event)
{
    // pub_markers_obu.publish(markers_all_this_frame);
    // std::cout<<"pub one markers all"<<endl;
    if(o_add_flag==1)
    {
      double tmp=ros::Time::now().toSec();
      int delta_time=tmp-o_time;
      if(delta_time>0.5)
      {
        o_add_flag=0;
      }
    }
}



/////////////////////回调函数

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obu_markers");
    ros::NodeHandle nh("~");
    // omp_set_num_threads(4);
    obu_to_xy my_obu;

    ros::spin();
    return 0;
}
