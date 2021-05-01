#include "driverless/avoiding.h"
#define __NAME__ "avoiding"

Avoiding::Avoiding():
	AutoDriveBase(__NAME__)
{
}

bool Avoiding::init(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
	nh_ = nh;
	nh_private_ = nh_private;

	nh_private_.param<std::string>("sub_topic_obstacle_array", sub_topic_obstacle_array_, "/obstacle_array");
	nh_private_.param<std::string>("pub_topic_marker_array", pub_topic_marker_array_, "/obstacles_in_base");
	nh_private_.param<std::string>("marker_array_frame_id", marker_array_frame_id_, "base_link");
	
	pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>(pub_topic_marker_array_, 1);
	pub_local_path_ = nh_private_.advertise<nav_msgs::Path>("/local_path", 1);
	
	is_ready_ = true;

	return true;
}

bool Avoiding::start()
{
	if(!is_ready_)
	{
		ROS_ERROR("[%s] System is not ready!", __NAME__);
		return false;
	}
	if(global_path_.size() == 0)
	{
		ROS_ERROR("[%s] No global path!", __NAME__);
		return false;
	}
	if(global_path_.park_points.size() == 0)
	{
		ROS_ERROR("[%s] No parking points!", __NAME__);
		return false;
	}
	if(vehicle_params_.validity == false)
	{
		ROS_ERROR("[%s] Vehicle parameters is invalid, please set them firstly.", __NAME__);
		return false;
	}

	if(!global_path_.park_points.isSorted())
		global_path_.park_points.sort(); // 确保停车点有序

	is_running_ = true;

	sub_obstacle_array_ = nh_.subscribe(sub_topic_obstacle_array_, 1, &Avoiding::obstacles_callback, this);
	cmd_timer_ = nh_.createTimer(ros::Duration(0.03), &Avoiding::timer_callback, this);

	return true;
}

void Avoiding::stop()
{
	cmd_timer_.stop();
	is_running_ = false;
}

void Avoiding::isRunning()
{
	return is_running_;
}

// 定时回调函数
// 根据全局路径设置局部路径
void Avoiding::timer_callback(const ros::TimerEvent&)
{
	static size_t cnt = 0;

	if((cnt++) % 50 == 0)
	{
		publishLocalPath();
	}
	
}

// 障碍物检测回调函数
void Avoiding::obstacles_callback(const perception_msgs::ObstacleArray::ConstPtr& obstacles)
{
	
}

void Avoiding::publishLocalPath()
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "base_link";
	size_t begin_idx = global_path_.pose_index;
	size_t end_idx = global_path_.final_index;
	if(end_idx <= begin_idx) return;

	Pose origin_point = vehicle_state_.getPose(LOCK);
	path.poses.reserve(end_idx - begin_idx + 1);
	
	for(size_t i = begin_idx; i < end_idx; i++)
	{
		std::pair<float, float> local_point = 
			global2local(origin_point.x, origin_point.y, origin_point.yaw, global_path_[i].x, global_path_[i].y);
		
		geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.position.x = local_point.first;
        poseStamped.pose.position.y = local_point.second;

        poseStamped.header.frame_id = "base_link";
        path.poses.push_back(poseStamped);
	}
	pub_local_path_.publish(path);
}