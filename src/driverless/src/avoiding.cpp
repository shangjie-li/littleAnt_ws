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

	nh_private_.param<std::string>("pub_topic_marker_array", pub_topic_marker_array_, "/obstacles_in_path");
	nh_private_.param<std::string>("pub_topic_global_path", pub_topic_global_path_, "/global_path");
	nh_private_.param<std::string>("pub_topic_local_path", pub_topic_local_path_, "/local_path");

	nh_private_.param<std::string>("marker_array_frame_id", marker_array_frame_id_, "base_link");
	nh_private_.param<std::string>("global_path_frame_id", global_path_frame_id_, "base_link");
	nh_private_.param<std::string>("local_path_frame_id", local_path_frame_id_, "base_link");

	nh_private_.param<float>("max_match_distance", max_match_distance_, 10.0); // m
	nh_private_.param<float>("max_deceleration", max_deceleration_, 1.0); // m/s2
	nh_private_.param<float>("min_following_distance", min_following_distance_, 7.5); // m

	nh_private_.param<int>("min_path_index_num", min_path_index_num_, 5);

	nh_private_.param<float>("dx_base2gps", dx_base2gps_, 0.0);
	nh_private_.param<float>("dy_base2gps", dy_base2gps_, 0.0);
	nh_private_.param<float>("phi_base2gps", phi_base2gps_, 0.02); // rad

	pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>(pub_topic_marker_array_, 1);
	pub_global_path_ = nh_private_.advertise<nav_msgs::Path>(pub_topic_global_path_, 1);
	pub_local_path_ = nh_private_.advertise<nav_msgs::Path>(pub_topic_local_path_, 1);
	
	// 设置默认值
	offset_ = 0.0; // m
	following_mode_ = false;
	nearest_obstacle_distance_ = 0.0;
	nearest_obstacle_index_ = 0.0;

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
	cmd_timer_ = nh_.createTimer(ros::Duration(0.03), &Avoiding::cmd_timer_callback, this);

	return true;
}

void Avoiding::stop()
{
	cmd_timer_.stop();
	sub_obstacle_array_.shutdown();
	is_running_ = false;
}

bool Avoiding::isRunning()
{
	return is_running_;
}

// 定时回调函数
// 根据全局路径和障碍物信息设置局部路径
// 维护停车信息
void Avoiding::cmd_timer_callback(const ros::TimerEvent&)
{
	if(!is_ready_) return;
	
	static size_t cnt = 0;

	// 读取车辆状态，创建副本避免多次读取
	const Pose vehicle_pose = vehicle_state_.getPose(LOCK);
	const float vehicle_speed = vehicle_state_.getSpeed(LOCK);

	dx_gps2global_ = vehicle_pose.x;
	dy_gps2global_ = vehicle_pose.y;
	phi_gps2global_ = vehicle_pose.yaw;

	// 计算局部路径长度
	float local_path_length;
	if(following_mode_)
	{
		local_path_length = nearest_obstacle_distance_ - min_following_distance_;
		if(local_path_length < 0.0) local_path_length = 0.0;
	}
	else
	{
		local_path_length = vehicle_speed * vehicle_speed / (2 * max_deceleration_) + min_following_distance_;
	}
	
	// 寻找当前自车位置对应的路径点，亦为局部路径起点，更新pose_index到全局路径
	size_t nearest_idx = findNearestPointInPath(global_path_, vehicle_pose, max_match_distance_);
	global_path_.pose_index = nearest_idx;

	// 根据设定长度和起始点索引，寻找局部路径终点在全局路径中的索引
	// 如果设定长度为0，返回起始点索引
	// 如果全局路径剩余距离不足，返回全局路径终点索引
	size_t farthest_idx = findPointInPath(global_path_, local_path_length, nearest_idx);

	// 如果当前点与全局路径终点过近，结束任务
	// 留出10个定位点余量，使得自车速度完全减为0时，近似到达期望终点位置
	if(global_path_.final_index - nearest_idx < 10)
	{
		ROS_ERROR("[%s] Path tracking completed.", __NAME__);
		is_running_ = false; // 该变量的状态被其他节点监听
		return;
	}
	
	// 设置局部路径
	local_path_.mutex.lock();
	local_path_.clear();

	// 当局部路径长度为0时，len = 1
	size_t len = farthest_idx - nearest_idx + 1;
	local_path_.points.resize(len);
	for(size_t i = 0; i < len; i++)
	{
		local_path_.points[i].x = global_path_.points[nearest_idx + i].x;
		local_path_.points[i].y = global_path_.points[nearest_idx + i].y;
		local_path_.points[i].yaw = global_path_.points[nearest_idx + i].yaw;
		local_path_.points[i].curvature = global_path_.points[nearest_idx + i].curvature;

		offsetPoint(local_path_.points[i], offset_);
	}
	local_path_.resolution = global_path_.resolution;
	
	if(!local_path_.has_curvature)
	{
		computePathCurvature(local_path_);
		local_path_.has_curvature = true;
	}
    
	local_path_.pose_index = 0;
	local_path_.final_index = local_path_.points.size() - 1;
	
	// 添加停车点信息
	// 从全局路径获得下一个停车点
	ParkingPoint& cur_park_point = global_path_.park_points.next();
	// 如果停车点位于局部路径内，将其存入局部路径中
	if(cur_park_point.index < farthest_idx)
	{
	    size_t idx = cur_park_point.index - nearest_idx;
	    float duration = cur_park_point.parkingDuration;
	    double time = cur_park_point.parkingTime;
	    bool flag = cur_park_point.isParking;
	    
	    local_path_.park_points.points.emplace_back(idx, duration, time, flag);
	}
    
    // 计算自车当前点到停车点距离，判定是否到达
	float dis2park = computeDistance(global_path_.points[global_path_.pose_index], global_path_[cur_park_point.index]);
    if(dis2park < 0.5 && !cur_park_point.isParking)
    {
        cur_park_point.isParking = true;
        cur_park_point.parkingTime = ros::Time::now().toSec();
		ROS_INFO("[%s] Start parking, parking point (global):%lu.", __NAME__, cur_park_point.index);
	}
	
	// 如果到达停车点，且停车时长满足要求，更新全局路径的停车点
	if(cur_park_point.isParking && ros::Time::now().toSec() - cur_park_point.parkingTime > cur_park_point.parkingDuration)
	{
		global_path_.park_points.next_index++;
		ROS_INFO("[%s] End parking, parking point (global):%lu.", __NAME__, cur_park_point.index);
    }
    
	// 不论停车点是否位于局部路径内，将局部路径的终点设置为一个永久停车点（每次回调更新），以便在路径跟踪过程中控制速度
	local_path_.park_points.points.emplace_back(local_path_.final_index, 0.0);
	
	// 路径拓展延伸，保证终点部分的路径跟踪过程正常，同时防止局部路径过短导致出错
	// 经过拓展延伸后，local_path_.final_index小于local_path.size() - 1
	if(local_path_.points.size() >= min_path_index_num_) computeExtendingPath(local_path_, 20.0);
	local_path_.mutex.unlock();

	if((cnt++) % 50 == 0)
	{
		ROS_INFO("[%s]",
		    __NAME__);
		ROS_INFO("[%s] nearest_idx:%d\t farthest_idx:%d\t path_lengh:%.2f\t dis2park:%.2f\t cur_park:%d",
		    __NAME__, nearest_idx, farthest_idx, local_path_length, dis2park, cur_park_point.index);
		    
		// 在base系显示全局路径
		publishPath(pub_global_path_, global_path_, global_path_.pose_index, global_path_.final_index, global_path_frame_id_);
		
		// 在base系显示局部路径
		publishPath(pub_local_path_, local_path_, local_path_.pose_index, local_path_.final_index, local_path_frame_id_);
	}
	
	
}

// 障碍物检测回调函数
void Avoiding::obstacles_callback(const perception_msgs::ObstacleArray::ConstPtr& obstacles)
{
}

void Avoiding::computePathCurvature(Path& path)
{
	size_t num = path.points.size();
	for(int i = 0; i < num - 1; i++)
	{
		float delta_theta = path.points[i + 1].yaw - path.points[i].yaw;
		if(delta_theta <= -M_PI) delta_theta += 2 * M_PI;
		if(delta_theta > M_PI) delta_theta -= 2 * M_PI;
		
		// 将两点距离作为弧长
		float length = computeDistance(path.points[i].x, path.points[i].y, path.points[i + 1].x, path.points[i + 1].y);
		if(length == 0)
			path.points[i].curvature = 0.0;
		else
			path.points[i].curvature = delta_theta / length;
	}
	
	// 均值滤波
	int n = 10;
	float sum = 0.0;
	for(int i = 0; i < num; i++)
	{
		if(i < n)
			sum += path.points[i].curvature;
		else
		{
			path.points[i - n / 2].curvature = sum / n;
			sum += (path.points[i].curvature - path.points[i - n].curvature);
		}
	}
}

void Avoiding::computeExtendingPath(Path& path,
						            const float& extending_dis)
{
	int n = min_path_index_num_;
	assert(path.points.size() >= n);
	size_t end_idx = path.points.size() - 1;
	
	// 取最后一个点与倒数第n个点的连线向后插值
	float dx = (path.points[end_idx].x - path.points[end_idx - n + 1].x) / n;
	float dy = (path.points[end_idx].y - path.points[end_idx - n + 1].y) / n;
	float ds = sqrt(dx * dx + dy * dy);

	GpsPoint p;
	float remaind_dis = 0.0;
	size_t j = 1;
	while(remaind_dis < extending_dis)
	{
		p.x = path.points[end_idx].x + dx * j;
		p.y = path.points[end_idx].y + dy * j;
		p.curvature = 0.0;
		path.points.push_back(p);
		
		remaind_dis += ds;
		j++;
	}
}

void Avoiding::transformGlobal2Gps(double& x,
                                   double& y)
{
	transform2DPoint(x, y, 0, -dx_gps2global_, -dy_gps2global_);
	transform2DPoint(x, y, -phi_gps2global_, 0, 0);
}

void Avoiding::transformGps2Base(double& x,
                                 double& y)
{
	transform2DPoint(x, y, 0, -dx_base2gps_, -dy_base2gps_);
	transform2DPoint(x, y, -phi_base2gps_, 0, 0);
}

void Avoiding::publishPath(ros::Publisher& pub,
                           const Path& path_to_pub,
						   const size_t& begin_idx,
						   const size_t& end_idx,
						   const std::string& frame_id)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id;

	if(begin_idx >= end_idx) return;
	
	// reserve为容器一次性分配内存大小，容量设置为end_idx - begin_idx + 1
	path.poses.reserve(end_idx - begin_idx + 1);
	
	for(size_t i = begin_idx; i < end_idx + 1; i++)
	{
		double x = path_to_pub.points[i].x;
		double y = path_to_pub.points[i].y;
		transformGlobal2Gps(x, y);
		transformGps2Base(x, y);
		
		geometry_msgs::PoseStamped p;
        p.pose.position.x = x;
        p.pose.position.y = y;

        p.header.frame_id = frame_id;
        path.poses.push_back(p);
	}
	pub.publish(path);
}
