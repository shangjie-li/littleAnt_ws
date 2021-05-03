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
	nh_private_.param<std::string>("pub_topic_marker_array", pub_topic_marker_array_, "/obstacles_in_gps");
	nh_private_.param<std::string>("pub_topic_global_path", pub_topic_global_path_, "/global_path");
	nh_private_.param<std::string>("pub_topic_local_path", pub_topic_local_path_, "/local_path");

	nh_private_.param<std::string>("marker_array_frame_id", marker_array_frame_id_, "base_link");
	nh_private_.param<std::string>("global_path_frame_id", global_path_frame_id_, "base_link");
	nh_private_.param<std::string>("local_path_frame_id", local_path_frame_id_, "base_link");

	nh_private_.param<float>("max_match_distance", max_match_distance_, 10.0); // m
	nh_private_.param<float>("local_path_length", local_path_length_, 50.0); // m

	pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>(pub_topic_marker_array_, 1);
	pub_global_path_ = nh_private_.advertise<nav_msgs::Path>(pub_topic_global_path_, 1);
	pub_local_path_ = nh_private_.advertise<nav_msgs::Path>(pub_topic_local_path_, 1);

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

bool Avoiding::isRunning()
{
	return is_running_;
}

// 定时回调函数
// 根据全局路径设置局部路径
// 维护全局路径和局部路径中的停车信息
void Avoiding::timer_callback(const ros::TimerEvent&)
{
	static size_t cnt = 0;

	// 读取车辆状态，创建副本避免多次读取
	const VehicleState vehicle = vehicle_state_;

	dx_gps2global_ = vehicle.pose.x;
	dy_gps2global_ = vehicle.pose.y;
	phi_gps2global_ = vehicle.pose.yaw;

	// 寻找当前车辆位置对应的路径点，亦为局部路径起点，更新pose_index到全局路径
	size_t nearest_idx = findNearestPointInPath(global_path_, vehicle.pose, max_match_distance_);
	global_path_.pose_index = nearest_idx;

	// 根据设定路径长度，寻找局部路径终点，当剩余距离不足时，返回全局路径终点索引
	size_t farthest_idx = findPointInPath(global_path_, local_path_length_, nearest_idx);

	size_t len = farthest_idx - nearest_idx + 1;
	if(len < 10)
	{
		ROS_ERROR("[%s] Path tracking completed.", __NAME__);
		is_running_ = false; // 该变量的状态被其他节点监听
		return;
	}
	
	local_path_.clear();
	local_path_.points.resize(len);
	for(size_t i = 0; i < len; i++)
	{
		local_path_.points[i].x = global_path_.points[nearest_idx + i].x;
		local_path_.points[i].y = global_path_.points[nearest_idx + i].y;
		local_path_.points[i].yaw = global_path_.points[nearest_idx + i].yaw;
	}
	computePathCurvature(local_path_);
	local_path_.has_curvature = true;

    local_path_.resolution = global_path_.resolution;
	local_path_.final_index = local_path_.points.size() - 1;

	// 将全局路径的停车点转换为局部路径的停车点
	bool has_park_point = false;
	for(size_t i = 0; i < global_path_.park_points.points.size(); i++)
	{
		// 如果停车点位于局部路径内，将其添加至局部路径
		// 只考虑尚未到达的停车点、已经到达且停车时长未满足要求的停车点
		// 忽略停车时长满足要求的停车点，从而自车可以继续行驶
		size_t idx = global_path_.park_points.points[i].index;
		if(idx >= nearest_idx && idx <= farthest_idx)
		{
			has_park_point = true;

			size_t _index = idx - nearest_idx;
			float _duration = global_path_.park_points.points[i].parkingDuration;
			double _time = global_path_.park_points.points[i].parkingTime;
			bool _parking = global_path_.park_points.points[i].isParking;

			// 停车点尚未到达
			if(!_parking)
			{
				local_path_.park_points.points.emplace_back(_index, _duration, _time, _parking);
			}
			
			// 停车点已经到达且停车时长未满足要求
			else if(_parking && ros::Time::now().toSec() - _time < _duration)
			{
				local_path_.park_points.points.emplace_back(_index, _duration, _time, _parking);
			}
		}
	}

	// 如果局部路径内没有停车点，将局部路径终点设置为停车点（每次回调更新）
	if(!has_park_point) local_path_.park_points.points.emplace_back(local_path_.final_index, 0.0);

	// 处理局部路径中的第一个停车点
	size_t _index = local_path_.park_points.points[0].index;
	bool _parking = global_path_.park_points.points[0].isParking;
	float dis = computeDistance(local_path_[0].x, local_path_[0].y, local_path_[_index].x, local_path_[_index].y);

	// 当接近停车点时，进入停车状态并计时
	if(!_parking && dis < 0.5)
	{
		local_path_.park_points.points[0].isParking = true;
		local_path_.park_points.points[0].parkingTime = ros::Time::now().toSec();

		// 将停车信息更新到全局路径，以便下次回调时读取正确的停车信息
		for(size_t i = 0; i < global_path_.park_points.points.size(); i++)
		{
			if(_index + nearest_idx == global_path_.park_points.points[i].index)
			{
				global_path_.park_points.points[i].isParking = true;
				global_path_.park_points.points[i].parkingTime = local_path_.park_points.points[0].parkingTime;
				break;
			}
		}
		ROS_INFO("[%s] Start parking, parking point(global):%lu.", __NAME__, _index + nearest_idx);
	}
	else if(_parking)
	{
		ROS_INFO("[%s] Keep parking.", __NAME__);
	}
	
	// 路径拓展延伸，保证全局路径终点部分的路径跟踪过程正常行驶（每次回调更新）
	computeExtendingPath(local_path_, 20.0);

	if((cnt++) % 50 == 0)
	{
		ROS_INFO("[%s]",
		    __NAME__);
		ROS_INFO("[%s] nearest_idx:%d\t farthest_idx:%d\t park_idx:%d\t dis2park:%.2f",
		    __NAME__, nearest_idx, farthest_idx, _index, dis);
		    
		publishPath(pub_global_path_, global_path_, global_path_.pose_index, global_path_.final_index, global_path_frame_id_);
		publishPath(pub_local_path_, local_path_, 0, local_path_.final_index, local_path_frame_id_);
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
	int n = 5;
	if(path.points.size() - 1 < n)
	{
		ROS_ERROR("[%s] Path points is too few (%lu), extend path failed", __NAME__, path.points.size() - 1);
		return;
	}
	int end_idx = path.points.size() - 1;
	
	// 取最后一个点与倒数第n个点的连线向后插值
	float dx = (path.points[end_idx].x - path.points[end_idx - n].x) / n;
	float dy = (path.points[end_idx].y - path.points[end_idx - n].y) / n;
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

void Avoiding::publishPath(ros::Publisher& pub,
                           const Path& path_to_pub,
						   const size_t& begin_idx,
						   const size_t& end_idx,
						   const std::string& frame_id)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id;

	if(end_idx <= begin_idx) return;
	
	// reserve为一次性分配容器大小，容量设置为end_idx - begin_idx + 1
	path.poses.reserve(end_idx - begin_idx + 1);
	
	for(size_t i = begin_idx; i < end_idx; i++)
	{
		double x = path_to_pub.points[i].x;
		double y = path_to_pub.points[i].y;
		transformGlobal2Gps(x, y);
		
		geometry_msgs::PoseStamped p;
        p.pose.position.x = x;
        p.pose.position.y = y;

        p.header.frame_id = frame_id;
        path.poses.push_back(p);
	}
	pub.publish(path);
}
