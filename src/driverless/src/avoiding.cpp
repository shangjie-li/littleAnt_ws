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
	nh_private_.param<std::string>("pub_topic_global_path", pub_topic_global_path_, "/global_path");
	nh_private_.param<std::string>("pub_topic_local_path", pub_topic_local_path_, "/local_path");

	nh_private_.param<std::string>("marker_array_frame_id", marker_array_frame_id_, "base_link");
	nh_private_.param<std::string>("global_path_frame_id", global_path_frame_id_, "base_link");
	nh_private_.param<std::string>("local_path_frame_id", local_path_frame_id_, "base_link");

	nh_private_.param<float>("max_match_distance", max_match_distance_, 10.0); // m
	nh_private_.param<float>("max_deceleration", max_deceleration_, 1.0); // m/s2
	nh_private_.param<float>("min_following_distance", min_following_distance_, 7.5); // m
	nh_private_.param<float>("max_search_range", max_search_range_, 30.0); // m
	nh_private_.param<float>("safe_margin", safe_margin_, 0.5); // m
	nh_private_.param<float>("lane_left_width", lane_left_width_, 5.25); // m
	nh_private_.param<float>("lane_right_width", lane_right_width_, 1.75); // m

	nh_private_.param<bool>("use_avoiding", use_avoiding_, true);
	nh_private_.param<float>("max_avoiding_speed", max_avoiding_speed_, 1.5); // m/s
	
	nh_private_.param<double>("obstacle_array_interval_threshold", obstacle_array_interval_threshold_, 0.2);

	nh_private_.param<float>("dx_sensor2base", dx_sensor2base_, 0.0);
	nh_private_.param<float>("dy_sensor2base", dy_sensor2base_, 0.0);
	nh_private_.param<float>("phi_sensor2base", phi_sensor2base_, 0.03); // rad

	nh_private_.param<float>("dx_base2gps", dx_base2gps_, 0.0);
	nh_private_.param<float>("dy_base2gps", dy_base2gps_, 0.0);
	nh_private_.param<float>("phi_base2gps", phi_base2gps_, 0.02); // rad

	pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>(pub_topic_marker_array_, 1);
	pub_global_path_ = nh_private_.advertise<nav_msgs::Path>(pub_topic_global_path_, 1);
	pub_local_path_ = nh_private_.advertise<nav_msgs::Path>(pub_topic_local_path_, 1);
	
	// 设置默认值
	obstacle_array_time_ = 0.0;
	offset_ = 0.0; // m
	following_mode_ = false;
	is_avoiding_ = false;

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

float Avoiding::getOffset()
{
    return offset_;
}

bool Avoiding::getAvoidingState()
{
    return is_avoiding_;
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

	if(ros::Time::now().toSec() - obstacle_array_time_ > obstacle_array_interval_threshold_)
	{
		offset_ = 0.0;
		following_mode_ = false;
	}
	
	float offset_temp = offset_;
	bool following_mode_temp = following_mode_;
	bool is_avoiding_temp = is_avoiding_;
	
	if(is_avoiding_ && vehicle_speed > max_avoiding_speed_)
	{
	    offset_temp = 0.0;
	    following_mode_temp = true;
	}

	// 计算局部路径长度
	float local_path_length;
	if(following_mode_temp)
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

		offsetPoint(local_path_.points[i], offset_temp);
	}
	local_path_.resolution = global_path_.resolution;
	
	if(!local_path_.has_curvature)
	{
		getCurvature(local_path_);
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
	if(local_path_.points.size() >= 5) getExtending(local_path_, 20.0);
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
	if(!is_ready_) return;

	if(obstacles->obstacles.size() == 0)
	{
		obstacle_array_time_ = ros::Time::now().toSec();
		offset_ = 0.0;
		following_mode_ = false;
		is_avoiding_ = false;
		return;
	}

	// 读取车辆状态，创建副本避免多次读取
	const Pose vehicle_pose = vehicle_state_.getPose(LOCK);
	
	// 选择路径中自车所在点和搜索区域终点
	size_t nearest_idx = global_path_.pose_index;
	size_t farthest_idx = findPointInPath(global_path_, max_search_range_, nearest_idx);
	
	// 获取停车点索引
	// 用于限制障碍物搜索距离，超出停车点的障碍物不予考虑
	// 保证车辆驶入停车点，不被前方障碍干扰
	size_t dest_idx = global_path_.park_points.next().index;
	if(farthest_idx >= dest_idx) farthest_idx = dest_idx;

	if(farthest_idx - nearest_idx < 2)
	{
		obstacle_array_time_ = ros::Time::now().toSec();
		offset_ = 0.0;
		following_mode_ = false;
		is_avoiding_ = false;
		return;
	}
	
	// 判定当前路径中是否存在障碍物
	bool obs_in_path = false;
	
	float nearest_obs_dis = FLT_MAX;
	float nearest_obs_idx;
	
	float half_width = safe_margin_ + vehicle_params_.width / 2;
	
	for(size_t i = 0; i < obstacles->obstacles.size(); i++)
	{
		const perception_msgs::Obstacle& obs = obstacles->obstacles[i];

		// 忽略后方的障碍物
		if(obs.pose.position.x < -2.0) continue;
		
		// 忽略过远的障碍物
		float dis2ego = computeObstacleDistance2Ego(obs);
		if(dis2ego > max_search_range_) continue;

		// 计算障碍物中心点
		double obs_x;
		double obs_y;
		getGlobalObstacle(obs, obs_x, obs_y);

		// 忽略终点以后的障碍物
		size_t idx = findNearestPointInPath(global_path_, obs_x, obs_y, nearest_idx, nearest_idx, farthest_idx);
		if(idx >= farthest_idx) continue;

		int which_side = judgeWhichSide(obs, global_path_, nearest_idx, farthest_idx);
		float gap2path = computeGapBetweenObstacleAndPath(obs, global_path_, nearest_idx, farthest_idx);
		
		// 如果路径穿过障碍物
		if(which_side == 0)
		{
			obs_in_path = true;
			if(dis2ego < nearest_obs_dis)
			{
				nearest_obs_dis = dis2ego;
				nearest_obs_idx = i;
			}
		}
		// 如果障碍物距离路径足够近
		else if(gap2path <= half_width)
		{
			obs_in_path = true;
			if(dis2ego < nearest_obs_dis)
			{
				nearest_obs_dis = dis2ego;
				nearest_obs_idx = i;
			}
		}
	}

	// 在base系显示所有障碍物
	publishMarkerArray(obstacles, obs_in_path, nearest_obs_idx);
	
	nearest_obstacle_distance_ = nearest_obs_dis;
	nearest_obstacle_index_ = nearest_obs_idx;

	// 如果路径中不存在障碍物，offset_置0，following_mode_置false
	if(!obs_in_path)
	{
		obstacle_array_time_ = ros::Time::now().toSec();
		offset_ = 0.0;
		following_mode_ = false;
		is_avoiding_ = false;
		return;
	}
	
	// 控制避障指令的有效性
	// 当不执行避障过程时，仍然根据障碍物位置设定局部路径长度
	if(!use_avoiding_)
	{
	    obstacle_array_time_ = ros::Time::now().toSec();
	    offset_ = 0.0;
	    following_mode_ = true;
	    is_avoiding_ = false;
	    return;
	}

	// 如果路径中存在障碍物，尝试避让
	// 将路径向左右两侧平移，使车辆避开道路内所有障碍物，计算offset_和following_mode_
	float left_min = 0;
	float left_max = lane_left_width_ - half_width;
	float right_min = 0;
	float right_max = lane_right_width_ - half_width;
	
	for(size_t i = 0; i < obstacles->obstacles.size(); i++)
	{
		const perception_msgs::Obstacle& obs = obstacles->obstacles[i];

		// 忽略后方的障碍物
		if(obs.pose.position.x < -2.0) continue;
		
		// 忽略过远的障碍物
		float dis2ego = computeObstacleDistance2Ego(obs);
		if(dis2ego > max_search_range_) continue;

		// 计算障碍物中心点
		double obs_x;
		double obs_y;
		getGlobalObstacle(obs, obs_x, obs_y);

		// 忽略终点以后的障碍物
		size_t idx = findNearestPointInPath(global_path_, obs_x, obs_y, nearest_idx, nearest_idx, farthest_idx);
		if(idx >= farthest_idx) continue;

		int which_side = judgeWhichSide(obs, global_path_, nearest_idx, farthest_idx);
		float gap2path = computeGapBetweenObstacleAndPath(obs, global_path_, nearest_idx, farthest_idx);
		
		// 如果路径穿过障碍物
		if(which_side == 0)
		{
			// 计算障碍物各顶点
			double obs_xs[4];
			double obs_ys[4];
			getGlobalObstacle(obs, obs_xs, obs_ys);
			
			for(int i = 0; i < 4; i++)
			{
				int which_side_point = judgeWhichSide(obs_xs[i], obs_ys[i], global_path_, nearest_idx, farthest_idx);
				double dis2path = findMinDistance2Path(global_path_, obs_xs[i], obs_ys[i], nearest_idx, nearest_idx, farthest_idx);
				
				// 点在路径上
				if(which_side_point == 0) continue;
				// 点在路径左侧
				else if(which_side_point == 1)
				{
				    float dis = dis2path + half_width;
				    left_min = dis > left_min ? dis : left_min;
				}
				// 点在路径右侧
				else if(which_side_point == -1)
				{
				    float dis = dis2path + half_width;
				    right_min = dis > right_min ? dis : right_min;
				}
			}
		}
		// 障碍物在路径左侧
		else if(which_side == 1)
		{
		    float dis = gap2path - half_width;
		    left_max = dis < left_max ? dis : left_max;
		    if(dis < 0.0) right_min = -dis > right_min ? -dis : right_min;
		}
		// 障碍物在路径右侧
		else if(which_side == -1)
		{
		    float dis = gap2path - half_width;
		    right_max = dis < right_max ? dis : right_max;
		    if(dis < 0.0) left_min = -dis > left_min ? -dis : left_min;
		}
	}

	bool left_passable = false;
	bool right_passable = false;
	float left_offset;
	float right_offset;
	
	if(left_max > left_min)
	{
		left_passable = true;
		left_offset = left_min;
		assert(left_offset >= 0.0);
	}

	if(right_max > right_min)
	{
		right_passable = true;
		right_offset = right_min;
		assert(right_offset >= 0.0);
	}
	
	// 根据避让结果，设置offset_和following_mode_
	// 车辆前进时，往左为负（offset应小于0）往右为正（offset应大于0）
	if(!left_passable && !right_passable)
	{
		offset_ = 0.0;
		following_mode_ = true;
	}
	else if(left_passable && !right_passable)
	{
		offset_ = -left_offset;
		following_mode_ = false;
	}
	else if(!left_passable && right_passable)
	{
		offset_ = right_offset;
		following_mode_ = false;
	}
	else
	{
		offset_ = left_offset < right_offset ? -left_offset : right_offset;
		following_mode_ = false;
	}

	obstacle_array_time_ = ros::Time::now().toSec();
	is_avoiding_ = true;

	ROS_INFO("[%s]",
	    __NAME__);
	ROS_INFO("[%s] nearest_idx:%d\t farthest_idx:%d\t search_dis:%.2f",
		__NAME__, nearest_idx, farthest_idx, max_search_range_);
	ROS_INFO("[%s] obs_dis:%.2f\t obs_idx:%d\t half_width:%.2f\t offset:%.2f",
		__NAME__, nearest_obstacle_distance_, nearest_obstacle_index_, half_width, offset_);
	ROS_INFO("[%s] obs_xlocal:%.2f\t obs_ylocal:%.2f",
		__NAME__, obstacles->obstacles[nearest_obstacle_index_].pose.position.x, obstacles->obstacles[nearest_obstacle_index_].pose.position.y);
	ROS_INFO("[%s] left_min:%.2f\t left_max:%.2f\t right_min:%.2f\t right_max:%.2f",
	    __NAME__, left_min, left_max, right_min, right_max);
	
}

int Avoiding::judgeWhichSide(const double& x,
							 const double& y,
							 const Path& path,
							 const size_t& nearest_idx,
							 const size_t& farthest_idx)
{
	assert(farthest_idx - nearest_idx >= 2);

	size_t idx = findNearestPointInPath(path, x, y, nearest_idx, nearest_idx, farthest_idx);

	// 用两点(p1x, p1y)和(p2x, p2y)表示路径切线
	double p1x, p2x, p1y, p2y;
	if(idx == nearest_idx)
	{
		p1x = path.points[idx].x;
		p1y = path.points[idx].y;
		p2x = path.points[idx + 1].x;
		p2y = path.points[idx + 1].y;
	}
	else if(idx == farthest_idx)
	{
		p1x = path.points[idx - 1].x;
		p1y = path.points[idx - 1].y;
		p2x = path.points[idx].x;
		p2y = path.points[idx].y;
	}
	else
	{
		p1x = path.points[idx - 1].x;
		p1y = path.points[idx - 1].y;
		p2x = path.points[idx + 1].x;
		p2y = path.points[idx + 1].y;
	}

	// 若已知向量AB和一点P
	// 当向量AB × 向量AP = 0时，P与AB共线
	// 当向量AB × 向量AP > 0时，P在AB左侧，ABP为逆时针排列
	// 当向量AB × 向量AP < 0时，P在AB右侧，ABP为顺时针排列
	float cross_product = (p2x - p1x) * (y - p1y) - (x - p1x) * (p2y - p1y);

	if(cross_product == 0.0) return 0;
	else if(cross_product > 0.0) return 1; // 左侧
	else if(cross_product < 0.0) return -1; // 右侧
}

int Avoiding::judgeWhichSide(const perception_msgs::Obstacle& obs,
							 const Path& path,
							 const size_t& nearest_idx,
							 const size_t& farthest_idx)
{
	// 计算障碍物中心点
	double obs_x;
	double obs_y;
	getGlobalObstacle(obs, obs_x, obs_y);

	int side_c = judgeWhichSide(obs_x, obs_y, path, nearest_idx, farthest_idx);

	// 计算障碍物各顶点
	double obs_xs[4];
	double obs_ys[4];
	getGlobalObstacle(obs, obs_xs, obs_ys);

	for(int i = 0; i < 4; i++)
	{
		int side = judgeWhichSide(obs_xs[i], obs_ys[i], path, nearest_idx, farthest_idx);
		if(side != side_c)
			return 0;
		else
			continue;
	}
	return side_c;
}

void Avoiding::getGlobalObstacle(const perception_msgs::Obstacle& obs,
								 double& x,
								 double& y)
{
	computeObstacleCenter(obs, x, y);
	transformSensor2Base(x, y);
	transformBase2Gps(x, y);
	transformGps2Global(x, y);
}

void Avoiding::getGlobalObstacle(const perception_msgs::Obstacle& obs,
								 double xs[4],
								 double ys[4]) // 数组作形参将自动转换为指针
{
	computeObstacleVertex(obs, xs, ys);
	transformSensor2Base(xs, ys);
	transformBase2Gps(xs, ys);
	transformGps2Global(xs, ys);
}

void Avoiding::computeObstacleCenter(const perception_msgs::Obstacle& obs,
									 double& obs_x,
									 double& obs_y)
{
	obs_x = obs.pose.position.x;
	obs_y = obs.pose.position.y;
}

void Avoiding::computeObstacleVertex(const perception_msgs::Obstacle& obs,
                                     double obs_xs[4],
									 double obs_ys[4]) // 数组作形参将自动转换为指针
{
	double obs_phi;
	computeObstacleOrientation(obs, obs_phi);
	
	obs_xs[0] = obs.scale.x / 2;
	obs_ys[0] = obs.scale.y / 2;

	obs_xs[1] = -obs.scale.x / 2;
	obs_ys[1] = obs.scale.y / 2;

	obs_xs[2] = -obs.scale.x / 2;
	obs_ys[2] = -obs.scale.y / 2;

	obs_xs[3] = obs.scale.x / 2;
	obs_ys[3] = -obs.scale.y / 2;

	double x0 = obs.pose.position.x;
	double y0 = obs.pose.position.y;

	transform2DPoints(obs_xs, obs_ys, obs_phi, x0, y0);
}

void Avoiding::computeObstacleOrientation(const perception_msgs::Obstacle& obs,
                                          double& phi)
{
	// atan(x)求的是x的反正切，其返回值为[-pi/2, pi/2]之间的一个数
    // atan2(y, x)求的是y/x的反正切，其返回值为[-pi, pi]之间的一个数
	
	// phi属于[0, pi)
	phi = 2 * atan(obs.pose.orientation.z / obs.pose.orientation.w);
}

void Avoiding::transformSensor2Base(double& phi)
{
	phi += phi_sensor2base_;
	if(phi < 0) phi += M_PI;
	if(phi >= M_PI) phi -= M_PI;
}

void Avoiding::transformSensor2Base(double& x,
                                    double& y)
{
	transform2DPoint(x, y, phi_sensor2base_, dx_sensor2base_, dy_sensor2base_);
}

void Avoiding::transformSensor2Base(double xs[4],
                                    double ys[4])
{
	transform2DPoints(xs, ys, phi_sensor2base_, dx_sensor2base_, dy_sensor2base_);
}

void Avoiding::transformBase2Gps(double& x,
                                 double& y)
{
	transform2DPoint(x, y, phi_base2gps_, dx_base2gps_, dy_base2gps_);
}

void Avoiding::transformBase2Gps(double xs[4],
                                 double ys[4])
{
	transform2DPoints(xs, ys, phi_base2gps_, dx_base2gps_, dy_base2gps_);
}

void Avoiding::transformGps2Global(double& x,
                                   double& y)
{
	transform2DPoint(x, y, phi_gps2global_, dx_gps2global_, dy_gps2global_);
}

void Avoiding::transformGps2Global(double xs[4],
                                   double ys[4])
{
	transform2DPoints(xs, ys, phi_gps2global_, dx_gps2global_, dy_gps2global_);
}

double Avoiding::computeObstacleDistance2Ego(const perception_msgs::Obstacle& obs)
{
	double x = obs.pose.position.x;
	double y = obs.pose.position.y;
	return sqrt(x * x + y * y);
}

double Avoiding::computeGapBetweenObstacleAndPath(const perception_msgs::Obstacle& obs,
												  const Path& path,
												  const size_t& nearest_idx,
												  const size_t& farthest_idx)
{
	// 计算障碍物各顶点
	double obs_xs[4];
	double obs_ys[4];
	getGlobalObstacle(obs, obs_xs, obs_ys);
	
	double gap = DBL_MAX;
	for(int i = 0; i < 4; i++)
	{
		double dis2path = findMinDistance2Path(path, obs_xs[i], obs_ys[i], nearest_idx, nearest_idx, farthest_idx);
		if(dis2path < gap) gap = dis2path;
	}

	return gap;
}

void Avoiding::publishMarkerArray(const perception_msgs::ObstacleArray::ConstPtr obstacles,
                                  const bool& obs_in_path,
								  const size_t& nearest_obs_idx)
{
	visualization_msgs::MarkerArray ma;
	for(size_t i = 0; i < obstacles->obstacles.size(); i++)
	{
		visualization_msgs::Marker m;
		m.header.frame_id = marker_array_frame_id_;
		m.header.stamp = ros::Time::now();

		m.ns = "obstacles";
		m.id = i;
		m.type = visualization_msgs::Marker::CUBE;
		m.action = visualization_msgs::Marker::ADD;

		double obs_x;
		double obs_y;
		computeObstacleCenter(obstacles->obstacles[i], obs_x, obs_y);
		transformSensor2Base(obs_x, obs_y);
		m.pose.position.x = obs_x;
		m.pose.position.y = obs_y;
		m.pose.position.z = obstacles->obstacles[i].pose.position.z;

		double obs_phi;
		computeObstacleOrientation(obstacles->obstacles[i], obs_phi);
		transformSensor2Base(obs_phi);
		m.pose.orientation.x = 0;
		m.pose.orientation.y = 0;
		m.pose.orientation.z = sin(0.5 * obs_phi);
		m.pose.orientation.w = cos(0.5 * obs_phi);

		m.scale.x = obstacles->obstacles[i].scale.x;
		m.scale.y = obstacles->obstacles[i].scale.y;
		m.scale.z = obstacles->obstacles[i].scale.z;

		if(obs_in_path && i == nearest_obs_idx)
		{
			// 淡红色
			m.color.r = 216 / 255.0;
			m.color.g = 0 / 255.0;
			m.color.b = 115 / 255.0;
			m.color.a = 0.85;
		}
		else
		{
			// 淡蓝色
			m.color.r = 91 / 255.0;
			m.color.g = 155 / 255.0;
			m.color.b = 213 / 255.0;
			m.color.a = 0.85;
		}

		m.lifetime = ros::Duration(0.1);
		ma.markers.push_back(m);
	}
    pub_marker_array_.publish(ma);
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
