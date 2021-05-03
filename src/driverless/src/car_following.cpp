#include "driverless/car_following.h"

#define __NAME__ "car_following"

CarFollowing::CarFollowing():
	AutoDriveBase(__NAME__)
{
}

bool CarFollowing::init(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
	nh_ = nh;
	nh_private_ = nh_private;

	nh_private_.param<std::string>("sub_topic_obstacle_array", sub_topic_obstacle_array_, "/obstacle_array");
	nh_private_.param<std::string>("pub_topic_marker_array", pub_topic_marker_array_, "/obstacles_in_base");
	nh_private_.param<std::string>("marker_array_frame_id", marker_array_frame_id_, "base_link");

	nh_private_.param<float>("max_following_speed", max_following_speed_, 12.5); // m/s
	nh_private_.param<float>("max_deceleration", max_deceleration_, 1.0); // m/s2
	nh_private_.param<float>("safe_margin", safe_margin_, 0.5); // m
	nh_private_.param<float>("dangerous_distance", dangerous_distance_, 3.5); // m

	nh_private_.param<float>("max_search_distance", max_search_distance_, 50.0);
	nh_private_.param<float>("min_search_distance", min_search_distance_, 7.5);
	nh_private_.param<float>("accelerate_coefficient", accelerate_coefficient_, 0.5);
	nh_private_.param<float>("decelerate_coefficient", decelerate_coefficient_, 0.3);

	nh_private_.param<double>("cmd_interval_threshold", cmd_interval_threshold_, 0.2);

	nh_private_.param<int>("obstacle_repeat_threshold", obstacle_repeat_threshold_, 2);

	nh_private_.param<float>("dx_sensor2base", dx_sensor2base_, 0.0);
	nh_private_.param<float>("dy_sensor2base", dy_sensor2base_, 0.0);
	nh_private_.param<float>("phi_sensor2base", phi_sensor2base_, 0.03); // rad

	nh_private_.param<float>("dx_base2gps", dx_base2gps_, 0.0);
	nh_private_.param<float>("dy_base2gps", dy_base2gps_, 0.0);
	nh_private_.param<float>("phi_base2gps", phi_base2gps_, 0.02); // rad

	pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>(pub_topic_marker_array_, 1);
	
	is_ready_ = true;
	
	return true;
}

// 启动跟随线程
bool CarFollowing::start()
{
	if(!is_ready_)
	{
		ROS_ERROR("[%s] System is not ready!", __NAME__);
		return false;
	}
	while(local_path_.size() == 0) 
	{
		ROS_ERROR("[%s] Waiting for local path...", __NAME__);
		ros::Duration(0.1).sleep(); // 等待0.1s
	}
	if(vehicle_params_.validity == false)
	{
		ROS_ERROR("[%s] Vehicle parameters is invalid, please set them firstly.", __NAME__);
		return false;
	}

	is_running_ = true;
	
	sub_obstacle_array_ = nh_.subscribe(sub_topic_obstacle_array_, 1, &CarFollowing::obstacles_callback, this);
	cmd_check_timer = nh_.createTimer(ros::Duration(0.10), &CarFollowing::cmd_check_timer_callback, this);
	
	return true;
}

void CarFollowing::stop()
{
	sub_obstacle_array_.shutdown();
	cmd_check_timer.stop();
	is_running_ = false;
	
	cmd_mutex_.lock();
	cmd_.speed = 0.0;
	cmd_.validity = false;
	cmd_.speed_validity = false;
	cmd_mutex_.unlock();
}

bool CarFollowing::isRunning()
{
	return is_running_;
}

void CarFollowing::setMaxSpeed(const float& speed)
{
    // speed单位km/h
	max_following_speed_ = speed / 3.6;
}

// 障碍物检测回调函数
// STEP1：寻找路径上且距离自车最近的障碍物，根据障碍物4个顶点与路径的位置关系判断（global系）
// STEP2：判断最近障碍物ID是否与上次ID一致
// STEP3：计算安全距离，利用比例控制对速度指令更新
void CarFollowing::obstacles_callback(const perception_msgs::ObstacleArray::ConstPtr& obstacles)
{
	if(!is_ready_) return;
	if(obstacles->obstacles.size() == 0) return;

	static int tracked_id = 0; // 跟踪障碍物的ID
	static int tracked_times = 0; // 跟踪障碍物的次数

	// 读取车辆状态，创建副本避免多次读取
	const Pose vehicle_pose = vehicle_state_.getPose(LOCK);
	const float vehicle_speed = vehicle_state_.getSpeed(LOCK);

	dx_gps2global_ = vehicle_pose.x;
	dy_gps2global_ = vehicle_pose.y;
	phi_gps2global_ = vehicle_pose.yaw;

	// 读取局部路径，创建副本防止中途被修改
	local_path_.mutex.lock();
	const Path t_path = local_path_;
	local_path_.mutex.unlock();

	if(!t_path.park_points.available()) 
	{
		ROS_ERROR("[%s] No Next Parking Point.", __NAME__);
		return;
	}
	
	// 选择路径中自车所在点和跟车区域终点
	size_t nearest_idx = t_path.pose_index;
	size_t farthest_idx = findPointInPath(t_path, max_search_distance_, nearest_idx);
	
	// 获取停车点索引
	// 用于限制障碍物搜索距离，超出停车点的障碍物不予考虑
	// 保证车辆驶入停车点，不被前方障碍干扰
	size_t dest_idx = t_path.park_points.points[0].index;
	if(farthest_idx >= dest_idx) farthest_idx = dest_idx;

	// 判定路径中是否存在障碍物
	bool obs_in_path = false;
	size_t nearest_obs_idx;
	float nearest_obs_dis2ego = FLT_MAX;
	
	for(size_t i = 0; i < obstacles->obstacles.size(); i++)
	{
		const perception_msgs::Obstacle& obs = obstacles->obstacles[i];

		// 忽略后方的障碍物
		if(obs.pose.position.x < 0) continue;
		
		// 忽略过远的障碍物
		float dis2ego = computeObstacleDistance2Ego(obs);
		if(dis2ego > max_search_distance_) continue;

		// 判定障碍物是否位于路径上
		if(isObstacleInPath(obs, safe_margin_, t_path, nearest_idx, farthest_idx))
		{
			obs_in_path = true;
			if(dis2ego < nearest_obs_dis2ego)
			{
				nearest_obs_dis2ego = dis2ego;
				nearest_obs_idx = i;
			}
		}
	}

	// 在base系显示所有障碍物
	publishMarkerArray(obstacles, obs_in_path, nearest_obs_idx);

	// 如果路径中没有障碍物，则返回
	if(!obs_in_path) return;
	
	// 当前时刻障碍物id与上一时刻一致，即障碍物重复出现，计数器自加
	// 当前时刻障碍物id与上一时刻不同，则障碍物为误检障碍物或者跟踪障碍物丢失
	// 未处于跟踪状态时，障碍物重复出现N次则开始跟踪
	// 处于跟踪状态时，若当前id与上次id不同，归零计数器，等待下次满足条件时继续跟踪
	const perception_msgs::Obstacle& obs_nearest = obstacles->obstacles[nearest_obs_idx];
	if(obs_nearest.id == tracked_id)
	{
		tracked_times++;
		if(tracked_times > obstacle_repeat_threshold_)
			tracked_times = obstacle_repeat_threshold_;
	}
	else
	{
		tracked_id = obs_nearest.id;
		tracked_times = 0;
	}
	
	// 避免误检
	if(tracked_times < obstacle_repeat_threshold_) return;

	// 有效检测到障碍物后，保持安全距离跟随
	// 安全距离：x = v * v / (2 * a) + C常，单位m
	// vehicle.speed单位m/s
	float following_distance = (vehicle_speed) * (vehicle_speed) / (2 * max_deceleration_) + min_search_distance_;

	// 速度指令，单位m/s
	float t_speed_mps;
	
	// 障碍物相对自车速度，单位m/s
	float obs_speed;
	if(obs_nearest.v_validity) obs_speed = computeObstacleSpeed(obs_nearest);
	else obs_speed = - vehicle_speed;

	if(nearest_obs_dis2ego < dangerous_distance_)
	{
		// 紧急避撞
		t_speed_mps = 0.0;
	}
	else
	{
		// 比例控制，加速度与减速度不同，单独计算
		float dis_err = nearest_obs_dis2ego - following_distance;
		if(dis_err >= 0)
			t_speed_mps = vehicle_speed + obs_speed + dis_err * accelerate_coefficient_;
		else
			t_speed_mps = vehicle_speed + obs_speed + dis_err * decelerate_coefficient_;
		
		// 防止速度失控
		if(t_speed_mps > max_following_speed_) t_speed_mps = max_following_speed_;
		
		// 防止速度抖动，保证速度非负
		if(t_speed_mps < 0.5) t_speed_mps = 0.0;
	}

	cmd_time_ = ros::Time::now().toSec();
	cmd_mutex_.lock();
	cmd_.validity = true;
	cmd_.speed_validity = true;
	cmd_.speed = t_speed_mps * 3.6;
	cmd_mutex_.unlock();
	
	ROS_INFO("[%s]",
	    __NAME__);
	ROS_INFO("[%s] dis2ego:%.2f\t obs_v:%.2fm/s\t ego_v:%.2fm/s\t following_dis:%.2f",
		__NAME__, nearest_obs_dis2ego, obs_speed, vehicle_speed, following_distance);
	ROS_INFO("[%s] cmd_v:%.2fkm/h\t t_speed:%.2fm/s\t max_v:%.2fm/s",
	    __NAME__, cmd_.speed, t_speed_mps, max_following_speed_);
	ROS_INFO("[%s] obs_x_local:%.2f\t obs_y_local:%.2f\t obs_id:%d",
		__NAME__, obs_nearest.pose.position.x, obs_nearest.pose.position.y, obs_nearest.id);
	ROS_INFO("[%s] max_decel:%.2f\t min_search_dis:%.2f\t max_search_dis:%.2f",
	    __NAME__, max_deceleration_, min_search_distance_, max_search_distance_);
}

// 定时回调函数
// 控制指令长时间未更新，有效位置false
void CarFollowing::cmd_check_timer_callback(const ros::TimerEvent&)
{
	if(ros::Time::now().toSec() - cmd_time_ > cmd_interval_threshold_)
	{
		cmd_mutex_.lock();
		cmd_.validity = false;
		cmd_.speed_validity = false;
		cmd_mutex_.unlock();
	}
}

bool CarFollowing::isObstacleInPath(const perception_msgs::Obstacle& obs,
									const double& margin,
									const Path& path,
									const size_t& nearest_idx,
									const size_t& farthest_idx)
{
	if(farthest_idx - nearest_idx < 2) return false;
	
	// 计算障碍物中心点
	double obs_x;
	double obs_y;

	computeObstacleCenter(obs, obs_x, obs_y);
	transformSensor2Base(obs_x, obs_y);
	transformBase2Gps(obs_x, obs_y);
	transformGps2Global(obs_x, obs_y);

	// 忽略终点以后的障碍物
	size_t idx = findNearestPointInPath(path, obs_x, obs_y, nearest_idx, nearest_idx, farthest_idx);
	if(idx >= farthest_idx) return false;

	// 判定路径是否穿过障碍物
	if(isPathThroughObstacle(obs, path, nearest_idx, farthest_idx))
	{
	    return true;
    }

	// 计算障碍物各顶点
	double obs_xs[4];
	double obs_ys[4];

	computeObstacleVertex(obs, obs_xs, obs_ys);
	transformSensor2Base(obs_xs, obs_ys);
	transformBase2Gps(obs_xs, obs_ys);
	transformGps2Global(obs_xs, obs_ys);
	
	// 判定障碍物各顶点与路径之间是否留有足够余量
	for(int i = 0; i < 4; i++)
	{
		double dis2path = findMinDistance2Path(path, obs_xs[i], obs_ys[i], idx, nearest_idx, farthest_idx);
		if(dis2path < margin + vehicle_params_.width / 2)
		{
		    return true;
		}
	}

	return false;
}

bool CarFollowing::isPathThroughObstacle(const perception_msgs::Obstacle& obs,
									     const Path& path,
									     const size_t& nearest_idx,
									     const size_t& farthest_idx)
{
	if(farthest_idx - nearest_idx < 2) return false;

	// 计算障碍物中心点
	double obs_x;
	double obs_y;

	computeObstacleCenter(obs, obs_x, obs_y);
	transformSensor2Base(obs_x, obs_y);
	transformBase2Gps(obs_x, obs_y);
	transformGps2Global(obs_x, obs_y);

	// 忽略终点以后的障碍物
	size_t idx = findNearestPointInPath(path, obs_x, obs_y, nearest_idx, nearest_idx, farthest_idx);
	if(idx >= farthest_idx) return false;

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

	// 计算障碍物各顶点
	double obs_xs[4];
	double obs_ys[4];

	computeObstacleVertex(obs, obs_xs, obs_ys);
	transformSensor2Base(obs_xs, obs_ys);
	transformBase2Gps(obs_xs, obs_ys);
	transformGps2Global(obs_xs, obs_ys);

	// 判定障碍物各顶点是否位于切线同侧
	bool flag[4];
	for(int i = 0; i < 4; i++)
	{
		if(p1x == p2x) flag[i] = obs_xs[i] > p1x;
		else flag[i] = obs_ys[i] > ((p1y - p2y) / (p1x - p2x)) * (obs_xs[i] - p1x) + p1y;
	}
	bool last_flag;
	for(int i = 0; i < 4; i++)
	{
		if(i == 0) last_flag = flag[i];
		else
		{
			if(last_flag != flag[i]) return true;
		}
	}

	return false;
}

void CarFollowing::computeObstacleCenter(const perception_msgs::Obstacle& obs,
										 double& obs_x,
										 double& obs_y)
{
	obs_x = obs.pose.position.x;
	obs_y = obs.pose.position.y;
}

void CarFollowing::computeObstacleVertex(const perception_msgs::Obstacle& obs,
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

void CarFollowing::computeObstacleOrientation(const perception_msgs::Obstacle& obs,
                                              double& phi)
{
	// atan(x)求的是x的反正切，其返回值为[-pi/2, pi/2]之间的一个数
    // atan2(y, x)求的是y/x的反正切，其返回值为[-pi, pi]之间的一个数
	
	// phi属于[0, pi)
	phi = 2 * atan(obs.pose.orientation.z / obs.pose.orientation.w);
}

void CarFollowing::transformSensor2Base(double& phi)
{
	phi += phi_sensor2base_;
	if(phi < 0) phi += M_PI;
	if(phi >= M_PI) phi -= M_PI;
}

void CarFollowing::transformSensor2Base(double& x,
                                        double& y)
{
	transform2DPoint(x, y, phi_sensor2base_, dx_sensor2base_, dy_sensor2base_);
}

void CarFollowing::transformSensor2Base(double xs[4],
                                        double ys[4])
{
	transform2DPoints(xs, ys, phi_sensor2base_, dx_sensor2base_, dy_sensor2base_);
}

void CarFollowing::transformBase2Gps(double& x,
                                     double& y)
{
	transform2DPoint(x, y, phi_base2gps_, dx_base2gps_, dy_base2gps_);
}

void CarFollowing::transformBase2Gps(double xs[4],
                                     double ys[4])
{
	transform2DPoints(xs, ys, phi_base2gps_, dx_base2gps_, dy_base2gps_);
}

void CarFollowing::transformGps2Global(double& x,
                                       double& y)
{
	transform2DPoint(x, y, phi_gps2global_, dx_gps2global_, dy_gps2global_);
}

void CarFollowing::transformGps2Global(double xs[4],
                                       double ys[4])
{
	transform2DPoints(xs, ys, phi_gps2global_, dx_gps2global_, dy_gps2global_);
}

double CarFollowing::computeObstacleDistance2Ego(const perception_msgs::Obstacle& obs)
{
	double x = obs.pose.position.x;
	double y = obs.pose.position.y;
	return sqrt(x * x + y * y);
}

double CarFollowing::computeObstacleSpeed(const perception_msgs::Obstacle& obs)
{
	return (double)obs.vx;
}

void CarFollowing::publishMarkerArray(const perception_msgs::ObstacleArray::ConstPtr obstacles,
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
