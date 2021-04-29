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

	nh_private_.param<float>("max_speed", max_speed_, 40); // km/h
	nh_private_.param<float>("max_deceleration", max_deceleration_, 4); // m/s2
	nh_private_.param<float>("safe_margin", safe_margin_, 0.5); // m
	nh_private_.param<float>("dangerous_distance", dangerous_distance_, 3.5); // m

	nh_private_.param<float>("max_following_distance", max_following_distance_, 20);
	nh_private_.param<float>("min_following_distance", min_following_distance_, 7.5);

	nh_private_.param<double>("cmd_interval_threshold", cmd_interval_threshold_, 0.2);
	nh_private_.param<double>("topic_obstacle_array_interval", topic_obstacle_array_interval_, 0.1);
	nh_private_.param<int>("obstacle_repeat_threshold", obstacle_repeat_threshold_, 2);

	nh_private_.param<float>("dx_sensor2base", dx_sensor2base_, 0);
	nh_private_.param<float>("dy_sensor2base", dy_sensor2base_, 0);
	nh_private_.param<float>("phi_sensor2base", phi_sensor2base_, 0.03);

	nh_private_.param<float>("dx_base2gps", dx_base2gps_, 0);
	nh_private_.param<float>("dy_base2gps", dy_base2gps_, 0);
	nh_private_.param<float>("phi_base2gps", phi_base2gps_, 0.02);

	pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>(pub_topic_marker_array_, 1);
	
	initDiagnosticPublisher(nh_, __NAME__);
	is_ready_ = true;
	return true;
}

// 启动跟驰线程
bool CarFollowing::start()
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

	// 获取终点索引
	// 用于限制障碍物搜索距离，超出终点的目标不予考虑
	// 保证车辆驶入终点，不被前方障碍干扰
	if(!global_path_.park_points.isSorted())
		global_path_.park_points.sort(); // 确保停车点有序

	for(const ParkingPoint& point : global_path_.park_points.points)
	{
		if(point.parkingDuration == 0.0) // 获取停车时间为0（终点）的点
			dest_index_ = point.index;
	}
	
	is_running_ = true;
	sub_obstacle_array_ = nh_.subscribe(sub_topic_obstacle_array_, 1, &CarFollowing::obstacles_callback, this);
	cmd_timer_ = nh_.createTimer(ros::Duration(0.10), &CarFollowing::timer_callback, this);
	return true;
}

void CarFollowing::stop()
{
	sub_obstacle_array_.shutdown();
	cmd_timer_.stop();
	is_running_ = false;
	
	cmd_mutex_.lock();
	cmd_.validity = false;
	cmd_mutex_.unlock();
}

void CarFollowing::timer_callback(const ros::TimerEvent&)
{
	// 控制指令长时间未更新，有效位置false
	if(ros::Time::now().toSec() - cmd_time_ > cmd_interval_threshold_)
	{
		cmd_mutex_.lock();
		cmd_.validity = false;
		cmd_.speed_validity = false;
		cmd_mutex_.unlock();
	}
}

// 障碍物检测回调函数
// STEP1：寻找路径上且距离自车最近的障碍物，根据障碍物4个顶点与路径的位置关系判断（global系）
// STEP2：判断最近目标ID是否与上次ID一致
// STEP3：计算安全跟驰距离，利用比例控制对速度指令更新
void CarFollowing::obstacles_callback(const perception_msgs::ObstacleArray::ConstPtr& obstacles)
{
	static int tracked_id = 0; // 跟踪目标的ID
	static int tracked_times = 0; // 跟踪目标的次数
	
	if(!is_ready_) return;
	if(obstacles->obstacles.size() == 0) return;

	// 读取车辆状态，创建副本避免多次读取
	const VehicleState vehicle = vehicle_state_;

	dx_gps2global_ = vehicle.pose.x;
	dy_gps2global_ = vehicle.pose.y;
	phi_gps2global_ = vehicle.pose.yaw;

	// 安全跟驰距离：x = v * v / (2 * a) + C常，单位m
	float following_distance = (vehicle.speed / 3.6) * (vehicle.speed / 3.6) / (2 * max_deceleration_) + min_following_distance_;

	size_t nearest_idx = global_path_.pose_index;
	size_t farthest_idx = findPointInPath(global_path_, max_following_distance_, nearest_idx);
	if(farthest_idx >= dest_index_) farthest_idx = dest_index_;

	bool obs_in_path = false;
	size_t nearest_obs_idx;
	float nearest_obs_dis2ego = FLT_MAX;
	
	for(size_t i = 0; i < obstacles->obstacles.size(); i++)
	{
		const perception_msgs::Obstacle& obs = obstacles->obstacles[i];

		// 忽略后方的目标
		if(obs.pose.position.x < 0) continue;
		
		// 忽略过远的目标
		float dis2ego = computeObstacleDistance2Ego(obs);
		if(dis2ego > max_following_distance_) continue;

		// 判定障碍物是否位于路径上
		if(isObstacleInPath(obs, safe_margin_, global_path_, nearest_idx, farthest_idx))
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

	if(!obs_in_path) return;
	const perception_msgs::Obstacle& obs_nearest = obstacles->obstacles[nearest_obs_idx];
	
	// 当前时刻目标id与上一时刻一致，即目标重复出现，计数器自加
	// 当前时刻目标id与上一时刻不同，则目标为误检目标或者跟踪目标丢失
	// 未处于跟踪状态时，目标重复出现N次则开始跟踪
	// 处于跟踪状态时，若当前id与上次id不同，归零计数器，等待下次满足条件时继续跟踪
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

	float t_speed; // 期望速度，m/s
	float obs_speed; // 目标相对自车速度，单位m/s
	if(obs_nearest.v_validity) obs_speed = computeObstacleSpeed(obs_nearest);
	else obs_speed = - vehicle.speed / 3.6;

	if(nearest_obs_dis2ego < dangerous_distance_)
	{
		// 紧急避撞
		t_speed = 0.0;
	}
	else
	{
		// 比例控制，加速度与减速度不同，单独计算
		float distanceErr = nearest_obs_dis2ego - following_distance;
		if(distanceErr >= 0)
			t_speed = vehicle.speed / 3.6 + obs_speed + distanceErr * 0.5;
		else
			t_speed = vehicle.speed / 3.6 + obs_speed + distanceErr * 0.3;
		
		// 防止速度失控
		if(t_speed > max_speed_ / 3.6) t_speed = max_speed_ / 3.6;
		
		// 防止速度抖动，保证速度非负
		if(t_speed < 1.0) t_speed = 0.0;
	}

	ROS_INFO("nearest_obs_dis2ego:%.2f\t obs_speed:%.2f\t ego_speed:%.2f\t t_speed:%.2f\t following_distance:%.2f",
		nearest_obs_dis2ego, obs_speed, vehicle.speed / 3.6, t_speed, following_distance);
	
	ROS_INFO("obs_x_local:%.2f\t obs_y_local:%.2f\t obs_id:%d",
		obs_nearest.pose.position.x, obs_nearest.pose.position.y, obs_nearest.id);

	cmd_time_ = ros::Time::now().toSec();
	cmd_mutex_.lock();
	cmd_.validity = true;
	cmd_.speed_validity = true;
	cmd_.speed = t_speed * 3.6;
	cmd_mutex_.unlock();
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
	
	double obs_x_copy;
	double obs_y_copy;
	double obs_x_b;
	double obs_y_b;
	double obs_x_g;
	double obs_y_g;
	double obs_xg_copy;
	double obs_yg_copy;

	computeObstacleCenter(obs, obs_x, obs_y);
	obs_x_copy = obs_x;
	obs_y_copy = obs_y;
	
	transformSensor2Base(obs_x, obs_y);
	obs_x_b = obs_x;
	obs_y_b = obs_y;
	
	transformBase2Gps(obs_x, obs_y);
	obs_x_g = obs_x;
	obs_y_g = obs_y;
	
	double a = obs_x_g * cos(4.77) - obs_y_g * sin(4.77) + dx_gps2global_;
	double b = obs_x_g * sin(4.77) + obs_y_g * cos(4.77) + dy_gps2global_;
	printf("yaw:%.2f\n", phi_gps2global_);
	printf("obs_x_g:%.2f\n", obs_x_g);
	printf("obs_y_g:%.2f\n", obs_y_g);
	printf("a:%.2f\n", a);
	printf("b:%.2f\n", b);
	printf("cos(4.77):%.2f\n", cos(4.77));
	printf("sin(4.77):%.2f\n", sin(4.77));
	printf("???\n");
	
	
	transformGps2Global(obs_x, obs_y);
	obs_xg_copy = obs_x;
	obs_yg_copy = obs_y;

	// 忽略终点以后的目标
	size_t idx = findNearestPointInPath(path, obs_x, obs_y, nearest_idx, nearest_idx, farthest_idx);
	if(idx >= farthest_idx) return false;

	// 判定路径是否穿过障碍物
	if(isPathThroughObstacle(obs, path, nearest_idx, farthest_idx))
	{
	    ROS_ERROR("isPathThroughObstacle");
	    return true;
    }

	// 计算障碍物各顶点
	double obs_xs[4];
	double obs_ys[4];
	
	double obs_xs_copy[4];
	double obs_ys_copy[4];
	double obs_xgs_copy[4];
	double obs_ygs_copy[4];

	computeObstacleVertex(obs, obs_xs, obs_ys);
	
	for(int i = 0; i < 4; i++)
	{
	    obs_xs_copy[i] = obs_xs[i];
	    obs_ys_copy[i] = obs_ys[i];
	}
	
	transformSensor2Base(obs_xs, obs_ys);
	transformBase2Gps(obs_xs, obs_ys);
	transformGps2Global(obs_xs, obs_ys);
	
	for(int i = 0; i < 4; i++)
	{
	    obs_xgs_copy[i] = obs_xs[i];
	    obs_ygs_copy[i] = obs_ys[i];
	}
	
	// 判定障碍物各顶点与路径之间是否留有足够余量
	for(int i = 0; i < 4; i++)
	{
		double dis2path = findMinDistance2Path(path, obs_xs[i], obs_ys[i], idx, nearest_idx, farthest_idx);
		if(dis2path < margin + vehicle_params_.width / 2)
		{
		    ROS_ERROR("dis2path:%.2f\t margin:%.2f\t width:%.2f",
		        dis2path, margin, vehicle_params_.width / 2);
		    
		    for(int i = 0; i < 4; i++)
		    {
				printf("obs_xs_copy:%.2f\t obs_ys_copy:%.2f\n", obs_xs_copy[i], obs_ys_copy[i]);
				printf("obs_xgs_copy:%.2f\t obs_ygs_copy:%.2f\n", obs_xgs_copy[i], obs_ygs_copy[i]);
		    }
		    
		    ROS_ERROR("obs_x_copy:%.2f\t obs_y_copy:%.2f\t obs_x_b:%.2f\t obs_y_b:%.2f\t obs_x_g:%.2f\t obs_y_g:%.2f\t obs_xg_copy:%.2f\t obs_yg_copy:%.2f\t idx:%d",
		        obs_x_copy, obs_y_copy, obs_x_b, obs_y_b, obs_x_g, obs_y_g, obs_xg_copy, obs_yg_copy, idx);
		    ROS_ERROR("dx_gps2global_:%.2f\t dy_gps2global_:%.2f", dx_gps2global_, dy_gps2global_);
		    
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

	// 忽略终点以后的目标
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

void CarFollowing::transform2DPoint(double& x,
                                    double& y,
									const double& phi,
									const double& x0,
									const double& y0)
{
	double temp_x = x;
	double temp_y = y;
	x = temp_x * cos(phi) - temp_y * sin(phi);
	y = temp_x * sin(phi) + temp_y * cos(phi);
	x += x0;
	y += y0;
}

void CarFollowing::transform2DPoints(double xs[4],
                                     double ys[4],
									 const double& phi,
									 const double& x0,
									 const double& y0) // 数组作形参将自动转换为指针
{
    for(int i = 0; i < 4; i++)
    {
        double temp_x = xs[i];
        double temp_y = ys[i];
        xs[i] = temp_x * cos(phi) - temp_y * sin(phi);
        ys[i] = temp_x * sin(phi) + temp_y * cos(phi);
		xs[i] += x0;
		ys[i] += y0;
    }
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

		m.lifetime = ros::Duration(topic_obstacle_array_interval_);
		ma.markers.push_back(m);
	}
    pub_marker_array_.publish(ma);
}
