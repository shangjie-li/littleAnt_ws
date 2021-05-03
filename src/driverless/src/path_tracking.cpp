#include "driverless/path_tracking.h"
#define __NAME__ "path_tracking"

PathTracking::PathTracking():
	AutoDriveBase(__NAME__)
{
}

bool PathTracking::init(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
	nh_ = nh;
	nh_private_ = nh_private;
	
	nh_private_.param<float>("expect_speed", expect_speed_, 3.0); // m/s
	nh_private_.param<float>("offset", offset_, 0.0); // m

	nh_private_.param<double>("cmd_interval_threshold", cmd_interval_threshold_, 0.2);
	
	nh_private_.param<float>("fd_speed_coefficient", fd_speed_coefficient_, 1.8);
	nh_private_.param<float>("fd_lateral_error_coefficient", fd_lateral_error_coefficient_, 2.0);
	nh_private_.param<float>("min_foresight_distance", min_foresight_distance_, 5.0); // m
	nh_private_.param<float>("max_side_acceleration", max_side_acceleration_, 1.5); // m/s2
	nh_private_.param<float>("max_deceleration", max_deceleration_, 1.0); // m/s2
	
	is_ready_ = true;
	
	return true;
}

// 启动跟踪线程
bool PathTracking::start()
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

	cmd_timer_ = nh_.createTimer(ros::Duration(0.03), &PathTracking::cmd_timer_callback, this);
	cmd_check_timer_ = nh_.createTimer(ros::Duration(0.10), &PathTracking::cmd_check_timer_callback, this);

	return true;
}

void PathTracking::stop()
{
	cmd_timer_.stop();
	cmd_check_timer_.stop();
	is_running_ = false;
	
	cmd_mutex_.lock();
	cmd_.speed = 0.0;
	cmd_.roadWheelAngle = 0.0;
	cmd_.validity = false;
	cmd_.speed_validity = false;
	cmd_mutex_.unlock();
}

bool PathTracking::isRunning()
{
	return is_running_;
}

void PathTracking::setExpectSpeed(const float& speed)
{
	// speed单位km/h
	expect_speed_ = speed / 3.6;
}

void PathTracking::setOffset(const float& offset)
{
    offset_ = offset;
}

// 定时回调函数
void PathTracking::cmd_timer_callback(const ros::TimerEvent&)
{
	if(!is_ready_) return;
	
	static size_t cnt = 0;
	
	// 读取车辆状态，创建副本避免多次读取
	const Pose vehicle_pose = vehicle_state_.getPose(LOCK);
	const float vehicle_speed = vehicle_state_.getSpeed(LOCK);

	// 读取局部路径
	local_path_.mutex.lock();
	const Path t_path = local_path_;
	local_path_.mutex.unlock();
	
	if(!t_path.park_points.available()) 
	{
		ROS_ERROR("[%s] No Next Parking Point.", __NAME__);
		return;
	}

	// 选择路径中自车所在点和终点
	size_t nearest_idx = t_path.pose_index;
	size_t farthest_idx = t_path.final_index;

	// 横向偏差，单位m
	float lat_err = findMinDistance2Path(t_path, vehicle_pose.x, vehicle_pose.y, nearest_idx, nearest_idx, farthest_idx);
	
	// 计算前视距离（预瞄距离）
	// vehicle.speed单位m/s
	float foresight_distance = fd_speed_coefficient_ * vehicle_speed + fd_lateral_error_coefficient_ * fabs(lat_err);
	if(foresight_distance < min_foresight_distance_) foresight_distance  = min_foresight_distance_;

	// 寻找满足foresight_distance的目标点作为预瞄点
	// 计算自车当前点与预瞄点的dis和yaw
	size_t target_idx = nearest_idx + 1;
	GpsPoint target_point = t_path[target_idx];
	target_point = computePointOffset(target_point, offset_);
	std::pair<float, float> dis_yaw = computeDisAndYaw(target_point, vehicle_pose);
	while(dis_yaw.first < foresight_distance)
	{
		target_idx++;
		
		// 当接近路径终点时，借助路径的延伸阶段正常行驶
		if(target_idx > t_path.points.size() - 1)
		{
			ROS_ERROR("[%s] Path tracking completed.", __NAME__);
			return;
		}

		target_point = t_path[target_idx];
		target_point = computePointOffset(target_point, offset_);
		dis_yaw = computeDisAndYaw(target_point, vehicle_pose);
	}

	// 航向偏差，单位rad，自车左偏为负，自车右偏为正，限制在[-M_PI, M_PI]
	float yaw_err = dis_yaw.second - vehicle_pose.yaw;
	if(yaw_err <= -M_PI) yaw_err += 2 * M_PI;
	if(yaw_err > M_PI) yaw_err -= 2 * M_PI;

	// 计算转弯半径
	if(sin(yaw_err) == 0) return;
	float turning_radius = (0.5 * dis_yaw.first) / sin(yaw_err);
	
	// 转向角指令，单位度，右转为负，左转为正
	float t_angle_deg;
	t_angle_deg = generateRoadwheelAngleByRadius(turning_radius);
	t_angle_deg = limitRoadwheelAngleBySpeed(t_angle_deg, vehicle_speed);
	
	// 速度指令，单位m/s
	float t_speed_mps = expect_speed_;
	
	float curvature_search_distance = vehicle_speed * vehicle_speed / (2 * max_deceleration_);
	float max_speed_by_curve = generateMaxSpeedByCurvature(t_path, nearest_idx, curvature_search_distance);
	t_speed_mps = t_speed_mps < max_speed_by_curve ? t_speed_mps : max_speed_by_curve;

	float max_speed_by_park = generateMaxSpeedByParkingPoint(t_path);
	t_speed_mps = t_speed_mps < max_speed_by_park ? t_speed_mps : max_speed_by_park;

	cmd_time_ = ros::Time::now().toSec();
	cmd_mutex_.lock();
	cmd_.validity = true;
	cmd_.speed_validity = true;
	cmd_.speed = t_speed_mps * 3.6;
	cmd_.roadWheelAngle = t_angle_deg;
	cmd_mutex_.unlock();
	
	if((cnt++) % 50 == 0)
	{
		ROS_INFO("[%s]",
		    __NAME__);
		ROS_INFO("[%s] cmd_v:%.2fkm/h\t true_v:%.2fm/s\t offset:%.2f\t max_decel:%.2f",
		    __NAME__, cmd_.speed, vehicle_speed, offset_, max_deceleration_);
		ROS_INFO("[%s] exp_v:%.2fm/s\t curve_v:%.2fm/s\t park_v:%.2fm/s\t t_speed:%.2fm/s\t t_angle:%.2f",
			__NAME__, expect_speed_, max_speed_by_curve, max_speed_by_park, t_speed_mps, t_angle_deg);
		ROS_INFO("[%s] yaw:%.2f\t dis_yaw.f:%.2f\t dis_yaw.s:%.2f",
		    __NAME__, vehicle_pose.yaw, dis_yaw.first, dis_yaw.second);
		ROS_INFO("[%s] yaw_err:%.2f\t lat_err:%.2f\t foresight_distance:%.2f",
		    __NAME__, yaw_err, lat_err, foresight_distance);
		ROS_INFO("[%s] nearest_idx:%lu\t target_idx:%lu\t final_index:%lu",
		    __NAME__, nearest_idx, target_idx, t_path.final_index);
		ROS_INFO("[%s] pose:(%.2f, %.2f)\t nearest:(%.2f, %.2f)\t target:(%.2f, %.2f)",
			__NAME__, vehicle_pose.x, vehicle_pose.y, t_path[nearest_idx].x, t_path[nearest_idx].y, target_point.x, target_point.y);
			
	}
	
}

// 定时回调函数
// 控制指令长时间未更新，有效位置false
void PathTracking::cmd_check_timer_callback(const ros::TimerEvent&)
{
	if(ros::Time::now().toSec() - cmd_time_ > cmd_interval_threshold_)
	{
		cmd_mutex_.lock();
		cmd_.validity = false;
		cmd_.speed_validity = false;
		cmd_mutex_.unlock();
	}
}

float PathTracking::generateRoadwheelAngleByRadius(const float& radius)
{
	assert(radius != 0);
	return atan(vehicle_params_.wheel_base / radius) * 180 / M_PI;
}

float PathTracking::limitRoadwheelAngleBySpeed(const float& angle,
                                               const float& speed)
{
	float result = angle;
	float min_steering_radius = speed * speed / max_side_acceleration_;
	
	if(min_steering_radius < 1.0)
		return result;
	
	float max_angle = fabs(generateRoadwheelAngleByRadius(min_steering_radius));
	if(max_angle > vehicle_params_.max_roadwheel_angle)
	   max_angle = vehicle_params_.max_roadwheel_angle;
	
	if(result > max_angle) result = max_angle;
	else if(result < -max_angle) result = -max_angle;

	return result;
}

float PathTracking::generateMaxSpeedByCurvature(const Path& path,
                                                const size_t& begin_idx,
											    const float& curvature_search_distance)
{
	float sum_dis = 0.0;
	float max_cur = 0.0;
	float now_cur;
	for(size_t i = begin_idx; i < path.points.size() - 1; i++)
	{
		now_cur = fabs(path.points[i].curvature);
		if(max_cur < now_cur) max_cur = now_cur;

		sum_dis	+= computeDistance(path.points[i].x, path.points[i].y, path.points[i + 1].x, path.points[i + 1].y);
		if(sum_dis >= curvature_search_distance) break;
	}

	if(max_cur == 0.0) return expect_speed_;
	return sqrt(1.0 / fabs(max_cur) * max_side_acceleration_);
}

float PathTracking::generateMaxSpeedByParkingPoint(const Path& path)
{
	// 只考虑最近一个停车点
	// 如果当前正在停车中，速度置0
	if(path.park_points.points[0].isParking)
	{
		return 0.0;
	}
	
	// 计算与停车点距离
	float dis2park = computeDistance(path.points[path.pose_index], path.points[path.park_points.points[0].index]);
	
	// 计算容许速度
	float max_speed = sqrt(2 * max_deceleration_ * dis2park);

	return max_speed;
}
