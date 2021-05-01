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
	
	nh_private_.param<float>("max_match_distance", max_match_distance_, 10.0); // m
	nh_private_.param<float>("fd_speed_coefficient", fd_speed_coefficient_, 1.8);
	nh_private_.param<float>("fd_lateral_error_coefficient", fd_lateral_error_coefficient_, 2.0);
	nh_private_.param<float>("min_foresight_distance", min_foresight_distance_, 5.0); // m
	nh_private_.param<float>("max_side_acceleration", max_side_acceleration_, 1.5); // m/s2
	nh_private_.param<float>("max_deceleration", max_deceleration_, 0.3); // m/s2

	pub_local_path_ = nh_private_.advertise<nav_msgs::Path>("/local_path", 2);

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

	cmd_timer_ = nh_.createTimer(ros::Duration(0.03), &PathTracking::timer_callback, this);

	return true;
}

void PathTracking::stop()
{
	cmd_timer_.stop();
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
void PathTracking::timer_callback(const ros::TimerEvent&)
{
	if(!is_ready_) return;
	
	static size_t cnt = 0;
	
	// 读取车辆状态，创建副本避免多次读取
	const VehicleState vehicle = vehicle_state_;

	// 寻找当前车辆位置对应的路径点，更新pose_index到global_path_
	size_t nearest_idx = findNearestPointInPath(global_path_, vehicle.pose, max_match_distance_);
	global_path_.pose_index = nearest_idx;

	// 如果当前点与路径终点过近，结束跟踪
	if(nearest_idx > global_path_.final_index - 5)
	{
		ROS_ERROR("[%s] Path tracking completed.", __NAME__);
		is_running_ = false; // 该变量的状态被其他节点监听
		return;
	}

	// 横向偏差，单位m
	float lat_err = findMinDistance2Path(global_path_, vehicle.pose.x, vehicle.pose.y, nearest_idx, nearest_idx, global_path_.final_index);
	
	// 计算前视距离（预瞄距离）
	// vehicle.speed单位m/s
	float foresight_distance = fd_speed_coefficient_ * vehicle.speed + fd_lateral_error_coefficient_ * fabs(lat_err);
	if(foresight_distance < min_foresight_distance_) foresight_distance  = min_foresight_distance_;

	// 寻找满足foresight_distance的目标点作为预瞄点
	// 计算自车当前点与预瞄点的dis和yaw
	size_t target_idx = nearest_idx + 1;
	GpsPoint target_point = global_path_[target_idx];
	target_point = computePointOffset(target_point, offset_);
	std::pair<float, float> dis_yaw = computeDisAndYaw(target_point, vehicle.pose);
	while(dis_yaw.first < foresight_distance)
	{
		target_point = global_path_[++target_idx];
		target_point = computePointOffset(target_point, offset_);
		dis_yaw = computeDisAndYaw(target_point, vehicle.pose);
	}

	// 航向偏差，单位rad，自车左偏为负，自车右偏为正，限制在[-M_PI, M_PI]
	float yaw_err = dis_yaw.second - vehicle.pose.yaw;
	if(yaw_err <= -M_PI) yaw_err += 2 * M_PI;
	if(yaw_err > M_PI) yaw_err -= 2 * M_PI;

	// 计算转弯半径
	if(sin(yaw_err) == 0) return;
	float turning_radius = (0.5 * dis_yaw.first) / sin(yaw_err);
	
	// 转向角指令，单位度，右转为负，左转为正
	float t_angle_deg;
	t_angle_deg = generateRoadwheelAngleByRadius(turning_radius);
	t_angle_deg = limitRoadwheelAngleBySpeed(t_angle_deg, vehicle.speed);
	
	// 速度指令，单位m/s
	float t_speed_mps = expect_speed_;
	
	float curvature_search_distance = vehicle.speed * vehicle.speed / (2 * max_deceleration_);
	float max_speed_by_curve = generateMaxSpeedByCurvature(global_path_, nearest_idx, curvature_search_distance);
	t_speed_mps = t_speed_mps < max_speed_by_curve ? t_speed_mps : max_speed_by_curve;

	float max_speed_by_park = limitSpeedByParkingPoint(t_speed_mps);
	t_speed_mps = t_speed_mps < max_speed_by_park ? t_speed_mps : max_speed_by_park;

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
		ROS_INFO("[%s] cmd_v:%.2fkm/h\t true_v:%.2fm/s\t offset:%.2f",
		    __NAME__, cmd_.speed, vehicle.speed, offset_);
		ROS_INFO("[%s] exp_v:%.2fm/s\t curve_v:%.2fm/s\t park_v:%.2fm/s\t t_speed:%.2fm/s\t t_angle:%.2f",
			__NAME__, expect_speed_, max_speed_by_curve, max_speed_by_park, t_speed_mps, t_angle_deg);
		ROS_INFO("[%s] yaw:%.2f\t dis_yaw.f:%.2f\t dis_yaw.s:%.2f",
		    __NAME__, vehicle.pose.yaw, dis_yaw.first, dis_yaw.second);
		ROS_INFO("[%s] yaw_err:%.2f\t lat_err:%.2f\t foresight_distance:%.2f",
		    __NAME__, yaw_err, lat_err, foresight_distance);
		ROS_INFO("[%s] nearest_idx:%lu\t target_idx:%lu\t final_index:%lu",
		    __NAME__, nearest_idx, target_idx, global_path_.final_index);
		ROS_INFO("[%s] pose:(%.2f, %.2f)\t nearest:(%.2f, %.2f)\t target:(%.2f, %.2f)",
			__NAME__, vehicle.pose.x, vehicle.pose.y, global_path_[nearest_idx].x, global_path_[nearest_idx].y, target_point.x, target_point.y);
		
		publishLocalPath();
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

	return sqrt(1.0 / fabs(max_cur) * max_side_acceleration_);
}

float PathTracking::limitSpeedByParkingPoint(const float& speed)
{
	// 没有停车点，立即停车
	if(!global_path_.park_points.available())
	{
		ROS_ERROR("[%s] No Next Parking Point.", __NAME__);
		return 0.0;
	}
	
	// 更新停车点，确保停车点在路径中的索引大于自车位置索引
	while(global_path_.park_points.next().index < global_path_.pose_index)
	{
		global_path_.park_points.next_index++;
		
		// 没有停车点，立即停车
		if(!global_path_.park_points.available())
			ROS_ERROR("[%s] No Next Parking Point.", __NAME__);
			return 0.0;
	}
		
	ParkingPoint& parking_point = global_path_.park_points.next();
	
	// 正在停车中
	if(parking_point.isParking)
	{
		// 停车周期为0，到达终点 
		if(parking_point.parkingDuration == 0.0)
			return 0.0;
		
		// 停车超时，移除当前停车点，下次利用下一停车点限速
		if(ros::Time::now().toSec() - parking_point.parkingTime >= parking_point.parkingDuration)
		{
			ROS_INFO("[%s] Parking overtime, parking point:%lu", __NAME__, parking_point.index);
			return speed;
		}
		
		// 正在停车，时间未到
		return 0.0;
	}
	
	// 计算与停车点距离
	float dis2end;
	if(global_path_.pose_index >= parking_point.index)
	{
		dis2end = 0.0;
	}
	else
	{
		double x1 = global_path_[parking_point.index].x;
		double y1 = global_path_[parking_point.index].y;
		double x2 = global_path_[global_path_.pose_index].x;
		double y2 = global_path_[global_path_.pose_index].y;
		dis2end = computeDistance(x1, y1, x2, y2);
	}

	float max_speed = sqrt(2 * max_deceleration_ * dis2end);
	
	// 到达停车点附近，速度置0，防止抖动
	if(dis2end < 0.2)
	{
		parking_point.parkingTime = ros::Time::now().toSec();
		parking_point.isParking = true;
		ROS_INFO("[%s] Start parking, parking point:%lu", __NAME__, parking_point.index);
		return 0.0;
	}

	return speed < max_speed ? speed : max_speed;
}

void PathTracking::publishLocalPath()
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
