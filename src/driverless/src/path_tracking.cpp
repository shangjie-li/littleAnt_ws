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
	
	nh_private_.param<float>("fd_speed_coefficient", fd_speed_coefficient_, 1.8);
	nh_private_.param<float>("fd_lateral_error_coefficient", fd_lateral_error_coefficient_, 2.0);
	nh_private_.param<float>("min_foresight_distance", min_foresight_distance_, 5.0); // m
	nh_private_.param<float>("max_side_acceleration", max_side_acceleration_, 1.5); // m/s2
	nh_private_.param<float>("max_deceleration", max_deceleration_, 1.0); // m/s2
	
	// 设置默认值
	expect_speed_ = 3.0; // m/s

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

	cmd1_timer_ = nh_.createTimer(ros::Duration(0.03), &PathTracking::cmd1_timer_callback, this);
	cmd1_check_timer_ = nh_.createTimer(ros::Duration(0.10), &PathTracking::cmd1_check_timer_callback, this);
	
	cmd2_timer_ = nh_.createTimer(ros::Duration(0.100), &PathTracking::cmd2_timer_callback, this);
	cmd2_check_timer_ = nh_.createTimer(ros::Duration(1.00), &PathTracking::cmd2_check_timer_callback, this);

	return true;
}

void PathTracking::stop()
{
	cmd1_timer_.stop();
	cmd1_check_timer_.stop();
	cmd2_timer_.stop();
	cmd2_check_timer_.stop();
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

// 定时回调函数（cmd1）
// 控制指令长时间未更新，恢复默认状态
void PathTracking::cmd1_check_timer_callback(const ros::TimerEvent&)
{
	if(ros::Time::now().toSec() - cmd1_time_ > 2.0)
	{
		cmd_mutex_.lock();
		cmd_.turnLight = 0;
		cmd_mutex_.unlock();
	}
	
}

// 定时回调函数（cmd2）
// 控制指令长时间未更新，有效位置false
void PathTracking::cmd2_check_timer_callback(const ros::TimerEvent&)
{
	if(ros::Time::now().toSec() - cmd2_time_ > 0.2)
	{
		cmd_mutex_.lock();
		cmd_.validity = false;
		cmd_.speed_validity = false;
		cmd_mutex_.unlock();
	}
	
}

// 定时回调函数（cmd1）
void PathTracking::cmd1_timer_callback(const ros::TimerEvent&)
{
    // 读取局部路径
	local_path_.mutex.lock();
	const Path t_path = local_path_;
	local_path_.mutex.unlock();
	
	if(t_path.turn_ranges.size() == 0)
	{
	    cmd1_time_ = ros::Time::now().toSec();
		cmd_mutex_.lock();
		cmd_.turnLight = 0;
		cmd_mutex_.unlock();
	    return;
	}
	else
	{
	    // 只考虑最近一个转向区间
	    if(t_path.turn_ranges.ranges[0].start_index == 0) // 车辆处于转向区间内
	    {
	        static int cnt = 0;
	        if(cnt >= 10)
	        {
	            cnt = 0;
	            ROS_INFO("[%s] Set turn light.", __NAME__);
	        }
	        cnt++;
	            
	        cmd1_time_ = ros::Time::now().toSec();
		    cmd_mutex_.lock();
		    cmd_.turnLight = t_path.turn_ranges.ranges[0].type;
		    cmd_mutex_.unlock();
	        return;
	    }
	    else
	    {
	        cmd1_time_ = ros::Time::now().toSec();
		    cmd_mutex_.lock();
		    cmd_.turnLight = 0;
		    cmd_mutex_.unlock();
	        return;
	    }
	}
}

// 定时回调函数（cmd2）
void PathTracking::cmd2_timer_callback(const ros::TimerEvent&)
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

	// 转向角指令，单位度，右转为负，左转为正
	float t_angle_deg = 0.0;
	
	// 速度指令，单位m/s
	float t_speed_mps = expect_speed_;

	// 当路径过短时，紧急制动
	if(t_path.points.size() < 5)
	{
		cmd2_time_ = ros::Time::now().toSec();
		cmd_mutex_.lock();
		cmd_.validity = true;
		cmd_.speed_validity = true;
		cmd_.speed = 0.0;
		cmd_.brake = 100.0;
		cmd_.roadWheelAngle = 0.0;
		cmd_mutex_.unlock();

		return;
	}
	// 否则释放刹车
	else
	{
	    cmd2_time_ = ros::Time::now().toSec();
		cmd_mutex_.lock();
		cmd_.validity = true;
		cmd_.speed_validity = true;
		cmd_.brake = 0.0;
		cmd_mutex_.unlock();
	}
	
	// 当路径不包含停车点信息时，结束任务
	if(!t_path.park_points.available()) 
	{
		ROS_ERROR("[%s] No Next Parking Point.", __NAME__);
		
		cmd2_time_ = ros::Time::now().toSec();
		cmd_mutex_.lock();
		cmd_.validity = false;
		cmd_.speed_validity = false;
		cmd_.speed = 0.0;
		cmd_.roadWheelAngle = 0.0;
		cmd_mutex_.unlock();

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
	// 当接近路径终点时，借助路径的延伸阶段正常行驶
	size_t target_idx = nearest_idx + 1;
	GpsPoint target_point = t_path[target_idx];
	std::pair<float, float> dis_yaw = computeDisAndYaw(target_point, vehicle_pose);
	while(dis_yaw.first < foresight_distance)
	{
		target_idx++;
		
		// 防止索引溢出
		if(target_idx > t_path.points.size() - 1)
		{
			ROS_ERROR("[%s] Path tracking completed.", __NAME__);
			
			cmd2_time_ = ros::Time::now().toSec();
			cmd_mutex_.lock();
			cmd_.validity = false;
			cmd_.speed_validity = false;
			cmd_.speed = 0.0;
			cmd_.roadWheelAngle = 0.0;
			cmd_mutex_.unlock();
			
			return;
		}

		target_point = t_path[target_idx];
		dis_yaw = computeDisAndYaw(target_point, vehicle_pose);
	}

	// 航向偏差，单位rad，自车左偏为负，自车右偏为正，限制在[-M_PI, M_PI]
	float yaw_err = dis_yaw.second - vehicle_pose.yaw;
	if(yaw_err <= -M_PI) yaw_err += 2 * M_PI;
	if(yaw_err > M_PI) yaw_err -= 2 * M_PI;
	
	// 当航向偏差为0时，转向角指令置0，不修改速度指令
	if(sin(yaw_err) == 0)
	{
		cmd2_time_ = ros::Time::now().toSec();
		cmd_mutex_.lock();
		cmd_.validity = true;
		cmd_.speed_validity = true;
		cmd_.roadWheelAngle = 0.0;
		cmd_mutex_.unlock();

		return;
	}

	// 计算转弯半径
	float turning_radius = (0.5 * dis_yaw.first) / sin(yaw_err);
	
	t_angle_deg = generateRoadwheelAngleByRadius(turning_radius);
	t_angle_deg = limitRoadwheelAngleBySpeed(t_angle_deg, vehicle_speed);
	
	float curvature_search_distance = vehicle_speed * vehicle_speed / (2 * max_deceleration_);
	float max_speed_by_curve = generateMaxSpeedByCurvature(t_path, nearest_idx, curvature_search_distance);
	t_speed_mps = t_speed_mps < max_speed_by_curve ? t_speed_mps : max_speed_by_curve;

	float max_speed_by_park = generateMaxSpeedByParkingPoint(t_path);
	t_speed_mps = t_speed_mps < max_speed_by_park ? t_speed_mps : max_speed_by_park;

	cmd2_time_ = ros::Time::now().toSec();
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
		ROS_INFO("[%s] cmd_v:%.2fkm/h\t true_v:%.2fm/s\t max_decel:%.2f",
		    __NAME__, cmd_.speed, vehicle_speed, max_deceleration_);
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
		ROS_INFO("[%s] Keep parking.", __NAME__);
		return 0.0;
	}
	
	// 计算与停车点距离
	float dis = computeDistance(path.points[path.pose_index], path.points[path.park_points.points[0].index]);
	
	// 计算容许速度
	float max_speed = sqrt(2 * max_deceleration_ * dis);

	return max_speed;
}
