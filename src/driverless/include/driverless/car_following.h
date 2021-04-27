#ifndef CAR_FOLLOWING_H_
#define CAR_FOLLOWING_H_

#include <ros/ros.h>
#include "auto_drive_base.h"
#include "utils.hpp"
#include "utils_new.hpp"

#include <perception_msgs/Obstacle.h>
#include <perception_msgs/ObstacleArray.h>

class CarFollowing : public AutoDriveBase
{
public:
	CarFollowing();
	virtual ~CarFollowing(){};
	
	virtual bool init(ros::NodeHandle nh, ros::NodeHandle nh_private) override;
	virtual bool start() override;
	virtual void stop() override;

private:
	void timer_callback(const ros::TimerEvent&);
	void obstacles_callback(const perception_msgs::ObstacleArray::ConstPtr& objects);

	bool isObstacleInPath(const perception_msgs::Obstacle& obs,
									const double& margin,
									const Path& path,
									const size_t& nearest_idx,
									const size_t& farthest_idx);
	bool isPathThroughObstacle(const perception_msgs::Obstacle& obs,
									     const Path& path,
									     const size_t& nearest_idx,
									     const size_t& farthest_idx);

	void computeObstacleCenter(const perception_msgs::Obstacle& obs,
										 double& obs_x,
										 double& obs_y);
	void computeObstacleVertex(const perception_msgs::Obstacle& obs,
                                         double obs_xs[4],
										 double obs_ys[4]);

	void transform2DPoint(double& x,
                                    double& y,
									const double& phi,
									const double& x0,
									const double& y0);
	void transform2DPoints(double xs[4],
                                     double ys[4],
									 const double& phi,
									 const double& x0,
									 const double& y0);

	void transformSensor2Global(double& x,
                                          double& y);
	void transformSensor2Global(double xs[4],
                                          double ys[4]);
	
	size_t findNearestObstacle(std::vector<perception_msgs::Obstacle>& obstacles);
	double computeObstacleDistance2Ego(const perception_msgs::Obstacle& obs);
	double computeObstacleSpeed(const perception_msgs::Obstacle& obs);
	
private:
	std::string sub_topic_obstacle_array_;
	ros::Subscriber sub_obstacle_array_;
	ros::Timer cmd_timer_;
	
	float max_speed_; // 自车最大速度（km/h）
	float max_deceleration_; // 自车最大减速度（m/s2）
	float safe_margin_; // 安全通过余量（m）
	float dangerous_distance_; // 危险距离
	
	float max_following_distance_; // 最大跟驰距离
	float min_following_distance_; // 最小跟驰距离

	size_t dest_index_; // 终点索引
	double cmd_time_; // 指令更新时间
	
	double cmd_interval_threshold_; // 指令更新时间间隔阈值
	double topic_obstacle_array_interval_; // 话题时间间隔
	int obstacle_repeat_threshold_; // 目标重复检测次数阈值

	float dx_sensor2gps_;
	float dy_sensor2gps_;
	float phi_sensor2gps_;

	float dx_gps2global_;
	float dy_gps2global_;
	float phi_gps2global_;

};

#endif

