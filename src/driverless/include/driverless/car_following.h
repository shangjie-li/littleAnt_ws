#ifndef CAR_FOLLOWING_H_
#define CAR_FOLLOWING_H_

#include <ros/ros.h>
#include "auto_drive_base.h"
#include "utils.hpp"
#include "utils_new.hpp"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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
	virtual bool isRunning() override;
	
	void setMaxSpeed(const float& speed);

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
	void computeObstacleOrientation(const perception_msgs::Obstacle& obs,
                                              double& phi);

	void transformSensor2Base(double& phi);

	void transformSensor2Base(double& x,
                                        double& y);
	void transformSensor2Base(double xs[4],
                                        double ys[4]);
	void transformBase2Gps(double& x,
                                     double& y);
	void transformBase2Gps(double xs[4],
                                     double ys[4]);
	void transformGps2Global(double& x,
                                       double& y);
	void transformGps2Global(double xs[4],
                                       double ys[4]);

	double computeObstacleDistance2Ego(const perception_msgs::Obstacle& obs);
	double computeObstacleSpeed(const perception_msgs::Obstacle& obs);
	void publishMarkerArray(const perception_msgs::ObstacleArray::ConstPtr obstacles,
                                      const bool& obs_in_path,
									  const size_t& nearest_obs_idx);
	
private:
	std::string sub_topic_obstacle_array_;
	std::string pub_topic_marker_array_;
	std::string marker_array_frame_id_;
	ros::Subscriber sub_obstacle_array_;
	ros::Publisher pub_marker_array_;
	ros::Timer cmd_timer_;
	
	float max_following_speed_; // 自车最大速度
	float max_deceleration_; // 自车最大减速度
	float safe_margin_; // 安全通过余量
	float dangerous_distance_; // 危险距离
	
	float max_search_distance_; // 最大搜索距离
	float min_search_distance_; // 最小搜索距离
	float accelerate_coefficient_;
	float decelerate_coefficient_;

	double cmd_time_; // 指令更新时间
	
	double cmd_interval_threshold_; // 指令更新时间间隔阈值
	double topic_obstacle_array_interval_; // 话题时间间隔
	int obstacle_repeat_threshold_; // 目标重复检测次数阈值

	// sensor系原点在base系下的坐标
	// sensor系X轴相对base系X轴的夹角，逆时针为正
	float dx_sensor2base_;
	float dy_sensor2base_;
	float phi_sensor2base_;

	// base系原点在gps系下的坐标
	// base系X轴相对gps系X轴的夹角，逆时针为正
	float dx_base2gps_;
	float dy_base2gps_;
	float phi_base2gps_;

	// gps系原点在global系下的坐标
	// gps系X轴相对global系X轴的夹角，逆时针为正
	float dx_gps2global_;
	float dy_gps2global_;
	float phi_gps2global_;

};

#endif

