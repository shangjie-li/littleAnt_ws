#ifndef AVOIDING_H_
#define AVOIDING_H_

#include <ros/ros.h>
#include "auto_drive_base.h"
#include "utils.hpp"
#include "utils_new.hpp"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <perception_msgs/Obstacle.h>
#include <perception_msgs/ObstacleArray.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

class Avoiding : public AutoDriveBase
{
public:
	Avoiding();
	virtual ~Avoiding(){};
	
	virtual bool init(ros::NodeHandle nh, ros::NodeHandle nh_private) override;
	virtual bool start() override;
	virtual void stop() override;
	virtual bool isRunning() override;

private:
    void cmd_timer_callback(const ros::TimerEvent&);
	void obstacles_callback(const perception_msgs::ObstacleArray::ConstPtr& obstacles);

	int judgeWhichSide(const double& x,
							 const double& y,
							 const Path& path,
							 const size_t& nearest_idx,
							 const size_t& farthest_idx);
	int judgeWhichSide(const perception_msgs::Obstacle& obs,
							 const Path& path,
							 const size_t& nearest_idx,
							 const size_t& farthest_idx);
	void getGlobalObstacle(const perception_msgs::Obstacle& obs,
								 double& x,
								 double& y);
	void getGlobalObstacle(const perception_msgs::Obstacle& obs,
								 double xs[4],
								 double ys[4]);
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
	double computeGapBetweenObstacleAndPath(const perception_msgs::Obstacle& obs,
												  const Path& path,
												  const size_t& nearest_idx,
												  const size_t& farthest_idx);
	void publishMarkerArray(const perception_msgs::ObstacleArray::ConstPtr obstacles,
                                  const bool& obs_in_path,
								  const size_t& nearest_obs_idx);
	void publishPath(ros::Publisher& pub,
                           const Path& path_to_pub,
						   const size_t& begin_idx,
						   const size_t& end_idx,
						   const std::string& frame_id);
	void transformGlobal2Gps(double& x,
                                   double& y);
	void transformGps2Base(double& x,
                                 double& y);

private:
	std::string sub_topic_obstacle_array_;

	std::string pub_topic_marker_array_;
	std::string pub_topic_global_path_;
	std::string pub_topic_local_path_;

	std::string marker_array_frame_id_;
	std::string global_path_frame_id_;
	std::string local_path_frame_id_;

	ros::Subscriber sub_obstacle_array_;

	ros::Publisher pub_marker_array_;
	ros::Publisher pub_global_path_;
	ros::Publisher pub_local_path_;

	ros::Timer cmd_timer_;

	float max_match_distance_; // 定位自车在路径位置的容许距离误差
	float max_deceleration_; // 自车最大减速度
	float min_following_distance_; // 最小跟随距离
	float max_search_range_; // 最大搜索范围
	float safe_margin_; // 安全通过余量
	float lane_left_width_;
	float lane_right_width_;

	float offset_;
	bool following_mode_;
	float nearest_obstacle_distance_;
	size_t nearest_obstacle_index_;
	
	double obstacle_array_time_; // 障碍物话题更新时间
	double obstacle_array_interval_threshold_; // 障碍物话题更新时间间隔阈值

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
