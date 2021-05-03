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
	void computePathCurvature(Path& path);
	void computeExtendingPath(Path& path,
						            const float& extending_dis);

	void transformGlobal2Gps(double& x,
                                   double& y);
	void transformGps2Base(double& x,
                                 double& y);

	void publishPath(ros::Publisher& pub,
                           const Path& path_to_pub,
						   const size_t& begin_idx,
						   const size_t& end_idx,
						   const std::string& frame_id);
	
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
	float local_path_length_; // 局部路径长度

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
