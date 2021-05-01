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
    void timer_callback(const ros::TimerEvent&);
	void obstacles_callback(const perception_msgs::ObstacleArray::ConstPtr& objects);

	void publishLocalPath();
	
private:
	std::string sub_topic_obstacle_array_;
	std::string pub_topic_marker_array_;
	std::string marker_array_frame_id_;
	ros::Subscriber sub_obstacle_array_;
	ros::Publisher pub_marker_array_;
	ros::Publisher pub_local_path_;
	ros::Timer cmd_timer_;
	
};

#endif
