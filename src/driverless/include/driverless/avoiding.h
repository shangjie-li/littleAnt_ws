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

class Avoiding : public AutoDriveBase
{
public:
	Avoiding();
	virtual ~Avoiding(){};
	
	virtual bool init(ros::NodeHandle nh, ros::NodeHandle nh_private) override;
	virtual bool start() override;
	virtual void stop() override;
	virtual bool isRunning() override;

	void setLocalPath();
	bool updateStatus(const GpsPoint& pose,const float& speed, const float& roadWheelAngle);
private:
    void setLocalPath();
	void avoidingThread();
	void publishDiagnostics(uint8_t level,const std::string& msg);

private:
	enum object_type_t
	{
		Unknown = 0,
		Person = 1,
		Vehicle = 2
	};

	void get_obstacle_msg(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects,
						  size_t objectIndex,
						  whatArea_t *obstacleArea,
						  float ** obstacleVertex_x_y,
						  float *obstacleDistance, 
						  size_t *obstacleIndex,
						  size_t &obstacleSequence);
								   
	whatArea_t which_area(float& x,float& y);

	float deceleration_2_brakingAperture(const float & deceleration);

	float brakingAperture_2_deceleration(const float & brakingAperture);

	void bubbleSort(const float * distance, size_t * index, size_t length);
	
	float calculate_dis2path(const double& X_,const double& Y_);
	float dis2path2(const double& X_,const double& Y_);
	std::pair<double,double> vehicleToWorldCoordination(float x,float y);
	void decision(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
				const float dis2vehicleArray[],const size_t indexArray[],const float dis2pathArray[],const int n_object);
	bool is_backToOriginalLane(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
						const float dis2vehicleArray[],const size_t indexArray[],const float dis2pathArray[],const int& n_object);
						
	bool is_dangerous(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& objects, 
					const float dis2vehicleArray[],const size_t indexArray[],const float dis2pathArray[],const int& n_object);

	void backToOriginalLane();
	
private:
	ros::Subscriber sub_objects_;
	ros::Publisher  pub_diagnostic_;
	diagnostic_msgs::DiagnosticStatus diagnostic_msg_;
	
	std::string objects_topic_;
	std::vector<GpsPoint> path_points_;
	float path_points_resolution_;

	std::mutex cmd_mutex_;
	controlCmd_t cmd_;
	
	float max_deceleration_;
	
	float safety_distance_front_;
	float safety_distance_side_ ;
	
	float danger_distance_front_;
	float danger_distance_side_;
	
	float vehicle_speed_; //m/s
	float deceleration_cofficient_;
	
	size_t target_point_index_;
	size_t nearest_point_index_;
	
	GpsPoint current_point_,target_point_;
	
	float avoiding_offest_;
	
	float maxOffset_right_,maxOffset_left_;
	bool is_ready_; //是否准备就绪
	bool is_running_;
	
};




#endif
