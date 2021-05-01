#include <ros/ros.h>
#include "auto_drive_base.h"
#include "utils.hpp"
#include "utils_new.hpp"

class PathTracking : public AutoDriveBase
{
public:
	PathTracking();
	virtual ~PathTracking(){};

	virtual bool init(ros::NodeHandle nh,ros::NodeHandle nh_private) override;
	virtual bool start() override;
	virtual void stop() override;
	virtual bool isRunning() override;

	void setExpectSpeed(const float& speed);
	void setOffset(const float& offset);

private:
	void timer_callback(const ros::TimerEvent&);

	float generateRoadwheelAngleByRadius(const float& radius);
	float limitRoadwheelAngleBySpeed(const float& angle,
                                               const float& speed);
	float generateMaxSpeedByCurvature(const Path& path,
                                                const size_t& begin_idx,
											    const float& curvature_search_distance);
	float limitSpeedByParkingPoint(const float& speed);
	
private:
	ros::Timer cmd_timer_;

	float expect_speed_;
	float offset_;
	
	float max_match_distance_; // 定位自车在路径位置的容许距离误差
	float fd_speed_coefficient_;
	float fd_lateral_error_coefficient_;
	float min_foresight_distance_; // 最小前视距离
	float max_side_acceleration_; // 自车最大侧向加速度
	float max_deceleration_; // 自车最大减速度

};
