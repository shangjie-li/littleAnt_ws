#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "ant_actions/DoDriverlessTaskAction.h"   // Note: "Action" is appended

class GlobalPathPlanning
{
public:
    typedef actionlib::SimpleActionClient<ant_actions::DoDriverlessTaskAction> DoDriverlessTaskClient;
    
    GlobalPathPlanning();
    ~GlobalPathPlanning(){};
    
    bool init(ros::NodeHandle nh, ros::NodeHandle nh_private);
    bool start();
    
private:
    void taskDoneCallback(const actionlib::SimpleClientGoalState& state,
                                          const ant_actions::DoDriverlessTaskResultConstPtr& result);
    void taskFeedbackCallback(const ant_actions::DoDriverlessTaskFeedbackConstPtr& feedback);
    void taskActivedCallback();

private:
    ros::NodeHandle nh_, nh_private_;
    bool is_ready_;

    int task_;
    int type_;
    
    float path_resolution_;
    float expect_speed_;
    std::string roadnet_file_;
    bool path_filp_;
    
    DoDriverlessTaskClient* ac_;
    
};
