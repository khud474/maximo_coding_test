#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SARBot {
public:
    SARBot(ros::NodeHandle& nh, std::string name, geometry_msgs::Pose start_location, std::function<void(void)> goal_reached_cb);
    bool isGoalRunning();
    bool isFinished();
    void sendNextGoal();
    void setGoal(geometry_msgs::Pose goal);

private:
    void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);

    std::atomic<bool> goal_sent_;
    std::atomic<bool> finished_;
    std::function<void(void)> goal_reached_cb_;
    MoveBaseClient move_client_;
    ros::NodeHandle nh_;
    std::mutex send_goal_m_;
    std::vector<move_base_msgs::MoveBaseGoal> goals_;


};