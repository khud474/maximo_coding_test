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

/**
 * Class that provides convenient wrapper for move_base action
 */
class SARBot {
public:
    /**
    * SARBot constructor
    * @param nh ros node handle
    * @param name base name for move client
    * @param start_location starting pose for robot
    * @param goal_reached_cb user provided callback called when move goal completes successfully
    */
    SARBot(ros::NodeHandle& nh, std::string name, geometry_msgs::Pose start_location, std::function<void(void)> goal_reached_cb);
    /**
    * getter for private goal_sent_ var
    */
    bool isGoalRunning();
    /**
    * getter for private finished_
    */
    bool isFinished();
    /**
    * sends next goal from stored goals_ vector to move server
    */
    void sendNextGoal();
    /**
    * adds goal to goals_ vector
    * @param goal pose to be added to goals_ vector 
    */
    void setGoal(geometry_msgs::Pose goal);

private:
    /**
    * called on move action return. on success removes goal from goals_ and calls user supplied callback
    * @param state indicates success or failure of action call
    * @param result returned result of move action call
    */
    void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);

    std::atomic<bool> goal_sent_;
    std::atomic<bool> finished_;
    std::function<void(void)> goal_reached_cb_;
    MoveBaseClient move_client_;
    ros::NodeHandle nh_;
    std::mutex send_goal_m_;
    std::vector<move_base_msgs::MoveBaseGoal> goals_;


};