#include "maximo_coding_test/sar_base.hpp"

SARBot::SARBot(ros::NodeHandle& nh, std::string name, geometry_msgs::Pose start_location, std::function<void(void)> goal_reached_cb) 
    : finished_(false),  
      goal_sent_(false), 
      goal_reached_cb_(goal_reached_cb),
      move_client_(name+"/move_base", true),
      nh_(nh)
{
    while (!move_client_.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up for explorer");
    }
    setGoal(start_location);
}

bool SARBot::isGoalRunning() 
{
    return goal_sent_.load();
}

bool SARBot::isFinished() 
{
    return finished_.load();
}

void SARBot::sendNextGoal() 
{
    std::lock_guard<std::mutex> lock(send_goal_m_);

    if(goal_sent_.load()) {
        ROS_WARN("Move goal already in progress");
        return;
    }

    goal_sent_.store(true);

    if(goals_.empty()) {
        ROS_WARN("Send next goal requested with no goals left");
        finished_.store(true);
        return;
    }

    move_client_.sendGoal(goals_.back(),
                        [this](const actionlib::SimpleClientGoalState& state,
                                const move_base_msgs::MoveBaseResultConstPtr& result) {
                                    this->doneCb(state, result);
                        },
                        [](){},
                        [](const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){} );
}

void SARBot::setGoal(geometry_msgs::Pose goal)
{
    goals_.push_back(move_base_msgs::MoveBaseGoal{});
    goals_.back().target_pose.header.frame_id = "map";
    goals_.back().target_pose.header.stamp = ros::Time::now();
    goals_.back().target_pose.pose = goal;
}


void SARBot::doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    if (state  == actionlib::SimpleClientGoalState::SUCCEEDED) {
        goals_.pop_back();
    }

    if(goal_reached_cb_ && goals_.size()) {
        goal_reached_cb_();
    }

    if (goals_.empty()) {
        finished_.store(true);
    }

    goal_sent_.store(false);
}