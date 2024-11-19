#include <maximo_coding_test/follower.hpp>

Follower::Follower(ros::NodeHandle& nh, double target_distance)
    : goals_received_(false),
      nh_(nh),
      target_distance_(target_distance)
{
    geometry_msgs::Pose start_pose;
    start_pose.position.x = -4.0;
    start_pose.position.y = 3.5;
    start_pose.orientation.w = 1.0;
    sar_bot_ = std::make_unique<SARBot>(nh_, "follower", start_pose, [](){});

    follower_goals_service_ = nh_.advertiseService( 
        "follower_goals", &Follower::getGoals, this
        );
}

void Follower::spin() 
{
    ros::Rate loop_rate(10);
    
    while (ros::ok() && !goals_received_.load()) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok()) {
        if(!sar_bot_->isGoalRunning() && !sar_bot_->isFinished()) 
        {
            ROS_INFO("Sending goal for follower");
            sar_bot_->sendNextGoal();
        }

        if(sar_bot_->isFinished()) 
        {
            ROS_INFO("Following complete.");
            break;
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    return;
    
}

bool Follower::getGoals(maximo_coding_test::FollowerGoalsRequest& req, maximo_coding_test::FollowerGoalsResponse& resp)
{
    if(req.marker_frames.size() != req.explorer_frames.size()) 
    {
        ROS_ERROR("Received different number of marker and explorer frames");
        resp.success = false;
        return false;
    }

    for(int i=0; i<req.marker_frames.size(); i++) 
    {
        Eigen::Vector3d marker_loc{req.marker_frames[i].translation.x, req.marker_frames[i].translation.y, req.marker_frames[i].translation.z};
        Eigen::Vector3d explorer_loc{req.explorer_frames[i].translation.x, req.explorer_frames[i].translation.y, req.explorer_frames[i].translation.z};
        Eigen::Vector3d direction = explorer_loc - marker_loc;
        Eigen::Vector3d offsetPosition = marker_loc + direction.normalized() * target_distance_;

        geometry_msgs::Pose goal;
        goal.position.x = offsetPosition.x();
        goal.position.y = offsetPosition.y();
        goal.orientation = req.explorer_frames[i].rotation;

        sar_bot_->setGoal(goal);
    }

    goals_received_.store(true);

    return true;
}

