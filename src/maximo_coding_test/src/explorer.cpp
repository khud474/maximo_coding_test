#include "maximo_coding_test/explorer.hpp"

Explorer::Explorer(ros::NodeHandle& nh) 
    : nh_(nh),
      tf_listener_(tf_buffer_),
      explorer_vel_pub_(nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 100)),
      fiducial_sub_(nh.subscribe("/fiducial_transforms", 100, &Explorer::arucoCallback, this)),
      follower_client_(nh_.serviceClient<maximo_coding_test::FollowerGoals>("follower_goals"))
{
    geometry_msgs::Pose start_pose;
    start_pose.position.x = -4.0;
    start_pose.position.y = 2.5;
    start_pose.orientation.w = 1.0;
    sar_bot_ = std::make_unique<SARBot>(nh_, "explorer", start_pose, [this](){return this->goalReachedCB();});
}

bool Explorer::init(std::string param_base, int num_markers) {
    for (int i=num_markers; i>=1; i--) {
        std::vector<double> coords;
        nh_.getParam((param_base+std::to_string(i)), coords);
        if (coords.size() != 2) {
        ROS_ERROR("Parameter %s incorrect size.", (param_base+std::to_string(i)).c_str());
        return false;
        }
        geometry_msgs::Pose pose;
        pose.position.x = coords.at(0);
        pose.position.y = coords.at(1);
        pose.orientation.w = 1.0;
        sar_bot_->setGoal(pose);
    }

    return true;
}

void Explorer::spin() {
    ros::Rate loop_rate(10);
    
    while (ros::ok()) {
        if(!sar_bot_->isGoalRunning() && !sar_bot_->isFinished()) {
            ROS_INFO("Sending goal for explorer");
            sar_bot_->sendNextGoal();
        }

        if(sar_bot_->isFinished()) {
            std::reverse(follower_goals_.marker_frames.begin(), follower_goals_.marker_frames.end());
            std::reverse(follower_goals_.explorer_frames.begin(), follower_goals_.explorer_frames.end());
            
            ROS_INFO("Exploration complete. Sending Follower");

            maximo_coding_test::FollowerGoalsResponse res;
            if (!follower_client_.call(follower_goals_, res)) {
                ROS_ERROR("Failed to call follower service");
            }

            break;
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    return;
}

void Explorer::arucoCallback(const fiducial_msgs::FiducialTransformArray& fiducials) {
  if (!fiducials.transforms.empty()) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";

    transformStamped.transform = fiducials.transforms.front().transform;
    br.sendTransform(transformStamped);
  }
}

void Explorer::goalReachedCB() 
{
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.angular.z = 0.1;
    explorer_vel_pub_.publish(cmd_vel_msg);

    geometry_msgs::TransformStamped marker_transform;
    geometry_msgs::TransformStamped explorer_transform;
    while(!listen("marker_frame", marker_transform)) {
        ROS_INFO("Waiting for marker frame");
    }

    cmd_vel_msg.angular.z = 0;
    explorer_vel_pub_.publish(cmd_vel_msg);

    //todo: wait for robot to stop

    while(!listen("marker_frame", marker_transform)) {
        ROS_INFO("Confirming marker frame");
    }
    while(!listen("explorer_tf/base_link", explorer_transform)) {
        ROS_INFO("Waiting for explorer frame");
    }

    follower_goals_.marker_frames.push_back(marker_transform.transform);
    follower_goals_.explorer_frames.push_back(explorer_transform.transform);
}

bool Explorer::listen(std::string frame, geometry_msgs::TransformStamped &transformStamped) 
{
    try {
        transformStamped = tf_buffer_.lookupTransform("map", frame, ros::Time(0));\
        return true;
    }
    catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    return false;
}
