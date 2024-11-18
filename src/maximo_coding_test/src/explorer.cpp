#include "maximo_coding_test/explorer.hpp"

Explorer::Explorer(ros::NodeHandle& nh) 
    : nh_(nh),
      tf_listener_(tf_buffer_),
      explorer_vel_pub_(nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 100)),
      fiducial_sub_(nh.subscribe("/fiducial_transforms", 100, &Explorer::arucoCallback, this)),
      sar_bot_(nh_, "explorer", MoveGoal{-4.0, 2.5, 1.0}, [this](){return this->goalReachedCB();})
{
    // sar_bot_ = std::make_unique<SARBot>(nh_, "explorer", MoveGoal{-4.0, 2.5, 1.0}, [this](){return goalReachedCB();});
}

bool Explorer::init(std::string param_base, int num_markers) {
    for (int i=num_markers; i>=1; i--) {
        std::vector<double> coords;
        nh_.getParam((param_base+std::to_string(i)), coords);
        if (coords.size() != 2) {
        ROS_ERROR("Parameter %s incorrect size.", (param_base+std::to_string(i)).c_str());
        return false;
        }
        sar_bot_.setGoal(MoveGoal{coords.at(0), coords.at(1), 1.0});
    }

    return true;
}

void Explorer::spin() {
    ros::Rate loop_rate(10);
    
    while (ros::ok()) {
        if(!sar_bot_.isGoalRunning() && !sar_bot_.isFinished()) {
            ROS_INFO("Sending goal for explorer");
            sar_bot_.sendNextGoal();
        }

        if(sar_bot_.isFinished()) {
            std::reverse(follower_goals_.begin(), follower_goals_.end());
            
            ROS_INFO("Exploration complete. Sending Follower");
            // send service call
            ROS_INFO("Follower goals size: %ld", follower_goals_.size());

            break;
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    return;
}

void Explorer::goalReachedCB() 
{
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.angular.z = 0.1;
    explorer_vel_pub_.publish(cmd_vel_msg);

    geometry_msgs::TransformStamped trans;
    while(!listen("marker_frame", trans)) {
        ROS_INFO("Waiting for marker_frame");
    }

    cmd_vel_msg.angular.z = 0;
    explorer_vel_pub_.publish(cmd_vel_msg);
    follower_goals_.push_back(
        MoveGoal{trans.transform.translation.x, trans.transform.translation.y, trans.transform.rotation.w}
    );
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

void Explorer::arucoCallback(const fiducial_msgs::FiducialTransformArray& fiducials) {
  if (!fiducials.transforms.empty()) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";

    // todo: better tolerance method
    transformStamped.transform = fiducials.transforms.front().transform;
    transformStamped.transform.translation.x /= 2;
    transformStamped.transform.translation.y /= 2;
    transformStamped.transform.translation.z /= 2;
    // ROS_INFO("Broadcasting");
    br.sendTransform(transformStamped);
  }
}