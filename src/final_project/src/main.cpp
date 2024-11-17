#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


void broadcast(const fiducial_msgs::FiducialTransform& fid_trans) {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  //broadcast the new frame to /tf Topic
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
  transformStamped.child_frame_id = "marker_frame";

  // todo: better tolerance method
  transformStamped.transform = fid_trans.transform;
  transformStamped.transform.translation.x /= 2;
  transformStamped.transform.translation.y /= 2;
  transformStamped.transform.translation.z /= 2;
  // ROS_INFO("Broadcasting");
  br.sendTransform(transformStamped);
}

bool listen(
  tf2_ros::Buffer& tfBuffer, 
  std::string frame, 
  geometry_msgs::TransformStamped &transformStamped) 
{
  try {
    transformStamped = tfBuffer.lookupTransform("map", frame, ros::Time(0));\
    return true;
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  return false;
}

void arucoCallback(const fiducial_msgs::FiducialTransformArray& fiducials) {
  if (!fiducials.transforms.empty()) {
    broadcast(fiducials.transforms.front());
  }
}

void findFiducial(
  ros::Publisher& vel_pub, 
  tf2_ros::Buffer& tfBuffer, 
  std::vector<move_base_msgs::MoveBaseGoal>& follower_goals)
{
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.angular.z = 0.1;
  vel_pub.publish(cmd_vel_msg);

  geometry_msgs::TransformStamped trans;
  while(!listen(tfBuffer, "marker_frame", trans)) {
    ROS_INFO("Waiting for marker_frame");
  }

  cmd_vel_msg.angular.z = 0;
  vel_pub.publish(cmd_vel_msg);

  follower_goals.push_back(move_base_msgs::MoveBaseGoal{});
  follower_goals.back().target_pose.header.frame_id = "map";
  follower_goals.back().target_pose.header.stamp = ros::Time::now();
  follower_goals.back().target_pose.pose.position.x = trans.transform.translation.x;
  follower_goals.back().target_pose.pose.position.y = trans.transform.translation.y;
  follower_goals.back().target_pose.pose.orientation.w = trans.transform.rotation.w;

}

int main(int argc, char** argv)
{
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  std::vector<move_base_msgs::MoveBaseGoal> explorer_goals;
  std::vector<move_base_msgs::MoveBaseGoal> follower_goals;

  ros::Subscriber fiducail_sub = nh.subscribe("/fiducial_transforms", 100, arucoCallback);
  ros::Publisher explorer_vel_pub = nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 100);

  geometry_msgs::TransformStamped explorer_start_location;
  while(!listen(tfBuffer, "explorer_tf/base_link", explorer_start_location)) {
    ROS_INFO("Waiting for explorer start location");
  }

  explorer_goals.push_back(move_base_msgs::MoveBaseGoal{});
  explorer_goals.back().target_pose.header.frame_id = "map";
  explorer_goals.back().target_pose.header.stamp = ros::Time::now();
  explorer_goals.back().target_pose.pose.position.x = explorer_start_location.transform.translation.x;
  explorer_goals.back().target_pose.pose.position.y = explorer_start_location.transform.translation.y;
  explorer_goals.back().target_pose.pose.orientation.w = explorer_start_location.transform.rotation.w;

  geometry_msgs::TransformStamped follower_start_location;
  while(!listen(tfBuffer, "follower_tf/base_link", follower_start_location)) {
    ROS_INFO("Waiting for follower start location");
  }

  // retrieve and store marker locations from paramter server
  std::string explorer_location_param_base = "/aruco_lookup_locations/target_";
  for (int i=4; i>=1; i--) {
    std::vector<double> coords;
    nh.getParam((explorer_location_param_base+std::to_string(i)), coords);
    if (coords.size() != 2) {
      ROS_FATAL("Parameter %s incorrect size.", (explorer_location_param_base+std::to_string(i)).c_str());
      ros::shutdown();
      return 1;
    }
    explorer_goals.push_back(move_base_msgs::MoveBaseGoal{});
    explorer_goals.back().target_pose.header.frame_id = "map";
    explorer_goals.back().target_pose.header.stamp = ros::Time::now();
    explorer_goals.back().target_pose.pose.position.x = coords.at(0);
    explorer_goals.back().target_pose.pose.position.y = coords.at(1);
    explorer_goals.back().target_pose.pose.orientation.w = 1.0;

  }

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }

  ros::Rate loop_rate(10);

  bool reversed = false;
  bool explorer_finished = false;
  bool follower_finished = false;

  while (ros::ok()) {
    if (!explorer_goal_sent)     {
      ROS_INFO("Sending goal for explorer");
      explorer_client.sendGoal(explorer_goals.back());//this should be sent only once
      explorer_goal_sent = true;
    }

    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && !explorer_finished) {
      ROS_INFO("Hooray, explorer robot reached goal # %lu", explorer_goals.size()-1);
      explorer_goals.pop_back();
      if(explorer_goals.size()) {
        findFiducial(explorer_vel_pub, tfBuffer, follower_goals);
        explorer_goal_sent = false;
      }
      else {
        explorer_finished = true;
        ROS_INFO("Explorer robot back home. Sending Follower");
      }
    }

    if (!follower_goal_sent && explorer_finished) {
      if(!reversed) {
        follower_goals.push_back(move_base_msgs::MoveBaseGoal{});
        follower_goals.back().target_pose.header.frame_id = "map";
        follower_goals.back().target_pose.header.stamp = ros::Time::now();
        follower_goals.back().target_pose.pose.position.x = follower_start_location.transform.translation.x;
        follower_goals.back().target_pose.pose.position.y = follower_start_location.transform.translation.y;
        follower_goals.back().target_pose.pose.orientation.w = follower_start_location.transform.rotation.w;
        std::reverse(follower_goals.begin(), follower_goals.end());
        reversed = true;
      }
      ROS_INFO("Sending goal for explorer");
      follower_client.sendGoal(follower_goals.back());//this should be sent only once
      follower_goal_sent = true;
    }

    if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && !follower_finished) {
      ROS_INFO("Hooray, follower robot reached goal # %lu", follower_goals.size()-1);
      follower_goals.pop_back();
      if(follower_goals.size()) {
        follower_goal_sent = false;
      }
      else {
        ROS_INFO("Follower robot back home. Finished");
        follower_finished = true;
      }
    }

    if(follower_finished) {
      ros::shutdown();
      break;
    }

    ros::spinOnce(); //uncomment this if you have subscribers in your code


    loop_rate.sleep();
  }


}