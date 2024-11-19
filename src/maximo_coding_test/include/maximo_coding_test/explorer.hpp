#include <geometry_msgs/Twist.h>
#include "maximo_coding_test/sar_base.hpp"
#include "maximo_coding_test/FollowerGoals.h"

class Explorer {
public:
    /**
    * class for explorer SAR bot. 
    * visits locations on parameter server, finds available aruco marker, and stores in follower_goals_ struct
    * then returns home and sends locations to follower bot
    * @param nh ROS node handle
    */
    Explorer(ros::NodeHandle& nh);
    /**
    * initialize explorer bot
    * @param param_base base string for aruco marker names on param server
    * @param num_markers number of marker locations stored on param server
    */
    bool init(std::string param_base, int num_markers);
    /**
    * runner function for Follower node 
    */
    void spin();

private:
    /**
    * callback for aruco listener
    * @param fiducials aruco markers detected by aruco node
    */
    void arucoCallback(const fiducial_msgs::FiducialTransformArray& fiducials);
    /**
    * user callback for move action completion
    * commands robot to circle to aruco marker found 
    */
    void goalReachedCB();
    /**
    * helper function to listen for a transform on tf tree from "map" to frame
    * @param frame frame to listen for
    * @param transformStamped transform recovered from tf tree
    */
    bool listen(std::string frame, geometry_msgs::TransformStamped &transformStamped);
    
    maximo_coding_test::FollowerGoalsRequest follower_goals_;
    ros::NodeHandle nh_;
    ros::Publisher explorer_vel_pub_;
    ros::ServiceClient follower_client_;
    ros::Subscriber fiducial_sub_;
    std::unique_ptr<SARBot> sar_bot_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

};