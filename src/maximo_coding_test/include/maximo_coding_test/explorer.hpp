#include <geometry_msgs/Twist.h>
#include "maximo_coding_test/sar_base.hpp"
#include "maximo_coding_test/FollowerGoals.h"

class Explorer {
public:
    Explorer(ros::NodeHandle& nh);
    bool init(std::string param_base, int num_markers);
    void spin();

private:
    void arucoCallback(const fiducial_msgs::FiducialTransformArray& fiducials);
    void goalReachedCB();
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