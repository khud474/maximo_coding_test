#include "maximo_coding_test/sar_base.hpp"

class Explorer {
public:
    Explorer(ros::NodeHandle& nh);
    bool init(std::string param_base, int num_markers);
    void spin();

private:
    void arucoCallback(const fiducial_msgs::FiducialTransformArray& fiducials);
    void goalReachedCB();
    bool listen(std::string frame, geometry_msgs::TransformStamped &transformStamped);
    
    std::vector<MoveGoal> follower_goals_;
    ros::NodeHandle nh_;
    ros::Publisher explorer_vel_pub_;
    ros::Subscriber fiducial_sub_;
    SARBot sar_bot_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

};