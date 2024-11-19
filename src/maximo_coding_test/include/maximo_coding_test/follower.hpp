#include <Eigen/Dense>
#include "maximo_coding_test/sar_base.hpp"
#include "maximo_coding_test/FollowerGoals.h"

class Follower {
public:
    /**
    * class for follower SAR bot. 
    * expects list of aruco locations then goes to each location in order
    * @param nh ROS node handle
    * @param target_distance acceptable approach distance to aruco target
    */
    Follower(ros::NodeHandle& nh, double target_distance);
    /**
    * runner function for Follower node 
    */
    void spin();

private:
    /**
    * callback for "follower_goals" service
    * receives aruco locations from explorer, applies approach offset, and stores goal 
    * @param req ROS service request
    * @param resp ROS service response
    */
    bool getGoals(maximo_coding_test::FollowerGoalsRequest& req, maximo_coding_test::FollowerGoalsResponse& resp);

    double target_distance_;
    std::atomic<bool> goals_received_;
    ros::NodeHandle nh_;
    ros::ServiceServer follower_goals_service_;
    std::unique_ptr<SARBot> sar_bot_;
};