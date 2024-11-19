#include <Eigen/Dense>
#include "maximo_coding_test/sar_base.hpp"
#include "maximo_coding_test/FollowerGoals.h"

class Follower {
public:
    Follower(ros::NodeHandle& nh, double target_distance);
    void spin();

private:
    bool getGoals(maximo_coding_test::FollowerGoalsRequest& req, maximo_coding_test::FollowerGoalsResponse& resp);

    double target_distance_;
    std::atomic<bool> goals_received_;
    ros::NodeHandle nh_;
    ros::ServiceServer follower_goals_service_;
    std::unique_ptr<SARBot> sar_bot_;
};