#include "maximo_coding_test/follower.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "follower_node");
    ros::NodeHandle nh;

    Follower follower(nh, 0.4);

    follower.spin();

    ros::shutdown();
    return 0;
}