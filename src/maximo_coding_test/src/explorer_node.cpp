#include "maximo_coding_test/explorer.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "explorer_node");
    ros::NodeHandle nh;

    Explorer explorer(nh);
    if (!explorer.init("/aruco_lookup_locations/target_", 4)) {
        ROS_FATAL("Failed to initialize exporer node");
        ros::shutdown();
        return 1;
    }

    explorer.spin();

    ros::shutdown();
    return 0;
}