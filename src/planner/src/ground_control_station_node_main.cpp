
#include <ros/ros.h>
#include "planner/GroundControlStation.h"
#include <iostream>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "GroundControlStation");
    GroundControl gcs;
    ros::spin();

    return 0;
}

