
#include <ros/ros.h>
#include "agent/Agent.h"
#include <iostream>

int main(int argc, char *argv[]) {
    std::cout << "Hello ny friend";
    ros::init(argc, argv, "Agent");
    Agent agent;
    ros::spin();

    return 0;
}

