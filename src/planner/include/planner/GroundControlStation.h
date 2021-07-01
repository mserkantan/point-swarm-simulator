#pragma once

#include <ros/ros.h>
#include <vector>
#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Int8.h"

struct Obstacle { // Curve || Line
    double  x;
    double  y;
    double  z;
    double  r; // r = 0, if obstacle is a line.
};

struct Agent {
    std::string             type;
    int                     id;
    int                     team_id = 0;
    bool                    is_online = false;
    short                   status = 0;
    double                  distance_to_center;
    std::vector<double>     position {0.0, 0.0, 0.0};
    std::vector<double>     velocity {0.0, 0.0, 0.0};

    ros::Subscriber         position_subscriber;
    ros::Subscriber         velocity_subscriber;
    ros::Subscriber         status_subscriber;

    ros::Publisher          goal_publisher;
};

class GroundControl{

    public:
        GroundControl();
    
    private:

        ros::NodeHandle                         nh_;
        std::vector<Agent>                      agent_list;
        std::vector<Obstacle>                   obstacle_data;
        
        bool                                    should_exit_;
        int                                     agent_count;
        int                                     LOOP_RATE = 40;
        
        void    add_agents_();
        void    set_known_obstacles_();
        void    initialize_subscribers_();
        void    add_new_agent_(std::string agent_name);

        void    agent_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void    agent_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void    agent_status_callback(const std_msgs::Header::ConstPtr& msg);

        void    set_goal_(std::vector<double>);
        void    loop_();

};

