#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Int8.h"

#include <sstream>
#include <vector>

class Agent{
    
    public:
        Agent();

    private:
        int id_;
        int team_id_;
        bool is_online_;
        short status_; // busy || idle || reached

        ros::Publisher	local_pos_pub_;
        ros::Publisher	local_vel_pub_;
        ros::Publisher  status_pub_;

        std::vector<double> position_ {0.0, 0.0, 0.0};
        std::vector<double> velocity_ {0.0, 0.0, 0.0};
        std::vector<double> goal_ {0.0, 0.0, 0.0};
        static constexpr double MAX_SPEED = 5;
        static constexpr double MAX_ACCELERATION = 1;
        static constexpr int LOOP_RATE = 60;


        
        void init_();

        void set_goal_(std::vector<double> &goal);
        void move_();
        void update_status_();
        void controller_();
        void loop_();
        void publish_position_();
        void publish_velocity_();
        void publish_status_();
        void goal_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void PRINT_AGENT_STATE();
};