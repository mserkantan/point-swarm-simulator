#include "agent/Agent.h"
#include <cmath> 
#include <string>

Agent::Agent(){
    init_();
}

void Agent::init_(){
    
    if (ros::param::has("/id")) {
        ros::param::get("/id", id_);
    } else id_= 1;

    if (ros::param::has("/team_id")) {
        ros::param::get("/team_id", team_id_);
    } else team_id_= 1;
    
    double x, y, goal_x, goal_y, goal_z;

    // Setting the home positions.
    if (ros::param::has("/x")) {
        ros::param::get("/x", x);
        position_[0] = x;
    } else position_[0] = 0.0;

    if (ros::param::has("/y")) {
        ros::param::get("/y", y);
        position_[1] = y;
    } else position_[1] = 0.0;

    // Setting the initial goals.
    if (ros::param::has("/goal_x")) {
        ros::param::get("/goal_x", goal_x);
        goal_[0] = goal_x;
    } else goal_[0] = position_[0];

    if (ros::param::has("/goal_y")) {
        ros::param::get("/goal_y", goal_y);
        goal_[1] = goal_y;
    } else goal_[1] = position_[1];

    if (ros::param::has("/goal_z")) {
        ros::param::get("/goal_z", goal_z);
        goal_[2] = goal_z;
    } else goal_[2] = 10.0;
    

    // Initialize the initial velocities.
    status_ = 1;
    is_online_ = true;

    ROS_INFO("Agent %d is Online", id_);
    if (id_ == -1){
        ROS_WARN("THE AGENT HAS IMPROPER ID");
    }
    ros::NodeHandle nh;

    local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("agent" + std::to_string(id_) + "/local_pos", 1000);
    local_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("agent" + std::to_string(id_) + "/local_vel", 1000);
    status_pub_ = nh.advertise<std_msgs::Int8>("agent" + std::to_string(id_) + "/status", 1000);

    ros::Subscriber goal_sub = nh.subscribe("agent" + std::to_string(id_) + "/goal_sub", 1000, &Agent::goal_pos_callback, this);

    loop_();
}

void Agent::loop_(){
    ros::Rate loop_rate(60);

    while (ros::ok()){
        controller_();
        publish_position_();
        publish_velocity_();
        update_status_();
        PRINT_AGENT_STATE();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void Agent::controller_(){
    double prev_error[3] {0.0, 0.0, 0.0};
    double error[3] {0.0, 0.0, 0.0};
    double p[3] {0.0, 0.0, 0.0};
    double d[3] {0.0, 0.0, 0.0};

    for (int i = 0; i < 3; i++)
    {
        prev_error[i] = error[i];
        error[i] = goal_[i] - position_[i];
        
        if (std::abs(error[i]) > MAX_SPEED){
            p[i] = MAX_SPEED * (std::abs(error[i]) / (error[i]+0.00001));
        } else p[i] = error[i];

        d[i] = (error[i] - prev_error[i]) * 60;

        velocity_[i] = (0.8*p[i] + 0.01*d[i]);
    }
    
    move_();

}

void Agent::move_(){
    for (int i = 0; i < 3; i++){
        position_[i] += (velocity_[i] / LOOP_RATE);
    }
}

void Agent::update_status_(){
    double distance_to_goal[3] {0.0, 0.0, 0.0};
    bool reached = false;

    for (int i = 0; i < 3; i++){
        if ((goal_[i] - position_[i]) > 1.0) {
            reached = false;
            status_ = 0;
            break;
        }else reached = true;
        
    }

    if (status_ == 0 && reached){
        status_ = 2;
    } else if (status_ == 2 && reached){
        status_ = 1;
    }

    publish_status_();
}

void Agent::PRINT_AGENT_STATE(){
    ROS_INFO("AGENT %d Status: %d: \n Position: x=%f, y=%f, z=%f \n Velocity: x=%f, y=%f, z=%f \n Goal: x=%f, y=%f, z=%f", 
        id_, 
        status_,
        position_[0], position_[1], position_[2], 
        velocity_[0], velocity_[1], velocity_[2],
        goal_[0], goal_[1], goal_[2]);
}

void Agent:: publish_position_(){
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "agent_" + std::to_string(id_);
    pose.pose.position.x = position_[0];
    pose.pose.position.y = position_[1];
    pose.pose.position.z = position_[2];

    local_pos_pub_.publish(pose);
}

void Agent::publish_velocity_(){
    geometry_msgs::TwistStamped vel;
    vel.header.stamp = ros::Time::now();
    vel.header.frame_id = "agent_" + std::to_string(id_);
    vel.twist.linear.x = velocity_[0];
    vel.twist.linear.y = velocity_[1];
    vel.twist.linear.z = velocity_[2];

    local_vel_pub_.publish(vel);
}

void Agent::publish_status_(){
    std_msgs::Int8 status;
    status.data = status_;

    status_pub_.publish(status);
}
void Agent::goal_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    goal_[0], goal_[1], goal_[2] =  msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

}


