#include "planner/GroundControlStation.h"

#include <fstream>
#include <regex>

GroundControl::GroundControl(){
    agent_count = 0;
    should_exit_ = false;

    add_agents_();
    set_known_obstacles_();
    loop_(); // To be replaced by local planner.
}

void GroundControl::add_agents_(){
    std::cout << "I can start reading agents txt!" << std::endl;
    std::ifstream infile("agent_names.txt");
    std::string line;
    std::vector<std::string> agent_names {"agent_0", "agent_1", "agent_2", "agent_3"};
    //while (infile >> line){
    //    std::cout << line << std::endl;
    //    agent_names.push_back(line);
    //}
    
    if (agent_names.size() - agent_count) {
        for (int i = agent_count; i < agent_names.size(); i++) {
            add_new_agent_(agent_names[i]);
        }
    }

}
void GroundControl::add_new_agent_(std::string agent_name){
    
    Agent agent;
    int index = agent_name.find("_");
    std::string str_id = agent_name.substr(index+1, agent_name.size());
    std::regex reg("^[0-9]{1,10}$");

    if (std::regex_match(str_id, reg))
        agent.id    = std::stoi(str_id);
    else{
        ROS_ERROR("An Agent has invalid id: %s", str_id.c_str());
        return;
    }

    agent.type  = agent_name.substr(0, index);
    agent.position_subscriber = nh_.subscribe("agent" + str_id + "/local_pos", 1000, &GroundControl::agent_position_callback, this); 
    agent.velocity_subscriber = nh_.subscribe("agent" + str_id + "/local_vel", 1000, &GroundControl::agent_velocity_callback, this); 
    agent.status_subscriber = nh_.subscribe("agent" + str_id + "/status", 1000, &GroundControl::agent_status_callback, this); 

    agent.goal_publisher = nh_.advertise<geometry_msgs::PoseStamped>("/goal_sub", 1000);
    agent_list.push_back(agent);

    
    agent_count += 1;

}

void GroundControl::agent_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    int id = std::stoi(msg->header.frame_id);
    if (id >= 0 && id < agent_list.size()){
        agent_list[id].position[0] = msg->pose.position.x; 
        agent_list[id].position[1] = msg->pose.position.y; 
        agent_list[id].position[2] = msg->pose.position.z; 
    }
}
void GroundControl::agent_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    int id = std::stoi(msg->header.frame_id);
    if (id >= 0 && id < agent_list.size()){
        agent_list[id].velocity[0] = msg->twist.linear.x;
        agent_list[id].velocity[1] = msg->twist.linear.y;
        agent_list[id].velocity[2] = msg->twist.linear.z;
    }
}
void GroundControl::agent_status_callback(const std_msgs::Header::ConstPtr& msg){
    int id = std::stoi(msg->frame_id);
    if (id >= 0 && id < agent_list.size()){
        agent_list[id].status = msg->seq;
    }
}

void GroundControl::set_known_obstacles_(){

}

void GroundControl::loop_(){
    ros::Rate loop_rate(40);
    std::vector<double> x_goals;
    
    for (int i=0; i<agent_list.size(); i++){
        x_goals.push_back(5+i);
    }

    while (nh_.ok() && ros::ok()){

        set_goal_(x_goals);
        ros::spinOnce();
        loop_rate.sleep();
    }

}   

void GroundControl::set_goal_(std::vector<double> goals){
    for (int i=0; i<goals.size(); i++){
        geometry_msgs::PoseStamped pos_msg;
        pos_msg.header.frame_id = agent_list[i].id;
        pos_msg.header.stamp = ros::Time::now();
        pos_msg.pose.position.x = goals[i];
        pos_msg.pose.position.y = goals[i];
        pos_msg.pose.position.z = goals[i];
        agent_list[i].goal_publisher.publish(pos_msg);
    }
}

