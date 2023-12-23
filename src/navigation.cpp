//
// Created by yawara on 23-12-1.
//

#include "navigation.h"

namespace behavior_tree{
Navigation::Navigation(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
                       const ros::NodeHandle &decision_nh)
            : BT::SyncActionNode(name, config) {

    mbf_client_ =
            std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
    game_robot_status_sub_ = root_nh.subscribe<rm_msgs::GameRobotStatus>("/rm_referee/game_robot_status", 10, &Navigation::gameRobotStatusCallback, this);
    timer_ = root_nh.createWallTimer(ros::WallDuration(5.0), &Navigation::timerCallback, this, false);

    ros::NodeHandle patrol_nh(decision_nh, "patrol");
    ros::NodeHandle chassis_nh(decision_nh, "chassis");

    chassis_command_sender_ = new rm_common::ChassisCommandSender(chassis_nh);

    XmlRpc::XmlRpcValue patrol_list;
    patrol_nh.getParam("goal_list", patrol_list);
    for (int i = 0; i < patrol_list.size(); ++i) {
        ROS_ASSERT(patrol_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";

        ROS_ASSERT(patrol_list[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble and
                   patrol_list[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble and
                   patrol_list[i][2].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        pose_stamped.pose.position.x = static_cast<double>(patrol_list[i][0]);
        pose_stamped.pose.position.y = static_cast<double>(patrol_list[i][1]);
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, static_cast<double>(patrol_list[i][2]));
        pose_stamped.pose.orientation = toMsg(quat);
        patrol_list_.push_back(pose_stamped);
    }
}
void Navigation::Chassis_init() {
    ros::Time time = ros::Time::now();
    chassis_command_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    chassis_command_sender_->getMsg()->command_source_frame = "base_link";

    chassis_command_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    chassis_command_sender_->updateGameRobotStatus(game_robot_status_);
    chassis_command_sender_->sendChassisCommand(time, false);
}

void Navigation::MoveChassisRAW() {
    if(goal_reached) {
        move_base_msgs::MoveBaseGoal mbf_goal;
        mbf_goal.target_pose.header.stamp = ros::Time::now();
        mbf_goal.target_pose = patrol_list_[rand_index];
        mbf_client_->sendGoal(mbf_goal);
        goal_reached = false;
    }

    // Ensure that the execution time of the callback function is shorter than the timer cycle
    timer_.start();
    if(mbf_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        timer_.stop();
        ROS_INFO("rand_index is %d",rand_index);
        ROS_INFO("The Goal achieved success !!!");
        rand_index++;
        if(rand_index >= rand_index_size){
            rand_index = 0;
        }
        goal_reached = true;
    }
}

void Navigation::timerCallback(const ros::WallTimerEvent&) {
    rand_index++;
    if(rand_index >= rand_index_size){
        rand_index = 0;
    }
    goal_reached = true;
}

void Navigation::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) {
    game_robot_status_ = *data;
}

BT::PortsList Navigation::providedPorts() {
    BT::PortsList ports_list;
    return ports_list;
}

BT::NodeStatus Navigation::tick(){
    Chassis_init();
    MoveChassisRAW();
    return BT::NodeStatus::SUCCESS;
}

}// behavior_tree