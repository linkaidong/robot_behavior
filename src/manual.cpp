//
// Created by yawara on 23-9-19.
//
#include "manual.h"

namespace behavior_tree {
Manual::Manual(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
               const ros::NodeHandle &decision_nh)
        : BT::SyncActionNode(name, config) {

    ros::NodeHandle vel2d_nh(decision_nh,"vel2d");
    ros::NodeHandle chassis_nh(decision_nh, "chassis");
    ros::NodeHandle gimbal_nh(decision_nh, "gimbal");
    ros::NodeHandle shooter_nh(decision_nh, "shooter");

    gimbal_command_sender_ = new rm_common::GimbalCommandSender(gimbal_nh);
    chassis_command_sender_ = new rm_common::ChassisCommandSender(chassis_nh);
    vel_2d_cmd_sender_ = new rm_common::Vel2DCommandSender(vel2d_nh);
    shooter_command_sender_ = new rm_common::ShooterCommandSender(shooter_nh);

    dbus_sub_ = root_nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &Manual::dbusCallback, this);
    game_robot_status_sub_ = root_nh.subscribe<rm_msgs::GameRobotStatus>("/rm_referee/game_robot_status", 10, &Manual::gameRobotStatusCallback, this);
}

void Manual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data){
    game_robot_status_ = *data;
}

void Manual::dbusCallback(const rm_msgs::DbusData::ConstPtr& data) {
    dbus_ = *data;
}

void Manual::sendGimbalCmd() {
    ros::Time time = ros::Time::now();
    gimbal_command_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    gimbal_command_sender_->setRate(-dbus_.ch_l_x,-dbus_.ch_l_y);
    gimbal_command_sender_->sendCommand(time);
}

void Manual::sendChassisCmd() {
    ros::Time time = ros::Time::now();

    chassis_command_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_2d_cmd_sender_->setAngularZVel(dbus_.wheel);
    // power_limit_????
    vel_2d_cmd_sender_->setLinearXVel(dbus_.ch_r_y);
    vel_2d_cmd_sender_->setLinearYVel(-dbus_.ch_r_x);

    chassis_command_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    chassis_command_sender_->updateGameRobotStatus(game_robot_status_);

    vel_2d_cmd_sender_->sendCommand(time);
    chassis_command_sender_->sendChassisCommand(time, true);
}

void Manual::sendShooterCmd() {
    ros::Time now_time = ros::Time::now();
    if (dbus_.s_l == rm_msgs::DbusData::UP) {
        if (now_time.toSec() - last_time_.toSec() > 1.0 || continue_shoot_) {
            if (one_shoot_){
                continue_shoot_ = true;
            }
            last_time_ = now_time;
            one_shoot_ = true;
            shooter_command_sender_->setMode(rm_msgs::ShootCmd::PUSH);
        } else{
            shooter_command_sender_->setMode(rm_msgs::ShootCmd::READY);
        }
    } else {
        continue_shoot_ = false;
        one_shoot_ = false;
        if (dbus_.s_l == rm_msgs::DbusData::MID){
            shooter_command_sender_->setMode(rm_msgs::ShootCmd::READY);
        } else {
            shooter_command_sender_->setMode(rm_msgs::ShootCmd::STOP);
        }
    }
    shooter_command_sender_->checkError(ros::Time::now());
    shooter_command_sender_->sendCommand(now_time);
}

BT::PortsList Manual::providedPorts(){
    BT::PortsList ports_list;
    return ports_list;
}

BT::NodeStatus Manual::tick(){
    sendGimbalCmd();
    sendChassisCmd();
    sendShooterCmd();
    return BT::NodeStatus::SUCCESS;
}

}// behavior_tree