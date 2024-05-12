//
// Created by yawara on 23-9-19.
//

#ifndef SRC_MANUAL_H
#define SRC_MANUAL_H

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "ros/ros.h"
#include <XmlRpcException.h>
#include <rm_common/decision/controller_manager.h>
#include <rm_common/decision/calibration_queue.h>
#include <rm_common//decision/command_sender.h>
#include <control_msgs/QueryCalibrationState.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/GameRobotStatus.h>

namespace behavior_tree {
class Manual : public BT::SyncActionNode{
public:
    Manual(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
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

    static BT::PortsList providedPorts() {
        BT::PortsList ports_list;
        return ports_list;
    }

    BT::NodeStatus tick() override{
        sendGimbalCmd();
        sendChassisCmd();
        sendShooterCmd();
        return BT::NodeStatus::SUCCESS;
    }

private:
    void sendGimbalCmd() {
        ros::Time time = ros::Time::now();
        gimbal_command_sender_->setMode(rm_msgs::GimbalCmd::RATE);
        gimbal_command_sender_->setRate(-dbus_.ch_l_x,-dbus_.ch_l_y);
        gimbal_command_sender_->sendCommand(time);
    }

    void sendChassisCmd() {
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

    void sendShooterCmd() {
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

    void dbusCallback(const rm_msgs::DbusData::ConstPtr& data) { dbus_ = *data; }
    void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) { game_robot_status_ = *data; }

    rm_msgs::DbusData dbus_;
    rm_msgs::GameRobotStatus game_robot_status_;

    rm_common::ShooterCommandSender* shooter_command_sender_;
    rm_common::GimbalCommandSender* gimbal_command_sender_;
    rm_common::ChassisCommandSender* chassis_command_sender_;
    rm_common::Vel2DCommandSender* vel_2d_cmd_sender_;
    ros::Subscriber dbus_sub_, game_robot_status_sub_;

    ros::Time last_time_;
    bool one_shoot_{false};
    bool continue_shoot_{false};
};
}// behavior_tree
#endif //SRC_MANUAL_H
