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
           const ros::NodeHandle &decision_nh);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    rm_msgs::DbusData dbus_;
    rm_msgs::GameRobotStatus game_robot_status_;
private:
    void dbusCallback(const rm_msgs::DbusData::ConstPtr& data);
    void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data);
    void sendGimbalCmd();
    void sendChassisCmd();
    void sendShooterCmd();

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
