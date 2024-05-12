//
// Created by yawara on 23-9-2.
//

#ifndef SRC_TRACKTOATTACK_H
#define SRC_TRACKTOATTACK_H

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "ros/ros.h"
#include <rm_msgs/GimbalCmd.h>
#include <realtime_tools/realtime_publisher.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rm_common/decision/command_sender.h>

namespace behavior_tree{

class TrackToAttack : public BT::SyncActionNode {
public:
    TrackToAttack(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
                  const ros::NodeHandle &decision_nh)
                  : BT::SyncActionNode(name, config) {
        gimbal_pub_ = std::make_unique<realtime_tools::RealtimePublisher<rm_msgs::GimbalCmd>>(
                root_nh, "/controllers/gimbal_controller/command", 1);
        ros::NodeHandle shooter_nh(decision_nh, "shooter");
        shooter_command_sender_ = new rm_common::ShooterCommandSender(shooter_nh);
    }

    static BT::PortsList providedPorts() {
        BT::PortsList ports_list;
        return ports_list;
    }

    BT::NodeStatus tick() override{
        MoveGimbalTrack();
        ShootBarrelPUSH();
        return BT::NodeStatus::SUCCESS;
    }

private:
    void MoveGimbalTrack() {
        if(gimbal_pub_->trylock()) {
            gimbal_pub_->msg_.mode = gimbal_pub_->msg_.TRACK;
            gimbal_pub_->msg_.bullet_speed = bullet_speed_;
            gimbal_pub_->unlockAndPublish();
        }
    }

    void ShootBarrelPUSH() {
        ros::Time now_time = ros::Time::now();
        shooter_command_sender_->setMode(rm_msgs::ShootCmd::PUSH);
        shooter_command_sender_->checkError(ros::Time::now());
        shooter_command_sender_->sendCommand(now_time);
    }

    std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalCmd>> gimbal_pub_;
    rm_common::ShooterCommandSender* shooter_command_sender_;
    double bullet_speed_{20.0};
};
}// behavior_tree

#endif //SRC_TRACKTOATTACK_H