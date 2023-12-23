//
// Created by yawara on 23-9-2.
//

#include "track_to_attack.h"

namespace behavior_tree{
TrackToAttack::TrackToAttack(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
              const ros::NodeHandle &decision_nh)
              : BT::SyncActionNode(name, config) {

    gimbal_pub_ = std::make_unique<realtime_tools::RealtimePublisher<rm_msgs::GimbalCmd>>(
            root_nh, "/controllers/gimbal_controller/command", 1);

    ros::NodeHandle shooter_nh(decision_nh, "shooter");

    shooter_command_sender_ = new rm_common::ShooterCommandSender(shooter_nh);
}

void TrackToAttack::MoveGimbalTrack(){
    if(gimbal_pub_->trylock()) {
        gimbal_pub_->msg_.mode = gimbal_pub_->msg_.TRACK;
        gimbal_pub_->msg_.bullet_speed = bullet_speed_;
        gimbal_pub_->unlockAndPublish();
    }
}

void TrackToAttack::ShootBarrelPUSH() {
    ros::Time now_time = ros::Time::now();
    shooter_command_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_command_sender_->checkError(ros::Time::now());
    shooter_command_sender_->sendCommand(now_time);
}

BT::PortsList TrackToAttack::providedPorts(){
    BT::PortsList ports_list;
    return ports_list;
}

BT::NodeStatus TrackToAttack::tick(){
    MoveGimbalTrack();
    ShootBarrelPUSH();
    return BT::NodeStatus::SUCCESS;
}

}// behavior_tree