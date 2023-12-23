//
// Created by yawara on 23-8-29.
//

#include "patrol.h"

namespace behavior_tree{
Patrol::Patrol(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
                   const ros::NodeHandle &decision_nh)
                    : BT::SyncActionNode(name, config) {

    gimbal_pub_ = std::make_unique<realtime_tools::RealtimePublisher<rm_msgs::GimbalCmd>>(
            root_nh, "/controllers/gimbal_controller/command", 1);
    ros::NodeHandle shooter_nh(decision_nh, "shooter");
    
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    shooter_command_sender_ = new rm_common::ShooterCommandSender(shooter_nh);
}

void Patrol::MoveGimbalRate() {
    try{
        base2yaw = tf_buffer_.lookupTransform("base_link","yaw",ros::Time(0));
        yaw2pitch = tf_buffer_.lookupTransform("yaw","pitch",ros::Time(0));
    }catch(tf2::TransformException& ex){
        ROS_WARN("%s", ex.what());
        return;
    }

    double yaw_delta = yawFromQuat(base2yaw.transform.rotation);
    double roll_temp, pitch_delta, yaw_temp;
    quatToRPY(yaw2pitch.transform.rotation, roll_temp, pitch_delta, yaw_temp);

    if(gimbal_pub_->trylock()){
        gimbal_pub_->msg_.mode = gimbal_pub_->msg_.RATE;
        gimbal_pub_->msg_.stamp = ros::Time::now();
        if(pitch_delta > pitch_upper_limit_){
            pitch_switch_ = -1;
        }else if(pitch_delta < pitch_lower_limit_){
            pitch_switch_ = 1;
        }

        if (yaw_delta > yaw_upper_limit_){
            yaw_switch_ = -1;
        } else if (yaw_delta < -yaw_upper_limit_){
            yaw_switch_ = 1;
        }

        gimbal_pub_->msg_.rate_pitch = pitch_switch_ * pitch_rate_;
        gimbal_pub_->msg_.rate_yaw = yaw_rate_ * yaw_switch_;
        gimbal_pub_->unlockAndPublish();
    }
}

void Patrol::SetBarrelReady() {
    ros::Time now_time = ros::Time::now();
    shooter_command_sender_->setMode(rm_msgs::ShootCmd::READY);
    shooter_command_sender_->sendCommand(now_time);
}

BT::PortsList Patrol::providedPorts(){
    BT::PortsList ports_list;
    return ports_list;
}

BT::NodeStatus Patrol::tick(){
    SetBarrelReady();
    MoveGimbalRate();
    return BT::NodeStatus::SUCCESS;
}

}// behavior_tree