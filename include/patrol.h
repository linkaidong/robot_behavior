//
// Created by yawara on 23-8-29.
//

#ifndef SRC_PATROL_H
#define SRC_PATROL_H

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "ros/ros.h"

#include <rm_msgs/GimbalCmd.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rm_msgs/TrackData.h>
#include "rm_common/ori_tool.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rm_common/decision/command_sender.h>

namespace behavior_tree{
class Patrol : public BT::SyncActionNode{
public:
    Patrol(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle& root_nh,
           const ros::NodeHandle& decision_nh)
           : BT::SyncActionNode(name, config) {
        gimbal_pub_ = std::make_unique<realtime_tools::RealtimePublisher<rm_msgs::GimbalCmd>>(
                root_nh, "/controllers/gimbal_controller/command", 1);
        ros::NodeHandle shooter_nh(decision_nh, "shooter");

        tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
        shooter_command_sender_ = new rm_common::ShooterCommandSender(shooter_nh);
    }

    static BT::PortsList providedPorts() {
        BT::PortsList ports_list;
        return ports_list;
    }

    BT::NodeStatus tick() override{
        SetBarrelReady();
        MoveGimbalRate();
        return BT::NodeStatus::SUCCESS;
    }

private:
    void MoveGimbalRate() {
        try{
            base2yaw = tf_buffer_.lookupTransform("base_link","yaw",ros::Time(0));
            yaw2pitch = tf_buffer_.lookupTransform("yaw","pitch",ros::Time(0));
        }catch(tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        double yaw_delta = yawFromQuat(base2yaw.transform.rotation);
        double roll_temp, pitch_delta, yaw_temp;
        quatToRPY(yaw2pitch.transform.rotation, roll_temp, pitch_delta, yaw_temp);

        if(gimbal_pub_->trylock()) {
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

    void SetBarrelReady() {
        ros::Time now_time = ros::Time::now();
        shooter_command_sender_->setMode(rm_msgs::ShootCmd::READY);
        shooter_command_sender_->sendCommand(now_time);
    }

    std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalCmd>> gimbal_pub_;
    rm_common::ShooterCommandSender* shooter_command_sender_;
    geometry_msgs::TransformStamped base2yaw;
    geometry_msgs::TransformStamped yaw2pitch;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener* tf_listener_;

    int pitch_switch_{1}, yaw_switch_{1};
    double yaw_rate_{3.0}, pitch_rate_{0.6};
    double pitch_lower_limit_{-0.28}, pitch_upper_limit_{0.25}, yaw_upper_limit_{2.8};
};
}// behavior_tree

#endif //SRC_PATROL_H
