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
               const ros::NodeHandle& decision_nh);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalCmd>> gimbal_pub_;
    rm_common::ShooterCommandSender* shooter_command_sender_;

    void MoveGimbalRate();
    void SetBarrelReady();

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
