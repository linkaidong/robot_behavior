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
                  const ros::NodeHandle &decision_nh);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalCmd>> gimbal_pub_;
    rm_common::ShooterCommandSender* shooter_command_sender_;

    void MoveGimbalTrack();
    void ShootBarrelPUSH();

    double bullet_speed_{20.0};
};
}// behavior_tree

#endif //SRC_TRACKTOATTACK_H
