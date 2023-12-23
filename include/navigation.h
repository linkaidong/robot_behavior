//
// Created by yawara on 23-12-1.
//

#ifndef SRC_NAVIGATION_H
#define SRC_NAVIGATION_H

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rm_common/decision/command_sender.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <realtime_tools/realtime_publisher.h>

namespace behavior_tree{
class Navigation : public BT::SyncActionNode {
public:
    Navigation(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
               const ros::NodeHandle &decision_nh);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::vector<geometry_msgs::PoseStamped> patrol_list_;
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> mbf_client_;

    rm_common::ChassisCommandSender* chassis_command_sender_;
    rm_msgs::GameRobotStatus game_robot_status_;
    ros::Subscriber game_robot_status_sub_;
    ros::WallTimer timer_;

    void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data);
    void timerCallback(const ros::WallTimerEvent&);
    void Chassis_init();
    void MoveChassisRAW();

    int rand_index = 0;
    int rand_index_size = 3;
    bool goal_reached{true};
};
}// behavior_tree

#endif //SRC_NAVIGATION_H
