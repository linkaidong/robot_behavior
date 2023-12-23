//
// Created by yawara on 23-9-2.
//

#ifndef SRC_IS_ENEMY_VISIBLE_H
#define SRC_IS_ENEMY_VISIBLE_H

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "ros/ros.h"
#include <rm_msgs/TrackData.h>

namespace behavior_tree{
class IsEnemyVisible : public BT::SyncActionNode{
public:
    IsEnemyVisible(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle& root_nh,
                   const ros::NodeHandle& decision_nh);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    ros::Subscriber track_sub_;
    rm_msgs::TrackData track_buffer_;

    void detectionCallback(const rm_msgs::TrackData::ConstPtr& msg);
};
}// behavior_tree

#endif //SRC_IS_ENEMY_VISIBLE_H
