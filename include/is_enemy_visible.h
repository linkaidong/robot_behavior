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
                   const ros::NodeHandle& decision_nh)
                   : BT::SyncActionNode(name, config) {
        ros::NodeHandle is_enemy_visible_nh(decision_nh, "is_enemy_visible");
        track_sub_ = root_nh.subscribe<rm_msgs::TrackData>("/track", 1, &IsEnemyVisible::detectionCallback, this);
    }

    static BT::PortsList providedPorts() {
        BT::PortsList ports_list;
        return ports_list;
    }

    BT::NodeStatus tick() override{
        if(track_buffer_.id != 0) {
            return BT::NodeStatus::SUCCESS;
        }else {
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    void detectionCallback(const rm_msgs::TrackData::ConstPtr& msg) { track_buffer_ = *msg; }

    ros::Subscriber track_sub_;
    rm_msgs::TrackData track_buffer_;
};
}// behavior_tree

#endif //SRC_IS_ENEMY_VISIBLE_H
